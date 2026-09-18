/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
#include <uart_async_adapter.h>

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>
#include <zephyr/storage/flash_map.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "Fusion.h"

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define FUSION_THREAD_STACK_SIZE 1024
#define FUSION_ADVANCED_THREAD_STACK_SIZE 1536
#define FUSION_SAMPLE_RATE_HZ 104
#define FUSION_LOG_DIVIDER FUSION_SAMPLE_RATE_HZ

#define SENSOR_THREAD_STACK_SIZE 1536
#define SENSOR_THREAD_PRIORITY 6
#define IMU_QUEUE_DEPTH 4

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME
#define NUS_SAFE_PAYLOAD_SIZE 20

static K_SEM_DEFINE(ble_init_ok, 0, 1);
static K_SEM_DEFINE(nus_init_ok, 0, 3);
static K_MUTEX_DEFINE(nus_tx_mutex);

struct fusion_imu_sample {
	FusionVector gyroscope;
	FusionVector accelerometer;
	int64_t timestamp_us;
};

K_MSGQ_DEFINE(simple_imu_queue, sizeof(struct fusion_imu_sample),
	      IMU_QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(advanced_imu_queue, sizeof(struct fusion_imu_sample),
	      IMU_QUEUE_DEPTH, 4);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;
static struct k_work adv_work;

#if DT_HAS_CHOSEN(nordic_nus_uart)
#define NUS_UART_NODE DT_CHOSEN(nordic_nus_uart)
#elif DT_HAS_CHOSEN(zephyr_shell_uart)
#define NUS_UART_NODE DT_CHOSEN(zephyr_shell_uart)
#else
#error "No NUS UART: add chosen 'nordic,nus-uart' to devicetree (e.g. in a board overlay)."
#endif
static const struct device *uart = DEVICE_DT_GET(NUS_UART_NODE);
static struct k_work_delayable uart_work;

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

#ifdef CONFIG_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
#define async_adapter NULL
#endif

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	static size_t aborted_len;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	static bool disable_req;

	switch (evt->type) {
	case UART_TX_DONE:
		LOG_DBG("UART_TX_DONE");
		if ((evt->data.tx.len == 0) ||
		    (!evt->data.tx.buf)) {
			return;
		}

		if (aborted_buf) {
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
					   data[0]);
			aborted_buf = NULL;
			aborted_len = 0;
		} else {
			buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t,
					   data[0]);
		}

		k_free(buf);

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf) {
			return;
		}

		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
			LOG_WRN("Failed to send data over UART");
		}

		break;

	case UART_RX_RDY:
		LOG_DBG("UART_RX_RDY");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data[0]);
		buf->len += evt->data.rx.len;

		if (disable_req) {
			return;
		}

		if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
		    (evt->data.rx.buf[buf->len - 1] == '\r')) {
			disable_req = true;
			uart_rx_disable(uart);
		}

		break;

	case UART_RX_DISABLED:
		LOG_DBG("UART_RX_DISABLED");
		disable_req = false;

		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data),
			       UART_WAIT_FOR_RX);

		break;

	case UART_RX_BUF_REQUEST:
		LOG_DBG("UART_RX_BUF_REQUEST");
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
		}

		break;

	case UART_RX_BUF_RELEASED:
		LOG_DBG("UART_RX_BUF_RELEASED");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
				   data[0]);

		if (buf->len > 0) {
			k_fifo_put(&fifo_uart_rx_data, buf);
		} else {
			k_free(buf);
		}

		break;

	case UART_TX_ABORTED:
		LOG_DBG("UART_TX_ABORTED");
		if (!aborted_buf) {
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}

		aborted_len += evt->data.tx.len;
		buf = CONTAINER_OF((void *)aborted_buf, struct uart_data_t,
				   data);

		uart_tx(uart, &buf->data[aborted_len],
			buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item)
{
	struct uart_data_t *buf;

	buf = k_malloc(sizeof(*buf));
	if (buf) {
		buf->len = 0;
	} else {
		LOG_WRN("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
}

static bool uart_test_async_api(const struct device *dev)
{
	const struct uart_driver_api *api =
			(const struct uart_driver_api *)dev->api;

	return (api->callback_set != NULL);
}

static int uart_init(void)
{
	int err;
	int pos;
	struct uart_data_t *rx;
	struct uart_data_t *tx;

	if (!device_is_ready(uart)) {
		return -ENODEV;
	}

	rx = k_malloc(sizeof(*rx));
	if (rx) {
		rx->len = 0;
	} else {
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);


	if (IS_ENABLED(CONFIG_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart)) {
		/* Implement API adapter */
		uart_async_adapter_init(async_adapter, uart);
		uart = async_adapter;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		k_free(rx);
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}

	if (IS_ENABLED(CONFIG_UART_LINE_CTRL)) {
		LOG_INF("Wait for DTR");
		while (true) {
			uint32_t dtr = 0;

			uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
			if (dtr) {
				break;
			}
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
		LOG_INF("DTR set");
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
		if (err) {
			LOG_WRN("Failed to set DCD, ret code %d", err);
		}
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
		if (err) {
			LOG_WRN("Failed to set DSR, ret code %d", err);
		}
	}

	tx = k_malloc(sizeof(*tx));

	if (tx) {
		pos = snprintf(tx->data, sizeof(tx->data),
			       "Starting Nordic UART service sample\r\n");

		if ((pos < 0) || (pos >= sizeof(tx->data))) {
			k_free(rx);
			k_free(tx);
			LOG_ERR("snprintf returned %d", pos);
			return -ENOMEM;
		}

		tx->len = pos;
	} else {
		k_free(rx);
		return -ENOMEM;
	}

	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	if (err) {
		k_free(rx);
		k_free(tx);
		LOG_ERR("Cannot display welcome message (err: %d)", err);
		return err;
	}

	err = uart_rx_enable(uart, rx->data, sizeof(rx->data), UART_WAIT_FOR_RX);
	if (err) {
		LOG_ERR("Cannot enable uart reception (err: %d)", err);
		/* Free the rx buffer only because the tx buffer will be handled in the callback */
		k_free(rx);
	}

	return err;
}

static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Advertising successfully started");
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);

	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
		dk_set_led_off(CON_STATUS_LED);
	}
}

static void recycled_cb(void)
{
	LOG_INF("Connection object available from previous conn. Disconnect is complete!");
	advertising_start();
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d %s", addr, level, err,
			bt_security_err_to_str(err));
	}
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
	.recycled         = recycled_cb,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);

	if (IS_ENABLED(CONFIG_SOC_SERIES_NRF54H) || IS_ENABLED(CONFIG_SOC_SERIES_NRF54L)) {
		LOG_INF("Press Button 0 to confirm, Button 1 to reject.");
	} else {
		LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
	}
}


static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d %s", addr, reason,
		bt_security_err_to_str(reason));
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", addr);

	for (uint16_t pos = 0; pos != len;) {
		struct uart_data_t *tx = k_malloc(sizeof(*tx));

		if (!tx) {
			LOG_WRN("Not able to allocate UART send data buffer");
			return;
		}

		/* Keep the last byte of TX buffer for potential LF char. */
		size_t tx_data_size = sizeof(tx->data) - 1;

		if ((len - pos) > tx_data_size) {
			tx->len = tx_data_size;
		} else {
			tx->len = (len - pos);
		}

		memcpy(tx->data, &data[pos], tx->len);

		pos += tx->len;

		/* Append the LF character when the CR character triggered
		 * transmission from the peer.
		 */
		if ((pos == len) && (data[len - 1] == '\r')) {
			tx->data[tx->len] = '\n';
			tx->len++;
		}

		err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
		if (err) {
			k_fifo_put(&fifo_uart_tx_data, tx);
		}
	}
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void num_comp_reply(bool accept)
{
	if (accept) {
		bt_conn_auth_passkey_confirm(auth_conn);
		LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
	} else {
		bt_conn_auth_cancel(auth_conn);
		LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
	}

	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

	if (auth_conn) {
		if (buttons & KEY_PASSKEY_ACCEPT) {
			num_comp_reply(true);
		}

		if (buttons & KEY_PASSKEY_REJECT) {
			num_comp_reply(false);
		}
	}
}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

static void configure_gpio(void)
{
	int err;

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	err = dk_buttons_init(button_changed);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}
}

static int spi_flash_read_write_demo(void)
{
	static const uint8_t write_data[] = "nRF54L15 SPI flash demo";
	uint8_t read_data[sizeof(write_data)];
	const struct flash_area *area;
	int err;

	err = flash_area_open(PARTITION_ID(spi_flash_test_partition),
			      &area);
	if (err != 0) {
		LOG_ERR("SPI flash test partition open failed: %d", err);
		return err;
	}

	LOG_INF("SPI flash demo: device=%s offset=0x%lx size=%lu",
		area->fa_dev->name, (unsigned long)area->fa_off,
		(unsigned long)area->fa_size);

	err = flash_area_erase(area, 0, area->fa_size);
	if (err != 0) {
		LOG_ERR("SPI flash erase failed: %d", err);
		goto close;
	}

	err = flash_area_write(area, 0, write_data, sizeof(write_data));
	if (err != 0) {
		LOG_ERR("SPI flash write failed: %d", err);
		goto close;
	}

	memset(read_data, 0, sizeof(read_data));
	err = flash_area_read(area, 0, read_data, sizeof(read_data));
	if (err != 0) {
		LOG_ERR("SPI flash read failed: %d", err);
		goto close;
	}

	if (memcmp(write_data, read_data, sizeof(write_data)) != 0) {
		LOG_ERR("SPI flash readback verification failed");
		err = -EIO;
		goto close;
	}

	LOG_INF("SPI flash read/write verified: %s", read_data);

close:
	flash_area_close(area);
	return err;
}

static int nus_send_data(const uint8_t *data, size_t length)
{
	size_t offset = 0;
	int err = 0;

	k_mutex_lock(&nus_tx_mutex, K_FOREVER);

	while (offset < length) {
		const uint16_t chunk_length =
			MIN(length - offset, NUS_SAFE_PAYLOAD_SIZE);

		err = bt_nus_send(NULL, &data[offset], chunk_length);
		if (err) {
			break;
		}
		offset += chunk_length;
	}

	k_mutex_unlock(&nus_tx_mutex);
	return err;
}

static void imu_queue_put_latest(struct k_msgq *queue,
				 const struct fusion_imu_sample *sample)
{
	struct fusion_imu_sample discarded;

	if (k_msgq_put(queue, sample, K_NO_WAIT) == 0) {
		return;
	}

	(void)k_msgq_get(queue, &discarded, K_NO_WAIT);
	(void)k_msgq_put(queue, sample, K_NO_WAIT);
}

static void sensor_acquisition_thread(void)
{
	const struct device *const sensor =
		DEVICE_DT_GET(DT_NODELABEL(lsm6dso));
	struct sensor_value gyroscope[3];
	struct sensor_value accelerometer[3];
	struct fusion_imu_sample sample;
	int err;

	k_sem_take(&nus_init_ok, K_FOREVER);

	while (!device_is_ready(sensor)) {
		static const uint8_t error_message[] = "LSM6DSO:NOT_READY\n";

		LOG_ERR("LSM6DSO Zephyr device is not ready");
		(void)nus_send_data(error_message, sizeof(error_message) - 1);
		k_sleep(K_SECONDS(2));
	}

	LOG_INF("LSM6DSO Zephyr driver ready at 104 Hz");

	for (;;) {
		err = sensor_sample_fetch(sensor);
		if (err == 0) {
			err = sensor_channel_get(sensor, SENSOR_CHAN_GYRO_XYZ,
						 gyroscope);
		}
		if (err == 0) {
			err = sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_XYZ,
						 accelerometer);
		}
		if (err != 0) {
			LOG_ERR("LSM6DSO Sensor API read failed: %d", err);
			k_sleep(K_MSEC(100));
			continue;
		}

		sample.gyroscope.axis.x = FusionRadiansToDegrees(
			(float)sensor_value_to_double(&gyroscope[0]));
		sample.gyroscope.axis.y = FusionRadiansToDegrees(
			(float)sensor_value_to_double(&gyroscope[1]));
		sample.gyroscope.axis.z = FusionRadiansToDegrees(
			(float)sensor_value_to_double(&gyroscope[2]));

		sample.accelerometer.axis.x =
			(float)sensor_ms2_to_ug(&accelerometer[0]) / 1000000.0f;
		sample.accelerometer.axis.y =
			(float)sensor_ms2_to_ug(&accelerometer[1]) / 1000000.0f;
		sample.accelerometer.axis.z =
			(float)sensor_ms2_to_ug(&accelerometer[2]) / 1000000.0f;
		sample.timestamp_us = k_uptime_get() * 1000;

		imu_queue_put_latest(&simple_imu_queue, &sample);
		imu_queue_put_latest(&advanced_imu_queue, &sample);

		k_sleep(K_USEC(1000000 / FUSION_SAMPLE_RATE_HZ));
	}
}

K_THREAD_DEFINE(sensor_acquisition_thread_id, SENSOR_THREAD_STACK_SIZE,
		sensor_acquisition_thread, NULL, NULL, NULL,
		SENSOR_THREAD_PRIORITY, 0, 0);

static void fusion_thread(void)
{
	FusionAhrs ahrs;
	FusionAhrsSettings settings = fusionAhrsDefaultSettings;
	struct fusion_imu_sample imu_sample;
	uint32_t sample_count = 0;

	k_sem_take(&nus_init_ok, K_FOREVER);

	FusionAhrsInitialise(&ahrs);
	settings.sampleRate = (float)FUSION_SAMPLE_RATE_HZ;
	FusionAhrsSetSettings(&ahrs, &settings);

	LOG_INF("Fusion 1.3.3 started at %d Hz", FUSION_SAMPLE_RATE_HZ);

	for (;;) {
		k_msgq_get(&simple_imu_queue, &imu_sample, K_FOREVER);

		FusionAhrsUpdateNoMagnetometer(&ahrs, imu_sample.gyroscope,
					       imu_sample.accelerometer);

		if (++sample_count == FUSION_LOG_DIVIDER) {
			const FusionEuler euler =
				FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
			char nus_data[128];
			const int length = snprintf(
				nus_data, sizeof(nus_data),
				"S:%.1f,%.1f,%.1f\n",
				(double)euler.angle.roll,
				(double)euler.angle.pitch,
				(double)euler.angle.yaw);

			if ((length > 0) && (length < sizeof(nus_data))) {
				(void)nus_send_data((const uint8_t *)nus_data,
						    length);
			}
			sample_count = 0;
		}
	}
}

K_THREAD_DEFINE(fusion_thread_id, FUSION_THREAD_STACK_SIZE, fusion_thread,
		NULL, NULL, NULL, PRIORITY, 0, 0);

static void fusion_advanced_thread(void)
{
	const FusionMatrix gyroscope_misalignment = FUSION_MATRIX_IDENTITY;
	const FusionVector gyroscope_sensitivity = FUSION_VECTOR_ONES;
	const FusionVector gyroscope_offset = FUSION_VECTOR_ZERO;
	const FusionMatrix accelerometer_misalignment = FUSION_MATRIX_IDENTITY;
	const FusionVector accelerometer_sensitivity = FUSION_VECTOR_ONES;
	const FusionVector accelerometer_offset = FUSION_VECTOR_ZERO;
	FusionAhrs ahrs;
	FusionBias bias;
	FusionBiasSettings bias_settings = fusionBiasDefaultSettings;
	struct fusion_imu_sample imu_sample;
	int64_t previous_timestamp_us = 0;
	uint32_t sample_count = 0;

	const FusionAhrsSettings settings = {
		.sampleRate = (float)FUSION_SAMPLE_RATE_HZ,
		.convention = FusionConventionNwu,
		.gain = 0.5f,
		.gyroscopeRange = 250.0f,
		.accelerationRejection = 10.0f,
		.magneticRejection = 10.0f,
		.rejectionTimeout = 5.0f,
	};

	k_sem_take(&nus_init_ok, K_FOREVER);

	FusionAhrsInitialise(&ahrs);
	FusionAhrsSetSettings(&ahrs, &settings);
	FusionBiasInitialise(&bias);
	bias_settings.sampleRate = (float)FUSION_SAMPLE_RATE_HZ;
	FusionBiasSetSettings(&bias, &bias_settings);

	LOG_INF("Fusion 1.3.3 advanced started at %d Hz",
		FUSION_SAMPLE_RATE_HZ);

	for (;;) {
		k_msgq_get(&advanced_imu_queue, &imu_sample, K_FOREVER);

		FusionVector gyroscope = imu_sample.gyroscope;
		FusionVector accelerometer = imu_sample.accelerometer;

		gyroscope = FusionModelInertial(
			gyroscope, gyroscope_misalignment,
			gyroscope_sensitivity, gyroscope_offset);
		accelerometer = FusionModelInertial(
			accelerometer, accelerometer_misalignment,
			accelerometer_sensitivity, accelerometer_offset);
		gyroscope = FusionBiasUpdate(&bias, gyroscope);

		if (previous_timestamp_us != 0) {
			FusionAhrsSetSamplePeriod(
				&ahrs,
				(float)(imu_sample.timestamp_us -
					previous_timestamp_us) /
					1000000.0f);
		}
		previous_timestamp_us = imu_sample.timestamp_us;
		FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer);

		sample_count++;

		if (sample_count == (FUSION_LOG_DIVIDER / 4)) {
			const FusionEuler euler =
				FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
			char euler_data[64];
			const int euler_length = snprintf(
				euler_data, sizeof(euler_data),
				"AE:%.1f,%.1f,%.1f\n",
				(double)euler.angle.roll,
				(double)euler.angle.pitch,
				(double)euler.angle.yaw);

			if ((euler_length > 0) &&
			    (euler_length < sizeof(euler_data))) {
				(void)nus_send_data((const uint8_t *)euler_data,
						    euler_length);
			}
		} else if (sample_count == ((FUSION_LOG_DIVIDER * 3) / 4)) {
			const FusionVector earth =
				FusionAhrsGetEarthAcceleration(&ahrs);
			char earth_data[64];
			const int earth_length = snprintf(
				earth_data, sizeof(earth_data),
				"AX:%.1f,%.1f,%.1f\n",
				(double)earth.axis.x,
				(double)earth.axis.y,
				(double)earth.axis.z);

			if ((earth_length > 0) &&
			    (earth_length < sizeof(earth_data))) {
				(void)nus_send_data((const uint8_t *)earth_data,
						    earth_length);
			}
		}

		if (sample_count == FUSION_LOG_DIVIDER) {
			sample_count = 0;
		}
	}
}

K_THREAD_DEFINE(fusion_advanced_thread_id,
		FUSION_ADVANCED_THREAD_STACK_SIZE, fusion_advanced_thread,
		NULL, NULL, NULL, PRIORITY, 0, 0);

int main(void)
{
	int blink_status = 0;
	int err = 0;

	configure_gpio();

	err = spi_flash_read_write_demo();
	if (err != 0) {
		LOG_ERR("SPI flash demo failed: %d", err);
	}

	err = uart_init();
	if (err) {
		error();
	}

	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization callbacks. (err: %d)", err);
			return 0;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization info callbacks. (err: %d)", err);
			return 0;
		}
	}

	err = bt_enable(NULL);
	if (err) {
		error();
	}

	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return 0;
	}
	k_sem_give(&nus_init_ok);
	k_sem_give(&nus_init_ok);
	k_sem_give(&nus_init_ok);

	k_work_init(&adv_work, adv_work_handler);
	advertising_start();

	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}

void ble_write_thread(void)
{
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);
	struct uart_data_t nus_data = {
		.len = 0,
	};

	for (;;) {
		/* Wait indefinitely for data to be sent over bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
						     K_FOREVER);

		int plen = MIN(sizeof(nus_data.data) - nus_data.len, buf->len);
		int loc = 0;

		while (plen > 0) {
			memcpy(&nus_data.data[nus_data.len], &buf->data[loc], plen);
			nus_data.len += plen;
			loc += plen;

			if (nus_data.len >= sizeof(nus_data.data) ||
			   (nus_data.data[nus_data.len - 1] == '\n') ||
			   (nus_data.data[nus_data.len - 1] == '\r')) {
				if (nus_send_data(nus_data.data, nus_data.len)) {
					LOG_WRN("Failed to send data over BLE connection");
				}
				nus_data.len = 0;
			}

			plen = MIN(sizeof(nus_data.data), buf->len - loc);
		}

		k_free(buf);
	}
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);
