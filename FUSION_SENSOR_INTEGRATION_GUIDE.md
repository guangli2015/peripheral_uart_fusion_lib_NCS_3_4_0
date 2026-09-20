# Fusion Integration Guide

## 1. Purpose

This document describes the differences between this application and the
original Nordic `peripheral_uart` sample and explains the Fusion 1.3.3
integration.

The target used for the current build is:

```text
nrf54l15dk/nrf54l15/cpuapp
```

## 2. Features Added to the Original Sample

The original Nordic sample provides a bidirectional bridge between a physical
UART and the Nordic UART Service (NUS) over Bluetooth Low Energy. That
functionality is still present.

This project adds the following features:

1. The x-io Technologies Fusion 1.3.3 C library is compiled into the Zephyr
   application.
2. The Zephyr LSM6DSO driver reads synchronized gyroscope and accelerometer
   samples over SPI21. The sensor ODR is 104 Hz and the application polls it
   at 100 Hz.
3. The LSM303AGR accelerometer is connected to the same SPI21 bus with a
   separate P1.07 chip-select. It operates at 100 Hz, +/-2 g, in high-resolution
   mode.
4. A Simple Fusion thread runs a gyroscope-and-accelerometer AHRS algorithm at
   100 Hz.
5. An Advanced Fusion thread runs gyroscope bias correction, sensor
   calibration, a gyroscope-and-accelerometer AHRS algorithm, and earth-frame
   acceleration calculation at 100 Hz.
6. Fusion results are sent to the phone through the existing NUS TX
   characteristic.
7. LSM303AGR acceleration is printed through RTT and is not sent through NUS.
8. The two sensor readiness and acquisition paths are independent. Failure of
   one sensor does not prevent the other sensor from operating.
9. Simple and Advanced messages are staggered to reduce competition for BLE
   transmit buffers.
10. Long NUS messages can be divided into payloads of no more than 20 bytes.
11. Floating-point text formatting is enabled for one-decimal-place output.

The original UART-to-NUS and NUS-to-UART paths remain available.

## 3. Current Sensor Data

The application reads both sensors through the Zephyr Sensor API. A dedicated
acquisition thread reads the LSM6DSO at an application rate of 100 Hz, converts
gyroscope values from radians per second to degrees per second, and converts
acceleration values from metres per second squared to g. The same timestamped
LSM6DSO sample is copied to separate queues for the Simple and Advanced Fusion
threads.

The LSM303AGR is read independently at 100 Hz. Its acceleration is currently
used for hardware verification only and is not supplied to either Fusion
algorithm. Once per second, its values are printed to RTT in millig:

```text
LSM303AGR accel [mg]: x=... y=... z=...
```

The LSM6DSO has no magnetometer. Both algorithms therefore use
`FusionAhrsUpdateNoMagnetometer()`, and yaw may drift over time.

## 4. NUS Output

The phone must subscribe to notifications on the NUS TX characteristic.

The current compact messages are:

```text
S:roll,pitch,yaw
AE:roll,pitch,yaw
AX:x,y,z
```

Example:

```text
S:0.0,0.0,0.0
AE:0.0,0.0,0.0
AX:0.0,0.0,0.0
```

`S` is produced by the Simple algorithm. `AE` contains the Advanced Euler
angles. `AX` contains the Advanced earth-frame acceleration. All values retain
one decimal place.

Each message type is generated once per second. Their transmission times are
staggered so that they are not submitted to the BLE stack simultaneously.

BLE link-layer packets are retransmitted when a radio packet is not
acknowledged. This does not provide end-to-end application delivery
guarantees. The application must check and retry failed `bt_nus_send()` calls,
and use sequence numbers plus phone acknowledgements if strict lossless
delivery is required.

## 5. Important Files and Code

### `CMakeLists.txt`

This file adds the Fusion source files to the Zephyr `app` target and adds the
Fusion include directory:

```cmake
target_sources(app PRIVATE
  src/main.c
  src/Fusion_1.3.3/Fusion/FusionAhrs.c
  src/Fusion_1.3.3/Fusion/FusionBias.c
  src/Fusion_1.3.3/Fusion/FusionCompass.c
  src/Fusion_1.3.3/Fusion/FusionConvention.c
  src/Fusion_1.3.3/Fusion/FusionRemap.c
)
```

### `prj.conf`

The original Bluetooth, NUS, UART, settings, and logging configuration remains
enabled. The project also enables the Zephyr sensor subsystem, LSM6DSO driver,
SPI, and floating-point formatting:

```text
CONFIG_SPI=y
CONFIG_SENSOR=y
CONFIG_LSM6DSO=y
CONFIG_LIS2DH=y
CONFIG_LIS2DH_TRIGGER_NONE=y
CONFIG_LIS2DH_ACCEL_RANGE_2G=y
CONFIG_LIS2DH_OPER_MODE_HIGH_RES=y
CONFIG_LIS2DH_ODR_5=y
CONFIG_LIS2DH_BLOCK_DATA_UPDATE=y
CONFIG_CBPRINTF_FP_SUPPORT=y
```

`CONFIG_BT_BUF_ACL_TX_COUNT=7` increases the number of host-side BLE transmit
buffers. It reduces `-ENOMEM` failures during bursts but is not a delivery
guarantee. It can be removed to restore the default value if RAM usage is more
important than burst tolerance.

### `src/main.c`

The main additions are identified by these symbols:

- `nus_send_data()` serializes NUS submissions and divides data into safe
  20-byte payloads.
- `sensor_acquisition_thread()` independently checks and reads the LSM6DSO and
  LSM303AGR. Only valid LSM6DSO samples are distributed through
  `simple_imu_queue` and `advanced_imu_queue`.
- `fusion_thread()` implements the Simple example.
- `fusion_advanced_thread()` implements the Advanced example.
- `nus_init_ok` prevents the acquisition and Fusion threads from running before
  NUS is ready.
- `FUSION_SAMPLE_RATE_HZ` defines the current 100 Hz acquisition and algorithm
  rate.

### `boards/nrf54l15dk_nrf54l15_cpuapp.overlay`

The overlay connects both devices to SPI21 at 1 MHz. They share P1.11 SCK,
P1.13 MOSI, and P1.14 MISO, but use independent chip-select signals:

```text
LSM6DSO       reg = 0   CS = P1.08
LSM303AGR     reg = 1   CS = P1.07
```

The LSM6DSO accelerometer is configured for +/-2 g at 104 Hz and its gyroscope
for +/-250 degrees per second at 104 Hz. LSM303AGR settings are selected
through the LIS2DH Kconfig options shown above.

In an SPI child node, `reg` selects an entry from the controller's `cs-gpios`
array; it is not a sensor register address.

The original `ble_write_thread()` still forwards physical UART input to NUS.
It now uses `nus_send_data()` so its transmissions are serialized with Fusion
output.

### `src/Fusion_1.3.3/`

This directory contains the unmodified third-party Fusion 1.3.3 library,
examples, and documentation. Application integration is implemented in
`src/main.c`; the upstream example `main.c` files are not built as separate
applications.

## 6. Sensor Readiness Behavior

At startup, `sensor_acquisition_thread()` checks both devices independently
with `device_is_ready()`:

- If neither device is ready, both errors are printed to RTT every two seconds.
- If at least one device is ready, acquisition starts for that device.
- An unavailable LSM6DSO leaves the two Fusion threads blocked safely on their
  message queues, while an available LSM303AGR continues to print RTT samples.
- An unavailable LSM303AGR does not stop LSM6DSO acquisition or Fusion.

Zephyr initializes these sensor devices during boot. Connecting a sensor after
its initialization has failed normally does not make `device_is_ready()` become
true. Correct the wiring or power and reset the board. The expected
`WHO_AM_I` values are `0x6C` for LSM6DSO and `0x33` for the LSM303AGR
accelerometer.

## 7. Build and Flash

Build:

```powershell
west build --build-dir build . --pristine `
  --board nrf54l15dk/nrf54l15/cpuapp
```

Flash:

```powershell
west flash -d build --dev-id <debug-probe-serial-number>
```

Perform a pristine build after changing Kconfig or board configuration.
