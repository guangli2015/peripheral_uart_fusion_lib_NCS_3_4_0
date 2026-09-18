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
   samples over SPI21 at 104 Hz.
3. A Simple Fusion thread runs a gyroscope-and-accelerometer AHRS algorithm at
   104 Hz.
4. An Advanced Fusion thread runs gyroscope bias correction, sensor
   calibration, a gyroscope-and-accelerometer AHRS algorithm, and earth-frame
   acceleration calculation at 104 Hz.
5. Fusion results are sent to the phone through the existing NUS TX
   characteristic.
6. Simple and Advanced messages are staggered to reduce competition for BLE
   transmit buffers.
7. Long NUS messages can be divided into payloads of no more than 20 bytes.
8. Floating-point text formatting is enabled for one-decimal-place output.

The original UART-to-NUS and NUS-to-UART paths remain available.

## 3. Current Sensor Data

The application reads real LSM6DSO hardware data through the Zephyr Sensor
API. A dedicated acquisition thread converts gyroscope values from radians per
second to degrees per second and acceleration values from metres per second
squared to g. The same timestamped sample is copied to separate queues for the
Simple and Advanced Fusion threads.

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
- `sensor_acquisition_thread()` reads the LSM6DSO and distributes timestamped
  samples through `simple_imu_queue` and `advanced_imu_queue`.
- `fusion_thread()` implements the Simple example.
- `fusion_advanced_thread()` implements the Advanced example.
- `nus_init_ok` prevents the acquisition and Fusion threads from running before
  NUS is ready.
- `FUSION_SAMPLE_RATE_HZ` defines the current 104 Hz algorithm rate.

### `boards/nrf54l15dk_nrf54l15_cpuapp.overlay`

The overlay connects the Zephyr LSM6DSO device to SPI21 at 1 MHz. It configures
the accelerometer for +/-2 g at 104 Hz and the gyroscope for +/-250 degrees per
second at 104 Hz.

The original `ble_write_thread()` still forwards physical UART input to NUS.
It now uses `nus_send_data()` so its transmissions are serialized with Fusion
output.

### `src/Fusion_1.3.3/`

This directory contains the unmodified third-party Fusion 1.3.3 library,
examples, and documentation. Application integration is implemented in
`src/main.c`; the upstream example `main.c` files are not built as separate
applications.

## 6. Build and Flash

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
