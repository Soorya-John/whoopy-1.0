# Tiny Indoor Brushed Quadcopter

An indoor-only brushed quad prototype using an **ESP32-C3 flight controller** and a separate **ESP32-S3 USB-host transmitter**. The original workbook, `tiny_indoor_brushed_drone_BOM_C3_5V_BEC.xlsx`, is preserved. See [BOM findings and assumptions](docs/architecture.md) before wiring: the workbook's design notes contain one stale RP2040 reference, while the actual component rows identify the C3 as the drone MCU.

## Repository layout

- `firmware/drone_esp32c3/` — C3 flight controller, radio receiver, MPU6050, motor PWM, Wi-Fi AP and config web API.
- `firmware/transmitter_esp32s3/` — S3 USB HID host and nRF24 transmitter.
- `shared/protocol/` — versioned, explicitly serialized 32-byte radio packets.
- `docs/` — architecture, wiring, protocol, web config, build, testing, safety, roadmap.
- `tests/` — host-side protocol and control logic tests.

## Architecture and safety

The S3 reads HID reports from a wired RadioMaster Boxer and sends normalized commands over nRF24. The C3 validates packets, reads the MPU6050 over I²C, runs a 250 Hz rate controller, and drives four external MOSFET gates. Startup is disarmed; radio loss, IMU failure, and critical timing faults command zero motor output. The drone's Wi-Fi AP is for local configuration and telemetry only; it is independent of the nRF24 flight link and cannot arm or command motors.

The GPIO map and Boxer HID mapping are **provisional/configurable** because exact dev-board models and the Boxer USB report layout are not in the BOM. Motor order, spin direction, sensor orientation, MOSFET wiring and supply capability must be checked on the actual build. This is not flight-proven firmware.

## Toolchain

Both projects target ESP-IDF **v5.2.x**: one common release line supports ESP32-C3 and ESP32-S3, the GPTimer/LEDC/I²C/SPI/NVS/Wi-Fi/HTTP server APIs used here, and the ESP USB host stack. The transmitter pins the maintained Espressif `usb_host_hid` component in its component manifest. Install ESP-IDF 5.2 and its targets per Espressif's official setup guide, then run commands from the relevant project directory.

## Build and flash

Drone (replace the port):

```powershell
cd firmware/drone_esp32c3
idf.py set-target esp32c3
idf.py build
idf.py -p COM5 flash monitor
```

Transmitter:

```powershell
cd firmware/transmitter_esp32s3
idf.py set-target esp32s3
idf.py build
idf.py -p COM6 flash monitor
```

The supplied projects use ESP-IDF's component manager for the S3 HID host dependency. Confirm dependency download and compile against the installed 5.2 patch release; this environment does not have ESP-IDF installed, so firmware builds could not be run here. See [build and flash](docs/build_and_flash.md).

## Drone hotspot and tuning

With default build settings, the C3 starts AP `Whoopy-Setup` with password `whoopy-setup-2026` and address `http://192.168.4.1`. Change the build-time defaults in `firmware/drone_esp32c3/main/Kconfig.projbuild`; never use this known example password as a security boundary in a public environment. The AP does not provide internet access. Open the dashboard, edit supported settings, choose **Apply temporarily**, then **Save to flash** to persist; PID changes become active at runtime. Configuration mutations require disarmed state. Calibration also requires disarmed/stationary preconditions. No web control can arm the craft.

## Tests

Run host tests with `python -m unittest discover -s tests -v`. Test results and hardware checklist are in [testing](docs/testing.md). Physical hardware has not been tested.

## License

No project license has been selected. All rights remain with their respective owners until the repository owner chooses a license.
