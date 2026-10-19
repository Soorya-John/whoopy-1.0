# Build, configure, flash

Use Espressif ESP-IDF 5.2.x and its supported Python/CMake/Ninja toolchain. Both targets share the same IDF release/API baseline. Install the `esp32c3` and `esp32s3` targets. The S3 project manifest resolves Espressif's `usb_host_hid` component through the IDF component manager.

```powershell
# C3 drone
cd firmware/drone_esp32c3
idf.py set-target esp32c3
idf.py menuconfig       # AP SSID/password and board pins
idf.py build
idf.py -p COM5 flash monitor

# S3 transmitter, separate terminal
cd firmware/transmitter_esp32s3
idf.py set-target esp32s3
idf.py menuconfig       # HID mapping and radio pins
idf.py build
idf.py -p COM6 flash monitor
```

Use the serial port actually assigned to each board. For the C3, remove propellers for all bench setup. For the Boxer, verify its USB mode is joystick/HID and connect through a data-capable cable. Inspect S3 startup diagnostics for report descriptor, usage values and axis ranges before configuring mapping; the default mapping is illustrative and must be confirmed.

The S3 uses Espressif's maintained `usb_host_hid` class driver (component version 1.0.0; ESP32-S3 supported). HID descriptor length and device VID/PID are logged; descriptor bytes are available at debug level. Axis bit offsets, field width, report-ID byte, raw min/max, inversion, deadzone, arm button bit, and arm polarity are build-configurable under `Whoopy transmitter settings`. Defaults are provisional and are not asserted to match a Boxer.

To map the Boxer, set its USB mode to joystick, enable debug logging, and connect it to the S3 host port. Inspect the report descriptor, then move one control at a time while watching the 1 Hz mapped-value log. Match each changing field/bit offset to a stick or switch; set its logical minimum/maximum, width and inversion; confirm neutral and endpoints. Confirm the arm bit changes only on the intended switch. The transmitter requires the arm switch to be observed released after every boot/reconnect before a fresh press can send ARM. If the input uses report IDs, leave `CONFIG_WHOOPY_HID_REPORT_ID_PRESENT` enabled; otherwise disable it. Keep propellers removed throughout mapping.

ESP-IDF is not installed in the authoring environment used to create this repository, so no firmware compile/flash result is claimed. Run both `idf.py build` commands on an installed toolchain and resolve any dependency/API differences for the selected 5.2 patch release before energizing motors.
