# WHOOPY local flight console

The ESP32-C3 starts a WPA2 access point named `whoopy` at `192.168.4.1`. Connect to it and open `http://192.168.4.1/`. The page and all assets are embedded in the firmware; it has no internet, CDN, or external font dependency. AP credentials are build-time Kconfig settings (`CONFIG_WHOOPY_AP_SSID`, `CONFIG_WHOOPY_AP_PASSWORD`). The example password is published in this repository and is for bench setup only; set a private password before routine use.

The dark, responsive console uses compact ground-station tabs and a persistent status strip. It has Overview, PID Tuning, Flight Configuration, Sensors & Calibration, Motors, Radio Link, and System & Diagnostics sections. Overview pairs the live MPU6050 artificial horizon with a lightweight quad-frame projection driven by measured roll and pitch; yaw is explicitly unavailable. It polls local JSON once per second. Browser-to-drone HTTP reachability and the nRF24 transmitter link have separate indicators. The browser event list records only state transitions observed during the current session; the firmware does not keep a historical event log.

## API

- `GET /api/status` reports the flight state, IMU and radio health, calibration state, attitude, accelerometer and gyro samples, normalized transmitter inputs, motor PWM commands, packet age and counters, sequence gaps, loop timing, active config version/source, save result, uptime, free heap, reset reason, firmware version, and the compiled radio channel/rate/address. It explicitly reports that battery sensing is unavailable.
- `GET /api/config` returns the active config. `GET /api/config/defaults` returns firmware factory defaults, and `GET /api/schema` returns the validation ranges used by the editor.
- `POST /api/config` validates a bounded JSON body against the same firmware config validator and applies it to RAM. Unsupported keys, malformed values, out-of-range values, and changes outside DISARMED state are rejected.
- `POST /api/save` writes the current validated configuration to NVS. Saving is explicit; a successful response and status field confirm the result.
- `POST /api/defaults` loads factory configuration into RAM without saving it. Save separately to persist it.
- `POST /api/calibrate` queues gyro calibration. The flight state must be DISARMED, and the calibration routine checks that the aircraft is stationary. Progress is reported as states, not a percentage.

PID defaults and bounds come from firmware (`config_defaults` and `whoopy_config_valid`). The editor keeps changes local until Apply. Apply changes controller RAM; Save commits to NVS. Discard removes browser-only edits. Restore defaults requires confirmation and changes flight configuration only. The firmware applies a short critical-section snapshot when configuration is read or replaced so the control task cannot observe a partially written config.

## Telemetry limits

Roll and pitch come from the MPU6050 complementary filter; yaw is not estimated. Motor values are PWM commands, not RPM measurements. The radio driver counts decoded packets, rejected packets, and forward sequence gaps; it has no nRF24 RSSI measurement and no transmitter-side health API. Wi-Fi connectivity is not used as a radio metric. Battery voltage is unavailable because this board has no ADC divider. Reset reason is shown as the ESP-IDF numeric code. There is no firmware event-history API, motor-test mode, firmware update endpoint, or battery monitor, so the interface does not imply those features exist.

The web server has no arm, disarm, or direct motor-output endpoint. Configuration changes, save, restore, and calibration are disallowed unless the controller reports DISARMED. Radio failsafe and the control loop continue independently of HTTP requests.
