# Testing and validation

## Host tests

With Python 3.10+, from repository root run `python -m unittest discover -s tests -v`. These cover protocol binary encoding/CRC/ranges/freshness and hardware-independent controller/mixer/config bounds/failsafe transitions. They do not substitute for firmware compilation or hardware validation. NVS recovery and HTTP/cJSON request handling require ESP-IDF runtime and are not exercised by host tests.

## Bench checklist

1. Remove propellers. Check pin map against each exact dev-board schematic and continuity/power polarity.
2. Connect Boxer by USB; verify HID device/report descriptor diagnostics, axes, switches, neutral centers and configured channel mapping; unplug/replug and verify arm remains off.
3. Verify each radio independently at low power/short range; confirm address/channel/rate, ACK status and packet age.
4. Verify MPU6050 ID/address, orientation, stationary gyro calibration and stable readings. Move each axis by hand and check sign.
5. Connect to AP and open dashboard. Confirm no internet requirement, status polling, config validation, automatic NVS save on Apply and receiver-mode selection, reboot persistence, restore defaults and calibration refusal when armed/moving.
6. With props removed, check zero motor outputs at boot/disarmed, explicit arm/disarm, motor order/direction and PWM response. Verify no web request changes outputs.
7. Stop transmitter packets and verify motor shutdown by configured timeout; restore radio and verify no automatic re-arm.
8. Disconnect/disable IMU and verify arming rejected and outputs disabled. Observe C3 brownout/reset behavior during motor transients.
9. Confirm battery rail, regulator current/temperature, radio 3.3 V stability, MOSFET temperature and motor stall-current margins before any guarded, restrained flight test.

No physical hardware tests have been run.
