# Architecture and BOM review

The workbook has `Drone BOM` and `Design Notes` sheets. The BOM rows select ESP32-C3 on the aircraft, ESP32-S3 as transmitter USB host, MPU6050, standard nRF24L01 on-drone, PA+LNA nRF24 on transmitter, four 8520 motors, a 1S 600 mAh LiPo and four AO3400A/MOT3400A MOSFET candidates. It lists a 5 V BEC, 100 Ω gate resistors, 10 kΩ gate pulldowns, 100 nF motor capacitors, a 1000 µF rail capacitor and radio/local decoupling. Props, frame, wiring and charger are marked needed; MOSFET part authenticity and motor stall current are unverified.

The first design-note cell in the workbook still names an RP2040. This repository follows the requested C3 flight-controller architecture; the original Excel file is untouched. No board model, GPIO assignment, motor geometry/spin direction, MPU6050 address/orientation, voltage divider, or Boxer HID report descriptor is supplied.

The C3 runs a 250 Hz control task and independent lower-priority Wi-Fi/configuration and logging work. The S3 reads USB HID, applies configurable axis mapping/dead zones and transmits at 50 Hz. Shared protocol encoding is byte-based, little-endian, and CRC protected. See [wiring](wiring.md), [protocol](wireless_protocol.md), and [limitations](safety_and_limitations.md).

The controller uses a configurable complementary alpha for accelerometer/gyro fusion, angle outer-loop with rate PID in stabilized mode, direct rate targets in acro mode, and a provisional X mixer. Runtime web settings include PID gains, angle/rate limits, filter alpha, failsafe timeout and maximum motor PWM fraction.
