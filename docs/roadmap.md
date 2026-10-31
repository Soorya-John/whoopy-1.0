# Development roadmap

1. Verify exact C3/S3 board variants, GPIO access, power rails, and USB host connector routing; update provisional pin maps.
2. Capture the Boxer HID report descriptor and determine channel/switch mapping on bench.
3. Compile both projects against pinned IDF 5.2.x and resolve any target-specific component/API issues.
4. Validate MPU orientation, gyro calibration, loop jitter and radio timeout with motors disconnected.
5. Validate MOSFET/battery/motor electrical limits and motor order/direction with props removed.
6. Tune low-gain restrained tests, then guarded indoor tests; record validated configuration separately.
7. Add ACK telemetry payloads, optional properly designed battery divider monitoring, and hardware-in-loop tests if needed.

No target should be marked flight-ready until each preceding safety and bench validation is complete.
