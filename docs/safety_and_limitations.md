# Safety and known limitations

Indoor prototype only. Remove propellers while flashing, configuring and debugging. Use eye protection and a LiPo-safe charging setup; inspect cells and wiring. Firmware is not flight validated and must not be relied on to protect people/property.

The state machine starts disarmed; arming requires a fresh arm rising edge, valid IMU, and throttle below the configured threshold. Explicit disarm, critical IMU failure and radio timeout drive all PWM outputs to zero. Link recovery does not re-arm. The Wi-Fi API cannot arm or command motors. Configuration writes require disarmed state.

The workbook does not identify a Boxer switch by name or provide its HID report mapping. No physical SA/SB/Sx switch is presumed by firmware. Choose a dedicated deliberate switch on the actual Boxer, map its report bit in transmitter menuconfig, and verify it on the bench. The default bit offset is only a placeholder. The transmitter requires release observed after boot/reconnect before a press can issue ARM.

PID units: gyro rate and setpoints are degrees/second. Kp is normalized motor fraction per (degree/second) of rate error; Ki is normalized motor fraction per integrated rate error (degree); Kd is normalized motor fraction per (degree/second squared) of measured rate derivative. Controller output is bounded to ±1 before motor mixing; the mixer and configured max PWM cap provide further saturation.

The MPU6050 has no magnetometer or absolute heading reference. Accelerometer/gyro fusion estimates roll/pitch but drifts in yaw; vibration and motor EMI can corrupt measurements. Initial code uses a complementary estimate and rate PID; gains need cautious bench tuning. Motor geometry, orientation, rotation direction and propeller pairing are not established by the BOM. Battery sensing is unavailable unless a correctly scaled divider is added and explicitly enabled.

Remaining prototype limits: no telemetry ACK payload, no hardware-in-loop simulator, no RC receiver redundancy, no brownout-aware motor ramp, no barometer/altitude hold, no onboard battery monitor, and no validated control gains. USB HID descriptors and board pins require verification against actual hardware.
