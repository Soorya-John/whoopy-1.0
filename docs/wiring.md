# Provisional pin map and wiring

No dev-board model or pin map is present in the BOM. These are proposed GPIOs only; check the chosen board schematic and pin availability before wiring. The same assignments live in the project `board_config.h` headers.

## Drone ESP32-C3

| Function | Proposed GPIO | Notes |
|---|---:|---|
| MPU6050 SDA / SCL | 5 / 6 | I²C, 400 kHz; verify board exposure and module pullups/address (0x68 assumed, 0x69 configurable). |
| nRF24 SCK / MOSI / MISO / CSN / CE | 4 / 7 / 2 / 10 / 3 | SPI; GPIO2/3/4/5/6/7/10 availability varies by C3 module/board. |
| ELRS CRSF UART RX / TX | 20 / 21 | 420 kbaud, 8N1. Connect receiver TX to controller RX; connect controller TX to receiver RX only if telemetry is needed. Share ground and use the receiver's supported supply voltage. |

The drone build routes console output through the ESP32-C3 USB Serial/JTAG controller so GPIO20/21 can serve CRSF UART1. Use the board's USB Serial/JTAG connector for programming and logs. Confirm these GPIOs are exposed and not occupied on the specific C3 board before wiring.
| motor 1..4 PWM | 0 / 1 / 8 / 9 | LEDC low-side gate signals; check strapping/flash/board conflicts before use. |

These assignments are a software starting point, not a claim that every C3 board exposes them safely. `CONFIG_WHOOPY_*_GPIO` values can be changed in `board_config.h` before build. Keep MOSFET gates pulled down in hardware so reset/boot means motor off.

## Transmitter ESP32-S3

Native USB OTG host D-/D+ use the board's USB connector and the S3 USB PHY. USB host pins are not arbitrary GPIO SPI pins; use a data-capable host connector/cable and board routing. Proposed radio SPI is SCK 12, MOSI 11, MISO 13, CSN 10, CE 9. These are provisional and must be checked against the specific S3 board/USB connector and any onboard peripherals.

The S3 must provide 5 V USB VBUS to the Boxer in host mode. The BOM does not identify a VBUS power switch, current limit, or board host connector; verify the chosen S3 board supports host power before plugging in. Do not assume its programming/device connector can supply host VBUS. Add a protected 5 V VBUS supply/switch if the board lacks one.

## Electrical constraints

- Motor current flows from battery positive through motor to MOSFET drain; source to common ground. C3 GPIO → 100 Ω → gate; 10 kΩ gate-to-ground. Fit one 100 nF capacitor directly across each motor. Verify MOSFET gate RDS(on) at 2.5 V, stall current, battery/connector/trace limits and thermal behavior.
- Motors run from the 1S battery rail. They are not driven by GPIO or the 5 V BEC.
- The BEC's 5 V may feed only a board input documented for 5 V. Never put 5 V on C3 3V3, MPU6050 supply unless that breakout accepts it, or nRF24 VCC.
- nRF24 VCC is 3.3 V. Fit 100 nF + 47 µF close to it. Confirm rail transient response.
- S3 PA+LNA supply needs a separate low-noise 3.3 V rail sized for the actual module's transmit peaks (allow at least 250 mA pending module verification); do not assume a dev-board 3V3 regulator is adequate. Common ground is required.
- No battery-voltage sensing divider is listed: battery ADC reporting is compiled out by default. Do not connect the cell directly to an ADC input.
