# Arduino Nano Encoder + OLED ESC Controller

This project is a small Arduino sketch for controlling an electronic speed controller (ESC) from a rotary encoder and a compact OLED display. It is designed for a classic 5 V Arduino Nano (ATmega328P) and uses a 128x32 SSD1306 display to show the current throttle and enable state.

## Features

- Rotary encoder-based throttle control from 0 to 100
- Encoder rotation decoded with pin-change interrupts on Nano pins 2 and 4; skipped intermediate states are inferred from rotation direction, and queued detents are applied before one display refresh
- Button press toggles the ESC on/off
- Bidirectional DShot 300 ESC control using the tested ATmega328P driver on pin D8
- DShot throttle uses a linear ramp in both directions, taking about 7 seconds from full stop to full throttle and updating at 60 Hz
- OLED dashboard in the existing portrait orientation, showing throttle, ESC temperature, current, voltage, and electrical RPM; long values wrap to a second line
- Auto-display sleep after inactivity
- DShot startup sequence for safe arming and extended telemetry enablement
- ESC telemetry is read continuously and the OLED dashboard refreshes every 50 ms (up to 20 times per second)
- OLED I²C recovery: bounded bus timeouts and OLED address checks trigger bus/display reinitialization and periodic retries without stopping the ESC control loop

## Hardware

The sketch expects the following hardware:

- Classic 5 V Arduino Nano (ATmega328P)
- SSD1306 128x32 I2C OLED display
- Rotary encoder with CLK, DT, and SW pins
- ESC signal connected to D8 (DShot)
- Common 5V/GND power distribution

## Pin mapping

| Function | Pin |
| --- | --- |
| Encoder CLK | 2 |
| Encoder DT | 4 |
| Encoder SW | 5 |
| ESC signal | 8 (DShot) |
| OLED SDA | SDA |
| OLED SCL | SCL |
| OLED GND | GND |
| OLED VCC | 3.3V or 5V depending on module |

The tested DShot driver writes the Port B register directly for its signal on D8 (PB0), so encoder CLK and DT must not use Port B pins. They use Nano pins 2 and 4 on Port D and the Port D pin-change interrupt group (PCINT2); the push button on pin 5 remains polled and debounced in the main loop. DShot output and telemetry capture require a 16 MHz ATmega328P-compatible board.

## Libraries

Install these libraries in the Arduino IDE or PlatformIO environment:

- `Wire`
- `Adafruit_GFX`
- `Adafruit_SSD1306`
- Bundled `BiDirectionalDShot` driver (GPL-3.0-only; see its `LICENSE` file)

## Operation

1. Upload the sketch to the Arduino Nano.
2. Power the ESC and display.
3. Rotate the encoder to adjust the throttle value in 1% steps below 20% and 2% steps from 20% upward.
4. Press the encoder button to enable/disable throttle output.
5. The OLED displays the throttle setting, enabled state, temperature, current, voltage, and eRPM.

## Notes

- The ESC startup routine sends DShot stop, then requests extended telemetry before normal operation. eRPM is calculated from the reported electrical period.
- Fatal ESC startup or throttle-update errors are shown on the OLED; the sketch does not log to Serial.
- The throttle is constrained to a range of 0 to 100 and is only sent to the ESC when enabled.
- Display sleep is enabled after a timeout to reduce burn-in and save power.
- Enabled throttle maps 0–100% directly to the standard DShot throttle range of 48–2047; disabled output is DShot stop (0).

## File structure

- `arduino_encoder_n_screen.ino` — main application sketch and DShot driver integration
- `DisplayController.h` — OLED display initialization and rendering
- `EncoderController.h` — rotary encoder and button input logic
- `BiDirectionalDShot/` — tested DShot driver source and license

## License

This project is provided as-is for personal and experimental use. If you plan to reuse or extend it, please keep the original attribution and follow the licensing terms of the included libraries.
