#ifndef DISPLAY_CONTROLLER_H
#define DISPLAY_CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

class DisplayController {
public:
  DisplayController(Adafruit_SSD1306 &display, TwoWire &wire)
    : display(display),
      wire(wire) {}

  bool begin() {
    initialized = initializeDisplay();
    if (!initialized) {
      return false;
    }

    showArming();
    initialized = checkConnection();
    return initialized;
  }

  void update() {
    const unsigned long now = millis();
    if (!initialized && now - lastRecoveryAttempt >= RECOVERY_INTERVAL) {
      lastRecoveryAttempt = now;
      recover();
    }
  }

  void render(
    int value,
    bool enabled,
    bool telemetryAvailable,
    uint8_t temperatureCelsius,
    uint8_t currentAmps,
    uint8_t voltageQuarterVolts,
    uint32_t electricalRpm
  ) {
    this->value = value;
    this->enabled = enabled;
    this->telemetryAvailable = telemetryAvailable;
    this->temperatureCelsius = temperatureCelsius;
    this->currentAmps = currentAmps;
    this->voltageQuarterVolts = voltageQuarterVolts;
    this->electricalRpm = electricalRpm;
    drawIfReady();
  }

  void sleep() {
    sleeping = true;
    if (!initialized) {
      return;
    }

    display.ssd1306_command(SSD1306_DISPLAYOFF);
    recoverIfNeeded();
  }

  void wake() {
    sleeping = false;
    if (!initialized) {
      return;
    }

    display.ssd1306_command(SSD1306_DISPLAYON);
    if (recoverIfNeeded()) {
      drawIfReady();
    }
  }

  bool showError(const __FlashStringHelper *message) {
    sleeping = false;
    if (!initialized && !initializeDisplay()) {
      return false;
    }

    display.ssd1306_command(SSD1306_DISPLAYON);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(F("ESC ERROR"));
    display.println(message);
    display.display();
    initialized = checkConnection();
    return initialized;
  }

private:
  static const uint8_t OLED_ADDRESS = 0x3C;
  static const uint8_t MOTOR_POLE_COUNT = 6;
  static const uint8_t PACK_CELL_COUNT = 3;
  static const uint16_t VOLTAGE_CALIBRATION_RAW_TENTHS = 218;
  static const uint16_t VOLTAGE_CALIBRATION_PACK_HUNDREDTHS = 1212;
  static const uint32_t I2C_CLOCK_HZ = 400000;
  static const uint32_t I2C_TIMEOUT_US = 25000;
  static const unsigned long RECOVERY_INTERVAL = 1000;

  Adafruit_SSD1306 &display;
  TwoWire &wire;
  int value = 14;
  bool enabled = false;
  bool telemetryAvailable = false;
  uint8_t temperatureCelsius = 0;
  uint8_t currentAmps = 0;
  uint8_t voltageQuarterVolts = 0;
  uint32_t electricalRpm = 0;
  bool initialized = false;
  bool sleeping = false;
  unsigned long lastRecoveryAttempt = 0;

  bool initializeDisplay() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
      return false;
    }

    display.setRotation(1);
    display.setTextColor(SSD1306_WHITE);
    return checkConnection();
  }

  bool checkConnection() {
    wire.beginTransmission(OLED_ADDRESS);
    const bool connected =
      wire.endTransmission() == 0 && !wire.getWireTimeoutFlag();
    wire.clearWireTimeoutFlag();
    return connected;
  }

  bool recoverIfNeeded() {
    if (!checkConnection()) {
      recover();
      return false;
    }
    return true;
  }

  void recover() {
    initialized = false;

    wire.end();
    wire.begin();
    wire.setClock(I2C_CLOCK_HZ);
    wire.setWireTimeout(I2C_TIMEOUT_US, true);

    if (!initializeDisplay()) {
      return;
    }

    if (sleeping) {
      display.ssd1306_command(SSD1306_DISPLAYOFF);
    } else {
      drawFrame();
    }

    initialized = checkConnection();
  }

  void drawIfReady() {
    if (!initialized) {
      return;
    }

    drawFrame();
    recoverIfNeeded();
  }

  void showArming() {
    display.clearDisplay();
    drawCenteredText("ARMING", 1, 33);
    display.display();
  }

  void drawCenteredText(
    const char *text,
    uint8_t textSize,
    int16_t y
  ) {
    int16_t x;
    int16_t textY;
    uint16_t textWidth;
    uint16_t textHeight;

    display.setTextSize(textSize);
    display.getTextBounds(
      text,
      0,
      y,
      &x,
      &textY,
      &textWidth,
      &textHeight
    );

    display.setCursor((display.width() - textWidth) / 2, y);
    display.print(text);
  }

  void drawFrame() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    char temperature[6];
    char current[5];
    char voltage[8];
    char rpm[11];
    char throttle[6];

    if (telemetryAvailable) {
      const uint16_t rawVoltageTenths =
        (static_cast<uint16_t>(voltageQuarterVolts) * 25 + 5) / 10;
      const uint16_t calibrationDenominator =
        VOLTAGE_CALIBRATION_RAW_TENTHS * PACK_CELL_COUNT;
      const uint32_t cellVoltageHundredths =
        (
          static_cast<uint32_t>(rawVoltageTenths) *
            VOLTAGE_CALIBRATION_PACK_HUNDREDTHS +
          calibrationDenominator / 2
        ) / calibrationDenominator;
      snprintf(
        temperature,
        sizeof(temperature),
        "%uC",
        static_cast<unsigned int>(temperatureCelsius)
      );
      snprintf(
        current,
        sizeof(current),
        "%uA",
        static_cast<unsigned int>(currentAmps)
      );
      snprintf(
        voltage,
        sizeof(voltage),
        "%lu.%02luV",
        static_cast<unsigned long>(cellVoltageHundredths / 100),
        static_cast<unsigned long>(cellVoltageHundredths % 100)
      );
      snprintf(
        rpm,
        sizeof(rpm),
        "%lu",
        static_cast<unsigned long>(
          electricalRpm / (MOTOR_POLE_COUNT / 2)
        )
      );
    } else {
      snprintf(temperature, sizeof(temperature), "--");
      snprintf(current, sizeof(current), "--A");
      snprintf(voltage, sizeof(voltage), "--V");
      snprintf(rpm, sizeof(rpm), "--");
    }

    if (enabled) {
      snprintf(throttle, sizeof(throttle), "%d%%", value);
    } else {
      snprintf(throttle, sizeof(throttle), "OFF");
    }

    drawCenteredText("THR", 1, 0);
    drawCenteredText(throttle, 1, 8);
    drawCenteredText("V/A", 1, 24);
    drawCenteredText(voltage, 1, 32);
    drawCenteredText(current, 1, 40);
    drawCenteredText("RPM", 1, 56);
    int16_t y = 64;
    drawWrappedCenteredText(rpm, y);
    y += 8;
    drawCenteredText("TEMP", 1, y);
    y += 8;
    drawCenteredText(temperature, 1, y);
    display.display();
  }

  void drawWrappedCenteredText(const char *text, int16_t &y) {
    const uint8_t maxCharacters = display.width() / 6;
    while (*text) {
      char line[6];
      uint8_t length = 0;
      while (*text && length < maxCharacters) {
        line[length++] = *text++;
      }
      line[length] = '\0';
      drawCenteredText(line, 1, y);
      y += 8;
    }
  }

};

#endif
