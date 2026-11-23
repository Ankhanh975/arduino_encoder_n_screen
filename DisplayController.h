#ifndef DISPLAY_CONTROLLER_H
#define DISPLAY_CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <string.h>

class SSD1306Display : public Adafruit_GFX {
public:
  enum {
    WHITE = 1,
    BLACK = 0,
    DISPLAY_OFF = 0xAE,
    DISPLAY_ON = 0xAF
  };

  explicit SSD1306Display(TwoWire &wire)
    : Adafruit_GFX(DISPLAY_WIDTH, DISPLAY_HEIGHT),
      wire(wire) {}

  bool begin(uint8_t address) {
    this->address = address;
    memset(buffer, 0, sizeof(buffer));

    static const uint8_t initializationCommands[] = {
      0xAE,       // Display off
      0xD5, 0x80, // Clock divide ratio
      0xA8, 0x1F, // Multiplex ratio for 128x32
      0xD3, 0x00, // Display offset
      0x40,       // Start line
      0x8D, 0x14, // Charge pump
      0x20, 0x00, // Horizontal addressing mode
      0xA1,       // Segment remap
      0xC8,       // COM scan direction
      0xDA, 0x02, // COM pins for 128x32
      0x81, 0x8F, // Contrast
      0xD9, 0xF1, // Pre-charge period
      0xDB, 0x40, // VCOMH deselect level
      0xA4,       // Resume display from RAM
      0xA6,       // Normal display
      DISPLAY_ON
    };

    if (!sendPacket(0x00, initializationCommands,
                    sizeof(initializationCommands))) {
      return false;
    }

    markAllDirty();
    return true;
  }

  bool command(uint8_t command) {
    return sendPacket(0x00, &command, 1);
  }

  void clearDisplay() {
    for (uint8_t page = 0; page < DISPLAY_PAGES; ++page) {
      for (uint8_t column = 0; column < DISPLAY_WIDTH; ++column) {
        const uint16_t index =
          static_cast<uint16_t>(page) * DISPLAY_WIDTH + column;
        if (buffer[index] != 0) {
          buffer[index] = 0;
          markDirty(column, page);
        }
      }
    }
  }

  bool display() {
    if (!dirty) {
      return true;
    }

    const uint8_t window[] = {
      0x21,
      dirtyColumnMin,
      dirtyColumnMax,
      0x22,
      dirtyPageMin,
      dirtyPageMax
    };
    if (!sendPacket(0x00, window, sizeof(window))) {
      return false;
    }

    uint8_t chunk[31];
    uint8_t chunkLength = 0;
    uint8_t page = dirtyPageMin;
    uint8_t column = dirtyColumnMin;
    while (page <= dirtyPageMax) {
      chunk[chunkLength++] =
        buffer[static_cast<uint16_t>(page) * DISPLAY_WIDTH + column];

      if (column == dirtyColumnMax) {
        column = dirtyColumnMin;
        ++page;
      } else {
        ++column;
      }

      if (chunkLength == sizeof(chunk) || page > dirtyPageMax) {
        if (!sendPacket(0x40, chunk, chunkLength)) {
          return false;
        }
        chunkLength = 0;
      }
    }

    dirty = false;
    return true;
  }

  void drawPixel(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || x >= width() || y < 0 || y >= height()) {
      return;
    }

    switch (getRotation()) {
      case 1: {
        const int16_t oldX = x;
        x = DISPLAY_WIDTH - 1 - y;
        y = oldX;
        break;
      }
      case 2:
        x = DISPLAY_WIDTH - 1 - x;
        y = DISPLAY_HEIGHT - 1 - y;
        break;
      case 3: {
        const int16_t oldX = x;
        x = y;
        y = DISPLAY_HEIGHT - 1 - oldX;
        break;
      }
    }

    const uint8_t page = static_cast<uint8_t>(y / 8);
    const uint16_t index =
      static_cast<uint16_t>(page) * DISPLAY_WIDTH + x;
    const uint8_t bit = static_cast<uint8_t>(1U << (y & 7));
    const uint8_t updated = color == WHITE
      ? buffer[index] | bit
      : buffer[index] & static_cast<uint8_t>(~bit);

    if (updated != buffer[index]) {
      buffer[index] = updated;
      markDirty(static_cast<uint8_t>(x), page);
    }
  }

private:
  static const uint8_t DISPLAY_WIDTH = 128;
  static const uint8_t DISPLAY_HEIGHT = 32;
  static const uint8_t DISPLAY_PAGES = DISPLAY_HEIGHT / 8;

  TwoWire &wire;
  uint8_t address = 0;
  uint8_t buffer[DISPLAY_WIDTH * DISPLAY_PAGES] = {};
  bool dirty = false;
  uint8_t dirtyColumnMin = DISPLAY_WIDTH - 1;
  uint8_t dirtyColumnMax = 0;
  uint8_t dirtyPageMin = DISPLAY_PAGES - 1;
  uint8_t dirtyPageMax = 0;

  bool sendPacket(
    uint8_t control,
    const uint8_t *bytes,
    uint8_t length
  ) {
    wire.beginTransmission(address);
    const bool complete =
      wire.write(control) == 1 &&
      wire.write(bytes, length) == length;
    const uint8_t status = wire.endTransmission();
    const bool timedOut = wire.getWireTimeoutFlag();
    wire.clearWireTimeoutFlag();
    return complete && status == 0 && !timedOut;
  }

  void markAllDirty() {
    dirty = true;
    dirtyColumnMin = 0;
    dirtyColumnMax = DISPLAY_WIDTH - 1;
    dirtyPageMin = 0;
    dirtyPageMax = DISPLAY_PAGES - 1;
  }

  void markDirty(uint8_t column, uint8_t page) {
    if (!dirty) {
      dirty = true;
      dirtyColumnMin = column;
      dirtyColumnMax = column;
      dirtyPageMin = page;
      dirtyPageMax = page;
      return;
    }

    if (column < dirtyColumnMin) dirtyColumnMin = column;
    if (column > dirtyColumnMax) dirtyColumnMax = column;
    if (page < dirtyPageMin) dirtyPageMin = page;
    if (page > dirtyPageMax) dirtyPageMax = page;
  }
};

class DisplayController {
public:
  DisplayController(SSD1306Display &display, TwoWire &wire)
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
    bool currentTelemetryAvailable,
    uint8_t temperatureCelsius,
    uint16_t currentAmps,
    uint8_t voltageQuarterVolts,
    uint32_t electricalRpm
  ) {
    this->value = value;
    this->enabled = enabled;
    this->telemetryAvailable = telemetryAvailable;
    this->currentTelemetryAvailable = currentTelemetryAvailable;
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

    display.command(SSD1306Display::DISPLAY_OFF);
    recoverIfNeeded();
  }

  void wake() {
    sleeping = false;
    if (!initialized) {
      return;
    }

    display.command(SSD1306Display::DISPLAY_ON);
    if (recoverIfNeeded()) {
      drawIfReady();
    }
  }

  bool showError(const __FlashStringHelper *message) {
    sleeping = false;
    if (!initialized && !initializeDisplay()) {
      return false;
    }

    hasRenderedFrame = false;
    display.command(SSD1306Display::DISPLAY_ON);
    display.clearDisplay();
    display.setTextColor(SSD1306Display::WHITE, SSD1306Display::BLACK);
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

  SSD1306Display &display;
  TwoWire &wire;
  int value = 14;
  bool enabled = false;
  bool telemetryAvailable = false;
  bool currentTelemetryAvailable = false;
  uint8_t temperatureCelsius = 0;
  uint16_t currentAmps = 0;
  uint8_t voltageQuarterVolts = 0;
  uint32_t electricalRpm = 0;
  bool initialized = false;
  bool sleeping = false;
  bool hasRenderedFrame = false;
  uint8_t lastRpmRows = 0;
  char lastTemperature[6] = {};
  char lastCurrent[7] = {};
  char lastVoltage[8] = {};
  char lastBattery[5] = {};
  char lastRpm[11] = {};
  char lastThrottle[6] = {};
  unsigned long lastRecoveryAttempt = 0;

  bool initializeDisplay() {
    if (!display.begin(OLED_ADDRESS)) {
      return false;
    }

    hasRenderedFrame = false;
    display.setRotation(1);
    display.setTextColor(SSD1306Display::WHITE, SSD1306Display::BLACK);
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
      display.command(SSD1306Display::DISPLAY_OFF);
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

  uint8_t estimateBatteryPercentage(uint16_t cellVoltageHundredths) {
    struct BatteryVoltagePoint {
      uint16_t voltageHundredths;
      uint8_t percentage;
    };

    static const BatteryVoltagePoint curve[] = {
      {300,  0},
      {330,  3},
      {340,  5},
      {350, 10},
      {360, 20},
      {365, 30},
      {370, 50},
      {375, 60},
      {380, 70},
      {385, 80},
      {390, 90},
      {395, 95},
      {400, 97},
      {405, 98},
      {410, 99},
      {415, 99},
      {420, 100}
    };

    const uint8_t pointCount =
      sizeof(curve) / sizeof(curve[0]);

    if (cellVoltageHundredths <= curve[0].voltageHundredths) {
      return 0;
    }

    for (uint8_t i = 1; i < pointCount; ++i) {
      if (cellVoltageHundredths <= curve[i].voltageHundredths) {
        const uint16_t voltageRange =
          curve[i].voltageHundredths -
          curve[i - 1].voltageHundredths;

        const uint16_t voltageAboveLowerPoint =
          cellVoltageHundredths -
          curve[i - 1].voltageHundredths;

        const uint8_t percentageRange =
          curve[i].percentage -
          curve[i - 1].percentage;

        return static_cast<uint8_t>(
          curve[i - 1].percentage +
          (
            static_cast<uint32_t>(voltageAboveLowerPoint) *
              percentageRange +
            voltageRange / 2
          ) / voltageRange
        );
      }
    }

    return 100;
  }

  void drawFrame() {
    display.setTextColor(SSD1306Display::WHITE, SSD1306Display::BLACK);

    char temperature[6];
    char current[7];
    char voltage[8];
    char battery[5];
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
      const uint8_t batteryPercentage = estimateBatteryPercentage(
        static_cast<uint16_t>(cellVoltageHundredths)
      );
      snprintf(
        temperature,
        sizeof(temperature),
        "%uC",
        static_cast<unsigned int>(temperatureCelsius)
      );
      if (currentTelemetryAvailable) {
        snprintf(
          current,
          sizeof(current),
          "%uA",
          static_cast<unsigned int>(currentAmps)
        );
      } else {
        snprintf(current, sizeof(current), "--A");
      }
      snprintf(
        voltage,
        sizeof(voltage),
        "%lu.%02luV",
        static_cast<unsigned long>(cellVoltageHundredths / 100),
        static_cast<unsigned long>(cellVoltageHundredths % 100)
      );
      snprintf(
        battery,
        sizeof(battery),
        "%u%%",
        static_cast<unsigned int>(batteryPercentage)
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
      snprintf(battery, sizeof(battery), "--%%");
      snprintf(rpm, sizeof(rpm), "--");
    }

    if (enabled) {
      snprintf(throttle, sizeof(throttle), "%d%%", value);
    } else {
      snprintf(throttle, sizeof(throttle), "OFF");
    }

    const uint8_t rpmRows = wrappedLineCount(rpm);
    const int16_t lowerPanelY = 40 + rpmRows * 8;

    if (hasRenderedFrame &&
        strcmp(temperature, lastTemperature) == 0 &&
        strcmp(current, lastCurrent) == 0 &&
        strcmp(voltage, lastVoltage) == 0 &&
        strcmp(battery, lastBattery) == 0 &&
        strcmp(rpm, lastRpm) == 0 &&
        strcmp(throttle, lastThrottle) == 0) {
      return;
    }

    if (!hasRenderedFrame) {
      display.clearDisplay();
      drawCenteredText("THR", 1, 0);
      drawTextRow(throttle, 8);
      drawCenteredText("RPM", 1, 24);
      int16_t rpmY = 32;
      drawWrappedCenteredText(rpm, rpmY);
      drawLowerPanel(
        lowerPanelY,
        voltage,
        current,
        temperature,
        battery
      );
    } else {
      const bool throttleChanged =
        strcmp(throttle, lastThrottle) != 0;
      const bool rpmChanged = strcmp(rpm, lastRpm) != 0;
      const bool layoutChanged = rpmRows != lastRpmRows;

      if (throttleChanged) {
        drawTextRow(throttle, 8);
      }

      if (rpmChanged) {
        const uint8_t rowsToClear =
          rpmRows > lastRpmRows ? rpmRows : lastRpmRows;
        display.fillRect(
          0,
          32,
          display.width(),
          rowsToClear * 8,
          SSD1306Display::BLACK
        );
        int16_t rpmY = 32;
        drawWrappedCenteredText(rpm, rpmY);
      }

      if (layoutChanged) {
        const int16_t oldLowerPanelY = 40 + lastRpmRows * 8;
        const int16_t clearFrom =
          oldLowerPanelY < lowerPanelY
            ? oldLowerPanelY
            : lowerPanelY;
        display.fillRect(
          0,
          clearFrom,
          display.width(),
          display.height() - clearFrom,
          SSD1306Display::BLACK
        );
        drawLowerPanel(
          lowerPanelY,
          voltage,
          current,
          temperature,
          battery
        );
      } else {
        if (strcmp(voltage, lastVoltage) != 0) {
          drawTextRow(voltage, lowerPanelY + 8);
        }
        if (strcmp(current, lastCurrent) != 0) {
          drawTextRow(current, lowerPanelY + 16);
        }
        if (strcmp(temperature, lastTemperature) != 0) {
          drawTextRow(temperature, lowerPanelY + 40);
        }
        if (strcmp(battery, lastBattery) != 0) {
          drawTextRow(battery, lowerPanelY + 64);
        }
      }
    }

    if (!display.display()) {
      return;
    }

    strcpy(lastTemperature, temperature);
    strcpy(lastCurrent, current);
    strcpy(lastVoltage, voltage);
    strcpy(lastBattery, battery);
    strcpy(lastRpm, rpm);
    strcpy(lastThrottle, throttle);
    lastRpmRows = rpmRows;
    hasRenderedFrame = true;
  }

  uint8_t wrappedLineCount(const char *text) {
    const uint8_t maxCharacters = display.width() / 6;
    const size_t textLength = strlen(text);
    return static_cast<uint8_t>(
      (textLength + maxCharacters - 1) / maxCharacters
    );
  }

  void drawTextRow(const char *text, int16_t y) {
    display.fillRect(
      0,
      y,
      display.width(),
      8,
      SSD1306Display::BLACK
    );
    drawCenteredText(text, 1, y);
  }

  void drawLowerPanel(
    int16_t y,
    const char *voltage,
    const char *current,
    const char *temperature,
    const char *battery
  ) {
    drawCenteredText("V/A", 1, y);
    drawCenteredText(voltage, 1, y + 8);
    drawCenteredText(current, 1, y + 16);
    drawCenteredText("TEMP", 1, y + 32);
    drawCenteredText(temperature, 1, y + 40);
    drawCenteredText("BAT", 1, y + 56);
    drawCenteredText(battery, 1, y + 64);
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
