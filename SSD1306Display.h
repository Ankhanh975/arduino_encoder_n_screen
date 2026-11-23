#ifndef SSD1306_DISPLAY_H
#define SSD1306_DISPLAY_H

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

#endif
