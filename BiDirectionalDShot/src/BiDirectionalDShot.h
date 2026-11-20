#ifndef BI_DIRECTIONAL_DSHOT_H
#define BI_DIRECTIONAL_DSHOT_H

#include <Arduino.h>

class BiDirectionalDShot {
public:
  static const uint8_t CaptureTickCount = 20;

  enum class Rate : uint16_t {
    Hz500 = 500,
    Hz1000 = 1000,
    Hz2000 = 2000,
    Hz4000 = 4000,
    Hz8000 = 8000
  };

  struct Telemetry {
    uint32_t rawResponse;
    uint32_t periodMicroseconds;
    uint32_t receivedPackets;
    uint32_t successfulPackets;
    uint16_t value;
    uint8_t crc;
    uint8_t expectedCrc;
    uint8_t temperatureCelsius;
    uint8_t voltageQuarterVolts;
    uint8_t currentAmps;
    uint8_t debug1;
    uint8_t debug2;
    uint8_t debug3;
    uint8_t state;
    bool gcrValid;
    bool crcValid;
    uint8_t decodedBitCount;
    uint8_t captureCount;
    uint16_t captureTicks[CaptureTickCount];
  };

  explicit BiDirectionalDShot(
      uint8_t pin = 8,
      bool inverted = true,
      Rate rate = Rate::Hz500,
      bool extendedTelemetry = false);

  bool begin();
  bool setThrottle(uint16_t value);
  bool sendCommand(uint8_t command, bool telemetryRequest = false);
  bool readTelemetry(Telemetry& telemetry);

private:
  uint8_t _pin;
  bool _inverted;
  Rate _rate;
  bool _extendedTelemetry;
  bool _begun;
  uint32_t _lastPeriodMicroseconds;
  uint32_t _receivedPackets;
  uint32_t _successfulPackets;
  uint8_t _temperatureCelsius;
  uint8_t _voltageQuarterVolts;
  uint8_t _currentAmps;
  uint8_t _debug1;
  uint8_t _debug2;
  uint8_t _debug3;
  uint8_t _state;
};

#endif
