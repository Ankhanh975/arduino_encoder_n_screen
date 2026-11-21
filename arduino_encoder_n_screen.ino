#include <Wire.h>
#if defined(__AVR_ATmega328P__)
#include <avr/interrupt.h>
#endif
#include "BiDirectionalDShot/src/BiDirectionalDShot.h"
#include "BiDirectionalDShot/src/BiDirectionalDShot.cpp"
#include "DisplayController.h"
#include "EncoderController.h"

// =========================
// Hardware configuration
// =========================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1

const int CLK_PIN = 2;
const int DT_PIN  = 4;
const int SW_PIN  = 5;
const int ESC_PIN = 8;
const uint16_t MIN_DSHOT_THROTTLE = 198;
const uint16_t MAX_DSHOT_THROTTLE = 2047;

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);

// =========================
// ESC controller
// =========================

class EscController {
public:
  EscController(
    int pin,
    uint16_t minThrottle,
    uint16_t maxThrottle
  )
    : esc(
        pin,
        true,
        BiDirectionalDShot::Rate::Hz500,
        true
      ),
      minThrottle(minThrottle),
      maxThrottle(maxThrottle) {}

  bool begin() {
    if (!esc.begin() || !esc.setThrottle(0)) {
      return false;
    }

    delay(1500);
    if (!esc.sendCommand(13, true)) {
      return false;
    }
    delay(24);
    if (!esc.setThrottle(0, true)) {
      return false;
    }

    currentThrottleQ16 = 0;
    currentThrottleCommand = 0;
    lastRampUpdateMicros = micros();
    return true;
  }

  void setEnabled(bool enabled) {
    this->enabled = enabled;
  }

  void setValue(int value) {
    this->value = constrain(value, 0, 100);
  }

  bool update() {
    const uint16_t targetThrottle = enabled
      ? map(value, 0, 100, minThrottle, maxThrottle)
      : 0;
    const uint32_t targetThrottleQ16 =
      static_cast<uint32_t>(targetThrottle) * Q16_ONE;
    const uint32_t now = micros();
    const uint32_t elapsed = now - lastRampUpdateMicros;

    if (elapsed < RAMP_UPDATE_INTERVAL_US) {
      return true;
    }

    const uint32_t elapsedSteps = elapsed / RAMP_UPDATE_INTERVAL_US;
    lastRampUpdateMicros += elapsedSteps * RAMP_UPDATE_INTERVAL_US;

    if (enabled &&
        currentThrottleQ16 < DSHOT_MIN_VALID_THROTTLE_Q16) {
      currentThrottleQ16 = DSHOT_MIN_VALID_THROTTLE_Q16;
      if (!esc.setThrottle(MIN_DSHOT_THROTTLE, true)) {
        return false;
      }

      currentThrottleCommand = MIN_DSHOT_THROTTLE;
      return true;
    }

    const uint32_t remaining = currentThrottleQ16 > targetThrottleQ16
      ? currentThrottleQ16 - targetThrottleQ16
      : targetThrottleQ16 - currentThrottleQ16;
    uint32_t stepsToTarget =
      (remaining + RAMP_STEP_Q16 - 1) / RAMP_STEP_Q16;
    const uint32_t steps = min(elapsedSteps, stepsToTarget);
    const uint32_t movement = RAMP_STEP_Q16 * steps;

    if (currentThrottleQ16 < targetThrottleQ16) {
      currentThrottleQ16 =
        movement >= remaining
          ? targetThrottleQ16
          : currentThrottleQ16 + movement;
    } else if (currentThrottleQ16 > targetThrottleQ16) {
      currentThrottleQ16 =
        movement >= remaining
          ? targetThrottleQ16
          : currentThrottleQ16 - movement;
    }

    const uint16_t nextThrottleCommand =
      currentThrottleQ16 < DSHOT_MIN_VALID_THROTTLE_Q16
        ? 0
        : currentThrottleQ16 / Q16_ONE;
    if (nextThrottleCommand == currentThrottleCommand) {
      return true;
    }

    if (!esc.setThrottle(nextThrottleCommand, true)) {
      return false;
    }

    currentThrottleCommand = nextThrottleCommand;
    return true;
  }

  bool readTelemetry(BiDirectionalDShot::Telemetry &telemetry) {
    return esc.readTelemetry(telemetry);
  }

private:
  BiDirectionalDShot esc;
  uint16_t minThrottle;
  uint16_t maxThrottle;
  int value = 22;
  uint32_t currentThrottleQ16 = 0;
  uint16_t currentThrottleCommand = 0;
  uint32_t lastRampUpdateMicros = 0;
  bool enabled = false;

  static const uint32_t Q16_ONE = 65536UL;
  static const uint32_t RAMP_UPDATE_INTERVAL_US = 16667UL;
  static const uint32_t RAMP_STEPS_FULL_SCALE = 7UL * 60UL / 3UL;
  static const uint32_t RAMP_STEP_Q16 =
    (static_cast<uint32_t>(MAX_DSHOT_THROTTLE) * Q16_ONE +
     RAMP_STEPS_FULL_SCALE / 2) /
    RAMP_STEPS_FULL_SCALE;
  static const uint32_t DSHOT_MIN_VALID_THROTTLE_Q16 =
    static_cast<uint32_t>(MIN_DSHOT_THROTTLE) * Q16_ONE;
};

// =========================
// Application
// =========================

EncoderController encoder(CLK_PIN, DT_PIN, SW_PIN);
EscController escController(
  ESC_PIN,
  MIN_DSHOT_THROTTLE,
  MAX_DSHOT_THROTTLE
);
DisplayController displayController(display, Wire);

#if defined(__AVR_ATmega328P__)
ISR(PCINT2_vect) {
  encoder.handleEncoderInterrupt();
}
#endif

const unsigned long DISPLAY_SLEEP_TIMEOUT = 60000;
const unsigned long TELEMETRY_DISPLAY_INTERVAL = 50;

int value = 22;
bool enabled = false;
bool telemetryAvailable = false;
uint8_t temperatureCelsius = 0;
uint16_t currentAmps = 0;
bool currentTelemetryAvailable = false;
uint8_t voltageQuarterVolts = 0;
uint32_t electricalRpm = 0;
unsigned long lastInputTime = 0;
unsigned long lastTelemetryDisplayTime = 0;
bool displayAsleep = false;

void stopWithError(const __FlashStringHelper *message) {
  displayController.showError(message);
  while (true) {}
}

int valueStep(int direction) {
  return value < 20 || (direction < 0 && value == 20) ? 1 : 2;
}

void refreshDisplay() {
  displayController.render(
    value,
    enabled,
    telemetryAvailable,
    currentTelemetryAvailable,
    temperatureCelsius,
    currentAmps,
    voltageQuarterVolts,
    electricalRpm
  );
}

void registerInput() {
  lastInputTime = millis();
}

void wakeDisplay() {
  if (displayAsleep) {
    displayController.wake();
    displayAsleep = false;
    refreshDisplay();
  }
}

void setup() {
  value = 22;
  enabled = false;

  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(25000, true);

  encoder.begin();
  displayController.begin();

  if (!escController.begin()) {
    stopWithError(F("Failed to start DShot ESC control."));
  }

  refreshDisplay();
  lastTelemetryDisplayTime = millis();
  pinMode(1, INPUT);
  lastInputTime = millis();
}

void loop() {
  BiDirectionalDShot::Telemetry telemetry;
  if (escController.readTelemetry(telemetry) && telemetry.crcValid) {
    telemetryAvailable = true;
    temperatureCelsius = telemetry.temperatureCelsius;
    currentAmps = telemetry.currentAmps;
    currentTelemetryAvailable = telemetry.currentAvailable;
    voltageQuarterVolts = telemetry.voltageQuarterVolts;
    electricalRpm = telemetry.periodMicroseconds > 0
      ? 60000000UL / telemetry.periodMicroseconds
      : 0;
  }

  const int rotation = encoder.readRotation();
  bool refreshNeeded = false;

  if (rotation != 0 && !displayAsleep) {
    registerInput();
    int remainingRotation = rotation;
    while (remainingRotation != 0) {
      const int direction = remainingRotation > 0 ? 1 : -1;
      value += direction * valueStep(direction);
      value = constrain(value, 0, 100);
      remainingRotation -= direction;
    }
    escController.setValue(value);
    refreshNeeded = true;
  }

  if (encoder.readButtonPressed()) {
    const bool wasAsleep = displayAsleep;
    registerInput();

    if (wasAsleep) {
      wakeDisplay();
    } else {
      enabled = !enabled;
      if (!enabled) {
        value = 22;
        escController.setValue(value);
      }
      escController.setEnabled(enabled);
      refreshNeeded = true;
    }
  }

  displayController.update();

  if (!escController.update()) {
    stopWithError(F("Failed to update DShot throttle."));
  }

  const unsigned long now = millis();
  if (now - lastTelemetryDisplayTime >= TELEMETRY_DISPLAY_INTERVAL) {
    lastTelemetryDisplayTime = now;
    if (!displayAsleep) {
      refreshNeeded = true;
    }
  }

  if (refreshNeeded && !displayAsleep) {
    refreshDisplay();
  }

  if (!enabled &&
      !displayAsleep &&
      millis() - lastInputTime >= DISPLAY_SLEEP_TIMEOUT) {
    displayController.sleep();
    displayAsleep = true;
  }
}
