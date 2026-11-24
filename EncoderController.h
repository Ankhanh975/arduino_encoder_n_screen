#ifndef ENCODER_CONTROLLER_H
#define ENCODER_CONTROLLER_H

#include <Arduino.h>

#if defined(__AVR_ATmega328P__)
#include <avr/io.h>
#include <util/atomic.h>
#endif

class EncoderController {
public:
  enum class ButtonEvent : uint8_t {
    None,
    ShortClick,
    LongClick
  };

  EncoderController(int clkPin, int dtPin, int switchPin)
    : clkPin(clkPin),
      dtPin(dtPin),
      switchPin(switchPin) {}

  void begin() {
    pinMode(clkPin, INPUT_PULLUP);
    pinMode(dtPin, INPUT_PULLUP);
    pinMode(switchPin, INPUT_PULLUP);

    lastEncoderState = readEncoderState();
    lastButtonReading = HIGH;
    stableButtonState = HIGH;
    lastDebounceTime = 0;
    longPressReported = false;
    transitionCount = 0;
    lastDirection = 0;
    unresolvedSkippedTransition = false;

#if defined(__AVR_ATmega328P__)
    if (clkPin >= 0 && clkPin <= 7 &&
        dtPin >= 0 && dtPin <= 7) {
      const uint8_t pinChangeMask =
        _BV(clkPin) | _BV(dtPin);
      PCMSK2 |= pinChangeMask;
      PCIFR |= _BV(PCIF2);
      PCICR |= _BV(PCIE2);
      interruptMode = true;
    }
#endif
  }

  int readRotation() {
#if defined(__AVR_ATmega328P__)
    if (interruptMode) {
      int direction = 0;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        direction = consumeRotation();
      }
      return direction;
    }
#endif

    updateEncoderState();
    return consumeRotation();
  }

  void handleEncoderInterrupt() {
    updateEncoderState();
  }

  ButtonEvent readButtonEvent() {
    const bool reading = digitalRead(switchPin);
    const unsigned long now = millis();
    ButtonEvent event = ButtonEvent::None;

    if (reading != lastButtonReading) {
      lastDebounceTime = now;
    }

    if (now - lastDebounceTime > DEBOUNCE_TIME &&
        reading != stableButtonState) {
      stableButtonState = reading;
      if (stableButtonState == LOW) {
        buttonPressStartedAt = now;
        longPressReported = false;
      } else {
        if (!longPressReported) {
          event = now - buttonPressStartedAt >= LONG_CLICK_TIME
            ? ButtonEvent::LongClick
            : ButtonEvent::ShortClick;
        }
        longPressReported = false;
      }
    }

    if (stableButtonState == LOW &&
        !longPressReported &&
        now - buttonPressStartedAt >= LONG_CLICK_TIME) {
      event = ButtonEvent::LongClick;
      longPressReported = true;
    }

    lastButtonReading = reading;
    return event;
  }

  bool isButtonHeld() const {
    return stableButtonState == LOW;
  }

private:
  static const unsigned long DEBOUNCE_TIME = 30;
  static const unsigned long LONG_CLICK_TIME = 1000;

  int clkPin;
  int dtPin;
  int switchPin;
  volatile uint8_t lastEncoderState = 0;
  volatile int transitionCount = 0;
  volatile int lastDirection = 0;
  volatile bool unresolvedSkippedTransition = false;
  bool interruptMode = false;
  bool lastButtonReading = HIGH;
  bool stableButtonState = HIGH;
  bool longPressReported = false;
  unsigned long lastDebounceTime = 0;
  unsigned long buttonPressStartedAt = 0;

  void updateEncoderState() {
    const uint8_t currentState = readEncoderState();
    int direction = 0;
    int transitionMagnitude = 1;

    if (currentState != lastEncoderState) {
      direction = transitionDirection(lastEncoderState, currentState);

      if (direction != 0) {
        if (unresolvedSkippedTransition) {
          transitionCount += direction * 2;
          unresolvedSkippedTransition = false;
        }
        lastDirection = direction;
      } else if ((lastEncoderState ^ currentState) == 0x03 &&
                 lastDirection != 0) {
        direction = lastDirection;
        transitionMagnitude = 2;
      } else if ((lastEncoderState ^ currentState) == 0x03) {
        unresolvedSkippedTransition = true;
      }

      transitionCount += direction * transitionMagnitude;
      lastEncoderState = currentState;
    }
  }

  int consumeRotation() {
    const int rotation = transitionCount / 2;
    transitionCount -= rotation * 2;
    return rotation;
  }

  uint8_t readEncoderState() const {
#if defined(__AVR_ATmega328P__)
    if (clkPin >= 0 && clkPin <= 7 &&
        dtPin >= 0 && dtPin <= 7) {
      const uint8_t portState = PIND;
      return (((portState >> clkPin) & 1) << 1) |
             ((portState >> dtPin) & 1);
    }
#endif
    return (digitalRead(clkPin) << 1) | digitalRead(dtPin);
  }

  int transitionDirection(
    uint8_t previousState,
    uint8_t currentState
  ) const {
    if ((previousState == 3 && currentState == 1) ||
        (previousState == 1 && currentState == 0) ||
        (previousState == 0 && currentState == 2) ||
        (previousState == 2 && currentState == 3)) {
      return 1;
    }

    if ((previousState == 1 && currentState == 3) ||
        (previousState == 0 && currentState == 1) ||
        (previousState == 2 && currentState == 0) ||
        (previousState == 3 && currentState == 2)) {
      return -1;
    }

    return 0;
  }
};

#endif
