#include "BiDirectionalDShot.h"

#include <avr/interrupt.h>
#include <util/atomic.h>

#if !defined(__AVR_ATmega328P__) || F_CPU != 16000000UL
#error "BiDirectionalDShot requires a 16 MHz ATmega328P (Arduino Uno compatible board)."
#endif

namespace {

const uint8_t dshotPin = 8;
const uint8_t captureBufferSize = 20;

volatile uint16_t frame;
volatile uint32_t capturedResponse;
volatile bool responseAvailable;
volatile uint8_t capturedDecodedBitCount;
volatile uint8_t capturedEdgeCount;
volatile uint16_t capturedEdgeTicks[captureBufferSize];
bool driverStarted;
bool invertedSignal;
uint16_t updateRate;
uint16_t edgeDurations[captureBufferSize];
uint8_t durationBits[] = {
  0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4
};

#define DELAY_CYCLES(n) __builtin_avr_delay_cycles(n)

uint8_t calculateCrc(uint16_t value, bool inverted) {
  uint8_t crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
  return inverted ? (~crc & 0x0F) : crc;
}

uint16_t buildFrame(uint16_t value, bool telemetryRequest, bool inverted) {
  const uint16_t payload = (value << 1) | (telemetryRequest ? 1 : 0);
  return (payload << 4) | calculateCrc(payload, inverted);
}

uint8_t revertMapping(uint8_t value) {
  switch(value) {
    case 0x19: return 0x00;
    case 0x1B: return 0x01;
    case 0x12: return 0x02;
    case 0x13: return 0x03;
    case 0x1D: return 0x04;
    case 0x15: return 0x05;
    case 0x16: return 0x06;
    case 0x17: return 0x07;
    case 0x1A: return 0x08;
    case 0x09: return 0x09;
    case 0x0A: return 0x0A;
    case 0x0B: return 0x0B;
    case 0x1E: return 0x0C;
    case 0x0D: return 0x0D;
    case 0x0E: return 0x0E;
    case 0x0F: return 0x0F;
    default: return 0xFF;
  }
}

bool mapResponse(uint32_t response, uint16_t& mapped) {
  mapped = 0;
  uint8_t shift = 0;

  for(uint8_t bit = 0; bit < 20; bit += 5) {
    const uint8_t nibble = revertMapping((response >> bit) & 0x1F);
    if(nibble == 0xFF) {
      return false;
    }
    mapped |= static_cast<uint16_t>(nibble) << shift;
    shift += 4;
  }
  return true;
}

void processTelemetryResponse() {
  pinMode(dshotPin, INPUT_PULLUP);
  DELAY_CYCLES(80);

  register uint8_t captureRisingEdge = 0b01000000;
  register uint16_t previousValue = 0;
  register uint8_t timerFlags;
  register uint16_t* capture = edgeDurations;

  TCCR1A = 0b00000001;
  TCCR1B = 0b00000010;
  OCR1A = 250;
  TCNT1 = 0x00;
  TIFR1 = (1 << ICF1) | (1 << OCF1A) | (1 << TOV1);

  for(; capture <= &edgeDurations[captureBufferSize - 1];) {
    while(!(timerFlags = (TIFR1 & ((1 << ICF1) | (1 << OCF1A))))) {}

    const uint16_t value = ICR1;
    if((timerFlags & (1 << OCF1A)) && capture != edgeDurations) {
      break;
    }

    TCCR1B ^= captureRisingEdge;
    TIFR1 = (1 << ICF1) | (1 << OCF1A);
    *capture = value - previousValue;
    previousValue = value;
    ++capture;
  }

  while(capture <= &edgeDurations[captureBufferSize - 1]) {
    *capture++ = 0;
  }
  pinMode(dshotPin, OUTPUT);

  uint32_t response = 0x001FFFFF;
  uint8_t bitValue = 0;
  uint8_t bitCount = 0;
  for(uint8_t i = 1; i < captureBufferSize; ++i) {
    if(edgeDurations[i] == 0 || edgeDurations[i] >= sizeof(durationBits)) {
      break;
    }

    bitValue ^= 0x01;
    const uint8_t duration = durationBits[edgeDurations[i]];
    for(uint8_t j = 0; j < duration && bitCount < 21; ++j) {
      response ^= static_cast<uint32_t>(bitValue) << (20 - bitCount++);
    }
  }

  response ^= (response >> 1);
  capturedDecodedBitCount = bitCount;
  for(uint8_t i = 0; i < captureBufferSize; ++i) {
    capturedEdgeTicks[i] = edgeDurations[i];
  }
  capturedEdgeCount = capture - edgeDurations;
  capturedResponse = response;
  responseAvailable = true;
}

void sendDshot300Bit(uint8_t bit) {
  if(bit) {
    PORTB = B00000001;
    DELAY_CYCLES(37);
    PORTB = B00000000;
    DELAY_CYCLES(7);
  } else {
    PORTB = B00000001;
    DELAY_CYCLES(16);
    PORTB = B00000000;
    DELAY_CYCLES(25);
  }
}

void sendInvertedDshot300Bit(uint8_t bit) {
  if(bit) {
    PORTB = B00000000;
    DELAY_CYCLES(37);
    PORTB = B00000001;
    DELAY_CYCLES(7);
  } else {
    PORTB = B00000000;
    DELAY_CYCLES(16);
    PORTB = B00000001;
    DELAY_CYCLES(25);
  }
}

void sendDshot300Frame() {
  uint16_t currentFrame = frame;
  for(uint8_t offset = 0; offset < 16; ++offset) {
    const uint8_t bit = (currentFrame & 0x8000) >> 15;
    if(invertedSignal) {
      sendInvertedDshot300Bit(bit);
    } else {
      sendDshot300Bit(bit);
    }
    currentFrame <<= 1;
  }
}

void setupTimer(uint16_t rate) {
  cli();
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  switch(rate) {
    case 500:
      OCR2A = 124;
      TCCR2B |= 0b00000110;
      break;
    case 1000:
      OCR2A = 124;
      TCCR2B |= 0b00000101;
      break;
    case 2000:
      OCR2A = 124;
      TCCR2B |= 0b00000100;
      break;
    case 4000:
      OCR2A = 124;
      TCCR2B |= 0b00000011;
      break;
    case 8000:
      OCR2A = 249;
      TCCR2B |= 0b00000010;
      break;
    default:
      return;
  }

  TCCR2A |= 0b00001010;
  TIMSK2 = 0b00000010;
  sei();
}

bool isValidRate(BiDirectionalDShot::Rate rate) {
  switch(rate) {
    case BiDirectionalDShot::Rate::Hz500:
    case BiDirectionalDShot::Rate::Hz1000:
    case BiDirectionalDShot::Rate::Hz2000:
    case BiDirectionalDShot::Rate::Hz4000:
    case BiDirectionalDShot::Rate::Hz8000:
      return true;
    default:
      return false;
  }
}

} // namespace

ISR(TIMER2_COMPA_vect) {
  sendDshot300Frame();
  if(invertedSignal) {
    processTelemetryResponse();
  }
}

BiDirectionalDShot::BiDirectionalDShot(
    uint8_t pin,
    bool inverted,
    Rate rate,
    bool extendedTelemetry)
  : _pin(pin),
    _inverted(inverted),
    _rate(rate),
    _extendedTelemetry(extendedTelemetry),
    _begun(false),
    _lastPeriodMicroseconds(0),
    _receivedPackets(0),
    _successfulPackets(0),
    _temperatureCelsius(0),
    _voltageQuarterVolts(0),
    _currentAmps(0),
    _debug1(0),
    _debug2(0),
    _debug3(0),
    _state(0) {}

bool BiDirectionalDShot::begin() {
  if(_begun || driverStarted || _pin != dshotPin || !isValidRate(_rate)) {
    return false;
  }

  invertedSignal = _inverted;
  updateRate = static_cast<uint16_t>(_rate);
  frame = buildFrame(0, false, invertedSignal);
  responseAvailable = false;

  pinMode(dshotPin, OUTPUT);
  PORTB = invertedSignal ? B00000001 : B00000000;

  driverStarted = true;
  setupTimer(updateRate);
  _begun = true;
  return true;
}

bool BiDirectionalDShot::setThrottle(uint16_t value) {
  if(!_begun || value > 2047) {
    return false;
  }

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    frame = buildFrame(value, false, invertedSignal);
  }
  return true;
}

bool BiDirectionalDShot::sendCommand(uint8_t command, bool telemetryRequest) {
  if(!_begun || command > 47) {
    return false;
  }

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    frame = buildFrame(command, telemetryRequest, invertedSignal);
  }
  return true;
}

bool BiDirectionalDShot::readTelemetry(Telemetry& telemetry) {
  if(!_begun || !_inverted) {
    return false;
  }

  uint32_t response;
  bool available;
  uint8_t decodedBitCount;
  uint8_t edgeCount;
  uint16_t edgeTicks[CaptureTickCount];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    available = responseAvailable;
    if(available) {
      response = capturedResponse;
      decodedBitCount = capturedDecodedBitCount;
      edgeCount = capturedEdgeCount;
      for(uint8_t i = 0; i < CaptureTickCount; ++i) {
        edgeTicks[i] = capturedEdgeTicks[i];
      }
      responseAvailable = false;
    }
  }
  if(!available) {
    return false;
  }

  uint16_t mapped = 0;
  const bool mappingValid = mapResponse(response, mapped);
  const uint16_t value = mapped >> 4;
  const uint8_t crc = mapped & 0x0F;
  const uint8_t expectedCrc = calculateCrc(value, _inverted);
  const bool crcValid = mappingValid && crc == expectedCrc;

  ++_receivedPackets;
  if(crcValid) {
    ++_successfulPackets;
  }

  const uint32_t periodBase = value & 0x01FF;
  const uint8_t periodShift = (value >> 9) & 0x07;
  uint32_t periodMicroseconds = periodBase << periodShift;
  const uint8_t packageType = (value >> 8) & 0x0F;

  if(crcValid && _extendedTelemetry) {
    if((packageType & 0x01) == 0) {
      switch(packageType) {
        case 0x02: _temperatureCelsius = periodBase; break;
        case 0x04: _voltageQuarterVolts = periodBase; break;
        case 0x06: _currentAmps = periodBase; break;
        case 0x08: _debug1 = periodBase; break;
        case 0x0A: _debug2 = periodBase; break;
        case 0x0C: _debug3 = periodBase; break;
        case 0x0E: _state = periodBase; break;
      }
      periodMicroseconds = _lastPeriodMicroseconds;
    } else {
      _lastPeriodMicroseconds = periodMicroseconds;
    }
  } else if(crcValid) {
    _lastPeriodMicroseconds = periodMicroseconds;
  }

  telemetry.rawResponse = response;
  telemetry.periodMicroseconds = periodMicroseconds;
  telemetry.receivedPackets = _receivedPackets;
  telemetry.successfulPackets = _successfulPackets;
  telemetry.value = value;
  telemetry.crc = crc;
  telemetry.expectedCrc = expectedCrc;
  telemetry.temperatureCelsius = _temperatureCelsius;
  telemetry.voltageQuarterVolts = _voltageQuarterVolts;
  telemetry.currentAmps = _currentAmps;
  telemetry.debug1 = _debug1;
  telemetry.debug2 = _debug2;
  telemetry.debug3 = _debug3;
  telemetry.state = _state;
  telemetry.gcrValid = mappingValid;
  telemetry.crcValid = crcValid;
  telemetry.decodedBitCount = decodedBitCount;
  telemetry.captureCount = edgeCount;
  for(uint8_t i = 0; i < CaptureTickCount; ++i) {
    telemetry.captureTicks[i] = edgeTicks[i];
  }
  return true;
}
