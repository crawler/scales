#include "Scales.h"

// TEENSYDUINO has a port of Dean Camera's ATOMIC_BLOCK macros for AVR to ARM
// Cortex M3.
#define HAS_ATOMIC_BLOCK (defined(ARDUINO_ARCH_AVR) || defined(TEENSYDUINO))

// Whether we are running on either the ESP8266 or the ESP32.
#define ARCH_ESPRESSIF \
  (defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32))

// Whether we are actually running on FreeRTOS.
#define IS_FREE_RTOS defined(ARDUINO_ARCH_ESP32)

// Define macro designating whether we're running on a reasonable
// fast CPU and so should slow down sampling from GPIO.
#define FAST_CPU                                                \
  (ARCH_ESPRESSIF || defined(ARDUINO_ARCH_SAM) ||               \
   defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_STM32) || \
   defined(TEENSYDUINO))

#if HAS_ATOMIC_BLOCK
// Acquire AVR-specific ATOMIC_BLOCK(ATOMIC_RESTORESTATE) macro.
#include <util/atomic.h>
#endif

Scales::Scales(uint8_t clock_pin, uint8_t data_pin, double scale, uint8_t gain, uint8_t tare) {
  clock_pin_ = clock_pin;
  data_pin_ = data_pin;
  this->SetGain(gain);
  scale_ = scale;
  tare_ = tare;
  Scales::RegisterScales(*this);
}

void Scales::SetGain(uint8_t gain) {
  switch (gain) {
    case 128:  // channel A, gain factor 128
      gain_ = 1;
      break;
    case 64:  // channel A, gain factor 64
      gain_ = 3;
      break;
    case 32:  // channel B, gain factor 32
      gain_ = 2;
      break;
  }
}

void Scales::SetTareWithAvg() {
  tare_ = GetAvgValue();
}

bool Scales::IsReady() {
  return digitalRead(data_pin_) == LOW;
}

void Scales::PullBit() {
  value_buffer_ = value_buffer_ << 1;
  if(digitalRead(data_pin_)) value_buffer_++;
  received_bit_count_++;

  if (received_bit_count_ >= 24) {
    last_value_ = Extend24To32(value_buffer_);
    received_values_.push(last_value_);
    value_buffer_ = 0;
    received_bit_count_ = 0;
  }
}

int32_t Scales::GetLastValue() {
  return last_value_;
}

double Scales::GetLastUnits() {
  return ((double)GetLastValue() - tare_) / scale_;
}

int32_t Scales::GetAvgValue() {
  int32_t sum = 0;

  for(int32_t value : received_values_) {
    sum += value;
  }

  return sum / received_values_.size();
}

double Scales::GetAvgUnits() {
  return ((double)GetAvgValue() - tare_) / scale_;
}

Scales::ScalesMap Scales::scales_by_clock_pin_;

void Scales::RegisterScales(Scales& scales) {
  Scales::scales_by_clock_pin_[scales.clock_pin_].push_back(
      (ScalesWrapper)scales);
}

void Scales::InitPins() {
  for (auto& pair : Scales::scales_by_clock_pin_) {
    pinMode(pair.first, OUTPUT);
    for (Scales& scales : pair.second) {
      pinMode(scales.data_pin_, INPUT_PULLUP);
    }
  }
}

void Scales::PullData(uint8_t clock_pin, ScalesList& scalesOnClock) {
  for (size_t i = 0; i < 24; ++i) {
    digitalWrite(clock_pin, HIGH);
    // delayMicroseconds(1);
    for (Scales& scales : scalesOnClock) {
      scales.PullBit();
    }
    digitalWrite(clock_pin, LOW);
    // delayMicroseconds(1);
  }
}

void Scales::PushGain(uint8_t gain, uint8_t clock_pin) {
  // Set the channel and the gain factor for the next reading using the clock
  // pin.
  for (unsigned int i = 0; i < gain; i++) {
    digitalWrite(clock_pin, HIGH);
#if ARCH_ESPRESSIF
    delayMicroseconds(1);
#endif
    digitalWrite(clock_pin, LOW);
#if ARCH_ESPRESSIF
    delayMicroseconds(1);
#endif
  }
}

bool Scales::IsAllReady() {
  for (auto& pair : Scales::scales_by_clock_pin_) {
    for (Scales& scales : pair.second) {
      if (!scales.IsReady()) return false;
    }
  }

  return true;
}

void Scales::SetTareForAll() {
  for (auto& pair : Scales::scales_by_clock_pin_) {
    for (Scales& scales : pair.second) {
      scales.SetTareWithAvg();
    }
  }
}

void Scales::ReadAll() {
  for (auto& pair : scales_by_clock_pin_) {
    BlockInterrupts();
    PullData(pair.first, pair.second);
    // FIXME: ensure that same gain set for all scales objects on the same clock
    // pin
    PushGain(pair.second.front().get().gain_, pair.first);
    UnblockInterrupts();
  }
}

void Scales::BlockInterrupts() {
// Protect the read sequence from system interrupts.  If an interrupt occurs
// during the time the PD_SCK signal is high it will stretch the length of the
// clock pulse. If the total pulse time exceeds 60 uSec this will cause the
// HX711 to enter power down mode during the middle of the read sequence.
// While the device will wake up when PD_SCK goes low again, the reset starts
// a new conversion cycle which forces DOUT high until that cycle is
// completed.
//
// The result is that all subsequent bits read by shiftIn() will read back as
// 1, corrupting the value returned by read().  The ATOMIC_BLOCK macro
// disables interrupts during the sequence and then restores the interrupt
// mask to its previous state after the sequence completes, insuring that the
// entire read-and-gain-set sequence is not interrupted.  The macro has a few
// minor advantages over bracketing the sequence between `noInterrupts()` and
// `interrupts()` calls.
#if HAS_ATOMIC_BLOCK
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
#elif IS_FREE_RTOS
  // Begin of critical section.
  // Critical sections are used as a valid protection method
  // against simultaneous access in vanilla FreeRTOS.
  // Disable the scheduler and call portDISABLE_INTERRUPTS. This prevents
  // context switches and servicing of ISRs during a critical section.
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL(&mux);

#else
  // Disable interrupts.
  noInterrupts();
#endif
}

void Scales::UnblockInterrupts() {
#if IS_FREE_RTOS
    // End of critical section.
    portEXIT_CRITICAL(&mux);

#elif HAS_ATOMIC_BLOCK
}

#else
  // Enable interrupts again.
  interrupts();
#endif
}

uint32_t Scales::Extend24To32(uint32_t val) {
  const int bits = 24;
  uint32_t m = 1u << (bits - 1);
  return (val ^ m) - m;
}
