// AS5048A angle sensor for SimpleFOC, replacing MagneticSensorSPI.
//
// MagneticSensorSPI reads the angle with two SPI frames plus a fixed 50 us
// delay on ESP32 and accepts corrupted responses unchecked, and SimpleFOC's
// base-class velocity is a raw angle difference per loop iteration (100 us
// minimum): on a 14-bit encoder at 33 rpm that is about +-2 rad/s of
// quantisation noise on a 3.5 rad/s target. This class:
//  - reads the angle with one pipelined 16-bit frame per loop. The AS5048A
//    answers each frame with the register requested by the previous frame,
//    and the command is always "read angle", so every frame after the first
//    returns a fresh angle;
//  - checks the even-parity bit 15 and the error flag bit 14 of every
//    response, clears a latched error flag through ERRFL (0x0001), and holds
//    the last good angle for a bad frame;
//  - rejects physically impossible jumps (a corrupted frame that still
//    passed parity) unless the sensor keeps reporting them;
//  - estimates velocity by least squares over an 8 ms window of every
//    loop sample (~32 at 4 kHz): about 0.3 % quantisation noise with 4 ms
//    group delay, instead of tens of percent per iteration. SimpleFOC's
//    LPF_velocity still applies on top.
// Frame format (AS5048A/B datasheet, SPI interface): read command =
// PAR(15) R/W(14)=1 ADDR(13:0); response = PAR(15) EF(14) DATA(13:0);
// SPI mode 1, up to 10 MHz, CSn high >= 350 ns between frames.
#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>

class PlatterSensor : public Sensor {
 public:
  PlatterSensor(SPIClass& spi, int cs, uint32_t clockHz)
      : spi_(spi), cs_(cs), clockHz_(clockHz) {}

  // Call after SPI.begin(). Primes the read pipeline and obtains a first
  // valid angle before SimpleFOC's smooth-start initialisation.
  void init() {
    pinMode(cs_, OUTPUT);
    digitalWrite(cs_, HIGH);
    primed_ = false;
    lastGood_ = -1.0f;
    for (int i = 0; i < 8 && lastGood_ < 0.0f; ++i) {
      getSensorAngle();
      delayMicroseconds(100);
    }
    Sensor::init();
  }

  // Two-frame read of any register, for diagnostics (magnitude, ERRFL,
  // DIAAGC). Disturbs the angle pipeline; the next angle read re-primes it.
  bool readRegister(uint16_t address, uint16_t& value, uint32_t clockHz = 0) {
    const uint32_t hz = clockHz ? clockHz : clockHz_;
    frame(withEvenParity(0x4000u | (address & 0x3FFFu)), hz);
    const uint16_t response = frame(0x0000u, hz);
    primed_ = false;
    value = response & 0x3FFFu;
    return replyValid(response);
  }

  // micros() at which the angle reported by getAngle() was read.
  uint32_t lastReadUs() const { return uint32_t(angle_prev_ts); }
  uint32_t frameErrors() const { return frameErrors_; }
  uint32_t rejectedJumps() const { return rejectedJumps_; }
  // False after 16 consecutive bad frames: the encoder is not usable.
  bool healthy() const { return consecutiveErrors_ < 16; }

  // Least-squares velocity over the last 8 ms of loop samples.
  float getVelocity() override {
    const uint32_t t = uint32_t(angle_prev_ts);
    const unsigned last = (head_ + N - 1) % N;
    if (count_ == 0 || t != time_[last]) {
      rot_[head_] = full_rotations;
      ang_[head_] = angle_prev;
      time_[head_] = t;
      head_ = (head_ + 1) % N;
      if (count_ < N) ++count_;
      velocity_ = fit(t, full_rotations, angle_prev);
    }
    return velocity_;
  }

 protected:
  float getSensorAngle() override {
    if (!primed_) {
      frame(READ_ANGLE, clockHz_);
      primed_ = true;
    }
    const uint16_t response = frame(READ_ANGLE, clockHz_);
    if (!replyValid(response)) {
      ++frameErrors_;
      ++consecutiveErrors_;
      if (response & 0x4000u) frame(READ_ERRFL, clockHz_);  // clear latched EF
      primed_ = false;
      return lastGood_;  // -1 before the first good read: SimpleFOC skips it
    }
    const float angle = float(response & 0x3FFFu) * (_2PI / 16384.0f);
    const uint32_t now = micros();
    if (lastGood_ >= 0.0f) {
      float d = fabsf(angle - lastGood_);
      if (d > _PI) d = _2PI - d;
      // A quarter turn in under 2 ms is 7500 rpm: a corrupted frame, not
      // platter motion, unless the sensor keeps saying so.
      if (d > _PI_2 && uint32_t(now - lastGoodUs_) < 2000 && ++suspect_ < 4) {
        ++rejectedJumps_;
        return lastGood_;
      }
    }
    suspect_ = 0;
    consecutiveErrors_ = 0;
    lastGood_ = angle;
    lastGoodUs_ = now;
    return angle;
  }

 private:
  static constexpr unsigned N = 48;  // 8 ms at up to 6 kHz
  static constexpr uint16_t READ_ANGLE = 0xFFFFu;  // parity | read | 0x3FFF
  static constexpr uint16_t READ_ERRFL = 0x4001u;  // read | 0x0001, even parity

  static uint16_t withEvenParity(uint16_t value) {
    value &= 0x7FFFu;
    if (__builtin_parity((unsigned)value)) value |= 0x8000u;
    return value;
  }
  static bool replyValid(uint16_t response) {
    return __builtin_parity((unsigned)response) == 0 && (response & 0x4000u) == 0;
  }

  uint16_t frame(uint16_t command, uint32_t hz) {
    spi_.beginTransaction(SPISettings(hz, MSBFIRST, SPI_MODE1));
    digitalWrite(cs_, LOW);
    const uint16_t response = spi_.transfer16(command);
    digitalWrite(cs_, HIGH);
    spi_.endTransaction();
    delayMicroseconds(1);  // CSn high time between frames
    return response;
  }

  float fit(uint32_t t, int32_t rotNow, float angNow) const {
    float sx = 0, sy = 0, sxx = 0, sxy = 0;
    int n = 0;
    uint32_t span = 0;
    for (unsigned i = 0; i < count_; ++i) {
      const uint32_t age = t - time_[i];
      if (age > 8000) continue;
      const float x = -float(age) * 1e-6f;
      const float y = float(rot_[i] - rotNow) * _2PI + (ang_[i] - angNow);
      sx += x; sy += y; sxx += x * x; sxy += x * y; ++n;
      if (age > span) span = age;
    }
    if (n < 3 || span < 4000) return 0.0f;
    const float denom = n * sxx - sx * sx;
    if (denom < 1e-12f) return velocity_;
    return (n * sxy - sx * sy) / denom;
  }

  SPIClass& spi_;
  int cs_;
  uint32_t clockHz_;
  bool primed_ = false;
  float lastGood_ = -1.0f;
  uint32_t lastGoodUs_ = 0;
  unsigned suspect_ = 0;
  uint32_t frameErrors_ = 0, rejectedJumps_ = 0, consecutiveErrors_ = 0;
  int32_t rot_[N] = {};
  float ang_[N] = {};
  uint32_t time_[N] = {};
  unsigned head_ = 0, count_ = 0;
  float velocity_ = 0.0f;
};
