// Minimal MPU6050 driver + complementary filter (pitch and roll). No external libraries.
#pragma once
#include <Arduino.h>

struct ImuSample {
  float ax, ay, az;     // acceleration in g
  float gx, gy, gz;     // rotation rate in degrees/second (bias removed)
};

class Imu {
 public:
  // Returns false if no MPU6050 answers on the I2C bus.
  bool begin(int sda, int scl);
  // Keep the robot still while this runs (about 1 s). Measures gyro bias.
  void calibrateGyro(int samples = 500);
  bool read(ImuSample &s);

  // Pitch from the accelerometer alone (noisy, but no drift), degrees.
  float accelPitch(const ImuSample &s) const;
  // Pitch rate from the gyro alone (smooth, but drifts), degrees/second.
  float gyroPitchRate(const ImuSample &s) const;

  // Complementary filter: call once per loop with dt in seconds. Returns pitch in degrees.
  float update(float dt);
  float pitch() const { return pitch_; }
  float pitchRate() const { return rate_; }
  // Added for the biped: sideways tilt (+ = leaning onto the right wheel) and turning rate
  // (+ = turning left), both in degrees and degrees/second.
  float roll() const { return roll_; }
  float rollRate() const { return rollRate_; }
  float yawRate() const { return yawRate_; }
  uint8_t whoAmI() const { return who_; }

 private:
  bool writeReg(uint8_t reg, uint8_t val);
  float gbx_ = 0, gby_ = 0, gbz_ = 0;
  float pitch_ = 0, rate_ = 0;
  float roll_ = 0, rollRate_ = 0, yawRate_ = 0;
  bool first_ = true;
  uint8_t who_ = 0;
  uint8_t addr_ = 0x68;
};
