#include "imu.h"
#include <Wire.h>
#include "config.h"

// MPU6050 registers
static const uint8_t REG_SMPLRT_DIV = 0x19;
static const uint8_t REG_CONFIG = 0x1A;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_WHO_AM_I = 0x75;

static const float ACC_LSB_PER_G = 8192.0f;   // +-4 g range
static const float GYRO_LSB_PER_DPS = 65.5f;  // +-500 deg/s range

bool Imu::writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr_);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool Imu::begin(int sda, int scl) {
  Wire.begin(sda, scl, 400000);
  // AD0 pin low -> 0x68, high -> 0x69. Try both.
  for (uint8_t a : {0x68, 0x69}) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { addr_ = a; break; }
    if (a == 0x69) return false;
  }
  Wire.beginTransmission(addr_);
  Wire.write(REG_WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(addr_, (uint8_t)1);
  who_ = Wire.available() ? Wire.read() : 0;  // 0x68 for a genuine MPU6050; clones may differ

  bool ok = writeReg(REG_PWR_MGMT_1, 0x01);   // wake up, clock = gyro X PLL
  delay(50);
  ok &= writeReg(REG_CONFIG, 0x03);           // digital low-pass ~44 Hz: cuts motor vibration
  ok &= writeReg(REG_SMPLRT_DIV, 0x04);       // 1 kHz / (1+4) = 200 Hz internal rate
  ok &= writeReg(REG_GYRO_CONFIG, 0x08);      // +-500 deg/s
  ok &= writeReg(REG_ACCEL_CONFIG, 0x08);     // +-4 g
  return ok;
}

bool Imu::read(ImuSample &s) {
  Wire.beginTransmission(addr_);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr_, (uint8_t)14) != 14) return false;
  int16_t raw[7];
  for (int i = 0; i < 7; i++) raw[i] = (int16_t)((Wire.read() << 8) | Wire.read());
  s.ax = raw[0] / ACC_LSB_PER_G;
  s.ay = raw[1] / ACC_LSB_PER_G;
  s.az = raw[2] / ACC_LSB_PER_G;
  // raw[3] is temperature
  s.gx = raw[4] / GYRO_LSB_PER_DPS - gbx_;
  s.gy = raw[5] / GYRO_LSB_PER_DPS - gby_;
  s.gz = raw[6] / GYRO_LSB_PER_DPS - gbz_;
  return true;
}

void Imu::calibrateGyro(int samples) {
  gbx_ = gby_ = gbz_ = 0;
  double sx = 0, sy = 0, sz = 0;
  int n = 0;
  ImuSample s;
  for (int i = 0; i < samples; i++) {
    if (read(s)) { sx += s.gx; sy += s.gy; sz += s.gz; n++; }
    delay(2);
  }
  if (n > 0) { gbx_ = sx / n; gby_ = sy / n; gbz_ = sz / n; }
  first_ = true;
}

float Imu::accelPitch(const ImuSample &s) const {
  float a = (PITCH_AXIS == 'X') ? atan2f(s.ay, s.az) : atan2f(-s.ax, s.az);
  return PITCH_SIGN * a * RAD_TO_DEG;
}

float Imu::gyroPitchRate(const ImuSample &s) const {
  return PITCH_SIGN * ((PITCH_AXIS == 'X') ? s.gx : s.gy);
}

float Imu::update(float dt) {
  ImuSample s;
  if (!read(s)) return pitch_;  // keep last value if the bus hiccups
  float acc = accelPitch(s);
  rate_ = gyroPitchRate(s);
  // roll: rotation about the forward axis (the other horizontal axis of the board)
  float accRoll = ROLL_SIGN * ((PITCH_AXIS == 'X') ? atan2f(s.ax, s.az) : atan2f(s.ay, s.az)) * RAD_TO_DEG;
  rollRate_ = ROLL_SIGN * ((PITCH_AXIS == 'X') ? -s.gy : s.gx);
  yawRate_ = YAW_SIGN * s.gz;
  if (first_) { pitch_ = acc; roll_ = accRoll; first_ = false; }
  // Gyro is trusted short-term, accelerometer slowly pulls it back (no drift).
  pitch_ = COMP_ALPHA * (pitch_ + rate_ * dt) + (1.0f - COMP_ALPHA) * acc;
  roll_ = COMP_ALPHA * (roll_ + rollRate_ * dt) + (1.0f - COMP_ALPHA) * accRoll;
  return pitch_;
}
