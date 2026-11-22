// Two DC motors on an MX1508 dual H-bridge.
#pragma once
#include <Arduino.h>

class Motors {
 public:
  void begin();
  // left, right in -1..1 (1 = full speed forward). Deadband compensation is applied.
  void set(float left, float right);
  // Raw duty in -1..1 with NO deadband compensation (used by the stage 2 ramp test).
  void setRaw(float left, float right);
  void stop();      // both off (coast)
  // Battery voltage, so the duty can be scaled down to MOTOR_VOLTS (6 V motors on 2S).
  void setBattery(float volts) { if (volts > 5.0f) vbat_ = volts; }
  float lastLeft() const { return lastL_; }
  float lastRight() const { return lastR_; }

 private:
  void drive(int in1, int in2, float duty);
  float lastL_ = 0, lastR_ = 0;
  float vbat_ = 7.6f;
};

// Encoders for N20 motors that have them (stage 5). Safe to call even if not wired.
namespace Encoders {
void begin();
long leftCount();
long rightCount();
}
