#include "motors.h"
#include "config.h"

static const int PWM_MAX = (1 << PWM_BITS) - 1;

void Motors::begin() {
  for (int pin : {PIN_LEFT_IN1, PIN_LEFT_IN2, PIN_RIGHT_IN1, PIN_RIGHT_IN2}) {
    ledcAttach(pin, PWM_FREQ_HZ, PWM_BITS);  // Arduino core 3.x API
    ledcWrite(pin, 0);
  }
}

// MX1508 truth table:  IN1  IN2
//                       1    0   forward
//                       0    1   reverse
//                       1    1   brake
//                       0    0   coast
void Motors::drive(int in1, int in2, float duty) {
  duty = constrain(duty, -1.0f, 1.0f);
  duty *= fminf(1.0f, MOTOR_VOLTS / vbat_);   // never more than ~6 V average on the 6 V motors
  int d = (int)(fabsf(duty) * PWM_MAX + 0.5f);
  if (MOTOR_BRAKE_MODE) {
    // slow decay: one input held high, the other pulses. More linear speed vs duty.
    if (duty >= 0) { ledcWrite(in1, PWM_MAX); ledcWrite(in2, PWM_MAX - d); }
    else           { ledcWrite(in2, PWM_MAX); ledcWrite(in1, PWM_MAX - d); }
  } else {
    // fast decay: one input pulses, the other held low.
    if (duty >= 0) { ledcWrite(in1, d); ledcWrite(in2, 0); }
    else           { ledcWrite(in2, d); ledcWrite(in1, 0); }
  }
}

static float compensate(float u) {
  // A small command would not overcome gearbox friction, so jump over the deadband.
  if (fabsf(u) < 0.005f) return 0.0f;
  float mag = MOTOR_DEADBAND + fabsf(u) * (1.0f - MOTOR_DEADBAND);
  return copysignf(fminf(mag, 1.0f), u);
}

void Motors::setRaw(float left, float right) {
  lastL_ = left; lastR_ = right;
  drive(PIN_LEFT_IN1, PIN_LEFT_IN2, LEFT_INVERT ? -left : left);
  drive(PIN_RIGHT_IN1, PIN_RIGHT_IN2, RIGHT_INVERT ? -right : right);
}

void Motors::set(float left, float right) {
  left = constrain(left, -1.0f, 1.0f);
  right = constrain(right, -1.0f, 1.0f);
  setRaw(compensate(left), compensate(right));
  lastL_ = left; lastR_ = right;
}

void Motors::stop() {
  for (int pin : {PIN_LEFT_IN1, PIN_LEFT_IN2, PIN_RIGHT_IN1, PIN_RIGHT_IN2}) ledcWrite(pin, 0);
  lastL_ = lastR_ = 0;
}

// ---------------------------------------------------------------------
//  Encoders: count both edges of channel A, read B for direction.
// ---------------------------------------------------------------------
namespace Encoders {
static volatile long cl = 0, cr = 0;

static void IRAM_ATTR isrLeft() {
  bool a = digitalRead(PIN_ENC_LEFT_A), b = digitalRead(PIN_ENC_LEFT_B);
  cl += (a == b) ? 1 : -1;
}
static void IRAM_ATTR isrRight() {
  bool a = digitalRead(PIN_ENC_RIGHT_A), b = digitalRead(PIN_ENC_RIGHT_B);
  cr += (a == b) ? 1 : -1;
}

void begin() {
  for (int pin : {PIN_ENC_LEFT_A, PIN_ENC_LEFT_B, PIN_ENC_RIGHT_A, PIN_ENC_RIGHT_B}) pinMode(pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT_A), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT_A), isrRight, CHANGE);
}
// Flip flags in config.h make "forward" count up on both sides.
long leftCount() { return ENC_LEFT_FLIP ? -cl : cl; }
long rightCount() { return ENC_RIGHT_FLIP ? -cr : cr; }
}  // namespace Encoders
