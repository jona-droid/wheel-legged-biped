#include "legs.h"
#include "config.h"

static const int PINS[4] = {PIN_SERVO_RF, PIN_SERVO_RR, PIN_SERVO_LF, PIN_SERVO_LR};
static const int DIRS[4] = {SERVO_DIR_RF, SERVO_DIR_RR, SERVO_DIR_LF, SERVO_DIR_LR};
static const float AT90[4] = {CRANK_AT_90_FRONT, CRANK_AT_90_REAR, CRANK_AT_90_FRONT, CRANK_AT_90_REAR};
static const int SERVO_BITS = 14;  // 50 Hz, 14 bit -> 1.22 us steps

// ---------------------------------------------------------------------
//  Inverse kinematics. Same maths as sim/leg_kinematics.py:
//    rear elbow ER: |ER - SR| = L1 and |W - ER| = LB   (pick the more backward solution)
//    knee pin   P : on link B, L2 from ER
//    front elbow EF: |EF - SF| = L1 and |P - EF| = L2  (pick the more forward solution)
// ---------------------------------------------------------------------
static bool circles(float x0, float y0, float r0, float x1, float y1, float r1,
                    float &ax, float &ay, float &bx, float &by) {
  float dx = x1 - x0, dy = y1 - y0, d = sqrtf(dx * dx + dy * dy);
  if (d > r0 + r1 || d < fabsf(r0 - r1) || d < 1e-6f) return false;
  float a = (r0 * r0 - r1 * r1 + d * d) / (2 * d);
  float h = sqrtf(fmaxf(r0 * r0 - a * a, 0));
  float mx = x0 + a * dx / d, my = y0 + a * dy / d;
  ax = mx - h * dy / d; ay = my + h * dx / d;
  bx = mx + h * dy / d; by = my - h * dx / d;
  return true;
}

LegAngles legIK(float y, float depth) {
  LegAngles r{0, 0, false};
  float wy = y, wz = -depth;
  float ax, az, bx, bz;
  // rear elbow
  if (!circles(-LEG_A, 0, LEG_L1, wy, wz, LEG_LB, ax, az, bx, bz)) return r;
  float ery = ax < bx ? ax : bx, erz = ax < bx ? az : bz;
  float py = ery + (wy - ery) * (LEG_L2 / LEG_LB), pz = erz + (wz - erz) * (LEG_L2 / LEG_LB);
  // front elbow
  if (!circles(LEG_A, 0, LEG_L1, py, pz, LEG_L2, ax, az, bx, bz)) return r;
  float efy = ax > bx ? ax : bx, efz = ax > bx ? az : bz;
  r.rear = atan2f(ery + LEG_A, -erz) * RAD_TO_DEG;
  r.front = atan2f(efy - LEG_A, -efz) * RAD_TO_DEG;
  r.ok = true;
  return r;
}

void Legs::begin() {
  for (int i = 0; i < 4; i++) ledcAttach(PINS[i], SERVO_HZ, SERVO_BITS);
  trim[RF] = SERVO_TRIM_RF; trim[RR] = SERVO_TRIM_RR;
  trim[LF] = SERVO_TRIM_LF; trim[LR] = SERVO_TRIM_LR;
}

void Legs::writeUs(ServoId id, float us) {
  const float periodUs = 1e6f / SERVO_HZ;
  ledcWrite(PINS[id], (uint32_t)(us / periodUs * ((1 << SERVO_BITS) - 1)));
}

void Legs::setServoDeg(ServoId id, float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  lastServoDeg[id] = deg;
  writeUs(id, SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * deg / 180.0f);
}

void Legs::setCrank(ServoId id, float crankDeg) {
  setServoDeg(id, 90.0f + DIRS[id] * (crankDeg - AT90[id]) + trim[id]);
}

void Legs::setHeights(float right, float left, float y) {
  right = constrain(right, HEIGHT_MIN, HEIGHT_MAX);
  left = constrain(left, HEIGHT_MIN, HEIGHT_MAX);
  LegAngles r = legIK(y, right), l = legIK(y, left);
  if (r.ok) { setCrank(RF, r.front); setCrank(RR, r.rear); rightHeight = right; }
  if (l.ok) { setCrank(LF, l.front); setCrank(LR, l.rear); leftHeight = left; }
}

void Legs::relax() {
  for (int i = 0; i < 4; i++) ledcWrite(PINS[i], 0);
}
