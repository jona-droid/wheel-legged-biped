// The two five-bar legs: inverse kinematics + the four MG996R servos.
#pragma once
#include <Arduino.h>

enum ServoId { RF = 0, RR = 1, LF = 2, LR = 3 };

struct LegAngles {
  float front, rear;   // crank angles in degrees, from straight down, + = toward the front
  bool ok;
};

// Wheel axle at (y forward, depth below the shafts), mm  ->  crank angles.
LegAngles legIK(float y, float depth);

class Legs {
 public:
  void begin();
  // Wheel depth (mm) for each leg. Clamped to HEIGHT_MIN..HEIGHT_MAX.
  void setHeights(float right, float left, float y = 0.0f);
  // Raw servo angle (0..180) for calibration. Ignores trim and direction.
  void setServoDeg(ServoId id, float deg);
  void setCrank(ServoId id, float crankDeg);   // crank angle -> servo angle (dir, trim applied)
  void relax();                                // stop sending pulses (servos go limp)
  float trim[4] = {0, 0, 0, 0};
  float lastServoDeg[4] = {90, 90, 90, 90};
  float rightHeight = 0, leftHeight = 0;
 private:
  void writeUs(ServoId id, float us);
};
