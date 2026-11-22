// =====================================================================
//  STAGE 2  -  spin the motors, check directions, measure the deadband
//
//  Wiring needed: MX1508 + motors + battery (see the wiring diagram).
//  Power: USB cable for the ESP32 AND the battery switched on for the motors.
//         The red "power jumper" (buck 5V -> ESP32 5V) must be UNPLUGGED
//         whenever USB is plugged in.
//  Lay the robot on its back (servos unpowered is fine) so the wheels spin freely.
//
//  Type a key in the Serial Monitor and press Enter:
//    f  both wheels "forward" for 1 s   -> robot would roll toward its FRONT
//    b  both wheels backward for 1 s
//    l  left wheel only forward          (left = your left when you stand behind the robot,
//    r  right wheel only forward          looking at its front)
//    d  deadband ramp: slowly raises the duty, watch when each wheel starts turning
//    s  stop
//
//  Fix directions in include/config.h: LEFT_INVERT / RIGHT_INVERT
//  (or swap the two wires of that motor on the MX1508).
//  Put the duty where the wheels start turning into MOTOR_DEADBAND.
// =====================================================================
#include <Arduino.h>
#include "config.h"
#include "motors.h"

Motors motors;

void help() {
  Serial.println("\nCommands: f=forward  b=backward  l=left fwd  r=right fwd  d=deadband ramp  s=stop");
}

void runFor(float l, float r, uint32_t ms) {
  motors.set(l, r);
  delay(ms);
  motors.stop();
}

void deadbandRamp() {
  Serial.println("Ramping raw duty from 0 to 0.40 (no deadband compensation). Note when each wheel starts.");
  Serial.println("Press any key to stop early.");
  while (Serial.available()) Serial.read();
  for (int i = 0; i <= 40; i++) {
    float duty = i / 100.0f;
    motors.setRaw(duty, duty);
    Serial.printf("duty = %.2f\n", duty);
    delay(400);
    if (Serial.available()) break;
  }
  motors.stop();
  Serial.println("Done. Put the first duty where BOTH wheels turned into MOTOR_DEADBAND (config.h).");
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  motors.begin();
  Serial.println("\n=== Stage 2: motor test ===");
  Serial.println("Wheels in the air! Battery ON, power jumper OFF while USB is plugged in.");
  help();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'f': Serial.println("forward");  runFor(0.5f, 0.5f, 1000); break;
    case 'b': Serial.println("backward"); runFor(-0.5f, -0.5f, 1000); break;
    case 'l': Serial.println("left wheel forward");  runFor(0.5f, 0, 1000); break;
    case 'r': Serial.println("right wheel forward"); runFor(0, 0.5f, 1000); break;
    case 'd': deadbandRamp(); break;
    case 's': motors.stop(); Serial.println("stop"); break;
    case '\n': case '\r': return;
    default: help();
  }
}
