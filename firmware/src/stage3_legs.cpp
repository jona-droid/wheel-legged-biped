// =====================================================================
//  STAGE 3  -  servos, horn mounting and leg calibration
//
//  Wiring needed: the 4 servo signal wires (GPIO10-13), servo power from the 6 V UBEC,
//  and the UBEC's GND connected to the ESP32 GND.
//  Power: USB for the ESP32 + battery ON for the servos. Power jumper OFF (USB is plugged in).
//  Hold the robot in the air or put the chassis on a box: the legs will move.
//
//  Serial Monitor commands (type, then Enter):
//    pose90             all servos to 90 degrees -> attach the horns + cranks now (see README)
//    h 110              both legs to 110 mm (wheel axle below the servo shafts)
//    h 120 100          right leg 120 mm, left leg 100 mm
//    s rf 95            one servo (rf rr lf lr) to a raw angle, for testing directions
//    trim rf 2.5        add 2.5 degrees to one servo's angle (fixes the horn spline steps)
//    sweep              crouch and stand 3 times (watch for anything rubbing)
//    relax              servos limp
//    save               store the trims in flash (stages 4-6 load them)
//    show               print the trims and current servo angles
//
//  Calibrated when: at "h 110" both wheels are directly below the middle of the chassis,
//  at the same height, and the legs look like the side view in the README.
// =====================================================================
#include <Arduino.h>
#include <Preferences.h>
#include "config.h"
#include "legs.h"

Legs legs;
Preferences prefs;
float hR = HEIGHT_DEFAULT, hL = HEIGHT_DEFAULT;

const char *NAMES[4] = {"rf", "rr", "lf", "lr"};

int servoIndex(const String &n) {
  for (int i = 0; i < 4; i++) if (n == NAMES[i]) return i;
  return -1;
}

void loadTrims() {
  prefs.begin("legs", true);
  for (int i = 0; i < 4; i++) legs.trim[i] = prefs.getFloat(NAMES[i], legs.trim[i]);
  prefs.end();
}

void saveTrims() {
  prefs.begin("legs", false);
  for (int i = 0; i < 4; i++) prefs.putFloat(NAMES[i], legs.trim[i]);
  prefs.end();
}

void show() {
  Serial.printf("trim  rf=%.1f rr=%.1f lf=%.1f lr=%.1f\n", legs.trim[0], legs.trim[1], legs.trim[2], legs.trim[3]);
  Serial.printf("servo rf=%.1f rr=%.1f lf=%.1f lr=%.1f  (height R %.0f, L %.0f mm)\n", legs.lastServoDeg[0],
                legs.lastServoDeg[1], legs.lastServoDeg[2], legs.lastServoDeg[3], hR, hL);
  LegAngles a = legIK(0, hR);
  Serial.printf("cranks at %.0f mm: front %.1f deg, rear %.1f deg (from straight down, + = forward)\n", hR, a.front, a.rear);
}

void moveTo(float r, float l) {
  // slow move so nothing slams
  for (int i = 1; i <= 50; i++) {
    legs.setHeights(hR + (r - hR) * i / 50, hL + (l - hL) * i / 50);
    delay(20);
  }
  hR = constrain(r, HEIGHT_MIN, HEIGHT_MAX);
  hL = constrain(l, HEIGHT_MIN, HEIGHT_MAX);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  legs.begin();
  loadTrims();
  Serial.println("\n=== Stage 3: legs ===");
  Serial.println("Commands: pose90 | h 110 | h 120 100 | s rf 95 | trim rf 2.5 | sweep | relax | save | show");
  Serial.println("Servos start limp. Type pose90 (before the horns are on) or h 110 (after).");
  legs.relax();
}

void loop() {
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') { line += c; continue; }
    line.trim();
    if (!line.length()) continue;
    String w[3];
    int n = 0, start = 0;
    for (int i = 0; i <= (int)line.length() && n < 3; i++) {
      if (i == (int)line.length() || line[i] == ' ') {
        if (i > start) w[n++] = line.substring(start, i);
        start = i + 1;
      }
    }
    if (w[0] == "pose90") {
      for (int i = 0; i < 4; i++) legs.setServoDeg((ServoId)i, 90);
      Serial.println("All servos at 90. Front cranks ~30 deg forward of straight down, rear cranks ~55 deg back.");
    } else if (w[0] == "h" && n >= 2) {
      float r = w[1].toFloat(), l = n == 3 ? w[2].toFloat() : r;
      moveTo(r, l);
      show();
    } else if (w[0] == "s" && n == 3 && servoIndex(w[1]) >= 0) {
      legs.setServoDeg((ServoId)servoIndex(w[1]), w[2].toFloat());
      show();
    } else if (w[0] == "trim" && n == 3 && servoIndex(w[1]) >= 0) {
      legs.trim[servoIndex(w[1])] = w[2].toFloat();
      legs.setHeights(hR, hL);
      show();
    } else if (w[0] == "sweep") {
      for (int k = 0; k < 3; k++) { moveTo(HEIGHT_MIN, HEIGHT_MIN); moveTo(HEIGHT_MAX, HEIGHT_MAX); }
      moveTo(HEIGHT_DEFAULT, HEIGHT_DEFAULT);
      show();
    } else if (w[0] == "relax") {
      legs.relax();
      Serial.println("servos limp");
    } else if (w[0] == "save") {
      saveTrims();
      Serial.println("saved");
    } else {
      show();
    }
    line = "";
  }
}
