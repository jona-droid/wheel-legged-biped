// =====================================================================
//  STAGE 1  -  read the IMU and check the tilt angle
//
//  Wiring needed: GY-521 only (VCC->3V3, GND->GND, SDA->GPIO8, SCL->GPIO9).
//  Power: USB cable only. Battery not needed.
//
//  What to check (open the Serial Monitor at 115200, or the Serial Plotter):
//   1. "MPU6050 found" appears. If not: check the 4 wires.
//   2. Hold the robot upright and still: angle close to 0 (a few degrees is fine).
//   3. Lean it FORWARD (front = the front servos' side): angle must go POSITIVE.
//      If it goes negative -> set PITCH_SIGN to -1 in include/config.h.
//      If it barely changes when you lean, but changes when you roll it sideways
//      -> set PITCH_AXIS to 'Y'.
//   4. Gently shake it: "acc" jumps around, "angle" stays smooth. That's the filter working.
//   5. Tip it onto its RIGHT wheel: roll must go POSITIVE (else ROLL_SIGN -1).
//   6. Turn it LEFT on the spot (counter-clockwise seen from above): yaw must go POSITIVE
//      while it turns (else YAW_SIGN -1). Roll and yaw are only used by stage 6 (legs).
// =====================================================================
#include <Arduino.h>
#include "config.h"
#include "imu.h"

Imu imu;

void setup() {
  Serial.begin(115200);
  delay(1500);  // give the USB serial port time to appear
  Serial.println("\n=== Stage 1: IMU test ===");
  if (!imu.begin(PIN_SDA, PIN_SCL)) {
    Serial.println("ERROR: no MPU6050 on I2C. Check VCC/GND/SDA(8)/SCL(9). Halting.");
    while (true) delay(1000);
  }
  Serial.printf("MPU6050 found, WHO_AM_I = 0x%02X (0x68 = genuine; other values are clones and usually fine)\n",
                imu.whoAmI());
  Serial.println("Keep the robot still: calibrating gyro...");
  imu.calibrateGyro();
  Serial.println("Done. Columns: angle (filtered), acc (accelerometer only), rate (deg/s)");
}

void loop() {
  static uint32_t last = micros();
  static uint32_t lastPrint = 0;
  const uint32_t period = 1000000 / LOOP_HZ;

  uint32_t now = micros();
  if (now - last < period) return;
  float dt = (now - last) * 1e-6f;
  last = now;

  imu.update(dt);

  if (millis() - lastPrint >= 50) {  // 20 lines per second
    lastPrint = millis();
    ImuSample s;
    imu.read(s);
    // "name:value" format works in both the Serial Monitor and the Serial Plotter
    Serial.printf("angle:%.2f acc:%.2f rate:%.1f roll:%.2f yaw:%.1f\n", imu.pitch(), imu.accelPitch(s),
                  imu.pitchRate(), imu.roll(), imu.yawRate());
  }
}
