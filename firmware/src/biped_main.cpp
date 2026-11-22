// =====================================================================
//  Wheel-legged biped: balancing + legs
//
//  STAGE 4 (ENABLE_DRIVE=0, ENABLE_LEGS=0)  balance with the legs held at HEIGHT_DEFAULT
//  STAGE 5 (ENABLE_DRIVE=1, ENABLE_LEGS=0)  + drive with a joystick on your phone
//  STAGE 6 (ENABLE_DRIVE=1, ENABLE_LEGS=1)  + height slider, leaning into turns, roll levelling
//
//  Power: battery ON, power jumper ON, USB unplugged.
//         (To read the serial output, pull the jumper first, then plug in USB.)
//
//  1. Switch on with the robot lying on its back. The legs move to HEIGHT_DEFAULT.
//     Keep it still ~2 s (gyro calibration).
//  2. On your phone join Wi-Fi "WheelBiped" (password in config.h), open http://192.168.4.1
//  3. Stand it up on its wheels and hold it near upright. Within a few degrees of balance
//     the wheel motors start. They stop again if it tips past 40 degrees.
//
//  Controller (same as the balancing bot, plus legs):
//    inner loop (PID on tilt):     wheel = Kp*error + Ki*sum(error) + Kd*tilt_rate
//    outer loop (PI on speed):     target tilt = trim + Kv*(wanted speed - speed) + integral
//    legs (stage 6):               height follows the slider slowly (HEIGHT_RATE);
//                                  lean = Lean * atan(speed * yaw rate / g): outer leg longer;
//                                  level = Level * roll: the low side's leg gets longer.
//  The legs are pure position commands here. The balance loop treats a height change as a
//  disturbance, which it handles fine at 50 mm/s (see ../sim, test "height").
// =====================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "config.h"
#include "imu.h"
#include "motors.h"
#include "legs.h"
#include "web_page.h"

Imu imu;
Motors motors;
Legs legs;
WebServer server(80);
Preferences prefs;

// --- tunable parameters (saved to flash) ---
struct Params {
  float kp = DEFAULT_KP, ki = DEFAULT_KI, kd = DEFAULT_KD;
  float trim = DEFAULT_TRIM, kv = DEFAULT_KV, kvi = DEFAULT_KVI;
  float lean = DEFAULT_LEAN, level = DEFAULT_LEVEL;
} P;

// --- shared between the control task and the web/serial code ---
volatile float cmdSpeed = 0, cmdTurn = 0;   // -1..1 from the joystick
volatile uint32_t lastCmdMs = 0;
volatile bool armed = false;
volatile float tAngle = 0, tTarget = 0, tOut = 0, tSpeed = 0, tLoopUs = 0;
volatile float sonarCm = -1, batteryV = 0;
volatile float cmdHeight = HEIGHT_DEFAULT;          // from the phone slider (stage 6)
volatile float tHeight = HEIGHT_DEFAULT, tDh = 0, tRoll = 0;

void loadParams() {
  prefs.begin("bot", true);
  P.kp = prefs.getFloat("kp", P.kp);
  P.ki = prefs.getFloat("ki", P.ki);
  P.kd = prefs.getFloat("kd", P.kd);
  P.trim = prefs.getFloat("trim", P.trim);
  P.kv = prefs.getFloat("kv", P.kv);
  P.kvi = prefs.getFloat("kvi", P.kvi);
  P.lean = prefs.getFloat("lean", P.lean);
  P.level = prefs.getFloat("level", P.level);
  prefs.end();
  // servo trims from stage 3
  prefs.begin("legs", true);
  const char *n[4] = {"rf", "rr", "lf", "lr"};
  for (int i = 0; i < 4; i++) legs.trim[i] = prefs.getFloat(n[i], legs.trim[i]);
  prefs.end();
}

void saveParams() {
  prefs.begin("bot", false);
  prefs.putFloat("kp", P.kp);
  prefs.putFloat("ki", P.ki);
  prefs.putFloat("kd", P.kd);
  prefs.putFloat("trim", P.trim);
  prefs.putFloat("kv", P.kv);
  prefs.putFloat("kvi", P.kvi);
  prefs.putFloat("lean", P.lean);
  prefs.putFloat("level", P.level);
  prefs.end();
}

static float approach(float x, float target, float maxStep) {
  if (target > x + maxStep) return x + maxStep;
  if (target < x - maxStep) return x - maxStep;
  return target;
}

// =====================================================================
//  Control loop: runs LOOP_HZ times per second on its own task (core 1)
// =====================================================================
void controlTask(void *) {
  const TickType_t period = pdMS_TO_TICKS(1000 / LOOP_HZ);
  const float dt = 1.0f / LOOP_HZ;
  TickType_t lastWake = xTaskGetTickCount();

  float angleI = 0;        // integral of tilt error
  float speedI = 0;        // integral of speed error (degrees of lean)
  float speedEst = 0;      // estimated forward speed, same units as motor output
  float speedTarget = 0, turn = 0;
  float height = HEIGHT_DEFAULT, leanF = 0;
  uint32_t uprightSince = 0;
  long lastL = 0, lastR = 0;

  for (;;) {
    vTaskDelayUntil(&lastWake, period);
    uint32_t t0 = micros();

    float angle = imu.update(dt);
    float rate = imu.pitchRate();

    // ---- legs ----
    float wantHeight = ENABLE_LEGS ? cmdHeight : HEIGHT_DEFAULT;
    height = approach(height, constrain(wantHeight, HEIGHT_MIN, HEIGHT_MAX), HEIGHT_RATE * dt);
    float dh = 0;  // + = right leg shorter, left leg longer
    if (ENABLE_LEGS && armed) {
      // lean into turns: centripetal acceleration = speed * yaw rate
      float v = speedEst * FULL_SPEED_MPS;                                  // m/s (estimate)
      float lean = atan2f(-v * imu.yawRate() * DEG_TO_RAD, 9.81f) * P.lean; // radians, + = lean right
      leanF += (lean - leanF) * (dt / 0.25f);                               // smooth
      float leanDeg = constrain(leanF * RAD_TO_DEG, -MAX_LEAN_DEG, MAX_LEAN_DEG);
      dh = tanf(leanDeg * DEG_TO_RAD) * TRACK_HALF;
      // levelling: robot rolls right (right side low) -> make the right leg longer
      dh -= P.level * tanf(imu.roll() * DEG_TO_RAD) * TRACK_HALF;
      dh = constrain(dh, -30.0f, 30.0f);
    } else {
      leanF = 0;
    }
    legs.setHeights(height - dh, height + dh, WHEEL_Y_OFFSET);
    tHeight = height; tDh = dh; tRoll = imu.roll();

    // ---- commands from the phone (with timeout and smooth ramps) ----
    float wantSpeed = 0, wantTurn = 0;
    if (ENABLE_DRIVE && millis() - lastCmdMs < CMD_TIMEOUT_MS) {
      wantSpeed = cmdSpeed * MAX_DRIVE_SPEED;
      wantTurn = cmdTurn * MAX_TURN;
    }
    if (USE_SONAR && sonarCm > 0 && sonarCm < 20 && wantSpeed > 0) wantSpeed = 0;  // obstacle ahead
    speedTarget = approach(speedTarget, wantSpeed, 0.6f * dt);  // limit acceleration
    turn = approach(turn, wantTurn, 1.5f * dt);

    // ---- arming / fall detection ----
    if (!armed) {
      motors.stop();
      if (fabsf(angle - P.trim) < ARM_ANGLE) {
        if (uprightSince == 0) uprightSince = millis();
        if (millis() - uprightSince > 500) {  // held upright for half a second
          armed = true;
          angleI = speedI = speedEst = speedTarget = 0;
        }
      } else {
        uprightSince = 0;
      }
    } else if (fabsf(angle) > FALL_ANGLE) {
      armed = false;
      uprightSince = 0;
      motors.stop();
    }

    if (armed) {
      // ---- speed estimate ----
#if USE_ENCODERS
      long l = Encoders::leftCount(), r = Encoders::rightCount();
      float rps = ((l - lastL) + (r - lastR)) * 0.5f / ENC_COUNTS_PER_REV / dt;
      lastL = l; lastR = r;
      speedEst += (rps / ENC_FULL_SPEED_RPS - speedEst) * 0.1f;   // light low-pass
#else
      // No encoders: guess speed from what we told the motors (low-pass, ~0.3 s).
      // Rough, but enough to stop slow drifting.
      speedEst += (tOut - speedEst) * (dt / 0.3f);
#endif

      // ---- outer loop: speed -> target lean ----
      float speedErr = speedTarget - speedEst;
      speedI = constrain(speedI + P.kvi * speedErr * dt, -6.0f, 6.0f);
      float target = constrain(P.trim + P.kv * speedErr + speedI, P.trim - 10.0f, P.trim + 10.0f);

      // ---- inner loop: PID on tilt ----
      float err = angle - target;
      angleI += err * dt;
      if (P.ki > 0) angleI = constrain(angleI, -1.0f / P.ki, 1.0f / P.ki);  // anti wind-up
      float out = P.kp * err + P.ki * angleI + P.kd * rate;
      out = constrain(out, -1.0f, 1.0f);

      motors.set(out + turn, out - turn);
      tOut = out;
      tTarget = target;
    } else {
      tOut = 0;
      tTarget = P.trim;
#if USE_ENCODERS
      lastL = Encoders::leftCount();
      lastR = Encoders::rightCount();
#endif
    }
    tAngle = angle;
    tSpeed = speedEst;
    tLoopUs = micros() - t0;
  }
}

// =====================================================================
//  Web server (phone page)
// =====================================================================
void handleState() {
  char buf[600];
  snprintf(buf, sizeof(buf),
           "{\"angle\":%.2f,\"target\":%.2f,\"out\":%.3f,\"speed\":%.3f,\"armed\":%d,"
           "\"batt\":%.2f,\"sonar\":%.0f,\"loop_us\":%.0f,\"drive\":%d,\"legs\":%d,"
           "\"height\":%.1f,\"dh\":%.1f,\"roll\":%.1f,\"hcmd\":%.0f,"
           "\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.5f,\"trim\":%.2f,\"kv\":%.3f,\"kvi\":%.3f,"
           "\"lean\":%.2f,\"level\":%.2f}",
           tAngle, tTarget, tOut, tSpeed, armed ? 1 : 0, batteryV, sonarCm, tLoopUs, ENABLE_DRIVE, ENABLE_LEGS,
           tHeight, tDh, tRoll, cmdHeight,
           P.kp, P.ki, P.kd, P.trim, P.kv, P.kvi, P.lean, P.level);
  server.send(200, "application/json", buf);
}

void handleSet() {
  if (server.hasArg("kp")) P.kp = server.arg("kp").toFloat();
  if (server.hasArg("ki")) P.ki = server.arg("ki").toFloat();
  if (server.hasArg("kd")) P.kd = server.arg("kd").toFloat();
  if (server.hasArg("trim")) P.trim = server.arg("trim").toFloat();
  if (server.hasArg("kv")) P.kv = server.arg("kv").toFloat();
  if (server.hasArg("kvi")) P.kvi = server.arg("kvi").toFloat();
  if (server.hasArg("lean")) P.lean = server.arg("lean").toFloat();
  if (server.hasArg("level")) P.level = server.arg("level").toFloat();
  if (server.hasArg("height")) cmdHeight = constrain(server.arg("height").toFloat(), HEIGHT_MIN, HEIGHT_MAX);
  server.send(200, "text/plain", "ok");
}

void handleCmd() {
  cmdSpeed = constrain(server.arg("v").toFloat(), -1.0f, 1.0f);
  cmdTurn = constrain(server.arg("t").toFloat(), -1.0f, 1.0f);
  lastCmdMs = millis();
  server.send(200, "text/plain", "ok");
}

void setupWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setSleep(false);  // lower latency
  server.on("/", [] { server.send_P(200, "text/html", WEB_PAGE); });
  server.on("/state", handleState);
  server.on("/set", handleSet);
  server.on("/cmd", handleCmd);
  server.on("/save", [] { saveParams(); server.send(200, "text/plain", "saved"); });
  server.on("/defaults", [] { P = Params(); server.send(200, "text/plain", "defaults"); });
  server.begin();
  Serial.printf("Wi-Fi \"%s\" ready -> open http://%s\n", WIFI_SSID, WiFi.softAPIP().toString().c_str());
}

// =====================================================================
//  Serial commands (when USB is connected):  kp 0.08   kd 0.003   trim -1.5   save   show
// =====================================================================
void handleSerial() {
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') { line += c; continue; }
    line.trim();
    if (line.length() == 0) continue;
    int sp = line.indexOf(' ');
    String key = sp > 0 ? line.substring(0, sp) : line;
    float val = sp > 0 ? line.substring(sp + 1).toFloat() : 0;
    if (key == "kp") P.kp = val;
    else if (key == "ki") P.ki = val;
    else if (key == "kd") P.kd = val;
    else if (key == "trim") P.trim = val;
    else if (key == "kv") P.kv = val;
    else if (key == "kvi") P.kvi = val;
    else if (key == "lean") P.lean = val;
    else if (key == "level") P.level = val;
    else if (key == "h") cmdHeight = constrain(val, HEIGHT_MIN, HEIGHT_MAX);
    else if (key == "save") { saveParams(); Serial.println("saved"); }
    Serial.printf("kp=%.4f ki=%.4f kd=%.5f trim=%.2f kv=%.3f kvi=%.3f lean=%.2f level=%.2f h=%.0f\n", P.kp, P.ki,
                  P.kd, P.trim, P.kv, P.kvi, P.lean, P.level, cmdHeight);
    line = "";
  }
}

#if USE_SONAR  // no sonar pins on the biped by default (GPIO10/11 drive servos)
float readSonarCm() {
  digitalWrite(PIN_SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SONAR_TRIG, LOW);
  unsigned long us = pulseIn(PIN_SONAR_ECHO, HIGH, 25000);  // give up after ~4 m
  return us ? us / 58.0f : -1;
}
#endif

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("\n=== Wheel biped (%s%s%s) ===\n", ENABLE_DRIVE ? "drive" : "balance only",
                ENABLE_LEGS ? ", legs" : ", legs fixed", USE_ENCODERS ? ", encoders" : "");
  motors.begin();
  motors.stop();
  legs.begin();
#if USE_ENCODERS
  Encoders::begin();
#endif
#if USE_SONAR
  pinMode(PIN_SONAR_TRIG, OUTPUT);
  pinMode(PIN_SONAR_ECHO, INPUT);
#endif
  loadParams();
  legs.setHeights(HEIGHT_DEFAULT, HEIGHT_DEFAULT, WHEEL_Y_OFFSET);  // stand the legs up (robot lying down)

  if (!imu.begin(PIN_SDA, PIN_SCL)) {
    Serial.println("ERROR: no MPU6050. Check wiring. Halting.");
    while (true) delay(1000);
  }
  Serial.println("Calibrating gyro, keep still...");
  imu.calibrateGyro();

  setupWeb();
  // Control loop on core 1 at high priority; Wi-Fi runs on core 0.
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 10, nullptr, 1);
}

void loop() {
  server.handleClient();
  handleSerial();

  static uint32_t lastSlow = 0;
  if (millis() - lastSlow > 100) {  // 10 Hz housekeeping
    lastSlow = millis();
#if USE_BATTERY_SENSE
    float v = analogReadMilliVolts(PIN_BATTERY) * BATTERY_DIVIDER / 1000.0f;
    batteryV = batteryV < 1 ? v : batteryV + (v - batteryV) * 0.2f;   // smooth
    motors.setBattery(batteryV);
    static uint32_t lastWarn = 0;
    if (batteryV > 3 && batteryV < BATTERY_LOW_V && millis() - lastWarn > 5000) {
      lastWarn = millis();
      Serial.printf("BATTERY LOW: %.2f V, charge it\n", batteryV);
    }
#endif
#if USE_SONAR
    sonarCm = readSonarCm();  // the control loop refuses to drive forward if < 20 cm
#endif
  }
  delay(1);
}
