// =====================================================================
//  config.h  -  every pin and every number you are likely to change.
//
//  "Front" of the robot = the side the front servos (and the MPU's +Y) face.
//  Angle > 0  means the robot leans FORWARD.
//  Motor command > 0 means the wheels push the robot FORWARD.
//  Leg "height" = how far the wheel axle is below the servo shafts, in mm (70 .. 145).
// =====================================================================
#pragma once

// ---------------- pins (ESP32-S3 DevKitC-1) ----------------
#define PIN_SDA            8       // MPU6050
#define PIN_SCL            9

#define PIN_LEFT_IN1       4       // MX1508 IN1  (left wheel)
#define PIN_LEFT_IN2       5       // MX1508 IN2
#define PIN_RIGHT_IN1      6       // MX1508 IN3  (right wheel)
#define PIN_RIGHT_IN2      7       // MX1508 IN4

// Servo signal wires (orange). Servo red/brown go to the 6 V UBEC, NOT to the ESP32.
#define PIN_SERVO_RF       10      // right leg, front servo
#define PIN_SERVO_RR       11      // right leg, rear servo
#define PIN_SERVO_LF       12      // left leg, front servo
#define PIN_SERVO_LR       13      // left leg, rear servo

#define PIN_BATTERY        1       // battery + -> 22k -> GPIO1 -> 10k -> GND
#define BATTERY_DIVIDER    3.2f    // (22k + 10k) / 10k. 2S full = 8.4 V -> 2.6 V at the pin
#define BATTERY_LOW_V      6.6f    // 2S: 3.3 V per cell. On 4xAA NiMH (no UBEC) set this to 4.5f

// Encoders (if you ever fit encoder N20s). Same as the balancing bot.
#define PIN_ENC_LEFT_A     15
#define PIN_ENC_LEFT_B     16
#define PIN_ENC_RIGHT_A    17
#define PIN_ENC_RIGHT_B    18

#define USE_SONAR          0
#define USE_BATTERY_SENSE  1

// ---------------- IMU orientation (checked in stage 1) ----------------
// GY-521 flat on the deck pad, printed X arrow along the axle (left-right).
#define PITCH_AXIS         'X'
#define PITCH_SIGN         1       // lean forward -> pitch must go POSITIVE
#define ROLL_SIGN          1       // tip it onto its RIGHT wheel -> roll must go POSITIVE
#define YAW_SIGN           1       // turn it to the LEFT (counter-clockwise from above) -> yaw rate POSITIVE

// ---------------- wheel motors ----------------
#define LEFT_INVERT        false
#define RIGHT_INVERT       true
#define PWM_FREQ_HZ        20000
#define PWM_BITS           10
#define MOTOR_DEADBAND     0.10f   // measure in stage 2
#define MOTOR_BRAKE_MODE   true
// The N20s are 6 V motors and 2S Li-ion is 7.4-8.4 V. The firmware scales the duty so the
// motors never see more than MOTOR_VOLTS on average (needs the battery divider on GPIO1).
#define MOTOR_VOLTS        6.5f
#define MOTOR_VBAT_DEFAULT 7.6f    // used if battery sensing is off / unwired

// ---------------- servos (MG996R) ----------------
#define SERVO_HZ           50
#define SERVO_MIN_US       500     // pulse for 0 degrees
#define SERVO_MAX_US       2500    // pulse for 180 degrees
// Crank angle (from straight down, + = toward the front) that each servo gives at 90 degrees.
// Mount the horns so that at 90 degrees the FRONT crank points ~30 deg forward and the REAR
// crank ~55 deg backward (stage 3 "pose 90" helps). Then fine-tune with SERVO_TRIM below.
#define CRANK_AT_90_FRONT  30.0f
#define CRANK_AT_90_REAR   -55.0f
// +1 / -1: does a bigger servo angle swing the crank FORWARD? The right and left servos face
// opposite ways, so one side is usually -1. Stage 3 tells you.
#define SERVO_DIR_RF       1
#define SERVO_DIR_RR       1
#define SERVO_DIR_LF       -1
#define SERVO_DIR_LR       -1
// Per-servo trim in degrees (horn splines are 14.4 deg apart; this fixes the rest).
// Stage 3: type "trim rf 3.5" etc., then "save". Saved values override these.
#define SERVO_TRIM_RF      0.0f
#define SERVO_TRIM_RR      0.0f
#define SERVO_TRIM_LF      0.0f
#define SERVO_TRIM_LR      0.0f

// ---------------- leg geometry (mm, must match cad/wheel_biped.scad) ----------------
#define LEG_A              13.5f   // half spacing of the two servo shafts
#define LEG_L1             55.0f   // crank
#define LEG_L2             75.0f   // link A, and ER -> knee pin on link B
#define LEG_LB             100.0f  // link B, ER -> wheel axle
#define TRACK_HALF         76.7f   // centre of robot to the middle of a tyre (608 bearing legs)
#define HEIGHT_MIN         72.0f   // below this the rear crank hits the deck
#define HEIGHT_MAX         145.0f  // near full stretch the servos lose leverage
#define HEIGHT_DEFAULT     110.0f  // stage 4/5 fixed height (robot ~22 cm tall)
#define HEIGHT_RATE        50.0f   // mm/s: how fast it crouches / stands
#define WHEEL_Y_OFFSET     0.0f    // mm: put the wheels this far forward of the shafts' midpoint

// ---------------- balancing loop ----------------
#define LOOP_HZ            200
#define COMP_ALPHA         0.98f
#define FALL_ANGLE         40.0f
#define ARM_ANGLE          4.0f

// Starting gains from the MuJoCo model in ../sim (survive pushes, 40% weaker/stronger motors,
// a battery 4 mm off-centre, extra delay, and leg heights 75 to 140 mm). Tune on the phone.
#define DEFAULT_KP         0.12f
#define DEFAULT_KI         3.5f
#define DEFAULT_KD         0.004f
#define DEFAULT_TRIM       0.0f
#define DEFAULT_KV         8.0f
#define DEFAULT_KVI        2.0f
#define DEFAULT_LEAN       0.5f    // 0 = no leaning into turns, 1 = full "physics" lean
#define DEFAULT_LEVEL      0.0f    // roll levelling on slopes (0 = off; try 0.5 .. 1.5 in stage 6)
#define MAX_LEAN_DEG       6.0f

// ---------------- driving ----------------
#define MAX_DRIVE_SPEED    0.40f
#define MAX_TURN           0.20f
#define CMD_TIMEOUT_MS     500
#define FULL_SPEED_MPS     1.04f   // 300 rpm x 67 mm wheel. Used for the lean estimate.

// ---------------- encoders ----------------
#define ENC_COUNTS_PER_REV 1400.0f
#define ENC_FULL_SPEED_RPS 5.0f
#define ENC_LEFT_FLIP      false
#define ENC_RIGHT_FLIP     false

// ---------------- Wi-Fi ----------------
#define WIFI_SSID          "WheelBiped"
#define WIFI_PASSWORD      "biped1234"

#ifndef ENABLE_DRIVE
#define ENABLE_DRIVE 1
#endif
#ifndef ENABLE_LEGS
#define ENABLE_LEGS 0          // 0 = legs held at HEIGHT_DEFAULT (stages 4-5), 1 = height + lean (stage 6)
#endif
#ifndef USE_ENCODERS
#define USE_ENCODERS 0
#endif
