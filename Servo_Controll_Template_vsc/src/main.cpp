/*
  PS4 Controller → Servo Arm Control
  Hardware:
    - Arduino Nano ESP32 / ESP32 Mini
    - PS4 DualShock (via Bluepad32 over Bluetooth)
    - PCA9685 PWM driver
    - RDS3235 / MG996R servos

  Stick mapping:
    Left  Y  →  shoulder pitch  (Ch 0)
    Left  X  →  shoulder yaw    (Ch 4)
    Right Y  →  elbow           (Ch 😎
    Right X  →  wrist turn      (Ch 12)
    L2/R2    →  speed scale (slow ↔ fast)

  D-pad / buttons are free for you to extend.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Bluepad32.h>

// ─── Servo configuration ────────────────────────────────────────────────────

#define SERVO_FREQ   50
#define OSC_FREQ     27000000

typedef struct {
  uint8_t channel;
  int     minTick;
  int     maxTick;
  int     rangeDeg;   // physical range of this joint in degrees
} ServoJoint;

ServoJoint shoulder_pitch = { 0,  67, 502, 210 };
ServoJoint shoulder_yaw   = { 4, 125, 575, 180 };
ServoJoint elbow          = { 8, 125, 575, 180 };
ServoJoint wrist_turn     = {12, 125, 575, 180 };

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ─── Controller state ───────────────────────────────────────────────────────

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Current angle for each joint (degrees), initialised to mid-range
float angle_pitch = 105.0f;
float angle_yaw   =  90.0f;
float angle_elbow =  90.0f;
float angle_wrist =  90.0f;

// ─── Tuning ─────────────────────────────────────────────────────────────────

// Deadzone: stick values inside ±DEADZONE are treated as zero
// (Bluepad32 range is -512 … 511)
const int   DEADZONE        = 30;

// Base speed: degrees moved per loop tick at full stick deflection
const float BASE_SPEED_DEG  = 1.5f;

// L2/R2 scale speed between MIN_SPEED_SCALE and MAX_SPEED_SCALE
// L2 pressed alone → slow, R2 pressed alone → fast
const float MIN_SPEED_SCALE = 0.2f;
const float MAX_SPEED_SCALE = 3.0f;

// Loop delay in ms (≈100 Hz)
const int   LOOP_DELAY_MS   = 10;

// ─── Helpers ────────────────────────────────────────────────────────────────

/**
 * Apply deadzone to a raw stick axis value.
 * Returns 0 if within the deadzone, otherwise the raw value.
 */
int applyDeadzone(int raw) {
  return (abs(raw) < DEADZONE) ? 0 : raw;
}

/**
 * Map a stick axis (-512 … 511) to a speed in degrees/tick.
 * The direction is preserved; magnitude is normalised to 0-1.
 */
float stickToDelta(int raw, float speedScale) {
  if (raw == 0) return 0.0f;
  float normalised = raw / 511.0f;          // -1.0 … +1.0
  return normalised * BASE_SPEED_DEG * speedScale;
}

/**
 * Move a joint by delta degrees, clamped to [0, rangeDeg].
 */
float moveJoint(float current, float delta, const ServoJoint& joint) {
  return constrain(current + delta, 0.0f, (float)joint.rangeDeg);
}

/**
 * Send a degree value to the PCA9685 for one joint.
 */
void setServoAngle(const ServoJoint& joint, float degrees) {
  int deg   = (int)constrain(degrees, 0, joint.rangeDeg);
  int pulse = map(deg, 0, joint.rangeDeg, joint.minTick, joint.maxTick);
  pwm.setPWM(joint.channel, 0, pulse);
}

// ─── Bluepad32 callbacks ────────────────────────────────────────────────────

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("Controller connected, index=%d\n", i);
      myControllers[i] = ctl;
      return;
    }
  }
  Serial.println("Too many controllers connected!");
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("Controller disconnected, index=%d\n", i);
      myControllers[i] = nullptr;
      return;
    }
  }
}

// ─── Core control logic ─────────────────────────────────────────────────────

void processController(ControllerPtr ctl) {
  if (!ctl->isConnected() || !ctl->hasData()) return;

  // --- Read sticks (apply deadzone) ---
  int lx = applyDeadzone(ctl->axisX());   // Left  X  → yaw
  int ly = applyDeadzone(ctl->axisY());   // Left  Y  → pitch  (up = negative on most pads)
  int rx = applyDeadzone(ctl->axisRX());  // Right X  → wrist
  int ry = applyDeadzone(ctl->axisRY());  // Right Y  → elbow

  // --- Speed scaling via triggers ---
  // L2 (brake) = 0-1023, R2 (throttle) = 0-1023
  int   l2    = ctl->brake();
  int   r2    = ctl->throttle();
  // Blend: pure L2 → MIN, pure R2 → MAX, neither → 1.0 (normal)
  float trig  = (r2 - l2) / 1023.0f;           // -1 … +1
  float speed = 1.0f + trig * (trig > 0
                  ? (MAX_SPEED_SCALE - 1.0f)
                  : (1.0f - MIN_SPEED_SCALE));

  // --- Compute deltas ---
  // Left Y is typically inverted (push forward = negative), so we negate it.
  float dPitch = stickToDelta(-ly, speed);
  float dYaw   = stickToDelta( lx, speed);
  float dElbow = stickToDelta(-ry, speed);
  float dWrist = stickToDelta( rx, speed);

  // --- Update joint angles ---
  angle_pitch = moveJoint(angle_pitch, dPitch, shoulder_pitch);
  angle_yaw   = moveJoint(angle_yaw,   dYaw,   shoulder_yaw);
  angle_elbow = moveJoint(angle_elbow, dElbow, elbow);
  angle_wrist = moveJoint(angle_wrist, dWrist, wrist_turn);

  // --- Drive servos ---
  setServoAngle(shoulder_pitch, angle_pitch);
  setServoAngle(shoulder_yaw,   angle_yaw);
  setServoAngle(elbow,          angle_elbow);
  setServoAngle(wrist_turn,     angle_wrist);

  // --- Optional: D-pad for homing / preset poses ---
  uint8_t dpad = ctl->dpad();
  if (dpad & DPAD_UP) {
    // Example: move to a "home" pose
    angle_pitch = 105.0f;
    angle_yaw   =  90.0f;
    angle_elbow =  90.0f;
    angle_wrist =  90.0f;
  }

  // --- Debug output (comment out for performance) ---
  Serial.printf(
    "P:%.1f Y:%.1f E:%.1f W:%.1f | spd:%.2f\n",
    angle_pitch, angle_yaw, angle_elbow, angle_wrist, speed
  );
}

// ─── Setup & loop ───────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  Serial.println("PS4 → Servo arm starting...");

  // PCA9685 init
  pwm.begin();
  pwm.setOscillatorFrequency(OSC_FREQ);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Move to home position so the arm doesn't jerk on first input
  setServoAngle(shoulder_pitch, angle_pitch);
  setServoAngle(shoulder_yaw,   angle_yaw);
  setServoAngle(elbow,          angle_elbow);
  setServoAngle(wrist_turn,     angle_wrist);

  // Bluepad32 init
  BP32.setup(&onConnectedController, &onDisconnectedController);
  // Uncomment to clear paired controllers on every boot:
  // BP32.forgetBluetoothKeys();
}

void loop() {
  bool updated = BP32.update();
  if (updated) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
      if (myControllers[i] && myControllers[i]->isConnected()) {
        processController(myControllers[i]);
      }
    }
  }
  delay(LOOP_DELAY_MS);
}