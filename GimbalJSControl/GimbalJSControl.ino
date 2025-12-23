#include <AccelStepper.h>
#include <ezButton.h>

// ----------------- PINS (match your example for motor1) -----------------
#define motor1DirPin  7
#define motor1StepPin 6
#define interfaceType 1

const int PIN_JOY_X  = A14;  // left/right
const int PIN_JOY_Y  = A15;  // (reserved for later), read+print only
const int PIN_JOY_SW = 22;   // HOME set (zero) at startup, active LOW

const int PIN_ROCK_L = 23;   // sens -
const int PIN_ROCK_R = 24;   // sens +
const int PIN_BTN1   = 26;   // Return-to-home
const int PIN_BTN2   = 27;   // Auto mode toggle

// ----------------- STEPPER CONFIG -----------------
const long STEPS_PER_REV = 200; // UPDATE for microstepping!
const float LIMIT_DEG = 150.0f;
const long LIMIT_STEPS = (long)((STEPS_PER_REV * (LIMIT_DEG / 360.0f)) + 0.5f);

const bool DIR_INVERT = false;

// Manual control base speed (steps/sec), scaled by sensitivity
const float BASE_MAX_SPEED = 400.0f;

// Auto mode speed (slow pan), steps/sec
const float AUTO_SPEED = 60.0f;

const int JOY_DEADBAND = 35;

// ----------------- OBJECTS -----------------
AccelStepper motor1(interfaceType, motor1StepPin, motor1DirPin);

ezButton joySW(PIN_JOY_SW);
ezButton rockL(PIN_ROCK_L);
ezButton rockR(PIN_ROCK_R);
ezButton btn1(PIN_BTN1);
ezButton btn2(PIN_BTN2);

// ----------------- STATE -----------------
bool homed = false;
int sensLevel = 3;           // 1..5
bool autoMode = false;
int autoDir = +1;            // +1 = toward +limit, -1 = toward -limit

unsigned long lastPrintMs = 0;
const unsigned long PRINT_PERIOD_MS = 100;

static int clampi(int v, int lo, int hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

static float joyToFrac(int adc) {
  int d = adc - 512;
  if (abs(d) <= JOY_DEADBAND) return 0.0f;
  if (d > 0) return (float)(d - JOY_DEADBAND) / (float)(511 - JOY_DEADBAND);
  return (float)(d + JOY_DEADBAND) / (float)(512 - JOY_DEADBAND);
}

// simple "go home" using speed mode (no blocking)
void updateReturnToHome(float maxSpeedStepsPerSec) {
  long pos = motor1.currentPosition();
  if (pos == 0) {
    motor1.setSpeed(0);
    return;
  }
  float s = maxSpeedStepsPerSec;
  if (pos > 0) motor1.setSpeed(-s);
  else motor1.setSpeed(+s);
}

void setup() {
  Serial.begin(115200);

  joySW.setDebounceTime(10);
  rockL.setDebounceTime(10);
  rockR.setDebounceTime(10);
  btn1.setDebounceTime(10);
  btn2.setDebounceTime(10);

  motor1.setMaxSpeed(BASE_MAX_SPEED * 5.0f);
  motor1.setAcceleration(200.0f);
  motor1.setCurrentPosition(0);

  Serial.println(F("Stepper+Joystick ready."));
  Serial.println(F("Startup: jog L/R then press joystick button to set HOME (zero)."));
  Serial.print(F("Soft limit: +/-")); Serial.print(LIMIT_DEG); Serial.println(F(" deg from home."));
}

void loop() {
  joySW.loop();
  rockL.loop();
  rockR.loop();
  btn1.loop();
  btn2.loop();

  // --------- events ----------
  if (joySW.isPressed()) {
    motor1.setCurrentPosition(0);
    homed = true;
    autoMode = false; // stop auto when homing
    Serial.println(F("[EVENT] HOME set: current position zeroed. Auto OFF."));
  }

  if (rockL.isPressed()) {
    sensLevel = clampi(sensLevel - 1, 1, 5);
    Serial.print(F("[EVENT] Sens - -> level ")); Serial.println(sensLevel);
  }
  if (rockR.isPressed()) {
    sensLevel = clampi(sensLevel + 1, 1, 5);
    Serial.print(F("[EVENT] Sens + -> level ")); Serial.println(sensLevel);
  }

  if (btn2.isPressed()) {
    autoMode = !autoMode;
    Serial.print(F("[EVENT] Auto mode -> ")); Serial.println(autoMode ? "ON" : "OFF");
    // When enabling auto, choose direction based on current pos to avoid immediate stall
    if (autoMode) {
      long p = motor1.currentPosition();
      autoDir = (p >= 0) ? -1 : +1; // head back toward center-ish first
    }
  }

  // Read joysticks (always)
  int x = analogRead(PIN_JOY_X);
  int y = analogRead(PIN_JOY_Y);

  // --------- control selection ----------
  float cmdSpeed = 0.0f;

  // Button1: return to home (overrides everything except joystick HOME set)
  bool returnHome = (btn1.getState() == LOW);
  if (btn1.isPressed()) Serial.println(F("[EVENT] Return-to-home pressed"));

  long pos = motor1.currentPosition();

  if (returnHome) {
    // return at a moderate speed, scaled by sensitivity
    float sensMul = (float)sensLevel / 5.0f;
    float homeSpeed = 180.0f * sensMul;
    updateReturnToHome(homeSpeed);
    cmdSpeed = motor1.speed();
    autoMode = false; // holding return-home cancels auto
  }
  else if (autoMode) {
    // Auto: sweep full range left<->right
    // Reverse direction at limits
    if (pos >= LIMIT_STEPS) autoDir = -1;
    if (pos <= -LIMIT_STEPS) autoDir = +1;

    cmdSpeed = (float)autoDir * AUTO_SPEED;
    if (DIR_INVERT) cmdSpeed = -cmdSpeed;
    motor1.setSpeed(cmdSpeed);
  }
  else {
    // Manual joystick on X only
    float frac = joyToFrac(x);
    if (DIR_INVERT) frac = -frac;

    float sensMul = (float)sensLevel / 5.0f;
    cmdSpeed = frac * (BASE_MAX_SPEED * sensMul);

    // enforce limits
    if (pos >= LIMIT_STEPS && cmdSpeed > 0) cmdSpeed = 0;
    if (pos <= -LIMIT_STEPS && cmdSpeed < 0) cmdSpeed = 0;

    motor1.setSpeed(cmdSpeed);
  }

  motor1.run();

  // --------- serial status ----------
  unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now;

    float deg = ((float)pos / (float)STEPS_PER_REV) * 360.0f;

    Serial.print(F("JOY_X:")); Serial.print(x);
    Serial.print(F(" JOY_Y:")); Serial.print(y);

    Serial.print(F(" | sens:")); Serial.print(sensLevel);
    Serial.print(F(" | homed:")); Serial.print(homed ? "1" : "0");
    Serial.print(F(" | auto:")); Serial.print(autoMode ? "1" : "0");

    Serial.print(F(" | steps:")); Serial.print(pos);
    Serial.print(F(" deg:")); Serial.print(deg, 1);

    Serial.print(F(" | JSW:")); Serial.print(joySW.getState() == LOW ? "1" : "0");
    Serial.print(F(" B1(home):")); Serial.print(btn1.getState() == LOW ? "1" : "0");
    Serial.print(F(" B2(auto):")); Serial.print(btn2.getState() == LOW ? "1" : "0");
    Serial.print(F(" RL:")); Serial.print(rockL.getState() == LOW ? "1" : "0");
    Serial.print(F(" RR:")); Serial.println(rockR.getState() == LOW ? "1" : "0");
  }
}
