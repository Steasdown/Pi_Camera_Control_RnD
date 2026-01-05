#include <Arduino.h>
#include <AccelStepper.h>
#include <ezButton.h>

#include "GimbalGlobal.h"
#include "PiCommunication.h"

/*
  GimbalPiControl.ino (Stage1 cleanup + compile fix)

  Goals (NO functional change):
  - Keep GimbalGlobal.h and PiCommunication.* as-is
  - Inline motor control (remove dependency on MotorControl.*)
  - Keep state machine behaviour/output the same as your current split version
  - PI_CONTROL only consumes Pi commands while in PI_CONTROL
*/

// ----------------- INPUT BUTTON OBJECTS -----------------
ezButton joySW(PIN_JOY_SW);
ezButton rockL(PIN_ROCK_L);
ezButton rockR(PIN_ROCK_R);
ezButton btn1(PIN_BTN1);
ezButton btn2(PIN_BTN2);

// ----------------- STEPPER OBJECT -----------------
// Uses pin/constants from GimbalGlobal.h
AccelStepper motor(interfaceType, motor1StepPin, motor1DirPin);

// ----------------- STATE MACHINE -----------------
/*
  ORIENTATION:
    - Jog motor with joystick (no limits until homed)
    - Press joystick switch to set HOME (pos=0), enter MANUAL

  MANUAL:
    - Joystick controls motor with soft limits +/- LIMIT_STEPS
    - Joystick switch toggles holdEnable (keeps driver enabled even when joystick released)

  RETURN_HOME:
    - Drive back to pos=0 at HOME_SPEED (scaled by sens)
    - When at home, go back to MANUAL

  PI_CONTROL:
    - Pi owns motion via serial commands (PANSPD)
    - Driver stays enabled (holds position even at 0 speed)
    - Limits still enforced
*/
enum State { ORIENTATION, MANUAL, RETURN_HOME, PI_CONTROL };
static State state = ORIENTATION;

static bool  homed = false;
static int   sensLevel = 3;        // 1..5
static bool  holdEnable = false;   // MANUAL only

// ----------------- PI CONTROL -----------------
/*
  PI_CONTROL expects commands parsed by PiCommunication:
    PANSPD <float>  -> sets piTargetSpeed (steps/s, signed)
    STOP            -> sets piTargetSpeed = 0
    HOME            -> set pos=0, leave PI_CONTROL, go MANUAL (same as joystick-home)
    MODE <...>      -> allow MANUAL or PI_CONTROL (AUTO_PI synonyms supported)
*/
PiCommunication piComm;
static float piTargetSpeed = 0.0f;

// ----------------- JOYSTICK CENTER + FILTER -----------------
static int   xCenter = 512;
static int   yCenter = 512;
static float xFilt = 512.0f;
static float yFilt = 512.0f;

// ----------------- SPEED RAMP (runSpeed mode) -----------------
/*
  We generate steps using motor.runSpeed() and implement our own accel ramp so
  behaviour matches your earlier implementation / MotorControl.apply().
*/
static float targetSpeed  = 0.0f;    // steps/sec
static float currentSpeed = 0.0f;    // steps/sec
static unsigned long lastSpeedUs = 0;

// ----------------- TELEMETRY TIMING -----------------
static unsigned long lastPrintMs = 0;

// ----------------- HELPERS -----------------
static const char* stateName(State s) {
  switch (s) {
    case ORIENTATION: return "ORIENT";
    case MANUAL:      return "MANUAL";
    case RETURN_HOME: return "HOME";
    case PI_CONTROL:  return "PICTRL";
    default:          return "?";
  }
}

static void setDriverEnable(bool enabled) {
  // A4988/DRV8825 typical: EN is ACTIVE LOW (ENABLE_ACTIVE_LOW=true in GimbalGlobal.h)
  if (ENABLE_ACTIVE_LOW) digitalWrite(motor1EnablePin, enabled ? LOW : HIGH);
  else                   digitalWrite(motor1EnablePin, enabled ? HIGH : LOW);
}

static void resetRamp() {
  targetSpeed = 0.0f;
  currentSpeed = 0.0f;
  lastSpeedUs = micros();
}

static float degFromSteps(long steps) {
  return ((float)steps / (float)STEPS_PER_REV) * 360.0f;
}

static void updateSpeedRamp(float accel_s2) {
  unsigned long nowUs = micros();
  if (lastSpeedUs == 0) lastSpeedUs = nowUs;

  float dt = (float)(nowUs - lastSpeedUs) / 1000000.0f;
  lastSpeedUs = nowUs;
  if (dt <= 0.0f) return;

  float maxDelta = accel_s2 * dt;
  float diff = targetSpeed - currentSpeed;

  if (diff > maxDelta)       currentSpeed += maxDelta;
  else if (diff < -maxDelta) currentSpeed -= maxDelta;
  else                       currentSpeed = targetSpeed;
}

static void applyMotor(bool enabled, float newTargetSpeed, float accel_s2) {
  setDriverEnable(enabled);

  if (!enabled) {
    // Hard stop + reset ramp (matches original / MotorControl.apply() behaviour)
    targetSpeed = 0.0f;
    currentSpeed = 0.0f;
    motor.setSpeed(0.0f);
    return;
  }

  targetSpeed = newTargetSpeed;
  updateSpeedRamp(accel_s2);
  motor.setSpeed(currentSpeed);
  motor.runSpeed();
}

static void transitionTo(State next) {
  if (state != next) {
    // On leaving PI_CONTROL, stop motion + clear pending command
    if (state == PI_CONTROL) {
      piTargetSpeed = 0.0f;
      piComm.clear();
    }
    state = next;
    Serial.print(F("[STATE] -> "));
    Serial.println(stateName(state));
  }
}

static void calibrateJoystickCenter() {
  long sx = 0, sy = 0;
  const int N = 120;
  for (int i = 0; i < N; i++) {
    sx += analogRead(PIN_JOY_X);
    sy += analogRead(PIN_JOY_Y);
    delay(5);
  }
  xCenter = (int)(sx / N);
  yCenter = (int)(sy / N);

  xFilt = (float)xCenter;
  yFilt = (float)yCenter;

  Serial.print(F("[BOOT] Center calibrated: Xc="));
  Serial.print(xCenter);
  Serial.print(F(" Yc="));
  Serial.println(yCenter);
}

static float joystickToSpeed(int xRaw, int xC, float maxSpeed) {
  int dx = xRaw - xC;
  if (abs(dx) < DEAD_BAND_ADC) return 0.0f;

  float frac = (float)dx / 512.0f;
  if (frac > 1.0f) frac = 1.0f;
  if (frac < -1.0f) frac = -1.0f;

  return frac * maxSpeed;
}

static bool eqI(const char* a, const char* b) {
  while (*a && *b) {
    char ca = *a++;
    char cb = *b++;
    if (ca >= 'a' && ca <= 'z') ca -= 32;
    if (cb >= 'a' && cb <= 'z') cb -= 32;
    if (ca != cb) return false;
  }
  return (*a == '\0' && *b == '\0');
}

static void handlePiCommandsOnlyInPiControl() {
  // Only read/act on commands when in PI_CONTROL
  if (state != PI_CONTROL) return;

  piComm.update();
  if (!piComm.hasCommand()) return;

  PiCommand c = piComm.lastCmd();
  piComm.clear();

  switch (c.type) {
    case PiCommandType::PANSPD:
      // Store speed; applied in PI_CONTROL state below
      piTargetSpeed = c.value;
      break;

    case PiCommandType::STOP:
      piTargetSpeed = 0.0f;
      break;

    case PiCommandType::HOME:
      // Same behaviour as setting home via joystick press
      motor.setCurrentPosition(0);
      homed = true;
      holdEnable = true;
      piTargetSpeed = 0.0f;
      resetRamp();
      setDriverEnable(true);
      Serial.println(F("[EVENT] HOME set (pos=0) [PI]."));
      transitionTo(MANUAL);
      break;

    case PiCommandType::MODE:
      // Support earlier plan strings as well
      if (eqI(c.arg0, "MANUAL")) {
        if (homed) transitionTo(MANUAL);
      } else if (eqI(c.arg0, "AUTO_PI") || eqI(c.arg0, "PICTRL") || eqI(c.arg0, "PI_CONTROL")) {
        if (homed) transitionTo(PI_CONTROL);
      }
      break;

    case PiCommandType::STATUS:
      // No extra output here (telemetry already periodic)
      break;

    default:
      break;
  }
}

// ----------------- SETUP -----------------
void setup() {
  Serial.begin(115200);

  // Inputs
  pinMode(PIN_JOY_SW, INPUT_PULLUP);
  pinMode(PIN_ROCK_L, INPUT_PULLUP);
  pinMode(PIN_ROCK_R, INPUT_PULLUP);
  pinMode(PIN_BTN1,   INPUT_PULLUP);
  pinMode(PIN_BTN2,   INPUT_PULLUP);

  joySW.setDebounceTime(BTN_DEBOUNCE_MS);
  rockL.setDebounceTime(BTN_DEBOUNCE_MS);
  rockR.setDebounceTime(BTN_DEBOUNCE_MS);
  btn1.setDebounceTime(BTN_DEBOUNCE_MS);
  btn2.setDebounceTime(BTN_DEBOUNCE_MS);

  // Motor driver
  pinMode(motor1EnablePin, OUTPUT);
  setDriverEnable(false);

  motor.setMaxSpeed(BASE_MAX_SPEED);
  motor.setCurrentPosition(0);
  resetRamp();

  // Boot info
  Serial.println(F("Stepper+Joystick (state machine)"));
  Serial.println(F("Boot state: ORIENT. Press joystick button to set HOME and enter MANUAL."));
  Serial.print(F("Limit: +/-")); Serial.print(LIMIT_DEG); Serial.println(F(" deg from HOME."));

  calibrateJoystickCenter();
  transitionTo(ORIENTATION);
}

// ----------------- LOOP -----------------
void loop() {
  // ---- 1) Update buttons ----
  joySW.loop();
  rockL.loop();
  rockR.loop();
  btn1.loop();
  btn2.loop();

  // ---- 2) Read + filter joystick ----
  int xRaw = analogRead(PIN_JOY_X);
  int yRaw = analogRead(PIN_JOY_Y);

  xFilt = (1.0f - LPF_ALPHA) * xFilt + LPF_ALPHA * (float)xRaw;
  yFilt = (1.0f - LPF_ALPHA) * yFilt + LPF_ALPHA * (float)yRaw;

  int xUse = (int)(xFilt + 0.5f);
  int yUse = (int)(yFilt + 0.5f);
  (void)yUse; // reserved

  // ---- 3) Sensitivity rocker (1..5) ----
  if (rockL.isPressed()) {
    sensLevel = clampi(sensLevel - 1, 1, 5);
    Serial.print(F("[EVENT] Sens- level=")); Serial.println(sensLevel);
  }
  if (rockR.isPressed()) {
    sensLevel = clampi(sensLevel + 1, 1, 5);
    Serial.print(F("[EVENT] Sens+ level=")); Serial.println(sensLevel);
  }

  // ---- 4) Joystick button behaviour ----
  // ORIENT: set home -> MANUAL
  // MANUAL: toggle holdEnable
  if (joySW.isPressed()) {
    if (state == ORIENTATION) {
      motor.setCurrentPosition(0);
      homed = true;
      holdEnable = true;
      piTargetSpeed = 0.0f;
      resetRamp();
      setDriverEnable(true);
      Serial.println(F("[EVENT] HOME set (pos=0)."));
      transitionTo(MANUAL);
    } else if (state == MANUAL) {
      holdEnable = !holdEnable;
      Serial.print(F("[EVENT] HoldEnable -> "));
      Serial.println(holdEnable ? "ON" : "OFF");
    }
  }

  // ---- 5) Return-home ----
  if (btn1.isPressed()) {
    if (homed) {
      Serial.println(F("[EVENT] Return-to-home requested."));
      transitionTo(RETURN_HOME);
    } else {
      Serial.println(F("[EVENT] Return-to-home ignored (not homed)."));
    }
  }

  // ---- 6) Button 2 toggles PI_CONTROL ----
  if (btn2.isPressed()) {
    if (!homed) {
      Serial.println(F("[EVENT] PiControl ignored (not homed)."));
    } else {
      if (state == PI_CONTROL) {
        Serial.println(F("[EVENT] PiControl OFF."));
        piTargetSpeed = 0.0f;
        transitionTo(MANUAL);
      } else {
        Serial.println(F("[EVENT] PiControl ON."));
        piTargetSpeed = 0.0f;
        transitionTo(PI_CONTROL);
      }
    }
  }

  // ---- 7) Pi commands (PI_CONTROL only) ----
  handlePiCommandsOnlyInPiControl();

  // ---- 8) Compute target speed + enable by state ----
  long pos = motor.currentPosition();
  float sensMul = (float)sensLevel / 5.0f;
  float accel_s2 = accelForSens(sensLevel); // from GimbalGlobal.h

  bool  driverEnable = false;
  float newTarget = 0.0f;

  switch (state) {
    case ORIENTATION: {
      // Joystick jog; no limits until homed
      int dx = xUse - xCenter;
      bool joyActive = (abs(dx) >= ENABLE_THRESH_ADC);

      driverEnable = true;
      newTarget = joyActive ? joystickToSpeed(xUse, xCenter, BASE_MAX_SPEED * sensMul) : 0.0f;
    } break;

    case MANUAL: {
      // Manual joystick with soft limits, enable if active or holdEnable
      int dx = xUse - xCenter;
      bool joyActive = (abs(dx) >= ENABLE_THRESH_ADC);

      driverEnable = joyActive || holdEnable;
      newTarget = joyActive ? joystickToSpeed(xUse, xCenter, BASE_MAX_SPEED * sensMul) : 0.0f;

      // Enforce +/- LIMIT
      if (pos >= LIMIT_STEPS && newTarget > 0) newTarget = 0.0f;
      if (pos <= -LIMIT_STEPS && newTarget < 0) newTarget = 0.0f;
    } break;

    case RETURN_HOME: {
      driverEnable = true;

      if (pos == 0) {
        newTarget = 0.0f;
        Serial.println(F("[EVENT] At HOME."));
        transitionTo(MANUAL);
      } else {
        float hs = HOME_SPEED * sensMul;
        newTarget = (pos > 0) ? -hs : +hs;
      }
    } break;

    case PI_CONTROL: {
      // Pi owns movement; always enabled to "hold"
      driverEnable = true;

      float maxPi = BASE_MAX_SPEED * sensMul;
      newTarget = piTargetSpeed;

      // Cap by sens
      if (newTarget >  maxPi) newTarget =  maxPi;
      if (newTarget < -maxPi) newTarget = -maxPi;

      // Enforce limits
      if (pos >= LIMIT_STEPS && newTarget > 0) newTarget = 0.0f;
      if (pos <= -LIMIT_STEPS && newTarget < 0) newTarget = 0.0f;
    } break;
  }

  // ---- 9) Apply motor enable + ramp + stepping ----
  applyMotor(driverEnable, newTarget, accel_s2);

  // ---- 10) Telemetry (condensed) ----
  unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now;

    float deg = degFromSteps(pos);

    Serial.print(F("T,"));
    Serial.print(stateName(state));
    Serial.print(F(","));
    Serial.print(sensLevel);
    Serial.print(F(","));
    Serial.print(driverEnable ? 1 : 0);
    Serial.print(F(","));
    Serial.print(holdEnable ? 1 : 0);
    Serial.print(F(","));
    Serial.print(pos);
    Serial.print(F(","));
    Serial.println(deg, 1);
  }
}
