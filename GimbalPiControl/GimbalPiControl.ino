#include <ezButton.h>
#include "GimbalGlobal.h"
#include "MotorControl.h"
#include "PiCommunication.h"

// ----------------- INPUT BUTTON OBJECTS -----------------
ezButton joySW(PIN_JOY_SW);
ezButton rockL(PIN_ROCK_L);
ezButton rockR(PIN_ROCK_R);
ezButton btn1(PIN_BTN1);
ezButton btn2(PIN_BTN2);

// ----------------- STATE MACHINE -----------------
enum State { ORIENTATION, MANUAL, RETURN_HOME, PI_CONTROL };
static State state = ORIENTATION;

static bool homed = false;
static int  sensLevel = 3;     // 1..5

// Hold enable (keeps driver enabled in MANUAL even when joystick released)
static bool holdEnable = false;

// PI_CONTROL: speed command from Pi (steps/sec, signed)
static float piTargetSpeed = 0.0f;

// Joystick center calibration
static int xCenter = 512;
static int yCenter = 512;

// IIR filter
static float xFilt = 512.0f;
static float yFilt = 512.0f;

// Periodic print timing
static unsigned long lastPrintMs = 0;

// ----------------- MODULES -----------------
MotorControl motor;
PiCommunication piComm;

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

static void transitionTo(State next) {
  if (state != next) {
    // On leaving PI_CONTROL, stop motion for safety
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
      // Store speed; applied by PI_CONTROL state below
      piTargetSpeed = c.value;
      break;

    case PiCommandType::STOP:
      piTargetSpeed = 0.0f;
      break;

    case PiCommandType::HOME:
      motor.setCurrentPosition(0);
      homed = true;
      holdEnable = false;
      piTargetSpeed = 0.0f;
      motor.resetRamp();
      motor.setDriverEnable(false);
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

  motor.begin();
  motor.setMaxSpeed(BASE_MAX_SPEED);
  motor.setCurrentPosition(0);

  Serial.println(F("Stepper+Joystick (state machine)"));
  Serial.println(F("Boot state: ORIENT. Press joystick button to set HOME and enter MANUAL."));
  Serial.print(F("Limit: +/-")); Serial.print(LIMIT_DEG); Serial.println(F(" deg from HOME."));

  calibrateJoystickCenter();
  transitionTo(ORIENTATION);
}

// ----------------- LOOP -----------------
void loop() {
  // Update buttons
  joySW.loop();
  rockL.loop();
  rockR.loop();
  btn1.loop();
  btn2.loop();

  // Read + filter joystick raw values
  int xRaw = analogRead(PIN_JOY_X);
  int yRaw = analogRead(PIN_JOY_Y);

  xFilt = (1.0f - LPF_ALPHA) * xFilt + LPF_ALPHA * (float)xRaw;
  yFilt = (1.0f - LPF_ALPHA) * yFilt + LPF_ALPHA * (float)yRaw;

  int xUse = (int)(xFilt + 0.5f);
  int yUse = (int)(yFilt + 0.5f);
  (void)yUse; // reserved for later

  // Sensitivity rocker
  if (rockL.isPressed()) {
    sensLevel = clampi(sensLevel - 1, 1, 5);
    Serial.print(F("[EVENT] Sens- level=")); Serial.println(sensLevel);
  }
  if (rockR.isPressed()) {
    sensLevel = clampi(sensLevel + 1, 1, 5);
    Serial.print(F("[EVENT] Sens+ level=")); Serial.println(sensLevel);
  }

  // Joystick button behaviour:
  // - In ORIENT: set HOME and go MANUAL
  // - In MANUAL: toggle HOLD enable
  if (joySW.isPressed()) {
    if (state == ORIENTATION) {
      motor.setCurrentPosition(0);
      homed = true;
      holdEnable = false;
      piTargetSpeed = 0.0f;
      motor.resetRamp();
      motor.setDriverEnable(false);
      Serial.println(F("[EVENT] HOME set (pos=0)."));
      transitionTo(MANUAL);
    } else if (state == MANUAL) {
      holdEnable = !holdEnable;
      Serial.print(F("[EVENT] HoldEnable -> "));
      Serial.println(holdEnable ? "ON" : "OFF");
    }
  }

  // Return-home
  if (btn1.isPressed()) {
    if (homed) {
      Serial.println(F("[EVENT] Return-to-home requested."));
      transitionTo(RETURN_HOME);
    } else {
      Serial.println(F("[EVENT] Return-to-home ignored (not homed)."));
    }
  }

  // Button 2 now toggles PI_CONTROL (replaces AUTO)
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

  // Enable pi comm only in PI_CONTROL mode
  handlePiCommandsOnlyInPiControl();

  // Compute command by state
  long pos = motor.currentPosition();
  float sensMul = (float)sensLevel / 5.0f;

  bool  driverEnable = false;
  float newTarget = 0.0f;

  float accel_s2 = accelForSens(sensLevel);

  switch (state) {
    case ORIENTATION: {
      int dx = xUse - xCenter;
      bool joyActive = (abs(dx) >= ENABLE_THRESH_ADC);

      driverEnable = joyActive;
      newTarget = joyActive ? joystickToSpeed(xUse, xCenter, BASE_MAX_SPEED * sensMul) : 0.0f;
    } break;

    case MANUAL: {
      int dx = xUse - xCenter;
      bool joyActive = (abs(dx) >= ENABLE_THRESH_ADC);

      driverEnable = joyActive || holdEnable;
      newTarget = joyActive ? joystickToSpeed(xUse, xCenter, BASE_MAX_SPEED * sensMul) : 0.0f;

      if (pos >= LIMIT_STEPS && newTarget > 0) newTarget = 0;
      if (pos <= -LIMIT_STEPS && newTarget < 0) newTarget = 0;
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
      // Pi owns movement. No auto sweep.
      driverEnable = true;

      // Optional: let sensitivity cap max speed
      float maxPi = BASE_MAX_SPEED * sensMul;
      newTarget = piTargetSpeed;
      if (newTarget >  maxPi) newTarget =  maxPi;
      if (newTarget < -maxPi) newTarget = -maxPi;

      if (pos >= LIMIT_STEPS && newTarget > 0) newTarget = 0;
      if (pos <= -LIMIT_STEPS && newTarget < 0) newTarget = 0;
    } break;
  }

  motor.apply(driverEnable, newTarget, accel_s2);

  // Serial status (condensed)
  unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now;

    float deg = motor.degFromSteps(pos);

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
