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
enum State { ORIENTATION, MANUAL, RETURN_HOME, AUTO };
static State state = ORIENTATION;

static bool homed = false;
static int  sensLevel = 3;     // 1..5
static int  autoDir = +1;      // +1 -> +limit, -1 -> -limit

// Hold enable (keeps driver enabled in MANUAL even when joystick released)
static bool holdEnable = false;

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
PiCommunication piComm; // reserved for later (NOT used yet)

// ----------------- HELPERS -----------------
static const char* stateName(State s) {
  switch (s) {
    case ORIENTATION: return "ORIENT";
    case MANUAL:      return "MANUAL";
    case RETURN_HOME: return "HOME";
    case AUTO:        return "AUTO";
    default:          return "?";
  }
}

static void transitionTo(State next) {
  if (state != next) {
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
  // NOTE: piComm.update() intentionally NOT called to preserve current behaviour.
  // piComm.update();

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
  // - In MANUAL: toggle HOLD enable (keep motor energized when stick released)
  if (joySW.isPressed()) {
    if (state == ORIENTATION) {
      motor.setCurrentPosition(0);
      homed = true;
      holdEnable = false;   // matches your current sketch
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

  // Auto toggle
  if (btn2.isPressed()) {
    if (!homed) {
      Serial.println(F("[EVENT] Auto ignored (not homed)."));
    } else {
      if (state == AUTO) {
        Serial.println(F("[EVENT] Auto OFF."));
        transitionTo(MANUAL);
      } else {
        Serial.println(F("[EVENT] Auto ON."));
        long p = motor.currentPosition();
        if (p >= LIMIT_STEPS) autoDir = -1;
        else if (p <= -LIMIT_STEPS) autoDir = +1;
        transitionTo(AUTO);
      }
    }
  }

  // Compute command by state
  long pos = motor.currentPosition();
  float sensMul = (float)sensLevel / 5.0f;

  bool driverEnable = false;
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

      // Enforce +/- limit
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

    case AUTO: {
      driverEnable = true;

      if (pos >= LIMIT_STEPS) autoDir = -1;
      if (pos <= -LIMIT_STEPS) autoDir = +1;

      newTarget = (float)autoDir * AUTO_SPEED;
    } break;
  }

  // Apply enable + ramped speed (identical behaviour)
  motor.apply(driverEnable, newTarget, accel_s2);

  // Serial status
  unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now;

    float deg = motor.degFromSteps(pos);

    Serial.print(F("STATE:")); Serial.print(stateName(state));
    Serial.print(F(" | EN:")); Serial.print(driverEnable ? "1" : "0");
    Serial.print(F(" | HOLD:")); Serial.print(holdEnable ? "1" : "0");

    Serial.print(F(" | X:")); Serial.print(xRaw);
    Serial.print(F(" Y:")); Serial.print(yRaw);
    Serial.print(F(" | Xc:")); Serial.print(xCenter);

    Serial.print(F(" | sens:")); Serial.print(sensLevel);
    Serial.print(F(" | accel:")); Serial.print(accel_s2, 0);

    Serial.print(F(" | steps:")); Serial.print(pos);
    Serial.print(F(" deg:")); Serial.print(deg, 1);

    Serial.print(F(" | tgtSpd:")); Serial.print(motor.targetSpeed(), 1);
    Serial.print(F(" curSpd:")); Serial.print(motor.currentSpeed(), 1);

    Serial.print(F(" | JSW:")); Serial.print(digitalRead(PIN_JOY_SW) == LOW ? "1" : "0");
    Serial.print(F(" B1:"));  Serial.print(digitalRead(PIN_BTN1) == LOW ? "1" : "0");
    Serial.print(F(" B2:"));  Serial.print(digitalRead(PIN_BTN2) == LOW ? "1" : "0");
    Serial.print(F(" RL:"));  Serial.print(digitalRead(PIN_ROCK_L) == LOW ? "1" : "0");
    Serial.print(F(" RR:"));  Serial.println(digitalRead(PIN_ROCK_R) == LOW ? "1" : "0");
  }
}
