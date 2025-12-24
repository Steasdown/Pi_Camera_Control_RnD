#include <AccelStepper.h>
#include <ezButton.h>

// ----------------- STEPPER PINS -----------------
#define motor1DirPin    7
#define motor1StepPin   6
#define motor1EnablePin 8
#define interfaceType   1

const bool ENABLE_ACTIVE_LOW = true;

// ----------------- INPUT PINS -----------------
const int PIN_JOY_X  = A14;
const int PIN_JOY_Y  = A15;
const int PIN_JOY_SW = 22;

const int PIN_ROCK_L = 23;
const int PIN_ROCK_R = 24;
const int PIN_BTN1   = 26;
const int PIN_BTN2   = 27;

// ----------------- MECHANICS -----------------
const long  STEPS_PER_REV = 6400;
const float LIMIT_DEG = 90.0f;
const long  LIMIT_STEPS = (long)((STEPS_PER_REV * (LIMIT_DEG / 360.0f)) + 0.5f);

// ----------------- CONTROL TUNING -----------------
const int   ENABLE_THRESH_ADC = 150;
const int   DEAD_BAND_ADC     = 40;

const float BASE_MAX_SPEED = 1800.0f;
const float HOME_SPEED     = 900.0f;
const float AUTO_SPEED     = 400.0f;

// Accel/decel (steps/s^2) scaled by sensLevel (1..5)
// You can tune these two numbers; higher = snappier response
const float ACCEL_MIN = 600.0f;
const float ACCEL_MAX = 5000.0f;

const unsigned long PRINT_PERIOD_MS = 100;

// ----------------- OBJECTS -----------------
AccelStepper motor1(interfaceType, motor1StepPin, motor1DirPin);

ezButton joySW(PIN_JOY_SW);
ezButton rockL(PIN_ROCK_L);
ezButton rockR(PIN_ROCK_R);
ezButton btn1(PIN_BTN1);
ezButton btn2(PIN_BTN2);

// ----------------- STATE MACHINE -----------------
enum State { ORIENTATION, MANUAL, RETURN_HOME, AUTO };
State state = ORIENTATION;

bool homed = false;
int  sensLevel = 3;        // 1..5
int  autoDir = +1;         // +1 -> +limit, -1 -> -limit

// NEW: hold enable (keeps driver enabled in MANUAL even when joystick released)
bool holdEnable = false;

// Joystick center calibration
int xCenter = 512;
int yCenter = 512;

// IIR filter
float xFilt = 512.0f;
float yFilt = 512.0f;
const float LPF_ALPHA = 0.20f;

// NEW: speed ramp state
float targetSpeed = 0.0f;     // steps/sec
float currentSpeed = 0.0f;    // steps/sec
unsigned long lastSpeedUs = 0;

unsigned long lastPrintMs = 0;

// ----------------- HELPERS -----------------
static int clampi(int v, int lo, int hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

void setDriverEnable(bool enabled) {
  if (ENABLE_ACTIVE_LOW) digitalWrite(motor1EnablePin, enabled ? LOW : HIGH);
  else                  digitalWrite(motor1EnablePin, enabled ? HIGH : LOW);
}

const char* stateName(State s) {
  switch (s) {
    case ORIENTATION: return "ORIENT";
    case MANUAL:      return "MANUAL";
    case RETURN_HOME: return "HOME";
    case AUTO:        return "AUTO";
    default:          return "?";
  }
}

void calibrateJoystickCenter() {
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

float joystickToSpeed(int xRaw, int xC, float maxSpeed) {
  int dx = xRaw - xC;
  if (abs(dx) < DEAD_BAND_ADC) return 0.0f;

  float frac = (float)dx / 512.0f;
  if (frac > 1.0f) frac = 1.0f;
  if (frac < -1.0f) frac = -1.0f;

  return frac * maxSpeed;
}

float degFromSteps(long steps) {
  return ((float)steps / (float)STEPS_PER_REV) * 360.0f;
}

void transitionTo(State next) {
  if (state != next) {
    state = next;
    Serial.print(F("[STATE] -> "));
    Serial.println(stateName(state));
  }
}

// NEW: accel scaled by sensLevel (1..5)
float accelForSens(int sLevel) {
  float t = (float)(clampi(sLevel, 1, 5) - 1) / 4.0f; // 0..1
  return ACCEL_MIN + t * (ACCEL_MAX - ACCEL_MIN);
}

// NEW: ramp currentSpeed toward targetSpeed using accel/decel limit
void updateSpeedRamp(float accel_s2) {
  unsigned long nowUs = micros();
  if (lastSpeedUs == 0) lastSpeedUs = nowUs;
  float dt = (float)(nowUs - lastSpeedUs) / 1000000.0f;
  lastSpeedUs = nowUs;

  if (dt <= 0.0f) return;

  float maxDelta = accel_s2 * dt;

  float diff = targetSpeed - currentSpeed;
  if (diff > maxDelta)      currentSpeed += maxDelta;
  else if (diff < -maxDelta) currentSpeed -= maxDelta;
  else                      currentSpeed = targetSpeed;
}

// ----------------- SETUP -----------------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_JOY_SW, INPUT_PULLUP);
  pinMode(PIN_ROCK_L, INPUT_PULLUP);
  pinMode(PIN_ROCK_R, INPUT_PULLUP);
  pinMode(PIN_BTN1,   INPUT_PULLUP);
  pinMode(PIN_BTN2,   INPUT_PULLUP);

  joySW.setDebounceTime(10);
  rockL.setDebounceTime(10);
  rockR.setDebounceTime(10);
  btn1.setDebounceTime(10);
  btn2.setDebounceTime(10);

  pinMode(motor1EnablePin, OUTPUT);
  setDriverEnable(false);

  motor1.setMaxSpeed(BASE_MAX_SPEED);
  motor1.setCurrentPosition(0);

  Serial.println(F("Stepper+Joystick (state machine)"));
  Serial.println(F("Boot state: ORIENT. Press joystick button to set HOME and enter MANUAL."));
  Serial.print(F("Limit: +/-")); Serial.print(LIMIT_DEG); Serial.println(F(" deg from HOME."));

  calibrateJoystickCenter();
  transitionTo(ORIENTATION);

  lastSpeedUs = micros();
}

// ----------------- LOOP -----------------
void loop() {
  joySW.loop();
  rockL.loop();
  rockR.loop();
  btn1.loop();
  btn2.loop();

  int xRaw = analogRead(PIN_JOY_X);
  int yRaw = analogRead(PIN_JOY_Y);
  xFilt = (1.0f - LPF_ALPHA) * xFilt + LPF_ALPHA * (float)xRaw;
  yFilt = (1.0f - LPF_ALPHA) * yFilt + LPF_ALPHA * (float)yRaw;
  int xUse = (int)(xFilt + 0.5f);
  int yUse = (int)(yFilt + 0.5f);

  // Sensitivity rocker
  if (rockL.isPressed()) {
    sensLevel = clampi(sensLevel - 1, 1, 5);
    Serial.print(F("[EVENT] Sens- level=")); Serial.println(sensLevel);
  }
  if (rockR.isPressed()) {
    sensLevel = clampi(sensLevel + 1, 1, 5);
    Serial.print(F("[EVENT] Sens+ level=")); Serial.println(sensLevel);
  }

  // Joystick button behavior:
  // - In ORIENT: set HOME and go MANUAL
  // - In MANUAL: toggle HOLD enable (keep motor energized when stick released)
  if (joySW.isPressed()) {
    if (state == ORIENTATION) {
      motor1.setCurrentPosition(0);
      homed = true;
      holdEnable = false;         // start manual without hold unless you prefer true
      targetSpeed = 0;
      currentSpeed = 0;
      setDriverEnable(false);
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
        long p = motor1.currentPosition();
        if (p >= LIMIT_STEPS) autoDir = -1;
        else if (p <= -LIMIT_STEPS) autoDir = +1;
        transitionTo(AUTO);
      }
    }
  }

  // Compute command by state
  long pos = motor1.currentPosition();
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

      // NEW: enable if joystick active OR holdEnable
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

  // Apply enable + ramped speed
  setDriverEnable(driverEnable);

  // If disabled, force hard stop + reset ramp
  if (!driverEnable) {
    targetSpeed = 0.0f;
    currentSpeed = 0.0f;
    motor1.setSpeed(0.0f);
  } else {
    targetSpeed = newTarget;
    updateSpeedRamp(accel_s2);
    motor1.setSpeed(currentSpeed);
    motor1.runSpeed(); // speed-mode stepping, we provide accel/decel via ramp
  }

  // Serial status
  unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now;

    float deg = degFromSteps(pos);

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

    Serial.print(F(" | tgtSpd:")); Serial.print(targetSpeed, 1);
    Serial.print(F(" curSpd:")); Serial.print(currentSpeed, 1);

    Serial.print(F(" | JSW:")); Serial.print(digitalRead(PIN_JOY_SW) == LOW ? "1" : "0");
    Serial.print(F(" B1:"));  Serial.print(digitalRead(PIN_BTN1) == LOW ? "1" : "0");
    Serial.print(F(" B2:"));  Serial.print(digitalRead(PIN_BTN2) == LOW ? "1" : "0");
    Serial.print(F(" RL:"));  Serial.print(digitalRead(PIN_ROCK_L) == LOW ? "1" : "0");
    Serial.print(F(" RR:"));  Serial.println(digitalRead(PIN_ROCK_R) == LOW ? "1" : "0");
  }
}
