#include <AccelStepper.h>
#include <ezButton.h>

// ----------------- STEPPER PINS -----------------
#define motor1DirPin    7
#define motor1StepPin   6
#define motor1EnablePin 8   // NEW: driver enable pin
#define interfaceType   1

// A4988/DRV8825 typical: EN is ACTIVE LOW
const bool ENABLE_ACTIVE_LOW = true;

// ----------------- INPUT PINS -----------------
const int PIN_JOY_X  = A14;  // left/right
const int PIN_JOY_Y  = A15;  // reserved for later; read+print now
const int PIN_JOY_SW = 22;   // set HOME, active LOW

const int PIN_ROCK_L = 23;   // sens -
const int PIN_ROCK_R = 24;   // sens +
const int PIN_BTN1   = 26;   // return-to-home
const int PIN_BTN2   = 27;   // auto toggle

// ----------------- MECHANICS -----------------
const long STEPS_PER_REV = 6400;  // confirmed full-step rev
const float LIMIT_DEG = 150.0f;
const long LIMIT_STEPS = (long)((STEPS_PER_REV * (LIMIT_DEG / 360.0f)) + 0.5f);

// ----------------- CONTROL TUNING -----------------
const int   ADC_MIN = 0;
const int   ADC_MAX = 1023;

const int   ENABLE_THRESH_ADC = 150; // enable only beyond +/-150 from center (manual/orientation)
const int   DEAD_BAND_ADC     = 40;  // additional dead band for speed calc once enabled

const float BASE_MAX_SPEED = 1800.0f; // steps/sec (manual), scaled by sensitivity
const float HOME_SPEED     = 900.0f;  // steps/sec (return home), scaled by sensitivity
const float AUTO_SPEED     = 180.0f;  // steps/sec constant sweep

const unsigned long PRINT_PERIOD_MS = 100;

// ----------------- OBJECTS -----------------
AccelStepper motor1(interfaceType, motor1StepPin, motor1DirPin);

ezButton joySW(PIN_JOY_SW);
ezButton rockL(PIN_ROCK_L);
ezButton rockR(PIN_ROCK_R);
ezButton btn1(PIN_BTN1);
ezButton btn2(PIN_BTN2);

// ----------------- STATE MACHINE -----------------
enum State {
  ORIENTATION,
  MANUAL,
  RETURN_HOME,
  AUTO
};

State state = ORIENTATION;
bool homed = false;
int sensLevel = 3;    // 1..5
int autoDir = +1;     // +1 -> +limit, -1 -> -limit

// Joystick center calibration (done at boot)
int xCenter = 512;
int yCenter = 512;

// Simple IIR filter to stabilize joystick reads
float xFilt = 512.0f;
float yFilt = 512.0f;
const float LPF_ALPHA = 0.20f; // higher = faster response, lower = smoother

unsigned long lastPrintMs = 0;

// ----------------- HELPERS -----------------
static int clampi(int v, int lo, int hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

void setDriverEnable(bool enabled) {
  // enabled=true => allow motor current
  if (ENABLE_ACTIVE_LOW) {
    digitalWrite(motor1EnablePin, enabled ? LOW : HIGH);
  } else {
    digitalWrite(motor1EnablePin, enabled ? HIGH : LOW);
  }
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
  // Assume user is not touching stick for ~0.6s
  long sx = 0, sy = 0;
  const int N = 120;
  for (int i = 0; i < N; i++) {
    int xr = analogRead(PIN_JOY_X);
    int yr = analogRead(PIN_JOY_Y);
    sx += xr;
    sy += yr;
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
  // Only called when we already decided joystick is "active" beyond ENABLE_THRESH_ADC.
  int dx = xRaw - xC;

  // Apply an extra dead band for speed once enabled
  if (abs(dx) < DEAD_BAND_ADC) return 0.0f;

  // Map dx to [-1..+1]
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

// ----------------- SETUP -----------------
void setup() {
  Serial.begin(115200);

  // Ensure stable input modes (avoid floating)
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
  motor1.setAcceleration(2500.0f); // higher default accel
  motor1.setCurrentPosition(0);

  Serial.println(F("Stepper+Joystick (state machine)"));
  Serial.println(F("Boot state: ORIENT (jog L/R). Press joystick button to set HOME and enter MANUAL."));
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
  int xRaw = analogRead(PIN_JOY_X); // 0..1023
  int yRaw = analogRead(PIN_JOY_Y); // 0..1023
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

  // Home-set (orientation -> manual)
  if (joySW.isPressed()) {
    motor1.setCurrentPosition(0);
    homed = true;
    setDriverEnable(false);
    motor1.setSpeed(0);
    Serial.println(F("[EVENT] HOME set (pos=0)."));
    transitionTo(MANUAL);
  }

  // Return-home button (momentary press triggers state; stays until at home)
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
        // choose direction away from current limit if sitting at a limit
        long p = motor1.currentPosition();
        if (p >= LIMIT_STEPS) autoDir = -1;
        else if (p <= -LIMIT_STEPS) autoDir = +1;
        transitionTo(AUTO);
      }
    }
  }

  // Compute motor command by state
  long pos = motor1.currentPosition();
  float sensMul = (float)sensLevel / 5.0f;
  float cmdSpeed = 0.0f;
  bool driverEnable = false;

  switch (state) {
    case ORIENTATION: {
      // Allow jogging before HOME is set (no soft limit until homed)
      int dx = xUse - xCenter;
      bool joyActive = (abs(dx) >= ENABLE_THRESH_ADC);

      driverEnable = joyActive;
      if (joyActive) {
        cmdSpeed = joystickToSpeed(xUse, xCenter, BASE_MAX_SPEED * sensMul);
      } else {
        cmdSpeed = 0.0f;
      }
    } break;

    case MANUAL: {
      // Manual joystick control with limits relative to HOME
      int dx = xUse - xCenter;
      bool joyActive = (abs(dx) >= ENABLE_THRESH_ADC);

      driverEnable = joyActive;
      if (joyActive) {
        cmdSpeed = joystickToSpeed(xUse, xCenter, BASE_MAX_SPEED * sensMul);

        // Enforce +/-150deg limit
        if (pos >= LIMIT_STEPS && cmdSpeed > 0) cmdSpeed = 0;
        if (pos <= -LIMIT_STEPS && cmdSpeed < 0) cmdSpeed = 0;
      } else {
        cmdSpeed = 0.0f;
      }
    } break;

    case RETURN_HOME: {
      // Enable regardless of joystick, drive back to 0
      driverEnable = true;

      if (pos == 0) {
        cmdSpeed = 0.0f;
        Serial.println(F("[EVENT] At HOME."));
        transitionTo(MANUAL);
      } else {
        float hs = HOME_SPEED * sensMul;
        cmdSpeed = (pos > 0) ? -hs : +hs;
      }
    } break;

    case AUTO: {
      // Enable regardless of joystick, sweep full range with limit reversals
      driverEnable = true;

      if (pos >= LIMIT_STEPS) autoDir = -1;
      if (pos <= -LIMIT_STEPS) autoDir = +1;

      cmdSpeed = (float)autoDir * AUTO_SPEED;
    } break;
  }

  // Apply enable + speed
  setDriverEnable(driverEnable);

  // If disabled, force speed 0 (prevents any stepping)
  if (!driverEnable) cmdSpeed = 0.0f;

  motor1.setSpeed(cmdSpeed);
  motor1.run(); // speed-mode stepping

  // Serial status
  unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now;

    float deg = degFromSteps(pos);

    Serial.print(F("STATE:")); Serial.print(stateName(state));
    Serial.print(F(" | EN:")); Serial.print(driverEnable ? "1" : "0");
    Serial.print(F(" | X:")); Serial.print(xRaw);
    Serial.print(F(" Y:")); Serial.print(yRaw);
    Serial.print(F(" | Xc:")); Serial.print(xCenter);
    Serial.print(F(" Yc:")); Serial.print(yCenter);

    Serial.print(F(" | sens:")); Serial.print(sensLevel);
    Serial.print(F(" | homed:")); Serial.print(homed ? "1" : "0");

    Serial.print(F(" | steps:")); Serial.print(pos);
    Serial.print(F(" deg:")); Serial.print(deg, 1);

    Serial.print(F(" | cmdSpd:")); Serial.print(cmdSpeed, 1);

    Serial.print(F(" | JSW:")); Serial.print(digitalRead(PIN_JOY_SW) == LOW ? "1" : "0");
    Serial.print(F(" B1:"));  Serial.print(digitalRead(PIN_BTN1) == LOW ? "1" : "0");
    Serial.print(F(" B2:"));  Serial.print(digitalRead(PIN_BTN2) == LOW ? "1" : "0");
    Serial.print(F(" RL:"));  Serial.print(digitalRead(PIN_ROCK_L) == LOW ? "1" : "0");
    Serial.print(F(" RR:"));  Serial.println(digitalRead(PIN_ROCK_R) == LOW ? "1" : "0");
  }
}
