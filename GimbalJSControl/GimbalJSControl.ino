#include <AccelStepper.h>
#include <ezButton.h>

// ----------------- STEPPER PINS -----------------
#define motor1DirPin    7
#define motor1StepPin   6
#define motor1EnablePin 8
#define interfaceType   1

// A4988/DRV8825 typical: EN is ACTIVE LOW
const bool ENABLE_ACTIVE_LOW = true;

// ----------------- INPUT PINS -----------------
const int PIN_JOY_X  = A14;  // left/right
const int PIN_JOY_Y  = A15;  // reserved for later (still read for center calibration stability)
const int PIN_JOY_SW = 22;   // active LOW

const int PIN_ROCK_L = 23;   // sens -
const int PIN_ROCK_R = 24;   // sens +
const int PIN_BTN1   = 26;   // return-to-home
const int PIN_BTN2   = 27;   // mode toggle/cycle

// ----------------- MECHANICS -----------------
const long  STEPS_PER_REV = 6400;
const float LIMIT_DEG     = 150.0f;
const long  LIMIT_STEPS   = (long)((STEPS_PER_REV * (LIMIT_DEG / 360.0f)) + 0.5f);

// ----------------- CONTROL TUNING -----------------
const int   ENABLE_THRESH_ADC = 90;   // joystick must move this far from center to "engage"
const int   DEAD_BAND_ADC     = 40;   // extra deadband after engaged

const float BASE_MAX_SPEED = 1800.0f; // steps/sec (manual), scaled by sens
const float HOME_SPEED     = 900.0f;  // steps/sec (return home), scaled by sens
const float AUTO_SPEED     = 400.0f;  // steps/sec (auto sweep)

// accel/decel for ramping (steps/s^2) scaled by sens (1..5)
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
enum State {
  ORIENTATION,
  MANUAL,
  RETURN_HOME,
  AUTO_SWEEP,
  AUTO_PI
};

State state = ORIENTATION;
bool homed = false;

// Sensitivity level 1..5
int sensLevel = 3;

// Auto sweep direction
int autoDir = +1;

// Hold enable toggle (MANUAL only); AUTO_* states always keep enabled
bool holdEnable = false;

// Joystick center calibration (boot)
int xCenter = 512;
int yCenter = 512;

// IIR filter for joystick reads
float xFilt = 512.0f;
float yFilt = 512.0f;
const float LPF_ALPHA = 0.20f;

// Speed ramp variables (we use runSpeed(), acceleration applied by our ramp)
float targetSpeed  = 0.0f;   // steps/sec
float currentSpeed = 0.0f;   // steps/sec
unsigned long lastSpeedUs = 0;

// AUTO_PI commanded speed from Pi (steps/sec)
float piCmdSpeed = 0.0f;

// Serial parsing buffer
static const uint16_t CMD_BUF_SZ = 96;
char cmdBuf[CMD_BUF_SZ];
uint16_t cmdLen = 0;

// Print timing
unsigned long lastPrintMs = 0;

// ----------------- HELPERS -----------------
static int clampi(int v, int lo, int hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

void setDriverEnable(bool enabled) {
  if (ENABLE_ACTIVE_LOW) digitalWrite(motor1EnablePin, enabled ? LOW : HIGH);
  else                   digitalWrite(motor1EnablePin, enabled ? HIGH : LOW);
}

const char* stateName(State s) {
  switch (s) {
    case ORIENTATION: return "ORIENT";
    case MANUAL:      return "MANUAL";
    case RETURN_HOME: return "HOME";
    case AUTO_SWEEP:  return "AUTO_SWEEP";
    case AUTO_PI:     return "AUTO_PI";
    default:          return "?";
  }
}

void emitState(State s) {
  Serial.print(F("S,"));
  Serial.println(stateName(s));
}

void emitEvent(const __FlashStringHelper* name) {
  Serial.print(F("E,"));
  Serial.println(name);
}

void emitEvent1(const __FlashStringHelper* name, const char* arg) {
  Serial.print(F("E,"));
  Serial.print(name);
  Serial.print(F(","));
  Serial.println(arg);
}

float degFromSteps(long steps) {
  return ((float)steps / (float)STEPS_PER_REV) * 360.0f;
}

void transitionTo(State next) {
  if (state != next) {
    state = next;
    emitState(state);
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

  Serial.print(F("E,BOOT_CENTER,"));
  Serial.print(xCenter);
  Serial.print(F(","));
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

float accelForSens(int sLevel) {
  float t = (float)(clampi(sLevel, 1, 5) - 1) / 4.0f; // 0..1
  return ACCEL_MIN + t * (ACCEL_MAX - ACCEL_MIN);
}

void updateSpeedRamp(float accel_s2) {
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

void hardStopAndDisable() {
  targetSpeed = 0.0f;
  currentSpeed = 0.0f;
  motor1.setSpeed(0.0f);
  setDriverEnable(false);
}

// ----------------- SERIAL COMMANDS -----------------
void replyOK(const char* cmd) {
  Serial.print(F("OK,"));
  Serial.println(cmd);
}

void replyERR(const char* reason) {
  Serial.print(F("ERR,"));
  Serial.println(reason);
}

void printStatusOnce() {
  long pos = motor1.currentPosition();
  float deg = degFromSteps(pos);

  // enabled definition for telemetry: whether driver is currently enabled
  // We track this by reading the EN pin state (works if pin driven only here).
  bool en = (ENABLE_ACTIVE_LOW) ? (digitalRead(motor1EnablePin) == LOW) : (digitalRead(motor1EnablePin) == HIGH);

  Serial.print(F("T,"));
  Serial.print(stateName(state));
  Serial.print(F(","));
  Serial.print(sensLevel);
  Serial.print(F(","));
  Serial.print(en ? 1 : 0);
  Serial.print(F(","));
  Serial.print(holdEnable ? 1 : 0);
  Serial.print(F(","));
  Serial.print(pos);
  Serial.print(F(","));
  Serial.println(deg, 1);
}

void setModeFromString(const char* m) {
  if (!homed && (strcmp(m, "AUTO_SWEEP") == 0 || strcmp(m, "AUTO_PI") == 0 || strcmp(m, "MANUAL") == 0)) {
    // MANUAL requires home to make sense with limits; keep strict
    replyERR("NOT_HOMED");
    return;
  }

  if (strcmp(m, "MANUAL") == 0) {
    transitionTo(MANUAL);
    replyOK("MODE");
    return;
  }
  if (strcmp(m, "AUTO_SWEEP") == 0) {
    transitionTo(AUTO_SWEEP);
    replyOK("MODE");
    return;
  }
  if (strcmp(m, "AUTO_PI") == 0) {
    transitionTo(AUTO_PI);
    replyOK("MODE");
    return;
  }

  replyERR("BAD_MODE");
}

void handleCommand(char* line) {
  // trim CR/LF
  while (*line == ' ') line++;

  if (strcmp(line, "STATUS?") == 0) {
    printStatusOnce();
    replyOK("STATUS?");
    return;
  }

  if (strcmp(line, "STOP") == 0) {
    piCmdSpeed = 0.0f;
    replyOK("STOP");
    return;
  }

  if (strcmp(line, "HOME") == 0) {
    if (!homed) {
      replyERR("NOT_HOMED");
      return;
    }
    transitionTo(RETURN_HOME);
    replyOK("HOME");
    return;
  }

  // MODE <...>
  if (strncmp(line, "MODE ", 5) == 0) {
    const char* m = line + 5;
    setModeFromString(m);
    return;
  }

  // PANSPD <float>
  if (strncmp(line, "PANSPD ", 7) == 0) {
    const char* v = line + 7;
    piCmdSpeed = (float)atof(v);
    replyOK("PANSPD");
    return;
  }

  replyERR("UNKNOWN_CMD");
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        handleCommand(cmdBuf);
        cmdLen = 0;
      }
      continue;
    }

    if (cmdLen < (CMD_BUF_SZ - 1)) {
      cmdBuf[cmdLen++] = c;
    } else {
      // overflow -> reset
      cmdLen = 0;
      replyERR("CMD_TOO_LONG");
    }
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

  joySW.setDebounceTime(10);
  rockL.setDebounceTime(10);
  rockR.setDebounceTime(10);
  btn1.setDebounceTime(10);
  btn2.setDebounceTime(10);

  pinMode(motor1EnablePin, OUTPUT);
  setDriverEnable(false);

  motor1.setMaxSpeed(BASE_MAX_SPEED);
  motor1.setCurrentPosition(0);

  calibrateJoystickCenter();

  transitionTo(ORIENTATION);

  lastSpeedUs = micros();
}

// ----------------- LOOP -----------------
void loop() {
  // 1) Poll serial early (Pi commands)
  pollSerial();

  // 2) Update buttons
  joySW.loop();
  rockL.loop();
  rockR.loop();
  btn1.loop();
  btn2.loop();

  // 3) Read + filter joystick
  int xRaw = analogRead(PIN_JOY_X);
  int yRaw = analogRead(PIN_JOY_Y);
  xFilt = (1.0f - LPF_ALPHA) * xFilt + LPF_ALPHA * (float)xRaw;
  yFilt = (1.0f - LPF_ALPHA) * yFilt + LPF_ALPHA * (float)yRaw;
  int xUse = (int)(xFilt + 0.5f);

  // 4) Sensitivity rocker
  if (rockL.isPressed()) {
    sensLevel = clampi(sensLevel - 1, 1, 5);
    emitEvent(F("SENS-"));
  }
  if (rockR.isPressed()) {
    sensLevel = clampi(sensLevel + 1, 1, 5);
    emitEvent(F("SENS+"));
  }

  // 5) Joystick press:
  // - ORIENT: set home + enter MANUAL
  // - MANUAL: toggle hold
  if (joySW.isPressed()) {
    if (state == ORIENTATION) {
      motor1.setCurrentPosition(0);
      homed = true;

      holdEnable = false;
      piCmdSpeed = 0.0f;

      targetSpeed = 0.0f;
      currentSpeed = 0.0f;
      setDriverEnable(false);

      emitEvent(F("HOME_SET"));
      transitionTo(MANUAL);
    } else if (state == MANUAL) {
      holdEnable = !holdEnable;
      emitEvent1(F("HOLD"), holdEnable ? "1" : "0");
    }
  }

  // 6) Return-home button
  if (btn1.isPressed()) {
    if (homed) {
      emitEvent(F("RET_HOME"));
      transitionTo(RETURN_HOME);
    } else {
      emitEvent(F("RET_HOME_IGN_NOT_HOMED"));
    }
  }

  // 7) Mode cycle button:
  // MANUAL -> AUTO_SWEEP -> AUTO_PI -> MANUAL (only if homed)
  if (btn2.isPressed()) {
    if (!homed) {
      emitEvent(F("MODE_IGN_NOT_HOMED"));
    } else {
      if (state == MANUAL) transitionTo(AUTO_SWEEP);
      else if (state == AUTO_SWEEP) transitionTo(AUTO_PI);
      else if (state == AUTO_PI) transitionTo(MANUAL);
      else transitionTo(MANUAL);
      emitEvent(F("MODE_CYCLE"));
    }
  }

  // 8) Compute target speed + enable logic
  long pos = motor1.currentPosition();
  float sensMul = (float)sensLevel / 5.0f;
  float accel_s2 = accelForSens(sensLevel);

  bool driverEnable = false;
  float newTarget = 0.0f;

  switch (state) {
    case ORIENTATION: {
      // Jogging allowed; no limit until homed (but still require joystick engagement)
      int dx = xUse - xCenter;
      bool joyActive = (abs(dx) >= ENABLE_THRESH_ADC);

      driverEnable = joyActive;
      newTarget = joyActive ? joystickToSpeed(xUse, xCenter, BASE_MAX_SPEED * sensMul) : 0.0f;
    } break;

    case MANUAL: {
      // Manual joystick with soft limits, enable only if active or holdEnable
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
        emitEvent(F("AT_HOME"));
        transitionTo(MANUAL);
      } else {
        float hs = HOME_SPEED * sensMul;
        newTarget = (pos > 0) ? -hs : +hs;
      }
    } break;

    case AUTO_SWEEP: {
      // Auto sweep across full range
      driverEnable = true;

      if (pos >= LIMIT_STEPS) autoDir = -1;
      if (pos <= -LIMIT_STEPS) autoDir = +1;

      newTarget = (float)autoDir * AUTO_SPEED;
    } break;

    case AUTO_PI: {
      // PI controls speed via PANSPD; always enable to hold position even at 0 speed
      driverEnable = true;

      // Enforce limits (clamp speed if command tries to push further)
      float cmd = piCmdSpeed;

      if (pos >= LIMIT_STEPS && cmd > 0) cmd = 0.0f;
      if (pos <= -LIMIT_STEPS && cmd < 0) cmd = 0.0f;

      // Optional: cap absolute speed to manual max (scaled by sens)
      float cap = BASE_MAX_SPEED * sensMul;
      if (cmd > cap) cmd = cap;
      if (cmd < -cap) cmd = -cap;

      newTarget = cmd;
    } break;
  }

  // 9) Apply enable + ramped speed
  setDriverEnable(driverEnable);

  if (!driverEnable) {
    targetSpeed = 0.0f;
    currentSpeed = 0.0f;
    motor1.setSpeed(0.0f);
  } else {
    targetSpeed = newTarget;
    updateSpeedRamp(accel_s2);
    motor1.setSpeed(currentSpeed);
    motor1.runSpeed();
  }

  // 10) Telemetry (B fields only)
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
