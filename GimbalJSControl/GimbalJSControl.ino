// Minimal input debug: joystick + joystick button + 2 buttons + 1P2T rocker
// Arduino Mega
// All digital inputs debounced (10 ms). Active LOW inputs using INPUT_PULLUP.

const int PIN_JOY_X   = A14;
const int PIN_JOY_Y   = A15;
const int PIN_JOY_SW  = 22;  // active LOW

const int PIN_BTN1    = 26;  // active LOW
const int PIN_BTN2    = 27;  // active LOW

const int PIN_ROCK_L  = 23;  // active LOW
const int PIN_ROCK_R  = 24;  // active LOW

const unsigned long DEBOUNCE_MS = 10;
unsigned long lastPrintMs = 0;
const unsigned long PRINT_PERIOD_MS = 100;  // 10 Hz update

struct DebouncedInput {
  int pin;
  bool stable;                 // stable debounced level (HIGH/LOW)
  bool lastRaw;
  unsigned long lastChangeMs;  // last time raw changed
};

DebouncedInput di_jsw  = {PIN_JOY_SW,  HIGH, HIGH, 0};
DebouncedInput di_b1   = {PIN_BTN1,    HIGH, HIGH, 0};
DebouncedInput di_b2   = {PIN_BTN2,    HIGH, HIGH, 0};
DebouncedInput di_rl   = {PIN_ROCK_L,  HIGH, HIGH, 0};
DebouncedInput di_rr   = {PIN_ROCK_R,  HIGH, HIGH, 0};

void initDebounced(DebouncedInput &d) {
  pinMode(d.pin, INPUT_PULLUP);
  bool r = digitalRead(d.pin);
  d.stable = r;
  d.lastRaw = r;
  d.lastChangeMs = millis();
}

void updateDebounced(DebouncedInput &d) {
  bool r = digitalRead(d.pin);
  unsigned long now = millis();

  if (r != d.lastRaw) {
    d.lastRaw = r;
    d.lastChangeMs = now;
  }

  if ((now - d.lastChangeMs) >= DEBOUNCE_MS && d.stable != r) {
    d.stable = r;
  }
}

bool isPressed(const DebouncedInput &d) {
  return d.stable == LOW; // active LOW
}

void setup() {
  Serial.begin(115200);

  initDebounced(di_jsw);
  initDebounced(di_b1);
  initDebounced(di_b2);
  initDebounced(di_rl);
  initDebounced(di_rr);

  Serial.println("Input monitor started (active LOW for buttons/switch).");
  Serial.println("Format: X Y | JSW BTN1 BTN2 | ROCK_L ROCK_R");
}

void loop() {
  // Update debounced inputs every loop
  updateDebounced(di_jsw);
  updateDebounced(di_b1);
  updateDebounced(di_b2);
  updateDebounced(di_rl);
  updateDebounced(di_rr);

  unsigned long now = millis();
  if (now - lastPrintMs < PRINT_PERIOD_MS) return;
  lastPrintMs = now;

  int x = analogRead(PIN_JOY_X);  // 0..1023
  int y = analogRead(PIN_JOY_Y);  // 0..1023

  bool jsw   = isPressed(di_jsw);
  bool b1    = isPressed(di_b1);
  bool b2    = isPressed(di_b2);
  bool rockL = isPressed(di_rl);
  bool rockR = isPressed(di_rr);

  Serial.print("X:");
  Serial.print(x);
  Serial.print(" Y:");
  Serial.print(y);

  Serial.print(" | JSW:");
  Serial.print(jsw ? "1" : "0");
  Serial.print(" B1:");
  Serial.print(b1 ? "1" : "0");
  Serial.print(" B2:");
  Serial.print(b2 ? "1" : "0");

  Serial.print(" | RL:");
  Serial.print(rockL ? "1" : "0");
  Serial.print(" RR:");
  Serial.println(rockR ? "1" : "0");
}
