#pragma once
#include <Arduino.h>

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

// Accel/decel scaled by sensLevel (1..5)
const float ACCEL_MIN = 600.0f;
const float ACCEL_MAX = 5000.0f;

const unsigned long PRINT_PERIOD_MS = 100;

// IIR filter
const float LPF_ALPHA = 0.20f;

// Buttons
const uint16_t BTN_DEBOUNCE_MS = 10;

// ----------------- HELPERS -----------------
static inline int clampi(int v, int lo, int hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

static inline float accelForSens(int sLevel) {
  float t = (float)(clampi(sLevel, 1, 5) - 1) / 4.0f; // 0..1
  return ACCEL_MIN + t * (ACCEL_MAX - ACCEL_MIN);
}
