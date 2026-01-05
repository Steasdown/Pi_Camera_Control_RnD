#include "MotorControl.h"

void MotorControl::begin() {
  pinMode(motor1EnablePin, OUTPUT);
  setDriverEnable(false);

  motor_.setMaxSpeed(BASE_MAX_SPEED);
  motor_.setCurrentPosition(0);

  resetRamp();
}

void MotorControl::setMaxSpeed(float sps) {
  motor_.setMaxSpeed(sps);
}

void MotorControl::setCurrentPosition(long steps) {
  motor_.setCurrentPosition(steps);
}

long MotorControl::currentPosition() const {
  return motor_.currentPosition();
}

void MotorControl::setDriverEnable(bool enabled) {
  if (ENABLE_ACTIVE_LOW) digitalWrite(motor1EnablePin, enabled ? LOW : HIGH);
  else                  digitalWrite(motor1EnablePin, enabled ? HIGH : LOW);
}

float MotorControl::degFromSteps(long steps) const {
  return ((float)steps / (float)STEPS_PER_REV) * 360.0f;
}

void MotorControl::resetRamp() {
  targetSpeed_ = 0.0f;
  currentSpeed_ = 0.0f;
  lastSpeedUs_ = micros();
}

void MotorControl::updateSpeedRamp_(float accel_s2) {
  unsigned long nowUs = micros();
  if (lastSpeedUs_ == 0) lastSpeedUs_ = nowUs;
  float dt = (float)(nowUs - lastSpeedUs_) / 1000000.0f;
  lastSpeedUs_ = nowUs;

  if (dt <= 0.0f) return;

  float maxDelta = accel_s2 * dt;
  float diff = targetSpeed_ - currentSpeed_;

  if (diff > maxDelta)       currentSpeed_ += maxDelta;
  else if (diff < -maxDelta) currentSpeed_ -= maxDelta;
  else                       currentSpeed_ = targetSpeed_;
}

void MotorControl::apply(bool enabled, float newTargetSpeed, float accel_s2) {
  setDriverEnable(enabled);

  if (!enabled) {
    // Hard stop + reset ramp (matches original behaviour)
    targetSpeed_ = 0.0f;
    currentSpeed_ = 0.0f;
    motor_.setSpeed(0.0f);
    return;
  }

  targetSpeed_ = newTargetSpeed;
  updateSpeedRamp_(accel_s2);

  motor_.setSpeed(currentSpeed_);
  motor_.runSpeed(); // speed-mode stepping
}
