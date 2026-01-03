#pragma once
#include <Arduino.h>
#include <AccelStepper.h>
#include "GimbalGlobal.h"

// BigMover-style: motor behaviour encapsulated in a class.
class MotorControl {
public:
  void begin();
  void setMaxSpeed(float sps);
  void setCurrentPosition(long steps);
  long currentPosition() const;

  void setDriverEnable(bool enabled);

  // Apply: enable pin + target speed + ramp update + step generation
  void apply(bool enabled, float newTargetSpeed, float accel_s2);

  void resetRamp();
  float targetSpeed() const { return targetSpeed_; }
  float currentSpeed() const { return currentSpeed_; }

  float degFromSteps(long steps) const;

private:
  AccelStepper motor_{interfaceType, motor1StepPin, motor1DirPin};

  float targetSpeed_ = 0.0f;     // steps/sec
  float currentSpeed_ = 0.0f;    // steps/sec
  unsigned long lastSpeedUs_ = 0;

  void updateSpeedRamp_(float accel_s2);
};
