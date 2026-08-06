#pragma once
#include <SimpleFOC.h>

#include "config.h"

class LowSpeedVelocityController {
 public:
  LowSpeedVelocityController();

  // Обработка задания (rate limit + deadzone)
  float process(float raw_target);

  // Компенсация трения (feedforward)
  float frictionCompensation(float velocity);

  void setParams(float max_accel, float deadzone, float soft_zone);
  void setFriction(float coulomb, float viscous, float soft_sign);
  void reset();

  // Commander
  float& maxAccel() { return max_accel_; }
  float& deadzone() { return deadzone_; }
  float& softZone() { return soft_zone_; }
  float& coulomb() { return coulomb_; }
  float& viscous() { return viscous_; }
  float& softSign() { return soft_sign_; }

 private:
  // Сглаживание задания
  float target_filtered_ = 0.0f;
  float max_accel_ = 10.0f;
  float deadzone_ = 0.08f;
  float soft_zone_ = 0.30f;

  // Трение
  float coulomb_ = 0.0f;     // В (напряжение, эквивалент сухого трения)
  float viscous_ = 0.0f;     // В / (рад/с)
  float soft_sign_ = 0.15f;  // зона сглаживания sign(), рад/с

  float rateLimit(float new_target);
  float applyDeadzone(float r);
  float smoothSign(float x);
};