#include "LowSpeedVelocityController.h"

LowSpeedVelocityController::LowSpeedVelocityController() {}

void LowSpeedVelocityController::setParams(float max_accel, float deadzone,
                                           float soft_zone) {
  max_accel_ = max_accel;
  deadzone_ = deadzone;
  soft_zone_ = soft_zone;
}

void LowSpeedVelocityController::setFriction(float coulomb, float viscous,
                                             float soft_sign) {
  coulomb_ = coulomb;
  viscous_ = viscous;
  soft_sign_ = soft_sign;
}

void LowSpeedVelocityController::reset() { target_filtered_ = 0.0f; }

float LowSpeedVelocityController::rateLimit(float new_target) {
  float max_delta = max_accel_ * CONTROL_DT;
  float delta = new_target - target_filtered_;
  if (delta > max_delta) delta = max_delta;
  if (delta < -max_delta) delta = -max_delta;
  target_filtered_ += delta;
  return target_filtered_;
}

float LowSpeedVelocityController::applyDeadzone(float r) {
  float abs_r = fabsf(r);
  if (abs_r < deadzone_) return 0.0f;

  if (abs_r < soft_zone_) {
    float k = (abs_r - deadzone_) / (soft_zone_ - deadzone_);
    return r * k * k;
  }
  return r;
}

float LowSpeedVelocityController::process(float raw_target) {
  return applyDeadzone(rateLimit(raw_target));
}

// Плавный sign() — убирает скачок при проходе через ноль
float LowSpeedVelocityController::smoothSign(float x) {
  if (soft_sign_ < 1e-4f) {
    return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f);
  }
  float a = x / soft_sign_;
  if (a > 1.0f) return 1.0f;
  if (a < -1.0f) return -1.0f;
  // гладкая аппроксимация: 1.5x - 0.5x³
  return 1.5f * a - 0.5f * a * a * a;
}

float LowSpeedVelocityController::frictionCompensation(float velocity) {
  float u_c = coulomb_ * smoothSign(velocity);
  float u_v = viscous_ * velocity;
  return u_c + u_v;
}