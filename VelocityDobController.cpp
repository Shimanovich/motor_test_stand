#include "VelocityDobController.h"

#include "config.h"

VelocityDobController::VelocityDobController() : LPF_vel(0.018f) {}

void VelocityDobController::setPID(float kp, float ki, float kd) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
}

void VelocityDobController::setDob(float bn, float g) {
  bn_ = bn;
  g_ = g;
}

void VelocityDobController::setFilters(float lpf_tf, float max_accel,
                                       float deadzone, float soft_zone) {
  LPF_vel.Tf = lpf_tf;
  max_accel_ = max_accel;
  deadzone_ = deadzone;
  soft_zone_ = soft_zone;
}

void VelocityDobController::setVoltageLimit(float limit) {
  voltage_limit_ = limit;
}

float VelocityDobController::rateLimit(float new_target) {
  float max_delta = max_accel_ * CONTROL_DT;
  float delta = new_target - target_filtered;
  if (delta > max_delta) delta = max_delta;
  if (delta < -max_delta) delta = -max_delta;
  target_filtered += delta;
  return target_filtered;
}

float VelocityDobController::processTarget(float r) {
  float abs_r = fabsf(r);
  if (abs_r < deadzone_) return 0.0f;
  if (abs_r < soft_zone_) {
    float k = (abs_r - deadzone_) / (soft_zone_ - deadzone_);
    return r * k * k;
  }
  return r;
}

float VelocityDobController::disturbanceObserver(float y, float u) {
  float inv_bn = 1.0f / bn_;
  float temp = inv_bn * y;
  float d_hat = g_ * (temp - dob_z);
  dob_z += CONTROL_DT * (u + d_hat);
  return d_hat;
}

float VelocityDobController::operator()(FOCMotor* motor) {
  // 1. Фильтрация скорости
  float y = LPF_vel(motor->shaft_velocity);

  // 2. Обработка задания
  float r = processTarget(rateLimit(motor->target));

  // 3. Ошибка
  float error = r - y;

  // 4. PID + anti-windup
  integral += error * CONTROL_DT;
  float max_int = voltage_limit_ / (Ki + 1e-6f);
  integral = constrain(integral, -max_int, max_int);

  float derivative = (error - last_error) / CONTROL_DT;
  last_error = error;

  float u_pid = Kp * error + Ki * integral + Kd * derivative;

  // 5. DOB
  float d_hat = disturbanceObserver(y, last_u);

  d_hat = 0.0;
  // 6. Итоговое управление
  float u = u_pid - d_hat;
  u = constrain(u, -voltage_limit_, voltage_limit_);

  last_u = u;
  return u;
}