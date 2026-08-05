#pragma once
#include <SimpleFOC.h>

class VelocityDobController {
 public:
  VelocityDobController();

  // Основная функция, которую регистрируем в SimpleFOC
  float operator()(FOCMotor* motor);

  // Настройка параметров
  void setPID(float kp, float ki, float kd = 0.0f);
  void setDob(float bn, float g);
  void setFilters(float lpf_tf, float max_accel, float deadzone,
                  float soft_zone);
  void setVoltageLimit(float limit);

  // Для Commander
  float& kp() { return Kp; }
  float& ki() { return Ki; }
  float& bn() { return bn_; }
  float& g() { return g_; }

 private:
  // PID
  float Kp = 0.8f;
  float Ki = 8.0f;
  float Kd = 0.0f;
  float integral = 0.0f;
  float last_error = 0.0f;

  // DOB
  float bn_ = 45.0f;
  float g_ = 35.0f;
  float dob_z = 0.0f;

  // Состояния
  float last_u = 0.0f;
  float voltage_limit_ = 6.0f;

  // Фильтры и ограничения
  LowPassFilter LPF_vel;
  float target_filtered = 0.0f;
  float max_accel_ = 12.0f;
  float deadzone_ = 0.10f;
  float soft_zone_ = 0.35f;

  // Вспомогательные методы
  float rateLimit(float new_target);
  float processTarget(float r);
  float disturbanceObserver(float y, float u);
};