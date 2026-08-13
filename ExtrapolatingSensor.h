#pragma once
#include "common/base_classes/Sensor.h"
#include "common/foc_utils.h"

class ExtrapolatingSensor : public Sensor {
 public:
  explicit ExtrapolatingSensor(Sensor& wrapped, float quant = 0.0003835f)
      : _wrapped(wrapped), quant_(quant) {}

  // Главная точка входа — вызывается из loopFOC()
  void update() override {
    // 1. Обновляем сырой датчик
    _wrapped.update();

    // 2. Считаем dt
    float t = _micros() * 1e-6f;
    float dt = t - t_prev_;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.001f;
    t_prev_ = t;

    // 3. Предсказание (экстраполяция)
    //    cmd_vel можно передавать снаружи через setCommandVelocity()
    float blend = 0.03f;
    vel_ = (1.0f - blend) * vel_ + blend * cmd_vel_;
    pos_ += vel_ * dt;

    // 4. Коррекция, если пришёл новый LSB
    float raw = _wrapped.getMechanicalAngle();
    float delta = raw - last_meas_;

    if (fabsf(delta) >= 0.5f * quant_) {
      float dt_jump = t - t_last_;
      if (dt_jump < 1e-4f) dt_jump = 0.001f;

      float vel_jump = delta / dt_jump;
      float alpha_v = constrain(0.12f / dt_jump, 0.02f, 0.35f);
      vel_ = (1.0f - alpha_v) * vel_ + alpha_v * vel_jump;

      float residual = raw - pos_;
      residual = constrain(residual, -3.0f * quant_, 3.0f * quant_);
      pos_ += 0.25f * residual;

      last_meas_ = raw;
      t_last_ = t;
    }

    // 5. Обновляем внутреннее состояние базового Sensor
    //    (full_rotations, angle_prev и т.д.)
    Sensor::update();
  }

  void init() override {
    Sensor::init();
    t_prev_ = _micros() * 1e-6f;
  }

  // Позволяет передать текущую целевую скорость из motion control
  void setCommandVelocity(float v) { cmd_vel_ = v; }

  float position() const { return pos_; }
  float velocity() const { return vel_; }

 protected:
  // Вызывается из Sensor::update()
  float getSensorAngle() override {
    // Просто возвращаем уже посчитанную экстраполированную позицию.
    // Вся логика сделана в update().
    return pos_;
  }

 private:
  Sensor& _wrapped;
  float quant_;

  float pos_ = 0.0f;
  float vel_ = 0.0f;
  float last_meas_ = 0.0f;
  float t_last_ = 0.0f;
  float t_prev_ = 0.0f;
  float cmd_vel_ = 0.0f;
  bool initialized_ = false;
};