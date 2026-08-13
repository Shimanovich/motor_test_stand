#pragma once
#include "common/base_classes/Sensor.h"
#include "common/foc_utils.h"

class ExtrapolatingSensor : public Sensor {
 public:
  explicit ExtrapolatingSensor(Sensor& wrapped, float quant = 0.0003835f)
      : _wrapped(wrapped), quant_(quant) {}

  void update() override {
    _wrapped.update();

    float t = _micros() * 1e-6f;
    float dt = t - t_prev_;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.001f;
    t_prev_ = t;

    float raw = _wrapped.getMechanicalAngle();  // [0, 2π)

    if (!initialized_) {
      pos_ = raw;
      last_meas_ = raw;
      vel_ = 0.0f;
      t_last_ = t;
      initialized_ = true;
      Sensor::update();
      return;
    }

    // 1. Предсказание
    const float blend = 0.02f;  // слабое притяжение к команде
    vel_ = (1.0f - blend) * vel_ + blend * cmd_vel_;
    pos_ += vel_ * dt;

    // 2. Коррекция только при новом LSB
    float delta = raw - last_meas_;
    // учёт перехода через 0
    if (delta > _PI) delta -= _2PI;
    if (delta < -_PI) delta += _2PI;

    if (fabsf(delta) >= 0.5f * quant_) {
      float dt_jump = t - t_last_;
      if (dt_jump < 1e-4f) dt_jump = 0.001f;

      float vel_jump = delta / dt_jump;

      // очень мягкое обновление скорости
      float alpha_v = constrain(0.08f / dt_jump, 0.01f, 0.25f);
      vel_ = (1.0f - alpha_v) * vel_ + alpha_v * vel_jump;

      // очень мягкая коррекция позиции (главное против рывков)
      float residual = raw - pos_;
      if (residual > _PI) residual -= _2PI;
      if (residual < -_PI) residual += _2PI;

      residual = constrain(residual, -2.0f * quant_, 2.0f * quant_);
      pos_ += 0.08f * residual;  // было 0.25 — слишком резко

      last_meas_ = raw;
      t_last_ = t;
    }

    // Нормализуем pos_ в [0, 2π) перед возвратом
    pos_ = _normalizeAngle(pos_);

    Sensor::update();  // вызовет getSensorAngle() → вернёт pos_
  }

  void init() override {
    //_wrapped.init();  // на всякий случай
    Sensor::init();
    t_prev_ = _micros() * 1e-6f;
    initialized_ =
        false;  // сброс, чтобы первый update() правильно инициализировал
  }

  void setCommandVelocity(float v) { cmd_vel_ = v; }

  float position() const { return pos_; }
  float velocity() const { return vel_; }

 protected:
  float getSensorAngle() override {
    return pos_;  // теперь гарантированно в [0, 2π)
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