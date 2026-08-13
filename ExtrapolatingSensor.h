#pragma once
#include "Arduino.h"
#include "common/base_classes/Sensor.h"
#include "common/foc_utils.h"

/**
 * Экстраполирующий датчик для сверхнизких скоростей.
 * Учитывает, что энкодер может долго отдавать одно и то же значение.
 */
class ExtrapolatingSensor : public Sensor {
 public:
  explicit ExtrapolatingSensor(Sensor& wrapped, float quant = 0.0003835f)
      : _wrapped(wrapped), quant_(quant) {}

  void update() override {
    _wrapped.update();
    Sensor::update();
  }

  void init() override { Sensor::init(); }

  // Можно вызывать из custom motion control
  void predict(float dt, float cmd_vel = 0.0f) {
    if (!initialized_ || dt <= 0.0f) return;

    // Лёгкое притяжение к команде на сверхнизких скоростях
    const float blend = 0.03f;
    vel_ = (1.0f - blend) * vel_ + blend * cmd_vel;

    pos_ += vel_ * dt;
  }

 protected:
  float getSensorAngle() override {
    float raw = _wrapped.getMechanicalAngle();
    float t = _micros() * 1e-6f;

    if (!initialized_) {
      pos_ = raw;
      last_meas_ = raw;
      t_last_ = t;
      vel_ = 0.0f;
      initialized_ = true;
      return pos_;
    }

    float delta = raw - last_meas_;

    // Ключевой момент: игнорируем повторные одинаковые отсчёты
    if (fabsf(delta) < 0.5f * quant_) {
      // Просто экстраполируем
      return pos_;
    }

    // === Пришёл новый LSB ===
    float dt_jump = t - t_last_;
    if (dt_jump < 1e-4f) dt_jump = 0.001f;

    // Оценка скорости по времени между скачками — главное на малых скоростях
    float vel_jump = delta / dt_jump;

    // Мягкая коррекция скорости
    float alpha_v = constrain(0.12f / dt_jump, 0.02f, 0.35f);
    vel_ = (1.0f - alpha_v) * vel_ + alpha_v * vel_jump;

    // Мягкая коррекция положения (не прыгаем на полный residual)
    float residual = raw - pos_;
    residual = constrain(residual, -3.0f * quant_, 3.0f * quant_);
    pos_ += 0.25f * residual;  // можно сделать ещё слабее

    last_meas_ = raw;
    t_last_ = t;

    return pos_;
  }

 private:
  Sensor& _wrapped;
  float quant_;
  float pos_ = 0.0f;
  float vel_ = 0.0f;
  float last_meas_ = 0.0f;
  float t_last_ = 0.0f;
  bool initialized_ = false;
};