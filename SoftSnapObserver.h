// SoftSnapObserver.h
#pragma once

#include "config.h"

/**
 * SoftSnapObserver — предсказание положения + мягкая коррекция
 * при появлении нового отсчёта энкодера.
 *
 * Единственный параметр настройки: tau (постоянная времени, сек).
 * Типичные значения: 0.15 … 0.40 с.
 */
class SoftSnapObserver {
 public:
  explicit SoftSnapObserver(float tau = 0.25f);

  // Предсказание (вызывать каждый цикл управления)
  void predict(float dt, float cmd_vel);

  // Новое измерение энкодера (вызывать каждый цикл)
  // Внутри сам определяет, изменилось ли значение
  void update(float measured_angle);

  // Плавная подтяжка к целевому положению (вызывать каждый цикл)
  void smooth(float dt);

  float position() const { return pos_; }
  float velocity() const { return vel_; }

  void reset(float angle = 0.0f, float velocity = 0.0f);
  void setTau(float tau);
  void setQuantization(float quant_rad);

  // Для Commander
  float& tau() { return tau_; }

 private:
  float pos_ = 0.0f;         // текущая оценка положения
  float target_pos_ = 0.0f;  // куда мягко тянем
  float vel_ = 0.0f;         // оценка скорости
  float last_meas_ = 0.0f;   // последнее принятое измерение

  float tau_ = 0.25f;      // постоянная времени коррекции [с]
  float quant_ = 0.0004f;  // квант энкодера (≈ 2π/16384)

  bool initialized_ = false;
};