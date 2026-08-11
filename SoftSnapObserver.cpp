// SoftSnapObserver.cpp
#include "SoftSnapObserver.h"

#include <Arduino.h>

#include <cmath>

SoftSnapObserver::SoftSnapObserver(float tau) { setTau(tau); }

void SoftSnapObserver::setTau(float tau) {
  tau_ = (tau < 0.001f) ? 0.001f : tau;
}

void SoftSnapObserver::setQuantization(float quant_rad) {
  quant_ = (quant_rad > 1e-6f) ? quant_rad : 0.0004f;
}

void SoftSnapObserver::reset(float angle, float velocity) {
  pos_ = angle;
  target_pos_ = angle;
  last_meas_ = angle;
  vel_ = velocity;
  initialized_ = true;
}

void SoftSnapObserver::predict(float dt, float cmd_vel) {
  if (!initialized_ || dt <= 0.0f) return;

  // Чистая модель + лёгкое влияние текущей оценки скорости
  float v = 0.90f * cmd_vel + 0.10f * vel_;
  pos_ += v * dt;

  // Скорость тоже слегка тянем к команде (очень мягко)
  vel_ = 0.98f * vel_ + 0.02f * cmd_vel;
}

void SoftSnapObserver::update(float measured_angle) {
  if (!initialized_) {
    pos_ = measured_angle;
    target_pos_ = measured_angle;
    last_meas_ = measured_angle;
    vel_ = 0.0f;
    initialized_ = true;
    return;
  }

  float delta = measured_angle - last_meas_;

  // Игнорируем, если значение не изменилось (квантование)
  if (fabsf(delta) < 0.5f * quant_) {
    return;
  }

  // Новое измерение → ставим новую цель для мягкой коррекции
  target_pos_ = measured_angle;
  last_meas_ = measured_angle;

  // Грубая оценка скорости по скачку (будет сглажена в predict)
  // dt здесь неизвестен, поэтому просто запоминаем факт изменения
  // Реальную скорость лучше брать из cmd_vel на сверхнизких оборотах
}

void SoftSnapObserver::smooth(float dt) {
  if (!initialized_ || dt <= 0.0f) return;

  // Экспоненциальная подтяжка к target_pos_ с постоянной времени tau_
  // alpha = dt / (tau + dt)  — эквивалент дискретного фильтра 1-го порядка
  float alpha = dt / (tau_ + dt);
  pos_ += alpha * (target_pos_ - pos_);
}