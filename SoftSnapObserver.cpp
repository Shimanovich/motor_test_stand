// SoftSnapObserver.cpp
#include "SoftSnapObserver.h"

#include <Arduino.h>

#include <cmath>

SoftSnapObserver::SoftSnapObserver(float tau_slow, float tau_fast) {
  setTau(tau_slow, tau_fast);
}

void SoftSnapObserver::setTau(float tau_slow, float tau_fast) {
  tau_slow_ = max(tau_slow, 0.01f);
  tau_fast_ = max(tau_fast, 0.005f);
  tau_ = tau_slow_;
}

void SoftSnapObserver::setQuantization(float quant_rad) {
  quant_ = quant_rad > 1e-6f ? quant_rad : 0.0004f;
}

void SoftSnapObserver::reset(float angle, float velocity) {
  pos_ = target_pos_ = last_meas_ = angle;
  vel_ = velocity;
  t_last_ = 0.0f;
  tau_ = tau_slow_;
  initialized_ = true;
}

void SoftSnapObserver::predict(float dt, float cmd_vel) {
  if (!initialized_ || dt <= 0.0f) return;

  // Чем выше скорость — тем больше доверяем оценке vel_
  float speed = fabsf(cmd_vel);
  float trust_cmd = constrain(1.0f - speed / 1.0f, 0.3f, 0.95f);  // до 1 рад/с

  float v = trust_cmd * cmd_vel + (1.0f - trust_cmd) * vel_;
  pos_ += v * dt;

  // Мягко тянем vel_ к команде
  vel_ = 0.97f * vel_ + 0.03f * cmd_vel;
}

void SoftSnapObserver::update(float measured_angle, float t) {
  if (!initialized_) {
    pos_ = target_pos_ = last_meas_ = measured_angle;
    vel_ = 0.0f;
    t_last_ = t;
    initialized_ = true;
    return;
  }

  float delta = measured_angle - last_meas_;
  if (fabsf(delta) < 0.5f * quant_) return;

  // === Новое измерение ===
  float dt_jump = t - t_last_;
  if (dt_jump < 1e-4f) dt_jump = CONTROL_DT;

  // 1. Оценка скорости по скачку
  float vel_jump = delta / dt_jump;

  // 2. Адаптивный tau: чем чаще приходят отсчёты — тем быстрее коррекция
  //    dt_jump = 0.5 с → tau ≈ tau_slow
  //    dt_jump = 0.02 с → tau ≈ tau_fast
  float ratio = constrain(dt_jump / 0.3f, 0.0f, 1.0f);
  tau_ = tau_fast_ + ratio * (tau_slow_ - tau_fast_);

  // 3. Обновляем скорость (сильнее, когда скачки частые)
  float blend = constrain(0.15f / dt_jump, 0.05f, 0.4f);
  vel_ = (1.0f - blend) * vel_ + blend * vel_jump;

  // 4. Новая цель для soft-snap
  target_pos_ = measured_angle;
  last_meas_ = measured_angle;
  t_last_ = t;
}

void SoftSnapObserver::smooth(float dt) {
  if (!initialized_ || dt <= 0.0f) return;

  float alpha = dt / (tau_ + dt);
  pos_ += alpha * (target_pos_ - pos_);
}