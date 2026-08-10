// EncoderObserver.cpp
#include "EncoderObserver.h"
#include <Arduino.h>
#include <cmath>

EncoderObserver::EncoderObserver(float alpha, float beta) {
  setGains(alpha, beta);
}

void EncoderObserver::setGains(float alpha, float beta) {
  alpha_ = constrain(alpha, 0.0f, 1.0f);
  beta_  = constrain(beta,  0.0f, 1.0f);
}

void EncoderObserver::setQuantization(float quant_rad) {
  quant_ = quant_rad > 1e-6f ? quant_rad : 0.0004f;
}

void EncoderObserver::reset(float angle, float velocity) {
  pos_ = angle;
  vel_ = velocity;
  last_meas_ = angle;
  t_last_change_ = 0.0f;
  initialized_ = true;
}

void EncoderObserver::predict(float dt) {
  if (!initialized_ || dt <= 0.0f) return;
  pos_ += vel_ * dt;
}

void EncoderObserver::update(float measured_angle, float t) {
  if (!initialized_) {
    pos_ = measured_angle;
    vel_ = 0.0f;
    last_meas_ = measured_angle;
    t_last_change_ = t;
    initialized_ = true;
    return;
  }

  // Всегда предсказываем
  // (dt можно передавать снаружи или считать здесь)
  // predict уже вызван в stage2MotionControl

  float delta = measured_angle - last_meas_;

  // Считаем, что измерение «новое», только если изменилось
  // заметно больше половины LSB (защита от шума)
  if (fabsf(delta) < 0.5f * quant_) {
    // то же значение — ничего не корректируем
    return;
  }

  // === Новое измерение ===
  float dt_change = t - t_last_change_;
  if (dt_change < 1e-4f) dt_change = CONTROL_DT; // защита

  // 1. Оценка скорости по времени между скачками (главное на малых скоростях)
  float vel_from_jump = delta / dt_change;

  // 2. Residual относительно текущей оценки
  float residual = measured_angle - pos_;

  // 3. Коррекция положения
  pos_ += alpha_ * residual;

  // 4. Коррекция скорости: смесь β-фильтра и прямого измерения скачка
  //    (чем реже скачки — тем сильнее доверяем vel_from_jump)
  float trust_jump = constrain(dt_change / 0.05f, 0.0f, 1.0f); // 50 мс → полный вес
  vel_ = (1.0f - trust_jump) * (vel_ + (beta_ / CONTROL_DT) * residual)
       + trust_jump * vel_from_jump;

  // Запоминаем
  last_meas_ = measured_angle;
  t_last_change_ = t;
}