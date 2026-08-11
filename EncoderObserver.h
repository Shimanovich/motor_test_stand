// EncoderObserver.h
#pragma once
#include "config.h"

class EncoderObserver {
 public:
  explicit EncoderObserver(float alpha = 0.15f, float beta = 0.008f);

  void predict(float dt, float cmd_vel);
  void update(float measured_angle,
              float t);  // каждый цикл (внутри сам решит)

  float position() const { return pos_; }
  float velocity() const { return vel_; }

  void setGains(float alpha, float beta);
  void setQuantization(float quant_rad);  // 2π / CPR энкодера
  void reset(float angle = 0.0f, float velocity = 0.0f);

  float& alpha() { return alpha_; }
  float& beta() { return beta_; }

 private:
  float pos_ = 0.0f;
  float vel_ = 0.0f;
  float alpha_ = 0.15f;
  float beta_ = 0.008f;

  float last_meas_ = 0.0f;      // последнее принятое измерение
  float t_last_change_ = 0.0f;  // время последнего изменения
  float quant_ = 0.0004f;       // ≈ 2π/16384 для 14-bit
  bool initialized_ = false;
};