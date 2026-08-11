// SoftSnapObserver.h
#pragma once
#include "config.h"

class SoftSnapObserver {
 public:
  /**
   * @param tau_slow  постоянная времени на сверхнизких скоростях (0.2…0.4 с)
   * @param tau_fast  постоянная времени на более высоких скоростях (0.03…0.08
   * с)
   */
  explicit SoftSnapObserver(float tau_slow = 0.25f, float tau_fast = 0.05f);

  void predict(float dt, float cmd_vel);
  void update(float measured_angle, float t);  // теперь передаём время
  void smooth(float dt);

  float position() const { return pos_; }
  float velocity() const { return vel_; }

  void reset(float angle = 0.0f, float velocity = 0.0f);
  void setTau(float tau_slow, float tau_fast = 0.05f);
  void setQuantization(float quant_rad);

  float& tauSlow() { return tau_slow_; }
  float& tauFast() { return tau_fast_; }

 private:
  float pos_ = 0.0f;
  float target_pos_ = 0.0f;
  float vel_ = 0.0f;
  float last_meas_ = 0.0f;
  float t_last_ = 0.0f;

  float tau_slow_ = 0.25f;
  float tau_fast_ = 0.05f;
  float tau_ = 0.25f;  // текущее рабочее значение
  float quant_ = 0.0004f;

  bool initialized_ = false;
};