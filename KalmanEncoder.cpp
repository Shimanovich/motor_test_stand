// KalmanEncoder.cpp
#include "KalmanEncoder.h"

#include <Arduino.h>
#include <cmath>

KalmanEncoder::KalmanEncoder(float q_pos, float q_vel, float r)
    : q_pos_(q_pos),
      q_vel_(q_vel),
      r_(r),
      last_meas_(0.0f),
      quant_(0.0004f),
      initialized_(false) {
  x_[0] = x_[1] = 0.0f;
  P_[0][0] = 1.0f;
  P_[0][1] = 0.0f;
  P_[1][0] = 0.0f;
  P_[1][1] = 1.0f;
}

void KalmanEncoder::setNoise(float q_pos, float q_vel, float r) {
  q_pos_ = max(q_pos, 0.0f);
  q_vel_ = max(q_vel, 0.0f);
  r_ = max(r, 1e-12f);
}

void KalmanEncoder::setQuantization(float quant_rad) {
  quant_ = quant_rad > 1e-6f ? quant_rad : 0.0004f;
}

void KalmanEncoder::reset(float angle, float velocity) {
  x_[0] = angle;
  x_[1] = velocity;
  last_meas_ = angle;

  // Large initial uncertainty so first measurements pull strongly
  P_[0][0] = 1.0f;
  P_[0][1] = 0.0f;
  P_[1][0] = 0.0f;
  P_[1][1] = 1.0f;

  initialized_ = true;
}

void KalmanEncoder::predict(float dt, float cmd_vel) {
  if (!initialized_ || dt <= 0.0f) return;

  // Optional mild pull of velocity toward command (helps at ultra-low speed)
  // Keep small so filter remains primarily measurement-driven at higher speeds.
  const float cmd_blend = 0.02f;
  x_[1] = (1.0f - cmd_blend) * x_[1] + cmd_blend * cmd_vel;

  // State transition: constant velocity
  // x = [1 dt; 0 1] * x
  x_[0] += x_[1] * dt;

  // Covariance prediction
  // P = F P F' + Q
  // F = [1 dt; 0 1]
  float p00 = P_[0][0];
  float p01 = P_[0][1];
  float p11 = P_[1][1];

  P_[0][0] = p00 + dt * (p01 + p01 + dt * p11) + q_pos_ * dt;
  P_[0][1] = p01 + dt * p11;
  P_[1][0] = P_[0][1];
  P_[1][1] = p11 + q_vel_ * dt;
}

void KalmanEncoder::update(float measured_angle) {
  if (!initialized_) {
    reset(measured_angle, 0.0f);
    return;
  }

  // Quantization gate: ignore repeated identical readings
  float delta = measured_angle - last_meas_;
  if (fabsf(delta) < 0.5f * quant_) {
    return;
  }
  last_meas_ = measured_angle;

  // Innovation
  float y = measured_angle - x_[0];

  // Innovation covariance S = H P H' + R, H = [1 0]
  float S = P_[0][0] + r_;
  if (S < 1e-12f) S = 1e-12f;

  // Kalman gain K = P H' / S
  float K0 = P_[0][0] / S;
  float K1 = P_[1][0] / S;

  // State update
  x_[0] += K0 * y;
  x_[1] += K1 * y;

  // Covariance update (Joseph form simplified for H=[1 0])
  // P = (I - K H) P
  float p00 = P_[0][0];
  float p01 = P_[0][1];
  float p11 = P_[1][1];

  P_[0][0] = (1.0f - K0) * p00;
  P_[0][1] = (1.0f - K0) * p01;
  P_[1][0] = P_[0][1];
  P_[1][1] = p11 - K1 * p01;

  // Keep covariance symmetric and positive-ish
  if (P_[0][0] < 0.0f) P_[0][0] = 0.0f;
  if (P_[1][1] < 0.0f) P_[1][1] = 0.0f;
}
