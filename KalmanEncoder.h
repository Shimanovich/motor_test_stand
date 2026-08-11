// KalmanEncoder.h
#pragma once

#include "config.h"

/**
 * 1D Constant-Velocity Kalman filter for magnetic encoder.
 *
 * State: [position, velocity]
 * - predict() every control cycle
 * - update() only when a new encoder LSB arrives (quantization-aware)
 *
 * Works from ~0.0001 rad/s up to ~1+ rad/s with proper Q/R tuning.
 */
class KalmanEncoder {
 public:
  /**
   * @param q_pos  process noise for position  (typical 1e-8 .. 1e-5)
   * @param q_vel  process noise for velocity  (typical 1e-5 .. 1e-2)
   * @param r      measurement noise           (typical 1e-7 .. 1e-4)
   */
  explicit KalmanEncoder(float q_pos = 1e-7f, float q_vel = 5e-4f,
                         float r = 2e-6f);

  /** Prediction step (call every control cycle) */
  void predict(float dt, float cmd_vel = 0.0f);

  /**
   * Measurement update.
   * Internally ignores repeated identical readings (quantization).
   */
  void update(float measured_angle);

  float position() const { return x_[0]; }
  float velocity() const { return x_[1]; }

  void reset(float angle = 0.0f, float velocity = 0.0f);
  void setNoise(float q_pos, float q_vel, float r);
  void setQuantization(float quant_rad);

  // For Commander
  float& qPos() { return q_pos_; }
  float& qVel() { return q_vel_; }
  float& r() { return r_; }

 private:
  float x_[2];        // [pos, vel]
  float P_[2][2];     // covariance

  float q_pos_;
  float q_vel_;
  float r_;

  float last_meas_;
  float quant_;
  bool initialized_;
};
