#include <PositionExtrapolator.h>

#include <cmath>

PositionExtrapolator::PositionExtrapolator(double T_corr) : T_(T_corr) {
  if (T_ <= 0.0) T_ = 0.7;  // защита
}
void PositionExtrapolator::setTcorr(double t) { T_ = t; }

void PositionExtrapolator::addMeasurement(double s, double t) {
  if (measure_count_ == 0) {
    // Первое измерение
    t_i_ = t;
    s_i_ = s;
    v_i_ = 0.0;
    a_i_ = 0.0;
    C3_ = C2_ = C1_ = C0_ = 0.0;

    prev_s_ = s;
    prev_t_ = t;
    measure_count_ = 1;
    return;
  }

  // === 1. Считаем ошибку от предыдущего прогноза ===
  double dt_pred = t - t_i_;
  double s_pred = evalPosition(dt_pred);
  double v_pred = evalVelocity(dt_pred);

  double e_s = s_pred - s;  // ошибка положения

  // === 2. Оценка новой скорости и ускорения ===
  double v_new = 0.0;
  double a_new = 0.0;

  if (measure_count_ >= 1) {
    double dt = t - prev_t_;
    if (dt > 1e-6) v_new = (s - prev_s_) / dt;
  }

  if (measure_count_ >= 2) {
    double dt1 = prev_t_ - prev2_t_;
    double dt2 = t - prev_t_;
    if (dt1 > 1e-6 && dt2 > 1e-6) {
      double v_prev = (prev_s_ - prev2_s_) / dt1;
      a_new = (v_new - v_prev) / dt2;
    }
  }

  double e_v = v_pred - v_new;  // ошибка скорости

  // === 3. Запускаем новый сегмент ===
  t_i_ = t;
  s_i_ = s;
  v_i_ = v_new;
  a_i_ = a_new;

  computeCoefficients(e_s, e_v);

  // Обновляем историю
  prev2_s_ = prev_s_;
  prev2_t_ = prev_t_;
  prev_s_ = s;
  prev_t_ = t;
  ++measure_count_;
}

double PositionExtrapolator::predict(double time) const {
  if (measure_count_ == 0) return 0.0;

  double dt = time - t_i_;
  if (dt < 0.0) dt = 0.0;  // не возвращаемся в прошлое

  return evalPosition(dt);
}

double PositionExtrapolator::predictVelocity(double time) const {
  if (measure_count_ == 0) return 0.0;

  double dt = time - t_i_;
  if (dt < 0.0) dt = 0.0;

  return evalVelocity(dt);
}

// ----------------- private -----------------

void PositionExtrapolator::computeCoefficients(double e_s, double e_v) {
  // Правильные коэффициенты (исправленные относительно статьи)
  const double T = T_;
  const double T2 = T * T;
  const double T3 = T2 * T;

  C3_ = (e_v * T + 2.0 * e_s) / T3;
  C2_ = -(2.0 * e_v * T + 3.0 * e_s) / T2;
  C1_ = e_v;
  C0_ = e_s;
}

double PositionExtrapolator::evalPosition(double dt) const {
  // Квадратичная часть
  double s = s_i_ + v_i_ * dt + 0.5 * a_i_ * dt * dt;

  // Кубическая коррекция (только на интервале [0, T])
  if (dt <= T_) {
    s += C3_ * dt * dt * dt + C2_ * dt * dt + C1_ * dt + C0_;
  }
  // после T коррекция = 0

  return s;
}

double PositionExtrapolator::evalVelocity(double dt) const {
  // Производная квадратичной части
  double v = v_i_ + a_i_ * dt;

  // Производная коррекции
  if (dt <= T_) {
    v += 3.0 * C3_ * dt * dt + 2.0 * C2_ * dt + C1_;
  }

  return v;
}