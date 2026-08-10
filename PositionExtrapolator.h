class PositionExtrapolator {
 public:
  // Инициализация. T_corr — время коррекции (обычно 0.5…1.0 с)
  explicit PositionExtrapolator(double T_corr);

  // Добавить новое измерение (смещение s в момент времени t)
  // Вызывать при каждом новом отсчёте GPS (после притяжки к маршруту)
  void addMeasurement(double s, double t);

  // Предсказать смещение в произвольный момент time
  // (time должен быть >= времени последнего измерения)
  double predict(double time) const;

  // Дополнительно: предсказать скорость (полезно для отладки)
  double predictVelocity(double time) const;

  void setTcorr(double t);

 private:
  double T_;  // время коррекции

  // Параметры текущего сегмента
  double t_i_ = 0.0;  // время начала текущего сегмента
  double s_i_ = 0.0;  // положение в t_i_
  double v_i_ = 0.0;  // скорость
  double a_i_ = 0.0;  // ускорение

  // Коэффициенты кубической коррекции
  double C3_ = 0.0, C2_ = 0.0, C1_ = 0.0, C0_ = 0.0;

  // История для оценки скорости и ускорения
  double prev_s_ = 0.0;
  double prev_t_ = 0.0;
  double prev2_s_ = 0.0;
  double prev2_t_ = 0.0;
  int measure_count_ = 0;

  // Вспомогательные методы
  void computeCoefficients(double e_s, double e_v);
  double evalPosition(double dt) const;
  double evalVelocity(double dt) const;
};