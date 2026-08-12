#include <cmath>
#include <stdexcept>

class NewLowPassFilter {
 public:
  explicit NewLowPassFilter(float time_constant)
      : tau_(time_constant),
        y_(0.0),
        x_last_(0.0),
        t_last_(0.0),
        initialized_(false) {
    if (tau_ <= 0.0) {
      throw std::invalid_argument("time_constant must be > 0");
    }
  }

  /*
   * set new integration time constant
   */
  float* getTauPtr(void) { return &tau_; }

  // Обновление новым измерением
  void update(float measurement, float timestamp) {
    if (!initialized_) {
      y_ = measurement;
      x_last_ = measurement;
      t_last_ = timestamp;
      initialized_ = true;
      return;
    }

    if (timestamp < t_last_) {
      throw std::runtime_error("timestamps must be non-decreasing");
    }

    // Сначала продвигаем состояние до момента нового измерения
    // (считая, что предыдущий вход x_last_ держался)
    propagate(timestamp);

    // Теперь принимаем новое измерение
    x_last_ = measurement;
    // y_ уже находится в момент timestamp, дальше обычное обновление не
    // требуется, потому что propagate уже учёл dt
  }

  // Получить сглаженное значение на произвольный момент времени
  // (обычно >= последнего update)
  float valueAt(float time) const {
    if (!initialized_) {
      throw std::runtime_error("filter is not initialized");
    }
    if (time < t_last_) {
      throw std::runtime_error("query time is in the past");
    }

    // Аналитическое решение при постоянном входе x_last_
    const float dt = time - t_last_;
    const float alpha = std::exp(-dt / tau_);
    return x_last_ + (y_ - x_last_) * alpha;
  }

  // Удобный комбинированный метод: обновить и сразу получить значение на нужный
  // момент
  float calculate(float measurement, float measurement_time, float query_time) {
    update(measurement, measurement_time);
    return valueAt(query_time);
  }

  void reset() {
    initialized_ = false;
    y_ = 0.0;
    x_last_ = 0.0;
    t_last_ = 0.0;
  }

  float timeConstant() const { return tau_; }
  bool isInitialized() const { return initialized_; }

 private:
  void propagate(float time) {
    if (time <= t_last_) return;

    const float dt = time - t_last_;
    const float alpha = std::exp(-dt / tau_);
    y_ = x_last_ + (y_ - x_last_) * alpha;
    t_last_ = time;
  }

  float tau_;
  float y_;       // текущее состояние фильтра
  float x_last_;  // последнее принятое измерение
  float t_last_;  // время последнего состояния
  bool initialized_;
};