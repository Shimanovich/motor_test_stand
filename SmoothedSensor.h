#ifndef __SMOOTHEDSENSOR_H__
#define __SMOOTHEDSENSOR_H__

#include "common/base_classes/Sensor.h"
#include "common/foc_utils.h"

/**
 * Обёртка над Sensor, выполняющая сглаживание угла.
 * Архитектура полностью аналогична CalibratedSensor.
 *
 * Использование:
 *   MagneticSensorMT6701SSI sensor(...);
 *   SmoothedSensor smoothed(sensor);
 *   motor.linkSensor(&smoothed);
 */
class SmoothedSensor : public Sensor {
 public:
  /**
   * @param wrapped  — исходный датчик (энкодер)
   */
  explicit SmoothedSensor(Sensor& wrapped) : _wrapped(wrapped) {}

  ~SmoothedSensor() = default;

  // Обязательно вызываем update у обёрнутого датчика
  virtual void update() override {
    _wrapped.update();
    Sensor::update();  // обновляем внутреннее состояние Sensor
  }

  virtual void init() override {
    // Предполагается, что _wrapped уже инициализирован
    Sensor::init();
  }

 protected:
  /**
   * Главный метод — здесь ты пишешь свою логику сглаживания.
   * Должен возвращать угол в радианах в диапазоне [0 … 2π).
   */
  virtual float getSensorAngle() override {
    // 1. Получаем сырой угол от реального энкодера
    float raw = _wrapped.getMechanicalAngle();

    // -------------------------------------------------
    //  ↓↓↓  МЕСТО ДЛЯ ТВОЕГО КОДА СГЛАЖИВАНИЯ  ↓↓↓
    // -------------------------------------------------

    // Пример (замени на свой фильтр):
    // static float filtered = 0.0f;
    // const float alpha = 0.1f;          // коэффициент сглаживания
    // filtered = alpha * raw + (1.0f - alpha) * filtered;
    // return filtered;

    // Пока просто возвращаем сырой угол
    return raw;

    // -------------------------------------------------
    //  ↑↑↑  КОНЕЦ ПОЛЬЗОВАТЕЛЬСКОГО КОДА  ↑↑↑
    // -------------------------------------------------
  }

  Sensor& _wrapped;  // ссылка на исходный датчик
};

#endif