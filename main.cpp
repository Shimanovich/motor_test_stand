#include <SimpleFOC.h>
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>

#include "VelocityDobController.h"
#include "config.h"
#include "encoders/calibrated/CalibratedSensor.h"

MagneticSensorMT6701SSI sensor = MagneticSensorMT6701SSI(SENSOR_CS_PIN);

CalibratedSensor sensor_calibrated = CalibratedSensor(sensor, 100);

BLDCMotor motor = BLDCMotor(MOTOR_PP, MOTOR_R, MOTOR_KV, MOTOR_L);
BLDCDriver3PWM driver =
    BLDCDriver3PWM(DRIVER_PWM_A, DRIVER_PWM_B, DRIVER_PWM_C, DRIVER_EN);

volatile float target_velocity = 0.0f;

#define configTICK_RATE_HZ ((TickType_t)10000)

int powerOn = 0;

// Экземпляр регулятора
VelocityDobController controller;                     // сам объект
VelocityDobController* controller_ptr = &controller;  // указатель для  обёртки

float customMotionControlWrapper(FOCMotor* motor) {
  return (*controller_ptr)(motor);  // вызываем operator()
}

// Commander
Commander command = Commander(Serial1);
void doTarget(char* cmd) { command.scalar((float*)&target_velocity, cmd); }
void doKp(char* cmd) { command.scalar(&controller.kp(), cmd); }
void doKi(char* cmd) { command.scalar(&controller.ki(), cmd); }
void doBn(char* cmd) { command.scalar(&controller.bn(), cmd); }
void doG(char* cmd) { command.scalar(&controller.g(), cmd); }
void dottf(char* cmd) { command.scalar(&motor.LPF_velocity.Tf, cmd); }

void doPower(char* cmd) {
  float in;
  command.scalar(&in, cmd);
  powerOn = (int)in;
}

// FreeRTOS задача 1 кГц
void motorControlTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 1;

  for (;;) {
    if (powerOn) {
      motor.loopFOC();
      motor.move(target_velocity);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup() {
  Serial.begin(230400);
  Serial1.begin(230400, SERIAL_8N1, CUSTOM_RX_PIN, CUSTOM_TX_PIN);
  SimpleFOCDebug::enable(&Serial);

  // Железо
  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = POWER_SUPPLY;
  driver.init();
  motor.linkDriver(&driver);

  // Регулятор
  controller.setPID(0.1f, 1.0f);
  controller.setDob(0.0f, 0.0f);
  controller.setFilters(0.018f, 12.0f, 0.10f, 0.35f);
  controller.setVoltageLimit(VOLTAGE_LIMIT);

  motor.linkCustomMotionControl(customMotionControlWrapper);
  motor.controller = MotionControlType::custom;
  motor.torque_controller = TorqueControlType::voltage;
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.voltage_limit = VOLTAGE_LIMIT;

  motor.useMonitoring(Serial);

  motor.init();
  sensor_calibrated.calibrate(motor);
  motor.linkSensor(&sensor_calibrated);

  motor.initFOC();

  // Команды
  command.add('T', doTarget, "target velocity");
  command.add('P', doKp, "Kp");
  command.add('I', doKi, "Ki");
  command.add('B', doBn, "bn");
  command.add('G', doG, "DOB g");
  command.add('W', doPower, "ON");
  command.add('F', dottf, "filter_set");

  command.verbose = VerboseMode::nothing;

  Serial.println(F("PID + DOB modular version ready"));
  _delay(1000);

  xTaskCreatePinnedToCore(motorControlTask, "MotorCtrl", 4096, NULL, 5, NULL,
                          0);

  // powerOn = 1.0;
  // target_velocity = 0.0f;
  // delay(500);
  // float angle = 0.0;

  // for (float i = 0.0f; i < 2.0 * PI; i += (2.0 * PI) / 1000.0) {
  //   target_velocity = i;
  //   delay(10);
  //   angle = motor.shaftAngle() + motor.sensor_offset;

  //   Serial1.printf("%.3f,%.3f\n", target_velocity, angle);
  // }
}

void loop() {
  command.run();
  Serial1.printf("%.3f,%.3f\n", target_velocity, motor.shaft_velocity);
}

// /**
//  * Velocity LADRC control @ 1 kHz
//  * Улучшенная версия для работы на очень низких скоростях (гироплатформа)
//  *
//  * Основные улучшения:
//  * - Фильтр скорости (LPF)
//  * - Хранение previous u
//  * - Мёртвая зона + мягкий переход через ноль
//  * - Rate-limiter целевой скорости
//  * - Мягкое затухание оценки возмущения около нуля
//  */

// #include <SimpleFOC.h>
// #include <encoders/mt6701/MagneticSensorMT6701SSI.h>

// #define CUSTOM_RX_PIN 16
// #define CUSTOM_TX_PIN 17

// MagneticSensorMT6701SSI sensor = MagneticSensorMT6701SSI(5);

// // BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(7, 2.3, 220, 0.00086);
// BLDCDriver3PWM driver = BLDCDriver3PWM(12, 13, 14, 27);

// // velocity set point variable (volatile — доступ из двух задач)
// volatile float target_velocity = 0;

// // ===================== LADRC параметры =====================
// const float h = 0.001f;  // период 1 мс

// // Настраиваемые параметры (для низких скоростей начинайте с этих значений)
// float omega_c = 0.0f;  // полоса контроллера [рад/с]  (8…18)
// float omega_o = 0.0f;  // полоса наблюдателя          (4–6 × omega_c)
// float b0 = 0.0f;       // оценка усиления (критично!)

// // Внутренние состояния ESO
// float z1 = 0.0f;      // оценка скорости
// float z2 = 0.0f;      // оценка суммарного возмущения
// float last_u = 0.0f;  // предыдущее управление (после ограничения)

// int powerOn = 0;

// // Коэффициенты (пересчитываются при изменении omega)
// float kp, beta1, beta2;

// void updateLADRCGains() {
//   kp = omega_c;
//   beta1 = 2.0f * omega_o;
//   beta2 = omega_o * omega_o;
// }

// // ===================== Фильтры и вспомогательные функции
// =====================

// // Фильтр измеренной скорости (очень важен на низких оборотах)
// LowPassFilter LPF_vel =
//     LowPassFilter(0.02f);  // Tf = 20 мс (пробовать 0.015…0.03)

// // Rate-limiter целевой скорости (защита от скачков гироскопа)
// float target_filtered = 0.0f;
// float max_accel = 12.0f;  // рад/с²  (подбирать 8…25)

// float rateLimit(float new_target) {
//   float max_delta = max_accel * h;
//   float delta = new_target - target_filtered;
//   if (delta > max_delta) delta = max_delta;
//   if (delta < -max_delta) delta = -max_delta;
//   target_filtered += delta;
//   return target_filtered;
// }

// // Мёртвая зона + мягкий переход через ноль
// float deadzone = 0.12f;   // рад/с
// float soft_zone = 0.40f;  // рад/с

// float processTarget(float r) {
//   float abs_r = fabsf(r);
//   if (abs_r < deadzone) return 0.0f;

//   if (abs_r < soft_zone) {
//     float k = (abs_r - deadzone) / (soft_zone - deadzone);
//     return r * k * k;  // квадратичное сглаживание
//   }
//   return r;
// }

// // ===================== Commander =====================
// Commander command = Commander(Serial1);
// void doTarget(char* cmd) { command.scalar((float*)&target_velocity, cmd); }
// void doOmegaC(char* cmd) {
//   command.scalar(&omega_c, cmd);
//   updateLADRCGains();
// }
// void doOmegaO(char* cmd) {
//   command.scalar(&omega_o, cmd);
//   updateLADRCGains();
// }
// void doB0(char* cmd) { command.scalar(&b0, cmd); }
// void dotf(char* cmd) { command.scalar(&LPF_vel.Tf, cmd); }

// void doPower(char* cmd) {
//   float in;
//   command.scalar(&in, cmd);
//   powerOn = (int)in;
// }

// // ===================== Задача управления двигателем 1 кГц
// // =====================
// void motorControlTask(void* pvParameters) {
//   const TickType_t xFrequency = 1;  // 1 тик = 1 мс
//   TickType_t xLastWakeTime = xTaskGetTickCount();

//   for (;;) {
//     if (powerOn) {
//       motor.loopFOC();
//       motor.move(target_velocity);
//     }
//     vTaskDelayUntil(&xLastWakeTime, xFrequency);
//   }
// }

// // ===================== Custom motion control (LADRC) =====================
// float ladrcVelocityControl(FOCMotor* motor) {
//   // 1. Фильтрация измеренной скорости
//   float y = LPF_vel(motor->shaft_velocity);

//   // 2. Обработка целевой скорости
//   float r_raw = motor->target;
//   float r = processTarget(rateLimit(r_raw));

//   // 3. LESO
//   float e = z1 - y;
//   z1 += h * (z2 - beta1 * e + b0 * last_u);
//   z2 += h * (-beta2 * e);

//   // 4. Мягкое затухание оценки возмущения около нуля
//   if (fabsf(r) < 0.35f && fabsf(y) < 0.35f) {
//     z2 *= 0.992f;
//   }

//   // 5. Закон управления
//   float u0 = kp * (r - z1);
//   float u = (u0 - z2) / b0;

//   // 6. Ограничение
//   float max_u = motor->voltage_limit;
//   if (u > max_u) u = max_u;
//   if (u < -max_u) u = -max_u;

//   last_u = u;
//   return u;
// }

// void setup() {
//   Serial.begin(115200);
//   Serial1.begin(115200, SERIAL_8N1, CUSTOM_RX_PIN, CUSTOM_TX_PIN);

//   SimpleFOCDebug::enable(&Serial);

//   // Sensor
//   sensor.init();
//   motor.linkSensor(&sensor);

//   // Driver
//   driver.voltage_power_supply = 12;
//   driver.init();
//   motor.linkDriver(&driver);

//   // ----- Custom LADRC -----
//   motor.linkCustomMotionControl(ladrcVelocityControl);
//   // motor.controller = MotionControlType::custom;
//   motor.controller = MotionControlType::velocity_openloop;
//   motor.torque_controller = TorqueControlType::voltage;
//   motor.foc_modulation = FOCModulationType::SinePWM;

//   motor.voltage_limit = 6.0f;  // можно уменьшить до 4–5 на гироплатформе

//   motor.useMonitoring(Serial);

//   // Initialize
//   motor.init();
//   motor.initFOC();

//   updateLADRCGains();

//   // Commands
//   command.add('T', doTarget, "target velocity");
//   command.add('C', doOmegaC, "omega_c");
//   command.add('O', doOmegaO, "omega_o");
//   command.add('B', doB0, "b0");
//   command.add('F', dotf, "TF");
//   command.add('P', doPower, "ON");
//   command.verbose = VerboseMode::nothing;

//   Serial.println(F("LADRC low-speed version ready @ 1 kHz"));
//   Serial.println(F("Commands: T, C, O, B"));
//   _delay(1000);

//   // FreeRTOS task
//   xTaskCreatePinnedToCore(motorControlTask, "MotorCtrl", 4096, NULL, 5, NULL,
//                           1);
// }

// void loop() {
//   command.run();

//   // Отладочный вывод (можно закомментировать)
//   // Serial1.print("vel_f:");
//   float filtred = LPF_vel(motor.shaft_velocity);
//   Serial1.print(filtred, 2);
//   Serial1.print(",");

//   // Serial1.print("vel_o:");
//   Serial1.print(motor.shaft_velocity, 2);
//   //  Serial1.print(",");

//   // Serial1.print("angle:");
//   // Serial1.print(motor.shaft_angle, 2);
//   Serial1.println("");
// }