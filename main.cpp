/**
 * Плавное управление BLDC на минимальных скоростях
 * EncoderObserver (α-β) + position control через custom motion
 */

#include <SimpleFOC.h>
#include <encoders/calibrated/CalibratedSensor.h>
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>

// #include "EncoderObserver.h"
#include "SoftSnapObserver.h"
#include "config.h"
#include "luts.h"

// ===================== Железо =====================
MagneticSensorMT6701SSI sensor = MagneticSensorMT6701SSI(SENSOR_CS_PIN);
BLDCMotor motor = BLDCMotor(MOTOR_PP, MOTOR_R, MOTOR_KV, MOTOR_L);
BLDCDriver3PWM driver =
    BLDCDriver3PWM(DRIVER_PWM_A, DRIVER_PWM_B, DRIVER_PWM_C, DRIVER_EN);

// ===================== Калибровка =====================
#define MOTOR_IS_CALIBRATED 1

#if MOTOR_IS_CALIBRATED
CalibratedSensor sensor_calibrated =
    CalibratedSensor(sensor, LUTS_TOTAL, calibrationLut);
#else
CalibratedSensor sensor_calibrated = CalibratedSensor(sensor, LUTS_TOTAL);
#endif

// ===================== Управление =====================
volatile float target_velocity = 0.0f;
int powerOn = 0;

double position_setpoint = 0.0f;  // интегрированное задание
// EncoderObserver observer(0.15f, 0.008f);  // α, β

SoftSnapObserver observer(0.25f);  // tau = 0.25 с

// ===================== Commander =====================
Commander command = Commander(Serial1);

void doTarget(char* cmd) { command.scalar((float*)&target_velocity, cmd); }

void doPower(char* cmd) {
  float in;
  command.scalar(&in, cmd);
  powerOn = (int)in;
  if (!powerOn) {
    target_velocity = 0.0f;
  }
}

// PID положения
void doP(char* cmd) { command.scalar(&motor.P_angle.P, cmd); }
void doI(char* cmd) { command.scalar(&motor.P_angle.I, cmd); }
void doD(char* cmd) { command.scalar(&motor.P_angle.D, cmd); }

void doVLim(char* cmd) {
  command.scalar(&motor.voltage_limit, cmd);
  motor.PID_velocity.limit = motor.voltage_limit;
}
void doVelLim(char* cmd) { command.scalar(&motor.velocity_limit, cmd); }

// Gains наблюдателя
void doAlpha(char* cmd) { command.scalar(&observer.tau(), cmd); }
// void doBeta(char* cmd) { command.scalar(&observer.beta(), cmd); }

// ===================== FreeRTOS задача 1 кГц =====================
void motorControlTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 1;  // 1 ms

  for (;;) {
    if (powerOn) {
      motor.enable();
      position_setpoint += (double)target_velocity * CONTROL_DT;
      motor.loopFOC();
      motor.move((float)position_setpoint);
    } else {
      motor.disable();
      // motor.move((float)position_setpoint);  // держим текущее положение
      //  или motor.move(0) + observer.reset(...) — по вкусу
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ===================== Custom motion control =====================
float stage2MotionControl(FOCMotor* m) {
  static float t_prev = 0.0f;
  float t = _micros() * 1e-6f;
  float dt = t - t_prev;
  if (dt <= 0.0f || dt > 0.01f) dt = CONTROL_DT;
  t_prev = t;

  // 1. Предсказание
  observer.predict(dt, target_velocity);

  // 2. Новое измерение (если есть)
  observer.update(m->shaft_angle);

  // 3. Мягкая коррекция
  observer.smooth(dt);

  // 4. Ошибка
  float error = m->target - observer.position();

  // 5. Небольшой deadzone
  const float dead = 0.0004f;
  if (fabsf(error) < dead) error = 0.0f;

  float u = motor.P_angle(error);
  return constrain(u, -m->voltage_limit, m->voltage_limit);
}

// ===================== Setup =====================
void setup() {
  delay(100);

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, CUSTOM_RX_PIN, CUSTOM_TX_PIN);

  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = POWER_SUPPLY;
  driver.init();
  motor.linkDriver(&driver);

  motor.linkCustomMotionControl(stage2MotionControl);
  motor.controller = MotionControlType::custom;
  motor.torque_controller = TorqueControlType::voltage;
  motor.foc_modulation = FOCModulationType::SinePWM;

  // Мягкий регулятор положения
  motor.P_angle.P = 5.0f;
  motor.P_angle.I = 0.0f;
  motor.P_angle.D = 0.0f;

  motor.voltage_limit = 4.0f;
  motor.PID_velocity.limit = motor.voltage_limit;
  motor.velocity_limit = 20.0f;
  motor.LPF_velocity.Tf = 0.04f;

  motor.init();

#if MOTOR_IS_CALIBRATED
  motor.zero_electric_angle = zero_electric_angle_calibrated;
  motor.sensor_direction = sensor_direction_calibrated;
  motor.linkSensor(&sensor_calibrated);
  motor.initFOC();
#else
  sensor_calibrated.calibrate(motor);
  motor.linkSensor(&sensor_calibrated);
  motor.initFOC();
#endif

  // Инициализация наблюдателя текущим положением
  position_setpoint = (double)motor.shaft_angle;
  observer.reset(motor.shaft_angle, 0.0f);
  observer.setQuantization(2.0f * PI / 16384.0f);

  // Commander
  command.add('T', doTarget, "target velocity [rad/s]");
  command.add('W', doPower, "power 0/1");
  command.add('P', doP, "P_angle.P");
  command.add('I', doI, "P_angle.I");
  command.add('D', doD, "P_angle.D");
  command.add('L', doVLim, "voltage_limit");
  command.add('V', doVelLim, "velocity_limit");
  command.add('A', doAlpha, "observer alpha");
  // command.add('B', doBeta, "observer beta");

  command.verbose = VerboseMode::nothing;

  Serial.println(F("EncoderObserver (alpha-beta) ready"));
  _delay(200);

  xTaskCreatePinnedToCore(motorControlTask, "MotorCtrl", 4096, NULL, 5, NULL,
                          1);
}

// ===================== Loop (телеметрия) =====================
void loop() {
  command.run();

  Serial1.printf("%f,%f,%f,%f,%f,%f\n", target_velocity, observer.velocity(),
                 observer.position(), motor.shaft_angle, motor.voltage.q,
                 (float)position_setpoint);
}