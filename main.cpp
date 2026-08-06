/**
 * Этап 1: плавное управление BLDC на минимальных скоростях
 */

#include <SimpleFOC.h>
#include <encoders/calibrated/CalibratedSensor.h>
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>

#include "LowSpeedVelocityController.h"
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

LowSpeedVelocityController lowSpeed;

// ===================== Commander =====================
Commander command = Commander(Serial1);

// --- Основные ---
void doTarget(char* cmd) { command.scalar((float*)&target_velocity, cmd); }

void doPower(char* cmd) {
  float in;
  command.scalar(&in, cmd);
  powerOn = (int)in;
  if (!powerOn) {
    target_velocity = 0.0f;
    lowSpeed.reset();
  }
}

// --- PID скорости ---
void doP(char* cmd) { command.scalar(&motor.PID_velocity.P, cmd); }
void doI(char* cmd) { command.scalar(&motor.PID_velocity.I, cmd); }
void doD(char* cmd) { command.scalar(&motor.PID_velocity.D, cmd); }
void doRamp(char* cmd) { command.scalar(&motor.PID_velocity.output_ramp, cmd); }

// --- Фильтры и ограничения ---
void doTf(char* cmd) { command.scalar(&motor.LPF_velocity.Tf, cmd); }
void doVLim(char* cmd) {
  command.scalar(&motor.voltage_limit, cmd);
  motor.PID_velocity.limit = motor.voltage_limit;
}
void doVelLim(char* cmd) { command.scalar(&motor.velocity_limit, cmd); }

// --- Сглаживание задания (LowSpeed) ---
void doAccel(char* cmd) { command.scalar(&lowSpeed.maxAccel(), cmd); }
void doDead(char* cmd) { command.scalar(&lowSpeed.deadzone(), cmd); }
void doSoft(char* cmd) { command.scalar(&lowSpeed.softZone(), cmd); }

void doCoulomb(char* cmd) { command.scalar(&lowSpeed.coulomb(), cmd); }
void doViscous(char* cmd) { command.scalar(&lowSpeed.viscous(), cmd); }
void doSoftSign(char* cmd) { command.scalar(&lowSpeed.softSign(), cmd); }

float last_smooth_target = 0.0f;
// ===================== FreeRTOS задача 1 кГц =====================
void motorControlTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 1;

  for (;;) {
    if (powerOn) {
      last_smooth_target = lowSpeed.process(target_velocity);
      motor.loopFOC();
      motor.move(last_smooth_target);
    } else {
      motor.move(0);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Глобально


float stage2MotionControl(FOCMotor* m) {
  // 1. Ошибка скорости
  float error = last_smooth_target - m->shaft_velocity;

  // 2. Встроенный PI SimpleFOC можно вызвать вручную,
  //    но проще использовать свой мини-PI или оставить motor.PID_velocity
  float u_pid = motor.PID_velocity(error);  // использует P, I, ramp, limit

  // 3. Компенсация трения
  float u_ff = lowSpeed.frictionCompensation(m->shaft_velocity);

  float u = u_pid + u_ff;
  u = constrain(u, -m->voltage_limit, m->voltage_limit);
  return u;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, CUSTOM_RX_PIN, CUSTOM_TX_PIN);
  
  // Временно отключите debug на время калибровки (можно включить потом)
  // SimpleFOCDebug::enable(&Serial);   // ← закомментируйте на время теста

  sensor.init();
  motor.linkSensor(&sensor);               // сначала сырой датчик

  driver.voltage_power_supply = POWER_SUPPLY;
  driver.init();
  motor.linkDriver(&driver);

  // Режим управления
  motor.linkCustomMotionControl(stage2MotionControl);
  motor.controller = MotionControlType::custom;
  motor.torque_controller = TorqueControlType::voltage;
  motor.foc_modulation = FOCModulationType::SinePWM;

  // PID и лимиты
  motor.PID_velocity.P = 0.15f;
  motor.PID_velocity.I = 3.0f;
  motor.PID_velocity.D = 0.0f;
  motor.PID_velocity.output_ramp = 300.0f;
  motor.PID_velocity.limit = VOLTAGE_LIMIT;
  motor.LPF_velocity.Tf = 0.02f;
  motor.voltage_limit = VOLTAGE_LIMIT;
  motor.velocity_limit = 20.0f;

  lowSpeed.setParams(8.0f, 0.08f, 0.30f);
  lowSpeed.setFriction(0.0f, 0.0f, 0.15f);

  // ===================== КРИТИЧНО: motor.init() ПЕРЕД calibrate =====================
  motor.useMonitoring(Serial);             // лучше после init, но можно здесь
  motor.init();                            // ← обязательно до calibrate

  #if MOTOR_IS_CALIBRATED
    // Уже откалибровано — просто задаём значения
    motor.zero_electric_angle = zero_electric_angle_calibrated;
    motor.sensor_direction = sensor_direction_calibrated;
    motor.linkSensor(&sensor_calibrated);  // переключаемся на калиброванный
    motor.initFOC();                       // выравнивание пропускается
  #else
    // Калибровка
    // sensor_calibrated.voltage_calibration = 3.0f;  // при необходимости уменьшить
    sensor_calibrated.calibrate(motor);    // теперь безопасно

    // После калибровки переключаемся
    motor.linkSensor(&sensor_calibrated);
    motor.initFOC();                       // или можно не вызывать, если calibrate уже сделал
  #endif

  motor.useMonitoring(Serial);
  motor.init();
  motor.linkSensor(&sensor_calibrated);
  motor.initFOC();

  // ========== Все команды Commander ==========
  command.add('T', doTarget, "target velocity [rad/s]");
  command.add('W', doPower, "power 0/1");

  command.add('P', doP, "PID P");
  command.add('I', doI, "PID I");
  command.add('D', doD, "PID D");
  command.add('R', doRamp, "PID output_ramp");

  command.add('F', doTf, "LPF velocity Tf");
  command.add('L', doVLim, "voltage_limit");
  command.add('V', doVelLim, "velocity_limit");

  command.add('A', doAccel, "max_accel [rad/s^2]");
  command.add('Z', doDead, "deadzone [rad/s]");
  command.add('S', doSoft, "soft_zone [rad/s]");

  command.add('C', doCoulomb, "coulomb friction [V]");
  command.add('B', doViscous, "viscous friction [V/(rad/s)]");
  command.add('N', doSoftSign, "soft sign zone [rad/s]");

  command.verbose = VerboseMode::nothing;

  Serial.println(F("=== Stage 1: Low-speed closed-loop velocity ==="));
  Serial.println(
      F("T-target  W-power  P/I/D/R-PID  F-LPF  L-Vlim  V-velLim  A-accel  "
        "Z-dead  S-soft"));
  _delay(500);

  xTaskCreatePinnedToCore(motorControlTask, "MotorCtrl", 4096, NULL, 5, NULL,
                          1);
}

void loop() {
  command.run();
  Serial1.printf("%.3f,%.3f,%.3f\n", target_velocity, motor.shaft_velocity,
                 motor.voltage.q);
}