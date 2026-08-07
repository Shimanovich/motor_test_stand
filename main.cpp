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
      last_smooth_target = 0.0f;
      lowSpeed.reset();
      motor.move(0);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Глобально

// ===================== Фильтр скорости для PI (сильнее, чем LPF SimpleFOC)
// =====================
LowPassFilter LPF_vel_ctrl(0.08f);  // 80 мс — критично для 0.2–0.5 рад/с

float stage2MotionControl(FOCMotor* m) {
  // Сильно сглаженная скорость только для регулятора
  float y = LPF_vel_ctrl(m->shaft_velocity);

  float error = last_smooth_target - y;

  float u_pid = motor.PID_velocity(error);
  float u_ff = lowSpeed.frictionCompensation(y);

  float u = u_pid + u_ff;
  return constrain(u, -m->voltage_limit, m->voltage_limit);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, CUSTOM_RX_PIN, CUSTOM_TX_PIN);
  // SimpleFOCDebug::enable(&Serial);

  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = POWER_SUPPLY;
  driver.init();
  motor.linkDriver(&driver);

  motor.linkCustomMotionControl(stage2MotionControl);
  motor.controller = MotionControlType::custom;
  motor.torque_controller = TorqueControlType::voltage;
  motor.foc_modulation = FOCModulationType::SinePWM;

  // ---- Мягкий PID под малые скорости ----
  motor.PID_velocity.P = 0.08f;
  motor.PID_velocity.I = 0.6f;
  motor.PID_velocity.D = 0.0f;
  motor.PID_velocity.output_ramp = 120.0f;
  motor.PID_velocity.limit = VOLTAGE_LIMIT;

  // Фильтр SimpleFOC (для shaft_velocity / телеметрии)
  motor.LPF_velocity.Tf = 0.04f;

  motor.voltage_limit = 4.0f;  // на время отладки низких скоростей
  motor.PID_velocity.limit = motor.voltage_limit;
  motor.velocity_limit = 20.0f;

  // Зоны: 0.26 не должно «ломаться» soft-зоной
  lowSpeed.setParams(5.0f, 0.04f, 0.12f);   // accel, deadzone, soft_zone
  lowSpeed.setFriction(0.0f, 0.0f, 0.12f);  // трение пока 0

  motor.init();
  motor.useMonitoring(Serial);

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

  // Commander
  command.add('T', doTarget, "target velocity [rad/s]");
  command.add('W', doPower, "power 0/1");
  command.add('P', doP, "PID P");
  command.add('I', doI, "PID I");
  command.add('D', doD, "PID D");
  command.add('R', doRamp, "PID output_ramp");
  command.add('F', doTf, "LPF SimpleFOC Tf");
  command.add('L', doVLim, "voltage_limit");
  command.add('V', doVelLim, "velocity_limit");
  command.add('A', doAccel, "max_accel");
  command.add('Z', doDead, "deadzone");
  command.add('S', doSoft, "soft_zone");
  command.add('C', doCoulomb, "coulomb");
  command.add('B', doViscous, "viscous");
  command.add('N', doSoftSign, "soft sign");

  // Отдельный фильтр регулятора
  command.add(
      'E', [](char* cmd) { command.scalar(&LPF_vel_ctrl.Tf, cmd); },
      "LPF controller Tf");

  command.verbose = VerboseMode::nothing;

  Serial.println(F("Low-speed fix: strong LPF + soft PI"));
  _delay(300);

  xTaskCreatePinnedToCore(motorControlTask, "MotorCtrl", 4096, NULL, 5, NULL,
                          1);
}
void loop() {
  command.run();
  Serial1.printf("%.3f,%.3f,%.3f\n", target_velocity, motor.shaft_velocity,
                 motor.voltage.q);
}