/**
 * Этап 1: плавное управление BLDC на минимальных скоростях
 */

#include <SimpleFOC.h>
#include <encoders/calibrated/CalibratedSensor.h>
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>

#include "LowSpeedVelocityController.h"
#include "config.h"

// ===================== Железо =====================
MagneticSensorMT6701SSI sensor = MagneticSensorMT6701SSI(SENSOR_CS_PIN);
BLDCMotor motor = BLDCMotor(MOTOR_PP, MOTOR_R, MOTOR_KV, MOTOR_L);
BLDCDriver3PWM driver =
    BLDCDriver3PWM(DRIVER_PWM_A, DRIVER_PWM_B, DRIVER_PWM_C, DRIVER_EN);

// ===================== Калибровка =====================
#define MOTOR_IS_CALIBRATED 1

float calibrationLut[100] = {
    -0.000688, -0.001537, -0.001537, -0.001537, -0.002271, -0.002271, -0.002271,
    -0.002507, -0.002507, -0.002507, -0.003049, -0.003049, -0.003049, -0.004359,
    -0.004359, -0.004359, -0.003674, -0.003674, -0.003674, -0.001992, -0.001992,
    -0.000617, -0.000617, -0.000617, 0.002100,  0.002100,  0.002100,  0.004779,
    0.004779,  0.004779,  0.005234,  0.005234,  0.005234,  0.005612,  0.005612,
    0.005612,  0.006067,  0.006067,  0.006067,  0.004566,  0.004566,  0.003486,
    0.003486,  0.003486,  0.003135,  0.003135,  0.003135,  0.001557,  0.001557,
    0.001557,  -0.000097, -0.000097, -0.000097, -0.001176, -0.001176, -0.001176,
    -0.002286, -0.002286, -0.002286, -0.003196, -0.003196, -0.004221, -0.004221,
    -0.004221, -0.004226, -0.004226, -0.004226, -0.004078, -0.004078, -0.004078,
    -0.003087, -0.003087, -0.003087, -0.001712, -0.001712, -0.001712, -0.000797,
    -0.000797, -0.000797, 0.000655,  0.000655,  0.002299,  0.002299,  0.002299,
    0.002331,  0.002331,  0.002331,  0.002096,  0.002096,  0.002096,  0.001937,
    0.001937,  0.001937,  0.000551,  0.000551,  0.000551,  -0.000835, -0.000835,
    -0.000835, -0.000688};

float zero_electric_angle_calibrated = 0.268594f;
Direction sensor_direction_calibrated = Direction::CW;

#if MOTOR_IS_CALIBRATED
CalibratedSensor sensor_calibrated =
    CalibratedSensor(sensor, 100, calibrationLut);
#else
CalibratedSensor sensor_calibrated = CalibratedSensor(sensor, 100);
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

// ===================== FreeRTOS задача 1 кГц =====================
void motorControlTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 1;

  for (;;) {
    if (powerOn) {
      float smooth_target = lowSpeed.process(target_velocity);
      motor.loopFOC();
      motor.move(smooth_target);
    } else {
      motor.move(0);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup() {
  Serial.begin(230400);
  Serial1.begin(230400, SERIAL_8N1, CUSTOM_RX_PIN, CUSTOM_TX_PIN);
  SimpleFOCDebug::enable(&Serial);

  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = POWER_SUPPLY;
  driver.init();
  motor.linkDriver(&driver);

  // Режим
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;
  motor.foc_modulation = FOCModulationType::SinePWM;

  // Стартовые мягкие настройки для малых скоростей
  motor.PID_velocity.P = 0.15f;
  motor.PID_velocity.I = 3.0f;
  motor.PID_velocity.D = 0.0f;
  motor.PID_velocity.output_ramp = 300.0f;
  motor.PID_velocity.limit = VOLTAGE_LIMIT;

  motor.LPF_velocity.Tf = 0.02f;
  motor.voltage_limit = VOLTAGE_LIMIT;
  motor.velocity_limit = 20.0f;

  lowSpeed.setParams(8.0f, 0.08f, 0.30f);

#if MOTOR_IS_CALIBRATED
  motor.zero_electric_angle = zero_electric_angle_calibrated;
  motor.sensor_direction = sensor_direction_calibrated;
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