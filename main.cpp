/**
 * Этап 1: плавное управление BLDC на минимальных скоростях
 */

#include <SimpleFOC.h>
#include <encoders/calibrated/CalibratedSensor.h>
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>
#include <esp_task_wdt.h>

#include "PositionExtrapolator.h"
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
float pr;
float lfp_multipler = 1.0;

PositionExtrapolator extrap(0.0001 * 2);

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
  }
}

// // --- PID скорости ---
// void doP(char* cmd) { command.scalar(&motor.PID_velocity.P, cmd); }
// void doI(char* cmd) { command.scalar(&motor.PID_velocity.I, cmd); }
// void doD(char* cmd) { command.scalar(&motor.PID_velocity.D, cmd); }

// --- PID положения ---
void doP(char* cmd) { command.scalar(&motor.P_angle.P, cmd); }
void doI(char* cmd) { command.scalar(&motor.P_angle.I, cmd); }
void doD(char* cmd) { command.scalar(&motor.P_angle.D, cmd); }

void doRamp(char* cmd) { command.scalar(&motor.PID_velocity.output_ramp, cmd); }

// --- Фильтры и ограничения ---
// void doTf(char* cmd) { command.scalar(&motor.LPF_velocity.Tf, cmd); }
void doTf(char* cmd) { command.scalar(&lfp_multipler, cmd); }

void doVLim(char* cmd) {
  command.scalar(&motor.voltage_limit, cmd);
  motor.PID_velocity.limit = motor.voltage_limit;
}
void doVelLim(char* cmd) { command.scalar(&motor.velocity_limit, cmd); }

double last_smooth_target = 0.0;
// ===================== FreeRTOS задача 1 кГц =====================
void motorControlTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 1;

  for (;;) {
    if (powerOn) {
      last_smooth_target += (double)target_velocity * 0.001;

      motor.loopFOC();
      motor.move((float)last_smooth_target);
    } else {
      last_smooth_target = 0.0f;
      motor.move(0);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

float stage2MotionControl(FOCMotor* m) {
  static float old_target = 0.0f;
  double old_t, new_t;

  new_t = (double)_micros() * 1e-6;

  if (old_target != m->shaft_angle) {
    extrap.setTcorr((new_t - old_t) * lfp_multipler);
    extrap.addMeasurement(m->shaft_angle, new_t);
    old_target = m->shaft_angle;

    //m->LPF_angle.Tf = lfp_multipler / (new_t - old_t);

    old_t = new_t;
  }

   pr = extrap.predict(new_t);
  //pr = m->LPF_angle(m->shaft_angle);

  m->shaft_angle_sp = m->target;

  // calculate the torque command - sensor precision: this calculation is ok,
  // but based on bad value from previous calculation
  return motor.P_angle(m->shaft_angle_sp - m->shaft_angle);
}

void setup() {
  // disableCore0WDT();
  // disableCore1WDT();

  delay(100);  // небольшая пауз

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
  // motor.controller = MotionControlType::angle_nocascade;

  motor.torque_controller = TorqueControlType::voltage;
  motor.foc_modulation = FOCModulationType::SinePWM;

  // ---- Мягкий PID под малые скорости ----
  motor.PID_velocity.P = 0.08f;
  motor.PID_velocity.I = 0.6f;
  motor.PID_velocity.D = 0.0f;

  motor.P_angle.P = 0.0;
  motor.P_angle.I = 0.0;
  motor.P_angle.D = 0.0;

  motor.PID_velocity.output_ramp = 120.0f;
  motor.PID_velocity.limit = VOLTAGE_LIMIT;

  // Фильтр SimpleFOC (для shaft_velocity / телеметрии)
  motor.LPF_velocity.Tf = 0.04f;

  motor.voltage_limit = 4.0f;  // на время отладки низких скоростей
  motor.PID_velocity.limit = motor.voltage_limit;
  motor.velocity_limit = 20.0f;

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

  command.verbose = VerboseMode::nothing;

  Serial.println(F("Low-speed fix: strong LPF + soft PI"));
  _delay(300);
  last_smooth_target = motor.shaftAngle();  // init position

  xTaskCreatePinnedToCore(motorControlTask, "MotorCtrl", 4096, NULL, 5, NULL,
                          1);
}
void loop() {
  command.run();

  float speedOmega;
  float posOmega;
  if (motor.controller == MotionControlType::velocity_openloop) {
    posOmega = motor.shaftAngle();
    speedOmega = motor.shaftVelocity();
  } else {
    speedOmega = motor.shaft_velocity;
    posOmega = motor.shaft_angle;
  }

  Serial1.printf("%f,%f,%f,%f,%f,%f\n", target_velocity, speedOmega, posOmega,
                 motor.voltage.q, last_smooth_target, pr);
}
