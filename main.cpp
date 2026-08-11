/**
 * Плавное управление BLDC на минимальных скоростях
 * KalmanEncoder (1D CV) + position control через custom motion
 */

#include <SimpleFOC.h>
#include <encoders/calibrated/CalibratedSensor.h>
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>

#include "KalmanEncoder.h"
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

double position_setpoint = 0.0;

// q_pos, q_vel, r — стартовые значения для MT6701 @ 1 kHz
KalmanEncoder observer(1e-7f, 5e-4f, 2e-6f);

// ===================== Commander =====================
Commander command = Commander(Serial1);

void doTarget(char* cmd) { command.scalar((float*)&target_velocity, cmd); }

void doPower(char* cmd) {
  float in;
  command.scalar(&in, cmd);
  powerOn = (int)in;
  if (!powerOn) {
    target_velocity = 0.0f;
    motor.disable();
  } else {
    motor.enable();
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

// Kalman noise parameters
void doQpos(char* cmd) { command.scalar(&observer.qPos(), cmd); }
void doQvel(char* cmd) { command.scalar(&observer.qVel(), cmd); }
void doR(char* cmd) { command.scalar(&observer.r(), cmd); }

// ===================== FreeRTOS задача 1 кГц =====================
void motorControlTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 1;  // 1 ms

  for (;;) {
    if (powerOn) {
      position_setpoint += (double)target_velocity * CONTROL_DT;
      motor.loopFOC();
      motor.move((float)position_setpoint);
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

  // 1. Prediction
  observer.predict(dt, target_velocity);

  // 2. Correction (only on new encoder LSB)
  observer.update(m->shaft_angle);

  // 3. Position error
  float error = m->target - observer.position();

  // 4. Small deadzone against residual quantization
  const float dead = 0.0004f;  // ~1 LSB of MT6701
  if (fabsf(error) < dead) error = 0.0f;

  // 5. Optional mild gain scheduling by speed
  float speed = fabsf(target_velocity);
  float P_low = 0.6f;
  float P_high = 4.0f;
  motor.P_angle.P = P_low + (P_high - P_low) * constrain(speed / 1.0f, 0.0f, 1.0f);
  // I stays as set by Commander (recommend 0 at ultra-low speeds)

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

  // Soft position regulator defaults (will be gain-scheduled online)
  motor.P_angle.P = 0.8f;
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

  // Init observer at current angle
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
  command.add('Q', doQpos, "Kalman q_pos");
  command.add('U', doQvel, "Kalman q_vel");
  command.add('R', doR, "Kalman r");

  command.verbose = VerboseMode::nothing;

  Serial.println(F("KalmanEncoder ready"));
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
