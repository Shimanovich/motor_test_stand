#include <SimpleFOC.h>
#include <encoders/calibrated/CalibratedSensor.h>
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>

#include "ExtrapolatingSensor.h"
#include "config.h"
#include "luts.h"
#include "newLfp.h"

MagneticSensorMT6701SSI sensor = MagneticSensorMT6701SSI(SENSOR_CS_PIN);
BLDCMotor motor = BLDCMotor(MOTOR_PP, MOTOR_R, MOTOR_KV, MOTOR_L);
BLDCDriver3PWM driver =
    BLDCDriver3PWM(DRIVER_PWM_A, DRIVER_PWM_B, DRIVER_PWM_C, DRIVER_EN);

#define MOTOR_IS_CALIBRATED 1

#if MOTOR_IS_CALIBRATED
CalibratedSensor sensor_calibrated =
    CalibratedSensor(sensor, LUTS_TOTAL, calibrationLut);
#else
CalibratedSensor sensor_calibrated = CalibratedSensor(sensor, LUTS_TOTAL);
#endif

ExtrapolatingSensor sensor_extrap = ExtrapolatingSensor(sensor_calibrated);

volatile float target_velocity = 0.0f;
int powerOn = 0;

double position_setpoint = 0.0;

NewLowPassFilter lfp(0.002);

float filtred = 0.0;

float filter2dval = 0.0f;

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
    motor.loopFOC();
    position_setpoint = (float)motor.shaft_angle;
  }
}

void doP(char* cmd) { command.scalar(&motor.P_angle.P, cmd); }
void doI(char* cmd) { command.scalar(&motor.P_angle.I, cmd); }
void doD(char* cmd) { command.scalar(&motor.P_angle.D, cmd); }

void doVLim(char* cmd) {
  command.scalar(&motor.voltage_limit, cmd);
  motor.PID_velocity.limit = motor.voltage_limit;
}
void doVelLim(char* cmd) { command.scalar(&motor.velocity_limit, cmd); }

void doLfp(char* cmd) { command.scalar(&motor.LPF_angle.Tf, cmd); }

void motorControlTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 1;

  for (;;) {
    if (powerOn) {
      position_setpoint += (double)target_velocity * CONTROL_DT;
      motor.loopFOC();
      motor.move((float)position_setpoint);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

float stage2MotionControl(FOCMotor* m) {
  static float cur_out = 0;
  float oldshaft_angle;
  m->shaft_angle_sp = m->target;

  if (oldshaft_angle != m->shaft_angle) {
    filtred = m->shaft_angle;
    cur_out = m->LPF_angle(m->P_angle(m->shaft_angle_sp - filtred));
    oldshaft_angle = m->shaft_angle;
  }
  return cur_out;
}

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

  motor.P_angle.P = 4.0f;
  motor.P_angle.I = 10.0f;
  motor.P_angle.D = 0.0f;

  motor.voltage_limit = 4.0f;
  motor.PID_velocity.limit = motor.voltage_limit;
  motor.velocity_limit = 20.0f;

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

  command.add('T', doTarget, "target velocity [rad/s]");
  command.add('W', doPower, "power 0/1");
  command.add('P', doP, "P_angle.P");
  command.add('I', doI, "P_angle.I");
  command.add('D', doD, "P_angle.D");
  command.add('L', doVLim, "voltage_limit");
  command.add('V', doVelLim, "velocity_limit");
  command.add('F', doLfp, "FilTER");

  command.verbose = VerboseMode::nothing;

  Serial.println(F("ready"));
  _delay(200);

  xTaskCreatePinnedToCore(motorControlTask, "MotorCtrl", 4096, NULL, 5, NULL,
                          1);
}

void loop() {
  command.run();

  Serial1.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", target_velocity,
                 motor.shaft_angle, motor.voltage.q, (float)position_setpoint,
                 filtred, filter2dval, motor.shaft_velocity,
                 (float)_micros() * 1e-6f);
}
