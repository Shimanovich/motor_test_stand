
#include <SimpleFOC.h>
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>

#include "VelocityDobController.h"
#include "config.h"
#include "encoders/calibrated/CalibratedSensor.h"

MagneticSensorMT6701SSI sensor = MagneticSensorMT6701SSI(SENSOR_CS_PIN);

BLDCMotor motor = BLDCMotor(MOTOR_PP, MOTOR_R, MOTOR_KV, MOTOR_L);
BLDCDriver3PWM driver =
    BLDCDriver3PWM(DRIVER_PWM_A, DRIVER_PWM_B, DRIVER_PWM_C, DRIVER_EN);

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

float zero_electric_angle_calibrated = 0.268594;
Direction sensor_direction_calibrated = Direction::CW;

#if MOTOR_IS_CALIBRATED
CalibratedSensor sensor_calibrated =
    CalibratedSensor(sensor, 100, calibrationLut);
#else
CalibratedSensor sensor_calibrated = CalibratedSensor(sensor, 100);
#endif

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
  // motor.controller = MotionControlType::custom;
  motor.controller = MotionControlType::velocity_openloop;

  motor.torque_controller = TorqueControlType::voltage;
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.voltage_limit = VOLTAGE_LIMIT;

#if MOTOR_IS_CALIBRATED
  motor.zero_electric_angle = zero_electric_angle_calibrated;
  motor.sensor_direction = sensor_direction_calibrated;
#endif

  motor.useMonitoring(Serial);

  motor.init();

  // sensor_calibrated.calibrate(motor);

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
}

void loop() {
  command.run();
  Serial1.printf("%.3f,%.3f\n", target_velocity, motor.shaft_velocity);
}
