#include <SimpleFOC.h>
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>

#include "VelocityDobController.h"
#include "config.h"

MagneticSensorMT6701SSI sensor = MagneticSensorMT6701SSI(SENSOR_CS_PIN);
BLDCMotor motor = BLDCMotor(MOTOR_PP, MOTOR_R, MOTOR_KV, MOTOR_L);
BLDCDriver3PWM driver =
    BLDCDriver3PWM(DRIVER_PWM_A, DRIVER_PWM_B, DRIVER_PWM_C, DRIVER_EN);

volatile float target_velocity = 0.0f;

// Экземпляр регулятора
VelocityDobController controller;                     // сам объект
VelocityDobController* controller_ptr = &controller;  // указатель для обёртки

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

// FreeRTOS задача 1 кГц
void motorControlTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 1;

  for (;;) {
    motor.loopFOC();
    motor.move(target_velocity);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, CUSTOM_RX_PIN, CUSTOM_TX_PIN);
  SimpleFOCDebug::enable(&Serial);

  // Железо
  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = POWER_SUPPLY;
  driver.init();
  motor.linkDriver(&driver);

  // Регулятор
  controller.setPID(0.8f, 8.0f);
  controller.setDob(45.0f, 35.0f);
  controller.setFilters(0.018f, 12.0f, 0.10f, 0.35f);
  controller.setVoltageLimit(VOLTAGE_LIMIT);

  motor.linkCustomMotionControl(customMotionControlWrapper);
  motor.controller = MotionControlType::custom;
  motor.torque_controller = TorqueControlType::voltage;
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.voltage_limit = VOLTAGE_LIMIT;

  motor.useMonitoring(Serial);

  motor.init();
  motor.initFOC();

  // Команды
  command.add('T', doTarget, "target velocity");
  command.add('P', doKp, "Kp");
  command.add('I', doKi, "Ki");
  command.add('B', doBn, "bn");
  command.add('G', doG, "DOB g");

  Serial.println(F("PID + DOB modular version ready"));
  _delay(1000);

  xTaskCreatePinnedToCore(motorControlTask, "MotorCtrl", 4096, NULL, 5, NULL,
                          1);
}

void loop() {
  command.run();
  Serial1.printf("vel:%.3f\n", motor.shaft_velocity);
}