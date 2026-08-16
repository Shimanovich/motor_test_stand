#include <SimpleFOC.h>
#include <encoders/calibrated/CalibratedSensor.h>
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>
#include <Wire.h>

#include "config.h"
#include "luts.h"
#include "icm20602.h"

// ===================== Датчики =====================
MagneticSensorMT6701SSI sensor = MagneticSensorMT6701SSI(SENSOR_CS_PIN);

#define MOTOR_IS_CALIBRATED 1

#if MOTOR_IS_CALIBRATED
CalibratedSensor sensor_calibrated = CalibratedSensor(sensor, LUTS_TOTAL, calibrationLut);
#else
CalibratedSensor sensor_calibrated = CalibratedSensor(sensor, LUTS_TOTAL);
#endif

// IMU
ICM20602 imu_platform;                 // на платформе (Wire)
TwoWire I2C_frame = TwoWire(1);        // второй контроллер
ICM20602 imu_frame;                    // на раме (Wire1, пины 25/26)

// Bias
float platform_gyro_bias = 0.0f;
float frame_gyro_bias    = 0.0f;

// ===================== Мотор =====================
BLDCMotor motor = BLDCMotor(MOTOR_PP, MOTOR_R, MOTOR_KV, MOTOR_L);
BLDCDriver3PWM driver = BLDCDriver3PWM(DRIVER_PWM_A, DRIVER_PWM_B, DRIVER_PWM_C, DRIVER_EN);

// ===================== Регуляторы =====================
PIDController rate_pid(0.45f, 1.2f, 0.001f, 1000.0f, 4.0f);  // P, I, D, ramp, limit
LowPassFilter rate_lpf(0.008f);                               // Tf = 8 мс

// Внешняя команда скорости (пока 0)
volatile float external_speed_cmd = 0.0f;

// Состояние
volatile int powerOn = 0;

float relative_rate = 0.0f; 
float platform_rate = 0.0f;
float frame_rate = 0.0f;

// Commander
Commander command = Commander(Serial1);

// ===================== Вспомогательные функции =====================
void zeroMotorVoltage() {
  motor.current_sp = 0.0f;
  motor.voltage.q = 0.0f;
  motor.voltage.d = 0.0f;
  rate_pid.reset();
}

void doPower(char* cmd) {
  float in = 0.0f;
  command.scalar(&in, cmd);

  if ((int)in == 0) {
    powerOn = 0;
    zeroMotorVoltage();
    motor.disable();
    return;
  }

  zeroMotorVoltage();
  motor.enable();
  powerOn = 1;
}

void doRateP(char* cmd) { command.scalar(&rate_pid.P, cmd); }
void doRateI(char* cmd) { command.scalar(&rate_pid.I, cmd); }
void doRateD(char* cmd) { command.scalar(&rate_pid.D, cmd); }
void doRateLpf(char* cmd) { command.scalar(&rate_lpf.Tf, cmd); }
void doSpeedCmd(char* cmd) { command.scalar((float*)&external_speed_cmd, cmd); }
void doVLim(char* cmd) {
  command.scalar(&motor.voltage_limit, cmd);
  rate_pid.limit = motor.voltage_limit;
}

// ===================== Bias-калибровка =====================
void calibrateGyroBias() {
  Serial.println(F("=== Bias calibration ==="));
  Serial.println(F("Hold platform COMPLETELY still for 2 seconds!"));

  const int samples = 2000;
  float sum_p = 0.0f;
  float sum_f = 0.0f;

  for (int i = 0; i < samples; i++) {
    float gp[3], ap[3], tp;
    float gf[3], af[3], tf;

    imu_platform.read(gp, ap, &tp);
    imu_frame.read(gf, af, &tf);

    sum_p += gp[2];
    sum_f += gf[2];
    delay(1);
  }

  platform_gyro_bias = sum_p / samples;
  frame_gyro_bias    = sum_f / samples;

  Serial.printf("Platform gyro bias Z: %.4f deg/s\n", platform_gyro_bias);
  Serial.printf("Frame gyro bias Z:    %.4f deg/s\n", frame_gyro_bias);
  Serial.println(F("Bias calibration done."));
}

// ===================== Основная задача управления (1 кГц) =====================
void controlTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 1;   // 1 мс → 1 кГц

  for (;;) {
    // ----- 1. Чтение IMU -----
    float gp[3], ap[3], tp;
    float gf[3], af[3], tf;

    imu_platform.read(gp, ap, &tp);
    imu_frame.read(gf, af, &tf);

    // ----- 2. Relative rate (рад/с) -----
    platform_rate = (gp[2] - platform_gyro_bias) * 0.017453292519943f;
    frame_rate    = (gf[2] - frame_gyro_bias)    * 0.017453292519943f;
    relative_rate = platform_rate - frame_rate;

    relative_rate = rate_lpf(relative_rate);

    // ----- 3. Rate PID -----
    // external_speed_cmd пока = 0 → удержание relative_rate = 0
    float rate_error = external_speed_cmd - relative_rate;
    float u = rate_pid(rate_error);

    // ----- 4. Мотор -----
    if (powerOn) {
      motor.loopFOC();
      motor.move(u);               // в torque-режиме это voltage.q
    } else {
      // просто обновляем датчик
      if (motor.sensor) {
        motor.sensor->update();
        motor.shaft_angle = motor.shaftAngle();
        motor.shaft_velocity = motor.shaftVelocity();
      }
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ===================== Setup =====================
void setup() {
  delay(200);

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, CUSTOM_RX_PIN, CUSTOM_TX_PIN);

  // ----- I2C -----
  Wire.begin(SDA_PIN, SCL_PIN, I2C_CLOCK);                        // платформа
  I2C_frame.begin(FRAME_SDA_PIN, FRAME_SCL_PIN, I2C_FRAME_CLOCK); // рама 25/26

  // ----- IMU -----
  Serial.println(F("Init platform IMU..."));
  if (imu_platform.begin(0x68, &Wire) != ICM20602_OK) {
    Serial.println(F("Platform ICM20602 FAILED"));
    while (true) delay(1000);
  }
  Serial.println(F("Platform IMU OK"));

  Serial.println(F("Init frame IMU..."));
  if (imu_frame.begin(0x68, &I2C_frame) != ICM20602_OK) {
    Serial.println(F("Frame ICM20602 FAILED"));
    while (true) delay(1000);
  }
  Serial.println(F("Frame IMU OK"));

  // Bias (платформа должна быть неподвижна!)
  calibrateGyroBias();

  // ----- Энкодер и мотор -----
  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = POWER_SUPPLY;
  driver.init();
  motor.linkDriver(&driver);

  // Режим torque + voltage
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.foc_modulation = FOCModulationType::SinePWM;

  motor.voltage_limit = 4.0f;
  rate_pid.limit = motor.voltage_limit;
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

  // ----- Commander -----
  command.add('W', doPower, "power 0/1");
  command.add('P', doRateP, "rate P");
  command.add('I', doRateI, "rate I");
  command.add('D', doRateD, "rate D");
  command.add('F', doRateLpf, "rate LPF Tf");
  command.add('S', doSpeedCmd, "external speed cmd");
  command.add('L', doVLim, "voltage limit");

  command.verbose = VerboseMode::nothing;

  Serial.println(F("System ready. Send W1 to enable motor."));
  Serial.println(F("Commands: A(P) B(I) C(D) E(LPF) S(speed) L(Vlim) W(power)"));

  // Задача управления на ядре 1
  xTaskCreatePinnedToCore(controlTask, "Control", 8192, NULL, 5, NULL, 1);
}

// ===================== Loop =====================
void loop() {
  command.run();

  // Телеметрия (можно смотреть в VOFA+ или Serial)
  // relative_rate можно добавить в вывод при необходимости
  Serial1.printf("%f,%f,%f,%f,%f,%f,%f\n",
                 external_speed_cmd,
                 motor.voltage.q,
                 motor.shaft_velocity,
                 (float)motor.shaft_angle,
                 relative_rate,
                 frame_rate,
                platform_rate);
}