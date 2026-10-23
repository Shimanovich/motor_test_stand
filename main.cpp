#include <SimpleFOC.h>
#include <Wire.h>
#include <driver/uart.h>
#include <encoders/calibrated/CalibratedSensor.h>
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"
#include "icm20602.h"
// #include "jv_kyt_g03.h"
#include "luts.h"

// ===================== Датчики =====================
MagneticSensorMT6701SSI sensor_yaw = MagneticSensorMT6701SSI(SENSOR_1_CS_PIN);
MagneticSensorMT6701SSI sensor_pitch = MagneticSensorMT6701SSI(SENSOR_2_CS_PIN);

#define MOTOR_IS_CALIBRATED 1

#if MOTOR_IS_CALIBRATED
CalibratedSensor sensor_yaw_calibrated =
    CalibratedSensor(sensor_yaw, LUTS_TOTAL, calibrationLut_1);
CalibratedSensor sensor_pitch_calibrated =
    CalibratedSensor(sensor_pitch, LUTS_TOTAL, calibrationLut_2);
#else
CalibratedSensor sensor_yaw_calibrated =
    CalibratedSensor(sensor_yaw, LUTS_TOTAL);
CalibratedSensor sensor_pitch_calibrated =
    CalibratedSensor(sensor_pitch, LUTS_TOTAL);
#endif

ICM20602 imu_platform;

// ===================== Мотор =====================
BLDCMotor motor_yaw = BLDCMotor(MOTOR_PP, MOTOR_R, MOTOR_KV, MOTOR_L);
BLDCDriver3PWM driver_yaw = BLDCDriver3PWM(DRIVER_YAW_PWM_A, DRIVER_YAW_PWM_B,
                                           DRIVER_YAW_PWM_C, DRIVER_YAW_EN);

BLDCMotor motor_pitch = BLDCMotor(MOTOR_PP, MOTOR_R, MOTOR_KV, MOTOR_L);
BLDCDriver3PWM driver_pitch =
    BLDCDriver3PWM(DRIVER_PITCH_PWM_A, DRIVER_PITCH_PWM_B, DRIVER_PITCH_PWM_C,
                   DRIVER_PITCH_EN);

// ===================== Регуляторы (2 кГц) =====================
PIDController rate_pid(0.10f, 0.0f, 0.0f, 2000.0f, 4.0f);
LowPassFilter rate_lpf(0.008f);

PIDController angle_pid(1.2f, 0.0f, 0.0f, 2000.0f, 5.0f);
LowPassFilter angle_lpf(0.003f);

// ===================== Состояние =====================
volatile int powerOn = 0;
volatile float voltage_yaw = 0.0f;
volatile float voltage_pitch = 0.0f;

float platform_gyro_bias = 0.0f;
float platform_rate_raw = 0.0f;
float platform_rate = 0.0f;
float platform_angle = 0.0f;
float desired_angle = 0.0f;
float desired_rate = 0.0f;

Commander command = Commander(Serial1);

static const float CONTROL_DT_2K = 0.0005f;
static const uint32_t CONTROL_PERIOD_US = 500;

static constexpr int RING_SIZE = 1024;
uint8_t ringBuf[RING_SIZE];
volatile int ringHead = 0;
volatile int ringTail = 0;

// ===================== Функции =====================
void zeroMotorVoltage() {
  motor_yaw.current_sp = 0.0f;
  motor_yaw.voltage.q = 0.0f;
  motor_yaw.voltage.d = 0.0f;

  motor_pitch.current_sp = 0.0f;
  motor_pitch.voltage.q = 0.0f;
  motor_pitch.voltage.d = 0.0f;

  rate_pid.reset();
  angle_pid.reset();
}

void doPower(char* cmd) {
  float in = 0.0f;
  command.scalar(&in, cmd);

  if ((int)in == 0) {
    powerOn = 0;
    zeroMotorVoltage();
    motor_yaw.disable();
    motor_pitch.disable();
    return;
  }

  platform_angle = 0.0f;
  desired_angle = 0.0f;
  zeroMotorVoltage();
  motor_yaw.enable();
  motor_pitch.enable();
  powerOn = 1;
}

void doRateP(char* cmd) { command.scalar(&rate_pid.P, cmd); }
void doRateI(char* cmd) { command.scalar(&rate_pid.I, cmd); }
void doRateD(char* cmd) { command.scalar(&rate_pid.D, cmd); }
void doRateLpf(char* cmd) { command.scalar(&rate_lpf.Tf, cmd); }

void doAngleP(char* cmd) { command.scalar(&angle_pid.P, cmd); }
void doAngleI(char* cmd) { command.scalar(&angle_pid.I, cmd); }
void doAngleD(char* cmd) { command.scalar(&angle_pid.D, cmd); }

void doYawSpeedCmd(char* cmd) { command.scalar((float*)&voltage_yaw, cmd); }
void doPitchSpeedCmd(char* cmd) { command.scalar((float*)&voltage_pitch, cmd); }

void doVLim(char* cmd) {
  // command.scalar(&motor.voltage_limit, cmd);
  // rate_pid.limit = motor.voltage_limit;
}

void calibrateGyroBias() {
  Serial.println(F("=== Bias calibration ==="));
  Serial.println(F("Hold platform COMPLETELY still for 2 seconds!"));

  const int samples = 4000;
  float sum = 0.0f;
  float g[3];

  for (int i = 0; i < samples; i++) {
    if (imu_platform.readGyro(g) == ICM20602_OK) {
      sum += g[2];
    }
    delayMicroseconds(500);
    delay(1);
  }

  platform_gyro_bias = sum / samples;
  Serial.printf("Platform gyro bias Z: %.4f deg/s\n", platform_gyro_bias);
  Serial.println(F("Bias calibration done."));
}

// ===================== Управление 2 кГц =====================
//
//   desired_angle += external_speed_cmd * dt
//   desired_rate   = angle_pid(desired_angle - platform_angle)
//   voltage        = rate_pid(desired_rate - platform_rate)
//
// S == 0 → desired_angle стоит, платформа держит угол.
// S != 0 → desired_angle растёт с заданной скоростью, платформа следует.
//
void controlTask(void* pvParameters) {
  const TickType_t period = pdMS_TO_TICKS(1);  // 1 мс
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    // float g[3] = {0, 0, 0};
    // if (imu_platform.readGyro(g) == ICM20602_OK) {
    //   platform_rate_raw = (g[2] - platform_gyro_bias) * 0.017453292519943f;
    // }

    // float rate_for_angle = angle_lpf(platform_rate_raw);
    // platform_rate = rate_lpf(platform_rate_raw);

    // if (powerOn) {
    //   platform_angle += rate_for_angle * CONTROL_DT_2K;
    //   desired_angle += external_speed_cmd * CONTROL_DT_2K;
    // }

    // float angle_error = desired_angle - platform_angle;
    // desired_rate = angle_pid(angle_error);

    // float rate_error = desired_rate - platform_rate;
    // float u = rate_pid(rate_error);

    if (powerOn) {
      motor_yaw.loopFOC();
      motor_pitch.loopFOC();

      motor_yaw.move(voltage_yaw);
      motor_pitch.move(voltage_pitch);

    } else {
      if (motor_yaw.sensor) {
        motor_yaw.sensor->update();
        motor_yaw.shaft_angle = motor_yaw.shaftAngle();
        motor_yaw.shaft_velocity = motor_yaw.shaftVelocity();
      }

      if (motor_pitch.sensor) {
        motor_pitch.sensor->update();
        motor_pitch.shaft_angle = motor_pitch.shaftAngle();
        motor_pitch.shaft_velocity = motor_pitch.shaftVelocity();
      }
    }

    vTaskDelayUntil(&lastWakeTime, period);
  }
}

// ===================== Setup =====================
void setup() {
  delay(200);

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, CUSTOM_RX_PIN, CUSTOM_TX_PIN);

  // uart для внешнего гироскопа
  //  // Настройка UART через IDF
  //  uart_config_t cfg = {
  //      .baud_rate = 921600,
  //      .data_bits = UART_DATA_8_BITS,
  //      .parity = UART_PARITY_EVEN,
  //      .stop_bits = UART_STOP_BITS_1,
  //      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  //      .source_clk = UART_SCLK_APB,
  //  };

  // uart_driver_install(GYRO_UART_NUM, GYRO_BUF_SIZE * 2, 0, 0, nullptr, 0);
  // uart_param_config(GYRO_UART_NUM, &cfg);
  // uart_set_pin(GYRO_UART_NUM, UART_PIN_NO_CHANGE, GYRO_RX_PIN,
  //              UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  Wire.begin(SDA_PIN, SCL_PIN, I2C_CLOCK);

  Serial.println(F("Init platform IMU..."));
  if (imu_platform.begin(0x68, &Wire) != ICM20602_OK) {
    Serial.println(F("Platform ICM20602 FAILED"));
    while (true) delay(1000);
  }
  Serial.println(F("Platform IMU OK (2 kHz control)"));

  // gyro.calibrate();

  sensor_yaw.init();
  sensor_pitch.init();

  motor_yaw.linkSensor(&sensor_yaw);
  motor_pitch.linkSensor(&sensor_pitch);

  driver_yaw.voltage_power_supply = POWER_SUPPLY;
  driver_pitch.voltage_power_supply = POWER_SUPPLY;

  driver_yaw.init();
  driver_pitch.init();

  motor_yaw.linkDriver(&driver_yaw);
  motor_pitch.linkDriver(&driver_pitch);

  motor_yaw.controller = motor_pitch.controller = MotionControlType::torque;
  motor_yaw.torque_controller = motor_pitch.torque_controller =
      TorqueControlType::voltage;
  motor_yaw.foc_modulation = motor_pitch.foc_modulation =
      FOCModulationType::SpaceVectorPWM;

  motor_yaw.voltage_limit = motor_pitch.voltage_limit = 3.5f;

  rate_pid.limit = motor_yaw.voltage_limit;
  motor_yaw.velocity_limit = 20.0f;
  motor_pitch.velocity_limit = 20.0f;

  rate_pid.Ts = CONTROL_DT_2K;
  angle_pid.Ts = CONTROL_DT_2K;

  motor_yaw.init();
  motor_pitch.init();

#if MOTOR_IS_CALIBRATED
  motor_yaw.zero_electric_angle = zero_electric_angle_calibrated_1;
  motor_yaw.sensor_direction = sensor_direction_calibrated_1;
  motor_yaw.linkSensor(&sensor_yaw_calibrated);
  motor_yaw.initFOC();

  motor_pitch.zero_electric_angle = zero_electric_angle_calibrated_2;
  motor_pitch.sensor_direction = sensor_direction_calibrated_2;
  motor_pitch.linkSensor(&sensor_pitch_calibrated);
  motor_pitch.initFOC();

#else
  sensor_1_calibrated.calibrate(motor);
  motor.linkSensor(&sensor_1_calibrated);
  motor.initFOC();
#endif

  command.add('W', doPower, "power 0/1");

  command.add('Y', doYawSpeedCmd, "yaw_volt");
  command.add('P', doPitchSpeedCmd, "pitch_volt");

  // command.add('P', doRateP, "rate P");
  // command.add('I', doRateI, "rate I");
  // command.add('D', doRateD, "rate D");
  // command.add('F', doRateLpf, "rate LPF Tf");

  // command.add('A', doAngleP, "angle P");
  // command.add('B', doAngleI, "angle I");
  // command.add('C', doAngleD, "angle D");

  // command.add('S', doSpeedCmd, "external speed rad/s");
  // command.add('L', doVLim, "voltage limit");

  command.verbose = VerboseMode::nothing;

  // Serial.println(F("Ready: 2 kHz single-gyro Angle+Rate"));
  // Serial.println(F("W1 to enable. Commands: P I D F A B C S L"));

  xTaskCreatePinnedToCore(controlTask, "Control", 8192, NULL, 5, NULL, 1);

  // calibrateGyroBias();
}

void loop() {
  command.run();

  // Serial1.printf("%f,%f,%f,%f,%f,%f\n", external_speed_cmd,
  // motor_yaw.voltage.q,
  //                motor_pitch.voltage.q, platform_rate, platform_angle,
  //                desired_rate);

  Serial1.printf("%f,%f,%f,%f\n", sensor_yaw_calibrated.getAngle(),
                 sensor_pitch_calibrated.getAngle(), motor_yaw.voltage.q,
                 motor_pitch.voltage.q);

  // Serial1.printf("%f,%f,%f\n", platform_rate_raw, platform_rate_raw_i2c,
  //                platform_rate_raw / platform_rate_raw_i2c);

  vTaskDelay(20);

  // static uint32_t lastCount = 0;

  // uint32_t count = pktCount;
  // if (count != lastCount) {
  //   Serial1.printf("%f,%f,%f,%f\n", gX, gY, gZ,
  //                  (float)((count - lastCount) * 10.0));
  //   lastCount = count;
  // }

  // vTaskDelay(pdMS_TO_TICKS(100));
}
