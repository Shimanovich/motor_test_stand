#include <SimpleFOC.h>
#include <encoders/calibrated/CalibratedSensor.h>
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>

#include "config.h"
#include "luts.h"
#include "icm20602.h"
#include "Wire.h"


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

ICM20602 imu;

volatile float target_velocity = 0.0f;
volatile int powerOn = 0;

double position_setpoint = 0.0;


float filtred = 0.0;

float filter2dval = 0.0f;

float tmp_speed = 0.0f;
PIDController gyro_pid = PIDController(1.0f, 0.0f, 0.0f, 0.0f, 20.0f);
LowPassFilter gyro_lpf = LowPassFilter(0.0f);

// Состояние кастомного регулятора: должно сбрасываться при включении,
// иначе после disable/enable на фазы уходит прошлое напряжение.
float motion_out = 0.0f;
float motion_old_shaft_angle = NAN;

Commander command = Commander(Serial1);

/**
 * Сканирует I2C-шину и выводит найденные устройства в Serial.
 * @return количество найденных устройств
 */
uint8_t scanI2C() {
  uint8_t error;
  uint8_t address;
  uint8_t nDevices = 0;

  Serial.println(F("Сканирование I2C..."));

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F("Найдено: 0x"));
      if (address < 16) Serial.print('0');
      Serial.println(address, HEX);
      nDevices++;
    }
    else if (error == 4) {
      Serial.print(F("Ошибка на адресе 0x"));
      if (address < 16) Serial.print('0');
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println(F("Устройства не найдены"));
  } else {
    Serial.print(F("Всего найдено: "));
    Serial.println(nDevices);
  }

  Serial.println();
  return nDevices;
}

void doTarget(char* cmd) { command.scalar((float*)&target_velocity, cmd); }

// Только силовая команда. Гироскоп не трогаем — его поток нужен и при выключенном моторе.
void zeroMotorVoltage() {
  motor.P_angle.reset();
  motor.PID_velocity.reset();
  motor.current_sp = 0.0f;
  motor.voltage.q = 0.0f;
  motor.voltage.d = 0.0f;
  motion_out = 0.0f;
  motion_old_shaft_angle = NAN;
}

void syncSetpointToRotor() {
  if (motor.sensor) motor.sensor->update();
  motor.shaft_angle = motor.shaftAngle();
  motor.electrical_angle = motor.electricalAngle();
  position_setpoint = (double)motor.shaft_angle;
  motor.target = motor.shaft_angle;
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

  powerOn = 0;
  zeroMotorVoltage();
  syncSetpointToRotor();

  motor.enable();
  // enable() ставит PWM 0/0/0; сразу выставляем Uq=0 (центрированная синусоида),
  // иначе первый loopFOC() из задачи применит старый current_sp.
  motor.current_sp = 0.0f;
  motor.setPhaseVoltage(0.0f, 0.0f, motor.electrical_angle);

  powerOn = 1;
}

void doP(char* cmd) { command.scalar(&motor.P_angle.P, cmd); }
void doI(char* cmd) { command.scalar(&motor.P_angle.I, cmd); }
void doD(char* cmd) { command.scalar(&motor.P_angle.D, cmd); }

void doGyroP(char* cmd) { command.scalar(&gyro_pid.P, cmd); }
void doGyroI(char* cmd) { command.scalar(&gyro_pid.I, cmd); }
void doGyroD(char* cmd) { command.scalar(&gyro_pid.D, cmd); }

void doVLim(char* cmd) {
  command.scalar(&motor.voltage_limit, cmd);
  motor.PID_velocity.limit = motor.voltage_limit;
  motor.P_angle.limit = motor.voltage_limit;
}
void doVelLim(char* cmd) { command.scalar(&motor.velocity_limit, cmd); }

void doLfp(char* cmd) { command.scalar(&motor.LPF_angle.Tf, cmd); }
void doGyroLpf(char* cmd) { command.scalar(&gyro_lpf.Tf, cmd); }

void motorControlTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 1;

  for (;;) {
    if (powerOn) {
      position_setpoint += (double)target_velocity * CONTROL_DT;
      motor.loopFOC();
      motor.move((float)position_setpoint);
    } else if (motor.sensor) {
      motor.sensor->update();
      motor.shaft_angle = motor.shaftAngle();
      motor.shaft_velocity = motor.shaftVelocity();
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void GyroTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 1;

  for (;;) {
    float gyro[3], accel[3], temp;
    imu.read(gyro, accel, &temp);
    tmp_speed = -gyro[2] * 0.0174533f; // rad/s
    tmp_speed = gyro_lpf(tmp_speed);
    target_velocity = gyro_pid(tmp_speed);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

float stage2MotionControl(FOCMotor* m) {
  m->shaft_angle_sp = m->target;

  if (isnan(motion_old_shaft_angle) || motion_old_shaft_angle != m->shaft_angle) {
    filtred = m->shaft_angle;
    motion_out = m->P_angle(m->shaft_angle_sp - filtred);
    motion_old_shaft_angle = m->shaft_angle;
  }
  return motion_out;
}

void setup() {

  
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, CUSTOM_RX_PIN, CUSTOM_TX_PIN);

  Wire.begin(SDA_PIN, SCL_PIN, I2C_CLOCK );        // SDA=21, SCL=22 по умолчанию

  scanI2C();

  int8_t status = imu.begin(0x68);   // или 0x69, если AD0 = VCC
    if (status != ICM20602_OK) {
        Serial.printf("ICM20602 init failed: %d\n", status);
        while (true) delay(1000);
    }
    Serial.println("ICM20602 OK mode 1");

    




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

  gyro_pid.P = 1.0f;
  gyro_pid.I = 0.0f;
  gyro_pid.D = 0.0f;
  gyro_lpf.Tf = 0.0f;

  motor.voltage_limit = 4.0f;
  motor.PID_velocity.limit = motor.voltage_limit;
  motor.P_angle.limit = motor.voltage_limit;
  motor.P_angle.Ts = CONTROL_DT;
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
  command.add('A', doGyroP, "gyro_pid.P");
  command.add('B', doGyroI, "gyro_pid.I");
  command.add('C', doGyroD, "gyro_pid.D");
  command.add('L', doVLim, "voltage_limit");
  command.add('V', doVelLim, "velocity_limit");
  command.add('F', doLfp, "FilTER");
  command.add('E', doGyroLpf, "gyro_lpf.Tf");

  command.verbose = VerboseMode::nothing;

  Serial.println(F("ready"));
  _delay(200);

   xTaskCreatePinnedToCore(motorControlTask, "MotorCtrl", 4096, NULL, 5, NULL,
                           1);
  xTaskCreate(GyroTask, "Gyro", 4096, NULL, 5, NULL);

}

void loop() {
  command.run();



  //   if (imu.read(gyro, accel, &temp) == ICM20602_OK) {
  //       Serial.printf("Gyro:  %+7.2f %+7.2f %+7.2f  °/s\n", gyro[0], gyro[1], gyro[2]);
  //       Serial.printf("Accel: %+7.3f %+7.3f %+7.3f  g\n",   accel[0], accel[1], accel[2]);
  //       Serial.printf("Temp:  %+6.1f °C\n\n", temp);
  //   } else {
  //       Serial.println("Read error");
  //   }

  //
  Serial1.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", target_velocity,
                 motor.shaft_angle, motor.voltage.q, (float)position_setpoint,
                 filtred, filter2dval, motor.shaft_velocity,
                 (float)_micros() * 1e-6f);
}
