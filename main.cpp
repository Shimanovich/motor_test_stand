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
int powerOn = 0;

double position_setpoint = 0.0;


float filtred = 0.0;

float filter2dval = 0.0f;

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

void GyroTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 1;

  for (;;) {
    float gyro[3], accel[3], temp;
    imu.read(gyro, accel, &temp);
    target_velocity = -gyro[2] * 0.0174533f; // rad/s

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

float stage2MotionControl(FOCMotor* m) {
  static float cur_out = 0;
  float oldshaft_angle;
  m->shaft_angle_sp = m->target;

  if (oldshaft_angle != m->shaft_angle) {
    filtred = m->shaft_angle;
    //cur_out = m->LPF_angle(m->P_angle(m->shaft_angle_sp - filtred));
    cur_out = m->P_angle(m->shaft_angle_sp - filtred);
    oldshaft_angle = m->shaft_angle;
  }
  return cur_out;
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
