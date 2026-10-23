#pragma once

#define GYRO_RX_PIN 33
#define GYRO_TX_PIN -1  // TX не используется

#define GYRO_UART_NUM UART_NUM_2
#define GYRO_BUF_SIZE 1024

// Пины
#define CUSTOM_RX_PIN 16
#define CUSTOM_TX_PIN 17

#define SENSOR_1_CS_PIN 5
#define SENSOR_2_CS_PIN 13

#define DRIVER_YAW_PWM_A 4
#define DRIVER_YAW_PWM_B 23
#define DRIVER_YAW_PWM_C 2
#define DRIVER_YAW_EN 27

#define DRIVER_PITCH_PWM_A 12
#define DRIVER_PITCH_PWM_B 14
#define DRIVER_PITCH_PWM_C 32
#define DRIVER_PITCH_EN 33

#define SDA_PIN 25
#define SCL_PIN 26
#define I2C_CLOCK 400000

// Параметры мотора
#define MOTOR_PP 7
#define MOTOR_R 2.3f
#define MOTOR_KV 220.0f
#define MOTOR_L 0.00086f
#define POWER_SUPPLY 12.0f
#define VOLTAGE_LIMIT 6.0f

// Управление 2 кГц
#define CONTROL_HZ 2000
#define CONTROL_DT 0.0005f

// === Маппинг осей IMU (настроить под ориентацию датчика) ===
// gyro[0] = X, gyro[1] = Y, gyro[2] = Z (deg/s)
// X параллельна оси pitch-мотора
#define IMU_AXIS_PITCH 0  // X
#define IMU_AXIS_YAW 2    // Z (поменять на 1 если нужно)
#define IMU_SIGN_PITCH -1.0f
#define IMU_SIGN_YAW 1.0f

// Комплементарный фильтр (pitch к горизонту)
#define COMPLEMENTARY_ALPHA 0.98f  // вес гироскопа (0.95–0.995)
#define ACCEL_PITCH_SIGN 1.0f      // знак atan2, подстроить по факту
