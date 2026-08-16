#pragma once

// Пины


#define CUSTOM_RX_PIN 16
#define CUSTOM_TX_PIN 17
#define SENSOR_CS_PIN 5
#define DRIVER_PWM_A 4
#define DRIVER_PWM_B 23
#define DRIVER_PWM_C 2
#define DRIVER_EN 27






#define SDA_PIN 25
#define SCL_PIN 26
#define I2C_CLOCK 400000

// Второй IMU (рама)
#define FRAME_SDA_PIN 21
#define FRAME_SCL_PIN 22
#define I2C_FRAME_CLOCK 400000

// Параметры мотора
#define MOTOR_PP 7
#define MOTOR_R 2.3f
#define MOTOR_KV 220.0f
#define MOTOR_L 0.00086f
#define POWER_SUPPLY 12.0f
#define VOLTAGE_LIMIT 6.0f

// Управление
#define CONTROL_HZ 1000
#define CONTROL_DT 0.001
