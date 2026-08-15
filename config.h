#pragma once

// Пины

#if 1
#define CUSTOM_RX_PIN 16
#define CUSTOM_TX_PIN 17
#define SENSOR_CS_PIN 5
#define DRIVER_PWM_A 4
#define DRIVER_PWM_B 21
#define DRIVER_PWM_C 2
#define DRIVER_EN 27
#endif

#if 0
#define CUSTOM_RX_PIN 16
#define CUSTOM_TX_PIN 17
#define SENSOR_CS_PIN 5
#define DRIVER_PWM_A 12
#define DRIVER_PWM_B 13
#define DRIVER_PWM_C 14
#define DRIVER_EN 27
#endif

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
