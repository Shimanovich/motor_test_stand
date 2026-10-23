#ifndef ICM20602_H
#define ICM20602_H

#include <Arduino.h>
#include <Wire.h>
#include <cstdint>

// Регистры
#define ICM20602_WHO_AM_I        0x75
#define ICM20602_WHO_AM_I_VALUE  0x12

#define ICM20602_PWR_MGMT_1      0x6B
#define ICM20602_PWR_MGMT_2      0x6C
#define ICM20602_USER_CTRL       0x6A
#define ICM20602_ACCEL_CONFIG    0x1C
#define ICM20602_ACCEL_CONFIG_2  0x1D
#define ICM20602_GYRO_CONFIG     0x1B
#define ICM20602_CONFIG          0x1A
#define ICM20602_SMPLRT_DIV      0x19
#define ICM20602_FIFO_EN         0x23
#define ICM20602_ACCEL_XOUT_H    0x3B
#define ICM20602_TEMP_OUT_H      0x41
#define ICM20602_GYRO_XOUT_H     0x43

#define ICM20602_OK              0
#define ICM20602_ERR_I2C         -1
#define ICM20602_ERR_ID          -2
#define ICM20602_ERR_CONFIG      -3

#define ICM20602_ACCEL_FS_2G     0x00
#define ICM20602_GYRO_FS_250DPS  0x00
#define ICM20602_ACCEL_DLPF_BYP  0x09

class ICM20602 {
public:
    ICM20602() = default;

    int8_t begin(uint8_t addr = 0x68, TwoWire* wire = &Wire);
    int8_t read(float gyro[3], float accel[3], float* temp);
    // Только гироскоп, 6 байт — быстрее для 2 кГц
    int8_t readGyro(float gyro[3]);

private:
    TwoWire* _wire = nullptr;
    uint8_t  _addr = 0;

    int8_t write_reg(uint8_t reg, uint8_t value);
    int8_t read_regs(uint8_t reg, uint8_t* buffer, uint8_t len);
};

inline int8_t ICM20602::begin(uint8_t addr, TwoWire* wire) {
    if (wire == nullptr) return ICM20602_ERR_CONFIG;

    _wire = wire;
    _addr = addr;

    uint8_t chip_id = 0;
    int8_t ret = read_regs(ICM20602_WHO_AM_I, &chip_id, 1);
    if (ret != ICM20602_OK || chip_id != ICM20602_WHO_AM_I_VALUE) {
        return ICM20602_ERR_ID;
    }

    ret = write_reg(ICM20602_PWR_MGMT_1, 0x80);
    if (ret != ICM20602_OK) return ret;
    delay(100);

    ret = write_reg(ICM20602_PWR_MGMT_1, 0x01);   // PLL
    if (ret != ICM20602_OK) return ret;

    ret = write_reg(ICM20602_PWR_MGMT_2, 0x3F);   // sleep all
    if (ret != ICM20602_OK) return ret;

    ret = write_reg(ICM20602_USER_CTRL, 0x00);
    if (ret != ICM20602_OK) return ret;

    // Accel: ±2g, DLPF bypass
    ret = write_reg(ICM20602_ACCEL_CONFIG_2, ICM20602_ACCEL_DLPF_BYP);
    if (ret != ICM20602_OK) return ret;
    ret = write_reg(ICM20602_ACCEL_CONFIG, ICM20602_ACCEL_FS_2G << 3);
    if (ret != ICM20602_OK) return ret;

    // Gyro: более широкая полоса (DLPF ~188 Hz), чтобы данные обновлялись чаще
    // CONFIG: 0x01 ≈ 188 Hz gyro DLPF, output rate 1 kHz
    // Для ещё более свежих данных можно поставить 0x00 (bypass, 8 kHz)
    ret = write_reg(ICM20602_CONFIG, 0x01);
    if (ret != ICM20602_OK) return ret;
    ret = write_reg(ICM20602_GYRO_CONFIG, (ICM20602_GYRO_FS_250DPS << 3));
    if (ret != ICM20602_OK) return ret;

    // Делитель 0 → полная частота выдачи
    ret = write_reg(ICM20602_SMPLRT_DIV, 0x00);
    if (ret != ICM20602_OK) return ret;

    ret = write_reg(ICM20602_PWR_MGMT_2, 0x00);   // enable all
    if (ret != ICM20602_OK) return ret;

    delay(50);
    return ICM20602_OK;
}

inline int8_t ICM20602::read(float gyro[3], float accel[3], float* temp) {
    if (_wire == nullptr) return ICM20602_ERR_CONFIG;

    uint8_t buffer[14];
    int8_t ret = read_regs(ICM20602_ACCEL_XOUT_H, buffer, 14);
    if (ret != ICM20602_OK) return ret;

    int16_t ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t az = (int16_t)((buffer[4] << 8) | buffer[5]);
    int16_t t_raw = (int16_t)((buffer[6] << 8) | buffer[7]);
    int16_t gx = (int16_t)((buffer[8] << 8) | buffer[9]);
    int16_t gy = (int16_t)((buffer[10] << 8) | buffer[11]);
    int16_t gz = (int16_t)((buffer[12] << 8) | buffer[13]);

    const float accel_sensitivity = 16384.0f;
    const float gyro_sensitivity  = 131.0f;

    accel[0] = ax / accel_sensitivity;
    accel[1] = ay / accel_sensitivity;
    accel[2] = az / accel_sensitivity;

    gyro[0] = gx / gyro_sensitivity;
    gyro[1] = gy / gyro_sensitivity;
    gyro[2] = gz / gyro_sensitivity;

    if (temp) *temp = t_raw / 326.8f + 12.0f;
    return ICM20602_OK;
}

inline int8_t ICM20602::readGyro(float gyro[3]) {
    if (_wire == nullptr) return ICM20602_ERR_CONFIG;

    uint8_t buffer[6];
    int8_t ret = read_regs(ICM20602_GYRO_XOUT_H, buffer, 6);
    if (ret != ICM20602_OK) return ret;

    int16_t gx = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t gy = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t gz = (int16_t)((buffer[4] << 8) | buffer[5]);

    const float gyro_sensitivity = 131.0f;
    gyro[0] = gx / gyro_sensitivity;
    gyro[1] = gy / gyro_sensitivity;
    gyro[2] = gz / gyro_sensitivity;
    return ICM20602_OK;
}

inline int8_t ICM20602::write_reg(uint8_t reg, uint8_t value) {
    if (_wire == nullptr) return ICM20602_ERR_CONFIG;
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(value);
    if (_wire->endTransmission() != 0) return ICM20602_ERR_I2C;
    return ICM20602_OK;
}

inline int8_t ICM20602::read_regs(uint8_t reg, uint8_t* buffer, uint8_t len) {
    if (_wire == nullptr) return ICM20602_ERR_CONFIG;

    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return ICM20602_ERR_I2C;

    uint8_t received = _wire->requestFrom(_addr, len);
    if (received != len) return ICM20602_ERR_I2C;

    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = _wire->read();
    }
    return ICM20602_OK;
}

#endif // ICM20602_H
