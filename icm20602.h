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

// Коды ошибок
#define ICM20602_OK              0
#define ICM20602_ERR_I2C         -1
#define ICM20602_ERR_ID          -2
#define ICM20602_ERR_CONFIG      -3

// Масштабы (как в оригинале)
#define ICM20602_ACCEL_FS_2G     0x00  // ±2g
#define ICM20602_GYRO_FS_250DPS  0x00  // ±250 dps
#define ICM20602_ACCEL_DLPF_BYP  0x09
#define ICM20602_GYRO_DLPF_20HZ  0x05

/**
 * Драйвер ICM-20602 для Arduino / PlatformIO (ESP32 и др.)
 * Адаптирован из STM32 HAL-версии (Shimanovich/post_stab)
 */
class ICM20602 {
public:
    ICM20602() = default;

    /**
     * @param addr  I2C-адрес (0x68 или 0x69)
     * @param wire  указатель на TwoWire (по умолчанию &Wire)
     * @return ICM20602_OK при успехе
     */
    int8_t begin(uint8_t addr = 0x68, TwoWire* wire = &Wire);

    /**
     * Чтение данных
     * @param gyro  [X,Y,Z] в °/с
     * @param accel [X,Y,Z] в g
     * @param temp  температура в °C
     */
    int8_t read(float gyro[3], float accel[3], float* temp);

private:
    TwoWire* _wire = nullptr;
    uint8_t  _addr = 0;

    int8_t write_reg(uint8_t reg, uint8_t value);
    int8_t read_regs(uint8_t reg, uint8_t* buffer, uint8_t len);
};

// ==================== РЕАЛИЗАЦИЯ ====================

int8_t ICM20602::begin(uint8_t addr, TwoWire* wire) {
    if (wire == nullptr) {
        return ICM20602_ERR_CONFIG;
    }

    _wire = wire;
    _addr = addr;

    // Wire уже должен быть инициализирован снаружи (Wire.begin(...))
    // либо можно раскомментировать следующую строку:
    // _wire->begin();

    uint8_t chip_id = 0;
    int8_t ret = read_regs(ICM20602_WHO_AM_I, &chip_id, 1);
    if (ret != ICM20602_OK || chip_id != ICM20602_WHO_AM_I_VALUE) {
        return ICM20602_ERR_ID;
    }

    // Сброс
    ret = write_reg(ICM20602_PWR_MGMT_1, 0x80);
    if (ret != ICM20602_OK) return ret;
    delay(100);

    // PLL
    ret = write_reg(ICM20602_PWR_MGMT_1, 0x01);
    if (ret != ICM20602_OK) return ret;

    // Standby accel + gyro
    ret = write_reg(ICM20602_PWR_MGMT_2, 0x3F);
    if (ret != ICM20602_OK) return ret;

    // FIFO off
    ret = write_reg(ICM20602_USER_CTRL, 0x00);
    if (ret != ICM20602_OK) return ret;

    // Accel ±2g + bypass DLPF
    ret = write_reg(ICM20602_ACCEL_CONFIG_2, ICM20602_ACCEL_DLPF_BYP);
    if (ret != ICM20602_OK) return ret;
    ret = write_reg(ICM20602_ACCEL_CONFIG, ICM20602_ACCEL_FS_2G << 3);
    if (ret != ICM20602_OK) return ret;

    // Gyro config
    ret = write_reg(ICM20602_CONFIG, 0x03);
    if (ret != ICM20602_OK) return ret;
    ret = write_reg(ICM20602_GYRO_CONFIG, (ICM20602_GYRO_FS_250DPS << 3) | 0x00);
    if (ret != ICM20602_OK) return ret;

    // Sample rate divider
    ret = write_reg(ICM20602_SMPLRT_DIV, 0x00);
    if (ret != ICM20602_OK) return ret;

    // Enable accel + gyro
    ret = write_reg(ICM20602_PWR_MGMT_2, 0x00);
    if (ret != ICM20602_OK) return ret;

    delay(50);
    return ICM20602_OK;
}

int8_t ICM20602::read(float gyro[3], float accel[3], float* temp) {
    if (_wire == nullptr) {
        return ICM20602_ERR_CONFIG;
    }

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

    // Чувствительности как в оригинале
    const float accel_sensitivity = 16384.0f;  // ±2g
    const float gyro_sensitivity  = 131.0f;    // ±250 dps

    accel[0] = ax / accel_sensitivity;
    accel[1] = ay / accel_sensitivity;
    accel[2] = az / accel_sensitivity;

    gyro[0] = gx / gyro_sensitivity;
    gyro[1] = gy / gyro_sensitivity;
    gyro[2] = gz / gyro_sensitivity;

    *temp = t_raw / 326.8f + 12.0f;

    return ICM20602_OK;
}

int8_t ICM20602::write_reg(uint8_t reg, uint8_t value) {
    if (_wire == nullptr) return ICM20602_ERR_CONFIG;

    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(value);
    if (_wire->endTransmission() != 0) {
        return ICM20602_ERR_I2C;
    }
    return ICM20602_OK;
}

int8_t ICM20602::read_regs(uint8_t reg, uint8_t* buffer, uint8_t len) {
    if (_wire == nullptr) return ICM20602_ERR_CONFIG;

    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) {   // false = repeated start
        return ICM20602_ERR_I2C;
    }

    uint8_t received = _wire->requestFrom(_addr, len);
    if (received != len) {
        return ICM20602_ERR_I2C;
    }

    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = _wire->read();
    }
    return ICM20602_OK;
}

#endif // ICM20602_H