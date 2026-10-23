#ifndef JV_KYT_G03_H
#define JV_KYT_G03_H

#include <Arduino.h>

// Протокол гироскопа JV-KYT-G03-V100
// RS422, 921600 бод, 8E1, пакет 21 байт

class JvKytG03 {
 public:
  static constexpr int PACKET_SIZE = 21;
  static constexpr float SCALE =
      (100.0f / 67108864.0f) * (3.14159265f / 180.0f);  // ±100

  struct Data {
    float x;     // °/с
    float y;     // °/с
    float z;     // °/с
    float temp;  // °C
    bool valid;  // CRC OK
  };

  JvKytG03() : _bufLen(0), _biasX(0), _biasY(0), _biasZ(0) {}

  // Обработка входящего байта, возвращает true если пакет готов
  bool feed(uint8_t byte, Data& out) {
    if (_bufLen < sizeof(_buf)) {
      _buf[_bufLen++] = byte;
    } else {
      memmove(_buf, _buf + 1, sizeof(_buf) - 1);
      _buf[sizeof(_buf) - 1] = byte;
    }

    if (_bufLen < PACKET_SIZE) return false;

    for (int i = 0; i <= _bufLen - PACKET_SIZE; i++) {
      if (_buf[i] == 0x80) {
        uint8_t* pkt = &_buf[i];

        out.valid = checkCRC(pkt);
        if (out.valid) {
          int32_t rawX = decodeAxis(pkt[1], pkt[2], pkt[3], pkt[4]);
          int32_t rawY = decodeAxis(pkt[5], pkt[6], pkt[7], pkt[8]);
          int32_t rawZ = decodeAxis(pkt[9], pkt[10], pkt[11], pkt[12]);

          out.x = rawX * SCALE - _biasX;
          out.y = rawY * SCALE - _biasY;
          out.z = rawZ * SCALE - _biasZ;

          int16_t tempRaw = ((int16_t)pkt[13] << 8) | pkt[14];
          out.temp = tempRaw / 30.0f;
        }

        // Удаляем обработанный пакет
        int remain = _bufLen - (i + PACKET_SIZE);
        if (remain > 0) memmove(_buf, &_buf[i + PACKET_SIZE], remain);
        _bufLen = remain;

        return out.valid;
      }
    }
    return false;
  }

  // Калибровка смещения (вызвать в покое)
  void calibrate(const Data& d) {
    _biasX = d.x + _biasX;
    _biasY = d.y + _biasY;
    _biasZ = d.z + _biasZ;
  }

  void resetBias() { _biasX = _biasY = _biasZ = 0; }

 private:
  uint8_t _buf[64];
  int _bufLen;
  float _biasX, _biasY, _biasZ;

  static int32_t decodeAxis(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t s) {
    int32_t base = ((int32_t)b0 << 16) | ((int32_t)b1 << 8) | b2;
    if (base >= 0x400000) base -= 0x800000;
    return (base << 3) | (s & 0x07);
  }

  static bool checkCRC(const uint8_t* pkt) {
    uint8_t crc = 0;
    for (int i = 1; i < 20; i++) crc ^= pkt[i];
    return crc == pkt[20];
  }
};

#endif  // JV_KYT_G03_H