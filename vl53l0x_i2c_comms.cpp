#include "vl53l0x_i2c_platform.h"

//bug: module presumes endianess mismatch
//BUG: errors can be detected and most of them will be rather fatal, but all are ignored

//#define I2C_DEBUG

int VL53L0X_i2c_init(TwoWire *i2c) {
  i2c->begin();
  return 0;
}

int VL53L0X_write_multi(uint8_t deviceAddress, uint8_t index, uint8_t *pdata, int count, TwoWire *i2c) {
  i2c->beginTransmission(deviceAddress);
  i2c->write(index);
#ifdef I2C_DEBUG
  Serial.print("\tWriting ");
  Serial.print(count);
  Serial.print(" to addr 0x");
  Serial.print(index, HEX);
  Serial.print(": ");
#endif
  if (count < 0) {
    count = -count;//now positive
    pdata += count;//past end
    while (count-- > 0) {
      i2c->write(pdata[count]);
    }
  } else {
    i2c->write(pdata, count);
  }
#ifdef I2C_DEBUG
  Serial.print("0x");
  Serial.print(pdata[0], HEX);
  Serial.print(", ");
#endif

#ifdef I2C_DEBUG
  Serial.println();
#endif
  i2c->endTransmission();
  return 0;
}

int VL53L0X_read_multi(uint8_t deviceAddress, uint8_t index, uint8_t *pdata, int count, TwoWire *i2c) {
  bool reversing = count < 0;
  if (reversing) {
    count = -count;//now positive
    pdata += count;//past end
  }
  i2c->beginTransmission(deviceAddress);
  i2c->write(index);
  i2c->endTransmission();
  i2c->requestFrom(deviceAddress, (byte) count);
#ifdef I2C_DEBUG
  Serial.print("\tReading ");
  Serial.print(count);
  Serial.print(" from addr 0x");
  Serial.print(index, HEX);
  Serial.print(": ");
#endif
  if (reversing) {
    while (count--) {
      pdata[count] = i2c->read();
#ifdef I2C_DEBUG
      Serial.print("0x");
      Serial.print(pdata[-1], HEX);
      Serial.print(", ");
#endif
    }
  } else {
    while (count--) {
      *pdata++ = i2c->read();
#ifdef I2C_DEBUG
      Serial.print("0x");
      Serial.print(pdata[-1], HEX);
      Serial.print(", ");
#endif
    }
  }
#ifdef I2C_DEBUG
  Serial.println();
#endif
  return 0;
}

int VL53L0X_write_byte(uint8_t deviceAddress, uint8_t index, uint8_t data, TwoWire *i2c) {
  return VL53L0X_write_multi(deviceAddress, index, &data, 1, i2c);
}

int VL53L0X_write_word(uint8_t deviceAddress, uint8_t index, uint16_t data, TwoWire *i2c) {
  return VL53L0X_write_multi(deviceAddress, index, reinterpret_cast<uint8_t *>(data), -2, i2c);
}

int VL53L0X_write_dword(uint8_t deviceAddress, uint8_t index, uint32_t data, TwoWire *i2c) {
  return VL53L0X_write_multi(deviceAddress, index, reinterpret_cast<uint8_t *>(data), -4, i2c);
}

int VL53L0X_read_byte(uint8_t deviceAddress, uint8_t index, uint8_t *data, TwoWire *i2c) {
  return VL53L0X_read_multi(deviceAddress, index, data, 1, i2c);
}

int VL53L0X_read_word(uint8_t deviceAddress, uint8_t index, uint16_t *data, TwoWire *i2c) {
  return VL53L0X_read_multi(deviceAddress, index, reinterpret_cast<uint8_t *>(data), -2, i2c);
}

int VL53L0X_read_dword(uint8_t deviceAddress, uint8_t index, uint32_t *data, TwoWire *i2c) {
  return VL53L0X_read_multi(deviceAddress, index, reinterpret_cast<uint8_t *>(data), -4, i2c);
}
