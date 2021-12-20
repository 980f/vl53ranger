//Copyright 2021 Andrew Heilveil, github/980f

/** implementation for vl53l0x_i2c_platform.h, given a spurious name for legacy reasons */

#include "vl53l0x_i2c_platform.h"
#include <Wire.h>

//#define I2C_DEBUG

uint8_t ArduinoWirer::changedAddress(uint8_t newAddress) {
  uint8_t was = devAddr;
  devAddr = newAddress;
  return was;
}

/** RAII widget to allow us easy bailout on unexpected NAK's from device. */
class I2cFramer {
  ArduinoWirer &parent;
public:
  I2cFramer(ArduinoWirer &parent) : parent(parent) {
    parent.i2c.beginTransmission(parent.devAddr);
  }

  ~I2cFramer() {
    parent.i2c.endTransmission();//#NB: this here is what takes all the time when sending.
  }
};

bool ArduinoWirer::write_multi(uint8_t index, const uint8_t *pdata, int count) {
  I2cFramer frameit(*this);//begins transmission and arranges to end it regardless of how we exit this method.
  if (i2c.write(index) != 1) {
    return false;
  }
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
      if (i2c.write(pdata[count]) != 1) {
        return false;
      }
    }
  } else {
    if (i2c.write(pdata, count) != count) {
      return false;
    }
  }
#ifdef I2C_DEBUG
  Serial.print("0x");
  Serial.print(pdata[0], HEX);
  Serial.print(", ");
#endif

#ifdef I2C_DEBUG
  Serial.println();
#endif
  return true;
}

bool ArduinoWirer::read_multi(uint8_t index, uint8_t *pdata, int count) {
  bool reversing = count < 0;
  if (reversing) {
    count = -count;//now positive
    pdata += count;//past end
  }
    i2c.beginTransmission(devAddr);
  i2c.write(index);
  i2c.endTransmission();
  auto didit = i2c.requestFrom(devAddr, count);
  if (didit != count) {
    return false;
  }
#ifdef I2C_DEBUG
  Serial.print("\tReading ");
  Serial.print(count);
  Serial.print(" from addr 0x");
  Serial.print(index, HEX);
  Serial.print(": ");
#endif
  if (reversing) {
    while (count--) {
      pdata[count] = i2c.read();
#ifdef I2C_DEBUG
      Serial.print("0x");
      Serial.print(pdata[-1], HEX);
      Serial.print(", ");
#endif
    }
  } else {
    while (count--) {
      *pdata++ = i2c.read();
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
  return true;
}

void ArduinoWirer::i2c_init() {
  i2c.begin();
  i2c.setClock(1000* comms_speed_khz);
}

