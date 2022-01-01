// Copyright 2021 by Andy Heilveil (github/980f)

/** implementation for vl53l0x_i2c_platform.h, given a spurious name for legacy reasons */

#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_platform_log.h"

#include <Arduino.h>

#define I2C_DEBUG 0
#include <Wire.h>


#include "trynester.h" //fancier than Exceptor



static TwoWire * const busses[]={
#if WIRE_INTERFACES_COUNT > 0
  &Wire
#endif
#if WIRE_INTERFACES_COUNT > 1
  , &Wire1
#endif
#if WIRE_INTERFACES_COUNT > 2
  , &Wire2
#endif
#if WIRE_INTERFACES_COUNT > 3
  , &Wire3
#endif
#if WIRE_INTERFACES_COUNT > 4
  , &Wire4
#endif
#if WIRE_INTERFACES_COUNT > 5
  , &Wire5
#endif
} ;

///** RAII widget to allow us easy bailout on tx buffer overflow was contrary to what was desired. */

bool ArduinoWirer::write_multi(uint8_t index, const uint8_t *pdata, int count) {
  tracer.count = count;
  tracer.index = index;
  tracer.value = *reinterpret_cast<const uint32_t *>(pdata);//sometimes garbage is accessed and stored.
  readError=0;
  TwoWire &i2c(*busses[busNumber]);
  i2c.beginTransmission(devAddr);///FYI no action takes place until endTransmission
  if (i2c.write(index) != 1) {
    THROW(VL53L0X::ERROR_CONTROL_INTERFACE);//tx buffer overflow
  }
#if I2C_DEBUG
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
        THROW(VL53L0X::ERROR_CONTROL_INTERFACE);//tx buffer overflow
      }
    }
  } else {
    if (i2c.write(pdata, count) != count) {
      THROW(VL53L0X::ERROR_CONTROL_INTERFACE);//tx buffer overflow
    }
  }
#if I2C_DEBUG
  Serial.print("0x");
  Serial.print(pdata[0], HEX);
  Serial.print(", ");
#endif

#if I2C_DEBUG
  Serial.println();
#endif
  readError = i2c.endTransmission();//#NB: this here is what takes all the time when sending.
  if (readError != 0) {
    readError=-readError;//mark as a write
    THROW(VL53L0X::ERROR_CONTROL_INTERFACE);//xmission failure, such as unexpected NAK.
  }
  return true;
}

bool ArduinoWirer::read_multi(uint8_t index, uint8_t *pdata, int count) {
  bool reversing = count < 0;
  if (reversing) {
    count = -count;//now positive
    pdata += count;//past end
  }
  TwoWire &i2c(*busses[busNumber]);
  i2c.beginTransmission(devAddr);
  i2c.write(index);
  i2c.endTransmission();
  auto didit = i2c.requestFrom(devAddr, count);
  readError= count-didit;
  if (readError) {
    THROW(VL53L0X::ERROR_CONTROL_INTERFACE);//wrong sized read
  }
#if I2C_DEBUG
  Serial.print("\tReading ");
  Serial.print(count);
  Serial.print(" from addr 0x");
  Serial.print(index, HEX);
  Serial.print(": ");
#endif
  if (reversing) {
    while (count--) {
      pdata[count] = i2c.read();
#if I2C_DEBUG
      Serial.print("0x");
      Serial.print(pdata[-1], HEX);
      Serial.print(", ");
#endif
    }
  } else {
    while (count--) {
      *pdata++ = i2c.read();
#if I2C_DEBUG
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
  TwoWire &i2c(*busses[busNumber]);
  i2c.begin();
  i2c.setClock(1000 * comms_speed_khz);
  readError=0;
  tracer.clear();
}

void WriteOperation::clear() {
  this->value=0;//keeping it clean, count is all that really matters.
  this->index=0;//all values are legal, but we'd rather not have garbage
  this->count=0;//indicates no-op
}
