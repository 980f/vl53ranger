#include "vl53l0x_i2c_platform.h"
#include <Wire.h>
i2c.beginTransmission(deviceAddress);

//#define I2C_DEBUG
namespace Arduinoi { //misspelled in case one already exist
  
void i2c_init(TwoWire& i2c) {
  i2c.begin();
}

/** RAII widget to allow us easy bailout on unexpected NAK's from device. */
class I2cFramer{
  TwoWire& i2c;
public:
  I2cFramer(TwoWire& i2c,uint8_t deviceAddress):i2c(i2c){
    i2c.beginTransmission(deviceAddress);
  }
  ~I2cFramer(){
    i2c.endTransmission();//#NB: this here is what takes all the time when sending.
  }
};

bool write_multi(TwoWire& i2c,uint8_t deviceAddress, uint8_t index, uint8_t *pdata, int count) {
  I2cFramer frameit(i2c,deviceAddress);
  if(i2c.write(index)!=1) return false;
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
      if(i2c.write(pdata[count])!=1){
        return false;
      }
    }
  } else {
    if(i2c.write(pdata, count) !=count){
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

bool read_multi(TwoWire& i2c, uint8_t deviceAddress, uint8_t index, uint8_t *pdata, int count) {
  bool reversing = count < 0;
  if (reversing) {
    count = -count;//now positive
    pdata += count;//past end
  }
  i2c.beginTransmission(deviceAddress);
  i2c.write(index);
  i2c.endTransmission();
  auto didit=i2c.requestFrom(deviceAddress,  count);
  if(didit!=count){
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

} 
