/**
 * The implementation of all of these methods return 0.
 *
 * 980F modified the multi() returns to reverse byte order on the fly if count is negative.
 **/
#include <Wire.h>
/**
 * user supplied interface to i2c or spi bus.
 * This is an Arduino i2c implementation.
 *
 * The VL53 is mostly bigendian, with a few exceptions, and the Arduino's all seem to be littlendian.
 * */
namespace Arduinoi {
// initialize I2C
  void i2c_init(TwoWire& i2c,unsigned kHz=400);
/** if @param count is negative then pdata will be byte reversed */
  bool write_multi(TwoWire& i2c,uint8_t deviceAddress, uint8_t index, uint8_t *pdata, int count);
/** if @param count is negative then pdata will be byte reversed */
  bool read_multi(TwoWire& i2c,uint8_t deviceAddress, uint8_t index, uint8_t *pdata, int count);

  template <typename Scalar> bool Write(TwoWire& bus,uint8_t deviceAddress,uint8_t index,Scalar data,bool swapendians= false){
    return write_multi(bus, deviceAddress, index, reinterpret_cast<uint8_t *>(data), sizeof (Scalar) * (swapendians ? -1 : 1));
  }

  template <typename Scalar> bool Read(TwoWire& bus,uint8_t deviceAddress,uint8_t index,Scalar data,bool swapendians= false){
    return read_multi(bus, deviceAddress, index, reinterpret_cast<uint8_t *>(data), sizeof (Scalar) * (swapendians ? -1 : 1));
  }
}
