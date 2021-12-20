/**
 * The implementation of all of these methods return 0.
 *
 * 980F modified the multi() returns to reverse byte order on the fly if count is negative.
 **/
#include <Wire.h>

/**
 * declarations for user supplied interface to i2c or spi bus.
 *
 * This is an Arduino i2c implementation.
 *
 * The VL53 is mostly bigendian, with a few exceptions, and the Arduino's all seem to be littlendian.
 * If you have a bigendian platform you will have to work on the swapendians management
 * */

struct Arg { //the class name here should be made a bit more specific ;)
  TwoWire &i2c;
  uint8_t devAddr;
  uint16_t comms_speed_khz = 400;
  Arg(Arg &&args):i2c(i2c),devAddr(devAddr),comms_speed_khz(comms_speed_khz){

  }
} ;


class ArduinoWirer : public Arg { //inheriting as a cheap way to not have to edit as much existing code while still hiding the details of the constructor args list.
public:
  ArduinoWirer(Arg &&arg):Arg(std::forward<Arg>(arg)){}

  /** one can change the device address, needed for when more than one is on the same i2c link.
   * you will also have to control a pin and that is getting well out of the scope of this layer */
  uint8_t changedAddress(uint8_t  newAddress);
// initialize I2C
  void i2c_init();
/** if @param count is negative then pdata will be byte reversed */
  bool write_multi(uint8_t index, const uint8_t *pdata, int count);
/** if @param count is negative then pdata will be byte reversed */
  bool read_multi(uint8_t index, uint8_t *pdata, int count);

  /** writes value from stack, copied into internal buffer  */
  template<typename Scalar> bool Write(uint8_t index, Scalar data, bool swapendians = false) {
    return write_multi(index, reinterpret_cast<uint8_t *>(&data), sizeof(Scalar) * (swapendians ? -1 : 1));
  }

  /** reads into variable passed by address */
  template<typename Scalar> bool Read(uint8_t index, Scalar &data, bool swapendians = false) {
    return read_multi(index, reinterpret_cast<uint8_t *>(data), sizeof(Scalar) * (swapendians ? -1 : 1));
  }

};

