// Copyright 2021 by Andy Heilveil (github/980f)
#pragma once
/**
 * The implementation of all of these methods return 0.
 *
 * 980F modified the multi() returns to reverse byte order on the fly if count is negative.
 **/
#include <Wire.h>
#include <utility>

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

  Arg(Arg &&args) : i2c(i2c), devAddr(devAddr), comms_speed_khz(comms_speed_khz) {
  }

  Arg(TwoWire &i2C, uint8_t devAddr, uint16_t commsSpeedKhz) : i2c(i2C), devAddr(devAddr), comms_speed_khz(commsSpeedKhz) {
  }
};

#include <csetjmp>

struct Exceptor {
  //written by setjmp:
  jmp_buf opaque;
  //written by throw:
  const char *location = "";
  unsigned line = 0;
  int errorcode = 0;

  /** record source location info and then longjmp.
   *
   * using operator parens makes for easier replacement by a macro */
  void operator()(const char *location, unsigned line, int error) {
    this->location = location;
    this->line = line;
    errorcode = error;
    longjmp(opaque, errorcode); // NOLINT(cert-err52-cpp)   exceptions not allowed on our platform
  }
};

struct WriteOperation {
  uint8_t index;
  uint8_t count;
  //the following will be just the first 4 bytes of a longer operation.
  uint32_t value;
};

class ArduinoWirer : public Arg { //inheriting as a cheap way to not have to edit as much existing code while still hiding the details of the constructor args list.
public:
  ArduinoWirer(Arg &&arg) : Arg(std::forward<Arg>(arg)) {
  }

  Exceptor Throw;
  WriteOperation tracer;
  /** one can change the device address, needed for when more than one is on the same i2c link.
   * you will also have to control a pin and that is getting well out of the scope of this layer */
  uint8_t changeAddress(uint8_t newAddress);
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

