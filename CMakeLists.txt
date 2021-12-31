CMAKE_MINIMUM_REQUIRED(VERSION 3.20)

#until linker issue is resolved:  (this must preceded PROJECT() as project() is what triggers the compiler testing)
SET(CMAKE_C_COMPILER_FORCED true)
SET(CMAKE_CXX_COMPILER_FORCED true)

PROJECT(vl53)

SET(CMAKE_CXX_STANDARD 17)

ADD_COMPILE_DEFINITIONS(
  ARDUINO=10817
  F_CPU=48000000L
  CRYSTALLESS
  ARDUINO_ARCH_SAMD
  ARDUINO_SAMD_ADAFRUIT
  ARDUINO_SAMD_ZERO
  ARM_MATH_CM0PLUS

  ARDUINO_QTPY_M0
  ADAFRUIT_QTPY_M0
  __SAMD21E18A__

  VL53L0X_LOG_ENABLE=1
  LOG_GET_TIME=micros
)

INCLUDE_DIRECTORIES(.
  /home/andyh/.arduino15/packages/adafruit/hardware/samd/1.7.5/cores/arduino/
  /home/andyh/.arduino15/packages/adafruit/hardware/samd/1.7.5/libraries/Wire
  /home/andyh/.arduino15/packages/adafruit/tools/CMSIS/5.4.0/CMSIS/Core/Include/
  /home/andyh/.arduino15/packages/adafruit/tools/CMSIS/5.4.0/CMSIS/DSP/Include/
  /home/andyh/.arduino15/packages/adafruit/tools/CMSIS-Atmel/1.2.2/CMSIS/Device/ATMEL/
  /home/andyh/.arduino15/packages/adafruit/hardware/samd/1.7.5/variants/qtpy_m0/
  )

SET(coreSources
  vl53l0x_api.cpp
  vl53l0x_api_calibration.cpp
  vl53l0x_api_core.cpp

  vl53l0x_api_strings.cpp
  vl53l0x_spadarray.cpp
  )

SET(platform
  vl53l0x_i2c_comms.cpp
  vl53l0x_platform.cpp
  /home/andyh/.arduino15/packages/adafruit/hardware/samd/1.7.5/libraries/Wire/Wire.cpp
  /home/andyh/.arduino15/packages/adafruit/hardware/samd/1.7.5/cores/arduino/Print.cpp
  /home/andyh/.arduino15/packages/adafruit/hardware/samd/1.7.5/cores/arduino/USB/CDC.cpp
  /home/andyh/.arduino15/packages/adafruit/hardware/samd/1.7.5/cores/arduino/USB/PluggableUSB.cpp
  /home/andyh/.arduino15/packages/adafruit/hardware/samd/1.7.5/cores/arduino/USB/SAMD21_USBDevice.cpp
  /home/andyh/.arduino15/packages/adafruit/hardware/samd/1.7.5/cores/arduino/USB/USBCore.cpp
  )

SET(adafruit
  Adafruit_VL53L0X.cpp
  )

SET(newapi
  nonblocking.cpp
  trynester.cpp
  )

ADD_EXECUTABLE(vl53 ${platform} ${coreSources} ${adafruit} ${newapi})
