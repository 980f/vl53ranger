CMAKE_MINIMUM_REQUIRED(VERSION 3.20)

#until linker issue is resolved:  (this must preceded PROJECT() as project() is what triggers the compiler testing)
set(CMAKE_C_COMPILER_FORCED true)
set(CMAKE_CXX_COMPILER_FORCED true)

PROJECT(vl53)

SET(CMAKE_CXX_STANDARD 17)

INCLUDE_DIRECTORIES(. inc)

set(coreSources
src/vl53l0x_api.c
src/vl53l0x_api_calibration.c
src/vl53l0x_api_core.c
src/vl53l0x_api_ranging.c
src/vl53l0x_api_strings.c
)

SET(platform
  vl53l0x_i2c_platform.c
  vl53l0x_i2c_win_serial_comms.c
  vl53l0x_platform.c
  vl53l0x_platform_log.c
  )

ADD_EXECUTABLE(vl53 vl53.cpp ${platform} ${coreSources})
