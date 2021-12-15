/*******************************************************************************
Copyright 2021 by Andy Heilveil github/980f via extensive rewrite of stuff originally
 Copyright 2015, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of STMicroelectronics nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#ifndef _VL53L0X_PLATFORM_LOG_H_
#define _VL53L0X_PLATFORM_LOG_H_

/* LOG Functions */

/**
 * @file vl53l0_platform_log.h
 *
 * @brief platform log function definition
 */

//do this via compile command line:  #define VL53L0X_LOG_ENABLE 0

enum TRACE_LEVEL {
  NONE
  , ERRORS
  , WARNING
  , INFO
  , DEBUG
  , ALL
  , IGNORE
};

#if VL53L0X_LOG_ENABLE

extern VL53L0X_Error Error;

class PerformanceTracer {
  const char *location;
  uint32_t starttime;
  //placeholders until we get the logging output defined:
  static uint32_t logclock();
  static void printf(const char *format, ...);
  static bool enable;//manipulate via debugger
  bool enabled;//todo: local enable defaulted to global one.
public:
  /** records location and emits start message to log */
  PerformanceTracer(const char *location) : location(location), starttime(logclock()),enabled(enable) {
    if(enabled){
      printf("START %s at %ud", location, starttime);
    }
  }
  //to mate to old system which allowed an additional bit of info on start messages:
  void operator()(const char *format="", ...) {
    if(*format && enabled) {
      printf(format);//todo: vprintf with passed along ... args. There are only two cases and we should just log those separately)
    }
  }
  /** use location and emits start message to log */
  ~PerformanceTracer() {
    printf("END %s after %ud, error=%d", location, logclock() - starttime,Error);//todo: compiletime option for error string
  }
};

#define LOG_FUNCTION_START  PerformanceTracer log(__FUNCTION__);
//for additional info put Log(fmt, ... args) after LOG_FUNCTION_START and before ;
#else
#define LOG_FUNCTION_START
#define Log(...)
#endif

#endif /* _VL53L0X_PLATFORM_LOG_H_ */
