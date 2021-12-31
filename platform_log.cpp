//arduino platform for VL53L0x
#include "log_platform.h"

#include <Arduino.h>


bool PerformanceTracer::enabled=false;//manipulate via debugger.. //todo:init with some compiler defined flag

//dummied until library with proper printer is in peoject scope
void PerformanceTracer::printf(const char *format, ...) {
}

void PerformanceTracer::operator()(const char *format, ...) {
  if(*format && enabled) {
    printf(format);//todo:- vprintf with passed along ... args. There are only two cases and we should just log those separately)
  }
}

PerformanceTracer::PerformanceTracer(const char *location) :location(location), starttime(logclock()) {
  if(enabled){
    printf("START %s at %ud", location, starttime);
  }
}

PerformanceTracer::~PerformanceTracer() {
  printf("END %s after %ud", location, logclock() - starttime);
}

bool PerformanceTracer::logError(const char *location, VL53L0X::Error error, bool always) {
  if(error!=VL53L0X::ERROR_NONE || always) {
    printf("ERROR from %s at %ud code=%d", location, logclock(), error);//todo:1 appened text form of error
  }
  return error!=VL53L0X::ERROR_NONE;
}

uint32_t PerformanceTracer::logclock() {
  return millis();
}
