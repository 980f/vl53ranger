//arduino platform for VL53L0x
//#include "log_platform.h"
#include "vl53l0x_platform_log.h"
#include <Arduino.h>
#include "vl53l0x_api_strings.h"

static void elapsedTimeReporter(const LocationStack::Element &loc, LocationStack::Ticks elapsed, bool onException) {
  bool showmillis=elapsed>10000;

  Serial.print(loc.function);
  Serial.print(loc.file);
  Serial.print(loc.line);
  if(showmillis){
    Serial.print(elapsed/1000);
    Serial.print("ms");
  } else {
    Serial.print(elapsed);
    Serial.print("us");
  }
  if (onException) {
    auto thrown= int(Thrower::top());
    Serial.print(thrown);
    Serial.print(VL53L0X::pal_error_string(static_cast<VL53L0X::Error>(thrown)));
    Serial.println(", but shit happened");
  } else {
    Serial.println();
  }
}

static LocationStack::Ticks stamper() {
  return micros();
}

void VL53L0X::initLogging() {
  LocationStack::reportElapsed = &elapsedTimeReporter;
  LocationStack::stamper = &stamper;
}
