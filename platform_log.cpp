//arduino platform for VL53L0x
//#include "log_platform.h"
#include "vl53l0x_platform_log.h"
#include <Arduino.h>

static void elapsedTimeReporter(const LocationStack::Element &loc, LocationStack::Ticks elapsed, bool onException) {
  Serial.print(loc.function);
  Serial.print(loc.file);
  Serial.print(loc.line);
  Serial.print(elapsed);//todo: millis versus micros
  if (onException) {
    auto thrown= int(Thrower::top());
    Serial.print(thrown);//todo: text rather than number
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
