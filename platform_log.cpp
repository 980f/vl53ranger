//arduino platform for VL53L0x

#include "vl53l0x_platform_log.h"
#include "vl53l0x_api_strings.h"

#include "trynester.h"

#include "Arduino.h"
class MyLogger : public LocationStack::Logger{
public:
  LocationStack::Ticks stamper() override {
    return micros();
  }

  void reportElapsed(const LocationStack::Element &loc, LocationStack::Ticks elapsed) override {
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
  }

  void exception(int throwncode) override {
    Serial.print(throwncode);
    Serial.print(": ");
    Serial.print(VL53L0X::pal_error_string(static_cast<VL53L0X::Error>(throwncode)));
    Serial.println(F("Stack trace, mot recent first:"));
  }
};
MyLogger myLogger;

void VL53L0X::initLogging() {
  LocationStack::logger = &myLogger;
  //in case we want to restart all:
  LocationStack::unwind( nullptr);
  Thrower::unwind(nullptr);
  Serial.println("VL53l0X logging initialized" );
}
