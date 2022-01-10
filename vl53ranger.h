#pragma  once
//
// Created by andyh on 12/31/21.
//


#include "nonblocking.h"
#include "trynester.h"

class VL53Ranger : public VL53L0X::NonBlocking, public VL53L0X::NonBlocking::UserAgent, public LocationStack::Logger {
public:
  VL53Ranger() noexcept;

  void processEvent(ProcessRequest process, ProcessResult stage) override;
  /** called when we are waiting for a measurement and have been told to configure the VL53 signal and so on.*/
   bool gpioSignal() override;
  // Adafruit add on:
  enum SensorConfiguration {
    SENSE_DEFAULT=0
    ,SENSE_LONG_RANGE
    ,SENSE_HIGH_SPEED
    ,SENSE_HIGH_ACCURACY
  };

  bool configSensor(SensorConfiguration vl_config) ;

  LocationStack::Ticks stamper() override;
  
  void reportElapsed(const LocationStack::Element &loc, LocationStack::Ticks elapsed) override;

  void exception(int throwncode) override;
};
