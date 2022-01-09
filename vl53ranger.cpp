/** Example of application for using VST VL53L0x device
 * Copyright 2022 by Andrew Heilveil (github/980f)
*/
#include <Arduino.h>
#include "nonblocking.h" // the driver
#include "vl53ranger.h"  // this basic demo application
#include "vl53l0x_platform_log.h"

//#ifndef ARDUINO  //then dummy up the interface so that we can compile
unsigned digitalRead(unsigned) {
  return 0;
}


//#endif

using namespace VL53L0X;

VL53Ranger api;

VL53Ranger::VL53Ranger() noexcept: NonBlocking(static_cast<NonBlocking::UserAgent &> (*this)) {
}

void VL53Ranger::processEvent(NonBlocking::ProcessRequest process, NonBlocking::ProcessResult stage) {
  switch (process) {
    case Operate: //receives measurements
      switch (stage) {
        case Busy:
          break;
        case Succeeded: {
          auto distance = agent.arg.theRangingMeasurementData.Range.distance();
        }
          break;
        case Failed:
          break;
      }
      break;
    case Idle: //acknowledges pwower down et al.
      break;

    case InitStatic://unnecessary or start operating
      break;
    case InitData:
      if (stage == Failed) {
        //serious system issue. Likely no I2C connection.
      }
      break;
    case RateTest://here's your diagnostic data
      break;
    case SetupSpads://only static init should care
      break;
    case Offset: //you may wish to backup parameters
      break;
    case CalVhvPhase://you may wish to backup parameters
      break;
    case CalPhase://you may wish to backup parameters
      break;
    case CrossTalk://you may wish to backup parameters
      break;
  }
}

/** */
bool VL53Ranger::gpioSignal() {
  return digitalRead(arg.gpioPin) == 0;//hard coding low active until we have configuration for it through the layers
}

/////////////////////////////

bool VL53Ranger::configSensor(SensorConfiguration vl_config) {
  // Serial.print(F("VL53L0X: configSensor "));
  // Serial.println((int)vl_config, DEC);
  // Enable/Disable Sigma and Signal check
  TRY {
      switch (vl_config) {
        case SENSE_DEFAULT:
          SetLimitCheck(CHECKENABLE_RANGE_IGNORE_THRESHOLD, {true, 1.5 * 0.023});
          break;
        case SENSE_LONG_RANGE:
//        Serial.println("  SENSE_LONG_RANGE");
          SetLimitCheck(CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, {true, 0.1});
          SetLimitCheck(CHECKENABLE_SIGMA_FINAL_RANGE, {true, 60.0});
          SetMeasurementTimingBudgetMicroSeconds(33000);
          SetVcselPulsePeriod(VCSEL_PERIOD_PRE_RANGE, 18);
          SetVcselPulsePeriod(VCSEL_PERIOD_FINAL_RANGE, 14);
          startProcess(CalVhvPhase, millis());//
          break;
        case SENSE_HIGH_SPEED:
          // Serial.println("  SENSE_HIGH_SPEED");
          SetLimitCheck(CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, {true, 0.25});
          SetLimitCheck(CHECKENABLE_SIGMA_FINAL_RANGE, {true, 32.0});
          SetMeasurementTimingBudgetMicroSeconds(30000);

          break;
        case SENSE_HIGH_ACCURACY:
          // increase timing budget to 200 ms
          SetLimitCheck(CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, {true, 0.25});
          SetLimitCheck(CHECKENABLE_SIGMA_FINAL_RANGE, {true, 18.0});
          SetMeasurementTimingBudgetMicroSeconds(200000);
          SetLimitCheckEnable(CHECKENABLE_RANGE_IGNORE_THRESHOLD, false);
          break;
      }
      return true;
    UNCAUGHT
      return false;
  }

}
