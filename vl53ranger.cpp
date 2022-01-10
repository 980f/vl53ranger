/** Example of application for using VST VL53L0x device
   Copyright 2022 by Andrew Heilveil (github/980f)
*/
#include "Arduino.h"
#include "nonblocking.h" // the driver
#include "vl53ranger.h"  // this basic demo application
#include "vl53l0x_api_strings.h"
#include "chainprinter.h"

extern ChainPrinter dbg;

using namespace VL53L0X;

VL53Ranger::VL53Ranger() noexcept: NonBlocking(static_cast<NonBlocking::UserAgent &> (*this)) {
}

void VL53Ranger::processEvent(NonBlocking::ProcessRequest process, NonBlocking::ProcessResult stage) {
  dbg("processEvent(",process,',',stage,").");
  switch (process) {
    case Operate: //receives measurements
      switch (stage) {
        case Busy:
          break;
        case Succeeded: {
            auto millimeters  = agent.arg.theRangingMeasurementData.Range.distance();
            static bool OOR = false;
            if (changed(OOR, millimeters.rounded() == 65535) || !OOR) {
              //todo: reimplement++    reportTime("Access: ", " us");
              if (OOR) {
                dbg("Out of range.");
              } else {
                dbg("Distance: mm:", millimeters, "\tin:", float(millimeters) / 25.4);
              }
            }
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
        dbg("InitData failed, likely due to I2C read failures.");
      }
      break;
    case RateTest://here's your diagnostic data

      break;
    case SetupSpads://only static init should care
      if (stage == Failed) {
        dbg("Spad configuration failed. Hopefully I2C failure.");
      }
      break;
    case Offset: //you may wish to backup parameters
      if (stage == Succeeded)
        dbg("Offset:", "gotta be here somewhere!");
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

LocationStack::Ticks VL53Ranger::stamper() {
  return micros();
}
/////////////////////////////

bool VL53Ranger::configSensor(SensorConfiguration vl_config) {
  dbg(F("VL53L0X: configSensor "), vl_config);
  TRY {
    switch (vl_config) {
      case SENSE_DEFAULT:
        SetLimitCheck(CHECKENABLE_RANGE_IGNORE_THRESHOLD, {true, 1.5 * 0.023});
        break;
      case SENSE_LONG_RANGE:
        SetLimitCheck(CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, {true, 0.1});
        SetLimitCheck(CHECKENABLE_SIGMA_FINAL_RANGE, {true, 60.0});
        SetMeasurementTimingBudgetMicroSeconds(33000);
        SetVcselPulsePeriod(VCSEL_PERIOD_PRE_RANGE, 18);
        SetVcselPulsePeriod(VCSEL_PERIOD_FINAL_RANGE, 14);
        startProcess(CalVhvPhase, millis());//
        break;
      case SENSE_HIGH_SPEED:
        SetLimitCheck(CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, {true, 0.25});
        SetLimitCheck(CHECKENABLE_SIGMA_FINAL_RANGE, {true, 32.0});
        SetMeasurementTimingBudgetMicroSeconds(30000);

        break;
      case SENSE_HIGH_ACCURACY:
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

void VL53Ranger::exception(int throwncode) {
  dbg("ERROR:", VL53L0X::pal_error_string(static_cast<VL53L0X::Error>(throwncode)), throwncode, CRLF, F("Stack trace, mot recent first:"));
}

void VL53Ranger::reportElapsed(const LocationStack::Element &loc, LocationStack::Ticks elapsed) {
  bool showmillis = elapsed > 10000;
  if (showmillis) {
    elapsed = roundedDivide(elapsed, 1000);
  }
  dbg(loc.function, '@', loc.file, '.', loc.line, " ", elapsed, showmillis ? "ms" : "us");
}

void VL53Ranger::moreInfo(const char *info, ...) {
 //todo:1 var_args printf, someday
  dbg(info);
}
