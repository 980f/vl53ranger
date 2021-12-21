/*!
 * @file Adafruit_VL53L0X.cpp
 *
 * @mainpage Adafruit VL53L0X time-of-flight sensor
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's VL53L0X driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit VL53L0X breakout: https://www.adafruit.com/product/3317
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Adafruit_VL53L0X.h"
#include "vl53l0x_api_core.h"
#include "vl53l0x_api_strings.h"

using namespace VL53L0X; //usually bad form but this class is a wrapper for this namespace.

#define VERSION_REQUIRED_MAJOR 1 ///< Required sensor major version
#define VERSION_REQUIRED_MINOR 0 ///< Required sensor minor version
#define VERSION_REQUIRED_BUILD 1 ///< Required sensor build

#define STR_HELPER(x) #x     ///< a string helper
#define STR(x) STR_HELPER(x) ///< string helper wrapper

/**************************************************************************/
/*!
 *   @brief  Setups the I2C interface and hardware
 *   @param  i2c_addr Optional I2C address the sensor can be found on. Default is
 *  0x29
 *   @param debug Optional debug flag. If true, debug information will print out
 *  via Serial.print during setup. Defaults to false.
 *   @param  i2c Optional I2C bus the sensor is located on. Default is Wire
 *   @param vl_config Sensor configuration
 *   @returns True if device is set up, false on any failure
 */
/**************************************************************************/
boolean Adafruit_VL53L0X::begin(boolean debug, Sense_config_t vl_config) {
  uint8_t VhvSettings;
  uint8_t PhaseCal;

  MyDevice.comm.init();//parameters formerly managed here are now constructor args.

  // unclear if this is even needed:
  //980F: use formal call to get version info and then check that.
//  if (IMPLEMENTATION_VER_MAJOR != VERSION_REQUIRED_MAJOR || IMPLEMENTATION_VER_MINOR != VERSION_REQUIRED_MINOR || IMPLEMENTATION_VER_SUB != VERSION_REQUIRED_BUILD) {
//    if (debug) {
//      Serial.println(F("Found " STR(IMPLEMENTATION_VER_MAJOR) "." STR(IMPLEMENTATION_VER_MINOR) "." STR(IMPLEMENTATION_VER_SUB) " rev " STR(IMPLEMENTATION_VER_REVISION)));
//      Serial.println(F("Requires " STR(VERSION_REQUIRED_MAJOR) "." STR(VERSION_REQUIRED_MINOR) "." STR(VERSION_REQUIRED_BUILD)));
//    }
//    Error = ERROR_NOT_SUPPORTED;
//    return false;
//  }

  Error = MyDevice.DataInit(); // Data initialization
/*980f: this did not make sense, either you init to its present address or you have no basis for knowing what address to use to set its address.
 * If you know it is at the default finish the overall init then change address after which you can then release and init others.
 *
 * IE changing the address is an unusual step and should be done at a higher level, not bound into a necessary function, which this is as it calls Api::StaticInit().
 */
//  if (!setAddress(i2c_addr)) {
//    return false;
//  }

  Error = MyDevice.GetDeviceInfo(&DeviceInfo);

  if (Error == ERROR_NONE) {
    if (debug) {
      Serial.println(F("VL53L0X Info:"));
      Serial.print(F("Device Name: "));
      Serial.print(DeviceInfo.Name);
      Serial.print(F(", Type: "));
      Serial.print(DeviceInfo.Type);
      Serial.print(F(", ID: "));
      Serial.println(DeviceInfo.ProductId);

      Serial.print(F("Rev major: "));
      Serial.print(DeviceInfo.ProductRevision.major);
      Serial.print(F(", minor: "));
      Serial.println(DeviceInfo.ProductRevision.minor);
    }

    if ((DeviceInfo.ProductRevision.major != 1) || (DeviceInfo.ProductRevision.minor != 1)) {
      if (debug) {
        Serial.print(F("Error expected cut 1.1 but found "));
        Serial.print(DeviceInfo.ProductRevision.major);
        Serial.print(',');
        Serial.println(DeviceInfo.ProductRevision.major);
      }
      Error = ERROR_NOT_SUPPORTED;
    }
  }

  if (Error == ERROR_NONE) {
    if (debug) {
      Serial.println(F("VL53L0X: StaticInit"));
    }
    Error = MyDevice.StaticInit(); // Device Initialization
  }

  if (Error == ERROR_NONE) {
    if (debug) {
      Serial.println(F("VL53L0X: PerformRefSpadManagement"));
    }

    auto info = MyDevice.PerformRefSpadManagement(); // Device Initialization

    if (debug) {
      Serial.print(F("refSpadCount = "));
      Serial.print(info.count);
      Serial.print(F(", isApertureSpads = "));
      Serial.println(info.isAperture);
    }
  }

  if (Error == ERROR_NONE) {
    if (debug) {
      Serial.println(F("VL53L0X: PerformRefCalibration"));
    }

    Error = MyDevice.PerformRefCalibration(&VhvSettings, &PhaseCal); // Device Initialization
  }

  if (Error == ERROR_NONE) {
    // no need to do this when we use PerformSingleRangingMeasurement
    if (debug) {
      Serial.println(F("VL53L0X: SetDeviceMode"));
    }

    Error = MyDevice.SetDeviceMode(DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
  }

  // call off to the config function to do the last part of configuration.
  if (Error == ERROR_NONE) {
    configSensor(vl_config);
  }

  if (Error == ERROR_NONE) {
    return true;
  } else {
    if (debug) {
      Serial.print(F("VL53L0X Error: "));
      Serial.println(Error);
    }
    return false;
  }
} // Adafruit_VL53L0X::begin

/**************************************************************************/
/*!
 *   @brief  Change the I2C address of the sensor
 *   @param  newAddr the new address to set the sensor to
 *   @returns True if address was set successfully, False otherwise
 *   NOTE WELL: you have to
 */
/**************************************************************************/
boolean Adafruit_VL53L0X::setAddress(uint8_t newAddr) {
//gratuitous since we are about to double it:  newAddr &= 0x7F;
  Error = MyDevice.SetDeviceAddress(newAddr * 2); // 7->8 bit

  delay(10);//BUG: evil to do delays instead of returning a value for user to mix in with other delay logic. There should be a description of what needs to be delayed.

  //908f: the following should be done in the API code!
  if (Error == ERROR_NONE) {
    MyDevice.comm.i2cDevAddr = newAddr; // 7 bit addr
    return true;
  }
  return false;
} // Adafruit_VL53L0X::setAddress

/**************************************************************************/
/*!
 *   @brief  Configure the sensor for one of the ways the example ST
 *   sketches configure the sensors for different usages.
 *   @param  vl_config Which configuration you are trying to configure for
 *   It should be one of the following
 *       MyDevice.SENSE_DEFAULT
 *       MyDevice.SENSE_LONG_RANGE
 *       MyDevice.SENSE_HIGH_SPEED,
 *       MyDevice.SENSE_HIGH_ACCURACY
 *
 *   @returns True if address was set successfully, False otherwise
 */
/**************************************************************************/
boolean Adafruit_VL53L0X::configSensor(Sense_config_t vl_config) {
  // All of them appear to configure a few things

  // Serial.print(F("VL53L0X: configSensor "));
  // Serial.println((int)vl_config, DEC);
  // Enable/Disable Sigma and Signal check
  Error = MyDevice.SetLimitCheckEnable(CHECKENABLE_SIGMA_FINAL_RANGE, 1);

  if (Error == ERROR_NONE) {
    Error = MyDevice.SetLimitCheckEnable(CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
  }

  if (Error != ERROR_NONE) {
    return false;
  }

  switch (vl_config) {
    case SENSE_DEFAULT:
      // Taken directly from SDK vl5310x_SingleRanging_example.c
      // Maybe should convert to helper functions but...
      // Serial.println("  SENSE_DEFAULT");
      if (Error == ERROR_NONE) {
        Error = MyDevice.SetLimitCheckEnable(CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
      }

      if (Error == ERROR_NONE) {
        Error = MyDevice.SetLimitCheckValue(CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t) (1.5 * 0.023 * 65536));
      }
      break;
    case SENSE_LONG_RANGE:
      Serial.println("  SENSE_LONG_RANGE");
      Error = MyDevice.SetLimitCheckValue(CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t) (0.1 * 65536));
      if (Error == ERROR_NONE) {
        Error = MyDevice.SetLimitCheckValue(CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t) (60 * 65536));
      }
      if (Error == ERROR_NONE) {
        Error = SetMeasurementTimingBudgetMicroSeconds(33000);
      }

      if (Error == ERROR_NONE) {
        Error = MyDevice.SetVcselPulsePeriod(VCSEL_PERIOD_PRE_RANGE, 18);
      }
      if (Error == ERROR_NONE) {
        Error = MyDevice.SetVcselPulsePeriod(VCSEL_PERIOD_FINAL_RANGE, 14);
      }
      break;
    case SENSE_HIGH_SPEED:
      // Serial.println("  SENSE_HIGH_SPEED");
      Error = MyDevice.SetLimitCheckValue(CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t) (0.25 * 65536));
      if (Error == ERROR_NONE) {
        Error = MyDevice.SetLimitCheckValue(CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t) (32 * 65536));
      }
      if (Error == ERROR_NONE) {
        Error = MyDevice.SetMeasurementTimingBudgetMicroSeconds(30000);
      }
      break;
    case SENSE_HIGH_ACCURACY:
      // increase timing budget to 200 ms
      if (Error == ERROR_NONE) {
        setLimitCheckValue(CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t) (0.25 * 65536));
      }
      if (Error == ERROR_NONE) {
        setLimitCheckValue(CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t) (18 * 65536));
      }
      if (Error == ERROR_NONE) {
        setMeasurementTimingBudgetMicroSeconds(200000);
      }
      // Not sure about ignore threhold, try turnning it off...
      if (Error == ERROR_NONE) {
        Error = MyDevice.SetLimitCheckEnable(CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);
      }
      break;
  } // switch
  return Error == ERROR_NONE;
} // Adafruit_VL53L0X::configSensor

/**************************************************************************/
/*!
 *   @brief  get a ranging measurement from the device
 *   @param  RangingMeasurementData the pointer to the struct the data will be
 *  stored in
 *   @param debug Optional debug flag. If true debug information will print via
 *  Serial.print during execution. Defaults to false.
 *   @returns True if address was set successfully, False otherwise
 */
/**************************************************************************/
Error Adafruit_VL53L0X::GetSingleRangingMeasurement(RangingMeasurementData_t &RangingMeasurementData, boolean debug) {
  Error = ERROR_NONE; //must we copy ST's ill advised technique?
  FixPoint1616_t LimitCheckCurrent;

  /*
   *  Step  4 : Test ranging mode
   */

  if (debug) {
    Serial.println(F("VL53L0X: PerformSingleRangingMeasurement"));
  }
  Error = MyDevice.PerformSingleRangingMeasurement(RangingMeasurementData);

  if (debug) {
    printRangeStatus(RangingMeasurementData);
  }

  if (debug) {
    MyDevice.GetLimitCheckCurrent(CHECKENABLE_RANGE_IGNORE_THRESHOLD, &LimitCheckCurrent);

    Serial.print(F("RANGE IGNORE THRESHOLD: "));
    Serial.println((float) LimitCheckCurrent / 65536.0);

    Serial.print(F("Measured distance: "));
    Serial.println(RangingMeasurementData->RangeMilliMeter);
  }

  return Error;
} // Adafruit_VL53L0X::getSingleRangingMeasurement

/**************************************************************************/
/*!
 *   @brief  print a ranging measurement out via Serial.print in a human-readable
 *  format
 *   @param pRangingMeasurementData a pointer to the ranging measurement data
 */
/**************************************************************************/
void Adafruit_VL53L0X::printRangeStatus(RangingMeasurementData_t &pRangingMeasurementData) {
  /* New Range Error: data is valid when pRangingMeasurementData->RangeStatus =  0   */
  auto RangeStatus = pRangingMeasurementData.RangeStatus;
  Serial.print(F("Range Error: "));
  Serial.print(RangeStatus);
  Serial.print(F(" : "));
  Serial.println(VL53L0X::range_status_string(RangeStatus));
} // Adafruit_VL53L0X::printRangeStatus

/**************************************************************************/
/*!
 *   @brief  Single shot ranging. Be sure to check the return of readRangeStatus
 *   to before using the return value!
 *   @return Distance in millimeters if valid
 */
/**************************************************************************/

uint16_t Adafruit_VL53L0X::readRange() {
  VL53L0X::RangingMeasurementData_t measure; // keep our own private copy

  Error = getSingleRangingMeasurement(measure, false);
  _rangeStatus = measure.RangeStatus;

  if (Error == VL53L0X::ERROR_NONE) {
    return measure.RangeMilliMeter;
  }
  // Other status return something totally out of bounds...
  return ~0;
} // Adafruit_VL53L0X::readRange

/**************************************************************************/
/*!
 *   @brief  Request ranging success/error message (retreive after ranging)
 *   @returns One of possible VL6180X_ERROR_* values
 */
/**************************************************************************/

uint8_t Adafruit_VL53L0X::readRangeStatus() {
  return _rangeStatus;
}

/**************************************************************************/
/*!
 *   @brief  Start a range operation
 *   @return true if range operation successfully started.
 */
/**************************************************************************/

boolean Adafruit_VL53L0X::startRange() {

  /* This function will do a complete single ranging
   * Here we fix the mode! */
  // first lets set the device in SINGLE_Ranging mode
  Error = MyDevice.SetDeviceMode(DEVICEMODE_SINGLE_RANGING);

  if (Error == MyDevice.ERROR_NONE) {
    // Lets start up the measurement
    Error = MyDevice.StartMeasurement();
  }
  return Error == MyDevice.ERROR_NONE;
} // Adafruit_VL53L0X::startRange

/**************************************************************************/
/*!
 *   @brief  Checks to see if a range operation has completed
 *   @return true if range operation completed or an error has happened
 */
/**************************************************************************/

boolean Adafruit_VL53L0X::isRangeComplete() {
  Erroneous<bool> mready = MyDevice.GetMeasurementDataReady();
  return mready.isOk() && mready;//or could trust that when status is bad the mready value has been coerced false.
}

/**************************************************************************/
/*!
 *   @brief  Wait until Range operation has completed.
 *   @return true if range operation completed, false if error.
 */
/**************************************************************************/

boolean Adafruit_VL53L0X::waitRangeComplete() {
  Error = MyDevice.measurement_poll_for_completion();

  return Error == ERROR_NONE;
}

/**************************************************************************/
/*!
 *   @brief  Return the range in mm for the last operation.
 *   @return Range in mm.
 */
/**************************************************************************/

uint16_t Adafruit_VL53L0X::readRangeResult() {
  RangingMeasurementData_t measure; // keep our own private copy

  Error = MyDevice.GetRangingMeasurementData(&measure);
  _rangeStatus = measure.RangeStatus;
  if (Error == ERROR_NONE) {
    Error = MyDevice.ClearInterruptMask(0);
  }

  if ((Error == ERROR_NONE) && (_rangeStatus != 4)) {
    return measure.RangeMilliMeter;
  }

  return ~0; // some out of range value
} // Adafruit_VL53L0X::readRangeResult

/**************************************************************************/
/*!
 *   @brief  Start a continuous range operation
 *   @param period_ms inter measurement period in milliseconds
 *   @return True if successful, false otherwise
 */
/**************************************************************************/
boolean Adafruit_VL53L0X::startRangeContinuous(uint16_t period_ms) {
  /* This function will do a complete single ranging
   * Here we fix the mode! */
  // first lets set the device in SINGLE_Ranging mode
  Error = MyDevice.SetDeviceMode(DEVICEMODE_CONTINUOUS_TIMED_RANGING);

  if (Error == ERROR_NONE) {
    Error = MyDevice.SetInterMeasurementPeriodMilliSeconds(period_ms);
  }

  if (Error == ERROR_NONE) {
    // Lets start up the measurement
    Error = MyDevice.StartMeasurement();
  }
  return Error == ERROR_NONE;
} // Adafruit_VL53L0X::startRangeContinuous

/**************************************************************************/
/*!
 *   @brief  Stop a continuous ranging operation
 */
/**************************************************************************/
bool Adafruit_VL53L0X::stopRangeContinuous() {
  Error = MyDevice.StopMeasurement();
  // Wait until it finished
  // use timeout to avoid deadlock
  if (Error == ERROR_NONE) {
    for (unsigned LoopNb = MyDevice.DEFAULT_MAX_LOOP; LoopNb-- > 0;) {
      // lets wait until that completes.
      uint32_t StopCompleted = 0;
      Error = MyDevice.GetStopCompletedStatus(&StopCompleted);
      if ((StopCompleted == 0x00) || Error != ERROR_NONE) {
        Error = MyDevice.ClearInterruptMask(GPIOFUNCTIONALITY_NEW_MEASURE_READY);
        return Error == ERROR_NONE;
      }
      MyDevice.PollingDelay();
    }
    Error = ERROR_TIME_OUT;
  }
  return false;
} // Adafruit_VL53L0X::stopRangeContinuous

/**************************************************************************/
/*!
 *   @brief  Wrapper to ST library code to budget how long a measurement
 *   should take
 *   @param  budget_us the new budget
 *   @returns True if success
 */
/**************************************************************************/
boolean Adafruit_VL53L0X::setMeasurementTimingBudgetMicroSeconds(uint32_t budget_us) {
  Error = MyDevice.SetMeasurementTimingBudgetMicroSeconds(budget_us);
  return Error == ERROR_NONE;
}

/**************************************************************************/
/*!
 *   @brief  Wrapper to ST library code to budget how long a measurement
 *   should take
 *   @returns the current budget time in microseconds.
 */
/**************************************************************************/
uint32_t Adafruit_VL53L0X::getMeasurementTimingBudgetMicroSeconds() {
  uint32_t budget_us;
  Error = MyDevice.GetMeasurementTimingBudgetMicroSeconds(&budget_us);
  return budget_us;
}

/**************************************************************************/
/*!
 *   @brief Sets the VCSEL pulse period.
 *
 *   @param   VcselPeriodType       VCSEL period identifier (pre-range|final).
 *   @param   VCSELPulsePeriod          VCSEL period value
 *   @returns True if success
 */
/**************************************************************************/
boolean Adafruit_VL53L0X::setVcselPulsePeriod(VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriod) {
  Error = MyDevice.SetVcselPulsePeriod(VcselPeriodType, VCSELPulsePeriod);
  return Error == ERROR_NONE;
}

/**************************************************************************/
/*!
 *   @brief Gets the VCSEL pulse period.
 *
 *   @param   VcselPeriodType       VCSEL period identifier (pre-range|final).
 *   @returns the current pulse peried for the given type.
 */
/**************************************************************************/
uint8_t Adafruit_VL53L0X::getVcselPulsePeriod(VcselPeriod VcselPeriodType) {
  uint8_t cur_period;
  Error = MyDevice.GetVcselPulsePeriod(VcselPeriodType, &cur_period);
  return cur_period;
}

/**************************************************************************/
/*!
 *   @brief  Enable/Disable a specific limit check
 *
 *   @param   LimitCheckId                  Limit Check ID
 *  (   0<= LimitCheckId < GetNumberOfLimitCheck() ).
 *   @param   LimitCheckEnable              if 1 the check limit
 *    corresponding to LimitCheckId is Enabled
 *                                          if 0 the check limit
 *    corresponding to LimitCheckId is disabled
 *   @return  true if succeeded
 */
/**************************************************************************/
boolean Adafruit_VL53L0X::setLimitCheckEnable(CheckEnable LimitCheckId, uint8_t LimitCheckEnable) {
  Error = MyDevice.SetLimitCheckEnable(LimitCheckId, LimitCheckEnable);
  return Error == ERROR_NONE;
}

/**************************************************************************/
/*!
 *   @brief  Get specific limit check enable state
 *   @param   LimitCheckId                  Limit Check ID
 *  (   0<= LimitCheckId < GetNumberOfLimitCheck() ).
 *   @return  current state of limit enabled
 */
/**************************************************************************/
bool Adafruit_VL53L0X::getLimitCheckEnable(CheckEnable LimitCheckId) {
  Erroneous<bool> cur_limit = MyDevice.GetLimitCheckEnable(LimitCheckId);
  Error = cur_limit.error;
  return cur_limit.wrapped;
}

/**************************************************************************/
/*!
 *   @brief  Set a specific limit check value
 *   @param  LimitCheckId  Limit Check ID
 *  (   0<= LimitCheckId < GetNumberOfLimitCheck() ).
 *   LimitCheckId
 *   @param  LimitCheckValue  Limit Check Value
 *   @return  true if succeeded.
 */
/**************************************************************************/

boolean Adafruit_VL53L0X::setLimitCheckValue(CheckEnable LimitCheckId, FixPoint1616_t LimitCheckValue) {
  Error = MyDevice.SetLimitCheckValue(LimitCheckId, LimitCheckValue);
  return Error == ERROR_NONE;
}

/**************************************************************************/
/*!
 *   @brief  Get a specific limit check value
 *   @param   LimitCheckId                  Limit Check ID
 *  (   0<= LimitCheckId < GetNumberOfLimitCheck() ).
 *   @return  limit check value in FixPoint1616
 */
/**************************************************************************/
FixPoint1616_t Adafruit_VL53L0X::getLimitCheckValue(CheckEnable LimitCheckId) {
  FixPoint1616_t LimitCheckValue;
  Error = MyDevice.GetLimitCheckValue(LimitCheckId, &LimitCheckValue);
  return LimitCheckValue;
}
