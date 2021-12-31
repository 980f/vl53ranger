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

constexpr Version_t Required {1, 0, 1};

//no longer needed as we are no longer trying to mashup strings for debug messages
//#define STR_HELPER(x) #x     ///< a string helper
//#define STR(x) STR_HELPER(x) ///< string helper wrapper

void showVersion(const __FlashStringHelper *prefix, const Version_t ver, bool andLine = true) {
  Serial.print(prefix);

  Serial.print(ver.ver.major);
  Serial.print('.');
  Serial.print(ver.ver.minor);
  Serial.print('.');
  Serial.print(ver.build);
  Serial.print('.');
  Serial.print(ver.revision);
  if (andLine) {
    Serial.println();
  }
}
////////////////////

//////////////////////////////////////
//
Adafruit_VL53L0X::Adafruit_VL53L0X(uint8_t i2c_addr, uint8_t busNumber) : MyDevice({busNumber, i2c_addr, 400}) {
  //but do not begin or start etc so that we can static init if we wish.
}

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
  static_assert((Required ==  ImplementationVersion),"application source does not match driver version");

  MyDevice.comm.init();//parameters formerly managed here are now constructor args.


  MyDevice.DataInit(); // Data initialization in the device itself
/*980f: this did not make sense, if you init at its default address then you have no basis for knowing what address to use to set its address.
 * If you know it is at the default finish the overall init then change address after which you can then release and init others.
 *
 * IE changing the address is an unusual step and should be done at a higher level, not bound into a necessary function, which this is as it calls Api::StaticInit().
 */
//  if (!setAddress(i2c_addr)) {
//    return false;
//  }

  if (!MyDevice.GetDeviceInfo(DeviceInfo)) {
    if (debug) {
      Serial.println(F("Failed to get Device Info"));
    }
    return false;
  }
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
    //980f: might work fine, perhaps set some flag but carry on as if OK.
//    return false;//Error = ERROR_NOT_SUPPORTED;
  }

  if (debug) {
    Serial.println(F("VL53L0X: StaticInit"));
  }
  if (!MyDevice.StaticInit()) {
    if (debug) {
      Serial.println(F("VL53L0X: StaticInit FAILED"));
    }
    return false;
  }

  if (debug) {
    Serial.println(F("VL53L0X: PerformRefSpadManagement"));
  }
  // Device Initialization
  if (!MyDevice.PerformRefSpadManagement()) {
    //todo: message about not enough receivers to get a good measureenmt.
    return false;
  }
  if (debug) {
    auto info = MyDevice.get_reference_spads();
    Serial.print(F("refSpadCount = "));
    Serial.print(info.quantity);
    Serial.print(F(", isApertureSpads = "));
    Serial.println(info.isAperture);
  }
  if (debug) {
    Serial.println(F("VL53L0X: PerformRefCalibration"));
  }
  if (MyDevice.PerformRefCalibration()) {
    auto calp = MyDevice.get_ref_calibration();
    //todo: show values to someone?
  } else {
    return false;
  }

  // no need to do this when we use PerformSingleRangingMeasurement
  if (debug) {
    Serial.println(F("VL53L0X: SetDeviceMode"));
  }

  //note: by now the device will be in single_ranging due to the calibrations performed above.
  if (!MyDevice.SetDeviceMode(DEVICEMODE_SINGLE_RANGING)) {  // Setup in single ranging mode
    //todo: print failed to set mode
    return false;
  }

  // call off to the config function to do the last part of configuration.
  configSensor(vl_config);

  return true;
} // Adafruit_VL53L0X::begin

/**************************************************************************/
/*!
 *   @brief  Change the I2C address of the sensor
 *   @param  newAddr the new address to set the sensor to
 *   @returns True if address was set successfully, False otherwise
 *   NOTE WELL: there is a delay of unknown purpose in this function, consider it to be blockin.
 */
/**************************************************************************/
boolean Adafruit_VL53L0X::setAddress(uint8_t newAddr) {
//gratuitous since we are about to double it:  newAddr &= 0x7F;
  bool didit = MyDevice.SetDeviceAddress(newAddr * 2); // 7->8 bit

  delay(10);//BUG: evil to do delays instead of returning a value for user to mix in with other delay logic. There should be a description of what needs to be delayed.

//  //980f: moved the following into the API code!
//  if (Error == ERROR_NONE) {
//    MyDevice.comm.wirer.devAddr = newAddr; // 7 bit addr
//    return true;
//  }
  return didit;
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
  //980F: the only possible errors here were invalid checkid and since we are using the enum values there are no errors.
  //908F: I made a real class for FixPoint1616_t and it knows how to convert floats so all the casting and *2^16 went away.
//   Serial.print(F("VL53L0X: configSensor "));
//   Serial.println(vl_config);

//980f: the enables that were here were combined with setting the associated value, one clause trusted the existing value with no reason to.

  switch (vl_config) {
    case SENSE_DEFAULT:
      MyDevice.SetLimitCheck(CHECKENABLE_RANGE_IGNORE_THRESHOLD, {true, 1.5 * 0.023});
      //dropped by 980f:  why would we expect the associated values to be good?
      //  MyDevice.SetLimitCheckEnable(CHECKENABLE_SIGMA_FINAL_RANGE, true);
      //  MyDevice.SetLimitCheckEnable(CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, true);
      break;
    case SENSE_LONG_RANGE:
//      Serial.println("  SENSE_LONG_RANGE");
      MyDevice.SetLimitCheck(CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, {true, 0.1});
      MyDevice.SetLimitCheck(CHECKENABLE_SIGMA_FINAL_RANGE, {true, 60.0F});
      MyDevice.SetMeasurementTimingBudgetMicroSeconds(33000);
      MyDevice.SetVcselPulsePeriod(VCSEL_PERIOD_PRE_RANGE, 18);
      MyDevice.SetVcselPulsePeriod(VCSEL_PERIOD_FINAL_RANGE, 14);
      //todo: perform_calibration which was formerly done twice because of the two calls above. I removed it from the calls as running it once is good and proper.
      break;
    case SENSE_HIGH_SPEED:
      // Serial.println("  SENSE_HIGH_SPEED");
      MyDevice.SetLimitCheck(CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, {true, 0.25});
      MyDevice.SetLimitCheck(CHECKENABLE_SIGMA_FINAL_RANGE, {true, 32.0});
      MyDevice.SetMeasurementTimingBudgetMicroSeconds(30000);//hmm: isn't 20,000 possible?

      break;
    case SENSE_HIGH_ACCURACY:
      // increase timing budget to 200 ms
      MyDevice.SetLimitCheck(CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, {true, 0.25});
      MyDevice.SetLimitCheck(CHECKENABLE_SIGMA_FINAL_RANGE, {true, 18.0});
      setMeasurementTimingBudgetMicroSeconds(200000);
      // Not sure about ignore threhold, try turnning it off...
      MyDevice.SetLimitCheckEnable(CHECKENABLE_RANGE_IGNORE_THRESHOLD, false);

      break;
    default:
      return false;
  } // switch
  return true;
} // Adafruit_VL53L0X::configSensor

/**************************************************************************/
/*!
 *   @brief  get a ranging measurement from the device
 *   @param  RangingMeasurementData the pointer to the struct the data will be
 *  stored in
 *   @param debug Optional debug flag. If true debug information will print via
 *  Serial.print during execution. Defaults to false.
 *   @returns True if address was set successfully, False otherwise and you can dig into the exception trace logic for details.
 */
/**************************************************************************/
bool Adafruit_VL53L0X::GetSingleRangingMeasurement(RangingMeasurementData_t &RangingMeasurementData, boolean debug) {
  /*
   *  Step  4 : Test ranging mode
   */
  if (debug) {
    Serial.println(F("VL53L0X: PerformSingleRangingMeasurement"));
  }
  bool worked = MyDevice.PerformSingleRangingMeasurement(RangingMeasurementData);

  if (debug) {
    if (worked) {
      Serial.print(worked ? F("Measurement Completed OK") : F("Measurement Failed"));
    }
    printRangeStatus(RangingMeasurementData);
    auto LimitCheckCurrent = MyDevice.GetLimitCheckCurrent(CHECKENABLE_RANGE_IGNORE_THRESHOLD);

    Serial.print(F("RANGE IGNORE THRESHOLD: "));
    Serial.println(static_cast<float> (LimitCheckCurrent));

    Serial.print(F("Measured distance: "));
    Serial.println(RangingMeasurementData.RangeMilliMeter);
  }
  return worked;
} // Adafruit_VL53L0X::getSingleRangingMeasurement

/**************************************************************************/
/*!
 *   @brief  print a ranging measurement out via Serial.print in a human-readable
 *  format
 *   @param pRangingMeasurementData a pointer to the ranging measurement data
 */
/**************************************************************************/
void Adafruit_VL53L0X::printRangeStatus(RangingMeasurementData_t &pRangingMeasurementData) {
  /* New Range Error: data is valid when pRangingMeasurementData->rangeError =  0   */
  auto RangeStatus = pRangingMeasurementData.rangeError;
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

  auto worked = GetSingleRangingMeasurement(measure, false);
  _rangeStatus = measure.rangeError;

  if (worked) {
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
  // first lets set the device in SINGLE_Ranging mode
  return MyDevice.SetDeviceMode(DEVICEMODE_SINGLE_RANGING) && MyDevice.StartMeasurement();
} // Adafruit_VL53L0X::startRange

/**************************************************************************/
/*!
 *   @brief  Checks to see if a range operation has completed
 *   @return true if range operation completed or an error has happened
 */
/**************************************************************************/

boolean Adafruit_VL53L0X::isRangeComplete() {
  return MyDevice.GetMeasurementDataReady();//or could trust that when status is bad the mready value has been coerced false.
}

/**************************************************************************/
/*!
 *   @brief  Wait until Range operation has completed.
 *   @return true if range operation completed, false if error.
 */
/**************************************************************************/

boolean Adafruit_VL53L0X::waitRangeComplete() {
  return MyDevice.measurement_poll_for_completion();
}

/**************************************************************************/
/*!
 *   @brief  Return the range in mm for the last operation.
 *   @return Range in mm.
 */
/**************************************************************************/

uint16_t Adafruit_VL53L0X::readRangeResult() {
  RangingMeasurementData_t measure; // keep our own private copy

  MyDevice.GetRangingMeasurementData(measure);
  _rangeStatus = measure.rangeError;
  MyDevice.ClearInterruptMask(0);
  //todo: below seems overly generous, should be checking for is RangeValid
  return (_rangeStatus != RangeStatus::Phase_fail) ? measure.RangeMilliMeter : ~0; // some out of range value
} // Adafruit_VL53L0X::readRangeResult

/**************************************************************************/
/*!
 *   @brief  Start a continuous range operation
 *   @param period_ms inter measurement period in milliseconds
 *   @return True if successful, false otherwise
 */
/**************************************************************************/
boolean Adafruit_VL53L0X::startRangeContinuous(uint16_t period_ms) {
  //bug: had stale comments!
  MyDevice.SetDeviceMode(DEVICEMODE_CONTINUOUS_TIMED_RANGING);
  MyDevice.SetInterMeasurementPeriodMilliSeconds(period_ms);
  return MyDevice.StartMeasurement();
} // Adafruit_VL53L0X::startRangeContinuous

/**************************************************************************/
/*!
 *   @brief  Stop a continuous ranging operation
 *   @note BLOCKING
 */
/**************************************************************************/
bool Adafruit_VL53L0X::stopRangeContinuous() {
  //todo: this exists in api somewhere.
//  Error = MyDevice.StopMeasurement();
//  // Wait until it finished
//  // use timeout to avoid deadlock
//  if (Error == ERROR_NONE) {
//    for (unsigned LoopNb = VL53L0X_DEFAULT_MAX_LOOP; LoopNb-- > 0;) {
//      // lets wait until that completes.
//      auto StopCompleted= MyDevice.GetStopCompletedStatus();
//
//        //above: failed or finished
//        Error = MyDevice.ClearInterruptMask(GPIOFUNCTIONALITY_NEW_MEASURE_READY);
//        return Error == ERROR_NONE;
//        MyDevice.PollingDelay();
//    }
//    Error = ERROR_TIME_OUT;
//  }
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
  return MyDevice.SetMeasurementTimingBudgetMicroSeconds(budget_us);
}

/**************************************************************************/
/*!
 *   @brief  Wrapper to ST library code to budget how long a measurement
 *   should take
 *   @returns the current budget time in microseconds.
 */
/**************************************************************************/
uint32_t Adafruit_VL53L0X::getMeasurementTimingBudgetMicroSeconds() {
  return MyDevice.GetMeasurementTimingBudgetMicroSeconds();
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
  return MyDevice.SetVcselPulsePeriod(VcselPeriodType, VCSELPulsePeriod);
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
  return MyDevice.GetVcselPulsePeriod(VcselPeriodType);
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
void Adafruit_VL53L0X::setLimitCheckEnable(CheckEnable LimitCheckId, bool LimitCheckEnable) {
  MyDevice.SetLimitCheckEnable(LimitCheckId, LimitCheckEnable);
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
  return MyDevice.GetLimitCheckEnable(LimitCheckId);
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

void Adafruit_VL53L0X::setLimitCheckValue(CheckEnable LimitCheckId, FixPoint<9, 7> LimitCheckValue) {
  MyDevice.SetLimitCheckValue(LimitCheckId, LimitCheckValue);
}

/**************************************************************************/
/*!
 *   @brief  Get a specific limit check value
 *   @param   LimitCheckId                  Limit Check ID
 *  (   0<= LimitCheckId < GetNumberOfLimitCheck() ).
 *   @return  limit check value in FixPoint1616
 */
/**************************************************************************/
FixPoint<9, 7> Adafruit_VL53L0X::getLimitCheckValue(CheckEnable LimitCheckId) {
  return MyDevice.GetLimitCheckValue(LimitCheckId);
}

void Adafruit_VL53L0X::printError() {
  Serial.print(pal_error_string(static_cast<Error>(MyDevice.comm.wirer.Throw.errorcode)));
  Serial.print(MyDevice.comm.wirer.Throw.location);
  Serial.print(MyDevice.comm.wirer.Throw.line);
}
