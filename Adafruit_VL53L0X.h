/*!
 * @file Adafruit_VL53L0X.h
 *
 *  This is a library for the Adafruit VL53L0X Sensor Breakout
 *
 *  Designed specifically to work with the VL53L0X sensor from Adafruit
 *  ----> https://www.adafruit.com/products/3317
 *
 *  These sensors use I2C to communicate, 2 pins are required to
 *  interface
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing
 *  products from Adafruit!
 *
 *  Written by Limor Fried/Ladyada for Adafruit Industries.
 *  BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifndef ADAFRUIT_VL53L0X_H
#define ADAFRUIT_VL53L0X_H

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "vl53l0x_api.h"

//value is owned by ST so 980F put it into their headers, and they like 8bit address values
#ifndef VL53L0X_I2C_ADDR
VL53L0X_I2C_ADDR 0x52
#endif
/**************************************************************************/
/*!
 *   @brief  Class that stores state and functions for interacting with VL53L0X
 *  time-of-flight sensor chips
 */
/**************************************************************************/
class Adafruit_VL53L0X {
public:
  /** Sensor configurations */
  enum  Sense_config_t{
    SENSE_DEFAULT = 0     //removed prefix that ST puts on their stuff to avoid future confusion
    , SENSE_LONG_RANGE
    , SENSE_HIGH_SPEED
    , SENSE_HIGH_ACCURACY
  } ;

  Adafruit_VL53L0X(uint8_t i2c_addr = VL53L0X_I2C_ADDR>>1, uint8_t busNumber=0);

  boolean begin( boolean debug = false,  Sense_config_t vl_config = SENSE_DEFAULT);

// multiSensor support temporarily NYI
//  /** changing the address is only needed if there are multiple sensors on the bus. */
//  boolean setAddress(uint8_t newAddr);

  /**************************************************************************/
  /*!
   *   @brief  get a ranging measurement from the device
   *   @param  pRangingMeasurementData the pointer to the struct the data will be
   *  stored in
   *   @param debug Optional debug flag. If true debug information will print via
   *  Serial.print during execution. Defaults to false.
   *   @returns True if address was set successfully, False otherwise
   */
  /**************************************************************************/
  bool rangingTest(VL53L0X::RangingMeasurementData_t &pRangingMeasurementData, boolean debug = false){
    return GetSingleRangingMeasurement(pRangingMeasurementData, debug);
  }

  bool GetSingleRangingMeasurement(VL53L0X::RangingMeasurementData_t &pRangingMeasurementData, boolean debug = false);

  void printRangeStatus(VL53L0X::RangingMeasurementData_t &pRangingMeasurementData);

  void printError();

  // Add similar methods as Adafruit_VL6180X class adapted to range of device
  uint16_t readRange();
  // float readLux(uint8_t gain);
  uint8_t readRangeStatus();

  boolean startRange();
  boolean isRangeComplete();
  boolean waitRangeComplete();
  uint16_t readRangeResult();

  boolean startRangeContinuous(uint16_t period_ms = 50);//hmm: ST shows 33 as a sweet spot between performance and accuracy.
  bool stopRangeContinuous();

  //  void setTimeout(uint16_t timeout) { io_timeout = timeout; }
  // uint16_t getTimeout(void) { return io_timeout; }
  /**************************************************************************/
  /*!
   *   @brief  timeout status
   *   @returns True if timeout has occurred, False otherwise
   */
  /**************************************************************************/
  boolean timeoutOccurred(){
    return false;//why not Error==ERROR_TIMEOUT?
  }

  boolean configSensor(Sense_config_t vl_config);

  // Export some wrappers to internal setting functions
  // that are used by the above helper function to allow
  // more complete control.
  boolean setMeasurementTimingBudgetMicroSeconds(uint32_t budget_us);
  uint32_t getMeasurementTimingBudgetMicroSeconds();

  boolean setVcselPulsePeriod(VL53L0X::VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriod);

  uint8_t getVcselPulsePeriod(VL53L0X::VcselPeriod VcselPeriodType);

  void setLimitCheckEnable(VL53L0X::CheckEnable LimitCheckId, bool LimitCheckEnable);
  boolean getLimitCheckEnable(VL53L0X::CheckEnable LimitCheckId);
  void setLimitCheckValue(VL53L0X::CheckEnable LimitCheckId, FixPoint<9, 7> LimitCheckValue);
  FixPoint<9,7> getLimitCheckValue(VL53L0X::CheckEnable LimitCheckId);

private:
  VL53L0X::Api MyDevice;
  VL53L0X::DeviceInfo_t DeviceInfo;
  VL53L0X::RangeStatus _rangeStatus;
}; // class Adafruit_VL53L0X

#endif // ifndef ADAFRUIT_VL53L0X_H
