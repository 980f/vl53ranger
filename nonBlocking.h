//
// Created by andyh on 12/27/21.
//

#ifndef VL53_NONBLOCKING_H
#define VL53_NONBLOCKING_H

#include "vl53l0x_api.h"

class NonBlocking: public VL53L0X::Api  {

public:
  VL53L0X::Error lastError=VL53L0X::ERROR_NONE;
  const char *errorLocation= nullptr;

  bool logError(VL53L0X::Error code,const char *location){
    lastError=code;
    errorLocation=location;
    return false;
  }

#define ON_ERROR(c)   return logError( c , __FUNCTION__ );

  NonBlocking(uint8_t i2c_addr = VL53L0X_I2C_ADDR>>1, TwoWire &i2c = Wire) : Api({i2c,i2c_addr,400})
  ,theXtalkProcess(*this)
  , theOffsetProcess(*this){
    //do no real actions so that we can statically construct
  }

  enum MeasurementAction {
    Abandoned = 0   //for state machine reset.
    , forRange  //normal use, when not in continuous mode
    , forVHV     //special seqconfig
    , forPhase   //special seqconfig
    , forRef     //special seqconfig
    , forOffset  //50 times
    , forXtalk   //50 times
  };

  virtual void onStopComplete(bool b);
  //placeholder for what to do when a wait on measurement complete is successful
  virtual void doMeasurementComplete(bool successful);

  void waitForMeasurement(unsigned loops = 250) {
    waitOnMeasurementComplete = loops;
    //if functional rather than virtual here is where we record the action to take on completion.
  }

  void waitForStop(unsigned loops = 250) {
    waitOnStop = loops;
    //if functional rather than virtual here is where we record the action to take on completion.
  }

/** call this from your dispatcher, such as loop() in Arduino  or   while(1){WFE(); ... }*/
  void inLoop();

  bool requestMeasurement(MeasurementAction use){
    //todo: check if any in progress and refuse, until we implement a queue
    switch (use) {
      case Abandoned:
        return false;
      case forXtalk:
        return theXtalkProcess.begin();
      case forRange:
        break;
      case forVHV:
        break;
      case forPhase:
        break;
      case forRef:
        break;
      case forOffset:
        break;
    }
  }

protected:
  /** makes a buch of measurements then sets XTalkCompensationRateMegaCps.
   * usage:    .theXtalkProcess.XTalkCalDistance= your value for it;  .requestMeasurement(forXtalk) */
  class XtalkProcess {
    NonBlocking &dev;
  public:
    FixPoint1616_t XTalkCalDistance;
  private:
    /* Perform 50 measurements and compute the averages */
    unsigned sum_ranging = 0;//bug: former use of 16 bit begged for integer overflow
    unsigned sum_spads = 0;//... which we will tolerate only on processors whose natural int is 16 bits
    FixPoint1616_t sum_signalRate = 0;
    uint32_t total_count = 0;//ick: unsigned is probably adequate
    unsigned measurementRemaining = 0;
  public:
    XtalkProcess(NonBlocking &dev);
    bool begin();
    /** @returns whether to continue the process. if not then dev.lastError details why  */
    bool onMeasurement(const VL53L0X::RangingMeasurementData_t &RangingMeasurementData);
  } theXtalkProcess;

  class OffsetProcess {
  public:
    OffsetProcess(NonBlocking &dev);
  private:
    NonBlocking &dev;
  public:
    FixPoint1616_t CalDistanceMilliMeter;
  private:
    bool SequenceStepWasEnabled=false;
    uint16_t sum_ranging = 0;
    uint32_t total_count = 0;
    unsigned measurementRemaining=0;
  public:
    bool begin();
    /** @returns whether to continue the process. if not then dev.lastError details why  */
    bool onMeasurement(const VL53L0X::RangingMeasurementData_t &RangingMeasurementData);
  } theOffsetProcess;

  /** result of most recent measurement of any type */
  VL53L0X::RangingMeasurementData_t theRangingMeasurementData;
  MeasurementAction theLastMeasurement;

public: //for debug, read only outside of this class.
  MeasurementAction measurementInProgress=Abandoned;
  unsigned waitOnStart=0;
  unsigned waitOnMeasurementComplete = 0;
  unsigned waitOnStop = 0;
  bool startMeasurement(MeasurementAction action);
};

#endif //VL53_NONBLOCKING_H
