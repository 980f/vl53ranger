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
//    , forVHV     //special seqconfig
//    , forPhase   //special seqconfig
  ,forRefCal   //vhv and phase
    , forRefSignal     //special seqconfig
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
      default:
      case Abandoned:
        return false;
      case forXtalk:
        return theXtalkProcess.begin();
      case forRange:
        break;
      case forRefCal:
        break;
      case forRefSignal:
        break;
      case forOffset:
        break;
    }
  }

protected:

  /** common base for Xtalk and Offset process
   *
    * Perform 50 measurements and compute the averages
    */
  class AveragingProcess {
    friend class NonBlocking;
  protected:
    NonBlocking &dev;
  public:
    FixPoint1616_t CalDistanceMilliMeter;
  protected:
    unsigned total_count = 0;//ick: former use of 32bit was excessive
    unsigned sum_ranging = 0;//bug: former use of 16 bit begged for integer overflow

    FixPoint1616_t sum_signalRate = 0;
    //measurement count
    unsigned measurementRemaining = 0;
  protected:
    explicit AveragingProcess(NonBlocking &dev):dev(dev){}
    virtual bool begin();
    virtual void startNext()=0;

    /** @returns whether to continue the process. if not then dev.lastError details why  */
    virtual void alsoSum(){}

    bool onMeasurement();
    /** overrides called when last measurement has been summed into dataset */
    virtual bool finish()=0;

  } ;


  /** makes a bunch of measurements then sets XTalkCompensationRateMegaCps.
   * usage:
   * place reflector at known distance
   * .theXtalkProcess.CalDistanceMilliMeter= your value for that;
   * .requestMeasurement(forXtalk) */
  class XtalkProcess:public AveragingProcess {
  public:
    FixPoint1616_t CalDistanceMilliMeter;
  private:
    unsigned sum_spads = 0;//... which we will tolerate only on processors whose natural int is 16 bits
  public:
    explicit XtalkProcess(NonBlocking &dev);
    bool begin();
    void startNext();
    void alsoSum();
    bool finish();
  } theXtalkProcess;

  /** makes a bunch of measurements then sets offset.
   * usage:
   * place reflector at known distance
   * .theOffsetProcess.CalDistanceMilliMeter= your value for that;
   * .requestMeasurement(forOffset) */
  class OffsetProcess:public AveragingProcess {
  public:
    explicit OffsetProcess(NonBlocking &dev);
  private:
    bool SequenceStepWasEnabled=false;
  public:
    bool begin();
    /** @returns whether to continue the process. if not then dev.lastError details why  */
    bool finish();
    void startNext();
  } theOffsetProcess;


  class CalProcess{
  public:
    explicit CalProcess(NonBlocking &dev);
  private:
    NonBlocking &dev;
    bool lastStep= false;//which of two measurements
    uint8_t mycache;//for seq value

  public:
    bool restore_config;
    CalibrationParameters p;
  public:
    bool begin();
    /** @returns whether to continue the process. if not then dev.lastError details why  */
    bool onMeasurement(const VL53L0X::RangingMeasurementData_t &RangingMeasurementData);
    void startNext();
  } theCalProcess;

  /** single measurement from which a reference signal rate is extracted*/
  class RefSignalProcess {
  public:
    explicit RefSignalProcess(NonBlocking &dev);
  private:
    NonBlocking &dev;
    uint8_t mycache;//for seq value
  public:

    uint16_t rate;//the output of the process
    bool begin();
    /** @returns whether to continue the process. if not then dev.lastError details why  */
    bool onMeasurement(const VL53L0X::RangingMeasurementData_t &RangingMeasurementData);
    void startNext();

  }theRefSignalProcess;

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
