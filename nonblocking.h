//
// Created by andyh on 12/27/21.
//

#ifndef VL53_NONBLOCKING_H
#define VL53_NONBLOCKING_H

#include "vl53l0x_api.h"

namespace VL53L0X {

  /** one instance for each sensor.
   * sensor address management is outside the scope of this driver
   * */
  class NonBlocking : public VL53L0X::Api {

  public:

    enum MeasurementAction : uint8_t {
      Abandoned = 0   //for state machine reset.
      , forRefCal   //vhv and phase
      , forRate     //rate requests, direct and spad setup
      , forSpads  //part of StaticInit
      , forRange  //prime use, user measurement (single or continuous)
      , forOffset  //50 times
      , forXtalk   //50 times
    };

/** actions that must complete before others are attempted */
    enum ProcessRequest {
      Idle
      , InitI2c    //400kHz and if more than one device reset pins and setAddress calls are made
      , InitStatic //does SetupSpads, some tuning parameters
      , InitData   //includes tuning table and other settings that shouldn't need to change often
      , OneShot
      , Continuous //references 'timed', will use non timed version for sample rate of 0
      , RefCal     //vhv phasecal, call when vcsel pulse width changes, save and restore from nvm?
      , RateTest   //use may wish to evaluate ambient light when deciding upon
      , SetupSpads //part of init data but callable for debug of device
      , Offset
    };

    enum ProcessResult {
      Busy        //such a process is in progress
      , Queued    //accepted, doesn't get initiated until the next loop() is called
      , Active    //chugging along
      , Succeeded //finished, data will have been delivered via pointers in ProcessArg
      , Failed
    };

    struct ProcessArg {
      uint8_t *tuningTable = nullptr;//if not same as present one then apply it, if null then feature disable (set default)
      unsigned sampleRate_ms = 0;//sample interval for continuous measurement, 0 for untimed
      unsigned sampleDistance_mm = 0; //for offset and xtalk calibrations, 0 may not be valid, suggested is 400mm
      //data targets
      VL53L0X::MegaCps *rateResult = nullptr;
      VL53L0X::RangingMeasurementData_t *details = nullptr; //if null may get set to temporary union member in api internal data
      CalibrationParameters *refCal = nullptr;
    };

    /** user may wish to extend this actual entities for the items pointed at in ProcessArg */
    class UserAgent {
    public:
      ProcessArg arg;

      /** called when data has been generated, or an error has occured while doing so.
       * might send progress reports once we have enables for doing so.
       * */
      virtual void afterProcess(ProcessRequest process, ProcessResult stage) {
      };

      /** called when something couldn't be handled. Expectation is report to user and a call to reset */
      virtual void unexpected() {
      };
    };//to avoid a lot of null checking

    UserAgent &agent;//user sets this before calling any methods. todo: add constructor arg as reference?

    /** call from Arduino setup. Probably useless*/
    void setup();

    /** call this from your dispatcher, such as loop() in Arduino  or   while(1){WFE(); ... }*/
    void loop();

    NonBlocking(UserAgent &agent, uint8_t i2c_addr = VL53L0X_I2C_ADDR >> 1, uint8_t busNumber = 0) : Api({busNumber, i2c_addr, 400})
                                                                                                     , agent(agent)
                                                                                                     , theXtalkProcess(*this)
                                                                                                     , theOffsetProcess(*this)
                                                                                                     , theCalProcess(*this)
                                                                                                     , theRefSignalProcess(*this) {
      //do no real actions so that we can statically construct
    }

  public:
    /** result of most recent measurement of any type */
    VL53L0X::RangingMeasurementData_t theRangingMeasurementData;
    MeasurementAction theLastMeasurement = Abandoned;

  private:
    ProcessRequest activeProcess = Idle;
    MeasurementAction measurementInProgress = Abandoned;

    struct Waiting {
      const uint8_t *tuning;         //tuning table (does items in tranches)
      unsigned interruptClear; //3 lsbs zero, number of polls for timeout
      unsigned forStop;        //wait on stop complete,, number of polls for timeout
      unsigned onStart;        //wait on start acknowledge (10 samples per call), number of polls for timeout
      unsigned onMeasurement;  //wait on measurement data ready (flag captured by ISR or poll device, 10 polls per call)

      void abandonAll() {
        tuning = nullptr;         //tuning table (does items in tranches)
        interruptClear = 0; //3 lsbs zero, number of polls for timeout
        forStop = 0;        //wait on stop complete,, number of polls for timeout
        onStart = 0;        //wait on start acknowledge (10 samples per call), number of polls for timeout
        onMeasurement = 0;  //wait on measurement data ready (flag captured by ISR or poll device, 10 polls per call)
      }

      Waiting() {
        abandonAll();
      }
    } waiting;

    bool startMeasurement(MeasurementAction action);
    void abandonTasks();

    /** user sets process parameters in their UserAgent.arg structure then calls this method.
     *
     * @returns whether request was accepted.
     * request is not accepted if
     * - any is in progress that is not abandonable
     * - system is not initialized
     *
     * TBD: afterProcess(process, reasonForFailure) may be called before this returns false.
     * */
    bool startProcess(ProcessRequest process);

  private:
    void onStopComplete(bool b);
    //placeholder for what to do when a wait on measurement complete is successful
    void doMeasurementComplete(bool successful);

    void waitForMeasurement(unsigned loops = 250) {
      waiting.onMeasurement = loops;
      //if functional rather than virtual here is where we record the action to take on completion.
    }

    void waitForStop(unsigned loops = 250) {
      waiting.forStop = loops;
      //if functional rather than virtual here is where we record the action to take on completion.
    }

    /**  */
    bool requestMeasurement(MeasurementAction use) {
      //todo: check if any in progress and refuse, until we implement a queue
      switch (use) {
        default:
        case Abandoned:
          return false;
        case forRefCal:
          break;
        case forRate:
          break;
        case forSpads:
          break;
        case forRange:
          break;
        case forOffset:
          break;
        case forXtalk:
          return theXtalkProcess.begin();
      }
    }

  protected:

    class MeasurementProcess {
      friend class NonBlocking;

    protected:
      NonBlocking &nb;

      uint8_t seqConfigCache;//for seq value
    public:
      MeasurementProcess(NonBlocking &dev);
    protected:
      /** overrides must call this, it caches seq config*/
      virtual bool begin() {
        seqConfigCache = nb.PALDevDataGet(SequenceConfig);//in case we keep the PAL updated at all times, unlike what might have been a bug in the past
        return true;
      }

      virtual void startNext() = 0;
      bool onMeasurement();

      void done() {
        /* restore the previous Sequence Config */
        //perhaps conditional on match of Get(SequenceConfig)
        nb.set_SequenceConfig(seqConfigCache, true);
      }
    };

    /** common base for Xtalk and Offset process
     *
      * Perform 50 measurements and compute the averages
      */
    class AveragingProcess : public MeasurementProcess {
      friend class NonBlocking;

    public:
      FixPoint1616_t CalDistanceMilliMeter;
    protected:
      unsigned total_count = 0;//ick: former use of 32bit was excessive
      unsigned sum_ranging = 0;//bug: former use of 16 bit begged for integer overflow
      unsigned sum_fractions = 0;//ick: former ignored fractions, could have lost 25 counts

      FixPoint1616_t sum_signalRate = 0;
      //measurement count
      unsigned measurementRemaining = 0;
    protected:
      explicit AveragingProcess(NonBlocking &dev) : MeasurementProcess(dev) {
      }

      /** overrides must call this */
      bool begin() override;
      /** begin and onmeasurement will call this, it must start a measurement */
      void startNext() override = 0;
      /** implements receiving measurement,  calls averaging stuff then finish of last measurement */
      bool onMeasurement();

      /** @returns whether to continue the process. if not then nb.lastError details why  */
      virtual void alsoSum() {
      }

      /** overrides called when last measurement has been summed into dataset */
      virtual bool finish() {
        done();
        return true;
      };
    };

    /** makes a bunch of measurements then sets XTalkCompensationRateMegaCps.
     * usage:
     * place reflector at known distance
     * .theXtalkProcess.CalDistanceMilliMeter= your value for that;
     * .requestMeasurement(forXtalk) */
    class XtalkProcess : public AveragingProcess {
    private:
      //additional sum
      unsigned sum_spads = 0;//... which we will tolerate only on processors whose natural int is 16 bits
    public:
      explicit XtalkProcess(NonBlocking &dev);
      bool begin() override;
      void startNext() override;
      void alsoSum() override;
      bool finish() override;
    } theXtalkProcess;

    /** makes a bunch of measurements then sets offset.
     * usage:
     * place reflector at known distance
     * .theOffsetProcess.CalDistanceMilliMeter= your value for that;
     * .requestMeasurement(forOffset) */
    class OffsetProcess : public AveragingProcess {
    public:
      explicit OffsetProcess(NonBlocking &dev);
    private:
      bool SequenceStepWasEnabled = false;
    public:
      bool begin() override;
      /** @returns whether to continue the process. if not then nb.lastError details why  */
      bool finish() override;
      void startNext() override;
    } theOffsetProcess;

    /** a pair of measurements which leave behind results accessible for curiousity's sake*/
    class CalProcess : public MeasurementProcess {
    public:
      explicit CalProcess(NonBlocking &dev);
    private:
      bool lastStep = false;//which of two measurements

    public:
      /** the result of the process */
      CalibrationParameters p;
    public:
      bool begin() override;
      /** @returns whether to continue the process. if not then nb.lastError details why  */
      bool onMeasurement();
      void startNext() override;
    } theCalProcess;

    /** single measurement from which a reference signal rate is extracted*/
    class RefSignalProcess : public MeasurementProcess {
    public:
      explicit RefSignalProcess(NonBlocking &dev);
    private:
    public:
      uint16_t rate;//the output of the process
    public:
      bool begin() override;
      /** @returns whether to continue the process. if not then nb.lastError details why  */
      bool onMeasurement();
      void startNext() override;
    } theRefSignalProcess;

    /** complicated bugger:
     * it does the CalProcess
     * then it does a variable number of RefSignalProcesses.
     *
     * our scheduler should tolerate more than one process active, ordered by this guys needs.
     * */
    class SpadSetupProcess : public MeasurementProcess {
    };
  };

  /** NYI
   *
   * each sensor has to have some reset pin control, typically a GPIO pin but if dealing with 8 or more could be an spi register or an I2C expander.
   * */
  struct Resetter {
    //implementation might implement numbers above 64 as located on I2C or SPI expanders
    uint8_t gpioHandle;
    uint8_t i2cAddress;
    uint8_t busNumber;

    bool releaseAndDetect() {
      //todo: release reset pin via gpioHandle
      //todo: spin for 30 us or some such number
      //todo: I2C probe for base address, false if not found
      //todo: call set address at base address argument from resetter
      //todo: I2C probe for new address, return whether found
      //the above probe is instead of traditional 10ms delay and hope it worked. We'll see if that works
      return false;
    }

    static
    void assignAddresses(unsigned tableLength, Resetter *table) {
      //todo: apply reset to all
      //todo: spin for minimum reset time
      while (tableLength-- > 0) {
        table[tableLength].releaseAndDetect();
      }
    }
  };
}
#endif //VL53_NONBLOCKING_H
