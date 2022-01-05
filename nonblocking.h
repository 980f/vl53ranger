//
// Created by andyh on 12/27/21.
//

#ifndef VL53_NONBLOCKING_H
#define VL53_NONBLOCKING_H

#include "vl53l0x_api.h"

namespace VL53L0X {

  /** one instance for each sensor.
   * sensor address management is outside the scope of this driver
   *
   * This guy entails all of the various processes of ST's Api that took more than most of a millisecond.
   * It is a bit gnarly as each such process is decomposed into phases and there are subclasses in here that for each phase consider all the processes.
   *
   * Adding a new process entails distributing its logic into the various phase handlers.
   * There is a base class MeasurementProcess which you should extend, allowing to bring the source for some of the phases into close proximity in the source code.
   *
   * */
  class NonBlocking : public VL53L0X::Api {

  public:

    /** among other things determines the code that starts a measurement  */
    enum MeasurementAction : uint8_t {
      Abandoned = 0   //for state machine reset.
      , forVHV   //vhv and phase
      , forPhase //
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
      , InitData   //includes tuning table and other settings that shouldn't need to change often
      , InitStatic //does SetupSpads, some tuning parameters
      , SetupSpads //part of init data but callable for debug of device
      , CalVhvPhase    // call when ??? staticinit
      , CalPhase      //call when vcsel pulse width changes, save and restore from nvm?
      , OneShot
      , Continuous //references 'timed', will use non timed version for sample rate of 0
      , RateTest   //use may wish to evaluate ambient light when deciding upon
      , Offset
      , CrossTalk
    };

    enum ProcessResult {
      Busy        //such a process is in progress
      , Queued    //accepted, doesn't get initiated until the next loop() is called
      , Active    //chugging along
      , Succeeded //finished, data will have been delivered via pointers in ProcessArg
      , Failed
    };

    /** data shared by background processing and the user application */
    struct ProcessArg {
      /** when you have enough writes to the chip that you have to break them up into groups to not stall the whole program: */
      uint8_t *tuningTable = nullptr;//todo: implement the "only byte writes" version.
      /** sample interval for continuous measurement, 0 for untimed */
      unsigned sampleRate_ms = 0;
      /** hardware reset control. will be a 'virtual pin number' where values beyond the Arduino GPIO refer to other means of controlling the XSHUT pin */
      unsigned gpioPin = ~0;//~0 is clearly not a legitimate Arduino pin designation
      /** //for offset and xtalk calibrations, 0 may not be valid, suggested is 400mm */
      MilliMeter sampleDistance_mm {0};

      /** diagnostic for who asked for the measurement, set when data is updated */
      MeasurementAction theLastMeasurement = Abandoned;
      //data targets
      /** result of most recent measurement of any type */
      VL53L0X::RangingMeasurementData_t theRangingMeasurementData;//arg.details often points to this
      MegaCps peakSignal; //rate measurements, computed from theRangingMeasurementData
      CalibrationParameters refCal;//vhv phase processes
      //program statistics, updated by processes but not used by them
      unsigned measurements = 0;//incremented each time data is acquired for any purpose
    };

    /** collation of call backs and their arguments, as well as process request arguments */
    class UserAgent {
    public:
      ProcessArg arg;

      /** called when data has been generated, or an error has occured while doing so.
       * might send progress reports once we have enables for doing so.
       * */
      virtual void processEvent(ProcessRequest process, ProcessResult stage) {
      };

      /** called when something couldn't be handled. Expectation is report to user and a call to reset. Caught exceptions might be reported here, then a processEvent will be called. */
      virtual void unexpected() {
      };

      /** called when we are waiting for a measurement and have been told to configure the VL53 signal and so on.*/
      virtual bool gpioSignal() {
        return false;//user override use arg.gpioPin to report on the hardware signal from the VL53
      }
    };

    /** a mixin' set at construction time: */
    UserAgent &agent;

    /** intended for call from Arduino setup. will reset state as best we can. */
    void setup();

    /** call this from your dispatcher, such as loop() in Arduino  or   while(1){WFE(); ... }*/
    void loop();

    /** user sets process parameters in their UserAgent.arg structure then calls this method.
       *
       * @returns whether request was accepted.
       * request is not accepted if
       * - any is in progress that is not abandonable
       * - system is not initialized
       *
       * TBD: processEvent(process, reasonForFailure) may be called before this returns false.
       * */
    bool startProcess(ProcessRequest process);

#if IncludeBlockers
    /** initially run the old code */
    bool doBlocking(ProcessRequest process);
#endif

    /** normally statically constructed */
    explicit NonBlocking(UserAgent &agent, uint8_t i2c_addr = VL53L0X_I2C_ADDR >> 1, uint8_t busNumber = 0);

  private:
    //#using bit fields instead of booleans for atomicity, guard against future multithreading
#if MultiThreaded
#define RQBIT(rq) unsigned rq:1
#else
#define RQBIT(rq) bool rq
#endif
    /** these are inspected when no activity is in progress, usually set by startProcess, cleared by the related process finishing */
    struct Requesting {
      RQBIT(vhv);   //periodically for drift compensation, powerup
      RQBIT(phase); //periodically for drift compensation, powerup
      RQBIT(rate);  //spad, diagnostics,
      RQBIT(spads); //powerup, diagnostics
      RQBIT(range); //oneshot, sticks on for continuous, xtalk, offset
      RQBIT(staticInit);//error recovery, powerup
      RQBIT(dataInit);  //error recovery, powerup
      RQBIT(resetSoft); //error recovery, powerup
      RQBIT(resetHard);//error recovery, powerup
      void clear() {
        vhv=phase=rate=spads=range=staticInit=dataInit=resetSoft=resetHard=false;
      }

      Requesting(){ // NOLINT(cppcoreguidelines-pro-type-member-init)
        clear();
      }
    } requesting;

    /** some state bits */
    struct Allowing {
      bool gpioAsReadyBit = false;//set when configured
      bool measurements = false;//set when inits are completed enough to invoke acquisition,
      bool ranging = false; //can do range measurements, cleared when any calprocess is invoked, set when completed.
      //note: when ranging is enabled app must check desired ranging mode and parameters against actual and request action which might temporarily clear it
      /** when 'nothing' is set here we have a fatal configuration or hardware error */
      bool nothing = true;//cleared by datainit, but could be set by static init
    } allowing;

    ProcessRequest activeProcess = Idle;
    MeasurementAction measurementInProgress = Abandoned;

    struct Waiting {
      const uint8_t *tuning = nullptr;         //tuning table (does items in tranches)
      unsigned interruptClear = 0; //expect 3 lsbs of mask are zero, number of polls for timeout
      unsigned forStop = 0;        //wait on stop complete,, number of polls for timeout
      unsigned onStart = 0;        //wait on start acknowledge (10 samples per call), number of polls for timeout
      unsigned onMeasurement = 0;  //wait on measurement data ready (check gpio or flag captured by ISR or poll device, 10 polls per call)
      /** drop all expectations */
      void abandonAll();
    } waiting;

    /** sets sequence config then issues a start */
    bool startMeasurement(MeasurementAction action);

    /** signal failure on active task and clear all requests */
    void abandonTasks();

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

  protected:

    class MeasurementProcess {
      friend class NonBlocking;

    protected:
      NonBlocking &nb;
      uint8_t seqConfigCache;//for seq value
    protected:
      explicit MeasurementProcess(NonBlocking &dev);
    protected:
      /** overrides must call this, it caches seq config*/
      virtual bool begin() {
        seqConfigCache = nb.PALDevDataGet(SequenceConfig);//in case we keep the PAL updated at all times, unlike what might have been a bug in the past
        return true;
      }

      /** triggers what is presumed to be a single shot measurement (someone else must set DeviceMode before this gets called )*/
      virtual void startNext();
      /** the non-blocking loop calls this when a measurement is ready.
      @return whether it has been deal with, ELSE the loop will abaondonAllTasks!
       */
      virtual bool onMeasurement(bool successful);

      bool bedone(bool failed) {
        /* restore the previous Sequence Config */
        //perhaps conditional on match of Get(SequenceConfig)
        nb.set_SequenceConfig(seqConfigCache, true);
        return failed;
      }

    public:

    };

    /** serves oneshot and continuous range requests */
    class RangeProcess:public MeasurementProcess {
    public:
      explicit RangeProcess(NonBlocking &dev);
    protected:
      bool onMeasurement(bool successful) override;
    }theRangeProcess;

    /** common base for Xtalk and Offset process
     *
      * Perform 50 measurements and compute the averages
      */
    class AveragingProcess : public MeasurementProcess {
      friend class NonBlocking;

    public:
      FixPoint1616_t CalDistanceMilliMeter;
    protected:
      unsigned total_count {0};//ick: former use of 32bit was excessive
      unsigned sum_ranging {0};//bug: former use of 16 bit begged for integer overflow
      //using signed type so that when we round the residual can be negative for rounded up:
      int sum_fractions {0};//ick: former ignored fractions, could have lost 25 counts

      MegaCps sum_signalRate {0};
      //measurement count
      unsigned measurementRemaining {0};
    protected:
      explicit AveragingProcess(NonBlocking &dev) : MeasurementProcess(dev) {
      }

      /** overrides must call this */
      bool begin() override;
      /** begin and onmeasurement will call this, it must start a measurement */
      void startNext() override = 0;
      /** implements receiving measurement,  calls averaging stuff then finish of last measurement */
      bool onMeasurement(bool successful) override;

      /** @returns whether to continue the process. if not then nb.lastError details why  */
      virtual void alsoSum() {
      }

      /** overrides called when last measurement has been summed into dataset */
      virtual bool finish(bool passthru) {
        return passthru;
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
      bool finish(bool successful) override;
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
      bool finish(bool successful) override;
      void startNext() override;
    } theOffsetProcess;

    /** vhv or phasecal
     * */
    class CalProcess : public MeasurementProcess {
    public:
      explicit CalProcess(NonBlocking &dev);
    public://temporary exposure, unless theCalProcess itself is private enough
      bool doingVhv = false;//which of two measurements

    public:
      bool begin() override;
      /** @returns whether to continue the process. if not then nb.lastError details why  */
      bool onMeasurement(bool successful) override;
      void startNext() override;
    } theCalProcess;

    /** single measurement from which a reference signal rate is extracted */
    class RateProcess : public MeasurementProcess {
    public:
      explicit RateProcess(NonBlocking &dev);
    private:
    public:
      bool begin() override;
      /** @returns whether to continue the process. if not then nb.lastError details why  */
      bool onMeasurement(bool successful) override;
      void startNext() override;
    } theRateProcess;

    /** complicated bugger:
     * it does the CalProcess for vhv and phase then it does a variable number rate measurements.
     * From ST source code (with errors corrected):
     * This initialization procedure determines the minimum amount of reference spads to be enables to achieve a target reference signal rate and should be performed once during initialization.
     *  Either aperture or non-aperture spads are applied but never both.
     * Firstly non-aperture spads are set, beginning with minSpadCount spads, and increased one spad at a time until the closest measurement to the target rate is achieved.
     *
    * If the target rate is exceeded when minSpadCount non-aperture spads are enabled, initialization is performed instead with aperture spads.
    *
    * When setting spads, a 'Good Spad Map' is applied.
    *
    * This procedure operates within a SPAD window of interest of a maximum 44 spads (SpadArray::maxcount).
    * The start point is fixed to 180 (Api::startSelect), which lies towards the end of the non-aperture quadrant and runs in to the adjacent aperture quadrant.
     *
     * From ST's actual code:
     * try minimum nonapeture spads, if too great do the rest of the algorithm with aperture spads
     * while too low add spads, if greater then compare excess to deficit of prior test to choose between the two.
     *
     * our code:
     * start with min + non
     * if too great then if non switch to aper else compare to previous pick one and exit
     * else add one of the present type.
 */

    class SpadSetupProcess : public MeasurementProcess {
      MegaCps targetRefRate;//FYI: PAL targetRefRate exists solely to feed this guy potentially from a tunings block.
      SpadArray::Scanner scanner;
      SpadCount sc;
      uint32_t rateDeficit = ~0;//init to wildly bad value

      /** send spad settings to device, read back and report on whether the settings stuck */
      bool setAndCheck();

      bool refreshCalibration();
      bool forEachMeasurement();
      bool laststage();

    public:
      explicit SpadSetupProcess(NonBlocking &dev);
      bool onMeasurement(bool successful) override;
      bool precheck();
    } theSpadder;

    /** object that is notified when data is ready */
    MeasurementProcess *inProgress = nullptr;

  protected:
    //// state machine fragments:
    void endStaticInit();
    bool gpioReady();
    void setProcess(ProcessRequest process);
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
