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
      , CalVhvPhase    // call when ???
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
      uint8_t *tuningTable = nullptr;//if not same as present one then apply it, if null then feature disable (set default)
      unsigned sampleRate_ms = 0;//sample interval for continuous measurement, 0 for untimed
      unsigned gpioPin = ~0;//~0 is clearly not a legitimate Arduino pin designation
      MilliMeter sampleDistance_mm {0}; //for offset and xtalk calibrations, 0 may not be valid, suggested is 400mm
      //data targets
      /** result of most recent measurement of any type */
      VL53L0X::RangingMeasurementData_t theRangingMeasurementData;//arg.details often points to this
      MegaCps peakSignal;
      /** diagnostic for who asked for the above measurement */
      MeasurementAction theLastMeasurement = Abandoned;
      CalibrationParameters refCal;
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
      virtual void afterProcess(ProcessRequest process, ProcessResult stage) {
      };

      /** called when something couldn't be handled. Expectation is report to user and a call to reset */
      virtual void unexpected() {
      };

      /** called when we are waiting for a measurement and have been told to configure the VL53 signal and so on.*/
      virtual bool gpioSignal() {
        return false;
      }
    };

    /** a mixin' set at construction time: */
    UserAgent &agent;

    /** call from Arduino setup. Probably useless*/
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
       * TBD: afterProcess(process, reasonForFailure) may be called before this returns false.
       * */
    bool startProcess(ProcessRequest process);

#if IncludeBlockers
    bool doBlocking(ProcessRequest process);
#endif

    /** normally statically constructed */
    explicit NonBlocking(UserAgent &agent, uint8_t i2c_addr = VL53L0X_I2C_ADDR >> 1, uint8_t busNumber = 0) : Api({busNumber, i2c_addr, 400})
                                                                                                              , agent(agent)
                                                                                                              , theXtalkProcess(*this)
                                                                                                              , theOffsetProcess(*this)
                                                                                                              , theCalProcess(*this)
                                                                                                              , theRefSignalProcess(*this)
                                                                                                              ,theSpadder(*this){
      //do no real actions so that we can statically construct
    }

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
      unsigned interruptClear = 0; //3 lsbs zero, number of polls for timeout
      unsigned forStop = 0;        //wait on stop complete,, number of polls for timeout
      unsigned onStart = 0;        //wait on start acknowledge (10 samples per call), number of polls for timeout
      unsigned onMeasurement = 0;  //wait on measurement data ready (flag captured by ISR or poll device, 10 polls per call)

      void abandonAll();
    } waiting;

    bool startMeasurement(MeasurementAction action);
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

    /**  */
    bool requestMeasurement(MeasurementAction use);

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
      CalibrationParameters p;//UserAgent args will often point to this
    public:
      bool begin() override;
      /** @returns whether to continue the process. if not then nb.lastError details why  */
      bool onMeasurement(bool successful);
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
      bool onMeasurement(bool successful);
      void startNext() override;
    } theRefSignalProcess;

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
 */

    class SpadSetupProcess : public MeasurementProcess {
      MegaCps targetRefRate;//FYI: PAL targetRefRate exists solely to feed this guy potentially from a tunings block.
      SpadArray::Scanner scanner;
      SpadCount sc;
      uint32_t rateDeficit=~0;//init to wildly bad value

      bool setAndCheck(){
        nb.set_ref_spad_map(scanner.spadArray);

        SpadArray checkSpadArray;
        nb.get_ref_spad_map(checkSpadArray);
        /* Compare spad maps. If not equal report error. */
        return scanner.spadArray != checkSpadArray;
        return false;
      }
    public:
      SpadSetupProcess(NonBlocking &dev);
      bool onMeasurement(bool successful) override;
      bool oldcode();
      bool stage2();
      bool stage3();
      bool laststage();
    } theSpadder;

    /** object that is notified when data is ready */
    MeasurementProcess *inProgress = nullptr;

    //// state machine fragments:
    bool staticInitStage1();
    void endStaticInit();
    bool afterSpadsSet(SpadCount &ref);
    bool gpioReady();
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
