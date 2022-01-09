//
// Created by andyh on 12/27/21.
//

#ifndef VL53_NONBLOCKING_H
#define VL53_NONBLOCKING_H

#include "vl53l0x_api.h"

namespace VL53L0X {

  /** make one instance for each sensor.
   * sensor address management is outside the scope of this driver (except see Resetter in this file)
   *
   * This guy entails all of the various processes of ST's Api that took more than most of a millisecond.
   * It is a bit gnarly as each such process is decomposed into phases and there are subclasses in here that for each phase consider all the processes.
   *
   * Adding a new process entails distributing its logic into the various phase handlers.
   * There is a base class MeasurementProcess which you should extend, allowing to bring the source for some of the phases into close proximity in the source code.
   * //////////////////////////////////////////////////////////////////////////////////////////////////
   * There are actions which are requested by processes, and processes are requested by the application.
   * Multiple actions can be pending and they have a fixed priority.
   * Only one process can be active.
   *
   * There are types of measurement, the differences are in SequenceConfig and Start bit written to the start register.
   *
   * arguments needed by the processes, and results of them are in a structure allocated on the agent.
   * An agent is the application's interface to the background activities of the API.
   * the background execution is in a member called loop() named for where to call it, not for what it does.
   * */
  class NonBlocking : public VL53L0X::Api {

  public:

    /** intended use after initialization and calibration */
  enum OperatingMode {
    //not operating
    Powerdown=0,
    /** app will request measurements when it wants one*/
    OnDemand,
    /** app wants a stream of values */
    DataStream,
    /** app sets up GPIO to be an "in range" indicator and then ignores it.*/
    Proximity,
  };

/** actions that must complete before others are attempted */
    enum ProcessRequest {
      Idle          //STOPS any active process
      , InitData    //includes tuning table and other settings that shouldn't need to change often
      , InitStatic  //does SetupSpads, some tuning parameters
      , SetupSpads  //part of init data but callable for debug of device
      , CalVhvPhase //call when vcsel pulse width changes, and static init
      , CalPhase    //call when vcsel pulse width changes, save and restore from nvm?
      , Offset      //zero the distance readings
      , CrossTalk   //remove effects of internal reflections and the like.
      , RateTest   //use may wish to evaluate ambient light when evaluating sensor behavior
      , Operate  //see operatingMode
    };

    /** notification of process progress. Busy only occurs when the application tries to poke the driver. */
    enum ProcessResult {
      Busy        //such a process is already in progress, might enable sending with each measurement as a watchdog reset kind of thing or to allow progress reports
      , Succeeded //finished, data will have been delivered via pointers in ProcessArg
      , Failed    //finished, don't expect results although partial success might have altered state.
    };

    /** type of acquisition.
     * among other things determines the code that starts a measurement  */
    enum MeasurementAction : uint8_t {
      Abandoned = 0   //for state machine reset.
      , forVHV   //vhv and phase
      , forPhase //
      , forRate     //rate requests, direct and spad setup
      , forRange  //prime use, user measurement (single or continuous)
    };

    /** data shared by background processing and the user application */
    struct ProcessArg {
      //application parameters
      OperatingMode operatingMode=Powerdown;
      /** sample interval for continuous measurement, 0 for untimed */
      unsigned sampleRate_ms = 0;
      /** for proximity mode */
      RangeWindow proximity;
      /** hardware reset control. will be a 'virtual pin number' where values beyond the Arduino GPIO refer to other means of controlling the XSHUT pin */
      unsigned gpioPin = ~0;//~0 is clearly not a legitimate Arduino pin designation
      /** //for offset and xtalk calibrations, 0 may not be valid, suggested is 400mm */
      MilliMeter sampleDistance_mm {0};
      //////////////////////////////////////
      //responses to application
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

      /** called when data has been generated, or an error has occurred while doing so.
       * might send progress reports once we have enables for doing so.
       * */
      virtual void processEvent(ProcessRequest process, ProcessResult stage) {
      };

      /** called when we are waiting for a measurement and have been told to configure the VL53 signal and so on.*/
      virtual bool gpioSignal() {
        return false;//user override use arg.gpioPin to report on the hardware signal from the VL53
      }

    };

    /** a mixin' set at construction time: */
    UserAgent &agent;

    /** intended for call from Arduino setup. will reset state as best we can. */
    void setup(OperatingMode mode);//or just set ProcessArgs.

    /** call this from your dispatcher, such as loop() in Arduino  or   while(1){WFE(); ... }*/
    void loop(uint32_t millisecondClock);

    /** user sets process parameters in their UserAgent.arg structure then calls this method.
       *
       * @returns whether request was accepted.
       * request is not accepted if
       * - any is in progress that is not abandonable
       * - system is not initialized
       * - any per process requirement not met such as realistic sample distance for xtalk and offset measurements
       * TBD: processEvent(process, reasonForFailure) may be called before this returns false.
       * */
    bool startProcess(NonBlocking::ProcessRequest process, uint32_t now);

    /** make driver call the agent with some status info */
    bool update();

#if IncludeBlockers
    /** initially run the old code as cleaned up by 980F, eventually will be rebuilt using nonblockngs pieces. */
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
      RQBIT(spads); //request process
      RQBIT(range); //oneshot, sticks on for continuous, xtalk, offset
      RQBIT(staticInit);//error recovery, powerup
      RQBIT(dataInit);  //error recovery, powerup
      RQBIT(resetSoft); //error recovery, powerup
      RQBIT(resetHard);//error recovery, powerup
      void clear() {
        vhv = phase = rate = spads = range = staticInit = dataInit = resetSoft = resetHard = false;
      }

      Requesting() { // NOLINT(cppcoreguidelines-pro-type-member-init)
        clear();
      }
    } requesting;

    /** some state bits. todo:1 actually set and honor them all */
    struct Allowing {
      bool gpioAsReadyBit = false;//set when configured
      bool measurements = false;//set when inits are completed enough to invoke acquisition,
      bool ranging = false; //can do range measurements, cleared when any calprocess is invoked, set when completed.
      //note: when ranging is enabled app must check desired ranging mode and parameters against actual and request action which might temporarily clear it
      /** when 'nothing' is set here we have a fatal configuration or hardware error */
      bool nothing = true;//cleared by datainit, but could be set by static init
    } allowing;

    bool forSomething();
    struct Waiting {
      /** when there are enough writes to the chip that you have to break them up into groups to not stall the whole program: */
      const uint8_t *tuning = nullptr;         //tuning table with terminator record (does items in tranches)
      //uniform tuning table with count (does items in tranches)
      struct CompactTable {
        const DeviceByte *item = nullptr;
        unsigned remaining=0;
        bool operator()(NonBlocking &nb);
      } compact;
      unsigned forStop = 0;        //wait on stop complete, number of polls for timeout
      unsigned onStart = 0;        //wait on start acknowledge (10 samples per call), number of polls for timeout
      unsigned onMeasurement = 0;  //wait on measurement data ready (check gpio or flag captured by ISR or poll device, 10 polls per call)
      /** drop all expectations */
      void abandonAll();
      bool forSomething() const;
    } waiting;

    /** sets sequence config and issues a start */
    bool startMeasurement(NonBlocking::MeasurementAction action, uint32_t now);

    /** signal failure on active task and clear all requests */
    void abandonTasks();

  private:
    void onStopComplete(bool b);
    //placeholder for what to do when a wait on measurement complete is successful
    void doMeasurementComplete(bool successful);

//////////////////////////////////////////////////////////////////////
  private: //results were moved into agent.arg instead of the related process class, so these classes are now totally hidden

    class MeasurementProcess {
      friend class NonBlocking;

    protected:
      NonBlocking &nb;
      explicit MeasurementProcess(NonBlocking &dev);

      /** the non-blocking loop calls this when a measurement is ready.
      @return whether it has been deal with, ELSE the loop will abaondonAllTasks!
       */
      virtual void onMeasurement(bool successful);
    };

    ProcessRequest activeProcess = Idle;
    /** object that is notified when data is ready */
    MeasurementProcess *inProgress = nullptr;
    MeasurementAction measurementInProgress = Abandoned;

    void setProcess(ProcessRequest process);
    bool endProcess(bool successful);
///////////////////////////////////////////////////////////////////////////////////////////////
    /** serves oneshot and continuous range requests */
    class RangeProcess : public MeasurementProcess {
    public:
      uint8_t sequenceConfig;//it is not clear how this gets set, apparently applications directly do so.
      explicit RangeProcess(NonBlocking &dev);
    protected:
      void onMeasurement(bool successful) override;
    } theRangeProcess;
/////////////////////////////////////////////////////////////////////////////////////////////////
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

      void onMeasurement(bool successful) override;

      /** @returns whether to continue the process. if not then nb.lastError details why  */
      virtual void alsoSum() {
      }

      /** overrides called when last measurement has been summed into dataset */
      virtual bool finish(bool successful) {
        return successful;
      };
      virtual void begin();
    };
///////////////////////////////////////////////////////////////////////////////////////
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
      void begin() override;
      void alsoSum() override;
      bool finish(bool successful) override;
    } theXtalkProcess;
//////////////////////////////////////////////////////////////////////////////////////////
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
      void begin() override;
      /** @returns whether to continue the process. if not then nb.lastError details why  */
      bool finish(bool successful) override;
    } theOffsetProcess;
////////////////////////////////////////////////////////////////////////////////////////////////
    /** vhv or phasecal */
    class CalProcess : public MeasurementProcess {
    public:
      explicit CalProcess(NonBlocking &dev);
    public://temporary exposure, unless theCalProcess itself is private enough
      bool doingVhv = false;//which of two measurements

    public:
      /** @returns whether to continue the process. if not then nb.lastError details why  */
      void onMeasurement(bool successful) override;
    } theCalProcess;
///////////////////////////////////////////////////////////////////////////////////////////////////
    /** single measurement from which a reference signal rate is extracted */
    class RateProcess : public MeasurementProcess {
    public:
      explicit RateProcess(NonBlocking &dev);
    private:
    public:
      /** @returns whether to continue the process. if not then nb.lastError details why  */
      void onMeasurement(bool successful) override;
    } theRateProcess;

////////////////////////////////////////////////////////////////////////////////////
    /** Spad selection process: a complicated bugger:
     * it does the CalProcess for vhv and phase then it does a variable number of rate measurements as it changes which spads are selected.
     *
     * From ST source code (with some errors corrected):
     * This initialization procedure determines the minimum amount of reference spads to be enables to achieve a target reference signal rate and should be performed once during initialization.
     * Either aperture or non-aperture spads are applied but never both.
     * Firstly non-aperture spads are set, beginning with minSpadCount spads, and increased one spad at a time until the closest measurement to the target rate is achieved.
     *
    * If the target rate is exceeded when minSpadCount non-aperture spads are enabled, initialization is performed instead with aperture spads.
    *
    * When setting spads, a 'Good Spad Map' is applied.
    *
    * This procedure operates within a SPAD window of interest of a maximum 44 spads (SpadArray::maxcount).
    * The start point is fixed to 180 (Api::startSelect), which lies towards the end of the non-aperture quadrant and runs in to the adjacent aperture quadrant.
     * //end ST's comment
     *
     * From ST's actual code:
     * try minimum non-aperture spads, if too great do the rest of the algorithm with aperture spads
     * while too low add spads, if greater then compare excess to deficit of prior test to choose between the two.
     *
     * our code:
     * start with min + non
     * if too great then if non switch to aper else compare to previous, pick one of them and exit
     * else add one of the present type.
 */

    class SpadSetupProcess : public MeasurementProcess {
      MegaCps targetRefRate;//FYI: PAL targetRefRate exists solely to feed this guy potentially from a tunings block.
      SpadArray::Scanner scanner;
      SpadCount sc;
      uint32_t rateDeficit = ~0;//init to wildly bad value

      /** send spad settings to device, read back and report on whether the settings stuck */
      bool setAndCheck();

      /** handles vhv/phase cal measurements*/
      void refreshCalibration();
      /** handles rate measurements*/
      bool forEachMeasurement();
      /** cleanup after spads determined */
      bool laststage();

    public:
      explicit SpadSetupProcess(NonBlocking &dev);
      void onMeasurement(bool successful) override;
      /** does pre first measurement stuff, which might fail */
      bool precheck();
    } theSpadder;

  protected:
    //// state machine fragments:
    void endStaticInit();
    bool gpioReady();

    bool startStream(uint32_t now);
    /** max time a measurement might take */
    uint32_t measurementTime();
  };
////////////////////////////////////////////////////////////////////////////////////
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

    static void assignAddresses(unsigned tableLength, Resetter *table) {
      //todo: apply reset to all
      //todo: spin for minimum reset time
      while (tableLength-- > 0) {
        table[tableLength].releaseAndDetect();
      }
    }
  };
}
#endif //VL53_NONBLOCKING_H
