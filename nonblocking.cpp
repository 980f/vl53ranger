//
// Copyright 2021 by Andy Heilveil (github/980f)
// Created by andyh on 12/27/21.
//

#include "nonblocking.h"
#include "trynester.h"
#include "algorithm"
#include "vl53l0x_interrupt_threshold_settings.h"

//from safely/cppext lib:
template<typename Scalar> Scalar take(Scalar &owner) {
  Scalar taken = owner;
  owner = static_cast<Scalar>(0);
  return taken;
}

using namespace VL53L0X; //# this file exists to manage entities from this namespace

/**
 * task priorities:
 *   tuning table (does items in tranches)
 *   wait on interrupt clear
 *   wait on stop complete
 *   wait on start acknowledge (10 samples per call)
 *   wait on measurement data ready (flag captured by ISR or poll device, 10 polls per call)
 *
 * data ready subsystem
 *   vhv/phasecal
 *   rate request
 *      user
 *      spad setup
 *   user measurement (single or continuous)
 *   offset cal
 *   xtalk cal
 * */

void NonBlocking::abandonTasks() {
  onStopComplete(false);
  doMeasurementComplete(false);//expected to notify process(failed)
  waiting.abandonAll();
  endProcess(false);
}

//called after tuning settings load.
void NonBlocking::endStaticInit() {

  VL53L0X_SETDEVICESPECIFICPARAMETER(OscFrequencyMHz, (FFread<FixPoint<4, 12>>(RegSystem(0x84))));//# extra parens required by macro processor.
  /* Disable MSRC and TCC by default */
  SetSequenceStepEnable(SEQUENCESTEP_TCC, false);
  SetSequenceStepEnable(SEQUENCESTEP_MSRC, false);

  /* in case someone bypassed the proper setters */
  PALDevDataSet(RangeFractionalEnable, GetFractionEnable());
  PALDevDataSet(CurrentParameters, GetDeviceParameters());//
  PALDevDataSet(SequenceConfig, get_SequenceConfig());//COA
  theRangeProcess.sequenceConfig = PALDevDataGet(SequenceConfig);//let this be the one for ranging, unless user modifies it.
  initRanger(VCSEL_PERIOD_PRE_RANGE, SEQUENCESTEP_PRE_RANGE, VL53L0X_GETDEVICESPECIFICPARAMETER(PreRange));
  initRanger(VCSEL_PERIOD_FINAL_RANGE, SEQUENCESTEP_FINAL_RANGE, VL53L0X_GETDEVICESPECIFICPARAMETER(FinalRange));
  PALDevDataSet(PalState, STATE_IDLE);
  requesting.staticInit = false;
  startProcess(SetupSpads);
  //todo: what if SetupSpads fails to start?
}

/** prioritizing:
 * tuning as that might be used by any process step
 * things that restart the device interface
 * measurement stages
 * measurement requests
 * process steps that need measurements
 * */
void NonBlocking::loop() {
  if (allowing.nothing) {
    return;
  }
  if (requesting.resetHard) {
    //todo: resetter not yet implemented
    requesting.resetSoft = true;
  }
  if (requesting.resetSoft) {
    //todo: implement this
    requesting.dataInit = true;
  }
  TRY {
      //if we are writing to a block of registers do not interrupt that for anything other than write failures
      if (waiting.tuning) {
        for (unsigned perpoll = 10; perpoll-- > 0;) {
          if (!oneTuning(waiting.tuning)) {//invalid records act as terminators
            //maydo: need some diagnostic for tuning configuration failures, might have to throw them
            waiting.tuning = nullptr;
            //onTuningComplete:
            if (requesting.staticInit) {
              endStaticInit();
            }
            break;
          }
        }
        if (waiting.tuning) {
          return;
        }
      }
      if(waiting.compact(*this)){// known user: thresholds for Proximity mode
        return;
      }
      //- if we are asked to init data and similar actions then we need to abandon all unrelated tasks
      if (requesting.dataInit) {
        abandonTasks();//COA
        allowing.nothing = false;//time to try again!
        DataInit();//long but not blocking
        requesting.dataInit = false;//follow DataInit to allow for exceptions
        requesting.staticInit = true;//we know that DataInit set the flag that we would check to set this, so just set it.
      }
      /////////////////////////////
      if (waiting.onStart) {
        uint8_t flags;
        fetch(flags, REG_SYSRANGE_START);
        if (getBit<0>(flags) == 0) {
          waiting.onStart = 0;
        } else {
          if (--waiting.onStart == 0) {
            abandonTasks();
            agent.processEvent(activeProcess, Failed);
          }
          return;//come back in later
        }
      }
      /////////////////////////////////
      if (waiting.onMeasurement) { //todo: replace this counter with a timer,
        if (allowing.gpioAsReadyBit ? gpioReady() : GetMeasurementDataReady()) {
          waiting.onMeasurement = 0;
          doMeasurementComplete(true);
        } else {
          if (--waiting.onMeasurement == 0) {
            //todo: log timeout error
            doMeasurementComplete(false);//will abandon process and tasks
          }
        }
      }
      ///the doMeasurementComplete(true) call often sets requesting.someflag, we wish to review them asap.
      ///it may issue a Stop.
      if (waiting.forStop) {
        if (GetStopCompletedStatus() == 0x00) {
//          //todo: deal with climask clear false.
          auto ignored = ClearInterruptMask(GPIOFUNCTIONALITY_NEW_MEASURE_READY);//copied from adafruit copy of st advice
          waiting.forStop = 0;
          onStopComplete(true);
        } else {
          if (--waiting.forStop == 0) {
            onStopComplete(false);//will abandon process and task
          }
        }
      }

      if (take(requesting.rate)) {//set by user or by spads
        if (inProgress == nullptr) {//there must be a receiver or we don't start a measurement
          //wtf!
          inProgress = &theRateProcess;//to avert NPE's
        }
        startMeasurement(forRate);
        return;
      }

      if (take(requesting.range)) {//set by user or xtalk or offset
        if (inProgress == nullptr) {//there must be a receiver or we don't start a measurement
          //wtf if not OneShot or Continuous
          inProgress = &theRangeProcess;
        }
        startMeasurement(forRange);
        return;
      }

      if (requesting.vhv || requesting.phase) {//traditionally vhv was done prior to phase, and sometimes phase is done by itself
        if (inProgress == nullptr) {
          theCalProcess.doingVhv = take(requesting.vhv);
          inProgress = &theCalProcess;
        }
        startMeasurement(theCalProcess.doingVhv ? forVHV : forPhase);
        if (!theCalProcess.doingVhv) {
          requesting.phase = false;
        }
        return;
      }

      //////
      // init should be automatic:
      auto palstate = GetPalState();
      switch (palstate) {
        case STATE_POWERDOWN://must do datainit
          //if not initialized and not initializing
          if (agent.arg.operatingMode != Powerdown) {
            startProcess(InitData);
          }
          break;
        case STATE_WAIT_STATICINIT://must do static init
          startProcess(InitStatic);
          break;
        case STATE_STANDBY://power up before doing much of anythng
          //if any user requests are pending wake up
          break;
        case STATE_IDLE:
          //if measurement requested start one?
          break;
        case STATE_RUNNING://continoous measurements in progress
          //recognize continuous measurement
          break;
        case STATE_ERROR: //should reset? no code instances of it being set
        //wtf
          break;
        case STATE_UNKNOWN: //sw developer error
        //wtf
          break;
      }

      //todo: if action request act on it:
    CATCH(ERROR_MODE_NOT_SUPPORTED:
          case ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED :
          case ERROR_GPIO_NOT_EXISTING :
          case ERROR_INVALID_PARAMS)
      //todo: report developer errors
      abandonTasks();
    CATCH(ERROR_CONTROL_INTERFACE)
      requesting.resetHard = true;//because we don't know how far we got, at a minimum we need a soft reset
      abandonTasks();
    UNCAUGHT
      abandonTasks();
      //todo: any stored exit routines that apply
      //todo: debug print the error code.
      //// end catches
      ////////////////////////////
  }
  //you get here both on TRY and if you don't return inside any CATCH or the UNCAUGHT clauses
}

void NonBlocking::onStopComplete(bool b) {
  //todo: notify stopped
}

void NonBlocking::doMeasurementComplete(bool successful) {
  if (measurementInProgress == Abandoned) {
    return;
  }
  if (successful) {//collect data
    GetRangingMeasurementData(agent.arg.theRangingMeasurementData);
    agent.arg.theLastMeasurement = take(measurementInProgress);//record user and acknowledge data copied
    agent.arg.peakSignal = GetMeasurementRefSignal();//GetRangingMeasurementData stores this value on the device rather than the associated measurement.
  }
  if (inProgress) {
    inProgress->onMeasurement(successful);
  } else {
    //wtf!
  }
}

bool NonBlocking::startStream(){
  allowing.gpioAsReadyBit = SetGpioConfig(0, {GPIOFUNCTIONALITY_NEW_MEASURE_READY, INTERRUPTPOLARITY_LOW});

  if (agent.arg.sampleRate_ms > 0) {//maydo: check if time looks like microseconds
    if(!SetMeasurementTimingBudgetMicroSeconds(agent.arg.sampleRate_ms * 1000)){
      return false;
    }
    SetDeviceMode(DeviceModes::DEVICEMODE_CONTINUOUS_TIMED_RANGING);
  } else {
    SetDeviceMode(DeviceModes::DEVICEMODE_CONTINUOUS_RANGING);
  }

  if (PALDevDataGet(SequenceConfig) != theRangeProcess.sequenceConfig) {//legacy does this before the stop, might be better after
    set_SequenceConfig(theRangeProcess.sequenceConfig);
  }
  magicStop();

  comm.WrByte(REG_SYSRANGE_START, REG_SYSRANGE_MODE_BACKTOBACK);
  measurementInProgress = forRange;
  waiting.onMeasurement = VL53L0X_DEFAULT_MAX_LOOP;//todo: convert measurement time into a loop count or make this a time!
  return true;
}

bool NonBlocking::startMeasurement(NonBlocking::MeasurementAction action) {
  //default standard range measurement
    uint8_t steps{theRangeProcess.sequenceConfig};
    uint8_t startcode{ REG_SYSRANGE_MODE_START_STOP};
  switch (action) {
    case Abandoned:
      return false;
    case forVHV:
      steps = Mask<6>::places;
      startcode = Mask<0>::places;//REG_SYSRANGE_MODE_START_STOP
      break;
    case forPhase:
      steps = 0;
      startcode = Mask<1>::places;
      break;
    case forRate:
      steps = Mask<7, 6>::places;
      //default start
      break;
    case forRange:
      //both defaults are good
      break;
  }

  if (PALDevDataGet(SequenceConfig) != steps) {//legacy does this before the stop, might be better after
    set_SequenceConfig(steps);
  }
  magicStop();
  comm.WrByte(REG_SYSRANGE_START, startcode);
  //single shot, continuous mode done elsewhere
  waiting.onStart = VL53L0X_DEFAULT_MAX_LOOP;//legacy, need to tune per platform or convert to uSec and wait on a timer.
  measurementInProgress = action;
  waiting.onMeasurement = VL53L0X_DEFAULT_MAX_LOOP;//todo: convert measurement time into a loop count or make this a time!
  return true;
}

void NonBlocking::setup(OperatingMode mode) {
  requesting.dataInit = true;
  agent.arg.operatingMode = mode;
}

void NonBlocking::setProcess(NonBlocking::ProcessRequest process) {
  switch (activeProcess = process) {//record and test
    case Idle:
      inProgress = nullptr;
      break;

    case InitData:
      break;
    case InitStatic:
      inProgress = &theCalProcess;//the only thing static waits on if it doesn't do spads
      break;
    case SetupSpads:
      inProgress = &theSpadder;
      break;
    case CalVhvPhase:
    case CalPhase:
      inProgress = &theCalProcess;
      break;
    case Operate:
      inProgress = &theRangeProcess;
      break;
    case RateTest:
      inProgress = &theRateProcess;
      break;
    case Offset:
      inProgress = &theOffsetProcess;
      break;
    case CrossTalk:
      inProgress = &theXtalkProcess;
      break;
  }
}

/** nominally asynchronous access. All 'real' activity takes place in the loop function. */
bool NonBlocking::startProcess(NonBlocking::ProcessRequest process) {
  if (activeProcess != Idle) { //process block each other
    if (activeProcess == process) {//indicate already running
      agent.processEvent(activeProcess, Busy);
    }
    return false;
  }

  setProcess(process);//a process starts when requested, but might end before we return from this function call.
  //initiators:
  switch (activeProcess) {
    case Idle:
      //todo: if continuous mode is operating we request its end then wait for it to stop
      agent.processEvent(activeProcess /* which we have tested to be Idle */, ProcessResult::Succeeded);
      abandonTasks();
      return true;

    case InitData:
      requesting.dataInit = true;
      return true;
    case InitStatic: {
      if (!get_info_from_device(InfoGroup::SpadStuff)) {
        return endProcess(false);
      }
      /* gets the ref spad from NVM */
      SpadCount ref = VL53L0X_GETDEVICESPECIFICPARAMETER(ReferenceSpad);
      /* check NVM value: two known types, 1 and 0, '1' has a max 32 spads, '0' has a max of 12 */
      if (ref.quantity > (ref.isAperture ? 32 : 12)) {//if true then invalid settings so compute correct ones
        requesting.spads = true;
        return true;//not cancelled
      }
      if (!set_reference_spads(ref)) {
        allowing.nothing = true;
        return requesting.staticInit = false;//quit and return failed
      }
      waiting.tuning = GetTuningSettingBuffer();//refresh tunings
      requesting.vhv = true;
      requesting.phase = true;
      return true;
    }
      break;
    case CalVhvPhase:
      requesting.vhv = true;
      [[fallthrough]];//tradition, call for both or just phase.
    case CalPhase:
      requesting.phase = true;
      break;

    case RateTest:
      requesting.rate = true;
      break;

    case SetupSpads:
      return theSpadder.precheck();
    case Offset:
      if (agent.arg.sampleDistance_mm.raw <= 0) {
        return endProcess(false);
      }
      theOffsetProcess.begin();
      break;
    case CrossTalk:
      if (agent.arg.sampleDistance_mm.raw <= 0) {
        return endProcess(false);
      }
      theXtalkProcess.begin();
      break;

    case Operate: {
      switch (agent.arg.operatingMode) {

        case Powerdown:
          //todo: power down stuff
          break;
        case OnDemand:
          if (allowing.ranging) {
            allowing.gpioAsReadyBit = SetGpioConfig(0, {GPIOFUNCTIONALITY_NEW_MEASURE_READY, INTERRUPTPOLARITY_LOW});
            SetDeviceMode(DeviceModes::DEVICEMODE_SINGLE_RANGING);
            return true;
          } else {
            return endProcess(false);
          }
        case Proximity:
          if (!allowing.ranging) {
            return endProcess(false); //not ready to start
          }
          SetInterruptThresholds(DEVICEMODE_CONTINUOUS_RANGING,agent.arg.proximity);//mode is ignored
          if(!validThresholds()){
            return endProcess(false);//compares won't work properly
          }
          waiting.compact= {InterruptThresholdSettings, SizeofInterruptThresholdSettings};
          [[fallthrough]];
        case DataStream:
          return startStream();
      }
    }
  }
  //software defect!
  return false;//not using endProcess() so that we can debug the bad arguments to this method.
}

bool NonBlocking::gpioReady() {
  return agent.gpioSignal();
}

#if IncludeBlockers

bool NonBlocking::doBlocking(ProcessRequest process) {
  switch (process) {
    case Idle:
      return StopMeasurement();

    case InitData:
      DataInit();
      return true;
    case InitStatic:
      return StaticInit();
    case SetupSpads:
      return PerformRefSpadManagement();
    case CalVhvPhase:
      return PerformRefCalibration();
    case CalPhase:
      return perform_phase_calibration(true);
    case Operate:
      switch (agent.arg.operatingMode) {
        case Powerdown:
          SetPowerMode(POWERMODE_STANDBY_LEVEL1);
          return true;
        case OnDemand:
          return PerformSingleRangingMeasurement(agent.arg.theRangingMeasurementData);
        case DataStream:
        case Proximity:
          //todo: apply proximity window
          if (agent.arg.sampleRate_ms > 0) {
            SetMeasurementTimingBudgetMicroSeconds(agent.arg.sampleRate_ms * 1000);
            SetDeviceMode(DeviceModes::DEVICEMODE_CONTINUOUS_TIMED_RANGING);
          } else {
            SetDeviceMode(DeviceModes::DEVICEMODE_CONTINUOUS_RANGING);
          }
          StartMeasurement();
          return true;
      }
      break;
    case RateTest:
      //todo: was this available in api?
      break;
    case Offset:
      return PerformOffsetCalibration(agent.arg.sampleDistance_mm);
    case CrossTalk:
      return PerformXTalkCalibration(agent.arg.sampleDistance_mm);
  }
  return false;
}

NonBlocking::NonBlocking(NonBlocking::UserAgent &agent, uint8_t i2c_addr, uint8_t busNumber) :
  Api({busNumber, i2c_addr, 400})
  , agent(agent)
  , theRangeProcess(*this)
  , theXtalkProcess(*this)
  , theOffsetProcess(*this)
  , theCalProcess(*this)
  , theRateProcess(*this)
  , theSpadder(*this) {
  //do no real actions so that we can statically construct
}

bool NonBlocking::endProcess(bool successful) {
  agent.processEvent(activeProcess, successful ? ProcessResult::Succeeded : ProcessResult::Failed);
  //might add endwaiting here, contolled by a boolean argument
  setProcess(Idle);
  return successful;
}

bool NonBlocking::update() {
  agent.processEvent(activeProcess, waiting.forSomething() ? Busy : Succeeded);
  return inProgress!= nullptr;
}

#endif
////////////////////////////////////////////

void NonBlocking::MeasurementProcess::onMeasurement(bool successful) {
  nb.endProcess(successful);
}

void NonBlocking::AveragingProcess::begin() {
  sum_ranging = 0;
  sum_fractions = 0;
  total_count = 0;

  measurementRemaining = 50;
  nb.requesting.range= true;
}

void NonBlocking::AveragingProcess::onMeasurement(bool successful) {
  TRACE_ENTRY
  if (!successful) {
    return MeasurementProcess::onMeasurement(false);
  }
  if (nb.agent.arg.theRangingMeasurementData.Range.error == VL53L0X::Range_Valid) {
    sum_ranging += nb.agent.arg.theRangingMeasurementData.Range.milliMeter;
    sum_fractions += nb.agent.arg.theRangingMeasurementData.Range.FractionalPart;
    sum_signalRate += nb.agent.arg.theRangingMeasurementData.SignalRateRtnMegaCps;
    ++total_count;
    alsoSum();
    if (--measurementRemaining > 0) {
      nb.startMeasurement(forRange);//perhaps just set request bit?
      return;
    }
    /* no valid values found */
    if (total_count == 0) {//ick: really should be a larger number, like 90% success rate
      nb.endProcess(false);//allow generic cleanup
    }
    sum_ranging += RangeDatum::carry(sum_fractions);//formerly the fractions got tossed, might have been as high as just under 50.
    nb.endProcess(finish(true));
  } else {
    if (--measurementRemaining > 0) {
      nb.startMeasurement(forRange);//perhaps just set request bit?
      return;
    }
    nb.endProcess(finish(false));
  }
}

void NonBlocking::XtalkProcess::begin() {
/* Disable the XTalk compensation */
  nb.SetXTalkCompensationEnable(false);
  /* Disable the RIT */
  nb.SetLimitCheckEnable(CHECKENABLE_RANGE_IGNORE_THRESHOLD, false);

  AveragingProcess::begin();
}

void NonBlocking::XtalkProcess::alsoSum() {
  sum_spads += nb.agent.arg.theRangingMeasurementData.EffectiveSpadRtnCount.rounded();//980f: formerly truncated
}

bool NonBlocking::XtalkProcess::finish(bool successful) {
  if(!successful){
    //no special cleanup needed
    return false;
  }
  /* FixPoint1616_t / uint16_t = FixPoint1616_t */
  MegaCps StoredMeanSignalRate(sum_signalRate, total_count, 0);//ick: was not rounded prior to 980f
  FixPoint1616_t StoredMeanRange(sum_ranging, total_count);//round the division, also the fractions were merged before this fn was called.
  FixPoint1616_t StoredMeanRtnSpads(sum_spads, total_count);

  /* Round Mean Spads to Whole Number.
   * Typically the calculated mean SPAD count is a whole number or very close to a whole number,
   * therefore any truncation will not result in a significant loss in accuracy.
   * Also, for a grey target at a typical distance of around 400mm, around 220 SPADs will be enabled,
   * therefore any truncation will result in a loss of accuracy of less than 0.5%.
   * 980F: rounding yields half the error of truncation, making it easier to compare to modeling done with true floating point.
   */
  unsigned StoredMeanRtnSpadsAsInt = StoredMeanRtnSpads.rounded();

  /* Round Cal Distance to Whole Number.
   * Note that the cal distance is in mm, therefore no resolution is lost.*/
  unsigned CalDistanceAsInt = CalDistanceMilliMeter.rounded();
  MegaCps XTalkCompensationRateMegaCps={0.0F};

  if (StoredMeanRtnSpadsAsInt != 0 && CalDistanceAsInt != 0 && StoredMeanRange < CalDistanceMilliMeter) {
    /* Apply division by mean spad count early in the calculation to keep the numbers small.
     * This ensures we can maintain a 32bit calculation.
     * Fixed1616 / int := Fixed1616 */
    uint32_t signalXTalkTotalPerSpad = roundedDivide(StoredMeanSignalRate, StoredMeanRtnSpadsAsInt);

    /* Complete the calculation for total Signal XTalk per SPAD
     * Fixed1616 * (Fixed1616 - Fixed1616/int) := (2^16 * Fixed1616)     */
    signalXTalkTotalPerSpad *= (Unity.raw - roundedDivide(StoredMeanRange, CalDistanceAsInt));

    /* Round from 2^16 * Fixed1616, to Fixed1616. */
    XTalkCompensationRateMegaCps = roundedScale(signalXTalkTotalPerSpad, 16);
  }
  nb.SetXTalkCompensationEnable(true);
  nb.SetXTalkCompensationRateMegaCps(XTalkCompensationRateMegaCps);

  return true;
}

NonBlocking::XtalkProcess::XtalkProcess(NonBlocking &dev) : AveragingProcess(dev) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
NonBlocking::OffsetProcess::OffsetProcess(NonBlocking &dev) : AveragingProcess(dev) {
}

void NonBlocking::OffsetProcess::begin() {
  /* Get the value of the TCC */
  SequenceStepWasEnabled = nb.GetSequenceStepEnable(SEQUENCESTEP_TCC);
  /* Disable the TCC */
  nb.SetSequenceStepEnable(SEQUENCESTEP_TCC, false);
  /* Disable the RIT */
  nb.SetLimitCheckEnable(CHECKENABLE_RANGE_IGNORE_THRESHOLD, false);//ick: why don't we save and restore this?
  AveragingProcess::begin();
}

bool NonBlocking::OffsetProcess::finish(bool passthru) {
  if (passthru) {
    /* FixPoint1616_t / uint16_t = FixPoint1616_t */
    FixPoint1616_t StoredMeanRange(sum_ranging + roundedScale(sum_fractions, RangeDatum::epsilon), total_count);
    /* Rounding distances to Whole Number.
     * Note that the cal distance is in mm, therefore no resolution is lost.
     * 980f: but why not retain the fractional part for tracability of the value? Rounding to resolution is unnecessary */
    int32_t OffsetMicroMeter = (CalDistanceMilliMeter.rounded() - StoredMeanRange.rounded()) * 1000;

    /* Apply the calculated offset */
    nb.Data.CurrentParameters.RangeOffsetMicroMeters = OffsetMicroMeter;//record
    nb.SetOffsetCalibrationDataMicroMeter(OffsetMicroMeter);//send to device
//  nb.logError(VL53L0X::ERROR_NONE, __FUNCTION__);
    /* Restore the TCC */
    if (SequenceStepWasEnabled) {
      nb.SetSequenceStepEnable(SEQUENCESTEP_TCC, true);
    }
  } else {
    //undo damage
  }
  return passthru;//done
}


/////////////////////////////////////////////////////

void NonBlocking::CalProcess::onMeasurement(bool successful) {
  if (successful) {
    if (take(doingVhv)) {
      nb.startMeasurement(forPhase);
    } else {
      nb.agent.arg.refCal = nb.get_ref_calibration();
      nb.endProcess(true);
    }
  }
}

NonBlocking::CalProcess::CalProcess(NonBlocking &dev) : MeasurementProcess(dev) {
  //no actions.
}

//////////////

NonBlocking::RateProcess::RateProcess(NonBlocking &dev) : MeasurementProcess(dev) {
  //do nothing here
}

void NonBlocking::RateProcess::onMeasurement(bool successful) {
  if (successful) {
    nb.agent.arg.peakSignal = nb.FFread<FixPoint<9, 7>>(REG_RESULT_PEAK_SIGNAL_RATE_REF);
  }
  nb.endProcess(successful);
}

////////////////////////////////////////////////
NonBlocking::MeasurementProcess::MeasurementProcess(NonBlocking &dev) : nb(dev) {
  //do nothing here
}

////////////////////////////////////////////////
void NonBlocking::Waiting::abandonAll() {
  *this = {};//sometimes C++ is wonderful. This sets all fields to their constructor defaults.
}
bool NonBlocking::Waiting::forSomething()const {
  if(tuning){
    return true;
  }
  if(compact.remaining){
    return true;
  }
  if(onMeasurement){
    return true;
  }
  if(onStart){
    return true;
  }
  if(forStop){
    return false;//todo: review ignoring waiting for stop.
  }
  return false;
}
/////////////////////////////////////////////////////////////////////////
/* called after vhv and phase measurements are successfully completed */
void NonBlocking::SpadSetupProcess::refreshCalibration() {
  nb.agent.arg.refCal = nb.get_ref_calibration();
  //process continues at forEachMeasurement
}

//on rate measurements:
bool NonBlocking::SpadSetupProcess::forEachMeasurement() {
  auto peakSignalRateRef = nb.agent.arg.peakSignal; //perform_ref_signal_measurement(0);

  if (peakSignalRateRef > targetRefRate) { /* Signal rate measurement too high, switch to APERTURE SPADs */
    if (sc.isAperture && sc.quantity == minimumSpadCount) {//then can't reduce any further
      /* Signal rate still too high after setting the minimum number of APERTURE spads.
         * Can do no more therefore set the min number of aperture spads as the result. */
      sc = {minimumSpadCount, true};//undo changes done by enable_ref_spads
      //todo: ensure laststage reports success
      return laststage();//save and exit
    }

    if (!sc.isAperture) {//first try to reduce via switching to aperture spads
      sc = {minimumSpadCount, true};
      nb.Data.SpadData.enables.clear();
      /* Increment to the first APERTURE spad */
      scanner.moveto(true, startSelect);
      if (scanner(sc) && setAndCheck()) {
        nb.requesting.rate = true;
        return true;
      }
      //todo: process failure
      return false;//attempt to set fewer spads failed
    }
    //we decide whether our "one past" here is better or worse than last below
    auto rateExcess = peakSignalRateRef - targetRefRate;//no abs() required as we have checked the relative sizes.
    if (rateExcess > rateDeficit) { /* Previous spad map produced a closer measurement, so choose this. */
      scanner.undoLast();
      --sc.quantity;//tradition, is anyone going to look at this? todo: see if old code sent this to the DevData
      if (!setAndCheck()) {
        //signal failure.
        return false;
      }
    }
    //present is best that we can do
    return laststage();
  } else {
    rateDeficit = targetRefRate - peakSignalRateRef;//no abs() required as we have checked the relative sizes.
    //increase spad count
    if (scanner({1, sc.isAperture})) {
      ++sc.quantity;
/* Proceed to apply the additional spad and perform measurement. */
      if (setAndCheck()) {
        //ST code did not check this instance as best I can tell
        nb.requesting.rate = true;
        return true;
      } else {
        return false;//todo signal failure
      }
    } else {
      //todo: signal failure
      return false;
    }
  }
}

bool NonBlocking::SpadSetupProcess::laststage() {
//todo: report process success
  nb.VL53L0X_SETDEVICESPECIFICPARAMETER(RefSpadsInitialised, true);
  nb.VL53L0X_SETDEVICESPECIFICPARAMETER(ReferenceSpad, sc);
  return true;
}

void NonBlocking::SpadSetupProcess::onMeasurement(bool successful) {
  if (!successful) {
    MeasurementProcess::onMeasurement(false);//use standard cleanup.
  }
  switch (nb.agent.arg.theLastMeasurement) {
    case forRate:
      if (!forEachMeasurement()) {
        //do we need to do any cleanup on failure?
        MeasurementProcess::onMeasurement(false);//use standard cleanup.
      }
      break;
    case forVHV: //then issue phase measurement
      nb.requesting.phase = true; //probably already is
      break;
    case forPhase:
      refreshCalibration();
      break;
    case Abandoned://won't happen
    case forRange://never asked for
      //wtf
      break;
  }
}

NonBlocking::SpadSetupProcess::SpadSetupProcess(NonBlocking &dev) : MeasurementProcess(dev), scanner(dev.Data.SpadData.goodones, dev.Data.SpadData.enables) {
  //other fields default constructors are fine.
}

bool NonBlocking::SpadSetupProcess::precheck() {
  scanner.restart();//clear all spad selects and set search pointer to 0th
  /* Enable Minimum NON-APERTURE Spads */
  sc = {minimumSpadCount, false};

  if (!scanner(sc)) {//if we can't find the minimum we don't even try. This is about existence, not utility.
    return nb.endProcess(false);
  }

  targetRefRate = nb.PALDevDataGet(targetRefRate);//different data types, convert just once.

  {
    SysPopper ffer = nb.push(Private_Pager, 0x01, 0x00);
    nb.comm.WrByte(REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0);
    nb.comm.WrByte(REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, SpadArray::MaxCount);
  }

  nb.comm.WrByte(REG_GLOBAL_CONFIG_REF_EN_START_SELECT, startSelect.absolute());
  nb.comm.WrByte(REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0);
  nb.requesting.vhv = true;
  nb.requesting.phase = true;
  //loop() actually fires up the measurement of vhv
  return true;
}

bool NonBlocking::SpadSetupProcess::setAndCheck() {
  nb.set_ref_spad_map(scanner.spadArray);

  SpadArray checkSpadArray;
  nb.get_ref_spad_map(checkSpadArray);
  /* Compare spad maps. If not equal report error. */
  return scanner.spadArray != checkSpadArray;
}
////////////////////////////////////////////////////////
NonBlocking::RangeProcess::RangeProcess(NonBlocking &dev) : MeasurementProcess(dev) {
}

void NonBlocking::RangeProcess::onMeasurement(bool successful) {
  if (nb.activeProcess == Operate) {//then "may I have another"
    if (nb.agent.arg.operatingMode > OperatingMode::OnDemand) {
      nb.requesting.range = true;//here is where we would restart a timer that defers checking ready until it is nearly due
    }
  }
  MeasurementProcess::onMeasurement(successful);
}
/////////////////////////////////////////////////////////////////
bool NonBlocking::Waiting::CompactTable::operator()(NonBlocking &nb) {
  unsigned qty= std::min(remaining,10U);
  if(qty>0) {
    nb.load_compact(item, qty);
    item += qty;
    remaining -= qty;
  }
  return remaining>0;
}
