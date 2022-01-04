//
// Copyright 2021 by Andy Heilveil (github/980f)
// Created by andyh on 12/27/21.
//

#include "nonblocking.h"
#include "trynester.h"

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
  doMeasurementComplete(false);
  waiting.abandonAll();
}

bool NonBlocking::staticInitStage1() {
  if (!get_info_from_device(InfoGroup::SpadStuff)) {
    return false;
  }
  /* set the ref spad from NVM */
  SpadCount ref = VL53L0X_GETDEVICESPECIFICPARAMETER(ReferenceSpad);
  /* check NVM value: two known types, 1 and 0, '1' has a max 32 spads, '0' has a max of 12 */
  if (ref.quantity > (ref.isAperture ? 32 : 12)) {//if true then invalid settings so compute correct ones
    requesting.spads = true;
    return true;//not cancelled
  }
  return afterSpadsSet(ref);
}

bool NonBlocking::afterSpadsSet(SpadCount &ref) {
  if (!set_reference_spads(ref)) {
    allowing.nothing = true;
    return false;
  }

  waiting.tuning = GetTuningSettingBuffer();
  return true;//not quite done yet
}

//call after tuning settings load.
void NonBlocking::endStaticInit() {
  /* Set interrupt config to new sample ready. Failure is now ignored as we are using enums and all real errors will THROW */
  allowing.gpioAsReadyBit = SetGpioConfig(0, {DEVICEMODE_SINGLE_RANGING, GPIOFUNCTIONALITY_NEW_MEASURE_READY, INTERRUPTPOLARITY_LOW});
  auto fix412 = FFread<FixPoint<4, 12>>(RegSystem(0x84));
  VL53L0X_SETDEVICESPECIFICPARAMETER(OscFrequencyMHz, fix412);//conversion from 412 to 1616 is inferred by compiler
  /* After static init actions, some device parameters may be changed, so update them */
  PALDevDataSet(RangeFractionalEnable, GetFractionEnable());
  auto CurrentParameters = GetDeviceParameters();
  PALDevDataSet(CurrentParameters, CurrentParameters);
  /* read the sequence config and save it */
  PALDevDataSet(SequenceConfig, get_SequenceConfig());

  /* Disable MSRC and TCC by default */
  SetSequenceStepEnable(SEQUENCESTEP_TCC, false);
  SetSequenceStepEnable(SEQUENCESTEP_MSRC, false);
  initRanger(VCSEL_PERIOD_PRE_RANGE, SEQUENCESTEP_PRE_RANGE, VL53L0X_GETDEVICESPECIFICPARAMETER(PreRange));
  initRanger(VCSEL_PERIOD_FINAL_RANGE, SEQUENCESTEP_FINAL_RANGE, VL53L0X_GETDEVICESPECIFICPARAMETER(FinalRange));

  PALDevDataSet(PalState, STATE_IDLE);
  requesting.staticInit = false;
}

void NonBlocking::loop() {
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
      //- if we are asked to init data and similar actions then we need to abandon all unrelated tasks
      if (requesting.dataInit) {
        abandonTasks();
        allowing.nothing = false;//time to try again!
        DataInit();//long but not blocking
        requesting.dataInit = false;//follow DataInit to allow for exceptions
      }
//      RQBIT(resetSoft); //error recovery, powerup
//      RQBIT(resetHard);//error recovery, powerup

      if (requesting.staticInit) {
        if (!requesting.spads) {
          requesting.staticInit = staticInitStage1();
        }
        //waiting on tuning is checked prior to getting here
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
            agent.afterProcess(activeProcess, Failed);
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
      //No else here so that the meas complete action can invoke a waitOnStop and get a quick check
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

      if (requesting.spads) { //powerup, diagnostics
        //if process not running then start it
        //if stopped running then WTF are we getting here?

      }

      //      RQBIT(vhv);   //periodically for drift compensation, powerup
//      RQBIT(phase); //periodically for drift compensation, powerup
//      RQBIT(rate);  //spad, diagnostics,
//      RQBIT(range); //oneshot, sticks on for continuous, xtalk, offset
      //////
      // init should be automatic, can add an overall 'be running' for power management, but not yet.
      auto palstate = GetPalState();
      switch (palstate) {
        case STATE_POWERDOWN://must do datainit
          //if not initialized and not initializing
          startProcess(InitData);
          break;
        case STATE_WAIT_STATICINIT://must do static init
          startProcess(InitStatic);
          break;
        case STATE_STANDBY://power up before doing much of anythng
          //if any user requests are pending wake up
          break;
        case STATE_IDLE:
          //if measurement requested start one
          break;
        case STATE_RUNNING://continoous measurements in progress
          //recognize continuous measurement
          break;
        case STATE_ERROR: //should reset? no code instanced of it being ste
          break;
        case STATE_UNKNOWN: //sw developer error
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
      //todo: device must be detected and fully reinit
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
  if (!successful) {
    if (!inProgress->onMeasurement(successful)) {
      abandonTasks();
    }
    return;
  }

  auto measok = GetRangingMeasurementData(agent.arg.theRangingMeasurementData);
  if (measok == VL53L0X::ERROR_NONE) {
    agent.arg.theLastMeasurement = take(measurementInProgress);
    agent.arg.peakSignal = GetMeasurementRefSignal();//GetRangingMeasurementData stores this value on the device rather than the associatd measurement.

    if (inProgress) {
      if (!inProgress->onMeasurement(successful)) {
        abandonTasks();
      }
    } else {
      //must have been abandoned
    }
  } else {
    measurementInProgress = Abandoned;
    //todo: notify process failure
  }
}

bool NonBlocking::startMeasurement(NonBlocking::MeasurementAction action) {
  //any setting of seqconfig or the like has been done
  //if not continuous then:
  measurementInProgress = action;
  return true;
}

void NonBlocking::setup() {
  //nothing to do so far.
  //we don't want to delay the loop() for init, we do that with a statemachine in the loop
  //this would be a place to declare the  mode of our use if it doesn't change dynamically.
  //usage modes:
  //  continuous gauge supplying a value stream
  //  ditto but using a GPIO as data ready indicator
  //  set thresholds and GPIO is simple "present" detector.
}

bool NonBlocking::startProcess(NonBlocking::ProcessRequest process) {
  if (activeProcess != Idle) {
    if (activeProcess == process) {//indicate already running
      agent.afterProcess(activeProcess, Busy);
      return false;
    }
  }
  activeProcess = process;//but we may terminate it before returning
  switch (process) {
    case Idle:
      abandonTasks();
      //todo: defer if Stoptask is activated
      agent.afterProcess(activeProcess, ProcessResult::Succeeded);
      return true;
    case InitI2c:
      break;
    case InitData:
      requesting.dataInit = true;
//      [[fallthrough]]
    case InitStatic:
      requesting.staticInit = true;
      requesting.spads = true;
//      [[fallthrough]]
    case CalVhvPhase:
      requesting.vhv = true;
    case CalPhase:
      requesting.phase = true;
      break;
    case OneShot:
      SetDeviceMode(DeviceModes::DEVICEMODE_SINGLE_RANGING);
      startMeasurement(forRange);
      break;
    case Continuous:
      SetDeviceMode(agent.arg.sampleRate_ms == 0 ? DeviceModes::DEVICEMODE_CONTINUOUS_RANGING : DeviceModes::DEVICEMODE_CONTINUOUS_TIMED_RANGING);
      startMeasurement(forRange);
      break;
    case RateTest:
      startMeasurement(forRate);
      break;
    case SetupSpads:
//todo: return false if can't make measurements. (state issues_
      requesting.spads = true;
      return true;
    case Offset:
      if (agent.arg.sampleDistance_mm.raw <= 0) {
        return false;
      }
      break;
    case CrossTalk:
      if (agent.arg.sampleDistance_mm.raw <= 0) {
        return false;
      }
      break;
  }
  //software defect!
  return false;
}

bool NonBlocking::gpioReady() {
  return agent.gpioSignal();
}

bool NonBlocking::requestMeasurement(NonBlocking::MeasurementAction use) {
  //todo: check if any in progress and refuse, until we implement a queue
  switch (use) {
    default:
    case Abandoned:
      return false;
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
    case forVHV:
      break;
    case forPhase:
      break;
  }
  return false;
}

#if IncludeBlockers

bool NonBlocking::doBlocking(ProcessRequest process) {
  switch (process) {
    case Idle:
      return StopMeasurement();
    case InitI2c:
      //todo: not actually a process
      return false;
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
    case OneShot:
      //NB: data is stored in the PAL: PALDevDataGet(LastRangeMeasure) = pRangingMeasurementData;
      //it would be efficient if we were to dig down to that instead of having a duplicate
      return PerformSingleRangingMeasurement(agent.arg.theRangingMeasurementData);
    case Continuous:
      if (agent.arg.sampleRate_ms) {
        SetMeasurementTimingBudgetMicroSeconds(agent.arg.sampleRate_ms * 1000);
        SetDeviceMode(DeviceModes::DEVICEMODE_CONTINUOUS_TIMED_RANGING);
      } else {
        SetDeviceMode(DeviceModes::DEVICEMODE_CONTINUOUS_RANGING);
      }
      StartMeasurement();
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

#endif
////////////////////////////////////////////

bool NonBlocking::MeasurementProcess::onMeasurement(bool successful) {
  return false;//default is to abandon the task.
}

bool NonBlocking::AveragingProcess::begin() {
  sum_ranging = 0;
  sum_fractions = 0;
  total_count = 0;

  measurementRemaining = 50;
  return true;
}

bool NonBlocking::AveragingProcess::onMeasurement(bool successful) {
  TRACE_ENTRY
  if (!successful) {
    //todo: some cleanup?
    return false;
  }
  if (nb.agent.arg.theRangingMeasurementData.Range.error == VL53L0X::Range_Valid) {
    sum_ranging += nb.agent.arg.theRangingMeasurementData.Range.milliMeter;
    sum_fractions += nb.agent.arg.theRangingMeasurementData.Range.FractionalPart;
    sum_signalRate += nb.agent.arg.theRangingMeasurementData.SignalRateRtnMegaCps;
    ++total_count;
    alsoSum();
    if (--measurementRemaining > 0) {
      return true;
    }
    /* no valid values found */
    if (total_count == 0) {//ick: really should be a larger number, like 90% success rate
      nb.agent.afterProcess(nb.activeProcess, ProcessResult::Failed);
      return false;//allow generic cleanup
    }
    sum_ranging += RangeDatum::carry(sum_fractions);//formerly the fractions got tossed, might have been as high as just under 50.
    finish();
    return false;
  } else {
    //todo: abend the process instead of skipping perhaps all but one measurement
    return true; //try for some more.
  }
}

bool NonBlocking::XtalkProcess::begin() {
  if (AveragingProcess::begin()) {
/* Disable the XTalk compensation */
    nb.SetXTalkCompensationEnable(false);
    /* Disable the RIT */
    nb.SetLimitCheckEnable(CHECKENABLE_RANGE_IGNORE_THRESHOLD, false);
    nb.startMeasurement(forXtalk);
    return true;
  }
  return false;
}

void NonBlocking::XtalkProcess::alsoSum() {
  sum_spads += nb.agent.arg.theRangingMeasurementData.EffectiveSpadRtnCount.rounded();//980f: formerly truncated
}

bool NonBlocking::XtalkProcess::finish() {
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
  unsigned StoredMeanRtnSpadsAsInt = roundedScale(StoredMeanRtnSpads, 16);

  /* Round Cal Distance to Whole Number.
   * Note that the cal distance is in mm, therefore no resolution is lost.*/
  unsigned CalDistanceAsInt = CalDistanceMilliMeter.rounded();
  MegaCps XTalkCompensationRateMegaCps;

  if (StoredMeanRtnSpadsAsInt == 0 || CalDistanceAsInt == 0 || StoredMeanRange >= CalDistanceMilliMeter) {
    XTalkCompensationRateMegaCps = 0.0F;
  } else {
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

  return false;
}

NonBlocking::XtalkProcess::XtalkProcess(NonBlocking &dev) : AveragingProcess(dev) {
}

void NonBlocking::XtalkProcess::startNext() {
}

//////////////////////////////////////////////////////////////////////////////////////////////
NonBlocking::OffsetProcess::OffsetProcess(NonBlocking &dev) : AveragingProcess(dev) {
}

bool NonBlocking::OffsetProcess::begin() {
  if (AveragingProcess::begin()) {
    /* Get the value of the TCC */
    SequenceStepWasEnabled = nb.GetSequenceStepEnable(SEQUENCESTEP_TCC);
    /* Disable the TCC */
    nb.SetSequenceStepEnable(SEQUENCESTEP_TCC, false);
    /* Disable the RIT */
    nb.SetLimitCheckEnable(CHECKENABLE_RANGE_IGNORE_THRESHOLD, false);//ick: why don't we save and restore this?
    //todo: actually fire off first measurement
    return true;
  } else {
    return false;
  }
}

bool NonBlocking::OffsetProcess::finish() {
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
  return false;//done
}

void NonBlocking::OffsetProcess::startNext() {
//todo: seqconfig and start
}

/////////////////////////////////////////////////////

void NonBlocking::CalProcess::startNext() {
  uint8_t magic = lastStep ? 0 : Bitter(6);
  nb.set_SequenceConfig(magic, false);//todo: debate the false here, it may have been a bug in the original code.
  nb.comm.WrByte(REG_SYSRANGE_START, REG_SYSRANGE_MODE_START_STOP | magic);
}

bool NonBlocking::CalProcess::begin() {
  MeasurementProcess::begin();
  lastStep = false;
  startNext();
  return true;
}

bool NonBlocking::CalProcess::onMeasurement(bool successful) {
  if (lastStep) {
    /* if measurement ok */{
      p = nb.get_ref_calibration();
    }
    /* restore the previous Sequence Config */
    nb.set_SequenceConfig(seqConfigCache, true);
    return false;
  } else {
    //vhv done
    lastStep = true;
    startNext();
    return true;
  }
}

NonBlocking::CalProcess::CalProcess(NonBlocking &dev) : MeasurementProcess(dev) {
  //no actions.
}

bool NonBlocking::RefSignalProcess::onMeasurement(bool successful) {
  // if measurement ok:
  rate = nb.FFread<uint16_t>(REG_RESULT_PEAK_SIGNAL_RATE_REF);
  done();
  return false;
}

bool NonBlocking::RefSignalProcess::begin() {
  //todo: startNext
  return true;
}

void NonBlocking::RefSignalProcess::startNext() {
  nb.set_SequenceConfig(0xC0, false);
}

NonBlocking::RefSignalProcess::RefSignalProcess(NonBlocking &dev) : MeasurementProcess(dev), rate(0) {
  //do nothing here
}

NonBlocking::MeasurementProcess::MeasurementProcess(NonBlocking &dev) : nb(dev), seqConfigCache(0) {
  //do nothing here
}

void NonBlocking::MeasurementProcess::startNext() {
  {
    auto popper = nb.magicWrapper();
    nb.comm.WrByte(REG_SYSRANGE_stopper, nb.PALDevDataGet(StopVariable));
  }
  nb.comm.WrByte(REG_SYSRANGE_START, REG_SYSRANGE_MODE_START_STOP | REG_SYSRANGE_MODE_SINGLESHOT);
  nb.waiting.onStart = VL53L0X_DEFAULT_MAX_LOOP;//legacy, need to tune per platform or convert to uSec and wait on a timer.
}

void NonBlocking::Waiting::abandonAll() {
  *this = {};//sometimes C++ is wonderful. This sets all fields to their constructor defaults.
}

bool NonBlocking::SpadSetupProcess::oldcode() {
  targetRefRate = nb.PALDevDataGet(targetRefRate);//different data types, convert just once.
  /*
   * Initialize Spad arrays.
   * Currently the good spad map is initialised to 'All good'.
   * This is a short term implementation. The good spad map will be
   * provided as an input.
   */
  nb.Data.SpadData.enables.clear();
  {
    SysPopper ffer = nb.push(Private_Pager, 0x01, 0x00);
    nb.comm.WrByte(REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0);
    nb.comm.WrByte(REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, SpadArray::MaxCount);
  }

  nb.comm.WrByte(REG_GLOBAL_CONFIG_REF_EN_START_SELECT, startSelect.absolute());
  nb.comm.WrByte(REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0);
//perform_ref_calibration
  nb.requesting.vhv = true;
  nb.requesting.phase = true;
  return true; //resumes at stage2
}

/* called after vhv and phase measurements are successfully completed */
bool NonBlocking::SpadSetupProcess::stage2() {
  scanner.restart();
  /* Enable Minimum NON-APERTURE Spads */
  sc = {minimumSpadCount, false};

  if (!scanner(sc)) {
    return false;//todo process failure
  }
  nb.requesting.rate = true;
  return true; //continues at stage3
}

//on rate measurements:
bool NonBlocking::SpadSetupProcess::stage3() {
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
      --sc.quantity;//tradition, is anyone going to look at this?
      if (setAndCheck()) {
        return laststage();
      } else {
        //signal failure.
        return false;
      }
    } else {
      //present is best that we can do
      return laststage();
    }
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

bool NonBlocking::SpadSetupProcess::onMeasurement(bool successful) {
  if (!successful) {
    return false;//allow standard cleanup.
  }
  switch (nb.agent.arg.theLastMeasurement) {
    case forRate:
      stage3();
      return true;
    case forVHV: //then issue phase measurement
      nb.requesting.phase = true; //probably already is
      return true;
    case forPhase:
      //if (!perform_ref_calibration()) {
      //    return false;
      //  }
      //todo: if no error:
      stage2();
      return true;

    case forSpads:
    case forOffset:
    case forXtalk:
    case Abandoned:
    case forRange:
      return false;
  }
  return false;
}

NonBlocking::SpadSetupProcess::SpadSetupProcess(NonBlocking &dev) : MeasurementProcess(dev), scanner(dev.Data.SpadData.goodones, dev.Data.SpadData.enables) {
  //other fields default constructors are fine.
}
