//
// Copyright 2021 by Andy Heilveil (github/980f)
// Created by andyh on 12/27/21.
//

#include "nonblocking.h"
#include "trynester.h"

#define THROW(error) nb.comm.wirer.Throw( __FUNCTION__,__LINE__,error);

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

void NonBlocking::abandonTasks(){
  onStopComplete(false);
  doMeasurementComplete(false);
}

void NonBlocking::loop() {
  CATCH {
    case ERROR_NONE: {
      if(waiting.tuning){
        for(unsigned perpoll=10;perpoll-->0;){
          if(!oneTuning(waiting.tuning)){//invalid records act as terminators
            waiting.tuning= nullptr;
            break;
          }
        }
        if (waiting.tuning) {
          return;
        }
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
            agent.afterProcess(activeProcess,Failed);
          }
          return;//come back in later
        }
      }
      /////////////////////////////////
      if (waiting.onMeasurement) {
        if (GetMeasurementDataReady()) {
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
          auto climask = ClearInterruptMask(GPIOFUNCTIONALITY_NEW_MEASURE_READY);//copied from adafruit copy of st advice
          //todo: deal with climask false.
          waiting.forStop=0;
          onStopComplete(true);
        } else {
          if (--waiting.forStop == 0) {
            onStopComplete(false);//will abandon process and task
          }
        }
      }
      //////
      // init should be automatic, can add an overall 'be running' for power management, but not yet.
      auto palstate=GetPalState();
      switch(palstate){
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
        case STATE_UNKNOWN:
          break;
        case STATE_ERROR: //should reset?
          break;
      }

      //todo: if action request act on it:
    }  ///////////////////////////// end of try clause
      break;
    //////////////////////////////////////
    /// catch clauses
    case ERROR_MODE_NOT_SUPPORTED:
    case ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED:
    case ERROR_GPIO_NOT_EXISTING:
    case ERROR_INVALID_PARAMS:
      //todo: developer errors
      abandonTasks();
      break;
    case ERROR_CONTROL_INTERFACE: //todo: suberrors
      //todo: device must be detected and fully reinit
      abandonTasks();
      break;
    default:
      abandonTasks();
      //todo: any stored exit routines that apply
      //todo: debug print the error code.
      break;
    //// end catches
    ////////////////////////////
  }
}

void NonBlocking::onStopComplete(bool b) {
  //todo: notify stopped
}

void NonBlocking::doMeasurementComplete(bool successful) {
  if (measurementInProgress == Abandoned) {
    return;
  }
  if (!successful) {
    //todo: handle failed measurement, most likely abandon all processes and mark 'need init'
    abandonTasks();
    return;
  }
  auto measok = GetRangingMeasurementData(theRangingMeasurementData);
  if (measok == VL53L0X::ERROR_NONE) {
    theLastMeasurement = take(measurementInProgress);
    switch (theLastMeasurement) {
      case forXtalk:
        if (theXtalkProcess.onMeasurement()) {
          theXtalkProcess.startNext();
        } else {
          //todo: do we notify on xtalk meas complete?
        }
        return;

      case forOffset:
        if (theOffsetProcess.onMeasurement()) {
          theOffsetProcess.startNext();
        } else {
          //todo: do we notify on offset meas complete?
        }
        return;

      case Abandoned:
        return; //already handled prior to switch
      case forRange: //the actual reason this device exists!
        //todo: notify we have a datum
        agent.afterProcess(activeProcess,ProcessResult::Succeeded);
        activeProcess=Idle;//todo: if continuous remain in active state.
        break;
      case forRefCal:
        if (theCalProcess.onMeasurement()) {
          theCalProcess.startNext();//phase one
        } else {
          //todo notify phasecal complete.
        }
        break;
      case forRate:
        break;
      case forSpads:
        break;
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
}

bool NonBlocking::startProcess(NonBlocking::ProcessRequest process) {
  if(activeProcess!=Idle){
    if(activeProcess==process){
      agent.afterProcess(activeProcess,Busy);
      return false;
    }

  }
  switch (process) {
    case Idle:
      agent.afterProcess(activeProcess,ProcessResult::Succeeded);
      return true;
    case InitI2c:

      break;
    case InitStatic:
      break;
    case InitData:
      break;
    case OneShot:
      activeProcess=OneShot;
      SetDeviceMode(DeviceModes::DEVICEMODE_SINGLE_RANGING);
      StartMeasurement();
      startMeasurement(forRange);
      break;
    case Continuous:
      activeProcess=Continuous;
      SetDeviceMode(agent.arg.sampleRate_ms==0?DeviceModes::DEVICEMODE_CONTINUOUS_RANGING:DeviceModes::DEVICEMODE_CONTINUOUS_TIMED_RANGING);
      StartMeasurement();
      startMeasurement(forRange);
      break;
    case RefCal:
      break;
    case RateTest:
      break;
    case SetupSpads:
      break;
    case Offset:
      break;
  }
  return false;
}

////////////////////////////////////////////

bool NonBlocking::AveragingProcess::begin() {
  sum_ranging = 0;
  sum_fractions=0;
  total_count = 0;
  if (CalDistanceMilliMeter.raw <= 0) {//ICK: type was unsigned, and so this is a compare to zero. Most likely a non-zero value is necessary
    measurementRemaining = 0;//COA
    return false;
  }
  measurementRemaining = 50;
  return true;
}

bool NonBlocking::AveragingProcess::onMeasurement() {
  if (nb.theRangingMeasurementData.Range.error == VL53L0X::Range_Valid) {
    sum_ranging += nb.theRangingMeasurementData.Range.MilliMeter;
    sum_fractions+=nb.theRangingMeasurementData.Range.FractionalPart;
    sum_signalRate += nb.theRangingMeasurementData.SignalRateRtnMegaCps;
    ++total_count;
    alsoSum();
    if (--measurementRemaining > 0) {
      return true;
    }
    /* no valid values found */
    if (total_count == 0) {//ick: really should be a larger number, like 90% success rate
      nb.comm.wirer.Throw( __FUNCTION__,__LINE__,VL53L0X::ERROR_RANGE_ERROR);
      return false;
    }
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
  sum_spads += nb.theRangingMeasurementData.EffectiveSpadRtnCount.rounded();//980f: formerly truncated
}

bool NonBlocking::XtalkProcess::finish() {
  /* FixPoint1616_t / uint16_t = FixPoint1616_t */
  FixPoint1616_t StoredMeanSignalRate(sum_signalRate, total_count, 0);//ick: was not rounded prior to 980f
  FixPoint1616_t StoredMeanRange(sum_ranging, total_count);//round the division
  FixPoint1616_t StoredMeanRtnSpads(sum_spads, total_count);

  /* Round Mean Spads to Whole Number.
   * Typically the calculated mean SPAD count is a whole number or very close to a whole number,
   * therefore any truncation will not result in a significant loss in accuracy.
   * Also, for a grey target at a typical distance of around 400mm, around 220 SPADs will be enabled,
   * therefore any truncation will result in a loss of accuracy of less than 0.5%.
   */
  unsigned StoredMeanRtnSpadsAsInt = roundedScale(StoredMeanRtnSpads, 16);

  /* Round Cal Distance to Whole Number.
   * Note that the cal distance is in mm, therefore no resolution is lost.*/
  unsigned CalDistanceAsInt = roundedScale(CalDistanceMilliMeter, 16);
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
    signalXTalkTotalPerSpad *= (Unity.raw - roundedDivide(StoredMeanRange , CalDistanceAsInt));

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
  FixPoint1616_t StoredMeanRange(sum_ranging + roundedScale(sum_fractions,RangeDatum::epsilon), total_count);
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
//
//const decltype(Api::CalibrationParameters::PhaseCal) phaseMask = Mask<6, 0>::places;

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

bool NonBlocking::CalProcess::onMeasurement() {
  if (lastStep) {
    /* if measurement ok */{
      p= nb.get_ref_calibration();
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

bool NonBlocking::RefSignalProcess::onMeasurement() {
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

NonBlocking::RefSignalProcess::RefSignalProcess(NonBlocking &dev) :MeasurementProcess(dev),rate(0){
  //do nothing here
}


NonBlocking::MeasurementProcess::MeasurementProcess(NonBlocking &dev) : nb(dev),seqConfigCache(0) {
  //do nothing here
}
