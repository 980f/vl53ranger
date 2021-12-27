//
// Created by andyh on 12/27/21.
//

#include "nonBlocking.h"

//from safely/cppext lib:
template<typename Scalar> Scalar take(Scalar &owner) {
  Scalar taken = owner;
  owner = static_cast<Scalar>(0);
  return taken;
}

using namespace VL53L0X; //# this file exists to manage entities from this namespace

void NonBlocking::inLoop() {
  if (waitOnStart) {
    Erroneous<uint8_t> flags;
    fetch(flags, REG_SYSRANGE_START);
    if (flags.isOk()) {
      if (getBit<0>(flags.wrapped) == 0) {
        waitOnStart = 0;
      } else {
        if (--waitOnStart == 0) {
          //toodo: abandon pending process and measurements
        }
      }
    } else {
      //todo: handle error on waiting for start, most likely same as timeout.
    }
  }
  if (waitOnMeasurementComplete) {
    Erroneous<bool> value;
    value = GetMeasurementDataReady();
    if (value.isOk()) {
      if (value == 1) {
        waitOnMeasurementComplete = 0;
        doMeasurementComplete(true);
      } else {
        if (--waitOnMeasurementComplete == 0) {
          //todo: log timeout error
          doMeasurementComplete(false);
        }
      }
    } else {
      //todo: decide if we stop on communications error, not sure if st code did/
    }
  }
  //No else here so that the meas complete action can invoke a waitOnStop and get a quick check
  if (waitOnStop) {
    auto StopCompleted = GetStopCompletedStatus();
    if (StopCompleted.isOk()) {
      if (StopCompleted == 0x00) {
        auto climask = ClearInterruptMask(GPIOFUNCTIONALITY_NEW_MEASURE_READY);//copied from adafruit copy of st advice
        onStopComplete(true);
      } else {
        if (--waitOnStop == 0) {
          onStopComplete(false);
        }
      }
    } else {
      //todo: handle comm error.
    }
  }
  //todo: if action request act on it:
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
  }
  auto measok = GetRangingMeasurementData(theRangingMeasurementData);
  if (measok == VL53L0X::ERROR_NONE) {
    theLastMeasurement = take(measurementInProgress);
    switch (theLastMeasurement) {
      case forXtalk:
        if (theXtalkProcess.onMeasurement(theRangingMeasurementData)) {
          requestMeasurement(forXtalk);
        } else {
          //todo: do we notify on xtalk meas complete?
        }
        return;
      case Abandoned:
        return; //already handled prior to switch
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

bool NonBlocking::XtalkProcess::begin() {
  if (XTalkCalDistance.raw <= 0) {//ICK: type was unsigned, and so this is a compare to zero.
//        return ERROR_INVALID_PARAMS;
    return false;
  }
/* Disable the XTalk compensation */
  dev.SetXTalkCompensationEnable(0);
  /* Disable the RIT */
  dev.SetLimitCheckEnable(CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);
  measurementRemaining = 50;
  dev.startMeasurement(forXtalk);
}

bool NonBlocking::XtalkProcess::onMeasurement(const RangingMeasurementData_t &RangingMeasurementData) {
  //process each:
  if (RangingMeasurementData.rangeError == 0) {
    sum_ranging += RangingMeasurementData.RangeMilliMeter;
    sum_signalRate += RangingMeasurementData.SignalRateRtnMegaCps;
    sum_spads += RangingMeasurementData.EffectiveSpadRtnCount.rounded();//980f: formerly truncated
    ++total_count;
  } else {
    //todo: abend the process
    return false;
  }

  if (--measurementRemaining == 0) {   //post process
    /* no valid values found */
    if (total_count == 0) {
//todo          return ERROR_RANGE_ERROR;
    }
    /* FixPoint1616_t / uint16_t = FixPoint1616_t */
    FixPoint1616_t xTalkStoredMeanSignalRate(sum_signalRate, total_count, 0);//ick: was not rounded prior to 980f
    FixPoint1616_t xTalkStoredMeanRange(sum_ranging, total_count);//round the division
    FixPoint1616_t xTalkStoredMeanRtnSpads(sum_spads, total_count);

    /* Round Mean Spads to Whole Number.
     * Typically the calculated mean SPAD count is a whole number or very close to a whole number,
     * therefore any truncation will not result in a significant loss in accuracy.
     * Also, for a grey target at a typical distance of around 400mm, around 220 SPADs will be enabled,
     * therefore any truncation will result in a loss of accuracy of less than 0.5%.
     */
    uint32_t xTalkStoredMeanRtnSpadsAsInt = roundedScale(xTalkStoredMeanRtnSpads, 16);

    /* Round Cal Distance to Whole Number.
     * Note that the cal distance is in mm, therefore no resolution is lost.*/
    uint32_t xTalkCalDistanceAsInt = roundedScale(XTalkCalDistance, 16);
    FixPoint1616_t XTalkCompensationRateMegaCps;

    if (xTalkStoredMeanRtnSpadsAsInt == 0 || xTalkCalDistanceAsInt == 0 || xTalkStoredMeanRange >= XTalkCalDistance) {
      XTalkCompensationRateMegaCps = 0.0F;
    } else {
      /* Apply division by mean spad count early in the calculation to keep the numbers small.
       * This ensures we can maintain a 32bit calculation.
       * Fixed1616 / int := Fixed1616 */
      uint32_t signalXTalkTotalPerSpad = (xTalkStoredMeanSignalRate) / xTalkStoredMeanRtnSpadsAsInt;//ick: round divide

      /* Complete the calculation for total Signal XTalk per SPAD
       * Fixed1616 * (Fixed1616 - Fixed1616/int) := (2^16 * Fixed1616)     */
      signalXTalkTotalPerSpad *= (Unity.raw - (xTalkStoredMeanRange / xTalkCalDistanceAsInt));//ick: round divide

      /* Round from 2^16 * Fixed1616, to Fixed1616. */
      XTalkCompensationRateMegaCps = roundedScale(signalXTalkTotalPerSpad, 16);
    }

    /* Enable the XTalk compensation */
    dev.SetXTalkCompensationEnable(true);

    /* Enable the XTalk compensation */
    dev.SetXTalkCompensationRateMegaCps(XTalkCompensationRateMegaCps);

    dev.logError(VL53L0X::ERROR_NONE, "Xtalk Process");
    //todo: how shall we signal 'process complete'?
    return false;
  } else {
    //caller makes the next request
    return true;//
  }
}

NonBlocking::XtalkProcess::XtalkProcess(NonBlocking &dev) : dev(dev) {
}

NonBlocking::OffsetProcess::OffsetProcess(NonBlocking &dev) : dev(dev) {
}

bool NonBlocking::OffsetProcess::begin() {
  /* Get the value of the TCC */
  SequenceStepWasEnabled = dev.GetSequenceStepEnable(SEQUENCESTEP_TCC);
  /* Disable the TCC */
   dev.SetSequenceStepEnable(SEQUENCESTEP_TCC, false);
  /* Disable the RIT */
  dev.SetLimitCheckEnable(CHECKENABLE_RANGE_IGNORE_THRESHOLD, false);
  /* Perform 50 measurements and compute the averages */
  sum_ranging = 0;
  total_count = 0;
  measurementRemaining=50;
  //todo: actually fire off first measurement
  return false;
}

bool NonBlocking::OffsetProcess::onMeasurement(const RangingMeasurementData_t &RangingMeasurementData) {
  if (RangingMeasurementData.rangeError == VL53L0X::Range_Valid) {
    sum_ranging += RangingMeasurementData.RangeMilliMeter;
    ++total_count;
  } else {
    //todo: how to handle a range Error? until we try again:
    dev.logError(VL53L0X::ERROR_RANGE_ERROR, __FUNCTION__);
    return false;
  }

  /* no valid values found */
  if (total_count == 0) {
    dev.logError(VL53L0X::ERROR_RANGE_ERROR, __FUNCTION__ );
    return false;
  }
  if (--measurementRemaining > 0) {
    return true;
  }

  /* FixPoint1616_t / uint16_t = FixPoint1616_t */
  FixPoint1616_t StoredMeanRange(sum_ranging, total_count);
  int32_t StoredMeanRangeAsInt = StoredMeanRange.rounded();

  /* Round Cal Distance to Whole Number.
   * Note that the cal distance is in mm, therefore no resolution is lost.
   * 980f: but why not retain the fractional part for tracability of the value? Rounding to resolution is unnecessary */
  int32_t CalDistanceAsInt_mm = CalDistanceMilliMeter.rounded();
  int32_t OffsetMicroMeter = (CalDistanceAsInt_mm - StoredMeanRangeAsInt) * 1000;

  /* Apply the calculated offset */
  dev.Data.CurrentParameters.RangeOffsetMicroMeters = OffsetMicroMeter;//record
  auto ok= dev.SetOffsetCalibrationDataMicroMeter(OffsetMicroMeter);//send to device
  if(!ok) {
    dev.logError(ok ,"Offset Measurement failed on send to device");
  } else {
    dev.logError(VL53L0X::ERROR_NONE,__FUNCTION__ );
  }
  /* Restore the TCC */
  if (SequenceStepWasEnabled) {
    return dev.SetSequenceStepEnable(SEQUENCESTEP_TCC, true);
  }
  return false;//done
}
