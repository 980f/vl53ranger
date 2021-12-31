/*******************************************************************************
Copyright 2021 by Andy Heilveil github/980f via extensive rewrite of stuff originally
 *  Copyright 2016, STMicroelectronics International N.V.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 * Neither the name of STMicroelectronics nor the
 *  names of its contributors may be used to endorse or promote products
 *  derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 *  NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
 *  IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

//bug: this is a platform choice, not an api choice. #define USE_I2C_2V8
//if the above is essential as in devices don't actually work at the lower voltage than that needs to be well documented and the option removed!

#include "bitmanipulators.h"
#include "vl53l0x_api.h"
#include "vl53l0x_api_core.h"
#include "vl53l0x_api_strings.h"  //should get rid of wrapping, only one ever had a concept of error and for that we can return a nullptr.
#include "vl53l0x_interrupt_threshold_settings.h" //some tuning parameters
#include "vl53l0x_tuning.h"  //tuning values packed into an array of bytes

#include "log_api.h"

#include "versioninfo.h"
#define  VL53L0X_NYI(fakeout)   LOG_ERROR(ERROR_NOT_IMPLEMENTED); return fakeout;

#ifdef VL53L0X_LOG_ENABLE
#define trace_print(level, ...)   trace_print_module_function(TRACE_MODULE_API, level, TRACE_FUNCTION_NONE, ## __VA_ARGS__)
#endif

namespace VL53L0X {

  const Version_t ImplementationVersion  {{VL53L0X_IMPLEMENTATION_VER_MAJOR, VL53L0X_IMPLEMENTATION_VER_MINOR}, VL53L0X_IMPLEMENTATION_VER_SUB,VL53L0X_IMPLEMENTATION_VER_REVISION};

  const Version_t PalSpecVersion { {VL53L0X_SPECIFICATION_VER_MAJOR, VL53L0X_SPECIFICATION_VER_MINOR}, VL53L0X_SPECIFICATION_VER_SUB,VL53L0X_SPECIFICATION_VER_REVISION};

  /* Group PAL General Functions */
  bool Api::measurement_poll_for_completion() {
    LOG_FUNCTION_START;
    for (unsigned LoopNb = VL53L0X_DEFAULT_MAX_LOOP; LoopNb-- > 0;) {
      auto ready = GetMeasurementDataReady();
      if (ready) {
        return true;
      }
      PollingDelay();
    }
    return LOG_ERROR(ERROR_TIME_OUT);
  } // VL53L0X_measurement_poll_for_completion


  bool Api::check_part_used(uint8_t &Revision, DeviceInfo_t &pDeviceInfo) {
    LOG_FUNCTION_START;

    if (get_info_from_device(IDStuff)) {
      if (VL53L0X_GETDEVICESPECIFICPARAMETER(ModuleId) == 0) {
        Revision = 0;
        COPYSTRING(pDeviceInfo.ProductId, "");
      } else {
        Revision = VL53L0X_GETDEVICESPECIFICPARAMETER(Revision);
        COPYSTRING(pDeviceInfo.ProductId, VL53L0X_GETDEVICESPECIFICPARAMETER(ProductId));
      }
      return true;
    }
    return false;
  } // check_part_used

  bool Api::get_device_info(DeviceInfo_t &pDeviceInfo) {
    uint8_t Revision;
    if (check_part_used(Revision, pDeviceInfo)) {
      if (Revision == 0) {
        COPYSTRING(pDeviceInfo.Name, VL53L0X_STRING_DEVICE_INFO_NAME_TS0);
      } else if ((Revision <= 34) && (Revision != 32)) {
        COPYSTRING(pDeviceInfo.Name, VL53L0X_STRING_DEVICE_INFO_NAME_TS1);
      } else if (Revision < 39) {
        COPYSTRING(pDeviceInfo.Name, VL53L0X_STRING_DEVICE_INFO_NAME_TS2);
      } else {
        COPYSTRING(pDeviceInfo.Name, VL53L0X_STRING_DEVICE_INFO_NAME_ES1);
      }
      COPYSTRING(pDeviceInfo.Type, VL53L0X_STRING_DEVICE_INFO_TYPE);

      comm.RdByte(REG_IDENTIFICATION_MODEL_ID, &pDeviceInfo.ProductType);

      //DUP: duplicate code.
      pDeviceInfo.ProductRevision = GetProductRevision();
      return true;
    } else {
      return false;
    }
  } // get_device_info

  SemverLite Core::GetProductRevision() {
    LOG_FUNCTION_START;
    uint8_t revision_id;
    fetch(revision_id, REG_IDENTIFICATION_REVISION_ID);
    return {1, static_cast<uint8_t>(revision_id >> 4)};
  } // GetProductRevision

  bool Api::GetDeviceInfo(DeviceInfo_t &pVL53L0X_DeviceInfo) {
    LOG_FUNCTION_START;
    return get_device_info(pVL53L0X_DeviceInfo);
  }

  DeviceError Api::GetDeviceErrorStatus() {
    LOG_FUNCTION_START;
    uint8_t rangeStatus;
    fetch(rangeStatus, REG_RESULT_RANGE_STATUS);
    return static_cast<DeviceError>(getBits<6, 3>(rangeStatus));
  } // GetDeviceErrorStatus

  const char *Api::GetDeviceErrorString(DeviceError ErrorCode) {
    return device_error_string(ErrorCode);
  }

  const char *Api::GetRangeStatusString(RangeStatus rangeStatus) {
    return range_status_string(rangeStatus);
  }

  const char *Api::GetPalErrorString(Error PalErrorCode) {
    return pal_error_string(PalErrorCode);
  }

  const char *Api::GetPalStateString(State PalStateCode) {
    return pal_state_string(PalStateCode);
  }

  State Api::GetPalState() {
    return PALDevDataGet(PalState);
  }

  bool Api::SetPowerMode(PowerModes PowerMode) {
    LOG_FUNCTION_START;

    if (PowerMode == POWERMODE_IDLE_LEVEL1) {
      comm.WrByte(0x80, 0x00);
      if (StaticInit()) {
        PALDevDataSet(PowerMode, POWERMODE_IDLE_LEVEL1);
        return true;
      } else {
        return false;
      }
    } else {/* set the standby level1 of power mode */
      if(PowerMode!=POWERMODE_STANDBY_LEVEL1){
        return false;// or perhaps throw
      }
      comm.WrByte(0x80, 0x00);
      PALDevDataSet(PalState, STATE_STANDBY);
      PALDevDataSet(PowerMode, POWERMODE_STANDBY_LEVEL1);
      return true;
    }
  } // VL53L0X_SetPowerMode

  PowerModes Api::GetPowerMode() {
    LOG_FUNCTION_START;
    uint8_t Byte;
    fetch(Byte, Private_PowerMode);
    /* Only level1 of Power mode exists */
    auto encoded = (Byte == 1) ? POWERMODE_IDLE_LEVEL1 : POWERMODE_STANDBY_LEVEL1;
    PALDevDataSet(PowerMode, encoded);
    return encoded;
  } // GetPowerMode

  void Api::SetOffsetCalibrationDataMicroMeter(int32_t OffsetCalibrationDataMicroMeter) {
    LOG_FUNCTION_START;
    set_offset_calibration_data_micro_meter(OffsetCalibrationDataMicroMeter);
  }

  int32_t Api::GetOffsetCalibrationDataMicroMeter() {
    LOG_FUNCTION_START;
    return get_offset_calibration_data_micro_meter();
  }

  bool Api::SetLinearityCorrectiveGain(uint16_t LinearityCorrectiveGain) {
    LOG_FUNCTION_START;
    if (!validGain(LinearityCorrectiveGain)) {
      LOG_ERROR(ERROR_INVALID_PARAMS);
      return false;
    }
    PALDevDataSet(LinearityCorrectiveGain, LinearityCorrectiveGain);

    if (!Data.unityGain()) { /* Disable FW Xtalk */
      //ick: there seems to have been an enable bit added after this code was written
      comm.WrWord(REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, 0);
    }

    return true;
  } // VL53L0X_SetLinearityCorrectiveGain

  uint16_t Api::GetLinearityCorrectiveGain() {
    return PALDevDataGet(LinearityCorrectiveGain);
  }

  MegaCps Api::GetTotalSignalRate() {
    LOG_FUNCTION_START;
    return get_total_signal_rate(PALDevDataGet(LastRangeMeasure));
  } // GetTotalSignalRate

/* End Group PAL General Functions */

/* Group PAL Init Functions */
  bool Api::SetDeviceAddress(uint8_t _8bitAddress) {
    LOG_FUNCTION_START;
    comm.WrByte(REG_I2C_SLAVE_DEVICE_ADDRESS, _8bitAddress >> 1);
    comm.wirer.devAddr = _8bitAddress; // 7 bit addr
    return true;
  }

  void Api::DataInit() {
    LOG_FUNCTION_START;

    /* by default the I2C is running at 1V8 if you want to change it you need to include this define at compilation level. */
#ifdef USE_I2C_2V8
    comm.UpdateBit(REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, 0, true);//ick: former code had stupid value (clearing and setting the same bit) that worked only because of a fine hidden detail in UpdateByte
#endif

    /* Set I2C standard mode */
    comm.WrByte(0x88, 0x00);

    /* read WHO_AM_I */
    uint8_t b;
    comm.RdByte(0xC0, &b);
    // Serial.print("WHOAMI: 0x"); Serial.println(b, HEX);

    /* read WHO_AM_I */
    VL53L0X_SETDEVICESPECIFICPARAMETER(ReadDataFromDeviceDone, 0);//forget we have ever read anything

#ifdef USE_IQC_STATION
    bool faild=apply_offset_adjustment();
    //but do we continue or not?
#endif

    /* Default value is 1000 for Linearity Corrective Gain */
    PALDevDataSet(LinearityCorrectiveGain, 1000);

    /* Dmax default Parameter */
    PALDevDataSet(dmaxCal.RangeMilliMeter, 400);
    PALDevDataSet(dmaxCal.SignalRateRtnMegaCps, 1.42f);//(FixPoint1616_t) ((0x00016B85)));             /* 1.42 No Cover Glass*/

    /* Set Default static parameters
     * set first temporary values 9.44MHz * 65536 = 618660 */
    VL53L0X_SETDEVICESPECIFICPARAMETER(OscFrequencyMHz, 9.44f);//618660);

    /* Set Default XTalkCompensationRateMegaCps to 0  */
    VL53L0X_SETPARAMETERFIELD(XTalkCompensationRateMegaCps, 0.0f);

    /* Get default parameters */
    auto CurrentParameters = GetDeviceParameters();

    //failure no longer flows here:
//      /* initialize PAL values */
//      CurrentParameters.DeviceMode = DEVICEMODE_SINGLE_RANGING;
//      CurrentParameters.HistogramMode = HISTOGRAMMODE_DISABLED;
//      PALDevDataSet(CurrentParameters, CurrentParameters);
//

    /* Sigma estimator variable */
    PALDevDataSet(SigmaEst.RefArray, 100);
    PALDevDataSet(SigmaEst.EffPulseWidth, 900);
    PALDevDataSet(SigmaEst.EffAmbWidth, 500);
    PALDevDataSet(targetRefRate, 20.0f); /* 20 MCPS in 9:7 format was 0x0A00 */

    /* Use internal default settings */
    PALDevDataSet(UseInternalTuningSettings, true);

    {
      auto magic = magicWrapper();
      comm.RdByte(REG_SYSRANGE_stopper, &PALDevDataGet(StopVariable));
    }
    /* Enable all check */
    for (unsigned ci = 0; ci < CHECKENABLE_NUMBER_OF_CHECKS; ++ci) {
      SetLimitCheckEnable(static_cast<CheckEnable>(ci), true);//doh more than half are then cleared by statements outside the loop!
    }

    /* Disable the following checks */
    SetLimitCheckEnable(CHECKENABLE_SIGNAL_REF_CLIP, false);
    SetLimitCheckValue(CHECKENABLE_SIGNAL_REF_CLIP, 35.0F);

    SetLimitCheckEnable(CHECKENABLE_RANGE_IGNORE_THRESHOLD, false);
    SetLimitCheckValue(CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);

    SetLimitCheckEnable(CHECKENABLE_SIGNAL_RATE_MSRC, false);
    SetLimitCheckEnable(CHECKENABLE_SIGNAL_RATE_PRE_RANGE, false);

    /* Limit default values */
    SetLimitCheckValue(CHECKENABLE_SIGMA_FINAL_RANGE, 18.0F);
    SetLimitCheckValue(CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 0.25F);
    set_SequenceConfig(0xFF, true);
    /* Set PAL state to tell that we are waiting for call to StaticInit */
    PALDevDataSet(PalState, STATE_WAIT_STATICINIT);

    VL53L0X_SETDEVICESPECIFICPARAMETER(RefSpadsInitialised, false);
  } // VL53L0X_DataInit

  bool Api::SetTuningSettingBuffer(Tunings pTuningSettingBuffer, bool UseInternalTuningSettings) {
    LOG_FUNCTION_START;
    if (UseInternalTuningSettings) { /* Force use internal settings */
      PALDevDataSet(UseInternalTuningSettings, true);
    } else {   /* check that the first byte is not 0 */
      if (*pTuningSettingBuffer != 0) {
        PALDevDataSet(pTuningSettingsPointer, pTuningSettingBuffer);
        PALDevDataSet(UseInternalTuningSettings, false);
      } else {
        return LOG_ERROR(ERROR_INVALID_PARAMS);
      }
    }
    return true;
  } // VL53L0X_SetTuningSettingBuffer

  Tunings Api::GetTuningSettingBuffer(bool theInternal) {
    if (theInternal || PALDevDataGet(UseInternalTuningSettings)) {
      return DefaultTuningSettings;
    } else {
      return PALDevDataGet(pTuningSettingsPointer);
    }
  } // GetTuningSettingBuffer


  void Api::initRanger(VcselPeriod periodType, SequenceStepId stepId, DeviceSpecificParameters_t::RangeSetting &ranger) {
    ranger.VcselPulsePeriod = GetVcselPulsePeriod(periodType);
    ranger.TimeoutMicroSecs = GetSequenceStepTimeout(stepId);
  }

  bool Api::StaticInit() {
    LOG_FUNCTION_START;

    if (!get_info_from_device(InfoGroup::SpadStuff)) {
      return false;
    }

    /* set the ref spad from NVM */
    SpadCount ref = VL53L0X_GETDEVICESPECIFICPARAMETER(ReferenceSpad);

    /* check NVM value: two known types, 1 and 0, '1' has a max 32 spads, '0' has a max of 12 */
    if (ref.quantity > (ref.isAperture ? 32 : 12)) {//if true then invalid settings so compute correct ones
      if (!perform_ref_spad_management()) {
        return false;
      }
    } else {
      if (!set_reference_spads(ref)) {
        return false;
      }
    }

    if (!load_tuning_settings(GetTuningSettingBuffer())) {
      return false;
    }
    /* Set interrupt config to new sample ready */
    if (!SetGpioConfig(0, {DEVICEMODE_SINGLE_RANGING, GPIOFUNCTIONALITY_NEW_MEASURE_READY, INTERRUPTPOLARITY_LOW})) {
      return false;
    }
    FixPoint<4, 12> fix412 = FFread<FixPoint<4, 12>>(RegSystem(0x84));

    VL53L0X_SETDEVICESPECIFICPARAMETER(OscFrequencyMHz, fix412);//conversion from 412 to 1616 is inferred by compiler


    /* After static init, some device parameters may be changed, so update them */
    auto CurrentParameters = GetDeviceParameters();

    auto fe = GetFractionEnable();
    PALDevDataSet(RangeFractionalEnable, fe);
    PALDevDataSet(CurrentParameters, CurrentParameters);

    /* read the sequence config and save it */
    PALDevDataSet(SequenceConfig, get_SequenceConfig());

    /* Disable MSRC and TCC by default */
    SetSequenceStepEnable(SEQUENCESTEP_TCC, false);
    SetSequenceStepEnable(SEQUENCESTEP_MSRC, false);
    /* Set PAL State to standby */
    PALDevDataSet(PalState, STATE_IDLE);
    initRanger(VCSEL_PERIOD_PRE_RANGE, SEQUENCESTEP_PRE_RANGE, Data.DeviceSpecificParameters.PreRange);
    initRanger(VCSEL_PERIOD_FINAL_RANGE, SEQUENCESTEP_FINAL_RANGE, Data.DeviceSpecificParameters.FinalRange);
    return true;
  }

  bool Api::waitOnResetIndicator(bool disappear) {
    for (uint8_t Byte = disappear ? ~0 : 0; disappear == (Byte != 0);) {//BUG: potentially infinite loop
      comm.RdByte(REG_IDENTIFICATION_MODEL_ID, &Byte);
    }
    return true;
  }

  bool Api::ResetDevice() {
    LOG_FUNCTION_START;
//todo: error handling here is about as worthless as elsewhere
    /* Set reset bit */
    comm.WrByte(REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x00);
    if (!waitOnResetIndicator(true)) {
      return false;
    }
    /* Release reset */
    comm.WrByte(REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x01);//ick: ignores error

    /* Wait until correct boot-up of the device */
    if (!waitOnResetIndicator(false)) {//ick: former code could hang on comm error, this quits
      return false;
    }

    /* Set PAL State to State_POWERDOWN */

    PALDevDataSet(PalState, STATE_POWERDOWN);

    return true;
  }
  // VL53L0X_ResetDevice

/* End Group PAL Init Functions */

/* Group PAL Parameters Functions */
  void Api::SetDeviceParameters(const DeviceParameters_t &pDeviceParameters) {
    LOG_FUNCTION_START;
    SetDeviceMode(pDeviceParameters.DeviceMode);
    SetInterMeasurementPeriodMilliSeconds(pDeviceParameters.InterMeasurementPeriodMilliSeconds);
    SetXTalkCompensationRateMegaCps(pDeviceParameters.XTalkCompensationRateMegaCps);
    SetOffsetCalibrationDataMicroMeter(pDeviceParameters.RangeOffsetMicroMeters);

    for (unsigned i = 0; i < CHECKENABLE_NUMBER_OF_CHECKS; i++) {
      SetLimitCheckEnable(static_cast<CheckEnable>(i), pDeviceParameters.LimitChecksEnable[i]);
      SetLimitCheckValue(static_cast<CheckEnable>(i), pDeviceParameters.LimitChecksValue[i]);
    }
    SetWrapAroundCheckEnable(pDeviceParameters.WrapAroundCheckEnable);
    SetMeasurementTimingBudgetMicroSeconds(pDeviceParameters.MeasurementTimingBudgetMicroSeconds);
  } // VL53L0X_SetDeviceParameters


  bool Api::SetDeviceMode(DeviceModes deviceMode) {
    LOG_FUNCTION_START;
    Error(" %d", deviceMode);//todo: add string

    switch (deviceMode) {
      case DEVICEMODE_SINGLE_RANGING:
      case DEVICEMODE_CONTINUOUS_RANGING:
      case DEVICEMODE_CONTINUOUS_TIMED_RANGING:
      case DEVICEMODE_GPIO_DRIVE:
      case DEVICEMODE_GPIO_OSC:
        /* Supported modes */
        VL53L0X_SETPARAMETERFIELD(DeviceMode, deviceMode);
        return true;
      default:
        /* Unsupported mode */
        return LOG_ERROR(ERROR_MODE_NOT_SUPPORTED);
    } // switch

  } // VL53L0X_SetDeviceMode

  DeviceModes Api::GetDeviceMode() {
    return Data.CurrentParameters.DeviceMode;
  }

  void Api::SetRangeFractionEnable(bool Enable) {
    LOG_FUNCTION_START;
    Error(" %u", Enable);
    comm.WrByte(REG_SYSTEM_RANGE_CONFIG, Enable);
    PALDevDataSet(RangeFractionalEnable, Enable);
  } // VL53L0X_SetRangeFractionEnable

  bool Api::GetFractionEnable() {
    LOG_FUNCTION_START;
    bool enabled;
    fetch(enabled, REG_SYSTEM_RANGE_CONFIG);
    return getBit<0>(enabled);
  } // GetFractionEnable


  uint32_t Api::GetMeasurementTimingBudgetMicroSeconds() {
    return get_measurement_timing_budget_micro_seconds();
  }

  bool Api::SetVcselPulsePeriod(VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriodPCLK) {
    //todo:0 depending upon success of following call then call perform_phase_calibration
    return set_vcsel_pulse_period(VcselPeriodType, VCSELPulsePeriodPCLK);
  }

  uint8_t Api::GetVcselPulsePeriod(VcselPeriod VcselPeriodType) {
    LOG_FUNCTION_START;
    return get_vcsel_pulse_period(VcselPeriodType);
  }

  void Api::SetSequenceStepEnable(SequenceStepId SequenceStepId, bool SequenceStepEnabled) {
    LOG_FUNCTION_START;
    unsigned bitnum = bitFor(SequenceStepId);
    if (bitnum == ~0) {
      THROW(ERROR_INVALID_PARAMS);//bypassed enum
    }
    auto SequenceConfig = get_SequenceConfig();
    uint8_t SequenceConfigNew = SequenceConfig;
    setBit(bitnum, SequenceConfigNew, SequenceStepEnabled);
    if (SequenceStepId == SEQUENCESTEP_DSS) {//legacy weirdness, set bit 5 in tandem wtih bit for DSS
      setBit<5>(SequenceConfigNew, SequenceStepEnabled);
    }

    if (SequenceConfigNew != SequenceConfig) {
      /* Apply New Setting */
      comm.WrByte(REG_SYSTEM_SEQUENCE_CONFIG, SequenceConfigNew);
      PALDevDataSet(SequenceConfig, SequenceConfigNew);
      /* Recalculate timing budget */
      PALDevDataSet(SequenceConfig, SequenceConfigNew);
      uint32_t MeasurementTimingBudgetMicroSeconds = VL53L0X_GETPARAMETERFIELD(MeasurementTimingBudgetMicroSeconds);
      SetMeasurementTimingBudgetMicroSeconds(MeasurementTimingBudgetMicroSeconds);
      //todo: check above. IDE scrambled.
    }
  } // VL53L0X_SetSequenceStepEnable

  SequenceStepId Api::GetNumberOfSequenceSteps() {
    return SEQUENCESTEP_NUMBER_OF_CHECKS;
  }

  const char *Api::GetSequenceStepsInfo(SequenceStepId SequenceStepId) {
    return sequence_steps_info(SequenceStepId);
  } // GetSequenceStepsInfo

  bool Api::SetSequenceStepTimeout(SequenceStepId SequenceStepId, FixPoint1616_t TimeOutMilliSecs) {
    LOG_FUNCTION_START;
    /* Read back the current value in case we need to revert back to this.  */
    auto OldTimeOutMicroSeconds = get_sequence_step_timeout(SequenceStepId);

    //a milli of a milli is a micro
    set_sequence_step_timeout(SequenceStepId, TimeOutMilliSecs.millis());
    uint32_t MeasurementTimingBudgetMicroSeconds = VL53L0X_GETPARAMETERFIELD(MeasurementTimingBudgetMicroSeconds);
    /* At this point we don't know if the requested value is valid,
       *  therefore proceed to update the entire timing budget and if this fails, revert back to the previous value.  */
    if (!SetMeasurementTimingBudgetMicroSeconds(MeasurementTimingBudgetMicroSeconds)) {
      //budget failed to restore prior value, and have to rerun budget because of its potential side effects
      set_sequence_step_timeout(SequenceStepId, OldTimeOutMicroSeconds);
      SetMeasurementTimingBudgetMicroSeconds(MeasurementTimingBudgetMicroSeconds);
      return false;
    }
    return true;
  } // VL53L0X_SetSequenceStepTimeout

  FixPoint1616_t Api::GetSequenceStepTimeout(SequenceStepId SequenceStepId) {
    LOG_FUNCTION_START;
    auto TimeoutMicroSeconds = get_sequence_step_timeout(SequenceStepId);
    return {TimeoutMicroSeconds, 1000};
  } // GetSequenceStepTimeout

  void Api::SetInterMeasurementPeriodMilliSeconds(unsigned int InterMeasurementPeriodMilliSeconds) {
    LOG_FUNCTION_START;
    uint16_t osc_calibrate_val;
    comm.RdWord(REG_OSC_CALIBRATE_VAL, &osc_calibrate_val);

    uint32_t IMPeriodMilliSeconds = (osc_calibrate_val != 0) ? InterMeasurementPeriodMilliSeconds * osc_calibrate_val : InterMeasurementPeriodMilliSeconds;
    comm.WrDWord(REG_SYSTEM_INTERMEASUREMENT_PERIOD, IMPeriodMilliSeconds);
  } // VL53L0X_SetInterMeasurementPeriodMilliSeconds

  uint32_t Api::GetInterMeasurementPeriodMilliSeconds() {
    LOG_FUNCTION_START;
    uint16_t osc_calibrate_val;
    fetch(osc_calibrate_val, REG_OSC_CALIBRATE_VAL);
    uint32_t IMPeriodMilliSeconds;
    fetch(IMPeriodMilliSeconds, REG_SYSTEM_INTERMEASUREMENT_PERIOD);
    auto ratio = roundedDivide(IMPeriodMilliSeconds, osc_calibrate_val);
    if (ratio != 0) {//presuming due to div by zero
      VL53L0X_SETPARAMETERFIELD(InterMeasurementPeriodMilliSeconds, ratio);
    }
    return ratio;
  } // GetInterMeasurementPeriodMilliSeconds



  void Api::SetRefCalibration(CalibrationParameters p) {
    LOG_FUNCTION_START;
    set_ref_calibration(p, true, true);
  }

  Api::CalibrationParameters Api::GetRefCalibration() {
    LOG_FUNCTION_START;
    return get_ref_calibration();
  }

/*
 * CHECK LIMIT FUNCTIONS
 */

  CheckEnable Api::GetNumberOfLimitCheck() {
    return CHECKENABLE_NUMBER_OF_CHECKS;
  }

  const char *Api::GetLimitCheckInfo(CheckEnable LimitCheckId) {
    return limit_check_info(LimitCheckId);
  }

  bool Api::GetLimitCheckStatus(CheckEnable LimitCheckId) {
    if (LimitCheckId >= CHECKENABLE_NUMBER_OF_CHECKS) {
      THROW(ERROR_INVALID_PARAMS);//bypassed enum
    }
    return Data.CurrentParameters.LimitChecksStatus[LimitCheckId];
  } // GetLimitCheckStatus

  bool Api::SetLimitCheckEnable(CheckEnable LimitCheckId, bool LimitCheckEnable) {
    if (LimitCheckId >= CHECKENABLE_NUMBER_OF_CHECKS) {
      THROW(ERROR_INVALID_PARAMS);//bypassed enum
    }

    LOG_FUNCTION_START;
    LimitCheckEnable = LimitCheckEnable != 0;//in case someone got around compiler checking
    bool LimitCheckDisable = !LimitCheckEnable;

    FixPoint<9, 7> TempFix1616 = LimitCheckEnable ? VL53L0X_GETARRAYPARAMETERFIELD(LimitChecksValue, LimitCheckId) : FixPoint<9, 7>(0);

    switch (LimitCheckId) {
      case CHECKENABLE_SIGMA_FINAL_RANGE:
      case CHECKENABLE_SIGNAL_REF_CLIP:
      case CHECKENABLE_RANGE_IGNORE_THRESHOLD:
        /* internal computation: */
        VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksEnable, LimitCheckId, LimitCheckEnable);
        break;

      case CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
        comm.WrWord(REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, TempFix1616);
        break;

      case CHECKENABLE_SIGNAL_RATE_MSRC:
        comm.UpdateByte(REG_MSRC_CONFIG_CONTROL, ~Bitter(0), LimitCheckDisable ? Bitter(1) : 0);//BUG:clear lsb set bit 1
        break;

      case CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
        comm.UpdateBit(REG_MSRC_CONFIG_CONTROL, 4, LimitCheckDisable);
        break;

      default:
        THROW(ERROR_INVALID_PARAMS);//bypassed enum
    } // switch

    VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksEnable, LimitCheckId, LimitCheckEnable);
    return true;
  } // VL53L0X_SetLimitCheckEnable


  void Api::SetLimitCheckValue(CheckEnable LimitCheckId, FixPoint<9, 7> LimitCheckValue) {
    LOG_FUNCTION_START;
    if (VL53L0X_GETARRAYPARAMETERFIELD(LimitChecksEnable, LimitCheckId)) {
      switch (LimitCheckId) {
        case CHECKENABLE_SIGMA_FINAL_RANGE:
        case CHECKENABLE_SIGNAL_REF_CLIP:
        case CHECKENABLE_RANGE_IGNORE_THRESHOLD:
          //these three don't go to the device
          break;
        case CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
          comm.WrWord(REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, LimitCheckValue);
          break;
        case CHECKENABLE_SIGNAL_RATE_MSRC:
        case CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
          comm.WrWord(REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT, LimitCheckValue);
          break;
        default:
          THROW (ERROR_INVALID_PARAMS);//bypassed enum
      } // switch
    }
    VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksValue, LimitCheckId, LimitCheckValue);
  } // VL53L0X_SetLimitCheckValue


  void Api::SetLimitCheck(CheckEnable LimitCheckId, Api::LimitTuple limit) {
    SetLimitCheckEnable(LimitCheckId, limit.enable);
    SetLimitCheckValue(LimitCheckId, limit.value);
  }

  MegaCps Api::GetLimitCheckCurrent(CheckEnable LimitCheckId) {
    if (LimitCheckId >= CHECKENABLE_NUMBER_OF_CHECKS) {
      THROW(ERROR_INVALID_PARAMS);//bypassed enum
    }
    switch (LimitCheckId) {
      case CHECKENABLE_SIGMA_FINAL_RANGE:
        /* Need to have run a ranging to have the latest values */
        return PALDevDataGet(SigmaEstimate);

      case CHECKENABLE_SIGNAL_REF_CLIP:
        /* Need to have run a ranging to have the latest values */
        return PALDevDataGet(LastSignalRefMcps);

      case CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
      case CHECKENABLE_RANGE_IGNORE_THRESHOLD:
      case CHECKENABLE_SIGNAL_RATE_MSRC:
      case CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
        /* Need to run a ranging to have the latest values */;
        return PALDevDataGet(LastRangeMeasure).SignalRateRtnMegaCps;

      default:
        THROW(ERROR_INVALID_PARAMS);
        return 0;//dummy to appease compiler
    } // switch

  } // GetLimitCheckCurrent

/*
 * WRAPAROUND Check
 */
  void Api::SetWrapAroundCheckEnable(bool WrapAroundCheckEnable) {
    LOG_FUNCTION_START;
    bool WrapAroundCheckEnableInt = WrapAroundCheckEnable != 0;//in case someone somehow circumvented compiler checks
    comm.UpdateBit(REG_SYSTEM_SEQUENCE_CONFIG, 7, WrapAroundCheckEnableInt);
    setBit<7>(Data.SequenceConfig, WrapAroundCheckEnableInt);
    VL53L0X_SETPARAMETERFIELD(WrapAroundCheckEnable, WrapAroundCheckEnableInt);
  } // VL53L0X_SetWrapAroundCheckEnable

  bool Api::GetWrapAroundCheckEnable() {
    LOG_FUNCTION_START;
    uint8_t seqconf = get_SequenceConfig();
    PALDevDataSet(SequenceConfig, seqconf);
    VL53L0X_SETPARAMETERFIELD(WrapAroundCheckEnable, getBit<7>(seqconf));
    return VL53L0X_GETPARAMETERFIELD(WrapAroundCheckEnable);
  } // GetWrapAroundCheckEnable

  bool Api::SetDmaxCalParameters(const DevData_t::DmaxCal &p) {
    LOG_FUNCTION_START;

    /* Check if one of input parameter is zero, in that case the value are get from NVM */
    if ((p.RangeMilliMeter == 0) || (p.SignalRateRtnMegaCps.raw == 0)) {
      /* NVM parameters */
      /* Run Get_info_from_device to get signal rate at 400 mm.
       * if the value have been already fetched this function will return with no access to device */
      if (get_info_from_device(PartUidEtc)) {
        PALDevDataSet(dmaxCal.RangeMilliMeter, 400);
        PALDevDataSet(dmaxCal.SignalRateRtnMegaCps, VL53L0X_GETDEVICESPECIFICPARAMETER(SignalRateMeasFixed400mm));
        return true;
      }
      return false;
    } else {
      /* User parameters */
      PALDevDataSet(dmaxCal.RangeMilliMeter, p.RangeMilliMeter);
      PALDevDataSet(dmaxCal.SignalRateRtnMegaCps, p.SignalRateRtnMegaCps);
      return true;
    }
  } // VL53L0X_SetDmaxCalParameters

  DevData_t::DmaxCal Api::GetDmaxCalParameters() {
    return PALDevDataGet(dmaxCal);
  } // GetDmaxCalParameters

/* End Group PAL Parameters Functions */

/* Group PAL Measurement Functions */
  bool Api::PerformSingleMeasurement() {
    LOG_FUNCTION_START;

    /* Get Current DeviceMode */
    auto DeviceMode = GetDeviceMode();
    /* Start immediately to run a single ranging measurement in case of
     * single ranging or single histogram */
    if (DeviceMode == DEVICEMODE_SINGLE_RANGING) {
      if (!StartMeasurement()) {
        return false;
      }
    }
//but if not in single then is continuous and one will have started and will complete on its own:
    auto completed = measurement_poll_for_completion();
    if (completed) {
      if (DeviceMode == DEVICEMODE_SINGLE_RANGING) {
        PALDevDataSet(PalState, STATE_IDLE);
      }
      return true;
    } else {
      return false;
    }
  } // VL53L0X_PerformSingleMeasurement


  bool Api::PerformRefCalibration() {
    LOG_FUNCTION_START;
    return perform_ref_calibration();
  }

  bool Api::PerformXTalkCalibration(FixPoint1616_t XTalkCalDistance) {
    LOG_FUNCTION_START;
    return perform_xtalk_calibration(XTalkCalDistance);
  }

  bool Api::PerformOffsetCalibration(FixPoint1616_t CalDistanceMilliMeter) {
    LOG_FUNCTION_START;
    return perform_offset_calibration(CalDistanceMilliMeter);
  }

  bool Api::CheckAndLoadInterruptSettings(bool StartNotStopFlag) {
    uint8_t InterruptConfig = VL53L0X_GETDEVICESPECIFICPARAMETER(Pin0GpioFunctionality);

    if ((InterruptConfig == GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW) ||
        (InterruptConfig == GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH) ||
        (InterruptConfig == GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT)) {
      RangeWindow Threshold = GetInterruptThresholds(DEVICEMODE_CONTINUOUS_RANGING);
      FixPoint1616_t BigEough(255.0F);//was 255 * 65536 which is the same as 255.0
      if ((Threshold.Low > BigEough) || (Threshold.High > BigEough)) {
        if (StartNotStopFlag) {
          load_tuning_settings(InterruptThresholdSettings);//todo: check error
        } else {
          comm.WrByte(0xFF, 0x04);
          comm.WrByte(0x70, 0x00);
          comm.WrByte(0xFF, 0x00);
          comm.WrByte(0x80, 0x00);
        }
      }
    }

    return true;
  } // VL53L0X_CheckAndLoadInterruptSettings

  bool Api::StartMeasurement() {
    LOG_FUNCTION_START;

    DeviceModes DeviceMode = GetDeviceMode();
    {
      auto popper = magicWrapper();
      comm.WrByte(REG_SYSRANGE_stopper, PALDevDataGet(StopVariable));
    }
    switch (DeviceMode) {
      case DEVICEMODE_SINGLE_RANGING:
        comm.WrByte(REG_SYSRANGE_START, REG_SYSRANGE_MODE_START_STOP | REG_SYSRANGE_MODE_SINGLESHOT);
        /* Wait until start bit has been cleared */
        for (unsigned LoopNb = VL53L0X_DEFAULT_MAX_LOOP; LoopNb-- > 0;) {
          uint8_t flags;
          fetch(flags, REG_SYSRANGE_START);
          if (getBit<0>(flags) == 0) {
            return true;
          }
        }
        return LOG_ERROR(ERROR_TIME_OUT);
      case DEVICEMODE_CONTINUOUS_RANGING: /* Back-to-back mode */
        /* Check if need to apply interrupt settings */
        if (Api::CheckAndLoadInterruptSettings(true)) {
          comm.WrByte(REG_SYSRANGE_START, REG_SYSRANGE_MODE_BACKTOBACK);
          /* Set PAL State to Running */
          PALDevDataSet(PalState, STATE_RUNNING);
          return true;
        }
        break;
      case DEVICEMODE_CONTINUOUS_TIMED_RANGING:   /* Continuous mode
         * Check if need to apply interrupt settings*/
        if (Api::CheckAndLoadInterruptSettings(true)) {
          comm.WrByte(REG_SYSRANGE_START, REG_SYSRANGE_MODE_TIMED);
          /* Set PAL State to Running */
          PALDevDataSet(PalState, STATE_RUNNING);
          return true;
        }
        break;
      default:
        /* Selected mode not supported */
        return false;
    } // switch

    return false;
  } // VL53L0X_StartMeasurement

  bool Api::StopMeasurement() {
    LOG_FUNCTION_START;

    comm.WrByte(REG_SYSRANGE_START, REG_SYSRANGE_MODE_SINGLESHOT);
    {
      auto popper = MagicDuo(comm);
      comm.WrByte(REG_SYSRANGE_stopper, 0x00);
    }

    /* Set PAL State to Idle */
    PALDevDataSet(PalState, STATE_IDLE);
    /* Check if need to apply interrupt settings */
    return CheckAndLoadInterruptSettings(0);
  } // VL53L0X_StopMeasurement


  bool Api::GetMeasurementDataReady() {
    LOG_FUNCTION_START;
    uint8_t InterruptConfig = VL53L0X_GETDEVICESPECIFICPARAMETER(Pin0GpioFunctionality);

    if (InterruptConfig == GPIOFUNCTIONALITY_NEW_MEASURE_READY) {
      auto mask = GetInterruptMaskStatus();
      return mask == GPIOFUNCTIONALITY_NEW_MEASURE_READY;//todo: see if magic bits should be masked here
    } else {
      uint8_t SysRangeStatusRegister;
      fetch(SysRangeStatusRegister, REG_RESULT_RANGE_STATUS);
      return getBit<0>(SysRangeStatusRegister);
    }
  } // GetMeasurementDataReady


  bool Api::GetRangingMeasurementData(RangingMeasurementData_t &pRangingMeasurementData) {
    LOG_FUNCTION_START;
    /*
     * use multi read even if some registers are not useful, result will
     * be more efficient
     * start reading at 0x14 dec20
     * end reading at 0x21 dec33 total 14 bytes to read  //ick: code says 12, not 14
     */

    uint8_t localBuffer[12];
    comm.ReadMulti(0x14, localBuffer, sizeof (localBuffer));

    pRangingMeasurementData.ZoneId = 0;    /* Only one zone */
    pRangingMeasurementData.TimeStamp = 0; /* Not Implemented */

    /* cut1.1 if SYSTEM__RANGE_CONFIG if 1 range is 2bits fractional
     *(format 11.2) else no fractional
     */

    pRangingMeasurementData.MeasurementTimeUsec = 0;

    /* peak_signal_count_rate_rtn_mcps */
    MegaCps SignalRate = FixPoint<9, 7>(MAKEUINT16(localBuffer[7], localBuffer[6]));
    pRangingMeasurementData.SignalRateRtnMegaCps = SignalRate;

    pRangingMeasurementData.AmbientRateRtnMegaCps = FixPoint<9, 7>(MAKEUINT16(localBuffer[9], localBuffer[8]));

    FixPoint<8, 8> EffectiveSpadRtnCount(MAKEUINT16(localBuffer[3], localBuffer[2]));
    pRangingMeasurementData.EffectiveSpadRtnCount = EffectiveSpadRtnCount;

    auto DeviceRangeStatus = localBuffer[0];

    bool RangeFractionalEnable = PALDevDataGet(RangeFractionalEnable);
    uint16_t LinearityCorrectiveGain = PALDevDataGet(LinearityCorrectiveGain);
    auto RangeMilliMeter = MAKEUINT16(localBuffer[11], localBuffer[10]);
    if (LinearityCorrectiveGain != UnityGain) {
      RangeMilliMeter = roundedDivide(LinearityCorrectiveGain * RangeMilliMeter, UnityGain);

      /* Implement Xtalk */
      bool XTalkCompensationEnable = VL53L0X_GETPARAMETERFIELD(XTalkCompensationEnable);

      if (XTalkCompensationEnable) {
        uint16_t XTalkCompensationRateMegaCps = VL53L0X_GETPARAMETERFIELD(XTalkCompensationRateMegaCps);//BUG: loses integer part, perhaps a 9,7 was intended?
        auto dry = SignalRate.raw - ((XTalkCompensationRateMegaCps * EffectiveSpadRtnCount) >> 8);
        RangeMilliMeter = dry > 0 ? (RangeMilliMeter * SignalRate) / dry : (RangeFractionalEnable ? 8888 : (8888 << 2));//bug: magic value
      }
    }

    if (RangeFractionalEnable) {
      pRangingMeasurementData.RangeMilliMeter = (uint16_t) ((RangeMilliMeter) >> 2);
      pRangingMeasurementData.RangeFractionalPart = RangeMilliMeter << (8 - 2);//ick: former mask was gratuitous along with the cast.
    } else {
      pRangingMeasurementData.RangeMilliMeter = RangeMilliMeter;
      pRangingMeasurementData.RangeFractionalPart = 0;
    }

    /*
     * For a standard definition of rangeError, this should
     * return 0 in case of good result after a ranging
     * The range status depends on the device so call a device
     * specific function to obtain the right Error.
     */
    auto PalRangeStatus = get_pal_range_status(DeviceRangeStatus, SignalRate, EffectiveSpadRtnCount, pRangingMeasurementData);
    pRangingMeasurementData.rangeError = PalRangeStatus;

    /* Copy last read data into Dev buffer */
    PALDevDataGet(LastRangeMeasure) = pRangingMeasurementData;

    return true;
  } // GetRangingMeasurementData

  MegaCps Api::GetMeasurementRefSignal() {
    return PALDevDataGet(LastSignalRefMcps);
  }

  bool Api::PerformSingleRangingMeasurement(RangingMeasurementData_t &pRangingMeasurementData) {
    LOG_FUNCTION_START;
    /* This function will do a complete single ranging
     * Here we fix the mode! */
    SetDeviceMode(DEVICEMODE_SINGLE_RANGING);
    if (PerformSingleMeasurement()) {
      GetRangingMeasurementData(pRangingMeasurementData);
      return ClearInterruptMask(0);
    }
    return false;
  } // VL53L0X_PerformSingleRangingMeasurement

#if HaveRoiZones
  bool Api::SetNumberOfROIZones(uint8_t NumberOfROIZones) {
    if (NumberOfROIZones != 1) {
      return LOG_ERROR(ERROR_INVALID_PARAMS);
    }
    return true;
  } // VL53L0X_SetNumberOfROIZones

  unsigned Api::GetNumberOfROIZones() {
    return 1;
  }

  unsigned Api::GetMaxNumberOfROIZones() {
    return 1;
  }
#endif

/* End Group PAL Measurement Functions */

  bool Api::SetGpioConfig(uint8_t Pin, GpioConfiguration gpioConfig) {
    LOG_FUNCTION_START;

    if (Pin != 0) {
      THROW(ERROR_GPIO_NOT_EXISTING);//bad argument, very unlikely unless built for the wrong device.
    }

    switch (gpioConfig.devMode) {
      case DEVICEMODE_GPIO_DRIVE:
        comm.WrByte(REG_GPIO_HV_MUX_ACTIVE_HIGH, (gpioConfig.polarity == INTERRUPTPOLARITY_LOW) ? Bitter(4) : 1);//ick: elsewhere does an update
        return true;
      case DEVICEMODE_GPIO_OSC://todo: convert into a table for tuning parameters.

        comm.WrByte(0xff, 0x01);
        comm.WrByte(0x00, 0x00);
        comm.WrByte(0xff, 0x00);

        comm.WrByte(0x80, 0x01);
        comm.WrByte(0x85, 0x02);

        comm.WrByte(0xff, 0x04);
        comm.WrByte(0xcd, 0x00);
        comm.WrByte(0xcc, 0x11);

        comm.WrByte(0xff, 0x07);
        comm.WrByte(0xbe, 0x00);

        comm.WrByte(0xff, 0x06);
        comm.WrByte(0xcc, 0x09);

        comm.WrByte(0xff, 0x00);
        comm.WrByte(0xff, 0x01);
        comm.WrByte(0x00, 0x00);
        return true;
      default:
        if (!valid(gpioConfig.function)) {
          return LOG_ERROR(ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED);
        } // switch

        comm.WrByte(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, gpioConfig.function);
        comm.UpdateBit(REG_GPIO_HV_MUX_ACTIVE_HIGH, 4, gpioConfig.polarity == INTERRUPTPOLARITY_HIGH);
        VL53L0X_SETDEVICESPECIFICPARAMETER(Pin0GpioFunctionality, gpioConfig.function);
        return ClearInterruptMask(0);
    }
  } // VL53L0X_SetGpioConfig

  GpioConfiguration Api::GetGpioConfig(uint8_t Pin) {
    if (Pin != 0) {
      THROW(ERROR_GPIO_NOT_EXISTING);//unlikley bad argument
    }

    LOG_FUNCTION_START;
    /* pDeviceMode not managed by Ewok it return the current mode */
    GpioConfiguration gpio;
    gpio.devMode = GetDeviceMode();
    uint8_t gpioConfig;
    fetch(gpioConfig, REG_SYSTEM_INTERRUPT_CONFIG_GPIO);
    gpio.function = static_cast<enum GpioFunctionality>(gpioConfig & Mask<2, 0>::places);

    if (!valid(gpio.function)) {
      THROW(ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED);//todo: actual nvm failure, should force a valid value into it.
    } // switch
    uint8_t polarity;
    fetch(polarity, REG_GPIO_HV_MUX_ACTIVE_HIGH);
    gpio.polarity = getBit<4>(polarity) ? INTERRUPTPOLARITY_LOW : INTERRUPTPOLARITY_HIGH;
    VL53L0X_SETDEVICESPECIFICPARAMETER(Pin0GpioFunctionality, gpio.function);
    return gpio;
  } // GetGpioConfig


  void Api::set_threshold(RegSystem index, FixPoint1616_t ThresholdLow) {
    /* no dependency on DeviceMode for Ewok
   * Need to divide by 2 because the FW will apply a x2*/
    uint16_t Threshold16 = ThresholdLow.scaled(17) & Mask<11, 0>::shifted;
    comm.WrWord(index, Threshold16);
  }

  FixPoint1616_t Api::get_threshold(RegSystem index) {
    uint16_t Threshold12;
    comm.RdWord(REG_SYSTEM_THRESH_LOW, &Threshold12);
    /* Need to multiply by 2 because the FW will apply a x2 */
    return {getBits<11, 0>(Threshold12) << 17, 1};
  }

  void Api::SetInterruptThresholds(DeviceModes DeviceMode, RangeWindow Threshold) {
    LOG_FUNCTION_START;
    /* no dependency on DeviceMode for Ewok
     * Need to divide by 2 because the FW will apply a x2*/
    set_threshold(REG_SYSTEM_THRESH_LOW, Threshold.Low);
    set_threshold(REG_SYSTEM_THRESH_HIGH, Threshold.High);
  } // VL53L0X_SetInterruptThresholds

  Api::RangeWindow Api::GetInterruptThresholds(DeviceModes DeviceMode) {
    LOG_FUNCTION_START;
    /* no dependency on DeviceMode for Ewok */
    RangeWindow pThreshold;
    pThreshold.Low = get_threshold(REG_SYSTEM_THRESH_LOW);
    pThreshold.High = get_threshold(REG_SYSTEM_THRESH_HIGH);
    return pThreshold;
  } // GetInterruptThresholds

  uint8_t Api::GetStopCompletedStatus() {
    LOG_FUNCTION_START;

    uint8_t Byte;
    {//must close before going onto writing 0x91
      auto pager = push(Private_Pager, 1, 0);
      fetch(Byte, Private_04);
    }
    if (Byte == 0) {
      auto magic = magicWrapper();
      comm.WrByte(REG_SYSRANGE_stopper, PALDevDataGet(StopVariable));
    }
    return Byte;
  } // GetStopCompletedStatus

/* Group PAL Interrupt Functions */
  bool Api::ClearInterruptMask(uint32_t InterruptMask) {
    LOG_FUNCTION_START;
    /* clear bit 0 range interrupt, bit 1 error interrupt */
    for (unsigned LoopCount = 3; LoopCount-- > 0;) {
      comm.WrByte(REG_SYSTEM_INTERRUPT_CLEAR, 1);
      comm.WrByte(REG_SYSTEM_INTERRUPT_CLEAR, 0);
      uint8_t Byte;
      comm.RdByte(REG_RESULT_INTERRUPT_STATUS, &Byte);

      if (getBits<2, 0>(Byte) == 0) { //ick: elsewhere bits 4,3 are also looked at.
        return true;
      }
    }
    return false;
  } // ClearInterruptMask

  uint8_t Api::GetInterruptMaskStatus() {
    LOG_FUNCTION_START;
    uint8_t mask;
    fetch(mask, REG_RESULT_INTERRUPT_STATUS);
//todo: this must be duplicated at each caller's site
//    if (getBits<4, 3>(mask)) {//if either bit? what are each of them?
//      return ERROR_RANGE_ERROR;
//    }
//    mask &= Mask<2, 0>::places;
    return mask;
  } // GetInterruptMaskStatus


/* End Group PAL Interrupt Functions */

/* Group SPAD functions */

  void Api::SetSpadAmbientDamperThreshold(uint16_t SpadAmbientDamperThreshold) {
    LOG_FUNCTION_START;
    FFwrap(RegSystem(0x40), SpadAmbientDamperThreshold);
  } // VL53L0X_SetSpadAmbientDamperThreshold

  uint16_t Api::GetSpadAmbientDamperThreshold() {
    LOG_FUNCTION_START;
    return FFread<uint16_t>(RegSystem(0x40));
  } // GetSpadAmbientDamperThreshold

  void Api::SetSpadAmbientDamperFactor(uint16_t SpadAmbientDamperFactor) {
    LOG_FUNCTION_START;
    FFwrap(RegSystem(0x42), uint8_t(SpadAmbientDamperFactor));
  } // VL53L0X_SetSpadAmbientDamperFactor

  uint8_t Api::GetSpadAmbientDamperFactor() {
    LOG_FUNCTION_START;
    return FFread<uint8_t>(RegSystem(0x42));
  } // GetSpadAmbientDamperFactor

/* END Group SPAD functions */

/*****************************************************************************
* Internal functions
*****************************************************************************/
//todo: nonblocking version
  bool Api::perform_ref_spad_management() {

    /*
     * The reference SPAD initialization procedure determines the minimum
     * amount of reference spads to be enables to achieve a target reference
     * signal rate and should be performed once during initialization.
     *
     * Either aperture or non-aperture spads are applied but never both.
     * Firstly non-aperture spads are set, begining with 5 spads, and
     * increased one spad at a time until the closest measurement to the
     * target rate is achieved.
     *
     * If the target rate is exceeded when 5 non-aperture spads are enabled,
     * initialization is performed instead with aperture spads.
     *
     * When setting spads, a 'Good Spad Map' is applied.
     *
     * This procedure operates within a SPAD window of interest of a maximum
     * 44 spads.
     * The start point is currently fixed to 180, which lies towards the end
     * of the non-aperture quadrant and runs in to the adjacent aperture
     * quadrant.
     */

    FixPoint<9, 7> //targetRefRate(20.0F);// = 0x0A00; /* 20 MCPS in 9:7 format */ //ick:why init with specific value that is immediately overwritten
    targetRefRate = PALDevDataGet(targetRefRate);

    /*
     * Initialize Spad arrays.
     * Currently the good spad map is initialised to 'All good'.
     * This is a short term implementation. The good spad map will be
     * provided as an input.
     */
    Data.SpadData.enables.clear();

    {
      SysPopper ffer = push(0xFF, 0x01, 0x00);
      comm.WrByte(REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
      comm.WrByte(REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    }

    comm.WrByte(REG_GLOBAL_CONFIG_REF_EN_START_SELECT, startSelect.absolute());
    comm.WrByte(REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0);

    if (!perform_ref_calibration()) {
      return false;
    }

    /* Enable Minimum NON-APERTURE Spads */
    SpadArray::Index currentSpadIndex = 0;

    SpadCount sc {minimumSpadCount, false};
    auto lastSpadIndex = enable_ref_spads(sc, Data.SpadData.goodones, Data.SpadData.enables, currentSpadIndex);
    if (!lastSpadIndex.isValid()) {
      return false;
    }
    currentSpadIndex = lastSpadIndex;

    auto peakSignalRateRef = perform_ref_signal_measurement(0);
    if (peakSignalRateRef == 0) {//todo: learn if 0 is acceptible here.
      return false;
    }

    if (peakSignalRateRef > targetRefRate) { /* Signal rate measurement too high, switch to APERTURE SPADs */
      Data.SpadData.enables.clear();

      /* Increment to the first APERTURE spad */
      while (currentSpadIndex.isValid() && !(startSelect + currentSpadIndex).is_aperture()) {
        ++currentSpadIndex;
      }

      sc = {minimumSpadCount, true};

      lastSpadIndex = enable_ref_spads(sc, Data.SpadData.goodones, Data.SpadData.enables, currentSpadIndex);

      if (lastSpadIndex.isValid()) {
        currentSpadIndex = lastSpadIndex;
        peakSignalRateRef = perform_ref_signal_measurement(0);
        if (peakSignalRateRef == 0) {
          return false;
        }
        if (peakSignalRateRef > targetRefRate) {
          /* Signal rate still too high after setting the minimum number of APERTURE spads.
           * Can do no more therefore set the min number of aperture spads as the result. */
          sc = {minimumSpadCount, true};//todo: 1 this seems to already be the state of it.
        }
      }
    } else {
//      sc.isAperture = false;//but we ERROR_OUT so why this lingering assign? changed to ref object instead of internal one
      return false;//todo: check original source
    }
    if (peakSignalRateRef < targetRefRate) {
/* At this point, the minimum number of either aperture or non-aperture spads have been set.
 * Proceed to add spads and perform measurements until the target reference is reached. */
//      sc.isAperture = needAptSpads;
      sc.quantity = minimumSpadCount;//perhaps superfluous

      SpadArray lastSpadArray = Data.SpadData.enables;

      uint32_t lastSignalRateDiff = abs(peakSignalRateRef - targetRefRate);
      bool complete = false;
      while (!complete) {
        SpadArray::Index nextGoodSpad = Data.SpadData.goodones.nextSet(currentSpadIndex);
        if (!nextGoodSpad.isValid()) {
          return ERROR_REF_SPAD_INIT;
        }
        ++sc.quantity;
/* Cannot combine Aperture and Non-Aperture spads, so ensure the current spad is of the correct type. */
        if ((startSelect + nextGoodSpad).is_aperture() != sc.isAperture) {
          return ERROR_REF_SPAD_INIT;
        }

        currentSpadIndex = nextGoodSpad;
        Data.SpadData.enables.enable(currentSpadIndex);
        ++currentSpadIndex;//post ++ not coded
/* Proceed to apply the additional spad and perform measurement. */
        set_ref_spad_map(Data.SpadData.enables);
        peakSignalRateRef = perform_ref_signal_measurement(0);
        if (peakSignalRateRef == 0) {
          return false;
        }
        uint32_t signalRateDiff = abs(peakSignalRateRef - targetRefRate);

        if (peakSignalRateRef > targetRefRate) { /* Select the spad map that provides the measurement closest to the target rate, either above or below it. */
          if (signalRateDiff > lastSignalRateDiff) { /* Previous spad map produced a closer measurement, so choose this. */
            set_ref_spad_map(lastSpadArray);
            Data.SpadData.enables = lastSpadArray;
            --sc.quantity;
          }
          complete = true;
        } else { /* Continue to add spads */
          lastSignalRateDiff = signalRateDiff;
          lastSpadArray = Data.SpadData.enables;
        }
      } /* while */
    }

    VL53L0X_SETDEVICESPECIFICPARAMETER(RefSpadsInitialised, true);
    VL53L0X_SETDEVICESPECIFICPARAMETER(ReferenceSpad, sc);

    return true;
  }

  DeviceParameters_t Api::GetDeviceParameters() {
    LOG_FUNCTION_START;
    DeviceParameters_t wad;

    auto DeviceMode = GetDeviceMode();
    if (isValid(DeviceMode)) {
      wad.DeviceMode = DeviceMode;
    } else {
      wad.DeviceMode = DEVICEMODE_SINGLE_RANGING;
      //todo: and send that back to the hardware
    }

    wad.InterMeasurementPeriodMilliSeconds = GetInterMeasurementPeriodMilliSeconds();

    wad.XTalkCompensationEnable = false;//ick: why don't we use the actual enable?
    wad.XTalkCompensationRateMegaCps = GetXTalkCompensationRateMegaCps();

    wad.RangeOffsetMicroMeters = GetOffsetCalibrationDataMicroMeter();

    for (unsigned i = 0; i < CHECKENABLE_NUMBER_OF_CHECKS; ++i) {
      /* get first the values, then the enables.
       * GetLimitCheckValue will modify the enable flags
       */
      wad.LimitChecksValue[i] = GetLimitCheckValue(static_cast<CheckEnable>(i));
      wad.LimitChecksEnable[i] = GetLimitCheckEnable(static_cast<CheckEnable>(i));
    }
    auto wcenable = GetWrapAroundCheckEnable();

    wad.WrapAroundCheckEnable = wcenable;
    wad.MeasurementTimingBudgetMicroSeconds = GetMeasurementTimingBudgetMicroSeconds();
    return wad;
  }

  bool Api::PerformRefSpadManagement() {
    LOG_FUNCTION_START;
    return perform_ref_spad_management();
  }

  void Api::SetReferenceSpads(SpadCount spad) {
    VL53L0X_SETDEVICESPECIFICPARAMETER(ReferenceSpad,spad);
  }

#if IncludeNotimplemented
  bool Api::SetGroupParamHold(uint8_t GroupParamHold) {
    VL53L0X_NYI(false);
  }

  uint16_t Api::GetUpperLimitMilliMeter() {
    VL53L0X_NYI(~0);
  }

  bool Api::WaitDeviceBooted() {
    VL53L0X_NYI(false);
  }

  bool Api::WaitDeviceReadyForNewMeasurement(unsigned int MaxLoop) {
    VL53L0X_NYI(false);
  }

  bool Api::EnableInterruptMask(uint8_t InterruptMask) {//ick: prior code used 32 bits when getMask only returns at most 8
    VL53L0X_NYI(false)
  }

#endif
}//end namespace
