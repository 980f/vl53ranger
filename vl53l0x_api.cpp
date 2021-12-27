/*******************************************************************************
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

#include "vl53l0x_api.h"
//inserted into core as it tangled the layers to the point of not having them: #include "vl53l0x_api_calibration.h"
#include "vl53l0x_api_core.h"
#include "vl53l0x_api_strings.h"  //should get rid of wrapping, only one ever had a concept of error and for that we can return a nullptr.
#include "vl53l0x_interrupt_threshold_settings.h" //some tuning parameters
#include "vl53l0x_tuning.h"  //tuning values packed into an array of bytes

#include "log_api.h"

#ifdef VL53L0X_LOG_ENABLE
#define trace_print(level, ...)   trace_print_module_function(TRACE_MODULE_API, level, TRACE_FUNCTION_NONE, ## __VA_ARGS__)
#endif

namespace VL53L0X {

#include "versioninfo.h"
#include "bitmanipulators.h"

  const Version_t Api::ImplementationVersion {VL53L0X_IMPLEMENTATION_VER_REVISION, {VL53L0X_IMPLEMENTATION_VER_MAJOR, VL53L0X_IMPLEMENTATION_VER_MINOR}, VL53L0X_IMPLEMENTATION_VER_SUB};

  const Version_t Api::PalSpecVersion {VL53L0X_SPECIFICATION_VER_REVISION, VL53L0X_SPECIFICATION_VER_MAJOR, VL53L0X_SPECIFICATION_VER_MINOR, VL53L0X_SPECIFICATION_VER_SUB};

/* Group PAL General Functions */


  Error Api::measurement_poll_for_completion() {
    LOG_FUNCTION_START;

    for (unsigned LoopNb = VL53L0X_DEFAULT_MAX_LOOP; LoopNb-- > 0;) {
      Erroneous<bool> value;
      value = GetMeasurementDataReady();
      if (value.isOk() && value == 1) {//980f: reversed legacy order of testing, test valid before testing value.
        return ERROR_NONE;
      }
      PollingDelay();//delay is on comm so that it can use platform specific technique
    }
    return ERROR_TIME_OUT;//was init to 'timeout'
  } // VL53L0X_measurement_poll_for_completion


  Error Api::check_part_used(uint8_t &Revision, DeviceInfo_t &pDeviceInfo) {
    LOG_FUNCTION_START;
    Error |= get_info_from_device(IDStuff);

    if (~Error) {
      if (VL53L0X_GETDEVICESPECIFICPARAMETER(ModuleId) == 0) {
        Revision = 0;
        COPYSTRING(pDeviceInfo.ProductId, "");
      } else {
        Revision = VL53L0X_GETDEVICESPECIFICPARAMETER(Revision);
        COPYSTRING(pDeviceInfo.ProductId, VL53L0X_GETDEVICESPECIFICPARAMETER(ProductId));
      }
    }
    return Error;
  } // check_part_used

  Error Api::get_device_info(DeviceInfo_t &pDeviceInfo) {
    uint8_t Revision;
    ErrorAccumulator Error = check_part_used(Revision, pDeviceInfo);
    ERROR_OUT;
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
    Error = comm.RdByte(REG_IDENTIFICATION_MODEL_ID, &pDeviceInfo.ProductType);
    ERROR_OUT;

    //DUP: duplicate code.
    uint8_t revision_id;
    Error = comm.RdByte(REG_IDENTIFICATION_REVISION_ID, &revision_id);
    pDeviceInfo.ProductRevision.major = 1;
    pDeviceInfo.ProductRevision.minor = revision_id >> 4; //BUG: rev id set even if read fails.

    return Error;
  } // get_device_info

  SemverLite Core::GetProductRevision() {

    LOG_FUNCTION_START;
    Erroneous<uint8_t> revision_id;

    fetch(revision_id, REG_IDENTIFICATION_REVISION_ID);

    if (revision_id.isOk()) {
      return {1, uint8_t(revision_id.wrapped >> 4)};
    }

    return {0, 0};//use a sentinal value instead of tedious error warpping.
  } // GetProductRevision

//Error Api::GetDeviceInfo(DeviceInfo_t &pVL53L0X_DeviceInfo){
//  LOG_FUNCTION_START;
//return get_device_info( pVL53L0X_DeviceInfo);
//}

  Erroneous<DeviceError> Api::GetDeviceErrorStatus() {
    LOG_FUNCTION_START;
    Erroneous<uint8_t> caster;
    fetch(caster, REG_RESULT_RANGE_STATUS);
    return {DeviceError((caster.wrapped & 0x78) >> 3), caster.error};
  } // GetDeviceErrorStatus

  const char *Api::GetDeviceErrorString(DeviceError ErrorCode) {
    return device_error_string(ErrorCode);
  }

  const char *Api::GetRangeStatusString(uint8_t RangeStatus) {
    return range_status_string(RangeStatus);
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

  Error Api::SetPowerMode(PowerModes PowerMode) {
    LOG_FUNCTION_START;

    /* Only level1 of Power mode exists */
    if ((PowerMode != POWERMODE_STANDBY_LEVEL1) && (PowerMode != POWERMODE_IDLE_LEVEL1)) {
      return LOG_ERROR(ERROR_MODE_NOT_SUPPORTED);
    }

    if (PowerMode == POWERMODE_STANDBY_LEVEL1) {
      /* set the standby level1 of power mode */
      Error |= comm.WrByte(0x80, 0x00);
      if (~Error) {
        /* Set PAL State to standby */
        PALDevDataSet(PalState, STATE_STANDBY);
        PALDevDataSet(PowerMode, POWERMODE_STANDBY_LEVEL1);
      }
    } else { /* VL53L0X_POWERMODE_IDLE_LEVEL1 */
      Error |= comm.WrByte(0x80, 0x00);
      if (~Error) {
        Error |= StaticInit();
      }
      if (~Error) {
        PALDevDataSet(PowerMode, POWERMODE_IDLE_LEVEL1);
      }
    }

    return ERROR_NONE;
  } // VL53L0X_SetPowerMode

  Erroneous<PowerModes> Api::GetPowerMode() {
    LOG_FUNCTION_START;
    Erroneous<uint8_t> Byte;
    fetch(Byte, Private_PowerMode);
    /* Only level1 of Power mode exists */
    if (Byte.isOk()) {
      auto encoded = (Byte == 1) ? POWERMODE_IDLE_LEVEL1 : POWERMODE_STANDBY_LEVEL1;
      PALDevDataSet(PowerMode, encoded);
      return {encoded};
    }

    return {Byte.error};
  } // GetPowerMode

  Error Api::SetOffsetCalibrationDataMicroMeter(int32_t OffsetCalibrationDataMicroMeter) {
    LOG_FUNCTION_START;
    return set_offset_calibration_data_micro_meter(OffsetCalibrationDataMicroMeter);
  }

  Erroneous<int32_t> Api::GetOffsetCalibrationDataMicroMeter() {
    LOG_FUNCTION_START;
    return get_offset_calibration_data_micro_meter();
  }

  Error Api::SetLinearityCorrectiveGain(int16_t LinearityCorrectiveGain) {
    LOG_FUNCTION_START;
    if ((LinearityCorrectiveGain < 0) || (LinearityCorrectiveGain > 1000)) {
      return ERROR_INVALID_PARAMS;
    }
    PALDevDataSet(LinearityCorrectiveGain, LinearityCorrectiveGain);

    if (LinearityCorrectiveGain != 1000) {
      /* Disable FW Xtalk */
      return comm.Write(REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, uint16_t(0));
    }

    return ERROR_NONE;
  } // VL53L0X_SetLinearityCorrectiveGain

  uint16_t Api::GetLinearityCorrectiveGain() {
    return PALDevDataGet(LinearityCorrectiveGain);
  }

  Erroneous<FixPoint1616_t> Api::GetTotalSignalRate() {
    LOG_FUNCTION_START;
    RangingMeasurementData_t LastRangeDataBuffer = PALDevDataGet(LastRangeMeasure);
    return get_total_signal_rate(LastRangeDataBuffer);
  } // GetTotalSignalRate

/* End Group PAL General Functions */

/* Group PAL Init Functions */
  bool Api::SetDeviceAddress(uint8_t _8bitAddress) {
    LOG_FUNCTION_START;
    Error |= comm.WrByte(REG_I2C_SLAVE_DEVICE_ADDRESS, _8bitAddress >> 1);
    if (Error == ERROR_NONE) {
      comm.wirer.devAddr = _8bitAddress; // 7 bit addr
      return true;
    } else {
      return false;
    }
  }

  Error Api::DataInit() {
    LOG_FUNCTION_START;

    /* by default the I2C is running at 1V8 if you want to change it you
     * need to include this define at compilation level. */
#ifdef USE_I2C_2V8
    Error |= comm.UpdateBit(REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, 0, 1);//ick: former code had stupid value (clearing and setting the same bit) that worked only because of a fine hidden detail in UpdateByte
#endif

    /* Set I2C standard mode */
    if (~Error) {
      Error |= comm.WrByte(0x88, 0x00);
    }

    /* read WHO_AM_I */
    uint8_t b;
    Error |= comm.RdByte(0xC0, &b);
    // Serial.print("WHOAMI: 0x"); Serial.println(b, HEX);

    /* read WHO_AM_I */
    VL53L0X_SETDEVICESPECIFICPARAMETER(ReadDataFromDeviceDone, 0);//forget we have ever read anything

#ifdef USE_IQC_STATION
    if (~Error) {
      //status was formerly ignored.
      MyDevice.apply_offset_adjustment();
    }
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

    if (CurrentParameters.isOk()) {//ick: did 980f invert this test?
      /* initialize PAL values */
      CurrentParameters.wrapped.DeviceMode = DEVICEMODE_SINGLE_RANGING;
      CurrentParameters.wrapped.HistogramMode = HISTOGRAMMODE_DISABLED;
      PALDevDataSet(CurrentParameters, CurrentParameters);
    }

    /* Sigma estimator variable */
    PALDevDataSet(SigmaEst.RefArray, 100);
    PALDevDataSet(SigmaEst.EffPulseWidth, 900);
    PALDevDataSet(SigmaEst.EffAmbWidth, 500);
    PALDevDataSet(targetRefRate, 20.0f); /* 20 MCPS in 9:7 format was 0x0A00 */

    /* Use internal default settings */
    PALDevDataSet(UseInternalTuningSettings, true);

    {
      auto magic = magicWrapper();
      Error |= comm.RdByte(REG_SYSRANGE_stopper, &PALDevDataGet(StopVariable));
    }
    ERROR_OUT;
    /* Enable all check */
    for (int i = 0; i < CHECKENABLE_NUMBER_OF_CHECKS; i++) {
      Error |= SetLimitCheckEnable(static_cast<CheckEnable>(i), true);
      ERROR_OUT;
    }

    /* Disable the following checks */
    Error |= SetLimitCheckEnable(CHECKENABLE_SIGNAL_REF_CLIP, 0);
    ERROR_OUT;
    Error |= SetLimitCheckEnable(CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);
    ERROR_OUT;
    Error |= SetLimitCheckEnable(CHECKENABLE_SIGNAL_RATE_MSRC, 0);
    ERROR_OUT;
    Error |= SetLimitCheckEnable(CHECKENABLE_SIGNAL_RATE_PRE_RANGE, 0);


    /* Limit default values */
    ERROR_OUT;
    Error |= SetLimitCheckValue(CHECKENABLE_SIGMA_FINAL_RANGE, 18.0F);
    ERROR_OUT;
    Error |= SetLimitCheckValue(CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 0.25F);
    ERROR_OUT;
    Error |= SetLimitCheckValue(CHECKENABLE_SIGNAL_REF_CLIP, 35.0F);
    ERROR_OUT;
    Error | SetLimitCheckValue(CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);
    ERROR_OUT;

    Error |= set_SequenceConfig(0xFF, true);//980f: changed to not save in data unless successfully set.
    /* Set PAL state to tell that we are waiting for call to StaticInit */
    PALDevDataSet(PalState, STATE_WAIT_STATICINIT);

    VL53L0X_SETDEVICESPECIFICPARAMETER(RefSpadsInitialised, false);
    return Error;
  } // VL53L0X_DataInit

  Error Api::SetTuningSettingBuffer(Tunings pTuningSettingBuffer, bool UseInternalTuningSettings) {
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
    return ERROR_NONE;
  } // VL53L0X_SetTuningSettingBuffer

  Tunings Api::GetTuningSettingBuffer(bool theInternal) {
    if (theInternal || PALDevDataGet(UseInternalTuningSettings)) {
      return DefaultTuningSettings;
    } else {
      return PALDevDataGet(pTuningSettingsPointer);
    }
  } // GetTuningSettingBuffer


  Error Api::initRanger(VcselPeriod periodType, SequenceStepId stepId, DeviceSpecificParameters_t::RangeSetting &ranger) {

    auto vcselPulsePeriodPCLK = GetVcselPulsePeriod(periodType);
    ERROR_ON(vcselPulsePeriodPCLK);
    ranger.VcselPulsePeriod = vcselPulsePeriodPCLK;

    auto seqTimeoutMilliSecs = GetSequenceStepTimeout(stepId);
    ERROR_ON(seqTimeoutMilliSecs);
    ranger.TimeoutMicroSecs = seqTimeoutMilliSecs.wrapped;
    return ERROR_NONE;
  }

  Error Api::StaticInit() {
    LOG_FUNCTION_START;

    Error |= get_info_from_device(InfoGroup::SpadStuff);

    /* set the ref spad from NVM */
    SpadCount ref = VL53L0X_GETDEVICESPECIFICPARAMETER(ReferenceSpad);

    /* NVM value invalid
     * two known types, 1 and 0, '1' has a max 32 spads, '0' has a max of 12
     * */
    if (ref.quantity > (ref.isAperture ? 32 : 12)) {//if true then invalid settings so compute correct ones
      Error |= perform_ref_spad_management(ref);
    } else {
      Error |= set_reference_spads(ref);
    }
    ERROR_OUT;

    Error |= load_tuning_settings(GetTuningSettingBuffer());
    ERROR_OUT;
    /* Set interrupt config to new sample ready */
    Error |= SetGpioConfig(0, {DEVICEMODE_SINGLE_RANGING, GPIOFUNCTIONALITY_NEW_MEASURE_READY, INTERRUPTPOLARITY_LOW});
    ERROR_OUT;
    Erroneous<FixPoint<4, 12>> fix412 = FFread<FixPoint<4, 12>>(RegSystem(0x84));
    ERROR_OUT;
    VL53L0X_SETDEVICESPECIFICPARAMETER(OscFrequencyMHz, fix412.wrapped);//conversion from 412 to 1616 is inferred by compiler


    /* After static init, some device parameters may be changed,
     * so update them */
    Erroneous<DeviceParameters_t> CurrentParameters = GetDeviceParameters();
    ERROR_ON(CurrentParameters)

    Erroneous<bool> fe = GetFractionEnable();
    ERROR_ON(fe);
    PALDevDataSet(RangeFractionalEnable, fe);
    PALDevDataSet(CurrentParameters, CurrentParameters);

    /* read the sequence config and save it */
    auto sse = get_SequenceConfig();
    ERROR_ON(sse)
    PALDevDataSet(SequenceConfig, sse.wrapped);

    /* Disable MSRC and TCC by default */
    Error |= SetSequenceStepEnable(SEQUENCESTEP_TCC, false);
    ERROR_OUT;
    Error |= SetSequenceStepEnable(SEQUENCESTEP_MSRC, false);
    ERROR_OUT;
    /* Set PAL State to standby */
    PALDevDataSet(PalState, STATE_IDLE);

    Error |= initRanger(VCSEL_PERIOD_PRE_RANGE, SEQUENCESTEP_PRE_RANGE, Data.DeviceSpecificParameters.PreRange);
    ERROR_OUT;
    Error |= initRanger(VCSEL_PERIOD_FINAL_RANGE, SEQUENCESTEP_FINAL_RANGE, Data.DeviceSpecificParameters.FinalRange);
    return Error;
  }

  Error Api::waitOnResetIndicator(bool disappear) {
    for (uint8_t Byte = disappear ? ~0 : 0; disappear == (Byte != 0);) {//BUG: potentially infinite loop
      if (comm.RdByte(REG_IDENTIFICATION_MODEL_ID, &Byte)) {
        return ERROR_CONTROL_INTERFACE;
      }
    }
    return ERROR_NONE;
  }

  Error Api::ResetDevice() {
    LOG_FUNCTION_START;
//todo: error handling here is about as worthless as elsewhere
    /* Set reset bit */
    Error |= comm.WrByte(REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x00);

    /* Wait for some time */
    if (!~Error) {
      Error |= waitOnResetIndicator(true);//ick: former code could hang on comm error, this quits
    }
    /* Release reset */
    Error |= comm.WrByte(REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x01);//ick: ignores error

    /* Wait until correct boot-up of the device */
    if (!~Error) {
      Error |= waitOnResetIndicator(false);//ick: former code could hang on comm error, this quits
    }

    /* Set PAL State to State_POWERDOWN */
    if (!~Error) {
      PALDevDataSet(PalState, STATE_POWERDOWN);
    }
    return Error;
  }
  // VL53L0X_ResetDevice

/* End Group PAL Init Functions */

/* Group PAL Parameters Functions */
  Error Api::SetDeviceParameters(const DeviceParameters_t &pDeviceParameters) {
    LOG_FUNCTION_START;
    Error |= SetDeviceMode(pDeviceParameters.DeviceMode);
    ERROR_OUT;
    Error |= SetInterMeasurementPeriodMilliSeconds(pDeviceParameters.InterMeasurementPeriodMilliSeconds);
    ERROR_OUT;
    Error |= SetXTalkCompensationRateMegaCps(pDeviceParameters.XTalkCompensationRateMegaCps);
    ERROR_OUT;
    Error |= SetOffsetCalibrationDataMicroMeter(pDeviceParameters.RangeOffsetMicroMeters);
    ERROR_OUT;

    for (unsigned i = 0; i < CHECKENABLE_NUMBER_OF_CHECKS; i++) {
      Error |= SetLimitCheckEnable(static_cast<CheckEnable>(i), pDeviceParameters.LimitChecksEnable[i]);
      ERROR_OUT;
      Error |= SetLimitCheckValue(static_cast<CheckEnable>(i), pDeviceParameters.LimitChecksValue[i]);
      ERROR_OUT;
    }
    Error |= SetWrapAroundCheckEnable(pDeviceParameters.WrapAroundCheckEnable);
    ERROR_OUT;
    Error |= SetMeasurementTimingBudgetMicroSeconds(pDeviceParameters.MeasurementTimingBudgetMicroSeconds);

    return Error;
  } // VL53L0X_SetDeviceParameters


  Error Api::SetDeviceMode(DeviceModes deviceMode) {
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
        break;
      default:
        /* Unsupported mode */
        return LOG_ERROR(ERROR_MODE_NOT_SUPPORTED);
    } // switch
    return ERROR_NONE;
  } // VL53L0X_SetDeviceMode

  DeviceModes Api::GetDeviceMode() {
    return Data.CurrentParameters.DeviceMode;
  }

  Error Api::SetRangeFractionEnable(bool Enable) {

    LOG_FUNCTION_START;
    Error(" %u", Enable);

    Error |= comm.WrByte(REG_SYSTEM_RANGE_CONFIG, Enable);

    if (Error == ERROR_NONE) {
      PALDevDataSet(RangeFractionalEnable, Enable);
    }

    return Error;
  } // VL53L0X_SetRangeFractionEnable

  Erroneous<bool> Api::GetFractionEnable() {
    LOG_FUNCTION_START;
    Erroneous<bool> enabled;
    fetch(enabled, REG_SYSTEM_RANGE_CONFIG);
    if (enabled.isOk()) {
      enabled.wrapped &= 1;
    }
    return enabled;
  } // GetFractionEnable


  Erroneous<uint32_t> Api::GetMeasurementTimingBudgetMicroSeconds() {
    return get_measurement_timing_budget_micro_seconds();
  }

  Error Api::SetVcselPulsePeriod(VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriodPCLK) {
    //todo: depending upon success of following call then call perform_phase_calibration

    return set_vcsel_pulse_period(VcselPeriodType, VCSELPulsePeriodPCLK);
  }

  Erroneous<uint8_t> Api::GetVcselPulsePeriod(VcselPeriod VcselPeriodType) {
    LOG_FUNCTION_START;
    return get_vcsel_pulse_period(VcselPeriodType);
  }

  Error Api::SetSequenceStepEnable(SequenceStepId SequenceStepId, bool SequenceStepEnabled) {
    LOG_FUNCTION_START;
    unsigned bitnum = bitFor(SequenceStepId);
    if (bitnum == ~0) {
      return ERROR_INVALID_PARAMS;
    }
    auto SequenceConfig = get_SequenceConfig();
    if (SequenceConfig.isOk()) {
      uint8_t SequenceConfigNew = SequenceConfig;
      setBit(bitnum, SequenceConfigNew, SequenceStepEnabled);
      if (SequenceStepId == SEQUENCESTEP_DSS) {//legacy weirdness, set bit 5 in tandem wtih bit for DSS
        setBit<5>(SequenceConfigNew, SequenceStepEnabled);
      }

      if (SequenceConfigNew != SequenceConfig) {
        /* Apply New Setting */
        Error |= comm.WrByte(REG_SYSTEM_SEQUENCE_CONFIG, SequenceConfigNew);
        if (Error == ERROR_NONE) {
          PALDevDataSet(SequenceConfig, SequenceConfigNew);
          /* Recalculate timing budget */
          uint32_t MeasurementTimingBudgetMicroSeconds = VL53L0X_GETPARAMETERFIELD(MeasurementTimingBudgetMicroSeconds);
          SetMeasurementTimingBudgetMicroSeconds(MeasurementTimingBudgetMicroSeconds);
        }
      }
    }

    return Error;
  } // VL53L0X_SetSequenceStepEnable

  SequenceStepId Api::GetNumberOfSequenceSteps() {
    return SEQUENCESTEP_NUMBER_OF_CHECKS;
  }

  const char *Api::GetSequenceStepsInfo(SequenceStepId SequenceStepId) {
    return sequence_steps_info(SequenceStepId);
  } // GetSequenceStepsInfo

  Error Api::SetSequenceStepTimeout(SequenceStepId SequenceStepId, FixPoint1616_t TimeOutMilliSecs) {
    LOG_FUNCTION_START;
    /* Read back the current value in case we need to revert back to this.  */

    auto OldTimeOutMicroSeconds = get_sequence_step_timeout(SequenceStepId);
    if (OldTimeOutMicroSeconds.isOk()) {
      auto TimeoutMicroSeconds = TimeOutMilliSecs.millis();
      Error |= set_sequence_step_timeout(SequenceStepId, TimeoutMicroSeconds);
      ERROR_OUT;
      uint32_t MeasurementTimingBudgetMicroSeconds = VL53L0X_GETPARAMETERFIELD(MeasurementTimingBudgetMicroSeconds);

      /* At this point we don't know if the requested value is valid,
       *  therefore proceed to update the entire timing budget and
       *  if this fails, revert back to the previous value.
       */
      Error |= SetMeasurementTimingBudgetMicroSeconds(MeasurementTimingBudgetMicroSeconds);
      if (~Error) {//budget failed to restore prior value, and have to rerun budget because of its potential side effects
        Error |= set_sequence_step_timeout(SequenceStepId, OldTimeOutMicroSeconds);
        ERROR_OUT;
        Error |= SetMeasurementTimingBudgetMicroSeconds(MeasurementTimingBudgetMicroSeconds);
      }
    }
    return Error;
  } // VL53L0X_SetSequenceStepTimeout

  Erroneous<FixPoint1616_t> Api::GetSequenceStepTimeout(SequenceStepId SequenceStepId) {
    LOG_FUNCTION_START;

    auto TimeoutMicroSeconds = get_sequence_step_timeout(SequenceStepId);
    if (TimeoutMicroSeconds.isOk()) {
      return FixPoint1616_t(TimeoutMicroSeconds, 1000);//gnarly stuff here moved into a constructor
    } else {
      return {TimeoutMicroSeconds.error};
    }
  } // GetSequenceStepTimeout

  Error Api::SetInterMeasurementPeriodMilliSeconds(uint32_t InterMeasurementPeriodMilliSeconds) {
    uint16_t osc_calibrate_val;
    uint32_t IMPeriodMilliSeconds;

    LOG_FUNCTION_START;

    Error |= comm.RdWord(REG_OSC_CALIBRATE_VAL, &osc_calibrate_val);

    if (Error == ERROR_NONE) {
      if (osc_calibrate_val != 0) {
        IMPeriodMilliSeconds = InterMeasurementPeriodMilliSeconds * osc_calibrate_val;
      } else {
        IMPeriodMilliSeconds = InterMeasurementPeriodMilliSeconds;
      }
      Error |= comm.WrDWord(REG_SYSTEM_INTERMEASUREMENT_PERIOD, IMPeriodMilliSeconds);
    }

    if (Error == ERROR_NONE) {
      VL53L0X_SETPARAMETERFIELD(InterMeasurementPeriodMilliSeconds, InterMeasurementPeriodMilliSeconds);
    }

    return Error;
  } // VL53L0X_SetInterMeasurementPeriodMilliSeconds

  Erroneous<uint32_t> Api::GetInterMeasurementPeriodMilliSeconds() {
    LOG_FUNCTION_START;
    Erroneous<uint16_t> osc_calibrate_val;
    if (fetch(osc_calibrate_val, REG_OSC_CALIBRATE_VAL)) {
      Erroneous<uint32_t> IMPeriodMilliSeconds;
      if (fetch(IMPeriodMilliSeconds, REG_SYSTEM_INTERMEASUREMENT_PERIOD)) {

        auto ratio = roundedDivide(IMPeriodMilliSeconds.wrapped, osc_calibrate_val);
        if (ratio != 0) {//presuming due to div by zero
          VL53L0X_SETPARAMETERFIELD(InterMeasurementPeriodMilliSeconds, ratio);
        }
        return ratio;
      }
    }
    return {osc_calibrate_val.error};
  } // GetInterMeasurementPeriodMilliSeconds


  bool Api::GetXTalkCompensationEnable() {
    LOG_FUNCTION_START;
    return VL53L0X_GETPARAMETERFIELD(XTalkCompensationEnable);
  } // GetXTalkCompensationEnable

  Error Api::SetXTalkCompensationRateMegaCps(FixPoint1616_t XTalkCompensationRateMegaCps) {
    LOG_FUNCTION_START;
    bool xtcEnable = GetXTalkCompensationEnable();
    uint16_t LinearityCorrectiveGain = PALDevDataGet(LinearityCorrectiveGain);

    if (!xtcEnable) { /* disabled write only internal value */
      VL53L0X_SETPARAMETERFIELD(XTalkCompensationRateMegaCps, XTalkCompensationRateMegaCps);
    } else {
      FixPoint<3, 13> data = (LinearityCorrectiveGain == 1000) ? XTalkCompensationRateMegaCps : FixPoint1616_t(0);
      Error |= comm.WrWord(REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, data);
      if (Error == ERROR_NONE) {
        VL53L0X_SETPARAMETERFIELD(XTalkCompensationRateMegaCps, XTalkCompensationRateMegaCps);
      }
    }

    return Error;
  } // VL53L0X_SetXTalkCompensationRateMegaCps

  Erroneous<FixPoint1616_t> Api::GetXTalkCompensationRateMegaCps() {
    LOG_FUNCTION_START;
    Erroneous<FixPoint<3, 13>> Value;
    if (fetch(Value, REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS)) {
      if (Value.wrapped.raw == 0) {
        /* the Xtalk is disabled, return value from memory */
        VL53L0X_SETPARAMETERFIELD(XTalkCompensationEnable, false);
      } else {
        VL53L0X_SETPARAMETERFIELD(XTalkCompensationEnable, true);
        VL53L0X_SETPARAMETERFIELD(XTalkCompensationRateMegaCps, Value.wrapped);
      }
      return VL53L0X_GETPARAMETERFIELD(XTalkCompensationRateMegaCps);
    } else {
      return {Value.error};
    }
  } // GetXTalkCompensationRateMegaCps

  Error Api::SetRefCalibration(CalibrationParameters p) {
    LOG_FUNCTION_START;
    return set_ref_calibration(p, true, true);
  }

  Erroneous<Api::CalibrationParameters> Api::GetRefCalibration() {
    LOG_FUNCTION_START;
    return get_ref_calibration(true, true);
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

  Erroneous<bool> Api::GetLimitCheckStatus(CheckEnable LimitCheckId) {
    if (LimitCheckId >= CHECKENABLE_NUMBER_OF_CHECKS) {
      return {false, LOG_ERROR(ERROR_INVALID_PARAMS)};
    } else {
      return Data.CurrentParameters.LimitChecksStatus[LimitCheckId];
    }
  } // GetLimitCheckStatus

  Error Api::SetLimitCheckEnable(CheckEnable LimitCheckId, bool LimitCheckEnable) {
    if (LimitCheckId >= CHECKENABLE_NUMBER_OF_CHECKS) {
      return LOG_ERROR(ERROR_INVALID_PARAMS);
    }

    LOG_FUNCTION_START;
    LimitCheckEnable = LimitCheckEnable != 0;//in case someone got around compiler checking
    bool LimitCheckDisable = !LimitCheckEnable;

    FixPoint<9, 7> TempFix1616 = LimitCheckEnable ? VL53L0X_GETARRAYPARAMETERFIELD(LimitChecksValue, LimitCheckId) : FixPoint<9, 7>(0);

    switch (LimitCheckId) {
      case CHECKENABLE_SIGMA_FINAL_RANGE:
        /* internal computation: */
        VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksEnable, CHECKENABLE_SIGMA_FINAL_RANGE, LimitCheckEnable);
        break;

      case CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
        Error |= comm.WrWord(REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, TempFix1616);
        break;

      case CHECKENABLE_SIGNAL_REF_CLIP:
        /* internal computation: */
        VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksEnable, CHECKENABLE_SIGNAL_REF_CLIP, LimitCheckEnable);
        break;

      case CHECKENABLE_RANGE_IGNORE_THRESHOLD:
        /* internal computation: */
        VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksEnable, CHECKENABLE_RANGE_IGNORE_THRESHOLD, LimitCheckEnable);
        break;

      case CHECKENABLE_SIGNAL_RATE_MSRC:
        Error |= comm.UpdateByte(REG_MSRC_CONFIG_CONTROL, ~(1 << 0), LimitCheckDisable << 1);//BUG:clear lsb set bit 1
        break;

      case CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
        Error |= comm.UpdateByte(REG_MSRC_CONFIG_CONTROL, ~(1 << 4), LimitCheckDisable << 4);
        break;

      default:
        return ERROR_INVALID_PARAMS;
    } // switch


    if (Error == ERROR_NONE) {
      VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksEnable, LimitCheckId, LimitCheckEnable);
    }
    return Error;
  } // VL53L0X_SetLimitCheckEnable


  Error Api::SetLimitCheckValue(CheckEnable LimitCheckId, FixPoint<9, 7> LimitCheckValue) {

    LOG_FUNCTION_START;

    auto Temp8 = VL53L0X_GETARRAYPARAMETERFIELD(LimitChecksEnable, LimitCheckId);

    if (Temp8 == 0) { /* disabled write only internal value */
      VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksValue, LimitCheckId, LimitCheckValue);
    } else {
      switch (LimitCheckId) {
        case CHECKENABLE_SIGMA_FINAL_RANGE:
          /* internal computation: */
          VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksValue, CHECKENABLE_SIGMA_FINAL_RANGE, LimitCheckValue);
          break;

        case CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
          Error |= comm.WrWord(REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, LimitCheckValue);
          break;

        case CHECKENABLE_SIGNAL_REF_CLIP:
          /* internal computation: */
          VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksValue, CHECKENABLE_SIGNAL_REF_CLIP, LimitCheckValue);
          break;

        case CHECKENABLE_RANGE_IGNORE_THRESHOLD:
          /* internal computation: */
          VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksValue, CHECKENABLE_RANGE_IGNORE_THRESHOLD, LimitCheckValue);
          break;

        case CHECKENABLE_SIGNAL_RATE_MSRC:
        case CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
          Error |= comm.WrWord(REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT, LimitCheckValue);
          break;

        default:
          return ERROR_INVALID_PARAMS;
      } // switch

      if (Error == ERROR_NONE) {
        VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksValue, LimitCheckId, LimitCheckValue);
      }
    }
    return Error;
  } // VL53L0X_SetLimitCheckValue


  Erroneous<FixPoint1616_t> Api::GetLimitCheckCurrent(CheckEnable LimitCheckId) {
    if (LimitCheckId >= CHECKENABLE_NUMBER_OF_CHECKS) {
      return ERROR_INVALID_PARAMS;
    }
    switch (LimitCheckId) {
      case CHECKENABLE_SIGMA_FINAL_RANGE:
        /* Need to run a ranging to have the latest values */
        return PALDevDataGet(SigmaEstimate);
        break;

      case CHECKENABLE_SIGNAL_REF_CLIP:
        /* Need to run a ranging to have the latest values */
        return PALDevDataGet(LastSignalRefMcps);
        break;

      case CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
      case CHECKENABLE_RANGE_IGNORE_THRESHOLD:
      case CHECKENABLE_SIGNAL_RATE_MSRC:
      case CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
        /* Need to run a ranging to have the latest values */;
        return PALDevDataGet(LastRangeMeasure).SignalRateRtnMegaCps;
        break;

      default:
        return LOG_ERROR(ERROR_INVALID_PARAMS);
    } // switch

  } // GetLimitCheckCurrent

/*
 * WRAPAROUND Check
 */
  Error Api::SetWrapAroundCheckEnable(bool WrapAroundCheckEnable) {
    bool WrapAroundCheckEnableInt = WrapAroundCheckEnable != 0;//in case someone somehow circumvented compiler checks
    LOG_FUNCTION_START;
    Error |= comm.UpdateBit(REG_SYSTEM_SEQUENCE_CONFIG, 7, WrapAroundCheckEnableInt);
    ERROR_OUT;
    setBit<7>(Data.SequenceConfig, WrapAroundCheckEnableInt);
    VL53L0X_SETPARAMETERFIELD(WrapAroundCheckEnable, WrapAroundCheckEnableInt);
    return ERROR_NONE;
  } // VL53L0X_SetWrapAroundCheckEnable

  Erroneous<bool> Api::GetWrapAroundCheckEnable() {
    LOG_FUNCTION_START;
    Erroneous<uint8_t> seqconf = get_SequenceConfig();
    if (seqconf.isOk()) {
      PALDevDataSet(SequenceConfig, seqconf);
      bool enable = getBit<7>(seqconf.wrapped);
      VL53L0X_SETPARAMETERFIELD(WrapAroundCheckEnable, enable);
      return ERROR_NONE;
    } else {
      return {seqconf.error};
    }
  } // GetWrapAroundCheckEnable

  Error Api::SetDmaxCalParameters(const DevData_t::DmaxCal &p) {
    LOG_FUNCTION_START;

    /* Check if one of input parameter is zero, in that case the
     * value are get from NVM */
    if ((p.RangeMilliMeter == 0) || (p.SignalRateRtnMegaCps.raw == 0)) {
      /* NVM parameters */
      /* Run Get_info_from_device to get signal rate at 400 mm.
       * if the value have been already fetched this function will return with no access to device */
      Error |= get_info_from_device(PartUidEtc);
      //BUG: need to check error
      PALDevDataSet(dmaxCal.RangeMilliMeter, 400);
      PALDevDataSet(dmaxCal.SignalRateRtnMegaCps, VL53L0X_GETDEVICESPECIFICPARAMETER(SignalRateMeasFixed400mm));
    } else {
      /* User parameters */
      PALDevDataSet(dmaxCal.RangeMilliMeter, p.RangeMilliMeter);
      PALDevDataSet(dmaxCal.SignalRateRtnMegaCps, p.SignalRateRtnMegaCps);
    }

    return Error;
  } // VL53L0X_SetDmaxCalParameters

  DevData_t::DmaxCal Api::GetDmaxCalParameters() {
    return PALDevDataGet(dmaxCal);
  } // GetDmaxCalParameters

/* End Group PAL Parameters Functions */

/* Group PAL Measurement Functions */
  Error Api::PerformSingleMeasurement() {
    LOG_FUNCTION_START;

    /* Get Current DeviceMode */
    auto DeviceMode = GetDeviceMode();
    /* Start immediately to run a single ranging measurement in case of
     * single ranging or single histogram */
    if (DeviceMode == DEVICEMODE_SINGLE_RANGING) {
      Error |= StartMeasurement();
      ERROR_OUT;
    }
//but if not in single then is continuous and one will have started and will complete on its own
    auto completed = measurement_poll_for_completion();
    ERROR_OUT;

    if (completed) {
      if (DeviceMode == DEVICEMODE_SINGLE_RANGING) {
        PALDevDataSet(PalState, STATE_IDLE);
      }
    }
    return ERROR_NONE;
  } // VL53L0X_PerformSingleMeasurement


  Erroneous<Api::CalibrationParameters> Api::PerformRefCalibration() {
    LOG_FUNCTION_START;
    return perform_ref_calibration(true);
  }

  Error Api::PerformXTalkCalibration(FixPoint1616_t XTalkCalDistance, FixPoint1616_t &pXTalkCompensationRateMegaCps) {
    LOG_FUNCTION_START;
    return perform_xtalk_calibration(XTalkCalDistance, pXTalkCompensationRateMegaCps);
  }

  Erroneous<int32_t> Api::PerformOffsetCalibration(FixPoint1616_t CalDistanceMilliMeter) {
    LOG_FUNCTION_START;
    return perform_offset_calibration(CalDistanceMilliMeter);
  }

  Error Api::CheckAndLoadInterruptSettings(bool StartNotStopFlag) {
    ErrorAccumulator Error = ERROR_NONE;

    uint8_t InterruptConfig = VL53L0X_GETDEVICESPECIFICPARAMETER(Pin0GpioFunctionality);

    if ((InterruptConfig == GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW) ||
        (InterruptConfig == GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH) ||
        (InterruptConfig == GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT)) {
      RangeWindow Threshold;
      Error = GetInterruptThresholds(DEVICEMODE_CONTINUOUS_RANGING, Threshold);
      ERROR_OUT;
      FixPoint1616_t BigEough(255.0F);//was 255 * 65536 which is the same as 255.0
      if ((Threshold.Low > BigEough) || (Threshold.High > BigEough)) {
        if (StartNotStopFlag) {
          Error = load_tuning_settings(InterruptThresholdSettings);
        } else {
          Error |= comm.WrByte(0xFF, 0x04);
          Error |= comm.WrByte(0x70, 0x00);
          Error |= comm.WrByte(0xFF, 0x00);
          Error |= comm.WrByte(0x80, 0x00);
        }
      }
    }

    return Error;
  } // VL53L0X_CheckAndLoadInterruptSettings

  Error Api::StartMeasurement() {
    LOG_FUNCTION_START;

    DeviceModes DeviceMode = GetDeviceMode();

    {
      auto popper = magicWrapper();
      Error |= comm.WrByte(REG_SYSRANGE_stopper, PALDevDataGet(StopVariable));
    }
    ERROR_OUT;
    switch (DeviceMode) {
      case DEVICEMODE_SINGLE_RANGING:
        Error |= comm.WrByte(REG_SYSRANGE_START, REG_SYSRANGE_MODE_START_STOP | REG_SYSRANGE_MODE_SINGLESHOT);
        if (Error == ERROR_NONE) {
          /* Wait until start bit has been cleared */
          for (unsigned LoopNb = VL53L0X_DEFAULT_MAX_LOOP; LoopNb-- > 0;) {
            Erroneous<uint8_t> flags;
            fetch(flags, REG_SYSRANGE_START);
            ERROR_ON(flags);
            if (getBit<0>(flags.wrapped) == 0) {
              return ERROR_NONE;
            }
          }
          return ERROR_TIME_OUT;
        }
        break;
      case DEVICEMODE_CONTINUOUS_RANGING: /* Back-to-back mode */
        /* Check if need to apply interrupt settings */
        Error |= Api::CheckAndLoadInterruptSettings(1);
        Error |= comm.WrByte(REG_SYSRANGE_START, REG_SYSRANGE_MODE_BACKTOBACK);
        if (Error == ERROR_NONE) {
          /* Set PAL State to Running */
          PALDevDataSet(PalState, STATE_RUNNING);
        }
        break;
      case DEVICEMODE_CONTINUOUS_TIMED_RANGING:   /* Continuous mode
         * Check if need to apply interrupt settings*/
        Error |= Api::CheckAndLoadInterruptSettings(1);
        Error |= comm.WrByte(REG_SYSRANGE_START, REG_SYSRANGE_MODE_TIMED);
        if (Error == ERROR_NONE) {
          /* Set PAL State to Running */
          PALDevDataSet(PalState, STATE_RUNNING);
        }
        break;
      default:
        /* Selected mode not supported */
        return ERROR_MODE_NOT_SUPPORTED;
    } // switch


    return Error;
  } // VL53L0X_StartMeasurement

  Error Api::StopMeasurement() {
    LOG_FUNCTION_START;

    Error |= comm.WrByte(REG_SYSRANGE_START, REG_SYSRANGE_MODE_SINGLESHOT);
    {
      auto popper = MagicDuo(comm);
      Error |= comm.WrByte(REG_SYSRANGE_stopper, 0x00);
    }

    ERROR_OUT;
    /* Set PAL State to Idle */
    PALDevDataSet(PalState, STATE_IDLE);
    /* Check if need to apply interrupt settings */
    Error |= CheckAndLoadInterruptSettings(0);
    return Error;
  } // VL53L0X_StopMeasurement


  Erroneous<bool> Api::GetMeasurementDataReady() {
    LOG_FUNCTION_START;
    uint8_t InterruptConfig = VL53L0X_GETDEVICESPECIFICPARAMETER(Pin0GpioFunctionality);

    if (InterruptConfig == GPIOFUNCTIONALITY_NEW_MEASURE_READY) {
      Erroneous<uint8_t> mask = GetInterruptMaskStatus();
      if (mask.isOk()) {
        return mask == GPIOFUNCTIONALITY_NEW_MEASURE_READY;
      } else {
        return {mask.error};
      }
    } else {
      Erroneous<uint8_t> SysRangeStatusRegister;
      fetch(SysRangeStatusRegister, REG_RESULT_RANGE_STATUS);
      if (SysRangeStatusRegister.isOk()) {
        return getBit<0>(SysRangeStatusRegister.wrapped);
      } else {
        return {false, SysRangeStatusRegister.error};
      }
    }
  } // GetMeasurementDataReady

  Error Api::WaitDeviceReadyForNewMeasurement(unsigned MaxLoop) {
    VL53L0X_NYI
  }

  Error Api::GetRangingMeasurementData(RangingMeasurementData_t &pRangingMeasurementData) {
    LOG_FUNCTION_START;
    /*
     * use multi read even if some registers are not useful, result will
     * be more efficient
     * start reading at 0x14 dec20
     * end reading at 0x21 dec33 total 14 bytes to read  //ick: code says 12, not 14
     */

    uint8_t localBuffer[12];
    Error |= comm.ReadMulti(0x14, localBuffer, 12);
    ERROR_OUT;

    pRangingMeasurementData.ZoneId = 0;    /* Only one zone */
    pRangingMeasurementData.TimeStamp = 0; /* Not Implemented */

    /* cut1.1 if SYSTEM__RANGE_CONFIG if 1 range is 2bits fractional
     *(format 11.2) else no fractional
     */

    pRangingMeasurementData.MeasurementTimeUsec = 0;

    /* peak_signal_count_rate_rtn_mcps */
    FixPoint1616_t SignalRate = FixPoint<9, 7>(MAKEUINT16(localBuffer[7], localBuffer[6]));
    pRangingMeasurementData.SignalRateRtnMegaCps = SignalRate;

    pRangingMeasurementData.AmbientRateRtnMegaCps = FixPoint<9, 7>(MAKEUINT16(localBuffer[9], localBuffer[8]));

    FixPoint<8, 8> EffectiveSpadRtnCount(MAKEUINT16(localBuffer[3], localBuffer[2]));
    pRangingMeasurementData.EffectiveSpadRtnCount = EffectiveSpadRtnCount;

    auto DeviceRangeStatus = localBuffer[0];

    /* Get Linearity Corrective Gain */
    uint16_t LinearityCorrectiveGain = PALDevDataGet(LinearityCorrectiveGain);

    /* Get ranging configuration */
    bool RangeFractionalEnable = PALDevDataGet(RangeFractionalEnable);

    auto RangeMilliMeter = MAKEUINT16(localBuffer[11], localBuffer[10]);//todo: needs informative name
    if (LinearityCorrectiveGain != 1000) {
      RangeMilliMeter = roundedDivide(LinearityCorrectiveGain * RangeMilliMeter, 1000);

      /* Implement Xtalk */
      bool XTalkCompensationEnable = VL53L0X_GETPARAMETERFIELD(XTalkCompensationEnable);

      if (XTalkCompensationEnable) {
        uint16_t XTalkCompensationRateMegaCps = VL53L0X_GETPARAMETERFIELD(XTalkCompensationRateMegaCps);
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
    ERROR_ON(PalRangeStatus)
    pRangingMeasurementData.rangeError = PalRangeStatus;

    /* Copy last read data into Dev buffer */
    PALDevDataGet(LastRangeMeasure) = pRangingMeasurementData;

    return Error;
  } // GetRangingMeasurementData

  FixPoint1616_t Api::GetMeasurementRefSignal() {
    return PALDevDataGet(LastSignalRefMcps);
  }

  Error Api::PerformSingleRangingMeasurement(RangingMeasurementData_t &pRangingMeasurementData) {
    LOG_FUNCTION_START;
    /* This function will do a complete single ranging
     * Here we fix the mode! */
    Error |= SetDeviceMode(DEVICEMODE_SINGLE_RANGING);
    ERROR_OUT;
    Error |= PerformSingleMeasurement();
    ERROR_OUT;
    Error |= GetRangingMeasurementData(pRangingMeasurementData);
    ERROR_OUT;
    Error |= ClearInterruptMask(0);

    return Error;
  } // VL53L0X_PerformSingleRangingMeasurement

  Error Api::SetNumberOfROIZones(uint8_t NumberOfROIZones) {
    if (NumberOfROIZones != 1) {
      return LOG_ERROR(ERROR_INVALID_PARAMS);
    }
    return ERROR_NONE;
  } // VL53L0X_SetNumberOfROIZones

  unsigned Api::GetNumberOfROIZones() {
    return 1;
  }

  unsigned Api::GetMaxNumberOfROIZones() {
    return 1;
  }

/* End Group PAL Measurement Functions */

  Error Api::SetGpioConfig(uint8_t Pin, GpioConfiguration gpioConfig) {
    LOG_FUNCTION_START;

    if (Pin != 0) {
      return ERROR_GPIO_NOT_EXISTING;
    }

    switch (gpioConfig.devMode) {
      case DEVICEMODE_GPIO_DRIVE:
        return comm.WrByte(REG_GPIO_HV_MUX_ACTIVE_HIGH, (gpioConfig.polarity == INTERRUPTPOLARITY_LOW) ? Bitter(4) : 1);//ick: elsewhere does an update
      case DEVICEMODE_GPIO_OSC:

        Error |= comm.WrByte(0xff, 0x01);
        Error |= comm.WrByte(0x00, 0x00);
        Error |= comm.WrByte(0xff, 0x00);

        Error |= comm.WrByte(0x80, 0x01);
        Error |= comm.WrByte(0x85, 0x02);

        Error |= comm.WrByte(0xff, 0x04);
        Error |= comm.WrByte(0xcd, 0x00);
        Error |= comm.WrByte(0xcc, 0x11);

        Error |= comm.WrByte(0xff, 0x07);
        Error |= comm.WrByte(0xbe, 0x00);

        Error |= comm.WrByte(0xff, 0x06);
        Error |= comm.WrByte(0xcc, 0x09);

        Error |= comm.WrByte(0xff, 0x00);
        Error |= comm.WrByte(0xff, 0x01);
        Error |= comm.WrByte(0x00, 0x00);
        break;
      default:
        if (!valid(gpioConfig.function)) {
          return ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED;
        } // switch

        Error |= comm.WrByte(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, gpioConfig.function);
        ERROR_OUT;
        Error |= comm.UpdateBit(REG_GPIO_HV_MUX_ACTIVE_HIGH, 4, gpioConfig.polarity == INTERRUPTPOLARITY_HIGH);
        ERROR_OUT;
        VL53L0X_SETDEVICESPECIFICPARAMETER(Pin0GpioFunctionality, gpioConfig.function);
        ERROR_OUT;
        Error |= ClearInterruptMask(0);
    }

    return Error;
  } // VL53L0X_SetGpioConfig

  Erroneous<GpioConfiguration> Api::GetGpioConfig(uint8_t Pin) {
    if (Pin != 0) {
      return LOG_ERROR(ERROR_GPIO_NOT_EXISTING);
    }
    LOG_FUNCTION_START;

    /* pDeviceMode not managed by Ewok it return the current mode */
    Erroneous<GpioConfiguration> gpio;
    gpio.wrapped.devMode = GetDeviceMode();
    Erroneous<uint8_t> gpioConfig;
    if (fetch(gpioConfig, REG_SYSTEM_INTERRUPT_CONFIG_GPIO)) {
      gpio.wrapped.function = static_cast<enum GpioFunctionality>(gpioConfig & Mask<2, 0>::shifted);

      if (!valid(gpio.wrapped.function)) {
        return ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED;
      } // switch
      Erroneous<uint8_t> polarity;
      if (fetch(polarity, REG_GPIO_HV_MUX_ACTIVE_HIGH)) {
        gpio.wrapped.polarity = getBit<4>(polarity.wrapped) ? INTERRUPTPOLARITY_LOW : INTERRUPTPOLARITY_HIGH;
        VL53L0X_SETDEVICESPECIFICPARAMETER(Pin0GpioFunctionality, gpio.wrapped.function);
        return gpio;
      } else {
        return {polarity.error};
      }
    } else {
      return {gpioConfig.error};
    }
  } // GetGpioConfig


  Error Api::set_threshold(RegSystem index, FixPoint1616_t ThresholdLow) {
    /* no dependency on DeviceMode for Ewok
   * Need to divide by 2 because the FW will apply a x2*/
    uint16_t Threshold16 = ThresholdLow.scaled(17) & Mask<11, 0>::shifted;
    return comm.WrWord(index, Threshold16);
  }

  Erroneous<FixPoint1616_t> Api::get_threshold(RegSystem index) {
    uint16_t Threshold12;
    ErrorAccumulator Error = comm.RdWord(REG_SYSTEM_THRESH_LOW, &Threshold12);
    ERROR_OUT;
    /* Need to multiply by 2 because the FW will apply a x2 */
    return FixPoint1616_t(getBits<11, 0>(Threshold12) << 17, 1);
  }

  Error Api::SetInterruptThresholds(DeviceModes DeviceMode, RangeWindow Threshold) {
    LOG_FUNCTION_START;
    /* no dependency on DeviceMode for Ewok
     * Need to divide by 2 because the FW will apply a x2*/
    Error |= set_threshold(REG_SYSTEM_THRESH_LOW, Threshold.Low);
    ERROR_OUT;
    return set_threshold(REG_SYSTEM_THRESH_HIGH, Threshold.High);
  } // VL53L0X_SetInterruptThresholds

  Error Api::GetInterruptThresholds(DeviceModes DeviceMode, RangeWindow &pThreshold) {
    LOG_FUNCTION_START;
    /* no dependency on DeviceMode for Ewok */
    pThreshold.Low = get_threshold(REG_SYSTEM_THRESH_LOW);//todo: restore checking errors
    pThreshold.High = get_threshold(REG_SYSTEM_THRESH_HIGH);
    return Error;
  } // GetInterruptThresholds

  Erroneous<uint8_t> Api::GetStopCompletedStatus() {
    LOG_FUNCTION_START;

    Erroneous<uint8_t> Byte;
    {//must close before going onto writing 0x91
      auto pager = autoCloser(Private_Pager, 1, 0);
      fetch(Byte, Private_04);
    }

    if (Byte.isOk() && Byte == 0) {
      auto magic = magicWrapper();
      comm.WrByte(REG_SYSRANGE_stopper, PALDevDataGet(StopVariable));//ick: error ignored
    }

    return Byte;
  } // GetStopCompletedStatus

/* Group PAL Interrupt Functions */
  Error Api::ClearInterruptMask(uint32_t InterruptMask) {
    LOG_FUNCTION_START;
    /* clear bit 0 range interrupt, bit 1 error interrupt */
    for (unsigned LoopCount = 3; LoopCount-- > 0;) {
      Error |= comm.WrByte(REG_SYSTEM_INTERRUPT_CLEAR, 1);
      Error |= comm.WrByte(REG_SYSTEM_INTERRUPT_CLEAR, 0);
      uint8_t Byte;
      Error |= comm.RdByte(REG_RESULT_INTERRUPT_STATUS, &Byte);
      ERROR_OUT;
      if (getBits<2, 0>(Byte) == 0) { //ick: elsewhere bits 4,3 are also looked at.
        return ERROR_NONE;
      }
    }
    return ERROR_INTERRUPT_NOT_CLEARED;
  } // ClearInterruptMask

  Erroneous<uint8_t> Api::GetInterruptMaskStatus() {
    LOG_FUNCTION_START;
    Erroneous<uint8_t> mask;

    fetch(mask, REG_RESULT_INTERRUPT_STATUS);
    EXIT_ON(mask)
    if (getBits<4, 3>(mask.wrapped)) {//if either bit? what are each of them?
      return ERROR_RANGE_ERROR;
    }
    mask.wrapped &= Mask<2, 0>::places;
    return mask;
  } // GetInterruptMaskStatus

  Error Api::EnableInterruptMask(uint32_t InterruptMask) {
    VL53L0X_NYI
  }

/* End Group PAL Interrupt Functions */

/* Group SPAD functions */

  Error Api::SetSpadAmbientDamperThreshold(uint16_t SpadAmbientDamperThreshold) {
    LOG_FUNCTION_START;
    return FFwrap(RegSystem(0x40), SpadAmbientDamperThreshold);
  } // VL53L0X_SetSpadAmbientDamperThreshold

  Erroneous<uint16_t> Api::GetSpadAmbientDamperThreshold() {
    LOG_FUNCTION_START;
    return FFread<uint16_t>(RegSystem(0x40));
  } // GetSpadAmbientDamperThreshold

  Error Api::SetSpadAmbientDamperFactor(uint16_t SpadAmbientDamperFactor) {
    LOG_FUNCTION_START;
    return FFwrap(RegSystem(0x42), uint8_t(SpadAmbientDamperFactor));
  } // VL53L0X_SetSpadAmbientDamperFactor

  Erroneous<uint8_t> Api::GetSpadAmbientDamperFactor() {
    LOG_FUNCTION_START;
    return FFread<uint8_t>(RegSystem(0x42));
  } // GetSpadAmbientDamperFactor

/* END Group SPAD functions */

/*****************************************************************************
* Internal functions
*****************************************************************************/

  Error Api::perform_ref_spad_management(SpadCount &ref) {

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
    Data.SpadData.RefSpadEnables.clear();

    {
      SysPopper Error = push(0xFF, 0x01, 0x00);
      Error |= comm.WrByte(REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
      ERROR_OUT;
      Error |= comm.WrByte(REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
      ERROR_OUT;
    }

    ErrorAccumulator Error = comm.WrByte(REG_GLOBAL_CONFIG_REF_EN_START_SELECT, startSelect.absolute());
    ERROR_OUT;
    Error = comm.WrByte(REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0);
    ERROR_OUT;
    /* Perform ref calibration */
    auto p = perform_ref_calibration(false);//ick: PhaseCal value then ignored. ditto for VhvSettings
    if (!p.isOk()) {
      return p.error;
    }
    /* Enable Minimum NON-APERTURE Spads */
    SpadArray::Index currentSpadIndex = 0;

    SpadCount sc;
    sc.isAperture = false;
    sc.quantity = minimumSpadCount;

    auto lastSpadIndex = enable_ref_spads(sc, Data.SpadData.RefGoodSpadMap, Data.SpadData.RefSpadEnables, currentSpadIndex);
    ERROR_ON(lastSpadIndex);
    currentSpadIndex = lastSpadIndex.wrapped;
    Erroneous<uint16_t> peakSignalRateRef = perform_ref_signal_measurement();
    ERROR_OUT;

    if (peakSignalRateRef > targetRefRate) { /* Signal rate measurement too high, switch to APERTURE SPADs */
      Data.SpadData.RefSpadEnables.clear();

      /* Increment to the first APERTURE spad */
      while (currentSpadIndex.isValid() && !(startSelect + currentSpadIndex).is_aperture()) {
        ++currentSpadIndex;
      }

      sc.isAperture = true;
      sc.quantity = minimumSpadCount;

      lastSpadIndex = enable_ref_spads(sc, Data.SpadData.RefGoodSpadMap, Data.SpadData.RefSpadEnables, currentSpadIndex);

      if (Error == ERROR_NONE) {
        currentSpadIndex = lastSpadIndex;
        peakSignalRateRef = perform_ref_signal_measurement();

        if ((Error == ERROR_NONE) &&
            (peakSignalRateRef > targetRefRate)) {
          /* Signal rate still too high after
           * setting the minimum number of
           * APERTURE spads. Can do no more
           * therefore set the min number of
           * aperture spads as the result.
           */
          sc.isAperture = true;
          sc.quantity = minimumSpadCount;
        }
      }
    } else {
      ref.isAperture = false;//but we ERROR_OUT so why this lingering assign? changed to ref object instead of internal one
      ERROR_OUT;
    }
    if (peakSignalRateRef < targetRefRate) {
/* At this point, the minimum number of either aperture
 * or non-aperture spads have been set. Proceed to add
 * spads and perform measurements until the target
 * reference is reached.
 */
//      sc.isAperture = needAptSpads;
      sc.quantity = minimumSpadCount;//perhaps superfluous

      SpadArray lastSpadArray = Data.SpadData.RefSpadEnables;

      uint32_t lastSignalRateDiff = abs(peakSignalRateRef.wrapped - targetRefRate);
      uint8_t complete = 0;
      while (!complete) {
        SpadArray::Index nextGoodSpad = get_next_good_spad(Data.SpadData.RefGoodSpadMap, currentSpadIndex);
        if (!nextGoodSpad.isValid()) {
          return ERROR_REF_SPAD_INIT;
        }
        ++sc.quantity;
/* Cannot combine Aperture and Non-Aperture spads, so
 * ensure the current spad is of the correct type.
 */
        if ((startSelect + nextGoodSpad).is_aperture() != sc.isAperture) {
          return ERROR_REF_SPAD_INIT;
        }

        currentSpadIndex = nextGoodSpad;
        Data.SpadData.RefSpadEnables.enable(currentSpadIndex);
        ++currentSpadIndex;//post ++ not coded
/* Proceed to apply the additional spad and
 * perform measurement. */
        Error = set_ref_spad_map(Data.SpadData.RefSpadEnables);

        ERROR_OUT;
        peakSignalRateRef = perform_ref_signal_measurement();
        ERROR_ON(peakSignalRateRef);
        uint32_t signalRateDiff = abs(peakSignalRateRef.wrapped - targetRefRate);

        if (peakSignalRateRef > targetRefRate) { /* Select the spad map that provides the measurement closest to the target rate, either above or below it. */
          if (signalRateDiff > lastSignalRateDiff) { /* Previous spad map produced a closer measurement, so choose this. */
            Error = set_ref_spad_map(lastSpadArray);
            Data.SpadData.RefSpadEnables = lastSpadArray;
            --sc.quantity;
          }
          complete = 1;
        } else { /* Continue to add spads */
          lastSignalRateDiff = signalRateDiff;
          lastSpadArray = Data.SpadData.RefSpadEnables;
        }
      } /* while */
    }

    ERROR_OUT;

    VL53L0X_SETDEVICESPECIFICPARAMETER(RefSpadsInitialised, 1);
    VL53L0X_SETDEVICESPECIFICPARAMETER(ReferenceSpad, sc);

    ref = sc;
    return ERROR_NONE;
  }

  Erroneous<DeviceParameters_t> Api::GetDeviceParameters() {
    LOG_FUNCTION_START;
    DeviceParameters_t wad;

    auto DeviceMode = GetDeviceMode();
    if (isValid(DeviceMode)) {
      wad.DeviceMode = DeviceMode;
    }

    Erroneous<uint32_t> arf = GetInterMeasurementPeriodMilliSeconds();
//todo:    EXIT_ON(arf)
    wad.InterMeasurementPeriodMilliSeconds = arf;

    wad.XTalkCompensationEnable = false;//ick: why don't we use the actual enable?
    auto arf2 = GetXTalkCompensationRateMegaCps();
    ERROR_OUT;
    wad.XTalkCompensationRateMegaCps = arf2;

    Erroneous<decltype(wad.RangeOffsetMicroMeters)> romm = GetOffsetCalibrationDataMicroMeter();
    if (!romm.isOk()) {
      return LOG_ERROR(romm.error);
    }
    wad.RangeOffsetMicroMeters = romm;

    for (unsigned i = 0; i < CHECKENABLE_NUMBER_OF_CHECKS; ++i) {
      /* get first the values, then the enables.
       * GetLimitCheckValue will modify the enable flags
       */
      auto ckvalue = GetLimitCheckValue(static_cast<CheckEnable>(i));
      if (~ckvalue) {
        return {ckvalue.error};
      }
      wad.LimitChecksValue[i] = ckvalue;
      Erroneous<bool> arf = GetLimitCheckEnable(static_cast<CheckEnable>(i));
      if (arf.isOk()) {
        wad.LimitChecksEnable[i] = arf;
      } else {
        return LOG_ERROR(arf.error);
      }
    }
    auto wcenable = GetWrapAroundCheckEnable();
    if (~wcenable) {
      return {wcenable.error};
    }
    wad.WrapAroundCheckEnable = wcenable;
    auto timbudget = GetMeasurementTimingBudgetMicroSeconds();
    if (~timbudget) {
      return {timbudget.error};
    }
    wad.MeasurementTimingBudgetMicroSeconds = timbudget;
    return ERROR_NONE;
  }
}//end namespace
