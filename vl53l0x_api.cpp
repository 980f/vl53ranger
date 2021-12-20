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

#define  VL53L0X_NYI   return LOG_ERROR(ERROR_NOT_IMPLEMENTED);

#ifdef VL53L0X_LOG_ENABLE
#define trace_print(level, ...)   trace_print_module_function(TRACE_MODULE_API, level, TRACE_FUNCTION_NONE, ## __VA_ARGS__)
#endif

namespace VL53L0X {

#include "versioninfo.h"

  const Version_t Api::ImplementationVersion {VL53L0X_IMPLEMENTATION_VER_REVISION, {VL53L0X_IMPLEMENTATION_VER_MAJOR, VL53L0X_IMPLEMENTATION_VER_MINOR}, VL53L0X_IMPLEMENTATION_VER_SUB};

  const Version_t Api::PalSpecVersion {VL53L0X_SPECIFICATION_VER_REVISION, VL53L0X_SPECIFICATION_VER_MAJOR, VL53L0X_SPECIFICATION_VER_MINOR, VL53L0X_SPECIFICATION_VER_SUB};

/* Group PAL General Functions */


  Erroneous<bool> Api::measurement_poll_for_completion() {
    LOG_FUNCTION_START;

    for (unsigned LoopNb = VL53L0X_DEFAULT_MAX_LOOP; LoopNb-- > 0;) {
      Erroneous<bool> value;
      value = GetMeasurementDataReady();
      if (value.isOk() && value == 1) {//980f: reversed legacy order of testing, test valid before testing value.
        return value;
      }
      PollingDelay();//delay is on comm so that it can use platform specific technique
    }
    return {false, ERROR_TIME_OUT};//was init to 'timeout'
  } // VL53L0X_measurement_poll_for_completion


  Error Api::check_part_used(uint8_t &Revision, DeviceInfo_t &pDeviceInfo) {
    LOG_FUNCTION_START;
    Error |= get_info_from_device(2);

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

  Error Api::SetGroupParamHold(uint8_t GroupParamHold) {
    VL53L0X_NYI
  }

  Error Api::GetUpperLimitMilliMeter(uint16_t *pUpperLimitMilliMeter) {
    VL53L0X_NYI
  }

  Erroneous<FixPoint1616_t> Api::GetTotalSignalRate() {
    LOG_FUNCTION_START;
    RangingMeasurementData_t LastRangeDataBuffer = PALDevDataGet(LastRangeMeasure);
    return get_total_signal_rate(LastRangeDataBuffer);
  } // GetTotalSignalRate

/* End Group PAL General Functions */

/* Group PAL Init Functions */
  Error Api::SetDeviceAddress(uint8_t DeviceAddress) {
    LOG_FUNCTION_START;
    return comm.WrByte(REG_I2C_SLAVE_DEVICE_ADDRESS, DeviceAddress / 2);
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
    PALDevDataSet(DmaxCalRangeMilliMeter, 400);
    PALDevDataSet(DmaxCalSignalRateRtnMegaCps, 1.42f);//(FixPoint1616_t) ((0x00016B85)));             /* 1.42 No Cover Glass*/

    /* Set Default static parameters
     * set first temporary values 9.44MHz * 65536 = 618660 */
    VL53L0X_SETDEVICESPECIFICPARAMETER(OscFrequencyMHz, 9.44f);//618660);

    /* Set Default XTalkCompensationRateMegaCps to 0  */
    VL53L0X_SETPARAMETERFIELD(XTalkCompensationRateMegaCps, 0.0f);

    /* Get default parameters */
    DeviceParameters_t CurrentParameters;

    Error |= GetDeviceParameters(CurrentParameters);

    if (~Error) {
      /* initialize PAL values */
      CurrentParameters.DeviceMode = DEVICEMODE_SINGLE_RANGING;
      CurrentParameters.HistogramMode = HISTOGRAMMODE_DISABLED;
      PALDevDataSet(CurrentParameters, CurrentParameters);
    }

    /* Sigma estimator variable */
    PALDevDataSet(SigmaEst.RefArray, 100);
    PALDevDataSet(SigmaEst.EffPulseWidth, 900);
    PALDevDataSet(SigmaEst.EffAmbWidth, 500);
    PALDevDataSet(targetRefRate, 20.0f); /* 20 MCPS in 9:7 format was 0x0A00 */

    /* Use internal default settings */
    PALDevDataSet(UseInternalTuningSettings, 1);

    {
      auto magic = magicWrapper();
      Error |= comm.RdByte(0x91, &PALDevDataGet(StopVariable));
    }
    ERROR_OUT;
    /* Enable all check */
    for (int i = 0; i < CHECKENABLE_NUMBER_OF_CHECKS; i++) {
      Error |= SetLimitCheckEnable(i, 1);
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

    PALDevDataSet(SequenceConfig, 0xFF);
    Error |= comm.WrByte(REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);
    /* Set PAL state to tell that we are waiting for call to StaticInit */
    PALDevDataSet(PalState, STATE_WAIT_STATICINIT);

    VL53L0X_SETDEVICESPECIFICPARAMETER(RefSpadsInitialised, false);
    return Error;
  } // VL53L0X_DataInit

  Error Api::SetTuningSettingBuffer(uint8_t *pTuningSettingBuffer, bool UseInternalTuningSettings) {
    LOG_FUNCTION_START;
    if (UseInternalTuningSettings) { /* Force use internal settings */
      PALDevDataSet(UseInternalTuningSettings, 1);
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

  uint8_t *Api::GetTuningSettingBuffer() {
    if (PALDevDataGet(UseInternalTuningSettings)) {
      return PALDevDataGet(pTuningSettingsPointer);
    } else {//# don't use ternary, so that we may breakpoint here.
      return nullptr;
    }
  } // GetTuningSettingBuffer

  Error Api::StaticInit() {
    DeviceParameters_t CurrentParameters;
    const uint8_t *pTuningSettingBuffer;
    uint16_t tempword = 0;
    uint8_t tempbyte = 0;
    uint8_t UseInternalTuningSettings = 0;
    uint32_t count = 0;
    bool isApertureSpads = 0;
    unsigned refSpadCount = 0;
    uint8_t ApertureSpads = 0;
    uint8_t vcselPulsePeriodPCLK;
    FixPoint1616_t seqTimeoutMilliSecs;

    LOG_FUNCTION_START;

    Error |= get_info_from_device(1);

    /* set the ref spad from NVM */
    count = (uint32_t) VL53L0X_GETDEVICESPECIFICPARAMETER(ReferenceSpadCount);
    ApertureSpads = VL53L0X_GETDEVICESPECIFICPARAMETER(ReferenceSpadType);

    /* NVM value invalid
     * two known types, 1 and 0, '1' has a max 32 spads, '0' has a max of 12
     * */
    if ((ApertureSpads > 1) || ((ApertureSpads == 1) && (count > 32)) || ((ApertureSpads == 0) && (count > 12))) {
      Error |= perform_ref_spad_management(refSpadCount, isApertureSpads);
    } else {
      Error |= set_reference_spads(count, ApertureSpads);
    }

    /* Initialize tuning settings buffer to prevent compiler warning. */
    pTuningSettingBuffer = DefaultTuningSettings;
    ERROR_OUT;
    UseInternalTuningSettings = PALDevDataGet(UseInternalTuningSettings);

    if (UseInternalTuningSettings == 0) {
      pTuningSettingBuffer = PALDevDataGet(pTuningSettingsPointer);
    } else {
      pTuningSettingBuffer = DefaultTuningSettings;
    }
    ERROR_OUT;
    Error |= load_tuning_settings(pTuningSettingBuffer);


    /* Set interrupt config to new sample ready */
    ERROR_OUT;
    Error |= SetGpioConfig(0, DEVICEMODE_SINGLE_RANGING, GPIOFUNCTIONALITY_NEW_MEASURE_READY, INTERRUPTPOLARITY_LOW);
    ERROR_OUT;
    Erroneous<uint16_t> fix412 = FFread<uint16_t>(RegSystem(0x84));
    ERROR_OUT;
    VL53L0X_SETDEVICESPECIFICPARAMETER(OscFrequencyMHz, VL53L0X_FIXPOINT412TOFIXPOINT1616(tempword));


    /* After static init, some device parameters may be changed,
     * so update them */
    ERROR_OUT;
    Error |= GetDeviceParameters(CurrentParameters);
    ERROR_OUT;
    Error |= GetFractionEnable(&tempbyte);
    ERROR_OUT;
    PALDevDataSet(RangeFractionalEnable, tempbyte);

    ERROR_OUT;
    PALDevDataSet(CurrentParameters, CurrentParameters);


    /* read the sequence config and save it */
    Error |= comm.RdByte(REG_SYSTEM_SEQUENCE_CONFIG, &tempbyte);
    ERROR_OUT;
    PALDevDataSet(SequenceConfig, tempbyte);


    /* Disable MSRC and TCC by default */
    Error |= SetSequenceStepEnable(SEQUENCESTEP_TCC, false);
    ERROR_OUT;
    Error |= SetSequenceStepEnable(SEQUENCESTEP_MSRC, false);
    ERROR_OUT;
    /* Set PAL State to standby */

    PALDevDataSet(PalState, STATE_IDLE);
    ERROR_OUT;

    /* Store pre-range vcsel period */
    Error |= GetVcselPulsePeriod(VCSEL_PERIOD_PRE_RANGE, &vcselPulsePeriodPCLK);
    ERROR_OUT;
    VL53L0X_SETDEVICESPECIFICPARAMETER(PreRange.VcselPulsePeriod, vcselPulsePeriodPCLK);
    ERROR_OUT;
    /* Store final-range vcsel period */

    Error |= GetVcselPulsePeriod(VCSEL_PERIOD_FINAL_RANGE, &vcselPulsePeriodPCLK);
    ERROR_OUT;
    VL53L0X_SETDEVICESPECIFICPARAMETER(FinalRange.VcselPulsePeriod, vcselPulsePeriodPCLK);
    ERROR_OUT;
    /* Store pre-range timeout */

    Error |= GetSequenceStepTimeout(SEQUENCESTEP_PRE_RANGE, &seqTimeoutMilliSecs);
    ERROR_OUT;
    VL53L0X_SETDEVICESPECIFICPARAMETER(PreRange.TimeoutMicroSecs, seqTimeoutMilliSecs);
    ERROR_OUT;
    /* Store final-range timeout */
    Error |= GetSequenceStepTimeout(SEQUENCESTEP_FINAL_RANGE, &seqTimeoutMilliSecs);
    ERROR_OUT;
    VL53L0X_SETDEVICESPECIFICPARAMETER(FinalRange.TimeoutMicroSecs, seqTimeoutMilliSecs);

    return Error;
  } // VL53L0X_StaticInit

  Error WaitDeviceBooted() {
    VL53L0X_NYI
  }

  Error Api::waitOnResetIndicator(bool disappear) {
    for (uint8_t Byte = disappear ? ~0 : 0; disappear == (Byte != 0);) {
      if (comm.RdByte(REG_IDENTIFICATION_MODEL_ID, &Byte)) {
        return ERROR_CONTROL_INTERFACE;
      }
    }
    return ERROR_NONE;
  }

  Error Api::ResetDevice() {
    LOG_FUNCTION_START;

    /* Set reset bit */
    Error |= comm.WrByte(REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x00);

    /* Wait for some time */
    if (~Error) {
      Error |= waitOnResetIndicator(true);//ick: former code could hang on comm error, this quits
    }
    /* Release reset */
    Error |= comm.WrByte(REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x01);//ick: ignores error

    /* Wait until correct boot-up of the device */
    if (~Error) {
      Error |= waitOnResetIndicator(false);//ick: former code could hang on comm error, this quits
    }

    /* Set PAL State to State_POWERDOWN */
    if (~Error) {
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

    for (int i = 0; i < CHECKENABLE_NUMBER_OF_CHECKS; i++) {
      Error |= SetLimitCheckEnable(i, pDeviceParameters.LimitChecksEnable[i]);
      ERROR_OUT;
      Error |= SetLimitCheckValue(i, pDeviceParameters.LimitChecksValue[i]);
      ERROR_OUT;
    }
    Error |= SetWrapAroundCheckEnable(pDeviceParameters.WrapAroundCheckEnable);
    ERROR_OUT;
    Error |= SetMeasurementTimingBudgetMicroSeconds(pDeviceParameters.MeasurementTimingBudgetMicroSeconds);

    return Error;
  } // VL53L0X_SetDeviceParameters

  Error Api::GetDeviceParameters(DeviceParameters_t &pDeviceParameters) {

    LOG_FUNCTION_START;
    Error |= GetDeviceMode(pDeviceParameters.DeviceMode);
    ERROR_OUT;

    Error |= GetInterMeasurementPeriodMilliSeconds(pDeviceParameters.InterMeasurementPeriodMilliSeconds);
    ERROR_OUT;
    pDeviceParameters.XTalkCompensationEnable = 0;
    ERROR_OUT;
    Error |= GetXTalkCompensationRateMegaCps(&(pDeviceParameters.XTalkCompensationRateMegaCps));
    ERROR_OUT;
    Erroneous<decltype(pDeviceParameters.RangeOffsetMicroMeters)> romm = GetOffsetCalibrationDataMicroMeter();
    if (!romm.isOk()) {
      return LOG_ERROR(romm.error);
    }
    pDeviceParameters.RangeOffsetMicroMeters = romm;

    for (unsigned i = 0; i < CHECKENABLE_NUMBER_OF_CHECKS; i++) {
      /* get first the values, then the enables.
       * GetLimitCheckValue will modify the enable
       * flags
       */

      Error |= GetLimitCheckValue(i, &(pDeviceParameters.LimitChecksValue[i]));
      ERROR_OUT;
      Erroneous<bool> arf = GetLimitCheckEnable(static_cast<CheckEnable>(i));
      if (arf.isOk()) {
        pDeviceParameters.LimitChecksEnable[i] = arf;
      } else {
        return LOG_ERROR(arf.error);
      }
    }
    Error |= GetWrapAroundCheckEnable(pDeviceParameters.WrapAroundCheckEnable);
    ERROR_OUT;
    Error |= GetMeasurementTimingBudgetMicroSeconds(pDeviceParameters.MeasurementTimingBudgetMicroSeconds);

    return Error;
  } // GetDeviceParameters

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

    LOG_FUNCTION_START; Error(" %u", Enable);

    Error |= comm.WrByte(REG_SYSTEM_RANGE_CONFIG, Enable);

    if (Error == ERROR_NONE) {
      PALDevDataSet(RangeFractionalEnable, Enable);
    }

    return Error;
  } // VL53L0X_SetRangeFractionEnable

  Error Api::GetFractionEnable(uint8_t *pEnabled) {

    LOG_FUNCTION_START;

    Error = comm.RdByte(REG_SYSTEM_RANGE_CONFIG, pEnabled);

    if (Error == ERROR_NONE) {
      *pEnabled = (*pEnabled & 1);
    }

    return Error;
  } // GetFractionEnable

  Error Api::SetHistogramMode(HistogramModes HistogramMode) {
    VL53L0X_NYI
  }

  Error Api::GetHistogramMode(HistogramModes *pHistogramMode) {
    VL53L0X_NYI
  }

  Error Api::SetMeasurementTimingBudgetMicroSeconds(uint32_t MeasurementTimingBudgetMicroSeconds) {
    return set_measurement_timing_budget_micro_seconds(MeasurementTimingBudgetMicroSeconds);
  } // SetMeasurementTimingBudgetMicroSeconds

  Error Api::GetMeasurementTimingBudgetMicroSeconds(uint32_t *pMeasurementTimingBudgetMicroSeconds) {
    LOG_ERROR(Get_measurement_timing_budget_micro_seconds(pMeasurementTimingBudgetMicroSeconds));
  }

  Error Api::SetVcselPulsePeriod(VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriodPCLK) {
    return set_vcsel_pulse_period(VcselPeriodType, VCSELPulsePeriodPCLK);
  }

  Error Api::GetVcselPulsePeriod(VcselPeriod VcselPeriodType, uint8_t *pVCSELPulsePeriodPCLK) {
    LOG_FUNCTION_START;
    return Get_vcsel_pulse_period(VcselPeriodType, pVCSELPulsePeriodPCLK);
  }

  Error Api::SetSequenceStepEnable(SequenceStepId SequenceStepId, uint8_t SequenceStepEnabled) {


    uint32_t MeasurementTimingBudgetMicroSeconds;
    LOG_FUNCTION_START;
    uint8_t SequenceConfig = 0;
    Error |= comm.RdByte(REG_SYSTEM_SEQUENCE_CONFIG, &SequenceConfig);


    uint8_t SequenceConfigNew = SequenceConfig;

    if (Error == ERROR_NONE) {
      if (SequenceStepEnabled == 1) {
        /* Enable requested sequence step */
        switch (SequenceStepId) {
          case SEQUENCESTEP_TCC:
            SequenceConfigNew |= 0x10;
            break;
          case SEQUENCESTEP_DSS:
            SequenceConfigNew |= 0x28;//BUG: 0x20 elsewhere , not 0x28
            break;
          case SEQUENCESTEP_MSRC:
            SequenceConfigNew |= 0x04;
            break;
          case SEQUENCESTEP_PRE_RANGE:
            SequenceConfigNew |= 0x40;
            break;
          case SEQUENCESTEP_FINAL_RANGE:
            SequenceConfigNew |= 0x80;
            break;
          default:
            Error = ERROR_INVALID_PARAMS;
        } // switch
      } else {
        /* Disable requested sequence step
         */
        switch (SequenceStepId) {
          case SEQUENCESTEP_TCC:
            SequenceConfigNew &= ~0x10;
            break;
          case SEQUENCESTEP_DSS:
            SequenceConfigNew &= ~0x28;
            break;
          case SEQUENCESTEP_MSRC:
            SequenceConfigNew &= ~0x04;
            break;
          case SEQUENCESTEP_PRE_RANGE:
            SequenceConfigNew &= ~0x40;
            break;
          case SEQUENCESTEP_FINAL_RANGE:
            SequenceConfigNew &= ~0x80;
            break;
          default:
            Error = ERROR_INVALID_PARAMS;
        } // switch
      }
    }

    if (SequenceConfigNew != SequenceConfig) {
      /* Apply New Setting */
      if (Error == ERROR_NONE) {
        Error = comm.WrByte(REG_SYSTEM_SEQUENCE_CONFIG, SequenceConfigNew);
      }
      if (Error == ERROR_NONE) {
        PALDevDataSet(SequenceConfig, SequenceConfigNew);
      }

      /* Recalculate timing budget */
      if (Error == ERROR_NONE) {
        GetPARAMETERFIELD(MeasurementTimingBudgetMicroSeconds, MeasurementTimingBudgetMicroSeconds);

        SetMeasurementTimingBudgetMicroSeconds(MeasurementTimingBudgetMicroSeconds);
      }
    }

    return Error;
  } // VL53L0X_SetSequenceStepEnable




  void Api::GetNumberOfSequenceSteps(uint8_t *pNumberOfSequenceSteps) {
    *pNumberOfSequenceSteps = SEQUENCESTEP_NUMBER_OF_CHECKS;
  }

  Error Api::GetSequenceStepsInfo(SequenceStepId SequenceStepId, char *pSequenceStepsString) {
    LOG_FUNCTION_START;
    return get_sequence_steps_info(SequenceStepId, pSequenceStepsString);
  } // GetSequenceStepsInfo

  Error Api::SetSequenceStepTimeout(SequenceStepId SequenceStepId, FixPoint1616_t TimeOutMilliSecs) {

    LOG_FUNCTION_START;

    /* Read back the current value in case we need to revert back to this.
     */

    FixPoint1616_t OldTimeOutMicroSeconds;
    Error = get_sequence_step_timeout(SequenceStepId, OldTimeOutMicroSeconds);

    if (Error == ERROR_NONE) {
      auto TimeoutMicroSeconds = TimeOutMilliSecs.millis();
      Error = set_sequence_step_timeout(SequenceStepId, TimeoutMicroSeconds);
    }

    if (Error == ERROR_NONE) {

      uint32_t MeasurementTimingBudgetMicroSeconds=VL53L0X_GETPARAMETERFIELD(MeasurementTimingBudgetMicroSeconds);

      /* At this point we don't know if the requested value is valid,
       *  therefore proceed to update the entire timing budget and
       *  if this fails, revert back to the previous value.
       */
      Error = SetMeasurementTimingBudgetMicroSeconds(MeasurementTimingBudgetMicroSeconds);

      if (Error != ERROR_NONE) {
        ErrorAccumulator Status1 ( set_sequence_step_timeout(SequenceStepId, OldTimeOutMicroSeconds));

        if (Status1 == ERROR_NONE) {
          Status1 = SetMeasurementTimingBudgetMicroSeconds(MeasurementTimingBudgetMicroSeconds);
        }

        return Status1;
      }
    }

    return Error;
  } // VL53L0X_SetSequenceStepTimeout

  Error Api::GetSequenceStepTimeout(SequenceStepId SequenceStepId, FixPoint1616_t *pTimeOutMilliSecs) {
    LOG_FUNCTION_START;

    uint32_t TimeoutMicroSeconds;
    uint32_t WholeNumber_ms = 0;
    uint32_t Fraction_ms = 0;

    Error = get_sequence_step_timeout(SequenceStepId, &TimeoutMicroSeconds);
    if (Error == ERROR_NONE) {
      WholeNumber_ms = TimeoutMicroSeconds / 1000;
      Fraction_ms = TimeoutMicroSeconds - (WholeNumber_ms * 1000);
      *pTimeOutMilliSecs = (WholeNumber_ms << 16) + (((Fraction_ms * 0xffff) + 500) / 1000);
    }

    return Error;
  } // GetSequenceStepTimeout

  Error Api::SetInterMeasurementPeriodMilliSeconds(uint32_t InterMeasurementPeriodMilliSeconds) {
    uint16_t osc_calibrate_val;
    uint32_t IMPeriodMilliSeconds;

    LOG_FUNCTION_START;

    Error = comm.RdWord(REG_OSC_CALIBRATE_VAL, &osc_calibrate_val);

    if (Error == ERROR_NONE) {
      if (osc_calibrate_val != 0) {
        IMPeriodMilliSeconds = InterMeasurementPeriodMilliSeconds * osc_calibrate_val;
      } else {
        IMPeriodMilliSeconds = InterMeasurementPeriodMilliSeconds;
      }
      Error = comm.WrDWord(REG_SYSTEM_INTERMEASUREMENT_PERIOD, IMPeriodMilliSeconds);
    }

    if (Error == ERROR_NONE) {
      VL53L0X_SETPARAMETERFIELD(InterMeasurementPeriodMilliSeconds, InterMeasurementPeriodMilliSeconds);
    }

    return Error;
  } // VL53L0X_SetInterMeasurementPeriodMilliSeconds

  Error Api::GetInterMeasurementPeriodMilliSeconds(uint32_t *pInterMeasurementPeriodMilliSeconds) {

    uint16_t osc_calibrate_val;
    uint32_t IMPeriodMilliSeconds;

    LOG_FUNCTION_START;

    Error = comm.RdWord(REG_OSC_CALIBRATE_VAL, &osc_calibrate_val);

    if (Error == ERROR_NONE) {
      Error = comm.RdDWord(REG_SYSTEM_INTERMEASUREMENT_PERIOD, &IMPeriodMilliSeconds);
    }

    if (Error == ERROR_NONE) {
      if (osc_calibrate_val != 0) {
        *pInterMeasurementPeriodMilliSeconds = IMPeriodMilliSeconds / osc_calibrate_val;
      }
      VL53L0X_SETPARAMETERFIELD(InterMeasurementPeriodMilliSeconds, *pInterMeasurementPeriodMilliSeconds);
    }

    return Error;
  } // GetInterMeasurementPeriodMilliSeconds


  Error Api::GetXTalkCompensationEnable(uint8_t *pXTalkCompensationEnable) {
    uint8_t Temp8;
    LOG_FUNCTION_START;

    GetPARAMETERFIELD(XTalkCompensationEnable, Temp8);
    *pXTalkCompensationEnable = Temp8;

    return Error;
  } // GetXTalkCompensationEnable

  Error Api::SetXTalkCompensationRateMegaCps(FixPoint1616_t XTalkCompensationRateMegaCps) {


    uint16_t LinearityCorrectiveGain;
    uint16_t data;
    LOG_FUNCTION_START;
    uint8_t Temp8=VL53L0X_GETPARAMETERFIELD(XTalkCompensationEnable);
    LinearityCorrectiveGain = PALDevDataGet(LinearityCorrectiveGain);

    if (Temp8 == 0) { /* disabled write only internal value */
      VL53L0X_SETPARAMETERFIELD(XTalkCompensationRateMegaCps, XTalkCompensationRateMegaCps);
    } else {
      /* the following register has a format 3.13 */
      if (LinearityCorrectiveGain == 1000) {
        data = VL53L0X_FIXPOINT1616TOFIXPOINT313(XTalkCompensationRateMegaCps);
      } else {
        data = 0;
      }

      Error = comm.WrWord(REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, data);

      if (Error == ERROR_NONE) {
        VL53L0X_SETPARAMETERFIELD(XTalkCompensationRateMegaCps, XTalkCompensationRateMegaCps);
      }
    }

    return Error;
  } // VL53L0X_SetXTalkCompensationRateMegaCps

  Error Api::GetXTalkCompensationRateMegaCps(FixPoint1616_t *pXTalkCompensationRateMegaCps) {
    Error Error = ERROR_NONE;
    uint16_t Value;
    FixPoint1616_t TempFix1616;

    LOG_FUNCTION_START;

    Error = comm.RdWord(REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, (uint16_t *) &Value);
    if (Error == ERROR_NONE) {
      if (Value == 0) {
        /* the Xtalk is disabled return value from memory */
        GetPARAMETERFIELD(XTalkCompensationRateMegaCps, TempFix1616);
        *pXTalkCompensationRateMegaCps = TempFix1616;
        VL53L0X_SETPARAMETERFIELD(XTalkCompensationEnable, 0);
      } else {
        TempFix1616 = VL53L0X_FIXPOINT313TOFIXPOINT1616(Value);
        *pXTalkCompensationRateMegaCps = TempFix1616;
        VL53L0X_SETPARAMETERFIELD(XTalkCompensationRateMegaCps, TempFix1616);
        VL53L0X_SETPARAMETERFIELD(XTalkCompensationEnable, 1);
      }
    }

    return Error;
  } // GetXTalkCompensationRateMegaCps

  Error Api::SetRefCalibration(uint8_t VhvSettings, uint8_t PhaseCal) {
    Error Error = ERROR_NONE;
    LOG_FUNCTION_START;

    Error = set_ref_calibration(VhvSettings, PhaseCal);

    return Error;
  }

  Error Api::GetRefCalibration(uint8_t *pVhvSettings, uint8_t *pPhaseCal) {
    Error Error = ERROR_NONE;
    LOG_FUNCTION_START;

    Error = Get_ref_calibration(pVhvSettings, pPhaseCal);

    return Error;
  }

/*
 * CHECK LIMIT FUNCTIONS
 */

  CheckEnable Api::GetNumberOfLimitCheck() {
    return CHECKENABLE_NUMBER_OF_CHECKS;
  }

  Error Api::GetLimitCheckInfo(CheckEnable LimitCheckId, char *pLimitCheckString) {
    return Get_limit_check_info(LimitCheckId, pLimitCheckString);
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
//  FixPoint1616_t TempFix1616 = 0;
    bool LimitCheckEnableInt = false;
    bool LimitCheckDisable = false;

    if (LimitCheckEnable == 0) {
      TempFix1616 = 0;
      LimitCheckEnableInt = false
      LimitCheckDisable = true;
    } else {
      VL53L0X_GETARRAYPARAMETERFIELD(LimitChecksValue, LimitCheckId, TempFix1616);
      LimitCheckDisable = 0;
      /* this to be sure to have either 0 or 1 */
      LimitCheckEnableInt = 1;
    }

    switch (LimitCheckId) {
      case CHECKENABLE_SIGMA_FINAL_RANGE:
        /* internal computation: */
        VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksEnable, CHECKENABLE_SIGMA_FINAL_RANGE, LimitCheckEnableInt);
        break;

      case CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
        Error = comm.WrWord(REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, VL53L0X_FIXPOINT1616TOFIXPOINT97(TempFix1616));
        break;

      case CHECKENABLE_SIGNAL_REF_CLIP:
        /* internal computation: */
        VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksEnable, CHECKENABLE_SIGNAL_REF_CLIP, LimitCheckEnableInt);
        break;

      case CHECKENABLE_RANGE_IGNORE_THRESHOLD:
        /* internal computation: */
        VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksEnable, CHECKENABLE_RANGE_IGNORE_THRESHOLD, LimitCheckEnableInt);
        break;

      case CHECKENABLE_SIGNAL_RATE_MSRC:
        Temp8 = (uint8_t) (LimitCheckDisable << 1);
        Error = comm.UpdateByte(REG_MSRC_CONFIG_CONTROL, 0xFE, Temp8);//clear lsb set bit 1
        break;

      case CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
        Temp8 = (uint8_t) (LimitCheckDisable << 4);
        Error = comm.UpdateByte(REG_MSRC_CONFIG_CONTROL, 0xEF, Temp8);
        break;

      default:
        return ERROR_INVALID_PARAMS;
    } // switch


    if (Error == ERROR_NONE) {
      if (LimitCheckEnable == 0) {
        VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksEnable, LimitCheckId, 0);
      } else {
        VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksEnable, LimitCheckId, 1);
      }
    }

    return Error;
  } // VL53L0X_SetLimitCheckEnable


  Error Api::SetLimitCheckValue(uint16_t LimitCheckId, FixPoint1616_t LimitCheckValue) {

    LOG_FUNCTION_START;

    VL53L0X_GETARRAYPARAMETERFIELD(LimitChecksEnable, LimitCheckId, Temp8);

    if (Temp8 == 0) { /* disabled write only internal value */
      VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksValue, LimitCheckId, LimitCheckValue);
    } else {
      switch (LimitCheckId) {
        case CHECKENABLE_SIGMA_FINAL_RANGE:
          /* internal computation: */
          VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksValue, CHECKENABLE_SIGMA_FINAL_RANGE, LimitCheckValue);
          break;

        case CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
          Error = comm.WrWord(REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, VL53L0X_FIXPOINT1616TOFIXPOINT97(LimitCheckValue));
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
          Error = comm.WrWord(REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT, VL53L0X_FIXPOINT1616TOFIXPOINT97(LimitCheckValue));
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

  Erroneous<FixPoint1616_t> Api::GetLimitCheckValue(CheckEnable LimitCheckId) {
    Error Error = ERROR_NONE;
    uint8_t EnableZeroValue = 0;
    uint16_t Temp16;
    FixPoint1616_t TempFix1616;

    LOG_FUNCTION_START;

    switch (LimitCheckId) {

      case CHECKENABLE_SIGMA_FINAL_RANGE:
        /* internal computation: */
        TempFix1616 = VL53L0X_GETARRAYPARAMETERFIELD(LimitChecksValue, CHECKENABLE_SIGMA_FINAL_RANGE);
        EnableZeroValue = 0;
        break;

      case CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
        Error = comm.RdWord(REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, &Temp16);
        if (Error == ERROR_NONE) {
          TempFix1616 = VL53L0X_FIXPOINT97TOFIXPOINT1616(Temp16);
        }
        EnableZeroValue = 1;
        break;

      case CHECKENABLE_SIGNAL_REF_CLIP:
        /* internal computation: */
        VL53L0X_GETARRAYPARAMETERFIELD(LimitChecksValue, CHECKENABLE_SIGNAL_REF_CLIP, TempFix1616);
        EnableZeroValue = 0;
        break;

      case CHECKENABLE_RANGE_IGNORE_THRESHOLD:
        /* internal computation: */
        VL53L0X_GETARRAYPARAMETERFIELD(LimitChecksValue, CHECKENABLE_RANGE_IGNORE_THRESHOLD, TempFix1616);
        EnableZeroValue = 0;
        break;

      case CHECKENABLE_SIGNAL_RATE_MSRC:
      case CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
        Error = comm.RdWord(REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT, &Temp16);
        if (Error == ERROR_NONE) {
          TempFix1616 = VL53L0X_FIXPOINT97TOFIXPOINT1616(Temp16);
        }
        EnableZeroValue = 0;
        break;

      default:
        return ERROR_INVALID_PARAMS;
    } // switch

    if (Error == ERROR_NONE) {
      if (EnableZeroValue == 1) {
        if (TempFix1616 == 0) {
          /* disabled: return value from memory */
          VL53L0X_GETARRAYPARAMETERFIELD(LimitChecksValue, LimitCheckId, TempFix1616);
          *pLimitCheckValue = TempFix1616;
          VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksEnable, LimitCheckId, 0);
        } else {
          *pLimitCheckValue = TempFix1616;
          VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksValue, LimitCheckId, TempFix1616);
          VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksEnable, LimitCheckId, 1);
        }
      } else {
        *pLimitCheckValue = TempFix1616;
      }
    }

    return Error;
  } // GetLimitCheckValue

  Error Api::GetLimitCheckCurrent(uint16_t LimitCheckId, FixPoint1616_t *pLimitCheckCurrent) {

    RangingMeasurementData_t LastRangeDataBuffer;

    LOG_FUNCTION_START;

    if (LimitCheckId >= CHECKENABLE_NUMBER_OF_CHECKS) {
      return ERROR_INVALID_PARAMS;
    }
      switch (LimitCheckId) {
        case CHECKENABLE_SIGMA_FINAL_RANGE:
          /* Need to run a ranging to have the latest values */
          *pLimitCheckCurrent = PALDevDataGet(SigmaEstimate);
          break;

        case CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
          /* Need to run a ranging to have the latest values */
          LastRangeDataBuffer = PALDevDataGet(LastRangeMeasure);
          *pLimitCheckCurrent = LastRangeDataBuffer.SignalRateRtnMegaCps;
          break;

        case CHECKENABLE_SIGNAL_REF_CLIP:
          /* Need to run a ranging to have the latest values */
          *pLimitCheckCurrent = PALDevDataGet(LastSignalRefMcps);
          break;

        case CHECKENABLE_RANGE_IGNORE_THRESHOLD:
          /* Need to run a ranging to have the latest values */
          LastRangeDataBuffer = PALDevDataGet(LastRangeMeasure);
          *pLimitCheckCurrent = LastRangeDataBuffer.SignalRateRtnMegaCps;
          break;

        case CHECKENABLE_SIGNAL_RATE_MSRC:
          /* Need to run a ranging to have the latest values */
          LastRangeDataBuffer = PALDevDataGet(LastRangeMeasure);
          *pLimitCheckCurrent = LastRangeDataBuffer.SignalRateRtnMegaCps;
          break;

        case CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
          /* Need to run a ranging to have the latest values */
          LastRangeDataBuffer = PALDevDataGet(LastRangeMeasure);
          *pLimitCheckCurrent = LastRangeDataBuffer.SignalRateRtnMegaCps;
          break;

        default:
          return ERROR_INVALID_PARAMS;
      } // switch


    return Error;
  } // GetLimitCheckCurrent

/*
 * WRAPAROUND Check
 */
  Error Api::SetWrapAroundCheckEnable(uint8_t WrapAroundCheckEnable) {
    Error Error = ERROR_NONE;
    uint8_t Byte;
    uint8_t WrapAroundCheckEnableInt;

    LOG_FUNCTION_START;

    Error = comm.RdByte(REG_SYSTEM_SEQUENCE_CONFIG, &Byte);
    if (WrapAroundCheckEnable == 0) {
      /* Disable wraparound */
      Byte = Byte & 0x7F;
      WrapAroundCheckEnableInt = 0;
    } else {
      /*Enable wraparound */
      Byte = Byte | 0x80;
      WrapAroundCheckEnableInt = 1;
    }

    Error = comm.WrByte(REG_SYSTEM_SEQUENCE_CONFIG, Byte);

    if (Error == ERROR_NONE) {
      PALDevDataSet(SequenceConfig, Byte);
      VL53L0X_SETPARAMETERFIELD(WrapAroundCheckEnable, WrapAroundCheckEnableInt);
    }

    return Error;
  } // VL53L0X_SetWrapAroundCheckEnable

  Error Api::GetWrapAroundCheckEnable(uint8_t *pWrapAroundCheckEnable) {
    Error Error = ERROR_NONE;
    uint8_t data;

    LOG_FUNCTION_START;

    Error = comm.RdByte(REG_SYSTEM_SEQUENCE_CONFIG, &data);
    if (Error == ERROR_NONE) {
      PALDevDataSet(SequenceConfig, data);
      if (data & (0x01 << 7)) {
        *pWrapAroundCheckEnable = 0x01;
      } else {
        *pWrapAroundCheckEnable = 0x00;
      }
    }
    if (Error == ERROR_NONE) {
      VL53L0X_SETPARAMETERFIELD(WrapAroundCheckEnable, *pWrapAroundCheckEnable);
    }

    return Error;
  } // GetWrapAroundCheckEnable

  Error Api::SetDmaxCalParameters(uint16_t RangeMilliMeter, FixPoint1616_t SignalRateRtnMegaCps) {
    FixPoint1616_t SignalRateRtnMegaCpsTemp = 0;

    LOG_FUNCTION_START;

    /* Check if one of input parameter is zero, in that case the
     * value are get from NVM */
    if ((RangeMilliMeter == 0) || (SignalRateRtnMegaCps == 0)) {
      /* NVM parameters */
      /* Run Get_info_from_device wit option 4 to get
       * signal rate at 400 mm if the value have been already
       * get this function will return with no access to device */
      Get_info_from_device(4);

      SignalRateRtnMegaCpsTemp = GetDEVICESPECIFICPARAMETER(SignalRateMeasFixed400mm);

      PALDevDataSet(DmaxCalRangeMilliMeter, 400);
      PALDevDataSet(DmaxCalSignalRateRtnMegaCps, SignalRateRtnMegaCpsTemp);
    } else {
      /* User parameters */
      PALDevDataSet(DmaxCalRangeMilliMeter, RangeMilliMeter);
      PALDevDataSet(DmaxCalSignalRateRtnMegaCps, SignalRateRtnMegaCps);
    }

    return Error;
  } // VL53L0X_SetDmaxCalParameters

  void Api::GetDmaxCalParameters(uint16_t *pRangeMilliMeter, FixPoint1616_t *pSignalRateRtnMegaCps) {

    *pRangeMilliMeter = PALDevDataGet(DmaxCalRangeMilliMeter);
    *pSignalRateRtnMegaCps = PALDevDataGet(DmaxCalSignalRateRtnMegaCps);

  } // GetDmaxCalParameters

/* End Group PAL Parameters Functions */

/* Group PAL Measurement Functions */
  Error Api::PerformSingleMeasurement() {
    DeviceModes DeviceMode;

    LOG_FUNCTION_START;

    /* Get Current DeviceMode */
    Error |= GetDeviceMode(&DeviceMode);

    /* Start immediately to run a single ranging measurement in case of
     * single ranging or single histogram */
    if (Error == ERROR_NONE &&
        DeviceMode == DEVICEMODE_SINGLE_RANGING) {
      Error |= StartMeasurement();
    }

    if (Error == ERROR_NONE) {
      Error = measurement_poll_for_completion();
    }

    /* Change PAL State in case of single ranging or single histogram */
    if (Error == ERROR_NONE &&
        DeviceMode == DEVICEMODE_SINGLE_RANGING) {
      PALDevDataSet(PalState, STATE_IDLE);
    }

    return Error;
  } // VL53L0X_PerformSingleMeasurement

  Error Api::PerformSingleHistogramMeasurement(HistogramMeasurementData_t *pHistogramMeasurementData) {
    VL53L0X_NYI
  }

  Error Api::PerformRefCalibration(uint8_t *pVhvSettings, uint8_t *pPhaseCal) {
    LOG_FUNCTION_START;

    return perform_ref_calibration(pVhvSettings, pPhaseCal, 1);

  }

  Error Api::PerformXTalkMeasurement(uint32_t TimeoutMs, FixPoint1616_t *pXtalkPerSpad, uint8_t *pAmbientTooHigh) {
     VL53L0X_NYI
  }

  Error Api::PerformXTalkCalibration(FixPoint1616_t XTalkCalDistance, FixPoint1616_t *pXTalkCompensationRateMegaCps) {
    LOG_FUNCTION_START;

    return perform_xtalk_calibration(XTalkCalDistance, pXTalkCompensationRateMegaCps);

  }

  Error Api::PerformOffsetCalibration(FixPoint1616_t CalDistanceMilliMeter, int32_t *pOffsetMicroMeter) {
    LOG_FUNCTION_START;
return perform_offset_calibration(CalDistanceMilliMeter, pOffsetMicroMeter);

  }

  Error Api::CheckAndLoadInterruptSettings(uint8_t StartNotStopFlag) {
    uint8_t InterruptConfig;
    FixPoint1616_t ThresholdLow;
    FixPoint1616_t ThresholdHigh;
    Error Error = ERROR_NONE;

    InterruptConfig = GetDEVICESPECIFICPARAMETER(Pin0GpioFunctionality);

    if ((InterruptConfig == GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW) ||
        (InterruptConfig == GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH) ||
        (InterruptConfig == GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT)) {

      Error = GetInterruptThresholds(DEVICEMODE_CONTINUOUS_RANGING, &ThresholdLow, &ThresholdHigh);

      if (((ThresholdLow > 255 * 65536) || (ThresholdHigh > 255 * 65536)) && (Error == ERROR_NONE)) {

        if (StartNotStopFlag != 0) {
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
    DeviceModes DeviceMode;
    uint8_t Byte;
    uint8_t StartStopByte = REG_SYSRANGE_MODE_START_STOP;
    uint32_t LoopNb;
    LOG_FUNCTION_START;

    /* Get Current DeviceMode */
    GetDeviceMode(&DeviceMode);

    Error = comm.WrByte(0x80, 0x01);
    Error = comm.WrByte(0xFF, 0x01);
    Error = comm.WrByte(0x00, 0x00);
    Error = comm.WrByte(0x91, PALDevDataGet(StopVariable));
    Error = comm.WrByte(0x00, 0x01);
    Error = comm.WrByte(0xFF, 0x00);
    Error = comm.WrByte(0x80, 0x00);

    switch (DeviceMode) {
      case DEVICEMODE_SINGLE_RANGING:
        Error = comm.WrByte(REG_SYSRANGE_START, 0x01);

        Byte = StartStopByte;
        if (Error == ERROR_NONE) {
          /* Wait until start bit has been cleared */
          LoopNb = 0;
          do {
            if (LoopNb > 0) {
              Error = comm.RdByte(REG_SYSRANGE_START, &Byte);
            }
            LoopNb = LoopNb + 1;
          } while (((Byte & StartStopByte) == StartStopByte) && (Error == ERROR_NONE) && (LoopNb < VL53L0X_DEFAULT_MAX_LOOP));

          if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Error = ERROR_TIME_OUT;
          }
        }
        break;
      case DEVICEMODE_CONTINUOUS_RANGING:
        /* Back-to-back mode */

        /* Check if need to apply interrupt settings */
        if (Error == ERROR_NONE) {
          Error = Api::CheckAndLoadInterruptSettings(1);
        }

        Error = comm.WrByte(REG_SYSRANGE_START, REG_SYSRANGE_MODE_BACKTOBACK);
        if (Error == ERROR_NONE) {
          /* Set PAL State to Running */
          PALDevDataSet(PalState, STATE_RUNNING);
        }
        break;
      case DEVICEMODE_CONTINUOUS_TIMED_RANGING:
        /* Continuous mode
         * Check if need to apply interrupt settings*/
        if (Error == ERROR_NONE) {
          Error = Api::CheckAndLoadInterruptSettings(1);
        }

        Error = comm.WrByte(REG_SYSRANGE_START, REG_SYSRANGE_MODE_TIMED);

        if (Error == ERROR_NONE) {
          /* Set PAL State to Running */
          PALDevDataSet(PalState, STATE_RUNNING);
        }
        break;
      default:
        /* Selected mode not supported */
        Error = ERROR_MODE_NOT_SUPPORTED;
    } // switch


    return Error;
  } // VL53L0X_StartMeasurement

  Error Api::StopMeasurement() {
    LOG_FUNCTION_START;

    Error = comm.WrByte(REG_SYSRANGE_START, REG_SYSRANGE_MODE_SINGLESHOT);

    Error = comm.WrByte(0xFF, 0x01);
    Error = comm.WrByte(0x00, 0x00);
    Error = comm.WrByte(0x91, 0x00);
    Error = comm.WrByte(0x00, 0x01);
    Error = comm.WrByte(0xFF, 0x00);

    if (Error == ERROR_NONE) {
      /* Set PAL State to Idle */
      PALDevDataSet(PalState, STATE_IDLE);
    }

    /* Check if need to apply interrupt settings */
    if (Error == ERROR_NONE) {
      Error = CheckAndLoadInterruptSettings(0);
    }

    return Error;
  } // VL53L0X_StopMeasurement

  Error Api::GetMeasurementDataReady(uint8_t *pMeasurementDataReady) {
    Error Error = ERROR_NONE;
    uint8_t SysRangeStatusRegister;
    uint8_t InterruptConfig;
    uint32_t InterruptMask;
    LOG_FUNCTION_START;

    InterruptConfig = GetDEVICESPECIFICPARAMETER(Pin0GpioFunctionality);

    if (InterruptConfig == GPIOFUNCTIONALITY_NEW_MEASURE_READY) {
      Error = GetInterruptMaskStatus(&InterruptMask);
      if (InterruptMask == GPIOFUNCTIONALITY_NEW_MEASURE_READY) {
        *pMeasurementDataReady = 1;
      } else {
        *pMeasurementDataReady = 0;
      }
    } else {
      Error = comm.RdByte(REG_RESULT_RANGE_STATUS, &SysRangeStatusRegister);
      if (Error == ERROR_NONE) {
        if (SysRangeStatusRegister & 0x01) {
          *pMeasurementDataReady = 1;
        } else {
          *pMeasurementDataReady = 0;
        }
      }
    }

    return Error;
  } // GetMeasurementDataReady

  Error Api::WaitDeviceReadyForNewMeasurement(uint32_t MaxLoop) {
    VL53L0X_NYI
  }

  Error Api::GetRangingMeasurementData(RangingMeasurementData_t *pRangingMeasurementData) {
    uint8_t DeviceRangeStatus;
    uint8_t RangeFractionalEnable;
    uint8_t PalRangeStatus;
    uint8_t XTalkCompensationEnable;
    uint16_t AmbientRate;
    FixPoint1616_t SignalRate;
    uint16_t XTalkCompensationRateMegaCps;
    uint16_t EffectiveSpadRtnCount;
    uint16_t tmpuint16;
    uint16_t XtalkRangeMilliMeter;
    uint16_t LinearityCorrectiveGain;
    uint8_t localBuffer[12];
    RangingMeasurementData_t LastRangeDataBuffer;

    LOG_FUNCTION_START;

    /*
     * use multi read even if some registers are not useful, result will
     * be more efficient
     * start reading at 0x14 dec20
     * end reading at 0x21 dec33 total 14 bytes to read
     */
    Error = comm.ReadMulti(0x14, localBuffer, 12);

    if (Error == ERROR_NONE) {

      pRangingMeasurementData->ZoneId = 0;    /* Only one zone */
      pRangingMeasurementData->TimeStamp = 0; /* Not Implemented */

      tmpuint16 = MAKEUINT16(localBuffer[11], localBuffer[10]);
      /* cut1.1 if SYSTEM__RANGE_CONFIG if 1 range is 2bits fractional
       *(format 11.2) else no fractional
       */

      pRangingMeasurementData->MeasurementTimeUsec = 0;

      SignalRate = VL53L0X_FIXPOINT97TOFIXPOINT1616(MAKEUINT16(localBuffer[7], localBuffer[6]));
      /* peak_signal_count_rate_rtn_mcps */
      pRangingMeasurementData->SignalRateRtnMegaCps = SignalRate;

      AmbientRate = MAKEUINT16(localBuffer[9], localBuffer[8]);
      pRangingMeasurementData->AmbientRateRtnMegaCps = VL53L0X_FIXPOINT97TOFIXPOINT1616(AmbientRate);

      EffectiveSpadRtnCount = MAKEUINT16(localBuffer[3], localBuffer[2]);
      /* EffectiveSpadRtnCount is 8.8 format */
      pRangingMeasurementData->EffectiveSpadRtnCount = EffectiveSpadRtnCount;

      DeviceRangeStatus = localBuffer[0];

      /* Get Linearity Corrective Gain */
      LinearityCorrectiveGain = PALDevDataGet(LinearityCorrectiveGain);

      /* Get ranging configuration */
      RangeFractionalEnable = PALDevDataGet(RangeFractionalEnable);

      if (LinearityCorrectiveGain != 1000) {
        tmpuint16 = (uint16_t) ((LinearityCorrectiveGain * tmpuint16 + 500) / 1000);

        /* Implement Xtalk */
        GetPARAMETERFIELD(XTalkCompensationRateMegaCps, XTalkCompensationRateMegaCps);
        GetPARAMETERFIELD(XTalkCompensationEnable, XTalkCompensationEnable);

        if (XTalkCompensationEnable) {

          if ((SignalRate - ((XTalkCompensationRateMegaCps * EffectiveSpadRtnCount) >> 8)) <= 0) {
            if (RangeFractionalEnable) {
              XtalkRangeMilliMeter = 8888;
            } else {
              XtalkRangeMilliMeter = 8888 << 2;
            }
          } else {
            XtalkRangeMilliMeter = (tmpuint16 * SignalRate) / (SignalRate - ((XTalkCompensationRateMegaCps * EffectiveSpadRtnCount) >> 8));
          }

          tmpuint16 = XtalkRangeMilliMeter;
        }
      }

      if (RangeFractionalEnable) {
        pRangingMeasurementData->RangeMilliMeter = (uint16_t) ((tmpuint16) >> 2);
        pRangingMeasurementData->RangeFractionalPart =
          (uint8_t) ((tmpuint16 & 0x03) << 6);
      } else {
        pRangingMeasurementData->RangeMilliMeter = tmpuint16;
        pRangingMeasurementData->RangeFractionalPart = 0;
      }

      /*
       * For a standard definition of RangeStatus, this should
       * return 0 in case of good result after a ranging
       * The range status depends on the device so call a device
       * specific function to obtain the right Error.
       */
      Error |= Get_pal_range_status(DeviceRangeStatus, SignalRate, EffectiveSpadRtnCount, pRangingMeasurementData, &PalRangeStatus);

      if (Error == ERROR_NONE) {
        pRangingMeasurementData->RangeStatus = PalRangeStatus;
      }
    }

    if (Error == ERROR_NONE) {
      /* Copy last read data into Dev buffer */
      LastRangeDataBuffer = PALDevDataGet(LastRangeMeasure);

      LastRangeDataBuffer.RangeMilliMeter = pRangingMeasurementData->RangeMilliMeter;
      LastRangeDataBuffer.RangeFractionalPart = pRangingMeasurementData->RangeFractionalPart;
      LastRangeDataBuffer.RangeDMaxMilliMeter = pRangingMeasurementData->RangeDMaxMilliMeter;
      LastRangeDataBuffer.MeasurementTimeUsec = pRangingMeasurementData->MeasurementTimeUsec;
      LastRangeDataBuffer.SignalRateRtnMegaCps = pRangingMeasurementData->SignalRateRtnMegaCps;
      LastRangeDataBuffer.AmbientRateRtnMegaCps = pRangingMeasurementData->AmbientRateRtnMegaCps;
      LastRangeDataBuffer.EffectiveSpadRtnCount = pRangingMeasurementData->EffectiveSpadRtnCount;
      LastRangeDataBuffer.RangeStatus = pRangingMeasurementData->RangeStatus;

      PALDevDataSet(LastRangeMeasure, LastRangeDataBuffer);
    }

    return Error;
  } // GetRangingMeasurementData

FixPoint1616_t Api::GetMeasurementRefSignal() {
    return PALDevDataGet(LastSignalRefMcps);
  }

  Error Api::GetHistogramMeasurementData(HistogramMeasurementData_t *pHistogramMeasurementData) {
    VL53L0X_NYI
  }

  Error Api::PerformSingleRangingMeasurement(RangingMeasurementData_t *pRangingMeasurementData) {

    LOG_FUNCTION_START;

    /* This function will do a complete single ranging
     * Here we fix the mode! */
    Error = SetDeviceMode(DEVICEMODE_SINGLE_RANGING);

    if (Error == ERROR_NONE) {
      Error = PerformSingleMeasurement();
    }

    if (Error == ERROR_NONE) {
      Error = GetRangingMeasurementData(pRangingMeasurementData);
    }

    if (Error == ERROR_NONE) {
      Error = ClearInterruptMask(0);
    }

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

  Error Api::SetGpioConfig(uint8_t Pin, DeviceModes DeviceMode, GpioFunctionality Functionality, InterruptPolarity Polarity) {
    uint8_t data;

    LOG_FUNCTION_START;

    if (Pin != 0) {
      Error = ERROR_GPIO_NOT_EXISTING;
    } else if (DeviceMode == DEVICEMODE_GPIO_DRIVE) {
      if (Polarity == INTERRUPTPOLARITY_LOW) {
        data = 0x10;
      } else {
        data = 1;
      }

      Error = comm.WrByte(REG_GPIO_HV_MUX_ACTIVE_HIGH, data);
    } else if (DeviceMode == DEVICEMODE_GPIO_OSC) {

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
    } else {

      if (Error == ERROR_NONE) {
        switch (Functionality) {
          case GPIOFUNCTIONALITY_OFF:
            data = 0x00;
            break;
          case GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW:
            data = 0x01;
            break;
          case GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH:
            data = 0x02;
            break;
          case GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT:
            data = 0x03;
            break;
          case GPIOFUNCTIONALITY_NEW_MEASURE_READY:
            data = 0x04;
            break;
          default:
            Error = ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED;
        } // switch
      }

      if (Error == ERROR_NONE) {
        Error = comm.WrByte(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, data);
      }

      if (Error == ERROR_NONE) {
        if (Polarity == INTERRUPTPOLARITY_LOW) {
          data = 0;
        } else {
          data = (uint8_t) (1 << 4);
        }

        Error = comm.UpdateByte(REG_GPIO_HV_MUX_ACTIVE_HIGH, 0xEF, data);
      }

      if (Error == ERROR_NONE) {
        VL53L0X_SETDEVICESPECIFICPARAMETER(Pin0GpioFunctionality, Functionality);
      }

      if (Error == ERROR_NONE) {
        Error = ClearInterruptMask(0);
      }
    }

    return Error;
  } // VL53L0X_SetGpioConfig

  Error Api::GetGpioConfig(uint8_t Pin, DeviceModes *pDeviceMode, GpioFunctionality *pFunctionality, InterruptPolarity *pPolarity) {
    GpioFunctionality GpioFunctionality;
    uint8_t data;

    LOG_FUNCTION_START;

    /* pDeviceMode not managed by Ewok it return the current mode */

    Error = GetDeviceMode(pDeviceMode);

    if (Error == ERROR_NONE) {
      if (Pin != 0) {
        Error = ERROR_GPIO_NOT_EXISTING ;
      } else {
        Error = comm.RdByte(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, &data);
      }
    }

    if (Error == ERROR_NONE) {
      switch (data & 0x07) {
        case 0x00:
          GpioFunctionality = GPIOFUNCTIONALITY_OFF;
          break;
        case 0x01:
          GpioFunctionality = GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW;
          break;
        case 0x02:
          GpioFunctionality = GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH;
          break;
        case 0x03:
          GpioFunctionality = GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT;
          break;
        case 0x04:
          GpioFunctionality = GPIOFUNCTIONALITY_NEW_MEASURE_READY;
          break;
        default:
          return ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED;
      } // switch
    }

    if (Error == ERROR_NONE) {
      Error = comm.RdByte(REG_GPIO_HV_MUX_ACTIVE_HIGH, &data);
    }

    if (Error == ERROR_NONE) {
      if ((data & (uint8_t) (1 << 4)) == 0) {
        *pPolarity = INTERRUPTPOLARITY_LOW;
      } else {
        *pPolarity = INTERRUPTPOLARITY_HIGH;
      }
    }

    if (Error == ERROR_NONE) {
      *pFunctionality = GpioFunctionality;
      VL53L0X_SETDEVICESPECIFICPARAMETER(Pin0GpioFunctionality, GpioFunctionality);
    }

    return Error;
  } // GetGpioConfig

  Error Api::SetInterruptThresholds(DeviceModes DeviceMode, FixPoint1616_t ThresholdLow, FixPoint1616_t ThresholdHigh) {
    uint16_t Threshold16;
    LOG_FUNCTION_START;

    /* no dependency on DeviceMode for Ewok
     * Need to divide by 2 because the FW will apply a x2*/
    Threshold16 = (uint16_t) ((ThresholdLow >> 17) & 0x00fff);
    Error = comm.WrWord(REG_SYSTEM_THRESH_LOW, Threshold16);

    if (Error == ERROR_NONE) {
      /* Need to divide by 2 because the FW will apply a x2 */
      Threshold16 = (uint16_t) ((ThresholdHigh >> 17) & 0x00fff);
      Error = comm.WrWord(REG_SYSTEM_THRESH_HIGH, Threshold16);
    }

    return Error;
  } // VL53L0X_SetInterruptThresholds

  Error Api::GetInterruptThresholds(DeviceModes DeviceMode, FixPoint1616_t *pThresholdLow, FixPoint1616_t *pThresholdHigh) {

    uint16_t Threshold16;
    LOG_FUNCTION_START;

    /* no dependency on DeviceMode for Ewok */

    Error = comm.RdWord(REG_SYSTEM_THRESH_LOW, &Threshold16);
    /* Need to multiply by 2 because the FW will apply a x2 */
    *pThresholdLow = (FixPoint1616_t) ((0x00fff & Threshold16) << 17);

    if (Error == ERROR_NONE) {
      Error = comm.RdWord(REG_SYSTEM_THRESH_HIGH, &Threshold16);
      /* Need to multiply by 2 because the FW will apply a x2 */
      *pThresholdHigh = (FixPoint1616_t) ((0x00fff & Threshold16) << 17);
    }

    return Error;
  } // GetInterruptThresholds

  Erroneous<uint8_t> Api::GetStopCompletedStatus() {
    LOG_FUNCTION_START;

    Erroneous<uint8_t> Byte;
    {
      auto pager = autoCloser(Private_Pager, 1, 0);
      fetch(Byte, Private_04);
    }

    if (Byte == 0) {
      auto magic = magicWrapper();
      comm.WrByte(0x91, PALDevDataGet(StopVariable));//errors ignored.
    }

    return Byte;
  } // GetStopCompletedStatus

/* Group PAL Interrupt Functions */
  Error Api::ClearInterruptMask(uint32_t InterruptMask) {
    uint8_t LoopCount;
    uint8_t Byte;
    LOG_FUNCTION_START;

    /* clear bit 0 range interrupt, bit 1 error interrupt */
    LoopCount = 0;
    do {
      Error = comm.WrByte(REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
      Error |= comm.WrByte(REG_SYSTEM_INTERRUPT_CLEAR, 0x00);
      Error |= comm.RdByte(REG_RESULT_INTERRUPT_STATUS, &Byte);
      LoopCount++;
    } while (((Byte & 0x07) != 0x00) && (LoopCount < 3) && (Error == ERROR_NONE));

    if (LoopCount >= 3) {
      return ERROR_INTERRUPT_NOT_CLEARED;
    }

    return Error;
  } // ClearInterruptMask

  Error Api::GetInterruptMaskStatus(uint32_t *pInterruptMaskStatus) {
    LOG_FUNCTION_START;
    uint8_t Byte;
    Error = comm.RdByte(REG_RESULT_INTERRUPT_STATUS, &Byte);
    *pInterruptMaskStatus = Byte & 0x07;

    if (Byte & 0x18) {//if either bit? what are each of them?
      return ERROR_RANGE_ERROR;
    }

    return Error;
  } // GetInterruptMaskStatus

  Error Api::EnableInterruptMask(uint32_t InterruptMask) {
    VL53L0X_NYI
  }

/* End Group PAL Interrupt Functions */

/* Group SPAD functions */

  Error Api::SetSpadAmbientDamperThreshold(uint16_t SpadAmbientDamperThreshold) {
    LOG_FUNCTION_START;
    Error  |= comm.WrByte(0xFF, 0x01);
    Error |= comm.WrWord(0x40, SpadAmbientDamperThreshold);
    Error |= comm.WrByte(0xFF, 0x00);

    return Error;
  } // VL53L0X_SetSpadAmbientDamperThreshold

  Error Api::GetSpadAmbientDamperThreshold(uint16_t *pSpadAmbientDamperThreshold) {
    LOG_FUNCTION_START;
    Error |= comm.WrByte(0xFF, 0x01);
    Error |= comm.RdWord(0x40, pSpadAmbientDamperThreshold);
    Error |= comm.WrByte(0xFF, 0x00);

    return Error;
  } // GetSpadAmbientDamperThreshold

  Error Api::SetSpadAmbientDamperFactor(uint16_t SpadAmbientDamperFactor) {
    LOG_FUNCTION_START;
    Error |= comm.WrByte(0xFF, 0x01);
    Error |= comm.WrByte(0x42, (uint8_t) (SpadAmbientDamperFactor & 0x00FF));
    Error |= comm.WrByte(0xFF, 0x00);

    return Error;
  } // VL53L0X_SetSpadAmbientDamperFactor

  Error Api::GetSpadAmbientDamperFactor(uint16_t *pSpadAmbientDamperFactor) {
    LOG_FUNCTION_START;

    Error |= comm.WrByte(0xFF, 0x01);
    uint8_t Byte;
    Error |= comm.RdByte(0x42, &Byte);
    Error |= comm.WrByte(0xFF, 0x00);
    *pSpadAmbientDamperFactor = (uint16_t) Byte;

    return Error;
  } // GetSpadAmbientDamperFactor

/* END Group SPAD functions */

/*****************************************************************************
* Internal functions
*****************************************************************************/

  Error Api::perform_ref_spad_management(unsigned &refSpadCount, bool &isApertureSpads) {

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

    uint16_t targetRefRate = 0x0A00; /* 20 MCPS in 9:7 format */ //ick:why init with specific value that is immediately overwritten
    targetRefRate = PALDevDataGet(targetRefRate);

    /*
     * Initialize Spad arrays.
     * Currently the good spad map is initialised to 'All good'.
     * This is a short term implementation. The good spad map will be
     * provided as an input.
     */
    Data.SpadData.RefSpadEnables.clear();

    ErrorAccumulator Error = comm.WrByte(0xFF, 0x01);
    ERROR_OUT;
    Error = comm.WrByte(REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    ERROR_OUT;
    Error = comm.WrByte(REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    ERROR_OUT;
    Error = comm.WrByte(0xFF, 0x00);
    ERROR_OUT;
    Error = comm.WrByte(REG_GLOBAL_CONFIG_REF_EN_START_SELECT, startSelect);
    ERROR_OUT;
    Error = comm.WrByte(REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0);
    ERROR_OUT;
    /* Perform ref calibration */
    uint8_t PhaseCal = 0;
    uint8_t VhvSettings = 0;
    Error = perform_ref_calibration(&VhvSettings, &PhaseCal, 0);//ick: PhaseCal value then ignored. ditto for VhvSettings
    ERROR_OUT;
    /* Enable Minimum NON-APERTURE Spads */
    uint32_t currentSpadIndex = 0;
    uint32_t lastSpadIndex = currentSpadIndex;
    uint32_t needAptSpads = 0;
    Error = enable_ref_spads(needAptSpads, Data.SpadData.RefGoodSpadMap, Data.SpadData.RefSpadEnables,  startSelect, currentSpadIndex, minimumSpadCount, &lastSpadIndex);
    ERROR_OUT;
    currentSpadIndex = lastSpadIndex;
    uint16_t peakSignalRateRef;
    Error = perform_ref_signal_measurement(&peakSignalRateRef);
    ERROR_OUT;

    uint8_t isApertureSpads_int = 0;
    uint32_t refSpadCount_int = 0;

    if (peakSignalRateRef > targetRefRate) { /* Signal rate measurement too high, switch to APERTURE SPADs */

      Data.SpadData.RefSpadEnables.clear();

      /* Increment to the first APERTURE spad */
      while (!is_aperture(startSelect + currentSpadIndex) && (currentSpadIndex < maxSpadCount)) {
        currentSpadIndex++;
      }
      needAptSpads = 1;
      Error = enable_ref_spads(needAptSpads, Dev->Data.SpadData.RefGoodSpadMap, Dev->Data.SpadData.RefSpadEnables, spadArraySize, startSelect, currentSpadIndex, minimumSpadCount, &lastSpadIndex);

      if (Error == ERROR_NONE) {
        currentSpadIndex = lastSpadIndex;
        Error = perform_ref_signal_measurement(&peakSignalRateRef);

        if ((Error == ERROR_NONE) &&
            (peakSignalRateRef > targetRefRate)) {
          /* Signal rate still too high after
           * setting the minimum number of
           * APERTURE spads. Can do no more
           * therefore set the min number of
           * aperture spads as the result.
           */
          isApertureSpads_int = 1;
          refSpadCount_int = minimumSpadCount;
        }
      }
    } else {
      needAptSpads = 0;
    }
    ERROR_OUT;
    if (peakSignalRateRef < targetRefRate) {
/* At this point, the minimum number of either aperture
 * or non-aperture spads have been set. Proceed to add
 * spads and perform measurements until the target
 * reference is reached.
 */
      isApertureSpads_int = needAptSpads;
      refSpadCount_int = minimumSpadCount;

      SpadArray lastSpadArray=Data.SpadData.RefSpadEnables;

      uint32_t lastSignalRateDiff = abs(peakSignalRateRef - targetRefRate);
      uint8_t complete = 0;
      while (!complete) {
        int32_t nextGoodSpad = 0;
        get_next_good_spad(Data.SpadData.RefGoodSpadMap, currentSpadIndex, &nextGoodSpad);
        if (nextGoodSpad == -1) {
          return ERROR_REF_SPAD_INIT;
        }
        ++refSpadCount_int;
/* Cannot combine Aperture and Non-Aperture spads, so
 * ensure the current spad is of the correct type.
 */
        if (is_aperture( startSelect + nextGoodSpad) != needAptSpads) {
          return ERROR_REF_SPAD_INIT;
        }

        currentSpadIndex = nextGoodSpad;
        Error = enable_spad_bit(Data.SpadData.RefSpadEnables,  currentSpadIndex);
        ERROR_OUT;
        currentSpadIndex++;
/* Proceed to apply the additional spad and
 * perform measurement. */
        Error = set_ref_spad_map(Data.SpadData.RefSpadEnables);

        ERROR_OUT;
        Error = perform_ref_signal_measurement(&peakSignalRateRef);
        ERROR_OUT;
        uint32_t signalRateDiff = abs(peakSignalRateRef - targetRefRate);

        if (peakSignalRateRef > targetRefRate) { /* Select the spad map that provides the measurement closest to the target rate, either above or below it. */
          if (signalRateDiff > lastSignalRateDiff) { /* Previous spad map produced a closer measurement, so choose this. */
            Error = set_ref_spad_map(lastSpadArray);
            Data.SpadData.RefSpadEnables= lastSpadArray;
            --refSpadCount_int;
          }
          complete = 1;
        } else { /* Continue to add spads */
          lastSignalRateDiff = signalRateDiff;
          lastSpadArray= Data.SpadData.RefSpadEnables;
        }
      } /* while */
    }

    ERROR_OUT;
    *refSpadCount = refSpadCount_int;
    *isApertureSpads = isApertureSpads_int;

    VL53L0X_SETDEVICESPECIFICPARAMETER(RefSpadsInitialised, 1);
    VL53L0X_SETDEVICESPECIFICPARAMETER(ReferenceSpadCount, (uint8_t) (*refSpadCount));
    VL53L0X_SETDEVICESPECIFICPARAMETER(ReferenceSpadType, *isApertureSpads);

    return ERROR_NONE;
  } // VL53L0X_perform_ref_spad_management

  Error Api::set_vcsel_pulse_period(VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriodPCLK) {
    uint8_t vcsel_period_reg;
    const uint8_t MinPreVcselPeriodPCLK = 12;
    const uint8_t MaxPreVcselPeriodPCLK = 18;
    const uint8_t MinFinalVcselPeriodPCLK = 8;
    const uint8_t MaxFinalVcselPeriodPCLK = 14;
    uint32_t MeasurementTimingBudgetMicroSeconds;
    uint32_t FinalRangeTimeoutMicroSeconds;
    uint32_t PreRangeTimeoutMicroSeconds;
    uint32_t MsrcTimeoutMicroSeconds;
    uint8_t PhaseCalInt = 0;

    /* Check if valid clock period requested */

    if ((VCSELPulsePeriodPCLK % 2) != 0) {
      /* Value must be an even number */
      return ERROR_INVALID_PARAMS;
    }
    if (VcselPeriodType == VCSEL_PERIOD_PRE_RANGE && (VCSELPulsePeriodPCLK < MinPreVcselPeriodPCLK || VCSELPulsePeriodPCLK > MaxPreVcselPeriodPCLK)) {
      return ERROR_INVALID_PARAMS;
    }
    if (VcselPeriodType == VCSEL_PERIOD_FINAL_RANGE && (VCSELPulsePeriodPCLK < MinFinalVcselPeriodPCLK || VCSELPulsePeriodPCLK > MaxFinalVcselPeriodPCLK)) {
      return ERROR_INVALID_PARAMS;
    }
    ErrorAccumulator Error = ERROR_NONE;
    //BUG: the code below repeatedly ignores errors of writing to ..._PHASE_HIGH.
    //BUG: the code below repeatedly ignores errors in I2C operations with the result that incoherent settings might be made
    //... if you are going to ignore errors then make some tables and iterate over them, that takes less code.
    /* Apply specific settings for the requested clock period */
    switch (VcselPeriodType) {
      case VCSEL_PERIOD_PRE_RANGE: {
        /* Set phase check limits */
        switch (VCSELPulsePeriodPCLK) {
          case 12:
            Error = setValidPhase(0x18, 0x08);
            break;
          case 14:
            Error = setValidPhase(0x30, 0x08);
            break;
          case 16:
            Error = setValidPhase(0x40, 0x08);
            break;
          case 18:
            Error = setValidPhase(0x50, 0x08);
            break;
        }
        break;
        case VCSEL_PERIOD_FINAL_RANGE:
          switch (VCSELPulsePeriodPCLK) {
            case 8:
              Error = setValidPhase(0x10, 0x08);
              Error |= comm.WrByte(REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
              Error |= comm.WrByte(REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
              Error |= setPhasecalLimit(0x30);
              break;
            case 10:
              Error = setValidPhase(0x28, 0x08);
              Error |= comm.WrByte(REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
              Error |= comm.WrByte(REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
              Error |= setPhasecalLimit(0x20);
              break;
            case 12:
              Error = setValidPhase(0x38, 0x08);
              Error |= comm.WrByte(REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
              Error |= comm.WrByte(REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
              Error |= setPhasecalLimit(0x20);
              break;
            case 14:
              Error = setValidPhase(0x048, 0x08);
              Error |= comm.WrByte(REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
              Error |= comm.WrByte(REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
              Error |= setPhasecalLimit(0x20);
              break;
              break;
            default:
              break;
          }
      }
    }
    /* Re-calculate and apply timeouts, in macro periods */

    if (!Error) {
      vcsel_period_reg = encode_vcsel_period(VCSELPulsePeriodPCLK);

      /* When the VCSEL period for the pre or final range is changed,
       * the corresponding timeout must be read from the device using
       * the current VCSEL period, then the new VCSEL period can be
       * applied. The timeout then must be written back to the device
       * using the new VCSEL period.
       *
       * For the MSRC timeout, the same applies - this timeout being
       * dependant on the pre-range vcsel period.
       */
      switch (VcselPeriodType) {
        case VCSEL_PERIOD_PRE_RANGE:
          Error = get_sequence_step_timeout(SEQUENCESTEP_PRE_RANGE, &PreRangeTimeoutMicroSeconds);

          if (Error == ERROR_NONE) {
            Error = get_sequence_step_timeout(SEQUENCESTEP_MSRC, &MsrcTimeoutMicroSeconds);
          }

          if (Error == ERROR_NONE) {
            Error = comm.WrByte(REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
          }

          if (Error == ERROR_NONE) {
            Error = set_sequence_step_timeout(SEQUENCESTEP_PRE_RANGE, PreRangeTimeoutMicroSeconds);
          }

          if (Error == ERROR_NONE) {
            Error = set_sequence_step_timeout(SEQUENCESTEP_MSRC, MsrcTimeoutMicroSeconds);
          }

          VL53L0X_SETDEVICESPECIFICPARAMETER(PreRange.VcselPulsePeriod, VCSELPulsePeriodPCLK);
          break;
        case VCSEL_PERIOD_FINAL_RANGE:
          Error = get_sequence_step_timeout(SEQUENCESTEP_FINAL_RANGE, &FinalRangeTimeoutMicroSeconds);

          if (Error == ERROR_NONE) {
            Error = comm.WrByte(REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
          }
          if (Error == ERROR_NONE) {
            Error = set_sequence_step_timeout(SEQUENCESTEP_FINAL_RANGE, FinalRangeTimeoutMicroSeconds);
          }
          VL53L0X_SETDEVICESPECIFICPARAMETER(FinalRange.VcselPulsePeriod, VCSELPulsePeriodPCLK);
          break;
        default:
          return ERROR_INVALID_PARAMS;
      } // switch
    }

    /* Finally, the timing budget must be re-applied */
    if (Error == ERROR_NONE) {
      MeasurementTimingBudgetMicroSeconds=VL53L0X_GETPARAMETERFIELD(MeasurementTimingBudgetMicroSeconds);
      return set_measurement_timing_budget_micro_seconds(MeasurementTimingBudgetMicroSeconds);
    }

    /* Perform the phase calibration. This is needed after changing on
     * vcsel period.
     * get_data_enable = 0, restore_config = 1 */
    return perform_phase_calibration(&PhaseCalInt, 0, 1);
  } // VL53L0X_set_vcsel_pulse_period
