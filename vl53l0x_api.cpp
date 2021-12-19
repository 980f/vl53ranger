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
#include "vl53l0x_api_strings.h"
#include "vl53l0x_interrupt_threshold_settings.h"
#include "vl53l0x_tuning.h"  //tuning values packed into an array of bytes

#include "log_api.h"

#ifdef VL53L0X_LOG_ENABLE
#define trace_print(level, ...)   trace_print_module_function(TRACE_MODULE_API, level, TRACE_FUNCTION_NONE, ## __VA_ARGS__)
#endif

namespace VL53L0X {

Version_t ImplementationVersion{VL53L0X_IMPLEMENTATION_VER_REVISION,{VL53L0X_IMPLEMENTATION_VER_MAJOR,VL53L0X_IMPLEMENTATION_VER_MINOR },VL53L0X_IMPLEMENTATION_VER_SUB};

/* Group PAL General Functions */


Version_t PalSpecVersion {VL53L0X_SPECIFICATION_VER_REVISION, VL53L0X_SPECIFICATION_VER_MAJOR, VL53L0X_SPECIFICATION_VER_MINOR, VL53L0X_SPECIFICATION_VER_SUB};

  Erroneous<bool> Api::measurement_poll_for_completion() {
    LOG_FUNCTION_START;

    for(unsigned LoopNb = VL53L0X_DEFAULT_MAX_LOOP;LoopNb-->0;) {
      Erroneous<bool> value;
      value = GetMeasurementDataReady();
      if (~value || value == 1) {//980f: reversed legacy order of testing, test valid before testing value.
        return value;
      }
      PollingDelay();//delay is on comm so that it can use platform specific technique
    }
    return {false,ERROR_TIME_OUT};//was init to 'timeout'
  } // VL53L0X_measurement_poll_for_completion


  Error Api::check_part_used(uint8_t &Revision, DeviceInfo_t &pDeviceInfo) {
    LOG_FUNCTION_START;
    Error |= get_info_from_device( 2);

    if (~Error) {
      if (VL53L0X_GETDEVICESPECIFICPARAMETER( this,ModuleId) == 0) {
        Revision = 0;
        COPYSTRING(pDeviceInfo.ProductId, "");
      } else {
        Revision =  VL53L0X_GETDEVICESPECIFICPARAMETER(this, Revision);
        COPYSTRING(pDeviceInfo.ProductId, VL53L0X_GETDEVICESPECIFICPARAMETER(this, ProductId));
      }
    }

    return Error;
  } // check_part_used

  Error Api::get_device_info(DeviceInfo_t &pDeviceInfo) {
    uint8_t Revision;
    Error Error = check_part_used( Revision, pDeviceInfo);
    ERROR_OUT;
    if (Revision == 0) {
      COPYSTRING(pDeviceInfo.Name, VL53L0X_STRING_DEVICE_INFO_NAME_TS0);
    } else if ((Revision <= 34) && (Revision != 32)) {
      COPYSTRING(pDeviceInfo.Name,VL53L0X_STRING_DEVICE_INFO_NAME_TS1);
    } else if (Revision < 39) {
      COPYSTRING(pDeviceInfo.Name,VL53L0X_STRING_DEVICE_INFO_NAME_TS2);
    } else {
      COPYSTRING(pDeviceInfo.Name,VL53L0X_STRING_DEVICE_INFO_NAME_ES1);
    }
    COPYSTRING(pDeviceInfo.Type, VL53L0X_STRING_DEVICE_INFO_TYPE);
    Error = comm.RdByte( REG_IDENTIFICATION_MODEL_ID, &pDeviceInfo.ProductType);
    ERROR_OUT;

    //DUP: duplicate code.
    uint8_t revision_id;
    Error = comm.RdByte(REG_IDENTIFICATION_REVISION_ID, &revision_id);
    pDeviceInfo.ProductRevision.major = 1;
    pDeviceInfo.ProductRevision.minor = revision_id  >> 4; //BUG: rev id set even if read fails.

    return Error;
  } // get_device_info

SemverLite Core::GetProductRevision(){

  LOG_FUNCTION_START;
  Erroneous<uint8_t> revision_id;

  fetch(revision_id,REG_IDENTIFICATION_REVISION_ID);

  if(revision_id.isOk()){
    return {1,uint8_t(revision_id.wrapped >> 4)};
  }
  
  return {0,0};//use a sentinal value instead of tedious error warpping.
} // GetProductRevision

//Error Api::GetDeviceInfo(DeviceInfo_t &pVL53L0X_DeviceInfo){
//  LOG_FUNCTION_START;
//return get_device_info( pVL53L0X_DeviceInfo);
//}

Erroneous<DeviceError> Api::GetDeviceErrorStatus(){
  LOG_FUNCTION_START;
  Erroneous<uint8_t> caster;
  fetch(caster, REG_RESULT_RANGE_STATUS);
  return { DeviceError((caster.wrapped & 0x78) >> 3),caster.error  };
} // GetDeviceErrorStatus

const char * Api::GetDeviceErrorString(DeviceError ErrorCode){
  return device_error_string(ErrorCode);
}

const char * Api::GetRangeStatusString(uint8_t RangeStatus){
  return range_status_string(RangeStatus);
}

const char * Api::GetPalErrorString(Error PalErrorCode){
  Error Status = ERROR_NONE;
  LOG_FUNCTION_START;

  return pal_error_string(PalErrorCode);

}

const char * Api::GetPalStateString(State PalStateCode){
  return pal_state_string(PalStateCode);
}

State  Api::GetPalState( ){
  return PALDevDataGet(this, PalState);
}

Error Api::SetPowerMode(PowerModes PowerMode){
  Error Status = ERROR_NONE;
  LOG_FUNCTION_START;

  /* Only level1 of Power mode exists */
  if ((PowerMode != POWERMODE_STANDBY_LEVEL1) &&
    (PowerMode != POWERMODE_IDLE_LEVEL1)) {
    Status = ERROR_MODE_NOT_SUPPORTED;
  } else if (PowerMode == POWERMODE_STANDBY_LEVEL1) {
    /* set the standby level1 of power mode */
    Status = comm.WrByte( 0x80, 0x00);
    if (Status == ERROR_NONE) {
      /* Set PAL State to standby */
      PALDevDataSet(this, PalState, STATE_STANDBY);
      PALDevDataSet(this, PowerMode, POWERMODE_STANDBY_LEVEL1);
    }

  } else {
    /* VL53L0X_POWERMODE_IDLE_LEVEL1 */
    Status = comm.WrByte( 0x80, 0x00);
    if (Status == ERROR_NONE) {
      Status = StaticInit();
    }

    if (Status == ERROR_NONE) {
      PALDevDataSet(this, PowerMode, POWERMODE_IDLE_LEVEL1);
    }
  }


  return Status;
} // VL53L0X_SetPowerMode

Erroneous<PowerModes> Api::GetPowerMode(){
  LOG_FUNCTION_START;

   Erroneous <uint8_t> Byte;
   fetch(Byte , Private_PowerMode);
  /* Only level1 of Power mode exists */
  if (Byte.isOk()) {
    auto encoded=(Byte == 1)?POWERMODE_IDLE_LEVEL1: POWERMODE_STANDBY_LEVEL1;
    PALDevDataSet(this, PowerMode,encoded);
    return {encoded};
  }

return {Byte.error};
} // GetPowerMode

Error Api::SetOffsetCalibrationDataMicroMeter( int32_t OffsetCalibrationDataMicroMeter){
  LOG_FUNCTION_START;

  return set_offset_calibration_data_micro_meter(  OffsetCalibrationDataMicroMeter);

}

Error Api::GetOffsetCalibrationDataMicroMeter( int32_t &pOffsetCalibrationDataMicroMeter){
  LOG_FUNCTION_START;

  return get_offset_calibration_data_micro_meter( pOffsetCalibrationDataMicroMeter);
}

Error VL53L0X_SetLinearityCorrectiveGain(int16_t LinearityCorrectiveGain){
  Error Status = ERROR_NONE;
  LOG_FUNCTION_START;

  if ((LinearityCorrectiveGain < 0) || (LinearityCorrectiveGain > 1000)) {
    Status = Error_INVALID_PARAMS;
  } else {
    PALDevDataSet(this, LinearityCorrectiveGain, LinearityCorrectiveGain);

    if (LinearityCorrectiveGain != 1000) {
      /* Disable FW Xtalk */
      Status = VL53L0X_WrWord( Dev, REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, 0);
    }
  }


  return Status;
} // VL53L0X_SetLinearityCorrectiveGain

Error GetLinearityCorrectiveGain(uint16_t *pLinearityCorrectiveGain){
  Error Status = ERROR_NONE;
  LOG_FUNCTION_START;

  *pLinearityCorrectiveGain = PALDevDataGet(Dev, LinearityCorrectiveGain);


  return Status;
}

Error VL53L0X_SetGroupParamHold(uint8_t GroupParamHold){
  Error Status = Error_NOT_IMPLEMENTED;
  LOG_FUNCTION_START;

  /* not implemented on VL53L0X */


  return Status;
}

Error GetUpperLimitMilliMeter(uint16_t *pUpperLimitMilliMeter){
  Error Status = Error_NOT_IMPLEMENTED;
  LOG_FUNCTION_START;

  /* not implemented on VL53L0X */


  return Status;
}

Error GetTotalSignalRate(FixPoint1616_t *pTotalSignalRate){
  Error Status = ERROR_NONE;
  VL53L0X_RangingMeasurementData_t LastRangeDataBuffer;

  LOG_FUNCTION_START;

  LastRangeDataBuffer = PALDevDataGet(Dev, LastRangeMeasure);

  Status = Get_total_signal_rate(Dev, &LastRangeDataBuffer, pTotalSignalRate);


  return Status;
} // GetTotalSignalRate

/* End Group PAL General Functions */

/* Group PAL Init Functions */
Error Api::SetDeviceAddress( uint8_t DeviceAddress){
  LOG_FUNCTION_START;
  return comm.WrByte(REG_I2C_SLAVE_DEVICE_ADDRESS, DeviceAddress / 2);
}

Error Api::DataInit() {

  DeviceParameters_t CurrentParameters;
  int i;
  uint8_t StopVariable;

  LOG_FUNCTION_START ("");

  /* by default the I2C is running at 1V8 if you want to change it you
   * need to include this define at compilation level. */
#ifdef USE_I2C_2V8
  Error |= comm.UpdateByte(REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, 0xFE, 0x01);
#endif

  /* Set I2C standard mode */
  if (Status == ERROR_NONE) {
    Status = comm.WrByte(0x88, 0x00);
  }

  /* read WHO_AM_I */
  uint8_t b;
  Status = comm.RdByte( 0xC0, &b);
  // Serial.print("WHOAMI: 0x"); Serial.println(b, HEX);

  /* read WHO_AM_I */

  VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, ReadDataFromDeviceDone, 0);

#ifdef USE_IQC_STATION
  if (Status == ERROR_NONE) {
    //status was ignored.
    MyDevice.apply_offset_adjustment();
  }
#endif

  /* Default value is 1000 for Linearity Corrective Gain */
  PALDevDataSet(this, LinearityCorrectiveGain, 1000);

  /* Dmax default Parameter */
  PALDevDataSet(this, DmaxCalRangeMilliMeter, 400);
  PALDevDataSet(this, DmaxCalSignalRateRtnMegaCps, (FixPoint1616_t) ((0x00016B85)));             /* 1.42 No Cover Glass*/

  /* Set Default static parameters
   * set first temporary values 9.44MHz * 65536 = 618660 */
  VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, OscFrequencyMHz, 618660);

  /* Set Default XTalkCompensationRateMegaCps to 0  */
  VL53L0X_SETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps, 0);

  /* Get default parameters */
  Status = GetDeviceParameters(Dev, &CurrentParameters);

  if (Status == ERROR_NONE) {
    /* initialize PAL values */
    CurrentParameters.DeviceMode = VL53L0X_DEVICEMODE_SINGLE_RANGING;
    CurrentParameters.HistogramMode = VL53L0X_HISTOGRAMMODE_DISABLED;
    PALDevDataSet(this, CurrentParameters, CurrentParameters);
  }

  /* Sigma estimator variable */
  PALDevDataSet(this, SigmaEstRefArray, 100);
  PALDevDataSet(this, SigmaEstEffPulseWidth, 900);
  PALDevDataSet(this, SigmaEstEffAmbWidth, 500);
  PALDevDataSet(this, targetRefRate, 0x0A00); /* 20 MCPS in 9:7 format */

  /* Use internal default settings */
  PALDevDataSet(this, UseInternalTuningSettings, 1);

  {
    auto magic=magicWrapper();
  
  Status |= comm.RdByte( 0x91, &StopVariable);
  PALDevDataSet(this, StopVariable, StopVariable);
}
  /* Enable all check */
  for (i = 0; i < VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS; i++) {
    if (Status == ERROR_NONE) {
      Status |= VL53L0X_SetLimitCheckEnable(Dev, i, 1);
    } else {
      break;
    }
  }

  /* Disable the following checks */
  if (Status == ERROR_NONE) {
    Status = VL53L0X_SetLimitCheckEnable( Dev, VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP, 0);
  }

  if (Status == ERROR_NONE) {
    Status = VL53L0X_SetLimitCheckEnable( Dev, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);
  }

  if (Status == ERROR_NONE) {
    Status = VL53L0X_SetLimitCheckEnable( Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC, 0);
  }

  if (Status == ERROR_NONE) {
    Status = VL53L0X_SetLimitCheckEnable( Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE, 0);
  }

  /* Limit default values */
  if (Status == ERROR_NONE) {
    Status = VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
        (FixPoint1616_t)(18 * 65536));
  }
  if (Status == ERROR_NONE) {
    Status = VL53L0X_SetLimitCheckValue( Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
      (FixPoint1616_t)(25 * 65536 / 100));
    /* 0.25 * 65536 */
  }

  if (Status == ERROR_NONE) {
    Status = VL53L0X_SetLimitCheckValue( Dev, VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP, (FixPoint1616_t)(35 * 65536));
  }

  if (Status == ERROR_NONE) {
    Status = VL53L0X_SetLimitCheckValue( Dev, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
      (FixPoint1616_t)(0 * 65536));
  }

  if (Status == ERROR_NONE) {

    PALDevDataSet(this, SequenceConfig, 0xFF);
    Status = comm.WrByte( REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);

    /* Set PAL state to tell that we are waiting for call to
     * VL53L0X_StaticInit */
    PALDevDataSet(this, PalState, State_WAIT_STATICINIT);
  }

  if (Status == ERROR_NONE) {
    VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, RefSpadsInitialised, 0);
  }


  return Status;
} // VL53L0X_DataInit

Error VL53L0X_SetTuningSettingBuffer( uint8_t *pTuningSettingBuffer,uint8_t UseInternalTuningSettings){
  Error Status = ERROR_NONE;

  LOG_FUNCTION_START;

  if (UseInternalTuningSettings == 1) {
    /* Force use internal settings */
    PALDevDataSet(this, UseInternalTuningSettings, 1);
  } else {

    /* check that the first byte is not 0 */
    if (*pTuningSettingBuffer != 0) {
      PALDevDataSet(this, pTuningSettingsPointer, pTuningSettingBuffer);
      PALDevDataSet(this, UseInternalTuningSettings, 0);

    } else {
      Status = Error_INVALID_PARAMS;
    }
  }


  return Status;
} // VL53L0X_SetTuningSettingBuffer

Error GetTuningSettingBuffer( uint8_t **ppTuningSettingBuffer,uint8_t *pUseInternalTuningSettings){
  Error Status = ERROR_NONE;

  LOG_FUNCTION_START;

  *ppTuningSettingBuffer = PALDevDataGet(Dev, pTuningSettingsPointer);
  *pUseInternalTuningSettings = PALDevDataGet(Dev, UseInternalTuningSettings);


  return Status;
} // GetTuningSettingBuffer

Error VL53L0X_StaticInit(VL53L0X_DEV Dev){
  Error Status = ERROR_NONE;
  VL53L0X_DeviceParameters_t CurrentParameters = {0};
  uint8_t *pTuningSettingBuffer;
  uint16_t tempword = 0;
  uint8_t tempbyte = 0;
  uint8_t UseInternalTuningSettings = 0;
  uint32_t count = 0;
  uint8_t isApertureSpads = 0;
  uint32_t refSpadCount = 0;
  uint8_t ApertureSpads = 0;
  uint8_t vcselPulsePeriodPCLK;
  FixPoint1616_t seqTimeoutMilliSecs;

  LOG_FUNCTION_START;

  Status = Get_info_from_device(Dev, 1);

  /* set the ref spad from NVM */
  count = (uint32_t)GetDEVICESPECIFICPARAMETER(Dev, ReferenceSpadCount);
  ApertureSpads = GetDEVICESPECIFICPARAMETER(Dev, ReferenceSpadType);

  /* NVM value invalid */
  if ((ApertureSpads > 1) || ((ApertureSpads == 1) && (count > 32)) ||
    ((ApertureSpads == 0) && (count > 12))) {
    Status = VL53L0X_perform_ref_spad_management(Dev, &refSpadCount, &isApertureSpads);
  } else {
    Status = VL53L0X_set_reference_spads(Dev, count, ApertureSpads);
  }

  /* Initialize tuning settings buffer to prevent compiler warning. */
  pTuningSettingBuffer = DefaultTuningSettings;

  if (Status == ERROR_NONE) {
    UseInternalTuningSettings = PALDevDataGet(Dev, UseInternalTuningSettings);

    if (UseInternalTuningSettings == 0) {
      pTuningSettingBuffer = PALDevDataGet(Dev, pTuningSettingsPointer);
    } else {
      pTuningSettingBuffer = DefaultTuningSettings;
    }
  }

  if (Status == ERROR_NONE) {
    Status = VL53L0X_load_tuning_settings(Dev, pTuningSettingBuffer);
  }

  /* Set interrupt config to new sample ready */
  if (Status == ERROR_NONE) {
    Status = VL53L0X_SetGpioConfig( Dev, 0, 0, REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY, VL53L0X_INTERRUPTPOLARITY_LOW);
  }

  if (Status == ERROR_NONE) {
    Status = comm.WrByte( 0xFF, 0x01);
    Status |= VL53L0X_RdWord(Dev, 0x84, &tempword);
    Status |= comm.WrByte( 0xFF, 0x00);
  }

  if (Status == ERROR_NONE) {
    VL53L0X_SETDEVICESPECIFICPARAMETER( Dev, OscFrequencyMHz, VL53L0X_FIXPOINT412TOFIXPOINT1616(tempword));
  }

  /* After static init, some device parameters may be changed,
   * so update them */
  if (Status == ERROR_NONE) {
    Status = GetDeviceParameters(Dev, &CurrentParameters);
  }

  if (Status == ERROR_NONE) {
    Status = GetFractionEnable(Dev, &tempbyte);
    if (Status == ERROR_NONE) {
      PALDevDataSet(this, RangeFractionalEnable, tempbyte);
    }
  }

  if (Status == ERROR_NONE) {
    PALDevDataSet(this, CurrentParameters, CurrentParameters);
  }

  /* read the sequence config and save it */
  if (Status == ERROR_NONE) {
    Status = comm.RdByte( REG_SYSTEM_SEQUENCE_CONFIG, &tempbyte);
    if (Status == ERROR_NONE) {
      PALDevDataSet(this, SequenceConfig, tempbyte);
    }
  }

  /* Disable MSRC and TCC by default */
  if (Status == ERROR_NONE) {
    Status = VL53L0X_SetSequenceStepEnable(Dev, VL53L0X_SEQUENCESTEP_TCC, 0);
  }

  if (Status == ERROR_NONE) {
    Status = VL53L0X_SetSequenceStepEnable(Dev, VL53L0X_SEQUENCESTEP_MSRC, 0);
  }

  /* Set PAL State to standby */
  if (Status == ERROR_NONE) {
    PALDevDataSet(this, PalState, State_IDLE);
  }

  /* Store pre-range vcsel period */
  if (Status == ERROR_NONE) {
    Status = GetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, &vcselPulsePeriodPCLK);
  }

  if (Status == ERROR_NONE) {
    VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, PreRangeVcselPulsePeriod, vcselPulsePeriodPCLK);
  }

  /* Store final-range vcsel period */
  if (Status == ERROR_NONE) {
    Status = GetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, &vcselPulsePeriodPCLK);
  }

  if (Status == ERROR_NONE) {
    VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, FinalRangeVcselPulsePeriod, vcselPulsePeriodPCLK);
  }

  /* Store pre-range timeout */
  if (Status == ERROR_NONE) {
    Status = GetSequenceStepTimeout(Dev, VL53L0X_SEQUENCESTEP_PRE_RANGE, &seqTimeoutMilliSecs);
  }

  if (Status == ERROR_NONE) {
    VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, PreRangeTimeoutMicroSecs, seqTimeoutMilliSecs);
  }

  /* Store final-range timeout */
  if (Status == ERROR_NONE) {
    Status = GetSequenceStepTimeout( Dev, VL53L0X_SEQUENCESTEP_FINAL_RANGE, &seqTimeoutMilliSecs);
  }

  if (Status == ERROR_NONE) {
    VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, FinalRangeTimeoutMicroSecs, seqTimeoutMilliSecs);
  }


  return Status;
} // VL53L0X_StaticInit

Error VL53L0X_WaitDeviceBooted(VL53L0X_DEV Dev){
  Error Status = Error_NOT_IMPLEMENTED;
  LOG_FUNCTION_START;

  /* not implemented on VL53L0X */


  return Status;
}

Error Api::ResetDevice(){
  Error Status = ERROR_NONE;
  uint8_t Byte;
  LOG_FUNCTION_START;

  /* Set reset bit */
  Status = comm.WrByte( REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x00);

  /* Wait for some time */
  if (Status == ERROR_NONE) {
    do {
      Status = comm.RdByte( REG_IDENTIFICATION_MODEL_ID, &Byte);
    } while (Byte != 0x00);
  }

  /* Release reset */
  Status = comm.WrByte( REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x01);

  /* Wait until correct boot-up of the device */
  if (Status == ERROR_NONE) {
    do {
      Status = comm.RdByte( REG_IDENTIFICATION_MODEL_ID, &Byte);
    } while (Byte == 0x00);
  }

  /* Set PAL State to State_POWERDOWN */
  if (Status == ERROR_NONE) {
    PALDevDataSet(this, PalState, State_POWERDOWN);
  }

  return Status;
} // VL53L0X_ResetDevice

/* End Group PAL Init Functions */

/* Group PAL Parameters Functions */
Error Api::SetDeviceParameters( const VL53L0X_DeviceParameters_t &pDeviceParameters){
  Error Status = ERROR_NONE;
  int i;
  LOG_FUNCTION_START;
  Status = VL53L0X_SetDeviceMode(Dev, pDeviceParameters->DeviceMode);

  if (Status == ERROR_NONE) {
    Status = VL53L0X_SetInterMeasurementPeriodMilliSeconds( Dev, pDeviceParameters->InterMeasurementPeriodMilliSeconds);
  }

  if (Status == ERROR_NONE) {
    Status = VL53L0X_SetXTalkCompensationRateMegaCps( Dev, pDeviceParameters->XTalkCompensationRateMegaCps);
  }

  if (Status == ERROR_NONE) {
    Status = VL53L0X_SetOffsetCalibrationDataMicroMeter( Dev, pDeviceParameters->RangeOffsetMicroMeters);
  }

  for (i = 0; i < VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS; i++) {
    if (Status == ERROR_NONE) {
      Status |= VL53L0X_SetLimitCheckEnable( Dev, i, pDeviceParameters->LimitChecksEnable[i]);
    } else {
      break;
    }

    if (Status == ERROR_NONE) {
      Status |= VL53L0X_SetLimitCheckValue( Dev, i, pDeviceParameters->LimitChecksValue[i]);
    } else {
      break;
    }
  }

  if (Status == ERROR_NONE) {
    Status = VL53L0X_SetWrapAroundCheckEnable( Dev, pDeviceParameters->WrapAroundCheckEnable);
  }

  if (Status == ERROR_NONE) {
    Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds( Dev, pDeviceParameters->MeasurementTimingBudgetMicroSeconds);
  }


  return Status;
} // VL53L0X_SetDeviceParameters

Error Api::GetDeviceParameters(VL53L0X_DeviceParameters_t &pDeviceParameters){

  LOG_FUNCTION_START;

  Error = GetDeviceMode(pDeviceParameters.DeviceMode);
  ERROR_OUT;

    Status = GetInterMeasurementPeriodMilliSeconds(pDeviceParameters.InterMeasurementPeriodMilliSeconds);
  ERROR_OUT;
    pDeviceParameters.XTalkCompensationEnable = 0;
  ERROR_OUT;
    Status = GetXTalkCompensationRateMegaCps( Dev, &(pDeviceParameters->XTalkCompensationRateMegaCps));
  ERROR_OUT;
    Status = GetOffsetCalibrationDataMicroMeter( Dev, &(pDeviceParameters->RangeOffsetMicroMeters));
  ERROR_OUT;
    for (unsigned i = 0; i < VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS; i++) {
      /* get first the values, then the enables.
       * GetLimitCheckValue will modify the enable
       * flags
       */
      if (Status == ERROR_NONE) {
        Status |= GetLimitCheckValue( Dev, i, &(pDeviceParameters->LimitChecksValue[i]));
      } else {
        break;
      }
      if (Status == ERROR_NONE) {
        Status |= GetLimitCheckEnable( Dev, i, &(pDeviceParameters->LimitChecksEnable[i]));
      } else {
        break;
      }
    }
  ERROR_OUT;
    Status = GetWrapAroundCheckEnable( pDeviceParameters.WrapAroundCheckEnable);
  ERROR_OUT;
    Status = GetMeasurementTimingBudgetMicroSeconds( pDeviceParameters->MeasurementTimingBudgetMicroSeconds);

  return Status;
} // GetDeviceParameters

Error VL53L0X_SetDeviceMode(VL53L0X_DeviceModes DeviceMode){
  Error Status = ERROR_NONE;

  LOG_FUNCTION_START("%d", (int)DeviceMode);

  switch (DeviceMode) {
  case VL53L0X_DEVICEMODE_SINGLE_RANGING:
  case VL53L0X_DEVICEMODE_CONTINUOUS_RANGING:
  case VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING:
  case VL53L0X_DEVICEMODE_GPIO_DRIVE:
  case VL53L0X_DEVICEMODE_GPIO_OSC:
    /* Supported modes */
    VL53L0X_SETPARAMETERFIELD(Dev, DeviceMode, DeviceMode);
    break;
  default:
    /* Unsupported mode */
    Status = Error_MODE_NOT_SUPPORTED;
  } // switch


  return Status;
} // VL53L0X_SetDeviceMode

Error GetDeviceMode(VL53L0X_DeviceModes *pDeviceMode){
  Error Status = ERROR_NONE;
  LOG_FUNCTION_START;

  GetPARAMETERFIELD(Dev, DeviceMode, *pDeviceMode);


  return Status;
}

Error VL53L0X_SetRangeFractionEnable( uint8_t Enable){
  Error Status = ERROR_NONE;

  LOG_FUNCTION_START("%d", (int)Enable);

  Status = comm.WrByte( REG_SYSTEM_RANGE_CONFIG, Enable);

  if (Status == ERROR_NONE) {
    PALDevDataSet(this, RangeFractionalEnable, Enable);
  }


  return Status;
} // VL53L0X_SetRangeFractionEnable

Error GetFractionEnable( uint8_t *pEnabled){
  Error Status = ERROR_NONE;
  LOG_FUNCTION_START;

  Status = comm.RdByte( REG_SYSTEM_RANGE_CONFIG, pEnabled);

  if (Status == ERROR_NONE) {
    *pEnabled = (*pEnabled & 1);
  }


  return Status;
} // GetFractionEnable

Error VL53L0X_SetHistogramMode(VL53L0X_HistogramModes HistogramMode){
  Error Status = Error_NOT_IMPLEMENTED;
  LOG_FUNCTION_START;

  /* not implemented on VL53L0X */


  return Status;
}

Error GetHistogramMode(VL53L0X_HistogramModes *pHistogramMode){
  Error Status = Error_NOT_IMPLEMENTED;
  LOG_FUNCTION_START;

  /* not implemented on VL53L0X */


  return Status;
}

Error Api::SetMeasurementTimingBudgetMicroSeconds( uint32_t MeasurementTimingBudgetMicroSeconds){
  return set_measurement_timing_budget_micro_seconds(MeasurementTimingBudgetMicroSeconds);
} // VL53L0X_SetMeasurementTimingBudgetMicroSeconds

Error GetMeasurementTimingBudgetMicroSeconds( uint32_t *pMeasurementTimingBudgetMicroSeconds){
  Error Status = ERROR_NONE;
  LOG_FUNCTION_START;

  Status = Get_measurement_timing_budget_micro_seconds( Dev, pMeasurementTimingBudgetMicroSeconds);


  return Status;
}

Error Api::SetVcselPulsePeriod(VcselPeriod VcselPeriodType,uint8_t VCSELPulsePeriodPCLK){
  return set_vcsel_pulse_period( VcselPeriodType, VCSELPulsePeriodPCLK);
}

Error GetVcselPulsePeriod(VL53L0X_VcselPeriod VcselPeriodType,uint8_t *pVCSELPulsePeriodPCLK){
  Error Status = ERROR_NONE;
  LOG_FUNCTION_START;

  Status = Get_vcsel_pulse_period(Dev, VcselPeriodType, pVCSELPulsePeriodPCLK);


  return Status;
}

Error VL53L0X_SetSequenceStepEnable(VL53L0X_SequenceStepId SequenceStepId,uint8_t SequenceStepEnabled){
  Error Status = ERROR_NONE;
  uint8_t SequenceConfig = 0;
  uint8_t SequenceConfigNew = 0;
  uint32_t MeasurementTimingBudgetMicroSeconds;
  LOG_FUNCTION_START;

  Status = comm.RdByte( REG_SYSTEM_SEQUENCE_CONFIG, &SequenceConfig);

  SequenceConfigNew = SequenceConfig;

  if (Status == ERROR_NONE) {
    if (SequenceStepEnabled == 1) {

      /* Enable requested sequence step
       */
      switch (SequenceStepId) {
      case VL53L0X_SEQUENCESTEP_TCC:
        SequenceConfigNew |= 0x10;
        break;
      case VL53L0X_SEQUENCESTEP_DSS:
        SequenceConfigNew |= 0x28;
        break;
      case VL53L0X_SEQUENCESTEP_MSRC:
        SequenceConfigNew |= 0x04;
        break;
      case VL53L0X_SEQUENCESTEP_PRE_RANGE:
        SequenceConfigNew |= 0x40;
        break;
      case VL53L0X_SEQUENCESTEP_FINAL_RANGE:
        SequenceConfigNew |= 0x80;
        break;
      default:
        Status = Error_INVALID_PARAMS;
      } // switch
    } else {
      /* Disable requested sequence step
       */
      switch (SequenceStepId) {
      case VL53L0X_SEQUENCESTEP_TCC:
        SequenceConfigNew &= 0xef;
        break;
      case VL53L0X_SEQUENCESTEP_DSS:
        SequenceConfigNew &= 0xd7;
        break;
      case VL53L0X_SEQUENCESTEP_MSRC:
        SequenceConfigNew &= 0xfb;
        break;
      case VL53L0X_SEQUENCESTEP_PRE_RANGE:
        SequenceConfigNew &= 0xbf;
        break;
      case VL53L0X_SEQUENCESTEP_FINAL_RANGE:
        SequenceConfigNew &= 0x7f;
        break;
      default:
        Status = Error_INVALID_PARAMS;
      } // switch
    }
  }

  if (SequenceConfigNew != SequenceConfig) {
    /* Apply New Setting */
    if (Status == ERROR_NONE) {
      Status = comm.WrByte( REG_SYSTEM_SEQUENCE_CONFIG, SequenceConfigNew);
    }
    if (Status == ERROR_NONE) {
      PALDevDataSet(this, SequenceConfig, SequenceConfigNew);
    }

    /* Recalculate timing budget */
    if (Status == ERROR_NONE) {
      GetPARAMETERFIELD(Dev, MeasurementTimingBudgetMicroSeconds, MeasurementTimingBudgetMicroSeconds);

      VL53L0X_SetMeasurementTimingBudgetMicroSeconds( Dev, MeasurementTimingBudgetMicroSeconds);
    }
  }



  return Status;
} // VL53L0X_SetSequenceStepEnable




void Api::GetNumberOfSequenceSteps(uint8_t *pNumberOfSequenceSteps){
  *pNumberOfSequenceSteps = SEQUENCESTEP_NUMBER_OF_CHECKS;
}

Error GetSequenceStepsInfo(SequenceStepId SequenceStepId,char *pSequenceStepsString){
  LOG_FUNCTION_START;
  return get_sequence_steps_info(SequenceStepId, pSequenceStepsString);
} // GetSequenceStepsInfo

Error Api::SetSequenceStepTimeout(SequenceStepId SequenceStepId,FixPoint1616_t TimeOutMilliSecs){
  auto TimeoutMicroSeconds = TimeOutMilliSecs.millis();
  uint32_t MeasurementTimingBudgetMicroSeconds;
  FixPoint1616_t OldTimeOutMicroSeconds;

  LOG_FUNCTION_START;

  /* Read back the current value in case we need to revert back to this.
   */
  Status = get_sequence_step_timeout( SequenceStepId, &OldTimeOutMicroSeconds);

  if (Status == ERROR_NONE) {
    Status = set_sequence_step_timeout(Dev, SequenceStepId, TimeoutMicroSeconds);
  }

  if (Status == ERROR_NONE) {
    GetPARAMETERFIELD(Dev, MeasurementTimingBudgetMicroSeconds, MeasurementTimingBudgetMicroSeconds);

    /* At this point we don't know if the requested value is valid,
     *  therefore proceed to update the entire timing budget and
     *  if this fails, revert back to the previous value.
     */
    Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds( Dev, MeasurementTimingBudgetMicroSeconds);

    if (Status != ERROR_NONE) {
      Status1 = set_sequence_step_timeout(Dev, SequenceStepId, OldTimeOutMicroSeconds);

      if (Status1 == ERROR_NONE) {
        Status1 = VL53L0X_SetMeasurementTimingBudgetMicroSeconds( Dev, MeasurementTimingBudgetMicroSeconds);
      }

      Status = Status1;
    }
  }



  return Status;
} // VL53L0X_SetSequenceStepTimeout

Error GetSequenceStepTimeout(VL53L0X_SequenceStepId SequenceStepId,FixPoint1616_t *pTimeOutMilliSecs){
  Error Status = ERROR_NONE;
  uint32_t TimeoutMicroSeconds;
  uint32_t WholeNumber_ms = 0;
  uint32_t Fraction_ms = 0;
  LOG_FUNCTION_START;

  Status = get_sequence_step_timeout(Dev, SequenceStepId, &TimeoutMicroSeconds);
  if (Status == ERROR_NONE) {
    WholeNumber_ms = TimeoutMicroSeconds / 1000;
    Fraction_ms = TimeoutMicroSeconds - (WholeNumber_ms * 1000);
    *pTimeOutMilliSecs = (WholeNumber_ms << 16) + (((Fraction_ms * 0xffff) + 500) / 1000);
  }


  return Status;
} // GetSequenceStepTimeout

Error VL53L0X_SetInterMeasurementPeriodMilliSeconds( uint32_t InterMeasurementPeriodMilliSeconds){
  Error Status = ERROR_NONE;
  uint16_t osc_calibrate_val;
  uint32_t IMPeriodMilliSeconds;

  LOG_FUNCTION_START;

  Status = VL53L0X_RdWord(Dev, REG_OSC_CALIBRATE_VAL, &osc_calibrate_val);

  if (Status == ERROR_NONE) {
    if (osc_calibrate_val != 0) {
      IMPeriodMilliSeconds = InterMeasurementPeriodMilliSeconds * osc_calibrate_val;
    } else {
      IMPeriodMilliSeconds = InterMeasurementPeriodMilliSeconds;
    }
    Status = VL53L0X_WrDWord(Dev, REG_SYSTEM_INTERMEASUREMENT_PERIOD, IMPeriodMilliSeconds);
  }

  if (Status == ERROR_NONE) {
    VL53L0X_SETPARAMETERFIELD(Dev, InterMeasurementPeriodMilliSeconds, InterMeasurementPeriodMilliSeconds);
  }


  return Status;
} // VL53L0X_SetInterMeasurementPeriodMilliSeconds

Error GetInterMeasurementPeriodMilliSeconds( uint32_t *pInterMeasurementPeriodMilliSeconds){
  Error Status = ERROR_NONE;
  uint16_t osc_calibrate_val;
  uint32_t IMPeriodMilliSeconds;

  LOG_FUNCTION_START;

  Status = VL53L0X_RdWord(Dev, REG_OSC_CALIBRATE_VAL, &osc_calibrate_val);

  if (Status == ERROR_NONE) {
    Status = VL53L0X_RdDWord(Dev, REG_SYSTEM_INTERMEASUREMENT_PERIOD, &IMPeriodMilliSeconds);
  }

  if (Status == ERROR_NONE) {
    if (osc_calibrate_val != 0) {
      *pInterMeasurementPeriodMilliSeconds = IMPeriodMilliSeconds / osc_calibrate_val;
    }
    VL53L0X_SETPARAMETERFIELD(Dev, InterMeasurementPeriodMilliSeconds, *pInterMeasurementPeriodMilliSeconds);
  }


  return Status;
} // GetInterMeasurementPeriodMilliSeconds


Error GetXTalkCompensationEnable(uint8_t *pXTalkCompensationEnable){
  Error Status = ERROR_NONE;
  uint8_t Temp8;
  LOG_FUNCTION_START;

  GetPARAMETERFIELD(Dev, XTalkCompensationEnable, Temp8);
  *pXTalkCompensationEnable = Temp8;


  return Status;
} // GetXTalkCompensationEnable

Error VL53L0X_SetXTalkCompensationRateMegaCps( FixPoint1616_t XTalkCompensationRateMegaCps){
  Error Status = ERROR_NONE;
  uint8_t Temp8;
  uint16_t LinearityCorrectiveGain;
  uint16_t data;
  LOG_FUNCTION_START;

  GetPARAMETERFIELD(Dev, XTalkCompensationEnable, Temp8);
  LinearityCorrectiveGain = PALDevDataGet(Dev, LinearityCorrectiveGain);

  if (Temp8 == 0) { /* disabled write only internal value */
    VL53L0X_SETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps, XTalkCompensationRateMegaCps);
  } else {
    /* the following register has a format 3.13 */
    if (LinearityCorrectiveGain == 1000) {
      data = VL53L0X_FIXPOINT1616TOFIXPOINT313(XTalkCompensationRateMegaCps);
    } else {
      data = 0;
    }

    Status = VL53L0X_WrWord( Dev, REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, data);

    if (Status == ERROR_NONE) {
      VL53L0X_SETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps, XTalkCompensationRateMegaCps);
    }
  }


  return Status;
} // VL53L0X_SetXTalkCompensationRateMegaCps

Error GetXTalkCompensationRateMegaCps( FixPoint1616_t *pXTalkCompensationRateMegaCps){
  Error Status = ERROR_NONE;
  uint16_t Value;
  FixPoint1616_t TempFix1616;

  LOG_FUNCTION_START;

  Status = VL53L0X_RdWord(Dev, REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, (uint16_t *)&Value);
  if (Status == ERROR_NONE) {
    if (Value == 0) {
      /* the Xtalk is disabled return value from memory */
      GetPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps, TempFix1616);
      *pXTalkCompensationRateMegaCps = TempFix1616;
      VL53L0X_SETPARAMETERFIELD(Dev, XTalkCompensationEnable, 0);
    } else {
      TempFix1616 = VL53L0X_FIXPOINT313TOFIXPOINT1616(Value);
      *pXTalkCompensationRateMegaCps = TempFix1616;
      VL53L0X_SETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps, TempFix1616);
      VL53L0X_SETPARAMETERFIELD(Dev, XTalkCompensationEnable, 1);
    }
  }


  return Status;
} // GetXTalkCompensationRateMegaCps

Error VL53L0X_SetRefCalibration( uint8_t VhvSettings,uint8_t PhaseCal){
  Error Status = ERROR_NONE;
  LOG_FUNCTION_START;

  Status = VL53L0X_set_ref_calibration(Dev, VhvSettings, PhaseCal);


  return Status;
}

Error GetRefCalibration( uint8_t *pVhvSettings,uint8_t *pPhaseCal){
  Error Status = ERROR_NONE;
  LOG_FUNCTION_START;

  Status = Get_ref_calibration(Dev, pVhvSettings, pPhaseCal);


  return Status;
}

/*
 * CHECK LIMIT FUNCTIONS
 */

Error GetNumberOfLimitCheck(uint16_t *pNumberOfLimitCheck){
  Error Status = ERROR_NONE;
  LOG_FUNCTION_START;

  *pNumberOfLimitCheck = VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS;


  return Status;
}

Error GetLimitCheckInfo( uint16_t LimitCheckId,char *pLimitCheckString){
  Error Status = ERROR_NONE;

  LOG_FUNCTION_START;

  Status = Get_limit_check_info(Dev, LimitCheckId, pLimitCheckString);


  return Status;
}

Error GetLimitCheckStatus(uint16_t LimitCheckId,uint8_t *pLimitCheckStatus){
  Error Status = ERROR_NONE;
  uint8_t Temp8;

  LOG_FUNCTION_START;

  if (LimitCheckId >= VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS) {
    Status = Error_INVALID_PARAMS;
  } else {
    GetARRAYPARAMETERFIELD(Dev, LimitChecksStatus, LimitCheckId, Temp8);
    *pLimitCheckStatus = Temp8;
  }


  return Status;
} // GetLimitCheckStatus

Error Api::SetLimitCheckEnable(CheckEnable LimitCheckId,uint8_t LimitCheckEnable){

  LOG_FUNCTION_START;

  if (LimitCheckId >= VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS) {
    return ERROR_INVALID_PARAMS;
  }
  FixPoint1616_t TempFix1616 = 0;
  uint8_t LimitCheckEnableInt = 0;
  uint8_t LimitCheckDisable = 0;

  if (LimitCheckEnable == 0) {
      TempFix1616 = 0;
      LimitCheckEnableInt = 0;
      LimitCheckDisable = 1;
    } else {
      GetARRAYPARAMETERFIELD(Dev, LimitChecksValue, LimitCheckId, TempFix1616);
      LimitCheckDisable = 0;
      /* this to be sure to have either 0 or 1 */
      LimitCheckEnableInt = 1;
    }

    switch (LimitCheckId) {
    case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE:
      /* internal computation: */
      VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, LimitCheckEnableInt);
      break;

    case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
      Status = VL53L0X_WrWord( Dev, REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, VL53L0X_FIXPOINT1616TOFIXPOINT97(TempFix1616));
      break;

    case VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP:
      /* internal computation: */
      VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable, VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP, LimitCheckEnableInt);
      break;

    case VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD:
      /* internal computation: */
      VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, LimitCheckEnableInt);
      break;

    case VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC:
      Temp8 = (uint8_t)(LimitCheckDisable << 1);
      Status = VL53L0X_UpdateByte(Dev, REG_MSRC_CONFIG_CONTROL, 0xFE, Temp8);//clear lsb set bit 1
      break;

    case VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
      Temp8 = (uint8_t)(LimitCheckDisable << 4);
      Status = VL53L0X_UpdateByte(Dev, REG_MSRC_CONFIG_CONTROL, 0xEF, Temp8);
      break;

    default:
      Status = Error_INVALID_PARAMS;
    } // switch


  if (Status == ERROR_NONE) {
    if (LimitCheckEnable == 0) {
      VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable, LimitCheckId, 0);
    } else {
      VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable, LimitCheckId, 1);
    }
  }


  return Status;
} // VL53L0X_SetLimitCheckEnable

Erroneous<uint8_t> Api::GetLimitCheckEnable(CheckEnable LimitCheckId){
  LOG_FUNCTION_START;
  if (LimitCheckId >= VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS) {
    return {0, ERROR_INVALID_PARAMS};
  }
    uint8_t Temp8;
    GetARRAYPARAMETERFIELD(this, LimitChecksEnable, LimitCheckId, Temp8);//ick: wrappers like this are painful. Is the layering every going to be useful?
    return {Temp8};
} // GetLimitCheckEnable

Error VL53L0X_SetLimitCheckValue( uint16_t LimitCheckId,FixPoint1616_t LimitCheckValue){
  Error Status = ERROR_NONE;
  uint8_t Temp8;

  LOG_FUNCTION_START;

  GetARRAYPARAMETERFIELD(Dev, LimitChecksEnable, LimitCheckId, Temp8);

  if (Temp8 == 0) { /* disabled write only internal value */
    VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue, LimitCheckId, LimitCheckValue);
  } else {
    switch (LimitCheckId) {
    case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE:
      /* internal computation: */
      VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, LimitCheckValue);
      break;

    case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
      Status = VL53L0X_WrWord( Dev, REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, VL53L0X_FIXPOINT1616TOFIXPOINT97(LimitCheckValue));
      break;

    case VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP:
      /* internal computation: */
      VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue, VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP, LimitCheckValue);
      break;

    case VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD:
      /* internal computation: */
      VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, LimitCheckValue);
      break;

    case VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC:
    case VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
      Status = VL53L0X_WrWord(Dev, REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT, VL53L0X_FIXPOINT1616TOFIXPOINT97(LimitCheckValue));
      break;

    default:
      Status = Error_INVALID_PARAMS;
    } // switch

    if (Status == ERROR_NONE) {
      VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue, LimitCheckId, LimitCheckValue);
    }
  }
  return Status;
} // VL53L0X_SetLimitCheckValue

Error GetLimitCheckValue( uint16_t LimitCheckId,FixPoint1616_t *pLimitCheckValue){
  Error Status = ERROR_NONE;
  uint8_t EnableZeroValue = 0;
  uint16_t Temp16;
  FixPoint1616_t TempFix1616;

  LOG_FUNCTION_START;

  switch (LimitCheckId) {

  case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE:
    /* internal computation: */
    GetARRAYPARAMETERFIELD(Dev, LimitChecksValue, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, TempFix1616);
    EnableZeroValue = 0;
    break;

  case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
    Status = VL53L0X_RdWord( Dev, REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, &Temp16);
    if (Status == ERROR_NONE) {
      TempFix1616 = VL53L0X_FIXPOINT97TOFIXPOINT1616(Temp16);
    }
    EnableZeroValue = 1;
    break;

  case VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP:
    /* internal computation: */
    GetARRAYPARAMETERFIELD(Dev, LimitChecksValue, VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP, TempFix1616);
    EnableZeroValue = 0;
    break;

  case VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD:
    /* internal computation: */
    GetARRAYPARAMETERFIELD(Dev, LimitChecksValue, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, TempFix1616);
    EnableZeroValue = 0;
    break;

  case VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC:
  case VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
    Status = VL53L0X_RdWord(Dev, REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT, &Temp16);
    if (Status == ERROR_NONE) {
      TempFix1616 = VL53L0X_FIXPOINT97TOFIXPOINT1616(Temp16);
    }
    EnableZeroValue = 0;
    break;

  default:
    Status = Error_INVALID_PARAMS;
  } // switch

  if (Status == ERROR_NONE) {
    if (EnableZeroValue == 1) {
      if (TempFix1616 == 0) {
        /* disabled: return value from memory */
        GetARRAYPARAMETERFIELD(Dev, LimitChecksValue, LimitCheckId, TempFix1616);
        *pLimitCheckValue = TempFix1616;
        VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable, LimitCheckId, 0);
      } else {
        *pLimitCheckValue = TempFix1616;
        VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue, LimitCheckId, TempFix1616);
        VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable, LimitCheckId, 1);
      }
    } else {
      *pLimitCheckValue = TempFix1616;
    }
  }


  return Status;
} // GetLimitCheckValue

Error GetLimitCheckCurrent(uint16_t LimitCheckId,FixPoint1616_t *pLimitCheckCurrent){
  Error Status = ERROR_NONE;
  VL53L0X_RangingMeasurementData_t LastRangeDataBuffer;

  LOG_FUNCTION_START;

  if (LimitCheckId >= VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS) {
    Status = Error_INVALID_PARAMS;
  } else {
    switch (LimitCheckId) {
    case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE:
      /* Need to run a ranging to have the latest values */
      *pLimitCheckCurrent = PALDevDataGet(Dev, SigmaEstimate);
      break;

    case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
      /* Need to run a ranging to have the latest values */
      LastRangeDataBuffer = PALDevDataGet(Dev, LastRangeMeasure);
      *pLimitCheckCurrent = LastRangeDataBuffer.SignalRateRtnMegaCps;
      break;

    case VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP:
      /* Need to run a ranging to have the latest values */
      *pLimitCheckCurrent = PALDevDataGet(Dev, LastSignalRefMcps);
      break;

    case VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD:
      /* Need to run a ranging to have the latest values */
      LastRangeDataBuffer = PALDevDataGet(Dev, LastRangeMeasure);
      *pLimitCheckCurrent = LastRangeDataBuffer.SignalRateRtnMegaCps;
      break;

    case VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC:
      /* Need to run a ranging to have the latest values */
      LastRangeDataBuffer = PALDevDataGet(Dev, LastRangeMeasure);
      *pLimitCheckCurrent = LastRangeDataBuffer.SignalRateRtnMegaCps;
      break;

    case VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
      /* Need to run a ranging to have the latest values */
      LastRangeDataBuffer = PALDevDataGet(Dev, LastRangeMeasure);
      *pLimitCheckCurrent = LastRangeDataBuffer.SignalRateRtnMegaCps;
      break;

    default:
      Status = Error_INVALID_PARAMS;
    } // switch
  }


  return Status;
} // GetLimitCheckCurrent

/*
 * WRAPAROUND Check
 */
Error VL53L0X_SetWrapAroundCheckEnable(uint8_t WrapAroundCheckEnable){
  Error Status = ERROR_NONE;
  uint8_t Byte;
  uint8_t WrapAroundCheckEnableInt;

  LOG_FUNCTION_START;

  Status = comm.RdByte( REG_SYSTEM_SEQUENCE_CONFIG, &Byte);
  if (WrapAroundCheckEnable == 0) {
    /* Disable wraparound */
    Byte = Byte & 0x7F;
    WrapAroundCheckEnableInt = 0;
  } else {
    /*Enable wraparound */
    Byte = Byte | 0x80;
    WrapAroundCheckEnableInt = 1;
  }

  Status = comm.WrByte( REG_SYSTEM_SEQUENCE_CONFIG, Byte);

  if (Status == ERROR_NONE) {
    PALDevDataSet(this, SequenceConfig, Byte);
    VL53L0X_SETPARAMETERFIELD(Dev, WrapAroundCheckEnable, WrapAroundCheckEnableInt);
  }


  return Status;
} // VL53L0X_SetWrapAroundCheckEnable

Error GetWrapAroundCheckEnable(uint8_t *pWrapAroundCheckEnable){
  Error Status = ERROR_NONE;
  uint8_t data;

  LOG_FUNCTION_START;

  Status = comm.RdByte( REG_SYSTEM_SEQUENCE_CONFIG, &data);
  if (Status == ERROR_NONE) {
    PALDevDataSet(this, SequenceConfig, data);
    if (data & (0x01 << 7)) {
      *pWrapAroundCheckEnable = 0x01;
    } else {
      *pWrapAroundCheckEnable = 0x00;
    }
  }
  if (Status == ERROR_NONE) {
    VL53L0X_SETPARAMETERFIELD(Dev, WrapAroundCheckEnable, *pWrapAroundCheckEnable);
  }


  return Status;
} // GetWrapAroundCheckEnable

Error VL53L0X_SetDmaxCalParameters( uint16_t RangeMilliMeter,FixPoint1616_t SignalRateRtnMegaCps){
  Error Status = ERROR_NONE;
  FixPoint1616_t SignalRateRtnMegaCpsTemp = 0;

  LOG_FUNCTION_START;

  /* Check if one of input parameter is zero, in that case the
   * value are get from NVM */
  if ((RangeMilliMeter == 0) || (SignalRateRtnMegaCps == 0)) {
    /* NVM parameters */
    /* Run Get_info_from_device wit option 4 to get
     * signal rate at 400 mm if the value have been already
     * get this function will return with no access to device */
    Get_info_from_device(Dev, 4);

    SignalRateRtnMegaCpsTemp = GetDEVICESPECIFICPARAMETER(Dev, SignalRateMeasFixed400mm);

    PALDevDataSet(this, DmaxCalRangeMilliMeter, 400);
    PALDevDataSet(this, DmaxCalSignalRateRtnMegaCps, SignalRateRtnMegaCpsTemp);
  } else {
    /* User parameters */
    PALDevDataSet(this, DmaxCalRangeMilliMeter, RangeMilliMeter);
    PALDevDataSet(this, DmaxCalSignalRateRtnMegaCps, SignalRateRtnMegaCps);
  }


  return Status;
} // VL53L0X_SetDmaxCalParameters

Error GetDmaxCalParameters( uint16_t *pRangeMilliMeter,FixPoint1616_t *pSignalRateRtnMegaCps){
  Error Status = ERROR_NONE;

  LOG_FUNCTION_START;

  *pRangeMilliMeter = PALDevDataGet(Dev, DmaxCalRangeMilliMeter);
  *pSignalRateRtnMegaCps = PALDevDataGet(Dev, DmaxCalSignalRateRtnMegaCps);


  return Status;
} // GetDmaxCalParameters

/* End Group PAL Parameters Functions */

/* Group PAL Measurement Functions */
Error VL53L0X_PerformSingleMeasurement(VL53L0X_DEV Dev){
  Error Status = ERROR_NONE;
  VL53L0X_DeviceModes DeviceMode;

  LOG_FUNCTION_START;

  /* Get Current DeviceMode */
  Status = GetDeviceMode(Dev, &DeviceMode);

  /* Start immediately to run a single ranging measurement in case of
   * single ranging or single histogram */
  if (Status == ERROR_NONE &&
    DeviceMode == VL53L0X_DEVICEMODE_SINGLE_RANGING) {
    Status = VL53L0X_StartMeasurement(Dev);
  }

  if (Status == ERROR_NONE) {
    Status = VL53L0X_measurement_poll_for_completion(Dev);
  }

  /* Change PAL State in case of single ranging or single histogram */
  if (Status == ERROR_NONE &&
    DeviceMode == VL53L0X_DEVICEMODE_SINGLE_RANGING) {
    PALDevDataSet(this, PalState, State_IDLE);
  }


  return Status;
} // VL53L0X_PerformSingleMeasurement

Error VL53L0X_PerformSingleHistogramMeasurement(VL53L0X_HistogramMeasurementData_t *pHistogramMeasurementData){
  Error Status = Error_NOT_IMPLEMENTED;
  LOG_FUNCTION_START;

  /* not implemented on VL53L0X */


  return Status;
}

Error VL53L0X_PerformRefCalibration(uint8_t *pVhvSettings,uint8_t *pPhaseCal){
  Error Status = ERROR_NONE;
  LOG_FUNCTION_START;

  Status = VL53L0X_perform_ref_calibration(Dev, pVhvSettings, pPhaseCal, 1);


  return Status;
}

Error VL53L0X_PerformXTalkMeasurement(uint32_t TimeoutMs,FixPoint1616_t *pXtalkPerSpad,uint8_t *pAmbientTooHigh){
  Error Status = Error_NOT_IMPLEMENTED;
  LOG_FUNCTION_START;

  /* not implemented on VL53L0X */


  return Status;
}

Error VL53L0X_PerformXTalkCalibration(FixPoint1616_t XTalkCalDistance,FixPoint1616_t *pXTalkCompensationRateMegaCps){
  Error Status = ERROR_NONE;
  LOG_FUNCTION_START;

  Status = VL53L0X_perform_xtalk_calibration(Dev, XTalkCalDistance, pXTalkCompensationRateMegaCps);


  return Status;
}

Error VL53L0X_PerformOffsetCalibration(FixPoint1616_t CalDistanceMilliMeter,int32_t *pOffsetMicroMeter){
  Error Status = ERROR_NONE;
  LOG_FUNCTION_START;

  Status = VL53L0X_perform_offset_calibration(Dev, CalDistanceMilliMeter, pOffsetMicroMeter);


  return Status;
}

Error VL53L0X_CheckAndLoadInterruptSettings(uint8_t StartNotStopFlag){
  uint8_t InterruptConfig;
  FixPoint1616_t ThresholdLow;
  FixPoint1616_t ThresholdHigh;
  Error Status = ERROR_NONE;

  InterruptConfig = GetDEVICESPECIFICPARAMETER(Dev, Pin0GpioFunctionality);

  if ((InterruptConfig == VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW) ||
    (InterruptConfig == VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH) ||
    (InterruptConfig == VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT)) {

    Status = GetInterruptThresholds( Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, &ThresholdLow, &ThresholdHigh);

    if (((ThresholdLow > 255 * 65536) || (ThresholdHigh > 255 * 65536)) && (Status == ERROR_NONE)) {

      if (StartNotStopFlag != 0) {
        Status = VL53L0X_load_tuning_settings(Dev, InterruptThresholdSettings);
      } else {
        Status |= comm.WrByte( 0xFF, 0x04);
        Status |= comm.WrByte( 0x70, 0x00);
        Status |= comm.WrByte( 0xFF, 0x00);
        Status |= comm.WrByte( 0x80, 0x00);
      }
    }
  }

  return Status;
} // VL53L0X_CheckAndLoadInterruptSettings

Error VL53L0X_StartMeasurement(VL53L0X_DEV Dev){
  Error Status = ERROR_NONE;
  VL53L0X_DeviceModes DeviceMode;
  uint8_t Byte;
  uint8_t StartStopByte = REG_SYSRANGE_MODE_START_STOP;
  uint32_t LoopNb;
  LOG_FUNCTION_START;

  /* Get Current DeviceMode */
  GetDeviceMode(Dev, &DeviceMode);

  Status = comm.WrByte( 0x80, 0x01);
  Status = comm.WrByte( 0xFF, 0x01);
  Status = comm.WrByte( 0x00, 0x00);
  Status = comm.WrByte( 0x91, PALDevDataGet(Dev, StopVariable));
  Status = comm.WrByte( 0x00, 0x01);
  Status = comm.WrByte( 0xFF, 0x00);
  Status = comm.WrByte( 0x80, 0x00);

  switch (DeviceMode) {
  case VL53L0X_DEVICEMODE_SINGLE_RANGING:
    Status = comm.WrByte( REG_SYSRANGE_START, 0x01);

    Byte = StartStopByte;
    if (Status == ERROR_NONE) {
      /* Wait until start bit has been cleared */
      LoopNb = 0;
      do {
        if (LoopNb > 0) {
          Status = comm.RdByte( REG_SYSRANGE_START, &Byte);
        }
        LoopNb = LoopNb + 1;
      } while (((Byte & StartStopByte) == StartStopByte) && (Status == ERROR_NONE) && (LoopNb < VL53L0X_DEFAULT_MAX_LOOP));

      if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
        Status = Error_TIME_OUT;
      }
    }
    break;
  case VL53L0X_DEVICEMODE_CONTINUOUS_RANGING:
    /* Back-to-back mode */

    /* Check if need to apply interrupt settings */
    if (Status == ERROR_NONE) {
      Status = VL53L0X_CheckAndLoadInterruptSettings(Dev, 1);
    }

    Status = comm.WrByte( REG_SYSRANGE_START, REG_SYSRANGE_MODE_BACKTOBACK);
    if (Status == ERROR_NONE) {
      /* Set PAL State to Running */
      PALDevDataSet(this, PalState, State_RUNNING);
    }
    break;
  case VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING:
    /* Continuous mode
     * Check if need to apply interrupt settings*/
    if (Status == ERROR_NONE) {
      Status = VL53L0X_CheckAndLoadInterruptSettings(Dev, 1);
    }

    Status = comm.WrByte( REG_SYSRANGE_START, REG_SYSRANGE_MODE_TIMED);

    if (Status == ERROR_NONE) {
      /* Set PAL State to Running */
      PALDevDataSet(this, PalState, State_RUNNING);
    }
    break;
  default:
    /* Selected mode not supported */
    Status = Error_MODE_NOT_SUPPORTED;
  } // switch


  return Status;
} // VL53L0X_StartMeasurement

Error VL53L0X_StopMeasurement(VL53L0X_DEV Dev){
  Error Status = ERROR_NONE;
  LOG_FUNCTION_START;

  Status = comm.WrByte( REG_SYSRANGE_START, REG_SYSRANGE_MODE_SINGLESHOT);

  Status = comm.WrByte( 0xFF, 0x01);
  Status = comm.WrByte( 0x00, 0x00);
  Status = comm.WrByte( 0x91, 0x00);
  Status = comm.WrByte( 0x00, 0x01);
  Status = comm.WrByte( 0xFF, 0x00);

  if (Status == ERROR_NONE) {
    /* Set PAL State to Idle */
    PALDevDataSet(this, PalState, State_IDLE);
  }

  /* Check if need to apply interrupt settings */
  if (Status == ERROR_NONE) {
    Status = VL53L0X_CheckAndLoadInterruptSettings(Dev, 0);
  }


  return Status;
} // VL53L0X_StopMeasurement

Error GetMeasurementDataReady(uint8_t *pMeasurementDataReady){
  Error Status = ERROR_NONE;
  uint8_t SysRangeStatusRegister;
  uint8_t InterruptConfig;
  uint32_t InterruptMask;
  LOG_FUNCTION_START;

  InterruptConfig = GetDEVICESPECIFICPARAMETER(Dev, Pin0GpioFunctionality);

  if (InterruptConfig == REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY) {
    Status = GetInterruptMaskStatus(Dev, &InterruptMask);
    if (InterruptMask == REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY) {
      *pMeasurementDataReady = 1;
    } else {
      *pMeasurementDataReady = 0;
    }
  } else {
    Status = comm.RdByte( REG_RESULT_RANGE_STATUS, &SysRangeStatusRegister);
    if (Status == ERROR_NONE) {
      if (SysRangeStatusRegister & 0x01) {
        *pMeasurementDataReady = 1;
      } else {
        *pMeasurementDataReady = 0;
      }
    }
  }


  return Status;
} // GetMeasurementDataReady

Error VL53L0X_WaitDeviceReadyForNewMeasurement(uint32_t MaxLoop){
  return ERROR_NOT_IMPLEMENTED;
}

Error GetRangingMeasurementData(VL53L0X_RangingMeasurementData_t *pRangingMeasurementData){
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
  VL53L0X_RangingMeasurementData_t LastRangeDataBuffer;

  LOG_FUNCTION_START;

  /*
   * use multi read even if some registers are not useful, result will
   * be more efficient
   * start reading at 0x14 dec20
   * end reading at 0x21 dec33 total 14 bytes to read
   */
  Status = VL53L0X_ReadMulti(Dev, 0x14, localBuffer, 12);

  if (Status == ERROR_NONE) {

    pRangingMeasurementData->ZoneId = 0;    /* Only one zone */
    pRangingMeasurementData->TimeStamp = 0; /* Not Implemented */

    tmpuint16 = VL53L0X_MAKEUINT16(localBuffer[11], localBuffer[10]);
    /* cut1.1 if SYSTEM__RANGE_CONFIG if 1 range is 2bits fractional
     *(format 11.2) else no fractional
     */

    pRangingMeasurementData->MeasurementTimeUsec = 0;

    SignalRate = VL53L0X_FIXPOINT97TOFIXPOINT1616( VL53L0X_MAKEUINT16(localBuffer[7], localBuffer[6]));
    /* peak_signal_count_rate_rtn_mcps */
    pRangingMeasurementData->SignalRateRtnMegaCps = SignalRate;

    AmbientRate = VL53L0X_MAKEUINT16(localBuffer[9], localBuffer[8]);
    pRangingMeasurementData->AmbientRateRtnMegaCps = VL53L0X_FIXPOINT97TOFIXPOINT1616(AmbientRate);

    EffectiveSpadRtnCount = VL53L0X_MAKEUINT16(localBuffer[3], localBuffer[2]);
    /* EffectiveSpadRtnCount is 8.8 format */
    pRangingMeasurementData->EffectiveSpadRtnCount = EffectiveSpadRtnCount;

    DeviceRangeStatus = localBuffer[0];

    /* Get Linearity Corrective Gain */
    LinearityCorrectiveGain = PALDevDataGet(Dev, LinearityCorrectiveGain);

    /* Get ranging configuration */
    RangeFractionalEnable = PALDevDataGet(Dev, RangeFractionalEnable);

    if (LinearityCorrectiveGain != 1000) {
      tmpuint16 = (uint16_t)((LinearityCorrectiveGain * tmpuint16 + 500) / 1000);

      /* Implement Xtalk */
      GetPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps, XTalkCompensationRateMegaCps);
      GetPARAMETERFIELD(Dev, XTalkCompensationEnable, XTalkCompensationEnable);

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
      pRangingMeasurementData->RangeMilliMeter = (uint16_t)((tmpuint16) >> 2);
      pRangingMeasurementData->RangeFractionalPart =
        (uint8_t)((tmpuint16 & 0x03) << 6);
    } else {
      pRangingMeasurementData->RangeMilliMeter = tmpuint16;
      pRangingMeasurementData->RangeFractionalPart = 0;
    }

    /*
     * For a standard definition of RangeStatus, this should
     * return 0 in case of good result after a ranging
     * The range status depends on the device so call a device
     * specific function to obtain the right Status.
     */
    Status |= Get_pal_range_status( Dev, DeviceRangeStatus, SignalRate, EffectiveSpadRtnCount, pRangingMeasurementData, &PalRangeStatus);

    if (Status == ERROR_NONE) {
      pRangingMeasurementData->RangeStatus = PalRangeStatus;
    }
  }

  if (Status == ERROR_NONE) {
    /* Copy last read data into Dev buffer */
    LastRangeDataBuffer = PALDevDataGet(Dev, LastRangeMeasure);

    LastRangeDataBuffer.RangeMilliMeter =  pRangingMeasurementData->RangeMilliMeter;
    LastRangeDataBuffer.RangeFractionalPart =  pRangingMeasurementData->RangeFractionalPart;
    LastRangeDataBuffer.RangeDMaxMilliMeter =  pRangingMeasurementData->RangeDMaxMilliMeter;
    LastRangeDataBuffer.MeasurementTimeUsec =  pRangingMeasurementData->MeasurementTimeUsec;
    LastRangeDataBuffer.SignalRateRtnMegaCps =  pRangingMeasurementData->SignalRateRtnMegaCps;
    LastRangeDataBuffer.AmbientRateRtnMegaCps = pRangingMeasurementData->AmbientRateRtnMegaCps;
    LastRangeDataBuffer.EffectiveSpadRtnCount = pRangingMeasurementData->EffectiveSpadRtnCount;
    LastRangeDataBuffer.RangeStatus = pRangingMeasurementData->RangeStatus;

    PALDevDataSet(this, LastRangeMeasure, LastRangeDataBuffer);
  }


  return Status;
} // GetRangingMeasurementData

Error GetMeasurementRefSignal(FixPoint1616_t *pMeasurementRefSignal){
  Error Status = ERROR_NONE;
  LOG_FUNCTION_START;

  *pMeasurementRefSignal = PALDevDataGet(Dev, LastSignalRefMcps);


  return Status;
}

Error GetHistogramMeasurementData(VL53L0X_HistogramMeasurementData_t *pHistogramMeasurementData){
  Error Status = Error_NOT_IMPLEMENTED;
  LOG_FUNCTION_START;


  return Status;
}

Error VL53L0X_PerformSingleRangingMeasurement(VL53L0X_RangingMeasurementData_t *pRangingMeasurementData){
  Error Status = ERROR_NONE;

  LOG_FUNCTION_START;

  /* This function will do a complete single ranging
   * Here we fix the mode! */
  Status = VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);

  if (Status == ERROR_NONE) {
    Status = VL53L0X_PerformSingleMeasurement(Dev);
  }

  if (Status == ERROR_NONE) {
    Status = GetRangingMeasurementData(Dev, pRangingMeasurementData);
  }

  if (Status == ERROR_NONE) {
    Status = VL53L0X_ClearInterruptMask(Dev, 0);
  }


  return Status;
} // VL53L0X_PerformSingleRangingMeasurement

Error VL53L0X_SetNumberOfROIZones(uint8_t NumberOfROIZones){
  Error Status = ERROR_NONE;

  LOG_FUNCTION_START;

  if (NumberOfROIZones != 1) {
    Status = Error_INVALID_PARAMS;
  }


  return Status;
} // VL53L0X_SetNumberOfROIZones

Error GetNumberOfROIZones(uint8_t *pNumberOfROIZones){
  Error Status = ERROR_NONE;

  LOG_FUNCTION_START;

  *pNumberOfROIZones = 1;


  return Status;
}

Error GetMaxNumberOfROIZones(uint8_t *pMaxNumberOfROIZones){
  Error Status = ERROR_NONE;

  LOG_FUNCTION_START;

  *pMaxNumberOfROIZones = 1;


  return Status;
}

/* End Group PAL Measurement Functions */

Error VL53L0X_SetGpioConfig( uint8_t Pin,VL53L0X_DeviceModes DeviceMode,VL53L0X_GpioFunctionality Functionality,VL53L0X_InterruptPolarity Polarity){
  Error Status = ERROR_NONE;
  uint8_t data;

  LOG_FUNCTION_START;

  if (Pin != 0) {
    Status = Error_GPIO_NOT_EXISTING;
  } else if (DeviceMode == VL53L0X_DEVICEMODE_GPIO_DRIVE) {
    if (Polarity == VL53L0X_INTERRUPTPOLARITY_LOW) {
      data = 0x10;
    } else {
      data = 1;
    }

    Status = comm.WrByte( REG_GPIO_HV_MUX_ACTIVE_HIGH, data);

  } else if (DeviceMode == VL53L0X_DEVICEMODE_GPIO_OSC) {

    Status |= comm.WrByte( 0xff, 0x01);
    Status |= comm.WrByte( 0x00, 0x00);

    Status |= comm.WrByte( 0xff, 0x00);
    Status |= comm.WrByte( 0x80, 0x01);
    Status |= comm.WrByte( 0x85, 0x02);

    Status |= comm.WrByte( 0xff, 0x04);
    Status |= comm.WrByte( 0xcd, 0x00);
    Status |= comm.WrByte( 0xcc, 0x11);

    Status |= comm.WrByte( 0xff, 0x07);
    Status |= comm.WrByte( 0xbe, 0x00);

    Status |= comm.WrByte( 0xff, 0x06);
    Status |= comm.WrByte( 0xcc, 0x09);

    Status |= comm.WrByte( 0xff, 0x00);
    Status |= comm.WrByte( 0xff, 0x01);
    Status |= comm.WrByte( 0x00, 0x00);

  } else {

    if (Status == ERROR_NONE) {
      switch (Functionality) {
      case VL53L0X_GPIOFUNCTIONALITY_OFF:
        data = 0x00;
        break;
      case VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW:
        data = 0x01;
        break;
      case VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH:
        data = 0x02;
        break;
      case VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT:
        data = 0x03;
        break;
      case VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY:
        data = 0x04;
        break;
      default:
        Status = Error_GPIO_FUNCTIONALITY_NOT_SUPPORTED;
      } // switch
    }

    if (Status == ERROR_NONE) {
      Status = comm.WrByte( REG_SYSTEM_INTERRUPT_CONFIG_GPIO, data);
    }

    if (Status == ERROR_NONE) {
      if (Polarity == VL53L0X_INTERRUPTPOLARITY_LOW) {
        data = 0;
      } else {
        data = (uint8_t)(1 << 4);
      }

      Status = VL53L0X_UpdateByte(Dev, REG_GPIO_HV_MUX_ACTIVE_HIGH, 0xEF, data);
    }

    if (Status == ERROR_NONE) {
      VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, Pin0GpioFunctionality, Functionality);
    }

    if (Status == ERROR_NONE) {
      Status = VL53L0X_ClearInterruptMask(Dev, 0);
    }
  }


  return Status;
} // VL53L0X_SetGpioConfig

Error GetGpioConfig( uint8_t Pin,VL53L0X_DeviceModes *pDeviceMode,VL53L0X_GpioFunctionality *pFunctionality,VL53L0X_InterruptPolarity *pPolarity){
  Error Status = ERROR_NONE;
  VL53L0X_GpioFunctionality GpioFunctionality;
  uint8_t data;

  LOG_FUNCTION_START;

  /* pDeviceMode not managed by Ewok it return the current mode */

  Status = GetDeviceMode(Dev, pDeviceMode);

  if (Status == ERROR_NONE) {
    if (Pin != 0) {
      Status = Error_GPIO_NOT_EXISTING;
    } else {
      Status = comm.RdByte( REG_SYSTEM_INTERRUPT_CONFIG_GPIO, &data);
    }
  }

  if (Status == ERROR_NONE) {
    switch (data & 0x07) {
    case 0x00:
      GpioFunctionality = VL53L0X_GPIOFUNCTIONALITY_OFF;
      break;
    case 0x01:
      GpioFunctionality = VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW;
      break;
    case 0x02:
      GpioFunctionality = VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH;
      break;
    case 0x03:
      GpioFunctionality = VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT;
      break;
    case 0x04:
      GpioFunctionality = VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY;
      break;
    default:
      Status = Error_GPIO_FUNCTIONALITY_NOT_SUPPORTED;
    } // switch
  }

  if (Status == ERROR_NONE) {
    Status = comm.RdByte( REG_GPIO_HV_MUX_ACTIVE_HIGH, &data);
  }

  if (Status == ERROR_NONE) {
    if ((data & (uint8_t)(1 << 4)) == 0) {
      *pPolarity = VL53L0X_INTERRUPTPOLARITY_LOW;
    } else {
      *pPolarity = VL53L0X_INTERRUPTPOLARITY_HIGH;
    }
  }

  if (Status == ERROR_NONE) {
    *pFunctionality = GpioFunctionality;
    VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, Pin0GpioFunctionality, GpioFunctionality);
  }


  return Status;
} // GetGpioConfig

Error VL53L0X_SetInterruptThresholds(VL53L0X_DeviceModes DeviceMode,FixPoint1616_t ThresholdLow,FixPoint1616_t ThresholdHigh){
  Error Status = ERROR_NONE;
  uint16_t Threshold16;
  LOG_FUNCTION_START;

  /* no dependency on DeviceMode for Ewok
   * Need to divide by 2 because the FW will apply a x2*/
  Threshold16 = (uint16_t)((ThresholdLow >> 17) & 0x00fff);
  Status = VL53L0X_WrWord(Dev, REG_SYSTEM_THRESH_LOW, Threshold16);

  if (Status == ERROR_NONE) {
    /* Need to divide by 2 because the FW will apply a x2 */
    Threshold16 = (uint16_t)((ThresholdHigh >> 17) & 0x00fff);
    Status = VL53L0X_WrWord(Dev, REG_SYSTEM_THRESH_HIGH, Threshold16);
  }


  return Status;
} // VL53L0X_SetInterruptThresholds

Error GetInterruptThresholds(VL53L0X_DeviceModes DeviceMode,FixPoint1616_t *pThresholdLow,FixPoint1616_t *pThresholdHigh){
  Error Status = ERROR_NONE;
  uint16_t Threshold16;
  LOG_FUNCTION_START;

  /* no dependency on DeviceMode for Ewok */

  Status = VL53L0X_RdWord(Dev, REG_SYSTEM_THRESH_LOW, &Threshold16);
  /* Need to multiply by 2 because the FW will apply a x2 */
  *pThresholdLow = (FixPoint1616_t)((0x00fff & Threshold16) << 17);

  if (Status == ERROR_NONE) {
    Status = VL53L0X_RdWord(Dev, REG_SYSTEM_THRESH_HIGH, &Threshold16);
    /* Need to multiply by 2 because the FW will apply a x2 */
    *pThresholdHigh = (FixPoint1616_t)((0x00fff & Threshold16) << 17);
  }


  return Status;
} // GetInterruptThresholds

  Erroneous<uint8_t> Api::GetStopCompletedStatus(){
  LOG_FUNCTION_START;

  Erroneous<uint8_t>Byte; {
    auto pager =autoCloser(Private_Pager,1,0);
    fetch(Byte,Private_04);
  }

  if (Byte == 0) {
    auto magic=magicWrapper();
    comm.WrByte( 0x91, PALDevDataGet(this, StopVariable));//errors ignored.
  }

  return Byte;
} // GetStopCompletedStatus

/* Group PAL Interrupt Functions */
Error VL53L0X_ClearInterruptMask(uint32_t InterruptMask){
  Error Status = ERROR_NONE;
  uint8_t LoopCount;
  uint8_t Byte;
  LOG_FUNCTION_START;

  /* clear bit 0 range interrupt, bit 1 error interrupt */
  LoopCount = 0;
  do {
    Status = comm.WrByte( REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    Status |= comm.WrByte( REG_SYSTEM_INTERRUPT_CLEAR, 0x00);
    Status |= comm.RdByte( REG_RESULT_INTERRUPT_STATUS, &Byte);
    LoopCount++;
  } while (((Byte & 0x07) != 0x00) && (LoopCount < 3) && (Status == ERROR_NONE));

  if (LoopCount >= 3) {
    Status = Error_INTERRUPT_NOT_CLEARED;
  }


  return Status;
} // VL53L0X_ClearInterruptMask

Error GetInterruptMaskStatus(uint32_t *pInterruptMaskStatus){
  LOG_FUNCTION_START;
  uint8_t Byte;
  Error Status = comm.RdByte( REG_RESULT_INTERRUPT_STATUS, &Byte);
  *pInterruptMaskStatus = Byte & 0x07;

  if (Byte & 0x18) {//if either bit? what are each of them?
    Status = Error_RANGE_ERROR;
  }

  return Status;
} // GetInterruptMaskStatus

Error VL53L0X_EnableInterruptMask(uint32_t InterruptMask){
  return Error_NOT_IMPLEMENTED;
}

/* End Group PAL Interrupt Functions */

/* Group SPAD functions */

Error VL53L0X_SetSpadAmbientDamperThreshold(uint16_t SpadAmbientDamperThreshold){
  LOG_FUNCTION_START;
  Error Status = comm.WrByte( 0xFF, 0x01);
  Status |= VL53L0X_WrWord(Dev, 0x40, SpadAmbientDamperThreshold);
  Status |= comm.WrByte( 0xFF, 0x00);

  return Status;
} // VL53L0X_SetSpadAmbientDamperThreshold

Error GetSpadAmbientDamperThreshold(uint16_t *pSpadAmbientDamperThreshold){
  LOG_FUNCTION_START;
  Error Status = comm.WrByte( 0xFF, 0x01);
  Status |= VL53L0X_RdWord(Dev, 0x40, pSpadAmbientDamperThreshold);
  Status |= comm.WrByte( 0xFF, 0x00);

  return Status;
} // GetSpadAmbientDamperThreshold

Error VL53L0X_SetSpadAmbientDamperFactor(uint16_t SpadAmbientDamperFactor){
  LOG_FUNCTION_START;
  Error Status = comm.WrByte( 0xFF, 0x01);
  Status |= comm.WrByte( 0x42, (uint8_t) (SpadAmbientDamperFactor & 0x00FF));
  Status |= comm.WrByte( 0xFF, 0x00);

  return Status;
} // VL53L0X_SetSpadAmbientDamperFactor

Error GetSpadAmbientDamperFactor(uint16_t *pSpadAmbientDamperFactor){
  LOG_FUNCTION_START;

  Error Status = comm.WrByte( 0xFF, 0x01);
  uint8_t Byte;
  Status |= comm.RdByte( 0x42, &Byte);
  Status |= comm.WrByte( 0xFF, 0x00);
  *pSpadAmbientDamperFactor = (uint16_t)Byte;


  return Status;
} // GetSpadAmbientDamperFactor

/* END Group SPAD functions */

/*****************************************************************************
* Internal functions
*****************************************************************************/

Error VL53L0X_SetReferenceSpads( uint32_t count,uint8_t isApertureSpads){
  LOG_FUNCTION_START;
  Error Status = VL53L0X_set_reference_spads(Dev, count, isApertureSpads);

  return Status;
}

Error GetReferenceSpads( uint32_t *pSpadCount,uint8_t *pIsApertureSpads){
  LOG_FUNCTION_START;
  Error Status = Get_reference_spads(Dev, pSpadCount, pIsApertureSpads);

  return Status;
}

Error VL53L0X_PerformRefSpadManagement(uint32_t *refSpadCount,uint8_t *isApertureSpads){
  LOG_FUNCTION_START;
  Error Status = VL53L0X_perform_ref_spad_management(Dev, refSpadCount, isApertureSpads);

  return Status;
} // VL53L0X_PerformRefSpadManagement
