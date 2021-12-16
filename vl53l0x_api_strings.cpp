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
//ick: the bone headed 'every function returns error' even when errors are not conceivable causes this module to be an expensive mistake versus a few maps of strings that won't require a copy to buffer but could be passed as pointers into printf and the like.
#include "vl53l0x_api_strings.h"
#include "vl53l0x_api_core.h"

#include "log_api.h"

#define COPYSTRING(target,string) strcpy(target,string)

namespace VL53L0X {

  Error check_part_used(uint8_t *Revision, DeviceInfo_t *pDeviceInfo) {
    LOG_FUNCTION_START
    ("");
    Error Status = get_info_from_device(Dev, 2);

    if (Status == ERROR_NONE) {
      if (GETDEVICESPECIFICPARAMETER(Dev, ModuleId) == 0) {
        *Revision = 0;
        COPYSTRING(pDeviceInfo->ProductId, "");
      } else {
        *Revision = GETDEVICESPECIFICPARAMETER(Dev, Revision);
        COPYSTRING(pDeviceInfo->ProductId, GETDEVICESPECIFICPARAMETER(Dev, ProductId));
      }
    }

    return Status;
  } // check_part_used

  Error get_device_info(DeviceInfo_t *pDeviceInfo) {
    uint8_t Revision;
    Error Error = check_part_used( &Revision, pDeviceInfo);
    ERROR_OUT;
    if (Revision == 0) {
      COPYSTRING(pDeviceInfo->Name, VL53L0X_STRING_DEVICE_INFO_NAME_TS0);
    } else if ((Revision <= 34) && (Revision != 32)) {
      COPYSTRING(pDeviceInfo->Name,VL53L0X_STRING_DEVICE_INFO_NAME_TS1);
    } else if (Revision < 39) {
      COPYSTRING(pDeviceInfo->Name,VL53L0X_STRING_DEVICE_INFO_NAME_TS2);
    } else {
      COPYSTRING(pDeviceInfo->Name,VL53L0X_STRING_DEVICE_INFO_NAME_ES1);
    }
    COPYSTRING(pDeviceInfo->Type, VL53L0X_STRING_DEVICE_INFO_TYPE);
    Error = RdByte(Dev, REG_IDENTIFICATION_MODEL_ID, &pDeviceInfo->ProductType);
    ERROR_OUT;
    uint8_t revision_id;
    Error = RdByte(Dev, REG_IDENTIFICATION_REVISION_ID, &revision_id);
    pDeviceInfo->ProductRevisionMajor = 1;
    pDeviceInfo->ProductRevisionMinor = (revision_id & 0xF0) >> 4; //BUG: rev id set even if read fails.

    return Error;
  } // get_device_info

  Error get_device_error_string(DeviceError ErrorCode, char *pDeviceErrorString) {
    LOG_FUNCTION_START
    ("");

    switch (ErrorCode) {
      case DEVICEERROR_NONE:
        COPYSTRING(pDeviceErrorString, VL53L0X_STRING_DEVICEERROR_NONE);
        break;
      case DEVICEERROR_VCSELCONTINUITYTESTFAILURE:
        COPYSTRING(pDeviceErrorString, VL53L0X_STRING_DEVICEERROR_VCSELCONTINUITYTESTFAILURE);
        break;
      case DEVICEERROR_VCSELWATCHDOGTESTFAILURE:
        COPYSTRING(pDeviceErrorString, VL53L0X_STRING_DEVICEERROR_VCSELWATCHDOGTESTFAILURE);
        break;
      case DEVICEERROR_NOVHVVALUEFOUND:
        COPYSTRING(pDeviceErrorString, VL53L0X_STRING_DEVICEERROR_NOVHVVALUEFOUND);
        break;
      case DEVICEERROR_MSRCNOTARGET:
        COPYSTRING(pDeviceErrorString, VL53L0X_STRING_DEVICEERROR_MSRCNOTARGET);
        break;
      case DEVICEERROR_SNRCHECK:
        COPYSTRING(pDeviceErrorString, VL53L0X_STRING_DEVICEERROR_SNRCHECK);
        break;
      case DEVICEERROR_RANGEPHASECHECK:
        COPYSTRING(pDeviceErrorString, VL53L0X_STRING_DEVICEERROR_RANGEPHASECHECK);
        break;
      case DEVICEERROR_SIGMATHRESHOLDCHECK:
        COPYSTRING(pDeviceErrorString, VL53L0X_STRING_DEVICEERROR_SIGMATHRESHOLDCHECK);
        break;
      case DEVICEERROR_TCC:
        COPYSTRING(pDeviceErrorString, VL53L0X_STRING_DEVICEERROR_TCC);
        break;
      case DEVICEERROR_PHASECONSISTENCY:
        COPYSTRING(pDeviceErrorString, VL53L0X_STRING_DEVICEERROR_PHASECONSISTENCY);
        break;
      case DEVICEERROR_MINCLIP:
        COPYSTRING(pDeviceErrorString, VL53L0X_STRING_DEVICEERROR_MINCLIP);
        break;
      case DEVICEERROR_RANGECOMPLETE:
        COPYSTRING(pDeviceErrorString, VL53L0X_STRING_DEVICEERROR_RANGECOMPLETE);
        break;
      case DEVICEERROR_ALGOUNDERFLOW:
        COPYSTRING(pDeviceErrorString, VL53L0X_STRING_DEVICEERROR_ALGOUNDERFLOW);
        break;
      case DEVICEERROR_ALGOOVERFLOW:
        COPYSTRING(pDeviceErrorString, VL53L0X_STRING_DEVICEERROR_ALGOOVERFLOW);
        break;
      case DEVICEERROR_RANGEIGNORETHRESHOLD:
        COPYSTRING(pDeviceErrorString, VL53L0X_STRING_DEVICEERROR_RANGEIGNORETHRESHOLD);
        break;
      default:
        COPYSTRING(pDeviceErrorString, VL53L0X_STRING_UNKNOW_ERROR_CODE);
    } // switch


    return ERROR_NONE;
  } // get_device_error_string

  Error get_range_status_string(uint8_t RangeStatus, char *pRangeStatusString) {

    LOG_FUNCTION_START
    ("");

    switch (RangeStatus) {
      case 0:
        COPYSTRING(pRangeStatusString, VL53L0X_STRING_RANGESTATUS_RANGEVALID);
        break;
      case 1:
        COPYSTRING(pRangeStatusString, VL53L0X_STRING_RANGESTATUS_SIGMA);
        break;
      case 2:
        COPYSTRING(pRangeStatusString, VL53L0X_STRING_RANGESTATUS_SIGNAL);
        break;
      case 3:
        COPYSTRING(pRangeStatusString, VL53L0X_STRING_RANGESTATUS_MINRANGE);
        break;
      case 4:
        COPYSTRING(pRangeStatusString, VL53L0X_STRING_RANGESTATUS_PHASE);
        break;
      case 5:
        COPYSTRING(pRangeStatusString, VL53L0X_STRING_RANGESTATUS_HW);
        break;

      default: /**/
        COPYSTRING(pRangeStatusString, VL53L0X_STRING_RANGESTATUS_NONE);
    } // switch

    return ERROR_NONE;
  } // get_range_status_string

  Error get_pal_error_string(Error PalErrorCode, char *pPalErrorString) {

    LOG_FUNCTION_START
    ("");

    switch (PalErrorCode) {
      case ERROR_NONE:
        COPYSTRING(pPalErrorString, VL53L0X_STRING_ERROR_NONE);
        break;
      case ERROR_CALIBRATION_WARNING:
        COPYSTRING(pPalErrorString, VL53L0X_STRING_ERROR_CALIBRATION_WARNING);
        break;
      case ERROR_MIN_CLIPPED:
        COPYSTRING(pPalErrorString, VL53L0X_STRING_ERROR_MIN_CLIPPED);
        break;
      case ERROR_UNDEFINED:
        COPYSTRING(pPalErrorString, VL53L0X_STRING_ERROR_UNDEFINED);
        break;
      case ERROR_INVALID_PARAMS:
        COPYSTRING(pPalErrorString, VL53L0X_STRING_ERROR_INVALID_PARAMS);
        break;
      case ERROR_NOT_SUPPORTED:
        COPYSTRING(pPalErrorString, VL53L0X_STRING_ERROR_NOT_SUPPORTED);
        break;
      case ERROR_INTERRUPT_NOT_CLEARED:
        COPYSTRING(pPalErrorString, VL53L0X_STRING_ERROR_INTERRUPT_NOT_CLEARED);
        break;
      case ERROR_RANGE_ERROR:
        COPYSTRING(pPalErrorString, VL53L0X_STRING_ERROR_RANGE_ERROR);
        break;
      case ERROR_TIME_OUT:
        COPYSTRING(pPalErrorString, VL53L0X_STRING_ERROR_TIME_OUT);
        break;
      case ERROR_MODE_NOT_SUPPORTED:
        COPYSTRING(pPalErrorString, VL53L0X_STRING_ERROR_MODE_NOT_SUPPORTED);
        break;
      case ERROR_BUFFER_TOO_SMALL:
        COPYSTRING(pPalErrorString, VL53L0X_STRING_ERROR_BUFFER_TOO_SMALL);
        break;
      case ERROR_GPIO_NOT_EXISTING:
        COPYSTRING(pPalErrorString, VL53L0X_STRING_ERROR_GPIO_NOT_EXISTING);
        break;
      case ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED:
        COPYSTRING(pPalErrorString, VL53L0X_STRING_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED);
        break;
      case ERROR_CONTROL_INTERFACE:
        COPYSTRING(pPalErrorString, VL53L0X_STRING_ERROR_CONTROL_INTERFACE);
        break;
      case ERROR_INVALID_COMMAND:
        COPYSTRING(pPalErrorString, VL53L0X_STRING_ERROR_INVALID_COMMAND);
        break;
      case ERROR_DIVISION_BY_ZERO:
        COPYSTRING(pPalErrorString, VL53L0X_STRING_ERROR_DIVISION_BY_ZERO);
        break;
      case ERROR_REF_SPAD_INIT:
        COPYSTRING(pPalErrorString, VL53L0X_STRING_ERROR_REF_SPAD_INIT);
        break;
      case ERROR_NOT_IMPLEMENTED:
        COPYSTRING(pPalErrorString, VL53L0X_STRING_ERROR_NOT_IMPLEMENTED);
        break;

      default:
        COPYSTRING(pPalErrorString, VL53L0X_STRING_UNKNOW_ERROR_CODE);
    } // switch
    return ERROR_NONE;
  } // get_pal_error_string

  Error get_pal_state_string(State PalStateCode, char *pPalStateString) {

    switch (PalStateCode) {
      case STATE_POWERDOWN:
        COPYSTRING(pPalStateString, VL53L0X_STRING_STATE_POWERDOWN);
        break;
      case STATE_WAIT_STATICINIT:
        COPYSTRING(pPalStateString, VL53L0X_STRING_STATE_WAIT_STATICINIT);
        break;
      case STATE_STANDBY:
        COPYSTRING(pPalStateString, VL53L0X_STRING_STATE_STANDBY);
        break;
      case STATE_IDLE:
        COPYSTRING(pPalStateString, VL53L0X_STRING_STATE_IDLE);
        break;
      case STATE_RUNNING:
        COPYSTRING(pPalStateString, VL53L0X_STRING_STATE_RUNNING);
        break;
      case STATE_UNKNOWN:
        COPYSTRING(pPalStateString, VL53L0X_STRING_STATE_UNKNOWN);
        break;
      case STATE_ERROR:
        COPYSTRING(pPalStateString, VL53L0X_STRING_STATE_ERROR);
        break;

      default:
        COPYSTRING(pPalStateString, VL53L0X_STRING_STATE_UNKNOWN);
    } // switch

    return ERROR_NONE;
  } // get_pal_state_string

  Error get_sequence_steps_info(SequenceStepId SequenceStepId, char *pSequenceStepsString) {
    switch (SequenceStepId) {
      case SEQUENCESTEP_TCC:
        COPYSTRING(pSequenceStepsString, VL53L0X_STRING_SEQUENCESTEP_TCC);
        break;
      case SEQUENCESTEP_DSS:
        COPYSTRING(pSequenceStepsString, VL53L0X_STRING_SEQUENCESTEP_DSS);
        break;
      case SEQUENCESTEP_MSRC:
        COPYSTRING(pSequenceStepsString, VL53L0X_STRING_SEQUENCESTEP_MSRC);
        break;
      case SEQUENCESTEP_PRE_RANGE:
        COPYSTRING(pSequenceStepsString, VL53L0X_STRING_SEQUENCESTEP_PRE_RANGE);
        break;
      case SEQUENCESTEP_FINAL_RANGE:
        COPYSTRING(pSequenceStepsString, VL53L0X_STRING_SEQUENCESTEP_FINAL_RANGE);
        break;

      default:
        return ERROR_INVALID_PARAMS;
    } // switch
    return ERROR_NONE;
  } // get_sequence_steps_info

  Error get_limit_check_info(uint16_t LimitCheckId, char *pLimitCheckString) {
    switch (LimitCheckId) {
      case CHECKENABLE_SIGMA_FINAL_RANGE:
        COPYSTRING(pLimitCheckString, VL53L0X_STRING_CHECKENABLE_SIGMA_FINAL_RANGE);
        break;
      case CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
        COPYSTRING(pLimitCheckString, VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE);
        break;
      case CHECKENABLE_SIGNAL_REF_CLIP:
        COPYSTRING(pLimitCheckString, VL53L0X_STRING_CHECKENABLE_SIGNAL_REF_CLIP);
        break;
      case CHECKENABLE_RANGE_IGNORE_THRESHOLD:
        COPYSTRING(pLimitCheckString, VL53L0X_STRING_CHECKENABLE_RANGE_IGNORE_THRESHOLD);
        break;

      case CHECKENABLE_SIGNAL_RATE_MSRC:
        COPYSTRING(pLimitCheckString, VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_MSRC);
        break;

      case CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
        COPYSTRING(pLimitCheckString, VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_PRE_RANGE);
        break;

      default:
        COPYSTRING(pLimitCheckString, VL53L0X_STRING_UNKNOW_ERROR_CODE);
    } // switch
    return ERROR_NONE;
  } // get_limit_check_info
} //end namespace
