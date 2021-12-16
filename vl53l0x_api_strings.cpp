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


#include "vl53l0x_api_strings.h" //translatable parts

#include "log_api.h"

#ifndef countof
#define countof(array) (sizeof (array)/sizeof (*array))
#endif

//BUG: platform specific option should not be in this file
#define COPYSTRING(target, string) strcpy(target,string)

namespace VL53L0X {

  struct Enumified {
    uint8_t code;
    const char *text;
  };

  const char *get_from_Table(const Enumified table[], uint8_t code, const char *ifnotfound = nullptr) {
    unsigned i = 0;
    for (; table[i].text != nullptr; ++i) {
      if (table[i].code == code) {
        return table[i].text;
      }
    }
    return nullptr;
  }

  Error copyit(const char *text, char *pDeviceErrorString) {
    if (text) {
      COPYSTRING(pDeviceErrorString, text);
      return ERROR_NONE;
    } else {
      return ERROR_INVALID_PARAMS;
    }
  }

  static const Enumified DEVICEERROR_table[] = {
    {DEVICEERROR_NONE,                       VL53L0X_STRING_DEVICEERROR_NONE},
    {DEVICEERROR_VCSELCONTINUITYTESTFAILURE, VL53L0X_STRING_DEVICEERROR_VCSELCONTINUITYTESTFAILURE},
    {DEVICEERROR_VCSELWATCHDOGTESTFAILURE,   VL53L0X_STRING_DEVICEERROR_VCSELWATCHDOGTESTFAILURE},
    {DEVICEERROR_NOVHVVALUEFOUND,            VL53L0X_STRING_DEVICEERROR_NOVHVVALUEFOUND},
    {DEVICEERROR_MSRCNOTARGET,               VL53L0X_STRING_DEVICEERROR_MSRCNOTARGET},
    {DEVICEERROR_SNRCHECK,                   VL53L0X_STRING_DEVICEERROR_SNRCHECK},
    {DEVICEERROR_RANGEPHASECHECK,            VL53L0X_STRING_DEVICEERROR_RANGEPHASECHECK},
    {DEVICEERROR_SIGMATHRESHOLDCHECK,        VL53L0X_STRING_DEVICEERROR_SIGMATHRESHOLDCHECK},
    {DEVICEERROR_TCC,                        VL53L0X_STRING_DEVICEERROR_TCC},
    {DEVICEERROR_PHASECONSISTENCY,           VL53L0X_STRING_DEVICEERROR_PHASECONSISTENCY},
    {DEVICEERROR_MINCLIP,                    VL53L0X_STRING_DEVICEERROR_MINCLIP},
    {DEVICEERROR_RANGECOMPLETE,              VL53L0X_STRING_DEVICEERROR_RANGECOMPLETE},
    {DEVICEERROR_ALGOUNDERFLOW,              VL53L0X_STRING_DEVICEERROR_ALGOUNDERFLOW},
    {DEVICEERROR_ALGOOVERFLOW,               VL53L0X_STRING_DEVICEERROR_ALGOOVERFLOW},
    {DEVICEERROR_RANGEIGNORETHRESHOLD,       VL53L0X_STRING_DEVICEERROR_RANGEIGNORETHRESHOLD},
    {0, nullptr}  // code is ignored, OK if overlaps valid values.
  };

  const char *device_error_string(DeviceError ErrorCode) {
    return get_from_Table(DEVICEERROR_table, ErrorCode, VL53L0X_STRING_UNKNOW_ERROR_CODE);
  }

  Error get_device_error_string(DeviceError ErrorCode, char *pDeviceErrorString) {
    return copyit(device_error_string(ErrorCode), pDeviceErrorString);
  } // get_device_error_string



  static const Enumified RANGESTATUS_table[] = {
    {0, VL53L0X_STRING_RANGESTATUS_RANGEVALID},
    {1, VL53L0X_STRING_RANGESTATUS_SIGMA},
    {2, VL53L0X_STRING_RANGESTATUS_SIGNAL},
    {3, VL53L0X_STRING_RANGESTATUS_MINRANGE},
    {4, VL53L0X_STRING_RANGESTATUS_PHASE},
    {5, VL53L0X_STRING_RANGESTATUS_HW},
    {0, 0}
  };

  Error get_range_status_string(uint8_t RangeStatus, char *pRangeStatusString) {
    return copyit(get_from_Table(RANGESTATUS_table, RangeStatus, VL53L0X_STRING_RANGESTATUS_NONE), pRangeStatusString);
  } // get_range_status_string

  static const Enumified PalERROR_table[] = {
    {ERROR_NONE, VL53L0X_STRING_ERROR_NONE},
    {ERROR_CALIBRATION_WARNING, VL53L0X_STRING_ERROR_CALIBRATION_WARNING},
    {ERROR_MIN_CLIPPED, VL53L0X_STRING_ERROR_MIN_CLIPPED},
    {ERROR_UNDEFINED, VL53L0X_STRING_ERROR_UNDEFINED}, {ERROR_INVALID_PARAMS, VL53L0X_STRING_ERROR_INVALID_PARAMS}, {ERROR_NOT_SUPPORTED, VL53L0X_STRING_ERROR_NOT_SUPPORTED}, {ERROR_INTERRUPT_NOT_CLEARED, VL53L0X_STRING_ERROR_INTERRUPT_NOT_CLEARED}, {ERROR_RANGE_ERROR, VL53L0X_STRING_ERROR_RANGE_ERROR}, {ERROR_TIME_OUT, VL53L0X_STRING_ERROR_TIME_OUT}, {ERROR_MODE_NOT_SUPPORTED, VL53L0X_STRING_ERROR_MODE_NOT_SUPPORTED}, {ERROR_BUFFER_TOO_SMALL, VL53L0X_STRING_ERROR_BUFFER_TOO_SMALL}, {ERROR_GPIO_NOT_EXISTING, VL53L0X_STRING_ERROR_GPIO_NOT_EXISTING}, {ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED, VL53L0X_STRING_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED}, {ERROR_CONTROL_INTERFACE, VL53L0X_STRING_ERROR_CONTROL_INTERFACE}, {ERROR_INVALID_COMMAND, VL53L0X_STRING_ERROR_INVALID_COMMAND}, {ERROR_DIVISION_BY_ZERO, VL53L0X_STRING_ERROR_DIVISION_BY_ZERO},
    {ERROR_REF_SPAD_INIT, VL53L0X_STRING_ERROR_REF_SPAD_INIT},
    {ERROR_NOT_IMPLEMENTED, VL53L0X_STRING_ERROR_NOT_IMPLEMENTED},
    {0, 0}
  };

  Error get_pal_error_string(Error PalErrorCode, char *pPalErrorString) {
    return copyit(get_from_Table(RANGESTATUS_table, PalErrorCode, VL53L0X_STRING_UNKNOW_ERROR_CODE), pPalErrorString);
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
