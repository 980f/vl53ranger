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

#include "vl53l0x_api_strings.h" //translatable parts

//logging was removed as there is no good reason to check performance on functions only called to produce error or logging messages.

#ifndef countof
#define countof(array) (sizeof (array) / sizeof (*array))
#endif


namespace VL53L0X {

  struct Enumified {
    uint8_t code;
    const char *text;
  };

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
    {0, nullptr}    // code is ignored, OK if overlaps valid values.
  };

  static const Enumified RANGESTATUS_table[] = {
    {0, VL53L0X_STRING_RANGESTATUS_RANGEVALID},
    {1, VL53L0X_STRING_RANGESTATUS_SIGMA},
    {2, VL53L0X_STRING_RANGESTATUS_SIGNAL},
    {3, VL53L0X_STRING_RANGESTATUS_MINRANGE},
    {4, VL53L0X_STRING_RANGESTATUS_PHASE},
    {5, VL53L0X_STRING_RANGESTATUS_HW},
    {0, 0}
  };

  static const Enumified PalERROR_table[] = {
    {ERROR_NONE,                             VL53L0X_STRING_ERROR_NONE},
    {ERROR_CALIBRATION_WARNING,              VL53L0X_STRING_ERROR_CALIBRATION_WARNING},
    {ERROR_MIN_CLIPPED,                      VL53L0X_STRING_ERROR_MIN_CLIPPED},
    {ERROR_UNDEFINED,                        VL53L0X_STRING_ERROR_UNDEFINED},
    {ERROR_INVALID_PARAMS,                   VL53L0X_STRING_ERROR_INVALID_PARAMS},
    {ERROR_NOT_SUPPORTED,                    VL53L0X_STRING_ERROR_NOT_SUPPORTED},
    {ERROR_INTERRUPT_NOT_CLEARED,            VL53L0X_STRING_ERROR_INTERRUPT_NOT_CLEARED},
    {ERROR_RANGE_ERROR,                      VL53L0X_STRING_ERROR_RANGE_ERROR},
    {ERROR_TIME_OUT,                         VL53L0X_STRING_ERROR_TIME_OUT},
    {ERROR_MODE_NOT_SUPPORTED,               VL53L0X_STRING_ERROR_MODE_NOT_SUPPORTED},
    {ERROR_BUFFER_TOO_SMALL,                 VL53L0X_STRING_ERROR_BUFFER_TOO_SMALL},
    {ERROR_GPIO_NOT_EXISTING,                VL53L0X_STRING_ERROR_GPIO_NOT_EXISTING},
    {ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED, VL53L0X_STRING_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED},
    {ERROR_CONTROL_INTERFACE,                VL53L0X_STRING_ERROR_CONTROL_INTERFACE},
    {ERROR_INVALID_COMMAND,                  VL53L0X_STRING_ERROR_INVALID_COMMAND},
    {ERROR_DIVISION_BY_ZERO,                 VL53L0X_STRING_ERROR_DIVISION_BY_ZERO},
    {ERROR_REF_SPAD_INIT,                    VL53L0X_STRING_ERROR_REF_SPAD_INIT},
    {ERROR_NOT_IMPLEMENTED,                  VL53L0X_STRING_ERROR_NOT_IMPLEMENTED},

    {0, 0}
  };

  static const Enumified STATE_table[] = {
    {STATE_POWERDOWN,       VL53L0X_STRING_STATE_POWERDOWN},
    {STATE_WAIT_STATICINIT, VL53L0X_STRING_STATE_WAIT_STATICINIT},
    {STATE_STANDBY,         VL53L0X_STRING_STATE_STANDBY},
    {STATE_IDLE,            VL53L0X_STRING_STATE_IDLE},
    {STATE_RUNNING,         VL53L0X_STRING_STATE_RUNNING},
    {STATE_UNKNOWN,         VL53L0X_STRING_STATE_UNKNOWN},
    {STATE_ERROR,           VL53L0X_STRING_STATE_ERROR},

    {0, nullptr}
  };

  static const Enumified SEQUENCESTEP_table[] = {
    {SEQUENCESTEP_TCC,         VL53L0X_STRING_SEQUENCESTEP_TCC},
    {SEQUENCESTEP_DSS,         VL53L0X_STRING_SEQUENCESTEP_DSS},
    {SEQUENCESTEP_MSRC,        VL53L0X_STRING_SEQUENCESTEP_MSRC},
    {SEQUENCESTEP_PRE_RANGE,   VL53L0X_STRING_SEQUENCESTEP_PRE_RANGE},
    {SEQUENCESTEP_FINAL_RANGE, VL53L0X_STRING_SEQUENCESTEP_FINAL_RANGE},

    {0, nullptr}
  };

  static const Enumified CHECKENABLE_table[] = {
    {CHECKENABLE_SIGMA_FINAL_RANGE,       VL53L0X_STRING_CHECKENABLE_SIGMA_FINAL_RANGE},
    {CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE},
    {CHECKENABLE_SIGNAL_REF_CLIP,         VL53L0X_STRING_CHECKENABLE_SIGNAL_REF_CLIP},
    {CHECKENABLE_RANGE_IGNORE_THRESHOLD,  VL53L0X_STRING_CHECKENABLE_RANGE_IGNORE_THRESHOLD},
    {CHECKENABLE_SIGNAL_RATE_MSRC,        VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_MSRC},
    {CHECKENABLE_SIGNAL_RATE_PRE_RANGE,   VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_PRE_RANGE},

    {0, nullptr}
  };

  static const char *scanTable(const Enumified table[], uint8_t code, const char *ifnotfound = nullptr) {
    for (unsigned i = 0; table[i].text != nullptr; ++i) { //linear search is fast enough for human visual consumables.
      if (table[i].code == code) {
        return table[i].text;
      }
    }
    return ifnotfound;
  }

  static Error copyit(const char *text, char *pDeviceErrorString) {
    if (text) {
      COPYSTRING(pDeviceErrorString, text);
      return ERROR_NONE;
    } else {
      return ERROR_INVALID_PARAMS;
    }
  }

  //// new approach, point to it so that printf etc can pick up the data without intermediate buffer or copy
  const char *device_error_string(DeviceError ErrorCode) {
    return scanTable(DEVICEERROR_table, ErrorCode, VL53L0X_STRING_UNKNOW_ERROR_CODE);
  }

  const char *range_status_string(uint8_t RangeStatus) {
    return scanTable(RANGESTATUS_table, RangeStatus, VL53L0X_STRING_RANGESTATUS_NONE);
  }

  const char *pal_error_string(const Error &PalErrorCode) {
    return scanTable(RANGESTATUS_table, PalErrorCode, VL53L0X_STRING_UNKNOW_ERROR_CODE);
  }

  const char *pal_state_string(const State &PalStateCode) {
    return scanTable(STATE_table, PalStateCode, VL53L0X_STRING_STATE_UNKNOWN);
  }

  const char *sequence_steps_info(const SequenceStepId &SequenceStepId) {
    return scanTable(SEQUENCESTEP_table, SequenceStepId); //odd man out: returns error instead of filling a string.
  }

  const char *limit_check_info(uint16_t LimitCheckId) {
    return scanTable(CHECKENABLE_table, LimitCheckId, VL53L0X_STRING_UNKNOW_ERROR_CODE);
  }

  ////////////////legacy approach: force allocation of receiver and copy
  void get_device_error_string(DeviceError ErrorCode, char *pDeviceErrorString) {
    copyit(device_error_string(ErrorCode), pDeviceErrorString);
  }   // get_device_error_string



  void get_range_status_string(uint8_t RangeStatus, char *pRangeStatusString) {
    copyit(range_status_string(RangeStatus), pRangeStatusString);
  }   // get_range_status_string


  void get_pal_error_string(Error PalErrorCode, char *pPalErrorString) {
    copyit(pal_error_string(PalErrorCode), pPalErrorString);
  }   // get_pal_error_string

  void get_pal_state_string(State PalStateCode, char *pPalStateString) {
    copyit(pal_state_string(PalStateCode), pPalStateString);
  }   // get_pal_state_string


  Error get_sequence_steps_info(SequenceStepId SequenceStepId, char *pSequenceStepsString) {
    return copyit(sequence_steps_info(SequenceStepId), pSequenceStepsString);
  }   // get_sequence_steps_info


  void get_limit_check_info(uint16_t LimitCheckId, char *pLimitCheckString) {
    copyit(limit_check_info(LimitCheckId), pLimitCheckString);
  }   // get_limit_check_info

} //end namespace
