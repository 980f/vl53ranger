/*******************************************************************************
*   Copyright 2016, STMicroelectronics International N.V.
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
*      notice, this list of conditions and the following disclaimer in the
*      documentation and/or other materials provided with the distribution.
* Neither the name of STMicroelectronics nor the
*      names of its contributors may be used to endorse or promote products
*      derived from this software without specific prior written permission.
*
*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
*   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
*   WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
*   NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
*   IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
*   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
*   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
*   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#ifndef VL53L0X_API_STRINGS_H_
#define VL53L0X_API_STRINGS_H_

#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"

namespace VL53L0X {
/**
 new approach, point to it so that printf etc can pick up the data without intermediate buffer or copy.

 Each of the below returns nullptr on an invalid code unless someone decided to put in a specific string for such.
 */
  const char *device_error_string(DeviceError ErrorCode);
  const char *range_status_string(uint8_t RangeStatus);
  const char *pal_error_string(const Error &PalErrorCode);
  const char *pal_state_string(const State &PalStateCode);
  const char *sequence_steps_info(const SequenceStepId &SequenceStepId);
  const char *limit_check_info(uint16_t LimitCheckId);

  //was in wrong module, is a device function not a stringifying function:
//  Error get_device_info(VL53L0X_DEV  VL53L0X_DeviceInfo_t *pVL53L0X_DeviceInfo);

/** all but one of the get__string|info methods used to return ERROR_NONE, and stuff in an error message to the text.
 * changing those to return void allows us to remove gratuitous checks that were most likely not made.
 * */
  void get_device_error_string(DeviceError ErrorCode, char *pDeviceErrorString);

  void get_range_status_string(uint8_t RangeStatus, char *pRangeStatusString);

  void get_pal_error_string(Error PalErrorCode, char *pPalErrorString);

  void get_pal_state_string(State PalStateCode, char *pPalStateString);

  void get_limit_check_info(uint16_t LimitCheckId, char *pLimitCheckString);

  /** @returns ERROR_INVALID_PARAMS for unknown step id, else ERROR_NONE. */
  Error get_sequence_steps_info(SequenceStepId SequenceStepId, char *pSequenceStepsString);
}// end name space

//note: use links in build to bring in the language of interest.
#include "vl53l0x.text" //moved to cpp file to see who bypassed the interface

#endif // ifndef VL53L0X_API_STRINGS_H_
