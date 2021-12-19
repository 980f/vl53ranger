/*******************************************************************************
Copyright 2016, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.
        * Neither the name of STMicroelectronics nor the
          names of its contributors may be used to endorse or promote products
          derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#ifndef _VL53L0X_API_CALIBRATION_H_
#define _VL53L0X_API_CALIBRATION_H_

#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"

//#include "vl53l0x_api_core.h"
namespace VL53L0X {

  class Core;

  class Calibrator {
    Core &core;

    Calibrator(Core &core) : core(core) {
    }

    Error perform_xtalk_calibration(FixPoint1616_t XTalkCalDistance, FixPoint1616_t *pXTalkCompensationRateMegaCps);

    Error perform_offset_calibration(FixPoint1616_t CalDistanceMilliMeter, int32_t *pOffsetMicroMeter);

    Error set_offset_calibration_data_micro_meter(int32_t OffsetCalibrationDataMicroMeter);

    Error get_offset_calibration_data_micro_meter(int32_t *pOffsetCalibrationDataMicroMeter);

    Error apply_offset_adjustment();
    Error perform_ref_spad_management(uint32_t *refSpadCount, uint8_t *isApertureSpads);

    Error set_reference_spads(uint32_t count, uint8_t isApertureSpads);

    Error get_reference_spads(uint32_t *pSpadCount, uint8_t *pIsApertureSpads);

    Error perform_phase_calibration(uint8_t *pPhaseCal, const uint8_t get_data_enable, const uint8_t restore_config);

    Error perform_ref_calibration(uint8_t *pVhvSettings, uint8_t *pPhaseCal, uint8_t get_data_enable);

    Error set_ref_calibration(uint8_t VhvSettings, uint8_t PhaseCal);

    Error get_ref_calibration(uint8_t *pVhvSettings, uint8_t *pPhaseCal);
  };
}
#endif /* _VL53L0X_API_CALIBRATION_H_ */
