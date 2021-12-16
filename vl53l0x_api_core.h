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

#ifndef _VL53L0X_API_CORE_H_
#define _VL53L0X_API_CORE_H_

//#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"

namespace VL53L0X {
  void reverse_bytes(uint8_t *data, uint32_t size);
  uint8_t encode_vcsel_period(uint8_t vcsel_period_pclks);
  uint8_t decode_vcsel_period(uint8_t vcsel_period_reg);
  uint32_t isqrt(uint32_t num);
  uint32_t quadrature_sum(uint32_t a, uint32_t b);
  uint32_t decode_timeout(uint16_t encoded_timeout);
  uint16_t encode_timeout(uint32_t timeout_macro_clks);

  class Core : public Dev_t {
  public:
    Core(TwoWire &i2c, uint8_t I2cDevAddr) : Dev_t(i2c, I2cDevAddr) {
      //do nothing here so that we can statically construct
    }

    /** values which have the same representation on the I2C message as in program storage can use this wrapper */
    template<typename Chunk> struct Parameter {
      Dev_t &parent;

      /** getter */
      operator Erroneous<Chunk>() {
      }

      /** setter */
      bool operator=(Chunk chunk) {
      }
    };

/** VCSELPulsePeriodPCLK */
    Erroneous<uint8_t> get_vcsel_pulse_period(VcselPeriod VcselPeriodType);

    Error set_vcsel_pulse_period(VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriodPCLK);

    Error get_info_from_device(uint8_t option);

    Error get_sequence_step_timeout(SequenceStepId SequenceStepId, uint32_t *pTimeOutMicroSecs);

    Error set_sequence_step_timeout(SequenceStepId SequenceStepId, uint32_t TimeOutMicroSecs);

    Error get_measurement_timing_budget_micro_seconds(uint32_t *pMeasurementTimingBudgetMicroSeconds);

    Error set_measurement_timing_budget_micro_seconds(uint32_t MeasurementTimingBudgetMicroSeconds);

    Error load_tuning_settings(const uint8_t *pTuningSettingBuffer);

    Error calc_sigma_estimate(RangingMeasurementData_t *pRangingMeasurementData, FixPoint1616_t *pSigmaEstimate, uint32_t *pDmax_mm);

    Error get_total_xtalk_rate(RangingMeasurementData_t *pRangingMeasurementData, FixPoint1616_t *ptotal_xtalk_rate_mcps);

    Error get_total_signal_rate(RangingMeasurementData_t *pRangingMeasurementData, FixPoint1616_t *ptotal_signal_rate_mcps);

    Error get_pal_range_status(uint8_t DeviceRangeStatus, FixPoint1616_t SignalRate, uint16_t EffectiveSpadRtnCount, RangingMeasurementData_t *pRangingMeasurementData, uint8_t *pPalRangeStatus);

    uint32_t calc_timeout_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
  };
}//end namespace
#endif /* _VL53L0X_API_CORE_H_ */
