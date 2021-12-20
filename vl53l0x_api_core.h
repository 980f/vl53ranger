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

#include "vl53l0x_platform.h" //for Dev_t
#include "vl53l0x_fixpoint.h"

namespace VL53L0X {
  //some device independent functions:
  void reverse_bytes(uint8_t *data, uint32_t size);
  uint8_t encode_vcsel_period(uint8_t vcsel_period_pclks);
  uint8_t decode_vcsel_period(uint8_t vcsel_period_reg);
  uint32_t isqrt(uint32_t num);
  uint32_t quadrature_sum(uint32_t a, uint32_t b);
  uint32_t decode_timeout(uint16_t encoded_timeout);
  uint16_t encode_timeout(uint32_t timeout_macro_clks);
  uint32_t calc_timeout_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

  template<unsigned whole, unsigned fract, typename RawType= uint32_t>
  static RawType quadrature_sum(FixPoint<whole, fract, RawType> a, FixPoint<whole, fract, RawType> b){
    //todo: debate whether the guard in the non-class quadrature_sum should apply
//      if (a > 65535 || b > 65535) {
//        return 65535;
//      }
    return isqrt(a.squared()+ b.squared());
  }


  class Core : public Dev_t {
  public:
    Core(Arg &&args) : Dev_t(args) {
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

    Erroneous<bool> GetSequenceStepEnable(SequenceStepId StepId); // GetSequenceStepEnable

    SemverLite GetProductRevision();

    /** VCSELPulsePeriodPCLK */
    Erroneous<uint8_t> get_vcsel_pulse_period(VcselPeriod VcselPeriodType);

    Error set_vcsel_pulse_period(VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriodPCLK);

    Error get_info_from_device(uint8_t option);

    Error get_sequence_step_timeout(SequenceStepId SequenceStepId, uint32_t *pTimeOutMicroSecs);

    Error set_sequence_step_timeout(SequenceStepId SequenceStepId, uint32_t TimeOutMicroSecs);

    Error get_measurement_timing_budget_micro_seconds(uint32_t *pMeasurementTimingBudgetMicroSeconds);

    Error set_measurement_timing_budget_micro_seconds(uint32_t MeasurementTimingBudgetMicroSeconds);

    Error load_tuning_settings(const uint8_t *pTuningSettingBuffer);

    /** ?secondary return via reference to dmm as most if it is computed as a side effect of primary return.
     * */
    Erroneous<FixPoint1616_t> calc_sigma_estimate(const RangingMeasurementData_t &pRangingMeasurementData,  uint32_t &dmm);

    FixPoint1616_t get_total_xtalk_rate(const RangingMeasurementData_t &pRangingMeasurementData);

    Erroneous<FixPoint1616_t> get_total_signal_rate(const RangingMeasurementData_t &pRangingMeasurementData);

    /** RMD not const due to setting of darkMM */
    Error get_pal_range_status(uint8_t DeviceRangeStatus, FixPoint1616_t SignalRate, uint16_t EffectiveSpadRtnCount,RangingMeasurementData_t &pRangingMeasurementData, uint8_t *pPalRangeStatus);

    /**
 * @brief  Get specific limit check enable state
 *
 * @par Function Description
 * This function get the enable state of a specific limit check.
 * The limit check is identified with the LimitCheckId.
 *
 * @note This function Access to the device
 *
 * @param   Dev                           Device Handle
 * @param   LimitCheckId                  Limit Check ID
 *  (0<= LimitCheckId < GetNumberOfLimitCheck() ).
 * @param   pLimitCheckEnable             Pointer to the check limit enable
 * value.
 *  if 1 the check limit
 *        corresponding to LimitCheckId is Enabled
 *  if 0 the check limit
 *        corresponding to LimitCheckId is disabled
 * @return  ERROR_NONE             Success
 * @return  ERROR_INVALID_PARAMS   This error is returned
 *  when LimitCheckId value is out of range.
 * @return  "Other error code"            See ::Error
 */
    Erroneous<bool> GetLimitCheckEnable(CheckEnable LimitCheckId);



    class MagicTrio {
      ErrorAccumulator Status;
      Physical &comm; //not a base as eventually we will pass in a reference to a baser class
    public:
      MagicTrio(Physical &comm):comm(comm),Status(ERROR_NONE){
        Status = comm.WrByte( 0x80, 0x01);
        Status = comm.WrByte(0xFF, 0x01);
        Status = comm.WrByte( 0x00, 0x00);
      }
      ~MagicTrio(){
        Status = comm.WrByte(0x00, 0x01);
        Status = comm.WrByte( 0xFF, 0x00);
        Status = comm.WrByte( 0x80, 0x00);
      }
    };
    /** RAII widget that surrounds some fetches. */
    MagicTrio magicWrapper(){
      return MagicTrio(comm);
    }





  protected:  //common code fragments or what were file static but didn't actually have the 'static' like they should have.
    Error setValidPhase(uint8_t high, uint8_t low);

    Error setPhasecalLimit(uint8_t value);

    Error FFwrap(RegSystem index,uint8_t value ){
      ErrorAccumulator Status = comm.WrByte( 0xFF, 0x01);
      Status |= comm.WrByte( index, value);
      Status |= comm.WrByte( 0xFF, 0x00);
      return Status;
    }

    template<typename Intish> Erroneous<Intish> FFread(RegSystem index){
      Erroneous<Intish> value;
      value.error = comm.WrByte( 0xFF, 0x01);
      value.error |= comm.ReadMulti(index, reinterpret_cast<uint8_t *>(&value.wrapped),sizeof(Intish) );
      value.error |= comm.WrByte( 0xFF, 0x00);
      return value;
    }

  protected: //some of this functionality was in api.cpp but used by core.cpp
    Error device_read_strobe();
    /** read up to 4 bytes from 0x90 after selecting which at 0x94
     * source for templates can be in the CPP if not used outside that module :) */
    template<typename Int> Erroneous<Int> packed90(uint8_t which);
    Erroneous<uint32_t> middleof64(unsigned int which);

/** gets value from device, @returns whether it worked OK.*/
    template<typename Scalar> bool fetch(Erroneous<Scalar> &item, RegSystem reg);
    Erroneous <SchedulerSequenceSteps_t> get_sequence_step_enables();
    uint32_t calc_dmax(FixPoint1616_t totalSignalRate_mcps, FixPoint1616_t totalCorrSignalRate_mcps, FixPoint1616_t pwMult, uint32_t sigmaEstimateP1, FixPoint1616_t sigmaEstimateP2, uint32_t peakVcselDuration_us);

    Error SetXTalkCompensationEnable(uint8_t XTalkCompensationEnable);

    Error set_ref_spad_map(SpadArray &refSpadArray);
    Error get_ref_spad_map(SpadArray &refSpadArray);
  };


}//end namespace
#endif /* _VL53L0X_API_CORE_H_ */
