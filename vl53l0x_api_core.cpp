/*******************************************************************************
Copyright 2021 Andy Heilveil, github/980F via mutation of source
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

#include <Arduino.h>
#include "vl53l0x_api_core.h"
#include "vl53l0x_api.h"
#include "vl53l0x_api_calibration.h"

#include "log_api.h"
#include "vl53l0x_fixpoint.h"
#include "bitmanipulators.h"

namespace VL53L0X {

  //the following us values were duplicated, and in one case with different values.
  const uint32_t StartOverheadMicroSeconds = 1320;
  const uint32_t StartOverheadMicroSecondsBudget = 1910;//BUG: there were two constants by the same name and apparently only one got updated for some change.

  const uint32_t EndOverheadMicroSeconds = 960;
  const uint32_t MsrcOverheadMicroSeconds = 660;
  const uint32_t TccOverheadMicroSeconds = 590;
  const uint32_t DssOverheadMicroSeconds = 690;
  const uint32_t PreRangeOverheadMicroSeconds = 660;
  const uint32_t FinalRangeOverheadMicroSeconds = 550;
  const uint32_t cMinTimingBudgetMicroSeconds = 20000;

  void reverse_bytes(uint8_t *data, uint32_t size) {
    uint8_t tempData;
    uint32_t mirrorIndex;
    uint32_t middle = size / 2;
    uint32_t index;

    for (index = 0; index < middle; index++) {
      mirrorIndex = size - index - 1;
      tempData = data[index];
      data[index] = data[mirrorIndex];
      data[mirrorIndex] = tempData;
    }
  } // VL53L0X_reverse_bytes


  uint8_t decode_vcsel_period(uint8_t vcsel_period_reg) {
    /*!
     * Converts the encoded VCSEL period register value into the real
     * period in PLL clocks
     */
    return (vcsel_period_reg + 1) << 1;
  }

  uint8_t encode_vcsel_period(uint8_t vcsel_period_pclks) {
    /*!
     * Converts the encoded VCSEL period register value into the real period
     * in PLL clocks
     */
    return (vcsel_period_pclks >> 1) - 1;
  }

  uint32_t isqrt(uint32_t num) {
    /*
     * Implements an integer square root
     *
     * From: http://en.wikipedia.org/wiki/Methods_of_computing_square_roots
     */

    uint32_t res = 0;
    uint32_t bit = 1 << 30;
    /* The second-to-top bit is set:
     *	1 << 14 for 16-bits, 1 << 30 for 32 bits */

    /* "bit" starts at the highest power of four <= the argument. */
    while (bit > num) {
      bit >>= 2;
    }

    while (bit != 0) {
      if (num >= res + bit) {
        num -= res + bit;
        res = (res >> 1) + bit;
      } else {
        res >>= 1;
      }
      bit >>= 2;
    }

    return res;
  } // VL53L0X_isqrt

  uint32_t quadrature_sum(uint32_t a, uint32_t b) {
    /*
     * Implements a quadrature sum
     *
     * rea = sqrt(a^2 + b^2)
     *
     * Trap overflow case max input value is 65535 (16-bit value)
     * as internal calc are 32-bit wide
     *
     * If overflow then set output to maximum
     */
    if (a > 65535 || b > 65535) {
      return 65535;
    }
    return isqrt(squared(a) + squared(b));
  } // VL53L0X_quadrature_sum

  bool Core::device_read_strobe() {
    LOG_FUNCTION_START;
    auto onexit = push(Private_Strober, 0x00, 0x01);
    /* polling with timeout to avoid deadlock*/
    for (unsigned LoopNb = VL53L0X_DEFAULT_MAX_LOOP; LoopNb-- > 0;) {
      uint8_t strobe;
      comm.Read(  Private_Strober, strobe);
      if (strobe != 0) {
        return true;
      }
    }
    return false;//timeout
  } // VL53L0X_device_read_strobe

  template<typename Int> Int Core::packed90(uint8_t which) {
    comm.WrByte(0x94, which);
    Int value;
    if (device_read_strobe()) {
      comm.Read( 0x90,value );
    }
    return value;
  }

  /** fetch 24 bits from one register and merge with 8 from the next.
   * former code didn't qualify data processing with success at getting raw data */
  uint32_t Core::middleof64(unsigned which) {
    auto fetched = packed90<uint32_t>(which);
    auto rest = packed90<uint32_t>(which + 1);
    return fetched << 8 | getByte<3>(rest);
  }

  bool Core::get_info_from_device(uint8_t infoGroups) {
    //read into temps so that we only apply if all are read successfully:
    //we could make a substruct in DeviceSpeciingParameters_t instead of redeclaring most of them here.
    //bit 1 group:
    uint8_t ModuleId;//BUG: was used when might not have been read, wrapped with status
    uint8_t Revision;//BUG: see ModuleId.
    decltype(DeviceSpecificParameters_t::ProductId) ProductId;/* buffer for Product Identifier String  */

    //bit 0 group:
    SpadCount ReferenceSpad;
    SpadArray NvmRefGoodSpadMap;


//bit 2 group
    const uint32_t DistMeasTgtFixed1104_mm(400 << 4);
    uint32_t DistMeasFixed1104_400_mm;
    uint32_t SignalRateMeasFixed1104_400_mm;
    FixPoint1616_t SignalRateMeasFixed400mmFix {0};

    DeviceSpecificParameters_t::PartUID_t PartUID;

    LOG_FUNCTION_START;
    uint8_t ReadDataFromDeviceDone = VL53L0X_GETDEVICESPECIFICPARAMETER(ReadDataFromDeviceDone);
    uint8_t needs = infoGroups & ~ReadDataFromDeviceDone;

    /* Each subgroup of this access is done only once after that a GetDeviceInfo or datainit is done */
    if (ReadDataFromDeviceDone != InfoGroup::ALL) { //if not all done
      {
        auto trio = magicWrapper();
        auto yappoper = YAPopper(comm);//sets some bit now, clears it later

        comm.WrByte(0xFF, 0x07);
        comm.WrByte(0x81, 0x01);
// dropped as inconvenient and timings are fixed and predictable. The caller can ask for one group at a time if asking for all of them takes too long.         Error |= PollingDelay();
        {
          auto popper = push(0x80, 0x01, 0x00);//NB: just outer item of magicWrapper

          if (getBit<0>(needs)) {
            auto packed = packed90<uint32_t>(0x6b);
              ReferenceSpad.quantity = getBits<14, 8>(packed.wrapped);
              ReferenceSpad.isAperture = getBit<15>(packed.wrapped);
            packed = packed90<uint32_t>(0x24);
              NvmRefGoodSpadMap[0] = getByte<3>(packed.wrapped);
              NvmRefGoodSpadMap[1] = getByte<2>(packed.wrapped);
              NvmRefGoodSpadMap[2] = getByte<1>(packed.wrapped);
              NvmRefGoodSpadMap[3] = getByte<0>(packed.wrapped);
            packed = packed90<uint32_t>(0x25);
              NvmRefGoodSpadMap[4] = getByte<3>(packed.wrapped);
              NvmRefGoodSpadMap[5] = getByte<2>(packed.wrapped);

          }

          if (getBit<1>(needs)) {
            ModuleId = packed90<uint8_t>(0x02);
            Revision = packed90<uint8_t>(0x7B);

            //now to unpack 7 bit fields from a series of 32 bit words:
            {//indent what might become a function someday
             uint32_t packed(0);//zero init here matters!
              const uint8_t mask7 = Mask<6, 0>::shifted;
              uint8_t pager = 0x77;//first page
              char *prodid = ProductId;//will increment as data is acquired sequentially
              int msb = 0;//misnamed, should be 'number of bits'
              char *const end = prodid + 19;//place where the null goes
              while (prodid < end) {
                if (msb >= 7) {//enough to pull an ascii char from the bitstream
                  msb -= 7;
                  *prodid++ |= mask7 & (packed >> msb);
                  *prodid = 0;//erases old content (garbage) as we go along, and makes sure string is terminated
                } else {
                  //partial character from lsbs of 32 bits into msbs of 7:
                  *prodid = mask7 & (packed << (7 - msb));//feed residual forward
                  packed = packed90<uint32_t>(pager++);
                  msb += 32;
                }
              }
            }
          }//end option 1

          if (getBit<2>(needs)) {
            PartUID.Upper = packed90<uint32_t>(0x7B);
            PartUID.Lower = packed90<uint32_t>(0x7C);
            //next items are 32 bits out of 64, kinda like a 24.8 out of a 32.32
            SignalRateMeasFixed1104_400_mm = middleof64(0x73);
            DistMeasFixed1104_400_mm = middleof64(0x75);
          }//end option 2

        }
      }

    }

    if (ReadDataFromDeviceDone != 7) {
      /* Assign to variable if status is ok */
      if (getBit<0>(needs)) {
        VL53L0X_SETDEVICESPECIFICPARAMETER(ReferenceSpad.quantity, ReferenceSpad.quantity);
        VL53L0X_SETDEVICESPECIFICPARAMETER(ReferenceSpad.isAperture, ReferenceSpad.isAperture);

        Data.SpadData.RefGoodSpadMap = NvmRefGoodSpadMap;
      }

      if (getBit<1>(needs)) {
        VL53L0X_SETDEVICESPECIFICPARAMETER(ModuleId, ModuleId);
        VL53L0X_SETDEVICESPECIFICPARAMETER(Revision, Revision);
        strcpy(VL53L0X_GETDEVICESPECIFICPARAMETER(ProductId), ProductId);
      }

      if (getBit<2>(needs)) {
        VL53L0X_SETDEVICESPECIFICPARAMETER(PartUID, PartUID);
//BUG: originally, types reversed!
//        SignalRateMeasFixed400mmFix = VL53L0X_FIXPOINT97TOFIXPOINT1616(SignalRateMeasFixed1104_400_mm);
        //ick: the ST code suggests that the 32 bits we extracted from the middle of 64 is really 16 bits in 9.7 format.
        SignalRateMeasFixed400mmFix = SignalRateMeasFixed1104_400_mm << 9;
        VL53L0X_SETDEVICESPECIFICPARAMETER(SignalRateMeasFixed400mm, SignalRateMeasFixed400mmFix);
        int32_t OffsetMicroMeters(0);//BUG: was int16, truncating too soon
        if (DistMeasFixed1104_400_mm != 0) {
          int32_t OffsetFixed1104_mm = DistMeasFixed1104_400_mm - DistMeasTgtFixed1104_mm;//was uint32_t despite being an intrinsically signed value
          OffsetMicroMeters = -((OffsetFixed1104_mm * 1000) >> 4);//ick: truncates
        }
        PALDevDataSet(Part2PartOffsetAdjustmentNVMMicroMeter, OffsetMicroMeters);
      }
      //BUG: we ignore errors in the fetch parts and by setting the flags here we will never recover: (todo: clear bits in infoGroups upon errors!)
      VL53L0X_SETDEVICESPECIFICPARAMETER(ReadDataFromDeviceDone, (ReadDataFromDeviceDone | infoGroups));
    }

    return true;
  } // VL53L0X_get_info_from_device

  uint32_t calc_macro_period_ps(uint8_t vcsel_period_pclks) {
    /* The above calculation will produce rounding errors,  therefore set fixed value //ICK: comment refers to non-existent code.
     */
    const unsigned PLL_period_ps = 1655;//ick: 64 bits for short constant was silly, compiler knows to extend as needed
    const unsigned macro_period_vclks = 2304;//ick: 32 bits for short constant was silly, compiler knows to extend as needed
    //casting the first variable is sufficient to cast the rest, extra parens added in case the compiler disagrees with me :)
    return (static_cast<uint32_t>(vcsel_period_pclks) * macro_period_vclks) * PLL_period_ps;
  } // VL53L0X_calc_macro_period_ps

  uint16_t encode_timeout(uint32_t timeout_macro_clks) {
    /*!
     * Encode timeout in macro periods in (LSByte * 2^MSByte) + 1 format
     */

    if (timeout_macro_clks > 0) {
      uint32_t ls_byte = timeout_macro_clks - 1;
      uint16_t ms_byte = 0;
      while (ls_byte >= 256) {//say what you mean. compiler will probably generate the optimized expression that formerly was here.
        ls_byte >>= 1;
        ++ms_byte;
      }
      return MAKEUINT16(ls_byte, ms_byte);//the while repeats until all the bits that the mask formerly here masked is zero, so no point in masking.
    } else {
      return 0;
    }
  } // VL53L0X_encode_timeout

  bool sequence_step_enabled(SequenceStepId stepId, uint8_t SequenceConfig) {
    return getBit(bitFor(stepId), SequenceConfig);
  } // sequence_step_enabled

  uint32_t decode_timeout(uint16_t encoded_timeout) {
    /*!
     * Decode 16-bit timeout register value - format (LSByte * 2^MSByte) + 1
     */
    return 1 + (uint32_t(encoded_timeout & 0x00FF) << (encoded_timeout >> 8));//ick: former cast of shrink to u32 was silly
  } // VL53L0X_decode_timeout

/* To convert ms into register value */
  uint32_t calc_timeout_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks) {
    const uint32_t macro_period_ns = roundedDivide(calc_macro_period_ps(vcsel_period_pclks), 1000);
    return roundedDivide(timeout_period_us * 1000, macro_period_ns);
  } // VL53L0X_calc_timeout_mclks

/* To convert register value into us */
  uint32_t calc_timeout_us(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks) {
    const uint32_t macro_period_ns = roundedDivide(calc_macro_period_ps(vcsel_period_pclks), 1000);
    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;//BUG: rogue tile? the ns/2 and 1000 do not make sense together here
  } // VL53L0X_calc_timeout_us


  void Core::SetXTalkCompensationEnable(bool XTalkCompensationEnable) {
    LOG_FUNCTION_START;
    uint16_t LinearityCorrectiveGain = PALDevDataGet(LinearityCorrectiveGain);
    FixPoint313_t saywhatyoumean(((XTalkCompensationEnable) && (LinearityCorrectiveGain == 1000)) ? VL53L0X_GETPARAMETERFIELD(XTalkCompensationRateMegaCps) : FixPoint1616_t(0));

    /* the following register has a format 3.13 */
    comm.WrWord(REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, saywhatyoumean);
    VL53L0X_SETPARAMETERFIELD(XTalkCompensationEnable, XTalkCompensationEnable);
  } // VL53L0X_SetXTalkCompensationEnable


  uint32_t Core::get_sequence_step_timeout(SequenceStepId stepId) {////BUG: uses many values even when they were not successfully fetched.
    switch (stepId) {
      case SEQUENCESTEP_TCC:
      case SEQUENCESTEP_DSS:
      case SEQUENCESTEP_MSRC: {
        uint8_t CurrentVCSELPulsePeriodPClk = get_vcsel_pulse_period(VCSEL_PERIOD_PRE_RANGE);
        uint8_t EncodedTimeOutByte;
        fetch(EncodedTimeOutByte, REG_MSRC_CONFIG_TIMEOUT_MACROP);
        uint16_t MsrcTimeOutMClks = decode_timeout(EncodedTimeOutByte);
        return calc_timeout_us(MsrcTimeOutMClks, CurrentVCSELPulsePeriodPClk);
      }
        break;
      case SEQUENCESTEP_PRE_RANGE: {
        /* Retrieve PRE-RANGE VCSEL Period */
        uint8_t CurrentVCSELPulsePeriodPClk = get_vcsel_pulse_period(VCSEL_PERIOD_PRE_RANGE);
        /* Retrieve PRE-RANGE Timeout in Macro periods (MCLKS) */
        //ick: formerly fetched it all over again
        uint16_t PreRangeEncodedTimeOut;
        fetch(PreRangeEncodedTimeOut, REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI);//there is a low that is ignored
        uint16_t PreRangeTimeOutMClks = decode_timeout(PreRangeEncodedTimeOut);
        return calc_timeout_us(PreRangeTimeOutMClks, CurrentVCSELPulsePeriodPClk);
      }
        break;
      case SEQUENCESTEP_FINAL_RANGE: {
        SchedulerSequenceSteps_t SchedulerSequenceSteps = get_sequence_step_enables();
        if (SchedulerSequenceSteps.PreRangeOn) {
//ick: useless code in main branch right about here.
          uint16_t PreRangeEncodedTimeOut;
          fetch(PreRangeEncodedTimeOut, REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI);
          uint16_t PreRangeTimeOutMClks = decode_timeout(PreRangeEncodedTimeOut);

          /* Retrieve FINAL-RANGE Timeout in Macro periods (MCLKS) */
          uint8_t FinalVCSELPulsePeriodPClk = get_vcsel_pulse_period(VCSEL_PERIOD_FINAL_RANGE);
          uint16_t FinalRangeEncodedTimeOut;
          fetch(FinalRangeEncodedTimeOut, REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI);
          uint16_t FinalRangeTimeOutMClks = decode_timeout(FinalRangeEncodedTimeOut);

          FinalRangeTimeOutMClks -= PreRangeTimeOutMClks;//BUG: uses value even if fetch fails
          return calc_timeout_us(FinalRangeTimeOutMClks, FinalVCSELPulsePeriodPClk);
        }
        //980f: review original code
      }
        break;
      default:
        THROW(ERROR_INVALID_PARAMS);
    }//end switch

    return 0;//this seems like residue of some error above.
  } // get_sequence_step_timeout

  bool Core::set_sequence_step_timeout(const SequenceStepId StepId, const uint32_t TimeOutMicroSecs) {
    switch (StepId) {
      case SEQUENCESTEP_TCC:
      case SEQUENCESTEP_DSS:
      case SEQUENCESTEP_MSRC: {
        uint8_t CurrentVCSELPulsePeriodPClk = get_vcsel_pulse_period(VCSEL_PERIOD_PRE_RANGE);//ick:formerly called up to api laye
        uint16_t MsrcRangeTimeOutMClks = calc_timeout_mclks(TimeOutMicroSecs, CurrentVCSELPulsePeriodPClk);
        auto MsrcEncodedTimeOut = saturated<uint8_t>(MsrcRangeTimeOutMClks - 1);
        VL53L0X_SETDEVICESPECIFICPARAMETER(LastEncodedTimeout, MsrcEncodedTimeOut);
        comm.WrByte(REG_MSRC_CONFIG_TIMEOUT_MACROP, MsrcEncodedTimeOut);
        VL53L0X_SETDEVICESPECIFICPARAMETER(LastEncodedTimeout, MsrcEncodedTimeOut);
        //todo: check original code, IDE scrambled this a bit.
      }
        break;
      case SEQUENCESTEP_PRE_RANGE: {
        auto CurrentVCSELPulsePeriodPClk = get_vcsel_pulse_period(VCSEL_PERIOD_PRE_RANGE);//ick:formerly called up to api laye
        auto PreRangeTimeOutMClks = calc_timeout_mclks(TimeOutMicroSecs, CurrentVCSELPulsePeriodPClk);
        auto PreRangeEncodedTimeOut = encode_timeout(PreRangeTimeOutMClks);

        VL53L0X_SETDEVICESPECIFICPARAMETER(LastEncodedTimeout, PreRangeEncodedTimeOut);

        comm.WrWord(REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, PreRangeEncodedTimeOut);
        VL53L0X_SETDEVICESPECIFICPARAMETER(PreRange.TimeoutMicroSecs, TimeOutMicroSecs);
        return true;
      }
        break;
      case SEQUENCESTEP_FINAL_RANGE: {
        /* For the final range timeout, the pre-range timeout
         * must be added. To do this both final and pre-range
         * timeouts must be expressed in macro periods MClks
         * because they have different vcsel periods.
         */

        auto SchedulerSequenceSteps = get_sequence_step_enables();
        uint16_t PreRangeTimeOutMClks = 0;
        if (SchedulerSequenceSteps.PreRangeOn) {
          auto CurrentVCSELPulsePeriodPClk = get_vcsel_pulse_period(VCSEL_PERIOD_PRE_RANGE);//ick:formerly called up to api laye
          uint16_t PreRangeEncodedTimeOut;
          fetch(PreRangeEncodedTimeOut, REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI);
          PreRangeTimeOutMClks = decode_timeout(PreRangeEncodedTimeOut);
          /* Calculate FINAL RANGE Timeout in Macro Periods (MCLKS) and add PRE-RANGE value */
          auto FinalVCSELPulsePeriodPClk = get_vcsel_pulse_period(VCSEL_PERIOD_FINAL_RANGE);
          auto FinalRangeTimeOutMClks = calc_timeout_mclks(TimeOutMicroSecs, (uint8_t) CurrentVCSELPulsePeriodPClk);
          FinalRangeTimeOutMClks += PreRangeTimeOutMClks;
          auto FinalRangeEncodedTimeOut = encode_timeout(FinalRangeTimeOutMClks);

          comm.WrWord(0x71, FinalRangeEncodedTimeOut);
          VL53L0X_SETDEVICESPECIFICPARAMETER(FinalRange.TimeoutMicroSecs, TimeOutMicroSecs);

          return true;
        }
      }
        break;
      default:
        THROW(ERROR_INVALID_PARAMS);
    }
//we only get here on failure to fetch a value.
    return true;
  } // set_sequence_step_timeout

  void Core::setValidPhase(uint8_t high, uint8_t low) {
    comm.WrByte(REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, high);//NB: mimicking old bug of ignoring status of first write
    comm.WrByte(REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, low);
  }

  void Core::setPhasecalLimit(uint8_t value) {
    FFwrap(REG_ALGO_PHASECAL_LIM, value);
  }


////////////////////////////////////////

  uint8_t Core::get_vcsel_pulse_period(VcselPeriod VcselPeriodType) {
    uint8_t vcsel_period_reg;

    switch (VcselPeriodType) {
      case VCSEL_PERIOD_PRE_RANGE:
        fetch(vcsel_period_reg, REG_PRE_RANGE_CONFIG_VCSEL_PERIOD);
        break;
      case VCSEL_PERIOD_FINAL_RANGE:
        fetch(vcsel_period_reg, REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD);
        break;
      default:
        THROW(ERROR_INVALID_PARAMS);
    } // switch

    return decode_vcsel_period(vcsel_period_reg);
  } // VL53L0X_get_vcsel_pulse_period

  uint8_t Core::get_SequenceConfig() {
    uint8_t SequenceConfig;
    fetch(SequenceConfig, REG_SYSTEM_SEQUENCE_CONFIG);
    return SequenceConfig;
  }

  SchedulerSequenceSteps_t Core::get_sequence_step_enables() {
    LOG_FUNCTION_START;
    return get_SequenceConfig();
  }

  bool Core::set_measurement_timing_budget_micro_seconds(uint32_t MeasurementTimingBudgetMicroSeconds) {
    LOG_FUNCTION_START;
    if (MeasurementTimingBudgetMicroSeconds < cMinTimingBudgetMicroSeconds) {
      return LOG_ERROR(ERROR_INVALID_PARAMS);
    }
    uint32_t FinalRangeTimingBudgetMicroSeconds = MeasurementTimingBudgetMicroSeconds - (StartOverheadMicroSeconds + EndOverheadMicroSeconds);

    auto SchedulerSequenceSteps = get_sequence_step_enables();

    if ((SchedulerSequenceSteps.anyOfMsrcDccTcc())) {

      /* TCC, MSRC and DSS all share the same timeout */
      auto MsrcDccTccTimeoutMicroSeconds = get_sequence_step_timeout(SEQUENCESTEP_MSRC);

      /* Subtract the TCC, MSRC and DSS timeouts if they are enabled. */

      /* TCC */
      if (SchedulerSequenceSteps.TccOn) {
        uint32_t SubTimeout = MsrcDccTccTimeoutMicroSeconds + TccOverheadMicroSeconds;

        if (SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
          FinalRangeTimingBudgetMicroSeconds -= SubTimeout;
        } else {
          /* Requested timeout too big. */
          return LOG_ERROR(ERROR_INVALID_PARAMS);
        }
      }


      /* DSS */
      if (SchedulerSequenceSteps.DssOn) {
        uint32_t SubTimeout = 2 * (MsrcDccTccTimeoutMicroSeconds + DssOverheadMicroSeconds);

        if (SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
          FinalRangeTimingBudgetMicroSeconds -= SubTimeout;
        } else {
          /* Requested timeout too big. */
          return LOG_ERROR(ERROR_INVALID_PARAMS);
        }
      } else if (SchedulerSequenceSteps.MsrcOn) {
        /* MSRC */
        uint32_t SubTimeout = MsrcDccTccTimeoutMicroSeconds + MsrcOverheadMicroSeconds;

        if (SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
          FinalRangeTimingBudgetMicroSeconds -= SubTimeout;
        } else {
          /* Requested timeout too big. */
          return LOG_ERROR(ERROR_INVALID_PARAMS);
        }
      }
    }

    if (SchedulerSequenceSteps.PreRangeOn) {
      /* Subtract the Pre-range timeout if enabled. */
      auto PreRangeTimeoutMicroSeconds = get_sequence_step_timeout(SEQUENCESTEP_PRE_RANGE);
      uint32_t SubTimeout = PreRangeTimeoutMicroSeconds + PreRangeOverheadMicroSeconds;

      if (SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
        FinalRangeTimingBudgetMicroSeconds -= SubTimeout;
      } else {
        /* Requested timeout too big. */
        return LOG_ERROR(ERROR_INVALID_PARAMS);
      }
    }

    if (SchedulerSequenceSteps.FinalRangeOn) {
      FinalRangeTimingBudgetMicroSeconds -= FinalRangeOverheadMicroSeconds;

      /* Final Range Timeout
       * Note that the final range timeout is determined by the timing
       * budget and the sum of all other timeouts within the sequence.
       * If there is no room for the final range timeout, then an error
       * will be set. Otherwise the remaining time will be applied to
       * the final range.
       */
      if (set_sequence_step_timeout(SEQUENCESTEP_FINAL_RANGE, FinalRangeTimingBudgetMicroSeconds)) {
        VL53L0X_SETPARAMETERFIELD(MeasurementTimingBudgetMicroSeconds, MeasurementTimingBudgetMicroSeconds);
      }
    }
    return true;
  } // VL53L0X_set_measurement_timing_budget_micro_seconds

  uint32_t Core::get_measurement_timing_budget_micro_seconds() {
    LOG_FUNCTION_START;
    SchedulerSequenceSteps_t SchedulerSequenceSteps = get_sequence_step_enables();

    /* Start and end overhead times always present */
    uint32_t measurementTimingBudgetMicroSeconds = StartOverheadMicroSecondsBudget + EndOverheadMicroSeconds;

    if (SchedulerSequenceSteps.anyOfMsrcDccTcc()) {
      auto MsrcDccTccTimeoutMicroSeconds = get_sequence_step_timeout(SEQUENCESTEP_MSRC);
      if (SchedulerSequenceSteps.TccOn) {
        measurementTimingBudgetMicroSeconds += MsrcDccTccTimeoutMicroSeconds + TccOverheadMicroSeconds;
      }
      if (SchedulerSequenceSteps.DssOn) {
        measurementTimingBudgetMicroSeconds += 2 * (MsrcDccTccTimeoutMicroSeconds + DssOverheadMicroSeconds);
      } else if (SchedulerSequenceSteps.MsrcOn) {
        measurementTimingBudgetMicroSeconds += MsrcDccTccTimeoutMicroSeconds + MsrcOverheadMicroSeconds;
      }
    }
    if (SchedulerSequenceSteps.PreRangeOn) {
      auto PreRangeTimeoutMicroSeconds = get_sequence_step_timeout(SEQUENCESTEP_PRE_RANGE);
      measurementTimingBudgetMicroSeconds += PreRangeTimeoutMicroSeconds + PreRangeOverheadMicroSeconds;
    }

    if (SchedulerSequenceSteps.FinalRangeOn) {
      auto FinalRangeTimeoutMicroSeconds = get_sequence_step_timeout(SEQUENCESTEP_FINAL_RANGE);
      measurementTimingBudgetMicroSeconds += (FinalRangeTimeoutMicroSeconds + FinalRangeOverheadMicroSeconds);
    }
    VL53L0X_SETPARAMETERFIELD(MeasurementTimingBudgetMicroSeconds, *measurementTimingBudgetMicroSeconds);
    return measurementTimingBudgetMicroSeconds;
  } // VL53L0X_get_measurement_timing_budget_micro_seconds


  static uint16_t unpack(const uint8_t *pTuningSettingBuffer, int &Index) {
    uint8_t msb = pTuningSettingBuffer[Index++];
    uint8_t lsb = pTuningSettingBuffer[Index++];
    //the above may NOT be inlined as that would get into 'undefined behavior' territory re 'sequence points'.
    return MAKEUINT16(lsb, msb);
  }

  /**
   * record formats:
   * 0xFF  0..3 msbof16 lsbof16  loads one of 4 fields of PALDevData
   * 0..3 index  number of bytes indicated in first byte  I2C multi byte write.
   * */
  unsigned Core::load_tuning_settings(const uint8_t *pTuningSettingBuffer) {
    LOG_FUNCTION_START;
    int Index = 0;//ick: only value over using the pTuning..Buffer as a pointer might be for debug display.

    while (pTuningSettingBuffer[Index]) {
      uint8_t NumberOfWrites = pTuningSettingBuffer[Index++];
      if (NumberOfWrites == 0xFF) {
        /* internal parameters */
        switch (pTuningSettingBuffer[Index++]) {
          case 0: /* uint16_t SigmaEstRefArray -> 2 bytes */
            PALDevDataSet(SigmaEst.RefArray, unpack(pTuningSettingBuffer, Index));
            break;
          case 1: /* uint16_t SigmaEstEffPulseWidth -> 2 bytes */
            PALDevDataSet(SigmaEst.EffPulseWidth, unpack(pTuningSettingBuffer, Index));
            break;
          case 2: /* uint16_t SigmaEstEffAmbWidth -> 2 bytes */
            PALDevDataSet(SigmaEst.EffAmbWidth, unpack(pTuningSettingBuffer, Index));
            break;
          case 3: /* uint16_t targetRefRate -> 2 bytes */
            PALDevDataSet(targetRefRate, unpack(pTuningSettingBuffer, Index));
            break;
          default: /* invalid parameter */
            return ~Index;
        } // switch
      } else if (NumberOfWrites <= 4) { //copy bytes that must be in device endian order.
        uint8_t Address = pTuningSettingBuffer[Index++];
        comm.WriteMulti(Address, &pTuningSettingBuffer[Index], NumberOfWrites);
        Index += NumberOfWrites;
      } else {
        return ~Index;
      }
    }
    return 0;
  } // VL53L0X_load_tuning_settings

  FixPoint1616_t Core::get_total_xtalk_rate(const RangingMeasurementData_t &pRangingMeasurementData) {

    bool xtalkCompEnable = VL53L0X_GETPARAMETERFIELD(XTalkCompensationEnable);
//todo: lost test value of xtalkCompeEnable

    FixPoint1616_t xtalkPerSpadMegaCps = VL53L0X_GETPARAMETERFIELD(XTalkCompensationRateMegaCps);

    /* FixPoint1616 * FixPoint 8:8 = FixPoint0824 */
    FixPoint1616_t totalXtalkMegaCps = pRangingMeasurementData.EffectiveSpadRtnCount.raw * xtalkPerSpadMegaCps.raw;

    /* FixPoint0824 >> 8 = FixPoint1616 */
    return totalXtalkMegaCps.shrink(8);
  } // VL53L0X_get_total_xtalk_rate

  FixPoint1616_t Core::get_total_signal_rate(const RangingMeasurementData_t &pRangingMeasurementData) {
    LOG_FUNCTION_START;
    //todo: restore from original code
//    FixPoint1616_t totalXtalkMegaCps {pRangingMeasurementData.SignalRateRtnMegaCps};//default in case returned error is ignored
    return get_total_xtalk_rate(pRangingMeasurementData);
  } // VL53L0X_get_total_signal_rate

  uint32_t Core::calc_dmax(const FixPoint1616_t totalSignalRate_mcps, const FixPoint1616_t totalCorrSignalRate_mcps, const FixPoint1616_t pwMult, const uint32_t sigmaEstimateP1, FixPoint1616_t sigmaEstimateP2, const uint32_t peakVcselDuration_us) {
    const uint32_t cSigmaLimit = 18;
    const FixPoint1616_t cSignalLimit {0.25};// = 0x4000;     /* 0.25 */
    const FixPoint1616_t cSigmaEstRef {0.0001};// = 0x00000042; /* 0.001 */
    const uint32_t cAmbEffWidthSigmaEst_ns = 6;
    const uint32_t cAmbEffWidthDMax_ns = 7;

    LOG_FUNCTION_START;

    uint32_t dmaxCalRange_mm = PALDevDataGet(dmaxCal.RangeMilliMeter);//goes from 16 to 32 bits.
    FixPoint1616_t dmaxCalSignalRateRtn_mcps = PALDevDataGet(dmaxCal.SignalRateRtnMegaCps);

    /* uint32 * FixPoint1616 = FixPoint1616 */

    FixPoint1616_t SignalAt0mm = dmaxCalRange_mm * dmaxCalSignalRateRtn_mcps.raw;

    /* FixPoint1616 >> 8 = FixPoint2408 */
    SignalAt0mm.shrink(8);
    SignalAt0mm.raw *= dmaxCalRange_mm;
    FixPoint1616_t minSignalNeeded_p1 {0};
    if (totalCorrSignalRate_mcps.raw > 0) {

      /* Shift by 10 bits to increase resolution prior to the
       * division */
      uint32_t signalRateTemp_mcps = totalSignalRate_mcps.boosted(10);

      /*  FixPoint0626/FixPoint1616 = FixPoint2210 */
      minSignalNeeded_p1 = roundedDivide(signalRateTemp_mcps, totalCorrSignalRate_mcps.raw);


      /* Apply a factored version of the speed of light.
       *  Correction to be applied at the end */
      minSignalNeeded_p1.raw *= 3;

      /* FixPoint2210 * FixPoint2210 = FixPoint1220 */
      minSignalNeeded_p1 *= minSignalNeeded_p1;

      /* FixPoint1220 >> 16 = FixPoint2804 */
      minSignalNeeded_p1 = minSignalNeeded_p1.rounded();
    }

    FixPoint1616_t minSignalNeeded_p2 = pwMult.raw * sigmaEstimateP1;

    /* FixPoint1616 >> 16 =	 uint32
     * then  uint32 * uint32	=  uint32 */
    minSignalNeeded_p2 = squared(minSignalNeeded_p2.rounded());

    /* Check sigmaEstimateP2
     * If this value is too high there is not enough signal rate
     * to calculate dmax value so set a suitable value to ensure
     * a very small dmax.
     */

    FixPoint1616_t sigmaEstP2Tmp = sigmaEstimateP2.rounded();
    sigmaEstP2Tmp = roundedDivide(sigmaEstP2Tmp, cAmbEffWidthSigmaEst_ns);
    sigmaEstP2Tmp.raw *= cAmbEffWidthDMax_ns;//todo: multiplyByRatio function. Can be given greater range than is easy to achieve inline.

    FixPoint1616_t minSignalNeeded_p3;
    if (sigmaEstP2Tmp >= Unity) {// >=1.0
      minSignalNeeded_p3 = uint32_t(0xfff00000);
    } else {
      /* DMAX uses a different ambient width from sigma, so apply correction.
       * Perform division before multiplication to prevent overflow.
       */
      sigmaEstimateP2 = roundedDivide(sigmaEstimateP2, cAmbEffWidthSigmaEst_ns);
      sigmaEstimateP2.raw *= cAmbEffWidthDMax_ns; //ick: isn't this already computed in sigmaEstP2Tmp?

      /* FixPoint1616 >> 16 = uint32 */
      minSignalNeeded_p3 = squared(sigmaEstimateP2.scaled(16));
    }

    /* FixPoint1814 / uint32 = FixPoint1814
     * then FixPoint1814 squared = FixPoint3628 := FixPoint0428 */
    FixPoint1616_t sigmaLimitTmp = squared(roundedDivide(cSigmaLimit << 14, 1000));

    /* FixPoint1616 * FixPoint1616 = FixPoint3232 */
    FixPoint1616_t sigmaEstSqTmp = squared(cSigmaEstRef);

    /* FixPoint3232 >> 4 = FixPoint0428 */
    sigmaEstSqTmp = sigmaEstSqTmp.scaled(4);

    /* FixPoint0428 - FixPoint0428	= FixPoint0428 */
    sigmaLimitTmp.raw -= sigmaEstSqTmp.raw;

    /* uint32_t * FixPoint0428 = FixPoint0428 */
    FixPoint1616_t minSignalNeeded_p4 = 4 * 12 * sigmaLimitTmp.raw;

    /* FixPoint0428 >> 14 = FixPoint1814 */
    minSignalNeeded_p4.shrink(14);

    /* uint32 + uint32 = uint32 */
    FixPoint1616_t minSignalNeeded = minSignalNeeded_p2 + minSignalNeeded_p3;

    /* uint32 / uint32 = uint32 */
    minSignalNeeded = roundedDivide(minSignalNeeded, peakVcselDuration_us);

    /* uint32 << 14 = FixPoint1814 */
    minSignalNeeded.boost(14);

    /* FixPoint1814 / FixPoint1814 = uint32
     * then FixPoint3200 * FixPoint2804 := FixPoint2804*/
    minSignalNeeded = minSignalNeeded_p1.raw * roundedDivide(minSignalNeeded, minSignalNeeded_p4.raw);

    /* Apply correction by dividing by 1000000.
     * This assumes 10E16 on the numerator of the equation
     * and 10E-22 on the denominator.
     * We do this because 32bit fix point calculation can't
     * handle the larger and smaller elements of this equation,
     * i.e. speed of light and pulse widths.
     */
    minSignalNeeded = roundedDivide(minSignalNeeded, 1000U);
    minSignalNeeded.boost(4);
    minSignalNeeded = roundedDivide(minSignalNeeded, 1000);//BUG: perhaps 980F screwed this up? Rounding each division by 1000 to get to /1000,000 is wrong if rounded twice.

    /* FixPoint2408/FixPoint2408 = uint32 */
    FixPoint1616_t dmaxAmbient = isqrt(roundedDivide(SignalAt0mm.raw, minSignalNeeded.raw));

    /* FixPoint1616 >> 8 = FixPoint2408 */
    FixPoint1616_t signalLimitTmp = cSignalLimit.scaled(8);

    /* FixPoint2408/FixPoint2408 = uint32 */
    FixPoint1616_t dmaxDark = isqrt(roundedDivide(SignalAt0mm.raw, signalLimitTmp.raw));//former check for zero moved into roundedDivide

    dmaxDark.lessen(dmaxAmbient);
    return dmaxDark.raw;
  } // calc_dmax

  FixPoint1616_t Core::calc_sigma_estimate(const RangingMeasurementData_t &pRangingMeasurementData, uint32_t &pDmm) {
    /* Expressed in 100ths of a ns, i.e. centi-ns */
    const uint32_t cPulseEffectiveWidth_centi_ns {800};
    /* Expressed in 100ths of a ns, i.e. centi-ns */
    const uint32_t cAmbientEffectiveWidth_centi_ns {600};
    const FixPoint1616_t cSigmaEstRef {0.001}; /* 0.001 */
    const uint32_t cVcselPulseWidth_ps = 4700;      /* pico secs */
    //655.5299 looks suspiciciously close to 655.36 and typos:
    const FixPoint1616_t cSigmaEstMax = 0x028F87AE;//todo: convert items like this to floating point as the conversion to integer will be done at compile time
    const FixPoint1616_t cSigmaEstRtnMax = 0xF000;// 15/16 ths?
    const FixPoint1616_t cAmbToSignalRatioMax = 0xF0000000 / cAmbientEffectiveWidth_centi_ns;//ick: may be off by 64k, need some explanation of how you can divide the u32's and get the correct 16.16
    /* Time Of Flight per mm (6.6 pico secs) */
    const FixPoint1616_t cTOF_per_mm_ps {6.6};//=0x0006999A; //ick: value doesn't seem to match reality, should be closer to 7.0
    const FixPoint1616_t cMaxXTalk_kcps {50.0};//= 0x00320000;
    const uint32_t cPllPeriod_ps {1655};

//    uint32_t vcselWidth;
//    uint32_t finalRangeMacroPCLKS;
//    uint32_t preRangeMacroPCLKS;
//    uint32_t peakVcselDuration_us;
//
    /*! \addtogroup calc_sigma_estimate
     * @{
     *
     * Estimates the range sigma based on the
     *
     *	- vcsel_rate_kcps
     *	- ambient_rate_kcps
     *	- signal_total_events
     *	- xtalk_rate
     *
     * and the following parameters
     *
     *	- SigmaEstRefArray
     *	- SigmaEstEffPulseWidth
     *	- SigmaEstEffAmbWidth
     */

    LOG_FUNCTION_START;


    /*
     * We work in kcps rather than mcps as this helps keep within the
     * confines of the 32 Fix1616 type.
     */

    FixPoint1616_t ambientRate_kcps = pRangingMeasurementData.AmbientRateRtnMegaCps.millis();//980f: now rounded instead of truncated

    FixPoint1616_t correctedSignalRate_mcps = pRangingMeasurementData.SignalRateRtnMegaCps;

    FixPoint1616_t totalSignalRate_mcps = get_total_signal_rate(pRangingMeasurementData);

    FixPoint1616_t xTalkCompRate_mcps = //VL53L0X_GETPARAMETERFIELD(XTalkCompensationRateMegaCps);
      get_total_xtalk_rate(pRangingMeasurementData);

    /* Signal rate measurement provided by device is the
     * peak signal rate, not average.
     */
    FixPoint1616_t peakSignalRate_kcps = totalSignalRate_mcps.millis();
    uint32_t xTalkCompRate_kcps = min(xTalkCompRate_mcps.raw * 1000, cMaxXTalk_kcps.raw);


    /* Calculate final range macro periods */

    auto finalRangeTimeoutMicroSecs = VL53L0X_GETDEVICESPECIFICPARAMETER(FinalRange.TimeoutMicroSecs);

    auto finalRangeVcselPCLKS = VL53L0X_GETDEVICESPECIFICPARAMETER(FinalRange.VcselPulsePeriod);

    auto finalRangeMacroPCLKS = calc_timeout_mclks(finalRangeTimeoutMicroSecs, finalRangeVcselPCLKS);

    /* Calculate pre-range macro periods */
    uint32_t preRangeTimeoutMicroSecs = VL53L0X_GETDEVICESPECIFICPARAMETER(PreRange.TimeoutMicroSecs);

    auto preRangeVcselPCLKS = VL53L0X_GETDEVICESPECIFICPARAMETER(PreRange.VcselPulsePeriod);

    auto preRangeMacroPCLKS = calc_timeout_mclks(preRangeTimeoutMicroSecs, preRangeVcselPCLKS);

    unsigned vcselWidth = (finalRangeVcselPCLKS == 8) ? 2 : 3;

    auto peakVcselDuration_us = kilo((vcselWidth << 11) * (preRangeMacroPCLKS + finalRangeMacroPCLKS));
    peakVcselDuration_us *= cPllPeriod_ps;
    peakVcselDuration_us = kilo(peakVcselDuration_us);

    /* Fix1616 >> 8 = Fix2408 */
    totalSignalRate_mcps = totalSignalRate_mcps.shrink(8);

    /* Fix2408 * uint32 = Fix2408
     * then Fix2408 >> 8 = uint32 */

    uint32_t vcselTotalEventsRtn = roundedScale(totalSignalRate_mcps.raw * peakVcselDuration_us, 8);

    /* Fix2408 << 8 = Fix1616 = */
    totalSignalRate_mcps.boost(8);

    if (peakSignalRate_kcps.raw == 0) {

      PALDevDataSet(SigmaEstimate, cSigmaEstMax);
      return {cSigmaEstMax};
    } else {
      if (vcselTotalEventsRtn < 1) {
        vcselTotalEventsRtn = 1;
      }

      /*
       * Calculate individual components of the main equation -
       * replicating the equation implemented in the script
       * OpenAll_Ewok_ranging_data.jsl.
       *
       * sigmaEstimateP1 represents the effective pulse width, which
       * is a tuning parameter, rather than a real value.
       *
       * sigmaEstimateP2 represents the ambient/signal rate ratio
       * expressed as a multiple of the effective ambient width
       * (tuning parameter).
       *
       * sigmaEstimateP3 provides the signal event component, with the
       * knowledge that
       *	- Noise of a square pulse is 1/sqrt(12) of the pulse
       *	 width.
       *	- at 0Lux, sigma is proportional to
       *	  effectiveVcselPulseWidth/sqrt(12 * signalTotalEvents)
       *
       * deltaT_ps represents the time of flight in pico secs for the
       * current range measurement, using the "TOF per mm" constant
       * (in ps).
       */

      FixPoint1616_t sigmaEstimateP1 = cPulseEffectiveWidth_centi_ns;

      /* ((FixPoint1616 << 16)* uint32)/uint32 = FixPoint1616 */
      FixPoint1616_t sigmaEstimateP2(ambientRate_kcps.boost(16), peakSignalRate_kcps.raw);

      /* Clip to prevent overflow. Will ensure safe max result. */
      sigmaEstimateP2.lessen(cAmbToSignalRatioMax);
      sigmaEstimateP2 *= cAmbientEffectiveWidth_centi_ns;
      FixPoint1616_t sigmaEstimateP3(isqrt(vcselTotalEventsRtn * 12) * 2);

      /* uint32 * FixPoint1616 = FixPoint1616 */
      FixPoint1616_t deltaT_ps(pRangingMeasurementData.RangeMilliMeter * cTOF_per_mm_ps.raw);

      /*
       * vcselRate - xtalkCompRate
       * (uint32 << 16) - FixPoint1616 = FixPoint1616.
       * Divide result by 1000 to convert to mcps.
       */
      FixPoint1616_t diff1_mcps(peakSignalRate_kcps.boosted(16) - xTalkCompRate_kcps, 1000, 0);

      /* vcselRate + xtalkCompRate */
      FixPoint1616_t diff2_mcps((peakSignalRate_kcps.boosted(16) + xTalkCompRate_kcps), 1000, 0);

      /* Shift by 8 bits to increase resolution prior to the division */
      diff1_mcps.boost(8);

      /* FixPoint0824/FixPoint1616 = FixPoint2408 */
      FixPoint1616_t xTalkCorrection = abs(((long long) diff1_mcps.raw / diff2_mcps.raw));

      /* FixPoint2408 << 8 = FixPoint1616 */
      xTalkCorrection.boost(8);

      /* FixPoint1616/uint32 = FixPoint1616 */
      FixPoint1616_t pwMult(deltaT_ps.raw, cVcselPulseWidth_ps, 0); /* smaller than 1.0f */ //ick: 980f added rounding

      /*
       * FixPoint1616 * FixPoint1616 = FixPoint3232, however both
       * values are small enough such that32 bits will not be
       * exceeded.
       */
      pwMult.raw *= (Unity.raw - xTalkCorrection.raw);

      /* (FixPoint3232 >> 16) = FixPoint1616 */
      pwMult.shrink(16);//980f: rounded

      /* FixPoint1616 + FixPoint1616 = FixPoint1616 */
      pwMult.raw += Unity.raw;

      /*
       * At this point the value will be 1.xx, therefore if we square
       * the value this will exceed 32 bits. To address this perform
       * a single shrink to the right before the multiplication.
       */
      pwMult.shrink(1);//980f: rounded
      /* FixPoint1715 * FixPoint1715 = FixPoint3430 */
      pwMult.square();

      /* (FixPoint3430 >> 14) = Fix1616 */
      pwMult.shrink(14);

      /* FixPoint1616 * uint32 = FixPoint1616 */
      FixPoint1616_t sqr1(pwMult.raw * sigmaEstimateP1.raw);
      /* (FixPoint1616 >> 16) = FixPoint3200 */
      sqr1.shrink(16);

      FixPoint1616_t sqr2 = sigmaEstimateP2;
      /* (FixPoint1616 >> 16) = FixPoint3200 */
      sqr2.shrink(16);

      /* SQRT(FixPoin6400) = FixPoint3200 */
      FixPoint1616_t sqrtResult_centi_ns(quadrature_sum(sqr1, sqr2));

      /* (FixPoint3200 << 16) = FixPoint1616 */
      sqrtResult_centi_ns.boost(16);

      /*
       * Note that the Speed Of Light is expressed in um per 1E-10
       * seconds (2997) Therefore to get mm/ns we have to divide by
       * 10000
       */
      FixPoint1616_t sigmaEstRtn((roundedDivide(sqrtResult_centi_ns, 100) / sigmaEstimateP3.raw));//ick: formerly not rounded
      sigmaEstRtn.raw *= SPEED_OF_LIGHT_IN_AIR;
      sigmaEstRtn.divideby(10000);
      /* Clip to prevent overflow. Will ensure safe max result. */
      sigmaEstRtn.lessen(cSigmaEstRtnMax);

      /* sqrt(FixPoint3232) = FixPoint1616 */
      FixPoint1616_t sqrtResult(quadrature_sum(sigmaEstRtn, cSigmaEstRef));
      /*
       * Note that the Shift by 4 bits increases resolution prior to
       * the sqrt, therefore the result must be shifted by 2 bits to
       * the right to revert back to the FixPoint1616 format.
       */

      FixPoint1616_t sigmaEstimate(1000 * sqrtResult);

      if ((peakSignalRate_kcps.raw < 1) || (vcselTotalEventsRtn < 1) || (sigmaEstimate.raw > cSigmaEstMax.raw)) {
        sigmaEstimate = cSigmaEstMax;
      }

      pDmm = calc_dmax(totalSignalRate_mcps, correctedSignalRate_mcps, pwMult, sigmaEstimateP1, sigmaEstimateP2, peakVcselDuration_us);
      PALDevDataSet(SigmaEstimate, sigmaEstimate);
      return sigmaEstimate;
    }
  }

  bool Core::GetSequenceStepEnable(SequenceStepId StepId) {
    LOG_FUNCTION_START;
    auto seqbit = bitFor(StepId);
    if(seqbit==~0){
      THROW(ERROR_INVALID_PARAMS);
      return false;
    }
    uint8_t SequenceConfig = get_SequenceConfig();
    return getBit(seqbit, SequenceConfig);
  }

  bool Core::GetLimitCheckEnable(CheckEnable LimitCheckId) {
    if (LimitCheckId >= CHECKENABLE_NUMBER_OF_CHECKS) {
      THROW (ERROR_INVALID_PARAMS);
    }
    return Data.CurrentParameters.LimitChecksEnable[LimitCheckId];
  } // GetLimitCheckEnable

  FixPoint<9, 7> Core::GetLimitCheckValue(CheckEnable LimitCheckId) {
    LOG_FUNCTION_START;
    bool EnableZeroValue = false;

    FixPoint<9, 7> limitChecksValue;
    switch (LimitCheckId) {
      case CHECKENABLE_SIGMA_FINAL_RANGE:
      case CHECKENABLE_SIGNAL_REF_CLIP:
      case CHECKENABLE_RANGE_IGNORE_THRESHOLD:
        /* internal computation: */
        limitChecksValue = VL53L0X_GETARRAYPARAMETERFIELD(LimitChecksValue, LimitCheckId);
        EnableZeroValue = false;
        break;

      case CHECKENABLE_SIGNAL_RATE_MSRC:
      case CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
        fetch(limitChecksValue.raw, REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT);
        EnableZeroValue = false;
        break;

      case CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
        fetch(limitChecksValue.raw, REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT);
        EnableZeroValue = true;
        break;

      default:
        THROW (ERROR_INVALID_PARAMS);
    } // switch

    //980f: this seems awful convoluted since only one of the enums sets this true. review original code!
    if (EnableZeroValue) {//ick: did 908f refactoring invert this decision?
      if (limitChecksValue.raw == 0) {
        /* disabled: return value from memory */
        limitChecksValue = VL53L0X_GETARRAYPARAMETERFIELD(LimitChecksValue, LimitCheckId);

        VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksEnable, LimitCheckId, false);
      } else {
        VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksValue, LimitCheckId, limitChecksValue);
        VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksEnable, LimitCheckId, true);
      }
    }
    return limitChecksValue;
  } // GetLimitCheckValue


  RangeStatus Core::get_pal_range_status(uint8_t DeviceRangeStatus, FixPoint1616_t SignalRate, uint16_t EffectiveSpadRtnCount, RangingMeasurementData_t &pRangingMeasurementData) {
    LOG_FUNCTION_START;

    /*
     * VL53L0X has a good ranging when the value of the DeviceRangeStatus = 11.
     * This function will replace the value 0 with the value 11 in the DeviceRangeStatus.
     * In addition, the SigmaEstimator is not included in the VL53L0X DeviceRangeStatus, this will be added in the PalRangeStatus.
     */
    uint8_t DeviceRangeStatusInternal = getBits<6, 3>(DeviceRangeStatus);
    bool NoneFlag = (DeviceRangeStatusInternal == 0 || DeviceRangeStatusInternal == 5 || DeviceRangeStatusInternal == 7 || DeviceRangeStatusInternal == 12 || DeviceRangeStatusInternal == 13 || DeviceRangeStatusInternal == 14 || DeviceRangeStatusInternal == 15);

    FixPoint<9, 7> tmpWord;
    /* LastSignalRefMcps */
    {
      auto pager = push(Private_Pager, 0x01, 0x00);
      fetch(tmpWord.raw, REG_RESULT_PEAK_SIGNAL_RATE_REF);//todo: fix template version of fetch(FixPoint)
    }

    FixPoint<16, 16> LastSignalRefMcps = tmpWord;
    PALDevDataSet(LastSignalRefMcps, LastSignalRefMcps);

    /*
     * Check if Sigma limit is enabled, if yes then do comparison with limit value and put the result back into pPalRangeStatus.
     */
    auto SigmaLimitCheckEnable = GetLimitCheckEnable(CHECKENABLE_SIGMA_FINAL_RANGE);

    bool SigmaLimitflag = false;
    if (SigmaLimitCheckEnable) {
      /*
       * compute the Sigma and check with limit
       */
      uint32_t Dmax_mm = 0;
      FixPoint1616_t SigmaEstimate = calc_sigma_estimate(pRangingMeasurementData, Dmax_mm);
        pRangingMeasurementData.RangeDMaxMilliMeter = Dmax_mm;
        auto SigmaLimitValue = GetLimitCheckValue(CHECKENABLE_SIGMA_FINAL_RANGE);
        if ((SigmaLimitValue.raw > 0) && (SigmaEstimate > SigmaLimitValue)) {//todo: check for factor of 2 error due to refactoring
          /* Limit Fail */
          SigmaLimitflag = true;
        }
    }

    /*
     * Check if Signal ref clip limit is enabled, if yes then do comparison
     * with limit value and put the result back into pPalRangeStatus.
     */
    bool SignalRefClipLimitCheckEnable = GetLimitCheckEnable(CHECKENABLE_SIGNAL_REF_CLIP);

    bool SignalRefClipflag = false;
    if (SignalRefClipLimitCheckEnable) {
      auto SignalRefClipValue = GetLimitCheckValue(CHECKENABLE_SIGNAL_REF_CLIP);
      if ((SignalRefClipValue.raw > 0) && (LastSignalRefMcps > SignalRefClipValue)) {//todo: check for factor of 2 error due to refactoring
        /* Limit Fail */
        SignalRefClipflag = true;
      }
    }

    /*
     * Check if Signal ref clip limit is enabled, if yes then do comparison
     * with limit value and put the result back into pPalRangeStatus.
     * EffectiveSpadRtnCount has a format 8.8
     * If (Return signal rate < (1.5 x Xtalk x number of Spads)) : FAIL
     */
    auto RangeIgnoreThresholdLimitCheckEnable = GetLimitCheckEnable(CHECKENABLE_RANGE_IGNORE_THRESHOLD);

    bool RangeIgnoreThresholdflag = false;

    if (RangeIgnoreThresholdLimitCheckEnable) {
      /* Compute the signal rate per spad */
      //floating point is laboriously averted even though that might retain a literal bit more precision.
      FixPoint1616_t SignalRatePerSpad = (EffectiveSpadRtnCount != 0) ? SignalRate.boost(8).divideby(EffectiveSpadRtnCount) : FixPoint1616_t(0);//had to cast the 0 to ensure floating point would not be used temporarily even though correct answer would have been achieved.

      auto RangeIgnoreThresholdValue = GetLimitCheckValue(CHECKENABLE_RANGE_IGNORE_THRESHOLD);

      if ((RangeIgnoreThresholdValue.raw > 0) && (SignalRatePerSpad < RangeIgnoreThresholdValue)) {//todo: check for factor of 2 error due to refactoring
        /* Limit Fail add 2^6 to range status */
        RangeIgnoreThresholdflag = true;
      }
    }

    RangeStatus rangeStatus;

    if (NoneFlag) {
      rangeStatus = Not_Set; /* NONE */
    } else if (DeviceRangeStatusInternal == 1 || DeviceRangeStatusInternal == 2 || DeviceRangeStatusInternal == 3) {
      rangeStatus = HW_fail;
    } else if (DeviceRangeStatusInternal == 6 || DeviceRangeStatusInternal == 9) {
      rangeStatus = Phase_fail;
    } else if (DeviceRangeStatusInternal == 8 || DeviceRangeStatusInternal == 10 || SignalRefClipflag) {
      rangeStatus = Min_range;
    } else if (DeviceRangeStatusInternal == 4 || RangeIgnoreThresholdflag) {
      rangeStatus = Signal_Fail;
    } else if (SigmaLimitflag) {
      rangeStatus = Sigma_Fail;
    } else {
      rangeStatus = Range_Valid;
    }


    /* DMAX only relevant during range error */
    if (rangeStatus == Range_Valid) {
      pRangingMeasurementData.RangeDMaxMilliMeter = 0;
    }

    /* fill the Limit Check Error */
    bool SignalRateFinalRangeLimitCheckEnable = GetLimitCheckEnable(CHECKENABLE_SIGNAL_RATE_FINAL_RANGE);
//below:  true if not looked at or if looked and was true
    VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksStatus, CHECKENABLE_SIGMA_FINAL_RANGE, !SigmaLimitCheckEnable || SigmaLimitflag);
    VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksStatus, CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, !SignalRateFinalRangeLimitCheckEnable || (DeviceRangeStatusInternal == 4));
    VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksStatus, CHECKENABLE_SIGNAL_REF_CLIP, !SignalRefClipLimitCheckEnable || SignalRefClipflag);
    VL53L0X_SETARRAYPARAMETERFIELD(LimitChecksStatus, CHECKENABLE_RANGE_IGNORE_THRESHOLD, !RangeIgnoreThresholdLimitCheckEnable || RangeIgnoreThresholdflag);

    return rangeStatus;
  } // VL53L0X_get_pal_range_status


  void Core::set_ref_spad_map(SpadArray &refSpadArray) {
    comm.WriteMulti(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_BASE, reinterpret_cast<uint8_t *>(&refSpadArray), SpadArray::NumberOfBytes);
  }

  void Core::get_ref_spad_map(SpadArray &refSpadArray) {
    comm.ReadMulti(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_BASE, reinterpret_cast<uint8_t *>(&refSpadArray), SpadArray::NumberOfBytes);
  }

  struct VcselRangeChecker {
    uint8_t Min;
    uint8_t Max;

    bool inRange(uint8_t value) const {
      return value >= Min && value <= Max;
    }
  };

  const VcselRangeChecker PreVcselCheck = {12, 18};
  const VcselRangeChecker FinalVcselCheck = {8, 14};

  bool Core::set_vcsel_pulse_period(VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriodPCLK) {
    if (unsigned(VcselPeriodType) > 1) {    //ick: invalid VcselPeriodType was formerly ignored
      return LOG_ERROR(ERROR_INVALID_PARAMS);
    }
    if (getBit<0>(VCSELPulsePeriodPCLK)) {/* Value must be an even number */
      return LOG_ERROR(ERROR_INVALID_PARAMS);
    }
    const bool isFinal = VcselPeriodType;
    /* Check if valid clock period requested */
    if (!((isFinal ? FinalVcselCheck : PreVcselCheck).inRange(VCSELPulsePeriodPCLK))) {
      return LOG_ERROR(ERROR_INVALID_PARAMS);
    }

    /* Apply specific settings for the requested clock period */
    if (isFinal) {
      switch (VCSELPulsePeriodPCLK) {
        case 8:
          setValidPhase(0x10);
          comm.WrByte(REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
          comm.WrByte(REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
          setPhasecalLimit(0x30);
          break;
        case 10:
          setValidPhase(0x28);
          comm.WrByte(REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
          comm.WrByte(REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
          setPhasecalLimit(0x20);
          break;
        case 12:
          setValidPhase(0x38);
          comm.WrByte(REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
          comm.WrByte(REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
          setPhasecalLimit(0x20);
          break;
        case 14:
          setValidPhase(0x048);
          comm.WrByte(REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
          comm.WrByte(REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
          setPhasecalLimit(0x20);
          break;
          break;
        default:
          break;
      }
    } else {
      /* Set phase check limits */
      switch (VCSELPulsePeriodPCLK) {
        case 12:
          setValidPhase(0x18);//24
          break;
        case 14:
          setValidPhase(0x30);//48
          break;
        case 16:
          setValidPhase(0x40);//64
          break;
        case 18:
          setValidPhase(0x50);//80
          break;
        default:
          break;
      }
    }
    /* Re-calculate and apply timeouts, in macro periods */

    uint8_t vcsel_period_reg = encode_vcsel_period(VCSELPulsePeriodPCLK);

    /* When the VCSEL period for the pre or final range is changed,
     * the corresponding timeout must be read from the device using
     * the current VCSEL period, then the new VCSEL period can be
     * applied. The timeout then must be written back to the device
     * using the new VCSEL period.
     *
     * For the MSRC timeout, the same applies - this timeout being
     * dependant on the pre-range vcsel period.
     */
    if (isFinal) {
      auto FinalRangeTimeoutMicroSeconds = get_sequence_step_timeout(SEQUENCESTEP_FINAL_RANGE);
      comm.WrByte(REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
      set_sequence_step_timeout(SEQUENCESTEP_FINAL_RANGE, FinalRangeTimeoutMicroSeconds);
      VL53L0X_SETDEVICESPECIFICPARAMETER(FinalRange.VcselPulsePeriod, VCSELPulsePeriodPCLK);
    } else {
      auto PreRangeTimeoutMicroSeconds = get_sequence_step_timeout(SEQUENCESTEP_PRE_RANGE);

      auto MsrcTimeoutMicroSeconds = get_sequence_step_timeout(SEQUENCESTEP_MSRC);

      comm.WrByte(REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
      set_sequence_step_timeout(SEQUENCESTEP_PRE_RANGE, PreRangeTimeoutMicroSeconds);
      set_sequence_step_timeout(SEQUENCESTEP_MSRC, MsrcTimeoutMicroSeconds);

      VL53L0X_SETDEVICESPECIFICPARAMETER(PreRange.VcselPulsePeriod, VCSELPulsePeriodPCLK);
    }
    /* Finally, the timing budget must be re-applied */
    return set_measurement_timing_budget_micro_seconds(VL53L0X_GETPARAMETERFIELD(MeasurementTimingBudgetMicroSeconds));
  } // VL53L0X_set_vcsel_pulse_period

  void Core::set_SequenceConfig(uint8_t packed, bool andCache) {
    comm.WrByte(REG_SYSTEM_SEQUENCE_CONFIG, packed);
    if (andCache) {
      PALDevDataSet(SequenceConfig, packed);
    }
  }
} //end namespace
