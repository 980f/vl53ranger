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

  Error Core::device_read_strobe() {
    LOG_FUNCTION_START;
    auto onexit = autoCloser(Private_Strober, 0x00, 0x01);
    if (onexit) {
      /* polling with timeout to avoid deadlock*/
      for (unsigned LoopNb = VL53L0X_DEFAULT_MAX_LOOP; LoopNb-- > 0;) {
        Erroneous<uint8_t> strobe {0};
        Fetch(strobe, Private_Strober);

        if (~strobe || strobe != 0) {
          return strobe.error;
        }
      }
      return ERROR_TIME_OUT;
    } else {
      return onexit;
    }
  } // VL53L0X_device_read_strobe

  template<typename Int> Erroneous<Int> Core::packed90(uint8_t which) {
    Erroneous<Int> value = comm.WrByte(0x94, which);
    value |= device_read_strobe();
    Fetch(value, 0x90);
    return value;
  }

  /** fetch 24 bits from one register and merge with 8 from the next.
   * former code didn't qualify data processing with success at getting raw data */
  Erroneous<uint32_t> Core::middleof64(unsigned which) {
    auto fetched = packed90<uint32_t>(which);
    if (fetched.isOk()) {
      auto rest = packed90<uint32_t>(which + 1);
      if (rest.isOk()) {
        rest.wrapped >>= (32 - 8);
        rest.wrapped |= fetched << 8;
      }
      return rest;
    }
    return fetched;
  }

  Error Core::get_info_from_device(uint8_t option) {
    //read into temps so that we only apply if all are read successfully:
    //we could make a substruct in DeviceSpeciingParameters_t instead of redeclaring most of othem here.
    Erroneous<uint8_t> ModuleId;//BUG: was used when might not have been read, wrapped with status
    Erroneous<uint8_t> Revision;//BUG: see ModuleId.
    uint8_t ReferenceSpadCount(0);
    uint8_t ReferenceSpadType(0);
    Erroneous<uint32_t> PartUIDUpper;
    Erroneous<uint32_t> PartUIDLower;

    uint32_t OffsetFixed1104_mm(0);
    int16_t OffsetMicroMeters(0);
    const uint32_t DistMeasTgtFixed1104_mm(400 << 4);
    Erroneous<uint32_t> DistMeasFixed1104_400_mm;
    Erroneous<uint32_t> SignalRateMeasFixed1104_400_mm;
    FixPoint1616_t SignalRateMeasFixed400mmFix {0};
    decltype(DeviceSpecificParameters_t::ProductId) ProductId;/* buffer for Product Identifier String  */

    SpadArray NvmRefGoodSpadMap;

    LOG_FUNCTION_START;
    uint8_t ReadDataFromDeviceDone = VL53L0X_GETDEVICESPECIFICPARAMETER( ReadDataFromDeviceDone);
    uint8_t needs = option & ~ReadDataFromDeviceDone;

    /* Each subgroup of this access is done only once after that a GetDeviceInfo or datainit is done */
    if (ReadDataFromDeviceDone != 7) { //if not all done

      Error |= comm.WrByte(0x80, 0x01);
      Error |= comm.WrByte(0xFF, 0x01);
      Error |= comm.WrByte(0x00, 0x00);
      Error |= comm.WrByte(0xFF, 0x06);

      Error |= comm.UpdateBit(0x83, 2, true);

      Error |= comm.WrByte(0xFF, 0x07);
      Error |= comm.WrByte(0x81, 0x01);

      Error |= PollingDelay();

      Error |= comm.WrByte(0x80, 0x01);

      if (getBit<0>(needs)) {
        auto packed = packed90<uint32_t>(0x6b);
        if(packed.isOk()) {//ick: some error checking has been added by 980f, but it is still not sane.
          ReferenceSpadCount = getBits<14, 8>(packed.wrapped);
          ReferenceSpadType = getBit<15>(packed.wrapped);
        }
        packed = packed90<uint32_t>(0x24);
        if(packed.isOk()) {
          NvmRefGoodSpadMap[0] = getByte<3>(packed.wrapped);
          NvmRefGoodSpadMap[1] = getByte<2>(packed.wrapped);
          NvmRefGoodSpadMap[2] = getByte<1>(packed.wrapped);
          NvmRefGoodSpadMap[3] = getByte<0>(packed.wrapped);
        }
        packed = packed90<uint32_t>(0x25);
        if(packed.isOk()) {
          NvmRefGoodSpadMap[4] = getByte<3>(packed.wrapped);
          NvmRefGoodSpadMap[5] = getByte<2>(packed.wrapped);
        }
      }

      if (getBit<1>(needs)) {
        ModuleId = packed90<uint8_t>(0x02);
        Revision = packed90<uint8_t>(0x7B);

        //now to unpack 7 bit fields from a series of 32 bit words:
        {//indent what might become a function someday
          Erroneous<uint32_t> packed {0};//zero init here matters!
          uint8_t pager = 0x77;//first page
          char *prodid = ProductId;//will increment as data is acquired sequentially
          int msb = 0;//misnamed, should be 'number of bits'
          char *const end = prodid + 19;//place where the null goes
          while (prodid < end) {
            if (msb >= 7) {//enough to pull an ascii char from the bitstream
              msb -= 7;
              *prodid++ |= 0x7F & (packed >> msb);
              *prodid = 0;//erases old content (garbage) as we go along, and makes sure string is terminated
            } else {
              //partial character from lsbs of 32 bits into msbs of 7:
              *prodid = 0x7F & (packed << (7 - msb));//feed residual forward
              packed = packed90<uint32_t>(pager++);
              if (~packed) {
                break;
              }
              msb += 32;
            }
          }
        }
      }//end option 1

      if (getBit<2>(needs)) {
        PartUIDUpper = packed90<uint32_t>(0x7B);
        PartUIDLower = packed90<uint32_t>(0x7C);
        //next items are 32 bits out of 64, kinda like a 24.8 out of a 32.32
        SignalRateMeasFixed1104_400_mm = middleof64(0x73);
        DistMeasFixed1104_400_mm = middleof64(0x75);
      }//end option 2

      Error |= comm.WrByte(0x81, 0x00);
      Error |= comm.WrByte(0xFF, 0x06);

      Error |= comm.UpdateBit(0x83, 2, false);

      Error |= comm.WrByte(0xFF, 0x01);
      Error |= comm.WrByte(0x00, 0x01);

      Error |= comm.WrByte(0xFF, 0x00);
      Error |= comm.WrByte(0x80, 0x00);

      ERROR_OUT;
    }

    if (ReadDataFromDeviceDone != 7) {
      /* Assign to variable if status is ok */
      if (getBit<0>(needs)) {
        VL53L0X_SETDEVICESPECIFICPARAMETER( ReferenceSpadCount, ReferenceSpadCount);
        VL53L0X_SETDEVICESPECIFICPARAMETER( ReferenceSpadType, ReferenceSpadType);

        Data.SpadData.RefGoodSpadMap = NvmRefGoodSpadMap;
      }

      if (getBit<1>(needs)) {
        VL53L0X_SETDEVICESPECIFICPARAMETER( ModuleId, ModuleId);
        VL53L0X_SETDEVICESPECIFICPARAMETER( Revision, Revision);
        strcpy(VL53L0X_GETDEVICESPECIFICPARAMETER( ProductId), ProductId);
      }

      if (getBit<2>(needs)) {
        VL53L0X_SETDEVICESPECIFICPARAMETER( PartUIDUpper, PartUIDUpper);
        VL53L0X_SETDEVICESPECIFICPARAMETER( PartUIDLower, PartUIDLower);
//BUG: originally, types reversed!
//        SignalRateMeasFixed400mmFix = VL53L0X_FIXPOINT97TOFIXPOINT1616(SignalRateMeasFixed1104_400_mm);
        //ick: the ST code suggests that the 32 bits we extracted from the middle of 64 is really 16 bits in 9.7 format.
        SignalRateMeasFixed400mmFix = SignalRateMeasFixed1104_400_mm << 9;//macro didn't play well with an Erroneous<u32>
        VL53L0X_SETDEVICESPECIFICPARAMETER( SignalRateMeasFixed400mm, SignalRateMeasFixed400mmFix);

        OffsetMicroMeters = 0;
        if (DistMeasFixed1104_400_mm != 0) {
          OffsetFixed1104_mm = DistMeasFixed1104_400_mm - DistMeasTgtFixed1104_mm;
          OffsetMicroMeters = -((OffsetFixed1104_mm * 1000) >> 4);//ick: truncates
        }
        PALDevDataSet( Part2PartOffsetAdjustmentNVMMicroMeter, OffsetMicroMeters);
      }
      VL53L0X_SETDEVICESPECIFICPARAMETER( ReadDataFromDeviceDone, (ReadDataFromDeviceDone | option));
    }

    return ERROR_NONE;
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
      return MAKEUINT16( ls_byte,ms_byte);//the while repeats until all the bits that the mask formerly here masked is zero, so no point in masking.
    } else {
      return 0;
    }
  } // VL53L0X_encode_timeout

  bool sequence_step_enabled(SequenceStepId stepId, uint8_t SequenceConfig) {
    LOG_FUNCTION_START;
    unsigned bitnumber = bitFor(stepId);
    if (bitnumber > 7) {
      return false;//
    }
    return (SequenceConfig >> bitnumber) & 1;
  } // sequence_step_enabled

  uint32_t decode_timeout(uint16_t encoded_timeout) {
    /*!
     * Decode 16-bit timeout register value - format (LSByte * 2^MSByte) + 1
     */
    return 1 + (uint32_t(encoded_timeout & 0x00FF) << (encoded_timeout >> 8));//ick: former cast of shrink to  u32 was silly
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

  template<typename Scalar> bool Core::fetch(Erroneous<Scalar> &item, RegSystem reg) {
    item.error = comm.Read<Scalar>(reg, item);
    return item.isOk();
  }

  static Error calcandReturn(uint32_t *pTimeOutMicroSecs, uint16_t MClks, uint8_t PeriodClock) {
    auto TimeoutMicroSeconds = calc_timeout_us(MClks, PeriodClock);
    *pTimeOutMicroSecs = TimeoutMicroSeconds;
    return ERROR_NONE;
  }

  Error Core::SetXTalkCompensationEnable(uint8_t XTalkCompensationEnable) {
    LOG_FUNCTION_START;

    uint16_t LinearityCorrectiveGain = PALDevDataGet( LinearityCorrectiveGain);

    FixPoint1616_t TempFix1616;
    if ((XTalkCompensationEnable == 0) || (LinearityCorrectiveGain != 1000)) {
      TempFix1616.raw = 0;
    } else {
      VL53L0X_GETPARAMETERFIELD( XTalkCompensationRateMegaCps.raw, TempFix1616.raw);
    }

    /* the following register has a format 3.13 */
    Error |= comm.WrWord(REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, VL53L0X_FIXPOINT1616TOFIXPOINT313(TempFix1616.raw));

    if (~Error) {
      SETPARAMETERFIELD( XTalkCompensationEnable, bool(XTalkCompensationEnable));
    }
    return Error;
  } // VL53L0X_SetXTalkCompensationEnable


  Error Core::get_sequence_step_timeout(SequenceStepId stepId, uint32_t *pTimeOutMicroSecs) {////BUG: uses many values even when they were not successfully fetched.
    uint32_t TimeoutMicroSeconds = 0;
    switch (stepId) {
      case SEQUENCESTEP_TCC:
      case SEQUENCESTEP_DSS:
      case SEQUENCESTEP_MSRC: {
        Erroneous<uint8_t> CurrentVCSELPulsePeriodPClk = get_vcsel_pulse_period(VCSEL_PERIOD_PRE_RANGE);//ick:formerly called up to api layer
        if (CurrentVCSELPulsePeriodPClk.isOk()) {
          Erroneous<uint8_t> EncodedTimeOutByte;
          if (fetch(EncodedTimeOutByte, REG_MSRC_CONFIG_TIMEOUT_MACROP)) {
            uint16_t MsrcTimeOutMClks = decode_timeout(EncodedTimeOutByte);    //BUG: formerly used EncodedTimeOutByte even when not fetched.
            return calcandReturn(pTimeOutMicroSecs, MsrcTimeOutMClks, CurrentVCSELPulsePeriodPClk);//BUG: formerly used CurrentVCSELPulsePeriodPClk even when not fetched.
          }
        }
      }
        break;
      case SEQUENCESTEP_PRE_RANGE: {
        /* Retrieve PRE-RANGE VCSEL Period */
        Erroneous<uint8_t> CurrentVCSELPulsePeriodPClk = get_vcsel_pulse_period(VCSEL_PERIOD_PRE_RANGE);//ick:formerly called up to api layer
        /* Retrieve PRE-RANGE Timeout in Macro periods (MCLKS) */
        if (CurrentVCSELPulsePeriodPClk.isOk()) {
          //ick: formerly fetched it all over again
          Erroneous<uint16_t> PreRangeEncodedTimeOut;
          if (fetch(PreRangeEncodedTimeOut, REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI)) {//there is a low that is ignored
            uint16_t PreRangeTimeOutMClks = decode_timeout(PreRangeEncodedTimeOut);
            return calcandReturn(pTimeOutMicroSecs, PreRangeTimeOutMClks, CurrentVCSELPulsePeriodPClk);
          }
        }
      }
        break;
      case SEQUENCESTEP_FINAL_RANGE: {
        Erroneous<SchedulerSequenceSteps_t> SchedulerSequenceSteps = get_sequence_step_enables();
        uint16_t PreRangeTimeOutMClks = 0;
        if (SchedulerSequenceSteps.wrapped.PreRangeOn) {
          /* Retrieve PRE-RANGE VCSEL Period */
          Erroneous<uint8_t> CurrentVCSELPulsePeriodPClk = get_vcsel_pulse_period(VCSEL_PERIOD_PRE_RANGE);//ick:formerly called up to api layer
          if (CurrentVCSELPulsePeriodPClk.isOk()) {
            Erroneous<uint16_t> PreRangeEncodedTimeOut;
            if (fetch(PreRangeEncodedTimeOut, REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI)) {
              PreRangeTimeOutMClks = decode_timeout(PreRangeEncodedTimeOut);//BUG: ignores error in fetch
            }
          }
          /* Retrieve FINAL-RANGE Timeout in Macro periods (MCLKS) */
          Erroneous<uint8_t> FinalVCSELPulsePeriodPClk = get_vcsel_pulse_period(VCSEL_PERIOD_FINAL_RANGE);
          if (FinalVCSELPulsePeriodPClk.isOk()) {
            Erroneous<uint16_t> FinalRangeEncodedTimeOut;
            if (fetch(FinalRangeEncodedTimeOut, REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI)) {
              uint16_t FinalRangeTimeOutMClks = decode_timeout(FinalRangeEncodedTimeOut);

              FinalRangeTimeOutMClks -= PreRangeTimeOutMClks;//BUG: uses value even if fetch fails
              return calcandReturn(pTimeOutMicroSecs, FinalRangeTimeOutMClks, FinalVCSELPulsePeriodPClk);
            }
          }
        }
      }
        break;
      default:
        return ERROR_INVALID_PARAMS;
    }//end switch

    return ERROR_CONTROL_INTERFACE;//the only error the various fetchings can return.
  } // get_sequence_step_timeout

  Error Core::set_sequence_step_timeout(const SequenceStepId StepId, const uint32_t TimeOutMicroSecs) {
    Error Error = ERROR_NONE;
    Erroneous<uint16_t> PreRangeEncodedTimeOut;
    uint16_t PreRangeTimeOutMClks;
    uint16_t FinalRangeTimeOutMClks;
    uint16_t FinalRangeEncodedTimeOut;

    switch (StepId) {
      case SEQUENCESTEP_TCC:
      case SEQUENCESTEP_DSS:
      case SEQUENCESTEP_MSRC: {
        Erroneous<uint8_t> CurrentVCSELPulsePeriodPClk = get_vcsel_pulse_period(VCSEL_PERIOD_PRE_RANGE);//ick:formerly called up to api laye
        if (CurrentVCSELPulsePeriodPClk.isOk()) {
          uint16_t MsrcRangeTimeOutMClks = calc_timeout_mclks(TimeOutMicroSecs, CurrentVCSELPulsePeriodPClk);
          uint8_t MsrcEncodedTimeOut = saturated<uint8_t, uint16_t>(MsrcRangeTimeOutMClks - 1);
          VL53L0X_SETDEVICESPECIFICPARAMETER( LastEncodedTimeout, MsrcEncodedTimeOut);
          Error = comm.WrByte(REG_MSRC_CONFIG_TIMEOUT_MACROP, MsrcEncodedTimeOut);
        }
      }
        break;
      case SEQUENCESTEP_PRE_RANGE: {
        Erroneous<uint8_t> CurrentVCSELPulsePeriodPClk = get_vcsel_pulse_period(VCSEL_PERIOD_PRE_RANGE);//ick:formerly called up to api laye
        if (CurrentVCSELPulsePeriodPClk.isOk()) {

          PreRangeTimeOutMClks = calc_timeout_mclks(TimeOutMicroSecs, CurrentVCSELPulsePeriodPClk);
          PreRangeEncodedTimeOut.wrapped = encode_timeout(PreRangeTimeOutMClks);

          VL53L0X_SETDEVICESPECIFICPARAMETER( LastEncodedTimeout, PreRangeEncodedTimeOut);

          Error = comm.WrWord(REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, PreRangeEncodedTimeOut);
          if (Error == ERROR_NONE) {
            VL53L0X_SETDEVICESPECIFICPARAMETER( PreRange.TimeoutMicroSecs, TimeOutMicroSecs);
          }
        }
      }
        break;
      case SEQUENCESTEP_FINAL_RANGE: {
        /* For the final range timeout, the pre-range timeout
         * must be added. To do this both final and pre-range
         * timeouts must be expressed in macro periods MClks
         * because they have different vcsel periods.
         */

        Erroneous<SchedulerSequenceSteps_t> SchedulerSequenceSteps = get_sequence_step_enables();
        PreRangeTimeOutMClks = 0;
        if (SchedulerSequenceSteps.wrapped.PreRangeOn) {
          /* Retrieve PRE-RANGE VCSEL Period */
          Erroneous<uint8_t> CurrentVCSELPulsePeriodPClk = get_vcsel_pulse_period(VCSEL_PERIOD_PRE_RANGE);//ick:formerly called up to api laye
          if (CurrentVCSELPulsePeriodPClk.isOk()) {
            if (fetch(PreRangeEncodedTimeOut, REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI)) {
              PreRangeTimeOutMClks = decode_timeout(PreRangeEncodedTimeOut);
              /* Calculate FINAL RANGE Timeout in Macro Periods
               * (MCLKS) and add PRE-RANGE value
               */
              Erroneous<uint8_t> FinalVCSELPulsePeriodPClk = get_vcsel_pulse_period(VCSEL_PERIOD_FINAL_RANGE);
              if (FinalVCSELPulsePeriodPClk.isOk()) {
                FinalRangeTimeOutMClks = calc_timeout_mclks(TimeOutMicroSecs, (uint8_t) CurrentVCSELPulsePeriodPClk);
                FinalRangeTimeOutMClks += PreRangeTimeOutMClks;
                FinalRangeEncodedTimeOut = encode_timeout(FinalRangeTimeOutMClks);

                Error = comm.WrWord(0x71, FinalRangeEncodedTimeOut);
                if (Error == ERROR_NONE) {
                  VL53L0X_SETDEVICESPECIFICPARAMETER( FinalRange.TimeoutMicroSecs, TimeOutMicroSecs);
                }
              }
            }
          }
        }
      }
        break;
      default:
        return ERROR_INVALID_PARAMS;
    }

    return ERROR_CONTROL_INTERFACE;
  } // set_sequence_step_timeout

  Error Core::setValidPhase(uint8_t high, uint8_t low) {
    comm.WrByte(REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, high);//NB: mimicking old bug of ignoring status of first write
    return comm.WrByte(REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, low);
  }

  Error Core::setPhasecalLimit(uint8_t value) {
    ErrorAccumulator Error {ERROR_NONE};
    //todo: RAII FF thingy
    Error |= comm.WrByte(0xff, 0x01);
    Error |= comm.WrByte(REG_ALGO_PHASECAL_LIM, value);
    Error |= comm.WrByte(0xff, 0x00);
    return Error;
  }


////////////////////////////////////////

  Erroneous<uint8_t> Core::get_vcsel_pulse_period(VcselPeriod VcselPeriodType) {
    Erroneous<uint8_t> vcsel_period_reg;

    switch (VcselPeriodType) {
      case VCSEL_PERIOD_PRE_RANGE:
        fetch(vcsel_period_reg, REG_PRE_RANGE_CONFIG_VCSEL_PERIOD);
        break;
      case VCSEL_PERIOD_FINAL_RANGE:
        fetch(vcsel_period_reg, REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD);
        break;
      default:
        return {ERROR_INVALID_PARAMS};
    } // switch

    if (vcsel_period_reg.isOk()) {
      vcsel_period_reg.wrapped = decode_vcsel_period(vcsel_period_reg);
    }

    return vcsel_period_reg;
  } // VL53L0X_get_vcsel_pulse_period



  Erroneous<SchedulerSequenceSteps_t> Core::get_sequence_step_enables() {
    LOG_FUNCTION_START;
    Erroneous<uint8_t> SequenceConfig;
    if (fetch(SequenceConfig, REG_SYSTEM_SEQUENCE_CONFIG)) {
      return {SequenceConfig.wrapped};
    }
    return {0, SequenceConfig.error};//if error ignored then all steps will be 'off'
  }

  Error Core::set_measurement_timing_budget_micro_seconds(uint32_t MeasurementTimingBudgetMicroSeconds) {
    LOG_FUNCTION_START;
    if (MeasurementTimingBudgetMicroSeconds < cMinTimingBudgetMicroSeconds) {
      return ERROR_INVALID_PARAMS;
    }
    uint32_t FinalRangeTimingBudgetMicroSeconds = MeasurementTimingBudgetMicroSeconds - (StartOverheadMicroSeconds + EndOverheadMicroSeconds);

    Erroneous<SchedulerSequenceSteps_t> SchedulerSequenceSteps = get_sequence_step_enables();

    if (SchedulerSequenceSteps.isOk() && (SchedulerSequenceSteps.wrapped.TccOn || SchedulerSequenceSteps.wrapped.MsrcOn || SchedulerSequenceSteps.wrapped.DssOn)) {

      uint32_t MsrcDccTccTimeoutMicroSeconds = 2000;
      /* TCC, MSRC and DSS all share the same timeout */
      Error |= get_sequence_step_timeout(SEQUENCESTEP_MSRC, &MsrcDccTccTimeoutMicroSeconds);
      ERROR_OUT;

      /* Subtract the TCC, MSRC and DSS timeouts if they are
       * enabled. */

      /* TCC */
      if (SchedulerSequenceSteps.wrapped.TccOn) {
        uint32_t SubTimeout = MsrcDccTccTimeoutMicroSeconds + TccOverheadMicroSeconds;

        if (SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
          FinalRangeTimingBudgetMicroSeconds -= SubTimeout;
        } else {
          /* Requested timeout too big. */
          return ERROR_INVALID_PARAMS;
        }
      }


      /* DSS */
      if (SchedulerSequenceSteps.wrapped.DssOn) {
        uint32_t SubTimeout = 2 * (MsrcDccTccTimeoutMicroSeconds + DssOverheadMicroSeconds);

        if (SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
          FinalRangeTimingBudgetMicroSeconds -= SubTimeout;
        } else {
          /* Requested timeout too big. */
          return ERROR_INVALID_PARAMS;
        }
      } else if (SchedulerSequenceSteps.wrapped.MsrcOn) {
        /* MSRC */
        uint32_t SubTimeout = MsrcDccTccTimeoutMicroSeconds + MsrcOverheadMicroSeconds;

        if (SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
          FinalRangeTimingBudgetMicroSeconds -= SubTimeout;
        } else {
          /* Requested timeout too big. */
          return ERROR_INVALID_PARAMS;
        }
      }
    }

    if (SchedulerSequenceSteps.wrapped.PreRangeOn) {
      /* Subtract the Pre-range timeout if enabled. */
      uint32_t PreRangeTimeoutMicroSeconds = 0;
      Error |= get_sequence_step_timeout(SEQUENCESTEP_PRE_RANGE, &PreRangeTimeoutMicroSeconds);
      ERROR_OUT;
      uint32_t SubTimeout = PreRangeTimeoutMicroSeconds + PreRangeOverheadMicroSeconds;

      if (SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
        FinalRangeTimingBudgetMicroSeconds -= SubTimeout;
      } else {
        /* Requested timeout too big. */
        return ERROR_INVALID_PARAMS;
      }
    }

    if (SchedulerSequenceSteps.wrapped.FinalRangeOn) {
      FinalRangeTimingBudgetMicroSeconds -= FinalRangeOverheadMicroSeconds;

      /* Final Range Timeout
       * Note that the final range timeout is determined by the timing
       * budget and the sum of all other timeouts within the sequence.
       * If there is no room for the final range timeout, then an error
       * will be set. Otherwise the remaining time will be applied to
       * the final range.
       */
      Error |= set_sequence_step_timeout(SEQUENCESTEP_FINAL_RANGE, FinalRangeTimingBudgetMicroSeconds);//BUG: error ignored

      SETPARAMETERFIELD( MeasurementTimingBudgetMicroSeconds, MeasurementTimingBudgetMicroSeconds);
    }

    return Error;
  } // VL53L0X_set_measurement_timing_budget_micro_seconds

  Error Core::get_measurement_timing_budget_micro_seconds(uint32_t *pMeasurementTimingBudgetMicroSeconds) {
    LOG_FUNCTION_START;
    /* Start and end overhead times always present */
    *pMeasurementTimingBudgetMicroSeconds = StartOverheadMicroSecondsBudget + EndOverheadMicroSeconds;
    Erroneous<SchedulerSequenceSteps_t> SchedulerSequenceSteps = get_sequence_step_enables();

    if (~SchedulerSequenceSteps) {
      return SchedulerSequenceSteps.error;
    }
    uint32_t MsrcDccTccTimeoutMicroSeconds = 2000;
    if (SchedulerSequenceSteps.wrapped.TccOn || SchedulerSequenceSteps.wrapped.MsrcOn || SchedulerSequenceSteps.wrapped.DssOn) {
      Error |= get_sequence_step_timeout(SEQUENCESTEP_MSRC, &MsrcDccTccTimeoutMicroSeconds);
      ERROR_OUT;
      if (SchedulerSequenceSteps.wrapped.TccOn) {
        *pMeasurementTimingBudgetMicroSeconds += MsrcDccTccTimeoutMicroSeconds + TccOverheadMicroSeconds;
      }
      if (SchedulerSequenceSteps.wrapped.DssOn) {
        *pMeasurementTimingBudgetMicroSeconds += 2 * (MsrcDccTccTimeoutMicroSeconds + DssOverheadMicroSeconds);
      } else if (SchedulerSequenceSteps.wrapped.MsrcOn) {
        *pMeasurementTimingBudgetMicroSeconds += MsrcDccTccTimeoutMicroSeconds + MsrcOverheadMicroSeconds;
      }
    }
    if (SchedulerSequenceSteps.wrapped.PreRangeOn) {
      uint32_t PreRangeTimeoutMicroSeconds = 0;
      Error |= get_sequence_step_timeout(SEQUENCESTEP_PRE_RANGE, &PreRangeTimeoutMicroSeconds);
      *pMeasurementTimingBudgetMicroSeconds += PreRangeTimeoutMicroSeconds + PreRangeOverheadMicroSeconds;
    }

    if (SchedulerSequenceSteps.wrapped.FinalRangeOn) {
      uint32_t FinalRangeTimeoutMicroSeconds;
      Error |= get_sequence_step_timeout(SEQUENCESTEP_FINAL_RANGE, &FinalRangeTimeoutMicroSeconds);
      *pMeasurementTimingBudgetMicroSeconds += (FinalRangeTimeoutMicroSeconds + FinalRangeOverheadMicroSeconds);
    }
    ERROR_OUT;
    SETPARAMETERFIELD( MeasurementTimingBudgetMicroSeconds, *pMeasurementTimingBudgetMicroSeconds);
    return Error;
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
  Error Core::load_tuning_settings(const uint8_t *pTuningSettingBuffer) {
    LOG_FUNCTION_START;
    int Index = 0;//ick: only value over using the pTuning..Buffer as a pointer might be for debug display.

    while (pTuningSettingBuffer[Index]) {
      uint8_t NumberOfWrites = pTuningSettingBuffer[Index++];
      if (NumberOfWrites == 0xFF) {
        /* internal parameters */
        switch (pTuningSettingBuffer[Index++]) {
          case 0: /* uint16_t SigmaEstRefArray -> 2 bytes */
            PALDevDataSet( SigmaEst.RefArray, unpack(pTuningSettingBuffer, Index));
            break;
          case 1: /* uint16_t SigmaEstEffPulseWidth -> 2 bytes */
            PALDevDataSet( SigmaEst.EffPulseWidth, unpack(pTuningSettingBuffer, Index));
            break;
          case 2: /* uint16_t SigmaEstEffAmbWidth -> 2 bytes */
            PALDevDataSet( SigmaEst.EffAmbWidth, unpack(pTuningSettingBuffer, Index));
            break;
          case 3: /* uint16_t targetRefRate -> 2 bytes */
            PALDevDataSet( targetRefRate, unpack(pTuningSettingBuffer, Index));
            break;
          default: /* invalid parameter */
            return ERROR_INVALID_PARAMS;
        } // switch
      } else if (NumberOfWrites <= 4) { //copy bytes that must be in device endian order.
        uint8_t Address = pTuningSettingBuffer[Index++];
        if (comm.WriteMulti(Address, &pTuningSettingBuffer[Index], NumberOfWrites)) {
          return ERROR_CONTROL_INTERFACE;
        }
        Index+=NumberOfWrites;
      } else {
        return ERROR_INVALID_PARAMS;
      }
    }
    return ERROR_NONE;
  } // VL53L0X_load_tuning_settings

  FixPoint1616_t Core::get_total_xtalk_rate(const RangingMeasurementData_t &pRangingMeasurementData) {

    uint8_t xtalkCompEnable;
    VL53L0X_GETPARAMETERFIELD( XTalkCompensationEnable, xtalkCompEnable);
//todo: lost test value of xtalkCompeEnable

    FixPoint1616_t xtalkPerSpadMegaCps;
    VL53L0X_GETPARAMETERFIELD( XTalkCompensationRateMegaCps, xtalkPerSpadMegaCps);

    /* FixPoint1616 * FixPoint 8:8 = FixPoint0824 */
    FixPoint1616_t totalXtalkMegaCps = pRangingMeasurementData.EffectiveSpadRtnCount * xtalkPerSpadMegaCps.raw;

    /* FixPoint0824 >> 8 = FixPoint1616 */
    return totalXtalkMegaCps.shrink(8);
  } // VL53L0X_get_total_xtalk_rate

  Erroneous<FixPoint1616_t> Core::get_total_signal_rate(const RangingMeasurementData_t &pRangingMeasurementData) {
    LOG_FUNCTION_START;
    Erroneous<FixPoint1616_t> totalXtalkMegaCps {pRangingMeasurementData.SignalRateRtnMegaCps};//default in case returned error is ignored
    return get_total_xtalk_rate(pRangingMeasurementData);
  } // VL53L0X_get_total_signal_rate

  uint32_t Core::calc_dmax(const FixPoint1616_t totalSignalRate_mcps, const FixPoint1616_t totalCorrSignalRate_mcps, const FixPoint1616_t pwMult, const uint32_t sigmaEstimateP1, FixPoint1616_t sigmaEstimateP2, const uint32_t peakVcselDuration_us) {
    const uint32_t cSigmaLimit = 18;
    const FixPoint1616_t cSignalLimit {0.25};// = 0x4000;     /* 0.25 */
    const FixPoint1616_t cSigmaEstRef {0.0001};// = 0x00000042; /* 0.001 */
    const uint32_t cAmbEffWidthSigmaEst_ns = 6;
    const uint32_t cAmbEffWidthDMax_ns = 7;

    LOG_FUNCTION_START;

    uint32_t dmaxCalRange_mm = PALDevDataGet( DmaxCalRangeMilliMeter);
    FixPoint1616_t dmaxCalSignalRateRtn_mcps = PALDevDataGet( DmaxCalSignalRateRtnMegaCps);

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
      minSignalNeeded_p1 = roundedDivide(signalRateTemp_mcps, totalCorrSignalRate_mcps);


      /* Apply a factored version of the speed of light.
       *  Correction to be applied at the end */
      minSignalNeeded_p1 *= 3;

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
    minSignalNeeded = minSignalNeeded_p1.raw * roundedDivide(minSignalNeeded, minSignalNeeded_p4.raw).raw;

    /* Apply correction by dividing by 1000000.
     * This assumes 10E16 on the numerator of the equation
     * and 10E-22 on the denominator.
     * We do this because 32bit fix point calculation can't
     * handle the larger and smaller elements of this equation,
     * i.e. speed of light and pulse widths.
     */
    minSignalNeeded = roundedDivide(minSignalNeeded, 1000);
    minSignalNeeded.boost(4);
    minSignalNeeded = roundedDivide(minSignalNeeded, 1000);//BUG: perhaps 980F screwed this up? Rounding each division by 1000 to get to /1000,000 is wrong if rounded twice.

    /* FixPoint2408/FixPoint2408 = uint32 */
    FixPoint1616_t dmaxAmbient = isqrt(roundedDivide(SignalAt0mm.raw, minSignalNeeded));

    /* FixPoint1616 >> 8 = FixPoint2408 */
    FixPoint1616_t signalLimitTmp = cSignalLimit.scaled(8);

    /* FixPoint2408/FixPoint2408 = uint32 */
    FixPoint1616_t dmaxDark = isqrt(roundedDivide(SignalAt0mm.raw, signalLimitTmp));//former check for zero moved into roundedDivide

    dmaxDark.lessen(dmaxAmbient);
    return dmaxDark.raw;
  } // calc_dmax

  Erroneous<FixPoint1616_t> Core::calc_sigma_estimate(const RangingMeasurementData_t &pRangingMeasurementData, uint32_t &pDmm) {
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

    uint32_t finalRangeTimeoutMicroSecs;
    uint32_t preRangeTimeoutMicroSecs;

    uint32_t vcselWidth;
    uint32_t finalRangeMacroPCLKS;
    uint32_t preRangeMacroPCLKS;
    uint32_t peakVcselDuration_us;
    uint8_t finalRangeVcselPCLKS;
    uint8_t preRangeVcselPCLKS;
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
    Erroneous<FixPoint1616_t> xTalkCompRate_mcps;
    VL53L0X_GETPARAMETERFIELD( XTalkCompensationRateMegaCps, xTalkCompRate_mcps);

    /*
     * We work in kcps rather than mcps as this helps keep within the
     * confines of the 32 Fix1616 type.
     */

    FixPoint1616_t ambientRate_kcps = pRangingMeasurementData.AmbientRateRtnMegaCps.millis();//980f: now rounded instead of truncated

    FixPoint1616_t correctedSignalRate_mcps = pRangingMeasurementData.SignalRateRtnMegaCps;

    FixPoint1616_t totalSignalRate_mcps = get_total_signal_rate(pRangingMeasurementData);

    xTalkCompRate_mcps = get_total_xtalk_rate(pRangingMeasurementData);

    if (~xTalkCompRate_mcps) {
      return {xTalkCompRate_mcps.error};
    }
    /* Signal rate measurement provided by device is the
     * peak signal rate, not average.
     */
    FixPoint1616_t peakSignalRate_kcps = totalSignalRate_mcps.millis();
    uint32_t xTalkCompRate_kcps = min(xTalkCompRate_mcps.wrapped * 1000, cMaxXTalk_kcps.raw);


    /* Calculate final range macro periods */
    finalRangeTimeoutMicroSecs = VL53L0X_GETDEVICESPECIFICPARAMETER( FinalRange.TimeoutMicroSecs);

    finalRangeVcselPCLKS = VL53L0X_GETDEVICESPECIFICPARAMETER( FinalRange.VcselPulsePeriod);

    finalRangeMacroPCLKS = calc_timeout_mclks(finalRangeTimeoutMicroSecs, finalRangeVcselPCLKS);

    /* Calculate pre-range macro periods */
    preRangeTimeoutMicroSecs = VL53L0X_GETDEVICESPECIFICPARAMETER( PreRange.TimeoutMicroSecs);

    preRangeVcselPCLKS = VL53L0X_GETDEVICESPECIFICPARAMETER( PreRange.VcselPulsePeriod);

    preRangeMacroPCLKS = calc_timeout_mclks(preRangeTimeoutMicroSecs, preRangeVcselPCLKS);

    vcselWidth = (finalRangeVcselPCLKS == 8) ? 2 : 3;

    peakVcselDuration_us = milli((vcselWidth << 11) * (preRangeMacroPCLKS + finalRangeMacroPCLKS));
    peakVcselDuration_us *= cPllPeriod_ps;
    peakVcselDuration_us = milli(peakVcselDuration_us);

    /* Fix1616 >> 8 = Fix2408 */
    totalSignalRate_mcps = totalSignalRate_mcps.shrink(8);

    /* Fix2408 * uint32 = Fix2408
     * then Fix2408 >> 8 = uint32 */

    uint32_t vcselTotalEventsRtn = roundedScale(totalSignalRate_mcps.raw * peakVcselDuration_us, 8);

    /* Fix2408 << 8 = Fix1616 = */
    totalSignalRate_mcps.boost(8);

    if (peakSignalRate_kcps.raw == 0) {

      PALDevDataSet( SigmaEstimate, cSigmaEstMax);
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
      FixPoint1616_t sigmaEstimateP2 = (ambientRate_kcps.raw << 16) / peakSignalRate_kcps.raw;

      if (sigmaEstimateP2.raw > cAmbToSignalRatioMax.raw) {
        /* Clip to prevent overflow. Will ensure safe
         * max result. */
        sigmaEstimateP2 = cAmbToSignalRatioMax;
      }

      sigmaEstimateP2.raw *= cAmbientEffectiveWidth_centi_ns;

      FixPoint1616_t sigmaEstimateP3 = 2 * isqrt(vcselTotalEventsRtn * 12);

      /* uint32 * FixPoint1616 = FixPoint1616 */
      FixPoint1616_t deltaT_ps = pRangingMeasurementData.RangeMilliMeter * cTOF_per_mm_ps.raw;

      /*
       * vcselRate - xtalkCompRate
       * (uint32 << 16) - FixPoint1616 = FixPoint1616.
       * Divide result by 1000 to convert to mcps.
       * 500 is added to ensure rounding when integer division
       * truncates.
       */
      FixPoint1616_t diff1_mcps = roundedDivide(peakSignalRate_kcps.boosted(16) - xTalkCompRate_kcps, 1000);

      /* vcselRate + xtalkCompRate */
      FixPoint1616_t diff2_mcps = roundedDivide((peakSignalRate_kcps.boosted(16) + xTalkCompRate_kcps), 1000);

      /* Shift by 8 bits to increase resolution prior to the division */
      diff1_mcps.boost(8);

      /* FixPoint0824/FixPoint1616 = FixPoint2408 */
      FixPoint1616_t xTalkCorrection = abs(((long long) diff1_mcps.raw / diff2_mcps.raw));

      /* FixPoint2408 << 8 = FixPoint1616 */
      xTalkCorrection.boost(8);

      /* FixPoint1616/uint32 = FixPoint1616 */
      FixPoint1616_t pwMult = deltaT_ps.raw / cVcselPulseWidth_ps; /* smaller than 1.0f */ //ick: not rounded

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
      FixPoint1616_t sqr1 = pwMult.raw * sigmaEstimateP1.raw;
      /* (FixPoint1616 >> 16) = FixPoint3200 */
      sqr1 = roundedScale(sqr1, 16);

      FixPoint1616_t sqr2 = sigmaEstimateP2;
      /* (FixPoint1616 >> 16) = FixPoint3200 */
      sqr2.shrink(16);

      /* SQRT(FixPoin6400) = FixPoint3200 */
      FixPoint1616_t sqrtResult_centi_ns = quadrature_sum(sqr1, sqr2);

      /* (FixPoint3200 << 16) = FixPoint1616 */
      sqrtResult_centi_ns.boost(16);

      /*
       * Note that the Speed Of Light is expressed in um per 1E-10
       * seconds (2997) Therefore to get mm/ns we have to divide by
       * 10000
       */
      FixPoint1616_t sigmaEstRtn = ((roundedDivide(sqrtResult_centi_ns, 100).raw / sigmaEstimateP3.raw));//ick: why not rounded?
      sigmaEstRtn.raw *= SPEED_OF_LIGHT_IN_AIR;
      sigmaEstRtn.divideby(10000);
      /* Clip to prevent overflow. Will ensure safe max result. */
      sigmaEstRtn.lessen(cSigmaEstRtnMax);

      /* sqrt(FixPoint3232) = FixPoint1616 */
      FixPoint1616_t sqrtResult = quadrature_sum(sigmaEstRtn, cSigmaEstRef);
      /*
       * Note that the Shift by 4 bits increases resolution prior to
       * the sqrt, therefore the result must be shifted by 2 bits to
       * the right to revert back to the FixPoint1616 format.
       */

      FixPoint1616_t sigmaEstimate = 1000 * sqrtResult;

      if ((peakSignalRate_kcps.raw < 1) || (vcselTotalEventsRtn < 1) || (sigmaEstimate.raw > cSigmaEstMax.raw)) {
        sigmaEstimate = cSigmaEstMax;
      }

      pDmm = calc_dmax(totalSignalRate_mcps, correctedSignalRate_mcps, pwMult, sigmaEstimateP1, sigmaEstimateP2, peakVcselDuration_us);
      PALDevDataSet( SigmaEstimate, sigmaEstimate);
      return {sigmaEstimate};
    }

    return ERROR_NONE;
  }

  Erroneous<bool> Core::GetSequenceStepEnable(SequenceStepId StepId) {
    LOG_FUNCTION_START;
    auto seqbit = bitFor(StepId);
    if (seqbit ==~0) {
      return LOG_ERROR( ERROR_INVALID_PARAMS);
    }

    Erroneous<uint8_t> SequenceConfig;
    if (fetch(SequenceConfig, REG_SYSTEM_SEQUENCE_CONFIG)) {
      return {getBit(seqbit, SequenceConfig.wrapped)};
    }
    return {LOG_ERROR( SequenceConfig.error)};
  }

  Erroneous<bool> Core::GetLimitCheckEnable(CheckEnable LimitCheckId){
    if (LimitCheckId >= CHECKENABLE_NUMBER_OF_CHECKS) {
      return ERROR_INVALID_PARAMS;
    }
    return Data.CurrentParameters.LimitChecksEnable[LimitCheckId];
  } // GetLimitCheckEnable


  Error Core::get_pal_range_status(uint8_t DeviceRangeStatus, FixPoint1616_t SignalRate, uint16_t EffectiveSpadRtnCount,  RangingMeasurementData_t &pRangingMeasurementData, uint8_t *pPalRangeStatus) {
    LOG_FUNCTION_START;

    /*
     * VL53L0X has a good ranging when the value of the
     * DeviceRangeStatus = 11. This function will replace the value 0 with
     * the value 11 in the DeviceRangeStatus.
     * In addition, the SigmaEstimator is not included in the VL53L0X
     * DeviceRangeStatus, this will be added in the PalRangeStatus.
     */
    uint8_t DeviceRangeStatusInternal = getBits<6,3>(DeviceRangeStatus);
    uint8_t NoneFlag = (DeviceRangeStatusInternal == 0 || DeviceRangeStatusInternal == 5 || DeviceRangeStatusInternal == 7 || DeviceRangeStatusInternal == 12 || DeviceRangeStatusInternal == 13 || DeviceRangeStatusInternal == 14 || DeviceRangeStatusInternal == 15);

    Erroneous<uint16_t> tmpWord;
    /* LastSignalRefMcps */
    {
      auto pager = autoCloser(Private_Pager, 0x01, 0x00);
      fetch(tmpWord, REG_RESULT_PEAK_SIGNAL_RATE_REF);
    }

    FixPoint1616_t LastSignalRefMcps = VL53L0X_FIXPOINT97TOFIXPOINT1616(tmpWord);//ick: ignoring error on getting tmpWord value.
    PALDevDataSet( LastSignalRefMcps, LastSignalRefMcps);

    /*
     * Check if Sigma limit is enabled, if yes then do comparison with limit
     * value and put the result back into pPalRangeStatus.
     */
    Erroneous<bool> SigmaLimitCheckEnable = GetLimitCheckEnable(CHECKENABLE_SIGMA_FINAL_RANGE);

    bool SigmaLimitflag = false;
    if (SigmaLimitCheckEnable.isOk() && SigmaLimitCheckEnable != 0) {
      /*
       * compute the Sigma and check with limit
       */
      uint32_t Dmax_mm = 0;
      Erroneous<FixPoint1616_t> SigmaEstimate = calc_sigma_estimate(pRangingMeasurementData, Dmax_mm);
      if (SigmaEstimate.isOk()) {
        pRangingMeasurementData.RangeDMaxMilliMeter = Dmax_mm;
        auto SigmaLimitValue = GetLimitCheckValue( CHECKENABLE_SIGMA_FINAL_RANGE);
        if ((SigmaLimitValue > 0) && (SigmaEstimate.wrapped > SigmaLimitValue)) {
          /* Limit Fail */
          SigmaLimitflag = true;
        }
      }

    }

    /*
     * Check if Signal ref clip limit is enabled, if yes then do comparison
     * with limit value and put the result back into pPalRangeStatus.
     */
    uint8_t SignalRefClipLimitCheckEnable = GetLimitCheckEnable( CHECKENABLE_SIGNAL_REF_CLIP);


    uint8_t SignalRefClipflag = 0;
    if ((SignalRefClipLimitCheckEnable != 0) && (Error == ERROR_NONE)) {
      FixPoint1616_t SignalRefClipValue=GetLimitCheckValue( CHECKENABLE_SIGNAL_REF_CLIP);
      if ((SignalRefClipValue > 0) && (LastSignalRefMcps > SignalRefClipValue)) {
        /* Limit Fail */
        SignalRefClipflag = 1;
      }
    }

    /*
     * Check if Signal ref clip limit is enabled, if yes then do comparison
     * with limit value and put the result back into pPalRangeStatus.
     * EffectiveSpadRtnCount has a format 8.8
     * If (Return signal rate < (1.5 x Xtalk x number of Spads)) : FAIL
     */
    uint8_t RangeIgnoreThresholdLimitCheckEnable = 0;
    if (Error == ERROR_NONE) {
      Error = GetLimitCheckEnable( CHECKENABLE_RANGE_IGNORE_THRESHOLD, &RangeIgnoreThresholdLimitCheckEnable);
    }
    uint8_t RangeIgnoreThresholdflag = 0;

    if ((RangeIgnoreThresholdLimitCheckEnable != 0) && (Error == ERROR_NONE)) {
      /* Compute the signal rate per spad */
      FixPoint1616_t SignalRatePerSpad = (EffectiveSpadRtnCount != 0) ? (FixPoint1616_t) ((256 * SignalRate) / EffectiveSpadRtnCount) : 0;

      FixPoint1616_t RangeIgnoreThresholdValue;
      Error = GetLimitCheckValue( CHECKENABLE_RANGE_IGNORE_THRESHOLD, &RangeIgnoreThresholdValue);

      if ((RangeIgnoreThresholdValue > 0) && (SignalRatePerSpad < RangeIgnoreThresholdValue)) {
        /* Limit Fail add 2^6 to range status */
        RangeIgnoreThresholdflag = 1;
      }
    }

    if (Error == ERROR_NONE) {
      if (NoneFlag == 1) {
        *pPalRangeStatus = 255; /* NONE */
      } else if (DeviceRangeStatusInternal == 1 || DeviceRangeStatusInternal == 2 || DeviceRangeStatusInternal == 3) {
        *pPalRangeStatus = 5; /* HW fail */
      } else if (DeviceRangeStatusInternal == 6 || DeviceRangeStatusInternal == 9) {
        *pPalRangeStatus = 4; /* Phase fail */
      } else if (DeviceRangeStatusInternal == 8 || DeviceRangeStatusInternal == 10 || SignalRefClipflag == 1) {
        *pPalRangeStatus = 3; /* Min range */
      } else if (DeviceRangeStatusInternal == 4 || RangeIgnoreThresholdflag == 1) {
        *pPalRangeStatus = 2; /* Signal Fail */
      } else if (SigmaLimitflag == 1) {
        *pPalRangeStatus = 1; /* Sigma	 Fail */
      } else {
        *pPalRangeStatus = 0; /* Range Valid */
      }
    }

    /* DMAX only relevant during range error */
    if (*pPalRangeStatus == 0) {
      pRangingMeasurementData.RangeDMaxMilliMeter = 0;
    }

    /* fill the Limit Check Error */
    bool SignalRateFinalRangeLimitCheckEnable  = GetLimitCheckEnable( CHECKENABLE_SIGNAL_RATE_FINAL_RANGE);

      VL53L0X_SETARRAYPARAMETERFIELD( LimitChecksStatus, CHECKENABLE_SIGMA_FINAL_RANGE, (SigmaLimitCheckEnable == 0) || (SigmaLimitflag == 1));
      VL53L0X_SETARRAYPARAMETERFIELD( LimitChecksStatus, CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (DeviceRangeStatusInternal == 4) || !SignalRateFinalRangeLimitCheckEnable);
      VL53L0X_SETARRAYPARAMETERFIELD( LimitChecksStatus, CHECKENABLE_SIGNAL_REF_CLIP, (SignalRefClipLimitCheckEnable == 0) || (SignalRefClipflag == 1));
      VL53L0X_SETARRAYPARAMETERFIELD( LimitChecksStatus, CHECKENABLE_RANGE_IGNORE_THRESHOLD, (RangeIgnoreThresholdLimitCheckEnable == 0) || (RangeIgnoreThresholdflag == 1));

    return Error;
  } // VL53L0X_get_pal_range_status


  Error Core::set_ref_spad_map( SpadArray &refSpadArray) {
    return comm.WriteMulti(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_BASE, reinterpret_cast<uint8_t *>(&refSpadArray), SpadArray::NumberOfBytes);
  }

  Error Core::get_ref_spad_map( SpadArray &refSpadArray) {
    return comm.ReadMulti(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_BASE, reinterpret_cast<uint8_t *>(&refSpadArray), SpadArray::NumberOfBytes);
  }

} //end namespace
