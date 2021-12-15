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

#include "vl53l0x_api_core.h"
#include "vl53l0x_api.h"
#include "vl53l0x_api_calibration.h"

#include "log_api.h"

using namespace VL53L0X; //violates general rule of not using blanket import as this is the implementation of the namespace
//we will be getting rid of this, so dup something to get a compile until we do.
#define VL53L0X_COPYSTRING(target,string) strcpy(target,string)

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

Erroneous<bool> Api::measurement_poll_for_completion() {
  Erroneous<bool> value(false);
  uint8_t NewDataReady = 0;


  LOG_FUNCTION_START("");

  for(unsigned LoopNb = VL53L0X_DEFAULT_MAX_LOOP;LoopNb-->0;) {
    Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDataReady);
    if (Status != 0) {
      break; /* the error is set */
    }
    if (NewDataReady == 1) {
      break; /* done note that status == 0 */
    }
    if () {
      Status =
      break;
    }
    VL53L0X_PollingDelay(Dev);
  } while (1);
  return {ERROR_TIME_OUT,false};
} // VL53L0X_measurement_poll_for_completion

uint8_t VL53L0X_decode_vcsel_period(uint8_t vcsel_period_reg) {
  /*!
   * Converts the encoded VCSEL period register value into the real
   * period in PLL clocks
   */
  return (vcsel_period_reg + 1) << 1;
}

uint8_t VL53L0X_encode_vcsel_period(uint8_t vcsel_period_pclks) {
  /*!
   * Converts the encoded VCSEL period register value into the real period
   * in PLL clocks
   */
  return (vcsel_period_pclks >> 1) - 1;
}

uint32_t VL53L0X_isqrt(uint32_t num) {
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

uint32_t VL53L0X_quadrature_sum(uint32_t a, uint32_t b) {
  /*
   * Implements a quadrature sum
   *
   * rea = sqrt(a^2 + b^2)
   *
   * Trap overflow case max input value is 65535 (16-bit value)
   * as internal calc are 32-bit wide
   *
   * If overflow then seta output to maximum
   */
  if (a > 65535 || b > 65535) {
    return 65535;
  }
  return VL53L0X_isqrt(a * a + b * b);
} // VL53L0X_quadrature_sum

VL53L0X_Error VL53L0X_device_read_strobe(VL53L0X_DEV Dev) {
  LOG_FUNCTION_START("");
  VL53L0X_Error Status = VL53L0X_WrByte(Dev, 0x83, 0x00);
  /* polling
   * use timeout to avoid deadlock*/
  if (Status == VL53L0X_ERROR_NONE) {
    uint32_t LoopNb = 0;
    do {
      uint8_t strobe = 0;//init in case RdByte doesn't execute.
      Status = VL53L0X_RdByte(Dev, 0x83, &strobe);
      if ((strobe != 0x00) || Status != VL53L0X_ERROR_NONE) {//BUG: why isnspect strobe if we have an error reading it?
        break;
      }
    } while (++LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

    if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
      Status = VL53L0X_ERROR_TIME_OUT;
    }
  }
  Status |= VL53L0X_WrByte(Dev, 0x83, 0x01);
//BUG: if timetout we still call WrByte and if it has an error then we return a garbled status.

  return Status;
} // VL53L0X_device_read_strobe

VL53L0X_Error VL53L0X_get_info_from_device(VL53L0X_DEV Dev, uint8_t option) {
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  uint8_t ModuleId = 255;//BUG: is used when might not have been read, added init value that hopefully will blow.
  uint8_t Revision = 255;//BUG: see ModuleId.
  uint8_t ReferenceSpadCount = 0;
  uint8_t ReferenceSpadType = 0;
  uint32_t PartUIDUpper = 0;
  uint32_t PartUIDLower = 0;
  uint32_t OffsetFixed1104_mm = 0;
  int16_t OffsetMicroMeters = 0;
  uint32_t DistMeasTgtFixed1104_mm = 400 << 4;
  uint32_t DistMeasFixed1104_400_mm = 0;
  uint32_t SignalRateMeasFixed1104_400_mm = 0;

  FixPoint1616_t SignalRateMeasFixed400mmFix = 0;
  uint8_t NvmRefGoodSpadMap[VL53L0X_REF_SPAD_BUFFER_SIZE];
  int i;

  LOG_FUNCTION_START("");
  uint8_t ReadDataFromDeviceDone = VL53L0X_GETDEVICESPECIFICPARAMETER(Dev, ReadDataFromDeviceDone);

  /* This access is done only once after that a GetDeviceInfo or
   * datainit is done*/
  if (ReadDataFromDeviceDone != 7) {

    Status |= VL53L0X_WrByte(Dev, 0x80, 0x01);
    Status |= VL53L0X_WrByte(Dev, 0xFF, 0x01);
    Status |= VL53L0X_WrByte(Dev, 0x00, 0x00);

    Status |= VL53L0X_WrByte(Dev, 0xFF, 0x06);
    uint8_t byte;//todo: use update call instead of this interruptable update
    Status |= VL53L0X_RdByte(Dev, 0x83, &byte);
    Status |= VL53L0X_WrByte(Dev, 0x83, byte | 4);

    Status |= VL53L0X_WrByte(Dev, 0xFF, 0x07);
    Status |= VL53L0X_WrByte(Dev, 0x81, 0x01);

    Status |= VL53L0X_PollingDelay(Dev);

    Status |= VL53L0X_WrByte(Dev, 0x80, 0x01);

    if (((option & 1) == 1) && ((ReadDataFromDeviceDone & 1) == 0)) {
      uint32_t TmpDWord;
      Status |= VL53L0X_WrByte(Dev, 0x94, 0x6b);
      Status |= VL53L0X_device_read_strobe(Dev);
      Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);

      ReferenceSpadCount = (uint8_t) ((TmpDWord >> 8) & 0x07f);
      ReferenceSpadType = (uint8_t) ((TmpDWord >> 15) & 0x01);

      Status |= VL53L0X_WrByte(Dev, 0x94, 0x24);
      Status |= VL53L0X_device_read_strobe(Dev);
      Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);

      NvmRefGoodSpadMap[0] = (uint8_t) ((TmpDWord >> 24) & 0xff);
      NvmRefGoodSpadMap[1] = (uint8_t) ((TmpDWord >> 16) & 0xff);
      NvmRefGoodSpadMap[2] = (uint8_t) ((TmpDWord >> 8) & 0xff);
      NvmRefGoodSpadMap[3] = (uint8_t) (TmpDWord & 0xff);

      Status |= VL53L0X_WrByte(Dev, 0x94, 0x25);
      Status |= VL53L0X_device_read_strobe(Dev);
      Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);

      NvmRefGoodSpadMap[4] = (uint8_t) ((TmpDWord >> 24) & 0xff);
      NvmRefGoodSpadMap[5] = (uint8_t) ((TmpDWord >> 16) & 0xff);
    }

    if (((option & 2) == 2) && ((ReadDataFromDeviceDone & 2) == 0)) {

      Status |= VL53L0X_WrByte(Dev, 0x94, 0x02);
      Status |= VL53L0X_device_read_strobe(Dev);
      Status |= VL53L0X_RdByte(Dev, 0x90, &ModuleId);

      Status |= VL53L0X_WrByte(Dev, 0x94, 0x7B);
      Status |= VL53L0X_device_read_strobe(Dev);
      Status |= VL53L0X_RdByte(Dev, 0x90, &Revision);

      Status |= VL53L0X_WrByte(Dev, 0x94, 0x77);
      Status |= VL53L0X_device_read_strobe(Dev);
      uint32_t TmpDWord;
      Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);

      ProductId[0] = (char) ((TmpDWord >> 25) & 0x07f);
      ProductId[1] = (char) ((TmpDWord >> 18) & 0x07f);
      ProductId[2] = (char) ((TmpDWord >> 11) & 0x07f);
      ProductId[3] = (char) ((TmpDWord >> 4) & 0x07f);

      byte = (uint8_t) ((TmpDWord & 0x00f) << 3);

      Status |= VL53L0X_WrByte(Dev, 0x94, 0x78);
      Status |= VL53L0X_device_read_strobe(Dev);
      Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);

      ProductId[4] = (char) (byte + ((TmpDWord >> 29) & 0x07f));
      ProductId[5] = (char) ((TmpDWord >> 22) & 0x07f);
      ProductId[6] = (char) ((TmpDWord >> 15) & 0x07f);
      ProductId[7] = (char) ((TmpDWord >> 8) & 0x07f);
      ProductId[8] = (char) ((TmpDWord >> 1) & 0x07f);

      byte = (uint8_t) ((TmpDWord & 0x001) << 6);

      Status |= VL53L0X_WrByte(Dev, 0x94, 0x79);

      Status |= VL53L0X_device_read_strobe(Dev);

      Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);

      ProductId[9] = (char) (byte + ((TmpDWord >> 26) & 0x07f));
      ProductId[10] = (char) ((TmpDWord >> 19) & 0x07f);
      ProductId[11] = (char) ((TmpDWord >> 12) & 0x07f);
      ProductId[12] = (char) ((TmpDWord >> 5) & 0x07f);

      byte = (uint8_t) ((TmpDWord & 0x01f) << 2);

      Status |= VL53L0X_WrByte(Dev, 0x94, 0x7A);

      Status |= VL53L0X_device_read_strobe(Dev);

      Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);

      ProductId[13] = (char) (byte + ((TmpDWord >> 30) & 0x07f));
      ProductId[14] = (char) ((TmpDWord >> 23) & 0x07f);
      ProductId[15] = (char) ((TmpDWord >> 16) & 0x07f);
      ProductId[16] = (char) ((TmpDWord >> 9) & 0x07f);
      ProductId[17] = (char) ((TmpDWord >> 2) & 0x07f);
      ProductId[18] = '\0';
    }

    if (((option & 4) == 4) && ((ReadDataFromDeviceDone & 4) == 0)) {

      Status |= VL53L0X_WrByte(Dev, 0x94, 0x7B);
      Status |= VL53L0X_device_read_strobe(Dev);
      Status |= VL53L0X_RdDWord(Dev, 0x90, &PartUIDUpper);

      Status |= VL53L0X_WrByte(Dev, 0x94, 0x7C);
      Status |= VL53L0X_device_read_strobe(Dev);
      Status |= VL53L0X_RdDWord(Dev, 0x90, &PartUIDLower);

      Status |= VL53L0X_WrByte(Dev, 0x94, 0x73);
      Status |= VL53L0X_device_read_strobe(Dev);
      uint32_t TmpDWord;
      Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);

      SignalRateMeasFixed1104_400_mm = (TmpDWord & 0x0000000ff) << 8;

      Status |= VL53L0X_WrByte(Dev, 0x94, 0x74);
      Status |= VL53L0X_device_read_strobe(Dev);
      Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);

      SignalRateMeasFixed1104_400_mm |= ((TmpDWord & 0xff000000) >> 24);

      Status |= VL53L0X_WrByte(Dev, 0x94, 0x75);
      Status |= VL53L0X_device_read_strobe(Dev);
      Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);

      DistMeasFixed1104_400_mm = (TmpDWord & 0x0000000ff) << 8;

      Status |= VL53L0X_WrByte(Dev, 0x94, 0x76);
      Status |= VL53L0X_device_read_strobe(Dev);
      Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);

      DistMeasFixed1104_400_mm |= ((TmpDWord & 0xff000000) >> 24);
    }

    Status |= VL53L0X_WrByte(Dev, 0x81, 0x00);
    Status |= VL53L0X_WrByte(Dev, 0xFF, 0x06);
    Status |= VL53L0X_RdByte(Dev, 0x83, &byte);
    Status |= VL53L0X_WrByte(Dev, 0x83, byte & 0xfb);
    Status |= VL53L0X_WrByte(Dev, 0xFF, 0x01);
    Status |= VL53L0X_WrByte(Dev, 0x00, 0x01);

    Status |= VL53L0X_WrByte(Dev, 0xFF, 0x00);
    Status |= VL53L0X_WrByte(Dev, 0x80, 0x00);
  }

  if ((Status == VL53L0X_ERROR_NONE) && (ReadDataFromDeviceDone != 7)) {
    /* Assign to variable if status is ok */
    if (((option & 1) == 1) && ((ReadDataFromDeviceDone & 1) == 0)) {
      VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, ReferenceSpadCount, ReferenceSpadCount);
      VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, ReferenceSpadType, ReferenceSpadType);

      for (i = 0; i < VL53L0X_REF_SPAD_BUFFER_SIZE; i++) {
        Dev->Data.SpadData.RefGoodSpadMap[i] = NvmRefGoodSpadMap[i];
      }
    }

    if (((option & 2) == 2) && ((ReadDataFromDeviceDone & 2) == 0)) {
      VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, ModuleId, ModuleId);
      VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, Revision, Revision);
      VL53L0X_COPYSTRING(VL53L0X_GETDEVICESPECIFICPARAMETER(Dev, ProductId), ProductId);
    }

    if (((option & 4) == 4) && ((ReadDataFromDeviceDone & 4) == 0)) {
      VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, PartUIDUpper, PartUIDUpper);
      VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, PartUIDLower, PartUIDLower);
      SignalRateMeasFixed400mmFix = VL53L0X_FIXPOINT97TOFIXPOINT1616(SignalRateMeasFixed1104_400_mm);
      VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, SignalRateMeasFixed400mm, SignalRateMeasFixed400mmFix);

      OffsetMicroMeters = 0;
      if (DistMeasFixed1104_400_mm != 0) {
        OffsetFixed1104_mm = DistMeasFixed1104_400_mm - DistMeasTgtFixed1104_mm;
        OffsetMicroMeters = (OffsetFixed1104_mm * 1000) >> 4;
        OffsetMicroMeters *= -1;
      }
      PALDevDataSet(Dev, Part2PartOffsetAdjustmentNVMMicroMeter, OffsetMicroMeters);
    }
    VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, ReadDataFromDeviceDone, (ReadDataFromDeviceDone | option));
  }


  return Status;
} // VL53L0X_get_info_from_device

uint32_t VL53L0X_calc_macro_period_ps(VL53L0X_DEV Dev, uint8_t vcsel_period_pclks) {
  LOG_FUNCTION_START("");
  /* The above calculation will produce rounding errors,  therefore set fixed value //ICK: comment refers to non-existent code.
   */
  const uint64_t PLL_period_ps = 1655;
  const uint32_t macro_period_vclks = 2304;
  return  (uint32_t) (macro_period_vclks * vcsel_period_pclks * PLL_period_ps);
} // VL53L0X_calc_macro_period_ps

uint16_t VL53L0X_encode_timeout(uint32_t timeout_macro_clks) {
  /*!
   * Encode timeout in macro periods in (LSByte * 2^MSByte) + 1 format
   */

  if (timeout_macro_clks > 0) {
    uint32_t ls_byte = timeout_macro_clks - 1;
    uint16_t ms_byte = 0;
    while ((ls_byte & 0xFFFFFF00) > 0) {
      ls_byte = ls_byte >> 1;
      ms_byte++;
    }
    return (ms_byte << 8) + ls_byte;//the while repeats until all the bits that the mask formerly here masked is zero, so no point in masking.
  } else {
    return 0;
  }
} // VL53L0X_encode_timeout

uint32_t VL53L0X_decode_timeout(uint16_t encoded_timeout) {
  /*!
   * Decode 16-bit timeout register value - format (LSByte * 2^MSByte) + 1
   */
  return ((uint32_t) (encoded_timeout & 0x00FF) << (uint32_t) ((encoded_timeout & 0xFF00) >> 8)) + 1;
} // VL53L0X_decode_timeout

/* To convert ms into register value */
uint32_t VL53L0X_calc_timeout_mclks(VL53L0X_DEV Dev, uint32_t timeout_period_us, uint8_t vcsel_period_pclks) {
  uint32_t macro_period_ps = VL53L0X_calc_macro_period_ps(Dev, vcsel_period_pclks);
  uint32_t macro_period_ns = (macro_period_ps + 500) / 1000;
  return (uint32_t) (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
} // VL53L0X_calc_timeout_mclks

/* To convert register value into us */
uint32_t VL53L0X_calc_timeout_us(VL53L0X_DEV Dev, uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks) {
  uint32_t macro_period_ps = VL53L0X_calc_macro_period_ps(Dev, vcsel_period_pclks);
  uint32_t macro_period_ns = (macro_period_ps + 500) / 1000;
  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
} // VL53L0X_calc_timeout_us

VL53L0X_Error get_sequence_step_timeout(VL53L0X_DEV Dev, VL53L0X_SequenceStepId SequenceStepId, uint32_t *pTimeOutMicroSecs) {////BUG: uses many values even when they were not successfully fetched.
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;

  uint32_t TimeoutMicroSeconds = 0;
  switch (SequenceStepId) {
    case VL53L0X_SEQUENCESTEP_TCC:
    case VL53L0X_SEQUENCESTEP_DSS:
    case VL53L0X_SEQUENCESTEP_MSRC: {
      uint8_t CurrentVCSELPulsePeriodPClk;

      Status = VL53L0X_GetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, &CurrentVCSELPulsePeriodPClk);
      uint8_t EncodedTimeOutByte = 0;
      if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_RdByte(Dev, VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP, &EncodedTimeOutByte);
      }
      uint16_t MsrcTimeOutMClks = VL53L0X_decode_timeout(EncodedTimeOutByte);    //BUG: uses EncodedTimeOutByte even when not fetched.
      TimeoutMicroSeconds = VL53L0X_calc_timeout_us(Dev, MsrcTimeOutMClks, CurrentVCSELPulsePeriodPClk);//BUG: uses CurrentVCSELPulsePeriodPClk even when not fetched.
    }
      break;
    case VL53L0X_SEQUENCESTEP_PRE_RANGE: {
      /* Retrieve PRE-RANGE VCSEL Period */
      uint8_t CurrentVCSELPulsePeriodPClk;
      Status = VL53L0X_GetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, &CurrentVCSELPulsePeriodPClk);
      /* Retrieve PRE-RANGE Timeout in Macro periods (MCLKS) */
      if (Status == VL53L0X_ERROR_NONE) {
        /* Retrieve PRE-RANGE VCSEL Period */
        Status = VL53L0X_GetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, &CurrentVCSELPulsePeriodPClk);
        uint16_t PreRangeEncodedTimeOut = 0;
        if (Status == VL53L0X_ERROR_NONE) {
          Status = VL53L0X_RdWord(Dev, VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &PreRangeEncodedTimeOut);
        }
        uint16_t PreRangeTimeOutMClks = VL53L0X_decode_timeout(PreRangeEncodedTimeOut);
        TimeoutMicroSeconds = VL53L0X_calc_timeout_us(Dev, PreRangeTimeOutMClks, CurrentVCSELPulsePeriodPClk);
      }
    }
      break;
    case VL53L0X_SEQUENCESTEP_FINAL_RANGE: {
      VL53L0X_SchedulerSequenceSteps_t SchedulerSequenceSteps;
      VL53L0X_GetSequenceStepEnables(Dev, &SchedulerSequenceSteps);
      uint16_t PreRangeTimeOutMClks = 0;
      if (SchedulerSequenceSteps.PreRangeOn) {
        /* Retrieve PRE-RANGE VCSEL Period */
        uint8_t CurrentVCSELPulsePeriodPClk;
        Status = VL53L0X_GetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, &CurrentVCSELPulsePeriodPClk);
        /* Retrieve PRE-RANGE Timeout in Macro periods
         * (MCLKS) */
        if (Status == VL53L0X_ERROR_NONE) {
          uint16_t PreRangeEncodedTimeOut = 0;
          Status = VL53L0X_RdWord(Dev, VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &PreRangeEncodedTimeOut);
          PreRangeTimeOutMClks = VL53L0X_decode_timeout(PreRangeEncodedTimeOut);//BUG: ignores error in fetch
        }
      }
      uint8_t CurrentVCSELPulsePeriodPClk;
      if (Status == VL53L0X_ERROR_NONE) {
        /* Retrieve FINAL-RANGE VCSEL Period */
        Status = VL53L0X_GetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, &CurrentVCSELPulsePeriodPClk);
      }
      /* Retrieve FINAL-RANGE Timeout in Macro periods (MCLKS) */

      uint16_t FinalRangeTimeOutMClks = 0;
      if (Status == VL53L0X_ERROR_NONE) {
        uint16_t FinalRangeEncodedTimeOut;
        Status = VL53L0X_RdWord(Dev, VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &FinalRangeEncodedTimeOut);
        FinalRangeTimeOutMClks = VL53L0X_decode_timeout(FinalRangeEncodedTimeOut);
      }
      FinalRangeTimeOutMClks -= PreRangeTimeOutMClks;//BUG: uses value even if fetch fails
      TimeoutMicroSeconds = VL53L0X_calc_timeout_us(Dev, FinalRangeTimeOutMClks, CurrentVCSELPulsePeriodPClk);
    }
      break;
    default:
      break;
  }//end switch

  *pTimeOutMicroSecs = TimeoutMicroSeconds;

  return Status;
} // get_sequence_step_timeout

VL53L0X_Error set_sequence_step_timeout(VL53L0X_DEV Dev, VL53L0X_SequenceStepId SequenceStepId, uint32_t TimeOutMicroSecs) {
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  uint8_t MsrcEncodedTimeOut;
  uint16_t PreRangeEncodedTimeOut;
  uint16_t PreRangeTimeOutMClks;
  uint16_t MsrcRangeTimeOutMClks;
  uint16_t FinalRangeTimeOutMClks;
  uint16_t FinalRangeEncodedTimeOut;
  VL53L0X_SchedulerSequenceSteps_t SchedulerSequenceSteps;

  uint8_t CurrentVCSELPulsePeriodPClk;
  if ((SequenceStepId == VL53L0X_SEQUENCESTEP_TCC) ||
      (SequenceStepId == VL53L0X_SEQUENCESTEP_DSS) ||
      (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)) {

    Status = VL53L0X_GetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, &CurrentVCSELPulsePeriodPClk);

    if (Status == VL53L0X_ERROR_NONE) {
      MsrcRangeTimeOutMClks = VL53L0X_calc_timeout_mclks(Dev, TimeOutMicroSecs, (uint8_t) CurrentVCSELPulsePeriodPClk);

      if (MsrcRangeTimeOutMClks > 256) {
        MsrcEncodedTimeOut = 255;
      } else {
        MsrcEncodedTimeOut = (uint8_t) MsrcRangeTimeOutMClks - 1;
      }

      VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, LastEncodedTimeout, MsrcEncodedTimeOut);
    }

    if (Status == VL53L0X_ERROR_NONE) {
      Status = VL53L0X_WrByte(Dev, VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP, MsrcEncodedTimeOut);
    }
  } else {

    if (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE) {

      if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_GetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, &CurrentVCSELPulsePeriodPClk);
        PreRangeTimeOutMClks = VL53L0X_calc_timeout_mclks(Dev, TimeOutMicroSecs, (uint8_t) CurrentVCSELPulsePeriodPClk);
        PreRangeEncodedTimeOut = VL53L0X_encode_timeout(PreRangeTimeOutMClks);

        VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, LastEncodedTimeout, PreRangeEncodedTimeOut);
      }

      if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_WrWord(Dev, VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, PreRangeEncodedTimeOut);
      }

      if (Status == VL53L0X_ERROR_NONE) {
        VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, PreRangeTimeoutMicroSecs, TimeOutMicroSecs);
      }
    } else if (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE) {

      /* For the final range timeout, the pre-range timeout
       * must be added. To do this both final and pre-range
       * timeouts must be expressed in macro periods MClks
       * because they have different vcsel periods.
       */

      VL53L0X_GetSequenceStepEnables(Dev, &SchedulerSequenceSteps);
      PreRangeTimeOutMClks = 0;
      if (SchedulerSequenceSteps.PreRangeOn) {

        /* Retrieve PRE-RANGE VCSEL Period */
        Status = VL53L0X_GetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, &CurrentVCSELPulsePeriodPClk);

        /* Retrieve PRE-RANGE Timeout in Macro periods
         * (MCLKS) */
        if (Status == VL53L0X_ERROR_NONE) {
          Status = VL53L0X_RdWord(Dev, 0x51, &PreRangeEncodedTimeOut);
          PreRangeTimeOutMClks = VL53L0X_decode_timeout(PreRangeEncodedTimeOut);
        }
      }

      /* Calculate FINAL RANGE Timeout in Macro Periods
       * (MCLKS) and add PRE-RANGE value
       */
      if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_GetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, &CurrentVCSELPulsePeriodPClk);
      }
      if (Status == VL53L0X_ERROR_NONE) {
        FinalRangeTimeOutMClks = VL53L0X_calc_timeout_mclks(Dev, TimeOutMicroSecs, (uint8_t) CurrentVCSELPulsePeriodPClk);
        FinalRangeTimeOutMClks += PreRangeTimeOutMClks;
        FinalRangeEncodedTimeOut = VL53L0X_encode_timeout(FinalRangeTimeOutMClks);

        if (Status == VL53L0X_ERROR_NONE) {
          Status = VL53L0X_WrWord(Dev, 0x71, FinalRangeEncodedTimeOut);
        }

        if (Status == VL53L0X_ERROR_NONE) {
          VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, FinalRangeTimeoutMicroSecs, TimeOutMicroSecs);
        }
      }
    } else {
      Status = VL53L0X_ERROR_INVALID_PARAMS;
    }
  }
  return Status;
} // set_sequence_step_timeout

static VL53L0X_Error setValidPhase(VL53L0X_DEV Dev, uint8_t high, uint8_t low) {
  VL53L0X_WrByte(Dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, high);//NB: mimicking old bug of ignoring status of first write
  return VL53L0X_WrByte(Dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, low);
}

static VL53L0X_Error setPhasecalLimit(VL53L0X_DEV Dev, uint8_t value) {
  VL53L0X_Error Error = VL53L0X_WrByte(Dev, 0xff, 0x01);
  Error |= VL53L0X_WrByte(Dev, VL53L0X_REG_ALGO_PHASECAL_LIM, value);
  Error |= VL53L0X_WrByte(Dev, 0xff, 0x00);
  return Error;
}

VL53L0X_Error VL53L0X_set_vcsel_pulse_period(VL53L0X_DEV Dev, VL53L0X_VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriodPCLK) {

  uint8_t vcsel_period_reg;
  uint8_t MinPreVcselPeriodPCLK = 12;
  uint8_t MaxPreVcselPeriodPCLK = 18;
  uint8_t MinFinalVcselPeriodPCLK = 8;
  uint8_t MaxFinalVcselPeriodPCLK = 14;
  uint32_t MeasurementTimingBudgetMicroSeconds;
  uint32_t FinalRangeTimeoutMicroSeconds;
  uint32_t PreRangeTimeoutMicroSeconds;
  uint32_t MsrcTimeoutMicroSeconds;
  uint8_t PhaseCalInt = 0;

  /* Check if valid clock period requested */

  if ((VCSELPulsePeriodPCLK % 2) != 0) {
    /* Value must be an even number */
    return VL53L0X_ERROR_INVALID_PARAMS;
  } else if (VcselPeriodType == VL53L0X_VCSEL_PERIOD_PRE_RANGE && (VCSELPulsePeriodPCLK < MinPreVcselPeriodPCLK || VCSELPulsePeriodPCLK > MaxPreVcselPeriodPCLK)) {
    return VL53L0X_ERROR_INVALID_PARAMS;
  } else if (VcselPeriodType == VL53L0X_VCSEL_PERIOD_FINAL_RANGE && (VCSELPulsePeriodPCLK < MinFinalVcselPeriodPCLK || VCSELPulsePeriodPCLK > MaxFinalVcselPeriodPCLK)) {
    return VL53L0X_ERROR_INVALID_PARAMS;
  }
  VL53L0X_Error Error = VL53L0X_ERROR_NONE;
  //BUG: the code below repeatedly ignores errors of writing to ..._PHASE_HIGH.
  //BUG: the code below repeatedly ignores errors in I2C operations with the result that incoherent settings might be made
  //... if you are going to ignore errors then make some tables and iterate over them, that takes less code.
  /* Apply specific settings for the requested clock period */
  switch (VcselPeriodType) {
    case VL53L0X_VCSEL_PERIOD_PRE_RANGE: {
      /* Set phase check limits */
      switch (VCSELPulsePeriodPCLK) {
        case 12:
          Error = setValidPhase(Dev, 0x18, 0x08);
          break;
        case 14:
          Error = setValidPhase(Dev, 0x30, 0x08);
          break;
        case 16:
          Error = setValidPhase(Dev, 0x40, 0x08);
          break;
        case 18:
          Error = setValidPhase(Dev, 0x50, 0x08);
          break;
      }
      break;
      case VL53L0X_VCSEL_PERIOD_FINAL_RANGE:
        switch (VCSELPulsePeriodPCLK) {
          case 8:
            Error = setValidPhase(Dev, 0x10, 0x08);
            Error |= VL53L0X_WrByte(Dev, VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
            Error |= VL53L0X_WrByte(Dev, VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
            Error |= setPhasecalLimit(Dev, 0x30);
            break;
          case 10:
            Error = setValidPhase(Dev, 0x28, 0x08);
            Error |= VL53L0X_WrByte(Dev, VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            Error |= VL53L0X_WrByte(Dev, VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
            Error |= setPhasecalLimit(Dev, 0x20);
            break;
          case 12:
            Error = setValidPhase(Dev, 0x38, 0x08);
            Error |= VL53L0X_WrByte(Dev, VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            Error |= VL53L0X_WrByte(Dev, VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
            Error |= setPhasecalLimit(Dev, 0x20);
            break;
          case 14:
            Error = setValidPhase(Dev, 0x048, 0x08);
            Error |= VL53L0X_WrByte(Dev, VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            Error |= VL53L0X_WrByte(Dev, VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
            Error |= setPhasecalLimit(Dev, 0x20);
            break;
            break;
          default:
            break;
        }
    }
  }
  /* Re-calculate and apply timeouts, in macro periods */

  if (!Error) {
    vcsel_period_reg = VL53L0X_encode_vcsel_period((uint8_t) VCSELPulsePeriodPCLK);

    /* When the VCSEL period for the pre or final range is changed,
     * the corresponding timeout must be read from the device using
     * the current VCSEL period, then the new VCSEL period can be
     * applied. The timeout then must be written back to the device
     * using the new VCSEL period.
     *
     * For the MSRC timeout, the same applies - this timeout being
     * dependant on the pre-range vcsel period.
     */
    switch (VcselPeriodType) {
      case VL53L0X_VCSEL_PERIOD_PRE_RANGE:
        Error = get_sequence_step_timeout(Dev, VL53L0X_SEQUENCESTEP_PRE_RANGE, &PreRangeTimeoutMicroSeconds);

        if (Error == VL53L0X_ERROR_NONE) {
          Error = get_sequence_step_timeout(Dev, VL53L0X_SEQUENCESTEP_MSRC, &MsrcTimeoutMicroSeconds);
        }

        if (Error == VL53L0X_ERROR_NONE) {
          Error = VL53L0X_WrByte(Dev, VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
        }

        if (Error == VL53L0X_ERROR_NONE) {
          Error = set_sequence_step_timeout(Dev, VL53L0X_SEQUENCESTEP_PRE_RANGE, PreRangeTimeoutMicroSeconds);
        }

        if (Error == VL53L0X_ERROR_NONE) {
          Error = set_sequence_step_timeout(Dev, VL53L0X_SEQUENCESTEP_MSRC, MsrcTimeoutMicroSeconds);
        }

        VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, PreRangeVcselPulsePeriod, VCSELPulsePeriodPCLK);
        break;
      case VL53L0X_VCSEL_PERIOD_FINAL_RANGE:
        Error = get_sequence_step_timeout(Dev, VL53L0X_SEQUENCESTEP_FINAL_RANGE, &FinalRangeTimeoutMicroSeconds);

        if (Error == VL53L0X_ERROR_NONE) {
          Error = VL53L0X_WrByte(Dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
        }
        if (Error == VL53L0X_ERROR_NONE) {
          Error = set_sequence_step_timeout(Dev, VL53L0X_SEQUENCESTEP_FINAL_RANGE, FinalRangeTimeoutMicroSeconds);
        }
        VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, FinalRangeVcselPulsePeriod, VCSELPulsePeriodPCLK);
        break;
      default:
        return VL53L0X_ERROR_INVALID_PARAMS;
    } // switch
  }

  /* Finally, the timing budget must be re-applied */
  if (Error == VL53L0X_ERROR_NONE) {
    VL53L0X_GETPARAMETERFIELD(Dev, MeasurementTimingBudgetMicroSeconds, MeasurementTimingBudgetMicroSeconds);

    return VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, MeasurementTimingBudgetMicroSeconds);
  }

  /* Perform the phase calibration. This is needed after changing on
   * vcsel period.
   * get_data_enable = 0, restore_config = 1 */
  return VL53L0X_perform_phase_calibration(Dev, &PhaseCalInt, 0, 1);
} // VL53L0X_set_vcsel_pulse_period

Erroneous<uint8_t> Api::get_vcsel_pulse_period( VcselPeriod VcselPeriodType) {
  Erroneous<uint8_t> vcsel_period_reg;

  switch (VcselPeriodType) {
    case VL53L0X_VCSEL_PERIOD_PRE_RANGE:
      Status = VL53L0X_RdByte(Dev, VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, &vcsel_period_reg);
      break;
    case VL53L0X_VCSEL_PERIOD_FINAL_RANGE:
      Status = VL53L0X_RdByte(Dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, &vcsel_period_reg);
      break;
    default:
      Status = VL53L0X_ERROR_INVALID_PARAMS;
  } // switch

  if (Status == VL53L0X_ERROR_NONE) {
    *pVCSELPulsePeriodPCLK = VL53L0X_decode_vcsel_period(vcsel_period_reg);
  }

  return Status;
} // VL53L0X_get_vcsel_pulse_period

VL53L0X_Error VL53L0X_set_measurement_timing_budget_micro_seconds(VL53L0X_DEV Dev, uint32_t MeasurementTimingBudgetMicroSeconds) {
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  uint32_t FinalRangeTimingBudgetMicroSeconds;
  VL53L0X_SchedulerSequenceSteps_t SchedulerSequenceSteps;
  uint32_t MsrcDccTccTimeoutMicroSeconds = 2000;
  uint32_t StartOverheadMicroSeconds = 1320;
  uint32_t EndOverheadMicroSeconds = 960;
  uint32_t MsrcOverheadMicroSeconds = 660;
  uint32_t TccOverheadMicroSeconds = 590;
  uint32_t DssOverheadMicroSeconds = 690;
  uint32_t PreRangeOverheadMicroSeconds = 660;
  uint32_t FinalRangeOverheadMicroSeconds = 550;
  uint32_t PreRangeTimeoutMicroSeconds = 0;
  uint32_t cMinTimingBudgetMicroSeconds = 20000;
  uint32_t SubTimeout = 0;

  LOG_FUNCTION_START("");

  if (MeasurementTimingBudgetMicroSeconds < cMinTimingBudgetMicroSeconds) {
    Status = VL53L0X_ERROR_INVALID_PARAMS;
    return Status;
  }

  FinalRangeTimingBudgetMicroSeconds = MeasurementTimingBudgetMicroSeconds - (StartOverheadMicroSeconds + EndOverheadMicroSeconds);

  Status = VL53L0X_GetSequenceStepEnables(Dev, &SchedulerSequenceSteps);

  if (Status == VL53L0X_ERROR_NONE && (SchedulerSequenceSteps.TccOn || SchedulerSequenceSteps.MsrcOn || SchedulerSequenceSteps.DssOn)) {

    /* TCC, MSRC and DSS all share the same timeout */
    Status = get_sequence_step_timeout(Dev, VL53L0X_SEQUENCESTEP_MSRC, &MsrcDccTccTimeoutMicroSeconds);

    /* Subtract the TCC, MSRC and DSS timeouts if they are
     * enabled. */

    if (Status != VL53L0X_ERROR_NONE) {
      return Status;
    }

    /* TCC */
    if (SchedulerSequenceSteps.TccOn) {
      SubTimeout = MsrcDccTccTimeoutMicroSeconds + TccOverheadMicroSeconds;

      if (SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
        FinalRangeTimingBudgetMicroSeconds -= SubTimeout;
      } else {
        /* Requested timeout too big. */
        Status = VL53L0X_ERROR_INVALID_PARAMS;
      }
    }

    if (Status != VL53L0X_ERROR_NONE) {

      return Status;
    }

    /* DSS */
    if (SchedulerSequenceSteps.DssOn) {
      SubTimeout = 2 * (MsrcDccTccTimeoutMicroSeconds + DssOverheadMicroSeconds);

      if (SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
        FinalRangeTimingBudgetMicroSeconds -= SubTimeout;
      } else {
        /* Requested timeout too big. */
        Status = VL53L0X_ERROR_INVALID_PARAMS;
      }
    } else if (SchedulerSequenceSteps.MsrcOn) {
      /* MSRC */
      SubTimeout = MsrcDccTccTimeoutMicroSeconds + MsrcOverheadMicroSeconds;

      if (SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
        FinalRangeTimingBudgetMicroSeconds -= SubTimeout;
      } else {
        /* Requested timeout too big. */
        Status = VL53L0X_ERROR_INVALID_PARAMS;
      }
    }
  }

  if (Status != VL53L0X_ERROR_NONE) {

    return Status;
  }

  if (SchedulerSequenceSteps.PreRangeOn) {
    /* Subtract the Pre-range timeout if enabled. */
    Status = get_sequence_step_timeout(Dev, VL53L0X_SEQUENCESTEP_PRE_RANGE, &PreRangeTimeoutMicroSeconds);
    SubTimeout = PreRangeTimeoutMicroSeconds + PreRangeOverheadMicroSeconds;

    if (SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
      FinalRangeTimingBudgetMicroSeconds -= SubTimeout;
    } else {
      /* Requested timeout too big. */
      Status = VL53L0X_ERROR_INVALID_PARAMS;
    }
  }

  if (Status == VL53L0X_ERROR_NONE && SchedulerSequenceSteps.FinalRangeOn) {
    FinalRangeTimingBudgetMicroSeconds -= FinalRangeOverheadMicroSeconds;

    /* Final Range Timeout
     * Note that the final range timeout is determined by the timing
     * budget and the sum of all other timeouts within the sequence.
     * If there is no room for the final range timeout, then an error
     * will be set. Otherwise the remaining time will be applied to
     * the final range.
     */
    Status = set_sequence_step_timeout(Dev, VL53L0X_SEQUENCESTEP_FINAL_RANGE, FinalRangeTimingBudgetMicroSeconds);

    VL53L0X_SETPARAMETERFIELD(Dev, MeasurementTimingBudgetMicroSeconds, MeasurementTimingBudgetMicroSeconds);
  }



  return Status;
} // VL53L0X_set_measurement_timing_budget_micro_seconds

VL53L0X_Error VL53L0X_get_measurement_timing_budget_micro_seconds(VL53L0X_DEV Dev, uint32_t *pMeasurementTimingBudgetMicroSeconds) {
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  VL53L0X_SchedulerSequenceSteps_t SchedulerSequenceSteps;
  uint32_t FinalRangeTimeoutMicroSeconds;
  uint32_t MsrcDccTccTimeoutMicroSeconds = 2000;
  uint32_t StartOverheadMicroSeconds = 1910;
  uint32_t EndOverheadMicroSeconds = 960;
  uint32_t MsrcOverheadMicroSeconds = 660;
  uint32_t TccOverheadMicroSeconds = 590;
  uint32_t DssOverheadMicroSeconds = 690;
  uint32_t PreRangeOverheadMicroSeconds = 660;
  uint32_t FinalRangeOverheadMicroSeconds = 550;
  uint32_t PreRangeTimeoutMicroSeconds = 0;

  LOG_FUNCTION_START("");

  /* Start and end overhead times always present */
  *pMeasurementTimingBudgetMicroSeconds = StartOverheadMicroSeconds + EndOverheadMicroSeconds;

  Status = VL53L0X_GetSequenceStepEnables(Dev, &SchedulerSequenceSteps);

  if (Status != VL53L0X_ERROR_NONE) {

    return Status;
  }

  if (SchedulerSequenceSteps.TccOn || SchedulerSequenceSteps.MsrcOn || SchedulerSequenceSteps.DssOn) {

    Status = get_sequence_step_timeout(Dev, VL53L0X_SEQUENCESTEP_MSRC, &MsrcDccTccTimeoutMicroSeconds);

    if (Status == VL53L0X_ERROR_NONE) {
      if (SchedulerSequenceSteps.TccOn) {
        *pMeasurementTimingBudgetMicroSeconds += MsrcDccTccTimeoutMicroSeconds + TccOverheadMicroSeconds;
      }

      if (SchedulerSequenceSteps.DssOn) {
        *pMeasurementTimingBudgetMicroSeconds += 2 * (MsrcDccTccTimeoutMicroSeconds + DssOverheadMicroSeconds);
      } else if (SchedulerSequenceSteps.MsrcOn) {
        *pMeasurementTimingBudgetMicroSeconds += MsrcDccTccTimeoutMicroSeconds + MsrcOverheadMicroSeconds;
      }
    }
  }

  if (Status == VL53L0X_ERROR_NONE) {
    if (SchedulerSequenceSteps.PreRangeOn) {
      Status = get_sequence_step_timeout(Dev, VL53L0X_SEQUENCESTEP_PRE_RANGE, &PreRangeTimeoutMicroSeconds);
      *pMeasurementTimingBudgetMicroSeconds += PreRangeTimeoutMicroSeconds + PreRangeOverheadMicroSeconds;
    }
  }

  if (Status == VL53L0X_ERROR_NONE) {
    if (SchedulerSequenceSteps.FinalRangeOn) {
      Status = get_sequence_step_timeout(Dev, VL53L0X_SEQUENCESTEP_FINAL_RANGE, &FinalRangeTimeoutMicroSeconds);
      *pMeasurementTimingBudgetMicroSeconds += (FinalRangeTimeoutMicroSeconds + FinalRangeOverheadMicroSeconds);
    }
  }

  if (Status == VL53L0X_ERROR_NONE) {
    VL53L0X_SETPARAMETERFIELD(Dev, MeasurementTimingBudgetMicroSeconds, *pMeasurementTimingBudgetMicroSeconds);
  }


  return Status;
} // VL53L0X_get_measurement_timing_budget_micro_seconds


static uint16_t unpack(const uint8_t *pTuningSettingBuffer, int &Index) {
  uint8_t msb = pTuningSettingBuffer[Index++];
  uint8_t lsb = pTuningSettingBuffer[Index++];
  //the above may NOT be inlined as that would get into 'undefined behavior' territory re 'sequence points'.
  return VL53L0X_MAKEUINT16(lsb, msb);
}

VL53L0X_Error VL53L0X_load_tuning_settings(VL53L0X_DEV Dev, const uint8_t *pTuningSettingBuffer) {

  LOG_FUNCTION_START("");
  int Index = 0;//ick: only value over using the pTuning..Buffer as a pointer might be for debug display.
  VL53L0X_Error Error = VL53L0X_ERROR_NONE;
  while ((pTuningSettingBuffer[Index] != 0) && !Error) {
    uint8_t NumberOfWrites = pTuningSettingBuffer[Index++];
    if (NumberOfWrites == 0xFF) {
      /* internal parameters */
      switch (pTuningSettingBuffer[Index++]) {
        case 0: /* uint16_t SigmaEstRefArray -> 2 bytes */
          PALDevDataSet(Dev, SigmaEstRefArray, unpack(pTuningSettingBuffer, Index));
          break;
        case 1: /* uint16_t SigmaEstEffPulseWidth -> 2 bytes */
          PALDevDataSet(Dev, SigmaEstEffPulseWidth, unpack(pTuningSettingBuffer, Index));
          break;
        case 2: /* uint16_t SigmaEstEffAmbWidth -> 2 bytes */
          PALDevDataSet(Dev, SigmaEstEffAmbWidth, unpack(pTuningSettingBuffer, Index));
          break;
        case 3: /* uint16_t targetRefRate -> 2 bytes */
          PALDevDataSet(Dev, targetRefRate, unpack(pTuningSettingBuffer, Index));
          break;
        default: /* invalid parameter */
          Error = VL53L0X_ERROR_INVALID_PARAMS;
      } // switch
    } else if (NumberOfWrites <= 4) {
      uint8_t localBuffer[4]; /* max */
      uint8_t Address = pTuningSettingBuffer[Index++];
      for (unsigned i = 0; i < NumberOfWrites; i++) {
        localBuffer[i] = pTuningSettingBuffer[Index++];
      }
      Error = VL53L0X_WriteMulti(Dev, Address, localBuffer, NumberOfWrites);
    } else {
      Error = VL53L0X_ERROR_INVALID_PARAMS;
    }
  }

  return Error;
} // VL53L0X_load_tuning_settings

VL53L0X_Error VL53L0X_get_total_xtalk_rate(VL53L0X_DEV Dev, VL53L0X_RangingMeasurementData_t *pRangingMeasurementData, FixPoint1616_t *ptotal_xtalk_rate_mcps) {
  *ptotal_xtalk_rate_mcps = 0;//nb: gets set to zero when there are communications errors trying to get the value.
  uint8_t xtalkCompEnable;
  VL53L0X_Error Error = VL53L0X_GetXTalkCompensationEnable(Dev, &xtalkCompEnable);
  ERROR_OUT;
  if (xtalkCompEnable) {
    FixPoint1616_t xtalkPerSpadMegaCps;
    VL53L0X_GETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps, xtalkPerSpadMegaCps);

    /* FixPoint1616 * FixPoint 8:8 = FixPoint0824 */
    FixPoint1616_t totalXtalkMegaCps = pRangingMeasurementData->EffectiveSpadRtnCount * xtalkPerSpadMegaCps;

    /* FixPoint0824 >> 8 = FixPoint1616 */
    *ptotal_xtalk_rate_mcps = (totalXtalkMegaCps + 0x80) >> 8;
  }
  return VL53L0X_ERROR_NONE;
} // VL53L0X_get_total_xtalk_rate

VL53L0X_Error VL53L0X_get_total_signal_rate(VL53L0X_DEV Dev, VL53L0X_RangingMeasurementData_t *pRangingMeasurementData, FixPoint1616_t *ptotal_signal_rate_mcps) {
  LOG_FUNCTION_START("");
  *ptotal_signal_rate_mcps = pRangingMeasurementData->SignalRateRtnMegaCps;
  FixPoint1616_t totalXtalkMegaCps;
  VL53L0X_Error Error = VL53L0X_get_total_xtalk_rate(Dev, pRangingMeasurementData, &totalXtalkMegaCps);
  ERROR_OUT;
  *ptotal_signal_rate_mcps += totalXtalkMegaCps;
  return VL53L0X_ERROR_NONE;
} // VL53L0X_get_total_signal_rate

VL53L0X_Error VL53L0X_calc_dmax(VL53L0X_DEV Dev, FixPoint1616_t totalSignalRate_mcps, FixPoint1616_t totalCorrSignalRate_mcps, FixPoint1616_t pwMult, uint32_t sigmaEstimateP1, FixPoint1616_t sigmaEstimateP2, uint32_t peakVcselDuration_us, uint32_t *pdmax_mm) {
  const uint32_t cSigmaLimit = 18;
  const FixPoint1616_t cSignalLimit = 0x4000;     /* 0.25 */
  const FixPoint1616_t cSigmaEstRef = 0x00000042; /* 0.001 */
  const uint32_t cAmbEffWidthSigmaEst_ns = 6;
  const uint32_t cAmbEffWidthDMax_ns = 7;

  LOG_FUNCTION_START("");

  uint32_t dmaxCalRange_mm = PALDevDataGet(Dev, DmaxCalRangeMilliMeter);
  FixPoint1616_t dmaxCalSignalRateRtn_mcps = PALDevDataGet(Dev, DmaxCalSignalRateRtnMegaCps);

  /* uint32 * FixPoint1616 = FixPoint1616 */

  FixPoint1616_t SignalAt0mm = dmaxCalRange_mm * dmaxCalSignalRateRtn_mcps;

  /* FixPoint1616 >> 8 = FixPoint2408 */
  SignalAt0mm = (SignalAt0mm + 0x80) >> 8;
  SignalAt0mm *= dmaxCalRange_mm;
  FixPoint1616_t minSignalNeeded_p1 = 0;
  if (totalCorrSignalRate_mcps > 0) {

    /* Shift by 10 bits to increase resolution prior to the
     * division */
    uint32_t signalRateTemp_mcps = totalSignalRate_mcps << 10;

    /* Add rounding value prior to division */
    minSignalNeeded_p1 = signalRateTemp_mcps + (totalCorrSignalRate_mcps / 2);

    /* FixPoint0626/FixPoint1616 = FixPoint2210 */
    minSignalNeeded_p1 /= totalCorrSignalRate_mcps;

    /* Apply a factored version of the speed of light.
     *  Correction to be applied at the end */
    minSignalNeeded_p1 *= 3;

    /* FixPoint2210 * FixPoint2210 = FixPoint1220 */
    minSignalNeeded_p1 *= minSignalNeeded_p1;

    /* FixPoint1220 >> 16 = FixPoint2804 */
    minSignalNeeded_p1 = (minSignalNeeded_p1 + 0x8000) >> 16;
  }

  FixPoint1616_t minSignalNeeded_p2 = pwMult * sigmaEstimateP1;

  /* FixPoint1616 >> 16 =	 uint32 */
  minSignalNeeded_p2 = (minSignalNeeded_p2 + 0x8000) >> 16;

  /* uint32 * uint32	=  uint32 */
  minSignalNeeded_p2 *= minSignalNeeded_p2;

  /* Check sigmaEstimateP2
   * If this value is too high there is not enough signal rate
   * to calculate dmax value so set a suitable value to ensure
   * a very small dmax.
   */

  FixPoint1616_t sigmaEstP2Tmp = (sigmaEstimateP2 + 0x8000) >> 16;
  sigmaEstP2Tmp = (sigmaEstP2Tmp + cAmbEffWidthSigmaEst_ns / 2) / cAmbEffWidthSigmaEst_ns;
  sigmaEstP2Tmp *= cAmbEffWidthDMax_ns;

  FixPoint1616_t minSignalNeeded_p3;
  if (sigmaEstP2Tmp > 0xffff) {
    minSignalNeeded_p3 = 0xfff00000;
  } else {
    /* DMAX uses a different ambient width from sigma, so apply correction.
     * Perform division before multiplication to prevent overflow.
     */
    sigmaEstimateP2 = (sigmaEstimateP2 + cAmbEffWidthSigmaEst_ns / 2) / cAmbEffWidthSigmaEst_ns;
    sigmaEstimateP2 *= cAmbEffWidthDMax_ns;

    /* FixPoint1616 >> 16 = uint32 */
    minSignalNeeded_p3 = (sigmaEstimateP2 + 0x8000) >> 16;
    minSignalNeeded_p3 *= minSignalNeeded_p3;
  }

  /* FixPoint1814 / uint32 = FixPoint1814 */
  FixPoint1616_t sigmaLimitTmp = ((cSigmaLimit << 14) + 500) / 1000;
  /* FixPoint1814 * FixPoint1814 = FixPoint3628 := FixPoint0428 */
  sigmaLimitTmp *= sigmaLimitTmp;

  /* FixPoint1616 * FixPoint1616 = FixPoint3232 */
  FixPoint1616_t sigmaEstSqTmp = cSigmaEstRef * cSigmaEstRef;

  /* FixPoint3232 >> 4 = FixPoint0428 */
  sigmaEstSqTmp = (sigmaEstSqTmp + 0x08) >> 4;

  /* FixPoint0428 - FixPoint0428	= FixPoint0428 */
  sigmaLimitTmp -= sigmaEstSqTmp;

  /* uint32_t * FixPoint0428 = FixPoint0428 */
  FixPoint1616_t minSignalNeeded_p4 = 4 * 12 * sigmaLimitTmp;

  /* FixPoint0428 >> 14 = FixPoint1814 */
  minSignalNeeded_p4 = (minSignalNeeded_p4 + 0x2000) >> 14;

  /* uint32 + uint32 = uint32 */
  FixPoint1616_t minSignalNeeded = (minSignalNeeded_p2 + minSignalNeeded_p3);

  /* uint32 / uint32 = uint32 */
  minSignalNeeded += (peakVcselDuration_us / 2);
  minSignalNeeded /= peakVcselDuration_us;

  /* uint32 << 14 = FixPoint1814 */
  minSignalNeeded <<= 14;

  /* FixPoint1814 / FixPoint1814 = uint32 */
  minSignalNeeded += (minSignalNeeded_p4 / 2);
  minSignalNeeded /= minSignalNeeded_p4;

  /* FixPoint3200 * FixPoint2804 := FixPoint2804*/
  minSignalNeeded *= minSignalNeeded_p1;

  /* Apply correction by dividing by 1000000.
   * This assumes 10E16 on the numerator of the equation
   * and 10E-22 on the denominator.
   * We do this because 32bit fix point calculation can't
   * handle the larger and smaller elements of this equation,
   * i.e. speed of light and pulse widths.
   */
  minSignalNeeded = (minSignalNeeded + 500) / 1000;
  minSignalNeeded <<= 4;
  minSignalNeeded = (minSignalNeeded + 500) / 1000;//BUG: perhaps 980F screwed this up?

  /* FixPoint1616 >> 8 = FixPoint2408 */
  FixPoint1616_t signalLimitTmp = (cSignalLimit + 0x80) >> 8;

  /* FixPoint2408/FixPoint2408 = uint32 */
  FixPoint1616_t dmaxDarkTmp = (signalLimitTmp != 0) ? (SignalAt0mm + (signalLimitTmp / 2)) / signalLimitTmp : dmaxDarkTmp = 0;
  FixPoint1616_t dmaxDark = VL53L0X_isqrt(dmaxDarkTmp);

  /* FixPoint2408/FixPoint2408 = uint32 */
  FixPoint1616_t dmaxAmbient = (minSignalNeeded != 0) ? (SignalAt0mm + minSignalNeeded / 2) / minSignalNeeded : 0;
  dmaxAmbient = VL53L0X_isqrt(dmaxAmbient);

  *pdmax_mm = min(dmaxDark, dmaxAmbient);
  return VL53L0X_ERROR_NONE;
} // VL53L0X_calc_dmax

VL53L0X_Error VL53L0X_calc_sigma_estimate(VL53L0X_DEV Dev, VL53L0X_RangingMeasurementData_t *pRangingMeasurementData, FixPoint1616_t *pSigmaEstimate, uint32_t *pDmax_mm) {
  /* Expressed in 100ths of a ns, i.e. centi-ns */
  const uint32_t cPulseEffectiveWidth_centi_ns = 800;
  /* Expressed in 100ths of a ns, i.e. centi-ns */
  const uint32_t cAmbientEffectiveWidth_centi_ns = 600;
  const FixPoint1616_t cSigmaEstRef = 0x00000042; /* 0.001 */
  const uint32_t cVcselPulseWidth_ps = 4700;      /* pico secs */
  const FixPoint1616_t cSigmaEstMax = 0x028F87AE;
  const FixPoint1616_t cSigmaEstRtnMax = 0xF000;
  const FixPoint1616_t cAmbToSignalRatioMax = 0xF0000000 / cAmbientEffectiveWidth_centi_ns;
  /* Time Of Flight per mm (6.6 pico secs) */
  const FixPoint1616_t cTOF_per_mm_ps = 0x0006999A;
  const uint32_t c16BitRoundingParam = 0x00008000;
  const FixPoint1616_t cMaxXTalk_kcps = 0x00320000;
  const uint32_t cPllPeriod_ps = 1655;

  uint32_t vcselTotalEventsRtn;
  uint32_t finalRangeTimeoutMicroSecs;
  uint32_t preRangeTimeoutMicroSecs;
  FixPoint1616_t sigmaEstimateP1;
  FixPoint1616_t sigmaEstimateP2;
  FixPoint1616_t sigmaEstimateP3;
  FixPoint1616_t deltaT_ps;
  FixPoint1616_t pwMult;
  FixPoint1616_t sigmaEstRtn;
  FixPoint1616_t sigmaEstimate;
  FixPoint1616_t xTalkCorrection;
  FixPoint1616_t ambientRate_kcps;
  FixPoint1616_t peakSignalRate_kcps;
  FixPoint1616_t xTalkCompRate_mcps;
  uint32_t xTalkCompRate_kcps;
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  FixPoint1616_t diff1_mcps;
  FixPoint1616_t diff2_mcps;
  FixPoint1616_t sqr1;
  FixPoint1616_t sqr2;
  FixPoint1616_t sqrSum;
  FixPoint1616_t sqrtResult_centi_ns;
  FixPoint1616_t sqrtResult;
  FixPoint1616_t totalSignalRate_mcps;
  FixPoint1616_t correctedSignalRate_mcps;
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

  LOG_FUNCTION_START("");

  VL53L0X_GETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps, xTalkCompRate_mcps);

  /*
   * We work in kcps rather than mcps as this helps keep within the
   * confines of the 32 Fix1616 type.
   */

  ambientRate_kcps = (pRangingMeasurementData->AmbientRateRtnMegaCps * 1000) >> 16;

  correctedSignalRate_mcps = pRangingMeasurementData->SignalRateRtnMegaCps;

  Status = VL53L0X_get_total_signal_rate(Dev, pRangingMeasurementData, &totalSignalRate_mcps);
  Status = VL53L0X_get_total_xtalk_rate(Dev, pRangingMeasurementData, &xTalkCompRate_mcps);

  /* Signal rate measurement provided by device is the
   * peak signal rate, not average.
   */
  peakSignalRate_kcps = (totalSignalRate_mcps * 1000);
  peakSignalRate_kcps = (peakSignalRate_kcps + 0x8000) >> 16;

  xTalkCompRate_kcps = xTalkCompRate_mcps * 1000;

  if (xTalkCompRate_kcps > cMaxXTalk_kcps) {
    xTalkCompRate_kcps = cMaxXTalk_kcps;
  }

  if (Status == VL53L0X_ERROR_NONE) {

    /* Calculate final range macro periods */
    finalRangeTimeoutMicroSecs = VL53L0X_GETDEVICESPECIFICPARAMETER(Dev, FinalRangeTimeoutMicroSecs);

    finalRangeVcselPCLKS = VL53L0X_GETDEVICESPECIFICPARAMETER(Dev, FinalRangeVcselPulsePeriod);

    finalRangeMacroPCLKS = VL53L0X_calc_timeout_mclks(Dev, finalRangeTimeoutMicroSecs, finalRangeVcselPCLKS);

    /* Calculate pre-range macro periods */
    preRangeTimeoutMicroSecs = VL53L0X_GETDEVICESPECIFICPARAMETER(Dev, PreRangeTimeoutMicroSecs);

    preRangeVcselPCLKS = VL53L0X_GETDEVICESPECIFICPARAMETER(Dev, PreRangeVcselPulsePeriod);

    preRangeMacroPCLKS = VL53L0X_calc_timeout_mclks(Dev, preRangeTimeoutMicroSecs, preRangeVcselPCLKS);

    vcselWidth = 3;
    if (finalRangeVcselPCLKS == 8) {
      vcselWidth = 2;
    }

    peakVcselDuration_us = vcselWidth * 2048 * (preRangeMacroPCLKS + finalRangeMacroPCLKS);
    peakVcselDuration_us = (peakVcselDuration_us + 500) / 1000;
    peakVcselDuration_us *= cPllPeriod_ps;
    peakVcselDuration_us = (peakVcselDuration_us + 500) / 1000;

    /* Fix1616 >> 8 = Fix2408 */
    totalSignalRate_mcps = (totalSignalRate_mcps + 0x80) >> 8;

    /* Fix2408 * uint32 = Fix2408 */
    vcselTotalEventsRtn = totalSignalRate_mcps * peakVcselDuration_us;

    /* Fix2408 >> 8 = uint32 */
    vcselTotalEventsRtn = (vcselTotalEventsRtn + 0x80) >> 8;

    /* Fix2408 << 8 = Fix1616 = */
    totalSignalRate_mcps <<= 8;
  }

  if (Status != VL53L0X_ERROR_NONE) {

    return Status;
  }

  if (peakSignalRate_kcps == 0) {
    *pSigmaEstimate = cSigmaEstMax;
    PALDevDataSet(Dev, SigmaEstimate, cSigmaEstMax);
    *pDmax_mm = 0;
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

    sigmaEstimateP1 = cPulseEffectiveWidth_centi_ns;

    /* ((FixPoint1616 << 16)* uint32)/uint32 = FixPoint1616 */
    sigmaEstimateP2 = (ambientRate_kcps << 16) / peakSignalRate_kcps;
    if (sigmaEstimateP2 > cAmbToSignalRatioMax) {
      /* Clip to prevent overflow. Will ensure safe
       * max result. */
      sigmaEstimateP2 = cAmbToSignalRatioMax;
    }
    sigmaEstimateP2 *= cAmbientEffectiveWidth_centi_ns;

    sigmaEstimateP3 = 2 * VL53L0X_isqrt(vcselTotalEventsRtn * 12);

    /* uint32 * FixPoint1616 = FixPoint1616 */
    deltaT_ps = pRangingMeasurementData->RangeMilliMeter * cTOF_per_mm_ps;

    /*
     * vcselRate - xtalkCompRate
     * (uint32 << 16) - FixPoint1616 = FixPoint1616.
     * Divide result by 1000 to convert to mcps.
     * 500 is added to ensure rounding when integer division
     * truncates.
     */
    diff1_mcps = (((peakSignalRate_kcps << 16) - xTalkCompRate_kcps) + 500) / 1000;

    /* vcselRate + xtalkCompRate */
    diff2_mcps = (((peakSignalRate_kcps << 16) + xTalkCompRate_kcps) + 500) / 1000;

    /* Shift by 8 bits to increase resolution prior to the
     * division */
    diff1_mcps <<= 8;

    /* FixPoint0824/FixPoint1616 = FixPoint2408 */
    xTalkCorrection = (FixPoint1616_t) abs((long long) (diff1_mcps / diff2_mcps));

    /* FixPoint2408 << 8 = FixPoint1616 */
    xTalkCorrection <<= 8;

    /* FixPoint1616/uint32 = FixPoint1616 */
    pwMult = deltaT_ps / cVcselPulseWidth_ps; /* smaller than 1.0f */

    /*
     * FixPoint1616 * FixPoint1616 = FixPoint3232, however both
     * values are small enough such that32 bits will not be
     * exceeded.
     */
    pwMult *= ((1 << 16) - xTalkCorrection);

    /* (FixPoint3232 >> 16) = FixPoint1616 */
    pwMult = (pwMult + c16BitRoundingParam) >> 16;

    /* FixPoint1616 + FixPoint1616 = FixPoint1616 */
    pwMult += (1 << 16);

    /*
     * At this point the value will be 1.xx, therefore if we square
     * the value this will exceed 32 bits. To address this perform
     * a single shift to the right before the multiplication.
     */
    pwMult >>= 1;
    /* FixPoint1715 * FixPoint1715 = FixPoint3430 */
    pwMult = pwMult * pwMult;

    /* (FixPoint3430 >> 14) = Fix1616 */
    pwMult >>= 14;

    /* FixPoint1616 * uint32 = FixPoint1616 */
    sqr1 = pwMult * sigmaEstimateP1;

    /* (FixPoint1616 >> 16) = FixPoint3200 */
    sqr1 = (sqr1 + 0x8000) >> 16;

    /* FixPoint3200 * FixPoint3200 = FixPoint6400 */
    sqr1 *= sqr1;

    sqr2 = sigmaEstimateP2;

    /* (FixPoint1616 >> 16) = FixPoint3200 */
    sqr2 = (sqr2 + 0x8000) >> 16;

    /* FixPoint3200 * FixPoint3200 = FixPoint6400 */
    sqr2 *= sqr2;

    /* FixPoint64000 + FixPoint6400 = FixPoint6400 */
    sqrSum = sqr1 + sqr2;

    /* SQRT(FixPoin6400) = FixPoint3200 */
    sqrtResult_centi_ns = VL53L0X_isqrt(sqrSum);

    /* (FixPoint3200 << 16) = FixPoint1616 */
    sqrtResult_centi_ns <<= 16;

    /*
     * Note that the Speed Of Light is expressed in um per 1E-10
     * seconds (2997) Therefore to get mm/ns we have to divide by
     * 10000
     */
    sigmaEstRtn = (((sqrtResult_centi_ns + 50) / 100) / sigmaEstimateP3);
    sigmaEstRtn *= VL53L0X_SPEED_OF_LIGHT_IN_AIR;

    /* Add 5000 before dividing by 10000 to ensure rounding. */
    sigmaEstRtn += 5000;
    sigmaEstRtn /= 10000;

    if (sigmaEstRtn > cSigmaEstRtnMax) {
      /* Clip to prevent overflow. Will ensure safe
       * max result. */
      sigmaEstRtn = cSigmaEstRtnMax;
    }

    /* FixPoint1616 * FixPoint1616 = FixPoint3232 */
    sqr1 = sigmaEstRtn * sigmaEstRtn;
    /* FixPoint1616 * FixPoint1616 = FixPoint3232 */
    sqr2 = cSigmaEstRef * cSigmaEstRef;

    /* sqrt(FixPoint3232) = FixPoint1616 */
    sqrtResult = VL53L0X_isqrt((sqr1 + sqr2));
    /*
     * Note that the Shift by 4 bits increases resolution prior to
     * the sqrt, therefore the result must be shifted by 2 bits to
     * the right to revert back to the FixPoint1616 format.
     */

    sigmaEstimate = 1000 * sqrtResult;

    if ((peakSignalRate_kcps < 1) || (vcselTotalEventsRtn < 1) ||
        (sigmaEstimate > cSigmaEstMax)) {
      sigmaEstimate = cSigmaEstMax;
    }

    *pSigmaEstimate = (uint32_t) (sigmaEstimate);
    PALDevDataSet(Dev, SigmaEstimate, *pSigmaEstimate);
    Status = VL53L0X_calc_dmax(Dev, totalSignalRate_mcps, correctedSignalRate_mcps, pwMult, sigmaEstimateP1, sigmaEstimateP2, peakVcselDuration_us, pDmax_mm);
  }


  return Status;
} // VL53L0X_calc_sigma_estimate

VL53L0X_Error VL53L0X_get_pal_range_status(VL53L0X_DEV Dev, uint8_t DeviceRangeStatus, FixPoint1616_t SignalRate, uint16_t EffectiveSpadRtnCount, VL53L0X_RangingMeasurementData_t *pRangingMeasurementData, uint8_t *pPalRangeStatus) {
  LOG_FUNCTION_START("");

  /*
   * VL53L0X has a good ranging when the value of the
   * DeviceRangeStatus = 11. This function will replace the value 0 with
   * the value 11 in the DeviceRangeStatus.
   * In addition, the SigmaEstimator is not included in the VL53L0X
   * DeviceRangeStatus, this will be added in the PalRangeStatus.
   */
  uint8_t DeviceRangeStatusInternal = ((DeviceRangeStatus & 0x78) >> 3);
  uint8_t NoneFlag = (DeviceRangeStatusInternal == 0 || DeviceRangeStatusInternal == 5 || DeviceRangeStatusInternal == 7 || DeviceRangeStatusInternal == 12 || DeviceRangeStatusInternal == 13 || DeviceRangeStatusInternal == 14 || DeviceRangeStatusInternal == 15);

  /* LastSignalRefMcps */
  VL53L0X_Error Status = VL53L0X_WrByte(Dev, 0xFF, 0x01);
  uint16_t tmpWord = 0;
  if (Status == VL53L0X_ERROR_NONE) {
    Status = VL53L0X_RdWord(Dev, VL53L0X_REG_RESULT_PEAK_SIGNAL_RATE_REF, &tmpWord);
  }
  if (Status == VL53L0X_ERROR_NONE) {
    Status = VL53L0X_WrByte(Dev, 0xFF, 0x00);
  }

  FixPoint1616_t LastSignalRefMcps = VL53L0X_FIXPOINT97TOFIXPOINT1616(tmpWord);
  PALDevDataSet(Dev, LastSignalRefMcps, LastSignalRefMcps);

  /*
   * Check if Sigma limit is enabled, if yes then do comparison with limit
   * value and put the result back into pPalRangeStatus.
   */
  uint8_t SigmaLimitCheckEnable = 0;
  if (Status == VL53L0X_ERROR_NONE) {
    Status = VL53L0X_GetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, &SigmaLimitCheckEnable);
  }

  uint8_t SigmaLimitflag = 0;
  if ((SigmaLimitCheckEnable != 0) && (Status == VL53L0X_ERROR_NONE)) {
    /*
     * compute the Sigma and check with limit
     */
    uint32_t Dmax_mm = 0;
    FixPoint1616_t SigmaEstimate;
    Status = VL53L0X_calc_sigma_estimate(Dev, pRangingMeasurementData, &SigmaEstimate, &Dmax_mm);
    if (Status == VL53L0X_ERROR_NONE) {
      pRangingMeasurementData->RangeDMaxMilliMeter = Dmax_mm;
    }

    if (Status == VL53L0X_ERROR_NONE) {
      FixPoint1616_t SigmaLimitValue;
      Status = VL53L0X_GetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, &SigmaLimitValue);
      if ((SigmaLimitValue > 0) && (SigmaEstimate > SigmaLimitValue)) {
        /* Limit Fail */
        SigmaLimitflag = 1;
      }
    }
  }

  /*
   * Check if Signal ref clip limit is enabled, if yes then do comparison
   * with limit value and put the result back into pPalRangeStatus.
   */
  uint8_t SignalRefClipLimitCheckEnable = 0;

  if (Status == VL53L0X_ERROR_NONE) {
    Status = VL53L0X_GetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP, &SignalRefClipLimitCheckEnable);
  }

  uint8_t SignalRefClipflag = 0;
  if ((SignalRefClipLimitCheckEnable != 0) && (Status == VL53L0X_ERROR_NONE)) {
    FixPoint1616_t SignalRefClipValue;
    Status = VL53L0X_GetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP, &SignalRefClipValue);
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
  if (Status == VL53L0X_ERROR_NONE) {
    Status = VL53L0X_GetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &RangeIgnoreThresholdLimitCheckEnable);
  }
  uint8_t RangeIgnoreThresholdflag = 0;

  if ((RangeIgnoreThresholdLimitCheckEnable != 0) && (Status == VL53L0X_ERROR_NONE)) {
    /* Compute the signal rate per spad */
    FixPoint1616_t SignalRatePerSpad = (EffectiveSpadRtnCount != 0) ? (FixPoint1616_t) ((256 * SignalRate) / EffectiveSpadRtnCount) : 0;

    FixPoint1616_t RangeIgnoreThresholdValue;
    Status = VL53L0X_GetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &RangeIgnoreThresholdValue);

    if ((RangeIgnoreThresholdValue > 0) && (SignalRatePerSpad < RangeIgnoreThresholdValue)) {
      /* Limit Fail add 2^6 to range status */
      RangeIgnoreThresholdflag = 1;
    }
  }

  if (Status == VL53L0X_ERROR_NONE) {
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
    pRangingMeasurementData->RangeDMaxMilliMeter = 0;
  }

  /* fill the Limit Check Status */
  uint8_t SignalRateFinalRangeLimitCheckEnable = 0;
  Status = VL53L0X_GetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, &SignalRateFinalRangeLimitCheckEnable);

  if (Status == VL53L0X_ERROR_NONE) {
    VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (SigmaLimitCheckEnable == 0) || (SigmaLimitflag == 1));
    VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (DeviceRangeStatusInternal == 4) || (SignalRateFinalRangeLimitCheckEnable == 0));
    VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus, VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP, (SignalRefClipLimitCheckEnable == 0) || (SignalRefClipflag == 1));
    VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (RangeIgnoreThresholdLimitCheckEnable == 0) || (RangeIgnoreThresholdflag == 1));
  }


  return Status;
} // VL53L0X_get_pal_range_status
