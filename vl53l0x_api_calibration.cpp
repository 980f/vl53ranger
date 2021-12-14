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

#include "vl53l0x_api_calibration.h"
#include "vl53l0x_api.h"
#include "vl53l0x_api_core.h"

#ifndef __KERNEL__

#include <stdlib.h>

#endif

#define LOG_FUNCTION_START(fmt, ...)  _LOG_FUNCTION_START(TRACE_MODULE_API, fmt, ## __VA_ARGS__)
#define LOG_FUNCTION_END(status, ...)  _LOG_FUNCTION_END(TRACE_MODULE_API, status, ## __VA_ARGS__)
#define LOG_FUNCTION_END_FMT(status, fmt, ...)  _LOG_FUNCTION_END_FMT(TRACE_MODULE_API, status, fmt, ## __VA_ARGS__)

//ick: use enums to constrain the range of an integer, not typeless symbols
#define REF_ARRAY_SPAD_0 0
#define REF_ARRAY_SPAD_5 5
#define REF_ARRAY_SPAD_10 10

uint32_t refArrayQuadrants[4] = {REF_ARRAY_SPAD_10, REF_ARRAY_SPAD_5, REF_ARRAY_SPAD_0, REF_ARRAY_SPAD_5};

//ick: these were often non-const locals when they are in actuality manifest constants.
static const uint32_t cSpadsPerByte = 8;

static const uint32_t cSpadArraySize = 6; //todo: apply to spad array allocations
static const uint32_t spadArraySize = 6;

static const uint32_t cMaxSpadCount = 44; //number of bits out of 48 (cSpadsPerByte * cSpadArraySize) allocated
static const uint32_t maxSpadCount = 44;

static const uint8_t startSelect = 180;// was 0xB4 but is not a bit pattern, rather it is a decimal number

static const uint32_t minimumSpadCount = 3;

VL53L0X_Error VL53L0X_perform_xtalk_calibration(VL53L0X_DEV Dev, FixPoint1616_t XTalkCalDistance, FixPoint1616_t *pXTalkCompensationRateMegaCps) {
  if (XTalkCalDistance <= 0) {
    return VL53L0X_ERROR_INVALID_PARAMS;
  }
  /* Disable the XTalk compensation */
  VL53L0X_Error Error = VL53L0X_SetXTalkCompensationEnable(Dev, 0);
  ERROR_OUT;

  /* Disable the RIT */
  Error = VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);
  ERROR_OUT;

  /* Perform 50 measurements and compute the averages */
  uint16_t sum_ranging = 0;
  uint16_t sum_spads = 0;
  FixPoint1616_t sum_signalRate = 0;
  FixPoint1616_t total_count = 0;
  for (uint8_t xtalk_meas = 0; xtalk_meas < 50; xtalk_meas++) {
    VL53L0X_RangingMeasurementData_t RangingMeasurementData;
    Error = VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingMeasurementData);

    if (Error != VL53L0X_ERROR_NONE) {
      return total_count == 0 ? VL53L0X_ERROR_RANGE_ERROR : Error;
    }

    /* The range is valid when RangeStatus = 0 */
    if (RangingMeasurementData.RangeStatus == 0) {
      sum_ranging += RangingMeasurementData.RangeMilliMeter;
      sum_signalRate += RangingMeasurementData.SignalRateRtnMegaCps;
      sum_spads += RangingMeasurementData.EffectiveSpadRtnCount / 256;
      ++total_count;
    }
  }
  /* no valid values found */
  if (total_count == 0) {
    return VL53L0X_ERROR_RANGE_ERROR;
  }

  /* FixPoint1616_t / uint16_t = FixPoint1616_t */
  FixPoint1616_t xTalkStoredMeanSignalRate = sum_signalRate / total_count;
  FixPoint1616_t xTalkStoredMeanRange = (FixPoint1616_t) ((uint32_t) (sum_ranging << 16) / total_count);
  FixPoint1616_t xTalkStoredMeanRtnSpads = (FixPoint1616_t) ((uint32_t) (sum_spads << 16) / total_count);

  /* Round Mean Spads to Whole Number.
   * Typically the calculated mean SPAD count is a whole number
   * or very close to a whole
   * number, therefore any truncation will not result in a
   * significant loss in accuracy.
   * Also, for a grey target at a typical distance of around
   * 400mm, around 220 SPADs will
   * be enabled, therefore, any truncation will result in a loss
   * of accuracy of less than
   * 0.5%.
   */
  uint32_t xTalkStoredMeanRtnSpadsAsInt = (xTalkStoredMeanRtnSpads + 0x8000) >> 16;

  /* Round Cal Distance to Whole Number.
   * Note that the cal distance is in mm, therefore no resolution
   * is lost.*/
  uint32_t xTalkCalDistanceAsInt = (XTalkCalDistance + 0x8000) >> 16;
  FixPoint1616_t XTalkCompensationRateMegaCps;

  if (xTalkStoredMeanRtnSpadsAsInt == 0 || xTalkCalDistanceAsInt == 0 || xTalkStoredMeanRange >= XTalkCalDistance) {
    XTalkCompensationRateMegaCps = 0;
  } else {
    /* Round Cal Distance to Whole Number.
     *  Note that the cal distance is in mm, therefore no
     *  resolution is lost.*/
    xTalkCalDistanceAsInt = (XTalkCalDistance + 0x8000) >> 16;

    /* Apply division by mean spad count early in the
     * calculation to keep the numbers small.
     * This ensures we can maintain a 32bit calculation.
     * Fixed1616 / int := Fixed1616 */
    uint32_t signalXTalkTotalPerSpad = (xTalkStoredMeanSignalRate) / xTalkStoredMeanRtnSpadsAsInt;

    /* Complete the calculation for total Signal XTalk per
     * SPAD
     * Fixed1616 * (Fixed1616 - Fixed1616/int) :=
     * (2^16 * Fixed1616)
     */
    signalXTalkTotalPerSpad *= ((1 << 16) - (xTalkStoredMeanRange / xTalkCalDistanceAsInt));

    /* Round from 2^16 * Fixed1616, to Fixed1616. */
    XTalkCompensationRateMegaCps = (signalXTalkTotalPerSpad + 0x8000) >> 16;
  }

  *pXTalkCompensationRateMegaCps = XTalkCompensationRateMegaCps;
  ERROR_OUT;
  /* Enable the XTalk compensation */
  Error = VL53L0X_SetXTalkCompensationEnable(Dev, 1);
  ERROR_OUT;

  /* Enable the XTalk compensation */
  return VL53L0X_SetXTalkCompensationRateMegaCps(Dev, XTalkCompensationRateMegaCps);
} // VL53L0X_perform_xtalk_calibration

VL53L0X_Error VL53L0X_perform_offset_calibration(VL53L0X_DEV Dev, FixPoint1616_t CalDistanceMilliMeter, int32_t *pOffsetMicroMeter) {
  if (CalDistanceMilliMeter <= 0) {
    return VL53L0X_ERROR_INVALID_PARAMS;
  }
  VL53L0X_Error Error = VL53L0X_SetOffsetCalibrationDataMicroMeter(Dev, 0);
  ERROR_OUT;
  /* Get the value of the TCC */
  uint8_t SequenceStepEnabled;
  Error = VL53L0X_GetSequenceStepEnable(Dev, VL53L0X_SEQUENCESTEP_TCC, &SequenceStepEnabled);
  ERROR_OUT;
  /* Disable the TCC */
  Error = VL53L0X_SetSequenceStepEnable(Dev, VL53L0X_SEQUENCESTEP_TCC, 0);
  ERROR_OUT;
  /* Disable the RIT */
  Error = VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);
  ERROR_OUT;
  /* Perform 50 measurements and compute the averages */
  uint16_t sum_ranging = 0;
  FixPoint1616_t total_count = 0;
  for (int meas = 0; meas < 50; meas++) {
    VL53L0X_RangingMeasurementData_t RangingMeasurementData;
    Error = VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingMeasurementData);
    if (Error != VL53L0X_ERROR_NONE) {
      break;
    }
    /* The range is valid when RangeStatus = 0 */
    if (RangingMeasurementData.RangeStatus == 0) {
      sum_ranging += RangingMeasurementData.RangeMilliMeter;
      ++total_count;
    }
  }
  /* no valid values found */
  if (total_count == 0) {
    return VL53L0X_ERROR_RANGE_ERROR;
  }
  /* FixPoint1616_t / uint16_t = FixPoint1616_t */
  FixPoint1616_t StoredMeanRange = (FixPoint1616_t) ((uint32_t) (sum_ranging << 16) / total_count);
  uint32_t StoredMeanRangeAsInt = (StoredMeanRange + 0x8000) >> 16;

  /* Round Cal Distance to Whole Number.
   * Note that the cal distance is in mm, therefore no resolution
   * is lost.*/
  uint32_t CalDistanceAsInt_mm = (CalDistanceMilliMeter + 0x8000) >> 16;
  *pOffsetMicroMeter = (CalDistanceAsInt_mm - StoredMeanRangeAsInt) * 1000;

  /* Apply the calculated offset */
  VL53L0X_SETPARAMETERFIELD(Dev, RangeOffsetMicroMeters, *pOffsetMicroMeter);
  Error = VL53L0X_SetOffsetCalibrationDataMicroMeter(Dev, *pOffsetMicroMeter);
  ERROR_OUT;

  /* Restore the TCC */
  if (SequenceStepEnabled != 0) {
    return VL53L0X_SetSequenceStepEnable(Dev, VL53L0X_SEQUENCESTEP_TCC, 1);
  }
  return VL53L0X_ERROR_NONE;
} // VL53L0X_perform_offset_calibration

VL53L0X_Error VL53L0X_set_offset_calibration_data_micro_meter(VL53L0X_DEV Dev, int32_t OffsetCalibrationDataMicroMeter) {
  const int32_t cMaxOffsetMicroMeter = 511000;
  const int32_t cMinOffsetMicroMeter = -512000;

  LOG_FUNCTION_START("");
  //coerce into bounds
  if (OffsetCalibrationDataMicroMeter > cMaxOffsetMicroMeter) {
    OffsetCalibrationDataMicroMeter = cMaxOffsetMicroMeter;
  } else if (OffsetCalibrationDataMicroMeter < cMinOffsetMicroMeter) {
    OffsetCalibrationDataMicroMeter = cMinOffsetMicroMeter;
  }

  /* The offset register is 10.2 format and units are mm
   * therefore conversion is applied by a division of
   * 250.
   */
  const int16_t cOffsetRange = 4096;
  uint32_t encodedOffsetVal = (OffsetCalibrationDataMicroMeter / 250) + ((OffsetCalibrationDataMicroMeter >= 0) ? 0 : cOffsetRange);
  VL53L0X_Error Status = VL53L0X_WrWord(Dev, VL53L0X_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM, encodedOffsetVal);
  LOG_FUNCTION_END(Status);
  return Status;
} // VL53L0X_set_offset_calibration_data_micro_meter

VL53L0X_Error VL53L0X_get_offset_calibration_data_micro_meter(VL53L0X_DEV Dev, int32_t *pOffsetCalibrationDataMicroMeter) {
  const int16_t cMaxOffset = 2047;
  const int16_t cOffsetRange = 4096;

  /* Note that offset has 10.2 format */
  uint16_t RangeOffsetRegister;
  VL53L0X_Error Error = VL53L0X_ERROR_NONE;

  Error = VL53L0X_RdWord(Dev, VL53L0X_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM, &RangeOffsetRegister);
  ERROR_OUT;
  RangeOffsetRegister &= 0x0fff;
  /* Apply 12 bit 2's compliment conversion */
  *pOffsetCalibrationDataMicroMeter = 250 * (int16_t) (RangeOffsetRegister - RangeOffsetRegister > cMaxOffset ? cOffsetRange : 0);
  return VL53L0X_ERROR_NONE;
} // VL53L0X_get_offset_calibration_data_micro_meter

VL53L0X_Error VL53L0X_apply_offset_adjustment(VL53L0X_DEV Dev) {
  /* if we run on this function we can read all the NVM info used by the API */
  VL53L0X_Error Error = VL53L0X_get_info_from_device(Dev, 7);
  ERROR_OUT;
  /* Read back current device offset */
  int32_t CurrentOffsetMicroMeters;
  Error = VL53L0X_GetOffsetCalibrationDataMicroMeter(Dev, &CurrentOffsetMicroMeters);
  ERROR_OUT;
  /* Apply Offset Adjustment derived from 400mm measurements */
  /* Store initial device offset */
  PALDevDataSet(Dev, Part2PartOffsetNVMMicroMeter, CurrentOffsetMicroMeters);
  int32_t CorrectedOffsetMicroMeters = CurrentOffsetMicroMeters + (int32_t) PALDevDataGet(Dev, Part2PartOffsetAdjustmentNVMMicroMeter);

  Error = VL53L0X_SetOffsetCalibrationDataMicroMeter(Dev, CorrectedOffsetMicroMeters);
  ERROR_OUT;
  /* store current, adjusted offset */
  VL53L0X_SETPARAMETERFIELD(Dev, RangeOffsetMicroMeters, CorrectedOffsetMicroMeters);

  return VL53L0X_ERROR_NONE;
} // VL53L0X_apply_offset_adjustment

void get_next_good_spad(uint8_t goodSpadArray[], uint32_t size, uint32_t curr, int32_t *next) {
  /*
   * Starting with the current good spad, loop through the array to find
   * the next. i.e. the next bit set in the sequence.
   *
   * The coarse index is the byte index of the array and the fine index is
   * the index of the bit within each byte.
   */
  unsigned startIndex = curr / cSpadsPerByte;
  unsigned fineOffset = curr % cSpadsPerByte;

  for (unsigned coarseIndex = startIndex; coarseIndex < size; coarseIndex++) {
    unsigned fineIndex = 0;
    uint8_t dataByte = goodSpadArray[coarseIndex];

    if (coarseIndex == startIndex) {
      /* locate the bit position of the provided current
       * spad bit before iterating */
      dataByte >>= fineOffset;
      fineIndex = fineOffset;
    }

    while (fineIndex < cSpadsPerByte) {
      if (dataByte & 0x1) {
        *next = coarseIndex * cSpadsPerByte + fineIndex;
        return;
      }
      dataByte >>= 1;
      fineIndex++;
    }
  }
  *next = -1;
} // get_next_good_spad

uint8_t is_aperture(uint32_t spadIndex) {
  /*
   * This function reports if a given spad index is an aperture SPAD by
   * deriving the quadrant.
   */
  return refArrayQuadrants[spadIndex >> 6] != REF_ARRAY_SPAD_0;
}

VL53L0X_Error enable_spad_bit(uint8_t spadArray[], uint32_t size, uint32_t spadIndex) {
  uint32_t coarseIndex = spadIndex / cSpadsPerByte;
  uint32_t fineIndex = spadIndex % cSpadsPerByte;
  if (coarseIndex >= size) {
    return VL53L0X_ERROR_REF_SPAD_INIT;
  }
  spadArray[coarseIndex] |= (1 << fineIndex);
  return VL53L0X_ERROR_NONE;
} // enable_spad_bit

VL53L0X_Error count_enabled_spads(uint8_t spadArray[], uint32_t byteCount, uint32_t maxSpads, uint32_t *pTotalSpadsEnabled, uint8_t *pIsAperture) {
  VL53L0X_Error status = VL53L0X_ERROR_NONE;
  uint8_t spadTypeIdentified = 0;

  /* The entire array will not be used for spads, therefore the last
   * byte and last bit is determined from the max spads value.
   */
  unsigned lastByte = maxSpads / cSpadsPerByte;
  unsigned lastBit = maxSpads % cSpadsPerByte;

  /* Check that the max spads value does not exceed the array bounds. */
  if (lastByte >= byteCount) {
    status = VL53L0X_ERROR_REF_SPAD_INIT;
    //BUG: SERIOUS either bail out or clip to given byteCount
  }

  //stupid: use a local to count and do a single assignment when done. Updating the result as we go precludes the compiler from using a register.
  *pTotalSpadsEnabled = 0;

  /* Count the bits enabled in the whole bytes */
  for (unsigned byteIndex = 0; byteIndex <= (lastByte - 1); byteIndex++) {
    uint8_t tempByte = spadArray[byteIndex];

    for (unsigned bitIndex = 0; bitIndex <= cSpadsPerByte; bitIndex++) {
      if ((tempByte & 0x01) == 1) {
        ++*pTotalSpadsEnabled;

        if (!spadTypeIdentified) {
          *pIsAperture = 1;
          if ((byteIndex < 2) && (bitIndex < 4)) {
            *pIsAperture = 0;
          }
          spadTypeIdentified = 1;
        }
      }
      tempByte >>= 1;
    }
  }
  /* Count the number of bits enabled in the last byte accounting
   * for the fact that not all bits in the byte may be used.
   */
  uint8_t tempByte = spadArray[lastByte];

  for (unsigned bitIndex = 0; bitIndex <= lastBit; bitIndex++) {
    if ((tempByte & 0x01) == 1) {
      ++*pTotalSpadsEnabled;
    }
    //BUG: this is the bug that triggered rewriting of this whole application, forgot to shift the tempByte while counting its bits.
    //... the real fix is to not bother with this since the unused bits will be zero and we inspect them without harm.
  }

  return status;
} // count_enabled_spads

VL53L0X_Error set_ref_spad_map(VL53L0X_DEV Dev, uint8_t *refSpadArray) {
  return VL53L0X_WriteMulti(Dev, VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, refSpadArray, 6);
}

VL53L0X_Error get_ref_spad_map(VL53L0X_DEV Dev, uint8_t *refSpadArray) {
  return VL53L0X_ReadMulti(Dev, VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, refSpadArray, 6);
}

VL53L0X_Error enable_ref_spads(VL53L0X_DEV Dev, uint8_t apertureSpads, uint8_t goodSpadArray[], uint8_t spadArray[], uint32_t size, uint32_t start, uint32_t offset, uint32_t spadCount, uint32_t *lastSpad) {


  /*
   * This function takes in a spad array which may or may not have SPADS
   * already enabled and appends from a given offset a requested number
   * of new SPAD enables. The 'good spad map' is applied to
   * determine the next SPADs to enable.
   *
   * This function applies to only aperture or only non-aperture spads.
   * Checks are performed to ensure this.
   */
  VL53L0X_Error Error = VL53L0X_ERROR_NONE;
  uint32_t currentSpad = offset;
  for (uint32_t index = 0; index < spadCount; index++) {
    int32_t nextGoodSpad;
    get_next_good_spad(goodSpadArray, size, currentSpad, &nextGoodSpad);

    if (nextGoodSpad == -1) {
      Error = VL53L0X_ERROR_REF_SPAD_INIT;
      break;
    }

    /* Confirm that the next good SPAD is non-aperture */
    if (is_aperture(start + nextGoodSpad) != apertureSpads) {
      /* if we can't get the required number of good aperture
       * spads from the current quadrant then this is an error
       */
      Error = VL53L0X_ERROR_REF_SPAD_INIT;
      break;
    }
    currentSpad = (uint32_t) nextGoodSpad;
    enable_spad_bit(spadArray, size, currentSpad);
    currentSpad++;
  }
  *lastSpad = currentSpad;
  ERROR_OUT;
  Error = set_ref_spad_map(Dev, spadArray);
  ERROR_OUT;

  uint8_t checkSpadArray[6];
  Error = get_ref_spad_map(Dev, checkSpadArray);
  /* Compare spad maps. If not equal report error. */
  for (uint32_t i = 0; i < size; ++i) {
    if (spadArray[i] != checkSpadArray[i]) {
      return VL53L0X_ERROR_REF_SPAD_INIT;
    }
  }

  return VL53L0X_ERROR_NONE;
} // enable_ref_spads

VL53L0X_Error perform_ref_signal_measurement(VL53L0X_DEV Dev, uint16_t *refSignalRate) {
  /* store the value of the sequence config,
   * this will be reset before the end of the function
   */
  uint8_t SequenceConfig = PALDevDataGet(Dev, SequenceConfig);
  /*
   * This function performs a reference signal rate measurement.
   */
  VL53L0X_Error Error = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xC0);
  ERROR_OUT;

  VL53L0X_RangingMeasurementData_t rangingMeasurementData;
  Error = VL53L0X_PerformSingleRangingMeasurement(Dev, &rangingMeasurementData);
  //BUG: object ignored above.
  ERROR_OUT;
  Error = VL53L0X_WrByte(Dev, 0xFF, 0x01);
  ERROR_OUT;
  Error = VL53L0X_RdWord(Dev, VL53L0X_REG_RESULT_PEAK_SIGNAL_RATE_REF, refSignalRate);
  ERROR_OUT;
  Error = VL53L0X_WrByte(Dev, 0xFF, 0x00);
  ERROR_OUT;
  /* restore the previous Sequence Config */
  Error = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, SequenceConfig);
  ERROR_OUT;
  PALDevDataSet(Dev, SequenceConfig, SequenceConfig);

  return Error;
} // perform_ref_signal_measurement

VL53L0X_Error VL53L0X_perform_ref_spad_management(VL53L0X_DEV Dev, uint32_t *refSpadCount, uint8_t *isApertureSpads) {


  /*
   * The reference SPAD initialization procedure determines the minimum
   * amount of reference spads to be enables to achieve a target reference
   * signal rate and should be performed once during initialization.
   *
   * Either aperture or non-aperture spads are applied but never both.
   * Firstly non-aperture spads are set, begining with 5 spads, and
   * increased one spad at a time until the closest measurement to the
   * target rate is achieved.
   *
   * If the target rate is exceeded when 5 non-aperture spads are enabled,
   * initialization is performed instead with aperture spads.
   *
   * When setting spads, a 'Good Spad Map' is applied.
   *
   * This procedure operates within a SPAD window of interest of a maximum
   * 44 spads.
   * The start point is currently fixed to 180, which lies towards the end
   * of the non-aperture quadrant and runs in to the adjacent aperture
   * quadrant.
   */

  uint16_t targetRefRate = 0x0A00; /* 20 MCPS in 9:7 format */ //ick:why init with specific value that is immediately overwritten
  targetRefRate = PALDevDataGet(Dev, targetRefRate);

  /*
   * Initialize Spad arrays.
   * Currently the good spad map is initialised to 'All good'.
   * This is a short term implementation. The good spad map will be
   * provided as an input.
   */
  for (unsigned index = 0; index < spadArraySize; index++) {
    Dev->Data.SpadData.RefSpadEnables[index] = 0;
  }

  VL53L0X_Error Error = VL53L0X_WrByte(Dev, 0xFF, 0x01);
  ERROR_OUT;
  Error = VL53L0X_WrByte(Dev, VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  ERROR_OUT;
  Error = VL53L0X_WrByte(Dev, VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  ERROR_OUT;
  Error = VL53L0X_WrByte(Dev, 0xFF, 0x00);
  ERROR_OUT;
  Error = VL53L0X_WrByte(Dev, VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT, startSelect);
  ERROR_OUT;
  Error = VL53L0X_WrByte(Dev, VL53L0X_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0);
  ERROR_OUT;
  /* Perform ref calibration */
  uint8_t PhaseCal = 0;
  uint8_t VhvSettings = 0;
  Error = VL53L0X_perform_ref_calibration(Dev, &VhvSettings, &PhaseCal, 0);//ick: PhaseCal value then ignored. ditto for VhvSettings
  ERROR_OUT;
  /* Enable Minimum NON-APERTURE Spads */
  uint32_t currentSpadIndex = 0;
  uint32_t lastSpadIndex = currentSpadIndex;
  uint32_t needAptSpads = 0;
  Error = enable_ref_spads(Dev, needAptSpads, Dev->Data.SpadData.RefGoodSpadMap, Dev->Data.SpadData.RefSpadEnables, spadArraySize, startSelect, currentSpadIndex, minimumSpadCount, &lastSpadIndex);
  ERROR_OUT;
  currentSpadIndex = lastSpadIndex;
  uint16_t peakSignalRateRef;
  Error = perform_ref_signal_measurement(Dev, &peakSignalRateRef);
  ERROR_OUT;

  uint8_t isApertureSpads_int = 0;
  uint32_t refSpadCount_int = 0;

  if (peakSignalRateRef > targetRefRate) { /* Signal rate measurement too high, switch to APERTURE SPADs */
    for (unsigned index = 0; index < spadArraySize; index++) {
      Dev->Data.SpadData.RefSpadEnables[index] = 0;
    }
    /* Increment to the first APERTURE spad */
    while (!is_aperture(startSelect + currentSpadIndex) && (currentSpadIndex < maxSpadCount)) {
      currentSpadIndex++;
    }
    needAptSpads = 1;
    Error = enable_ref_spads(Dev, needAptSpads, Dev->Data.SpadData.RefGoodSpadMap, Dev->Data.SpadData.RefSpadEnables, spadArraySize, startSelect, currentSpadIndex, minimumSpadCount, &lastSpadIndex);

    if (Error == VL53L0X_ERROR_NONE) {
      currentSpadIndex = lastSpadIndex;
      Error = perform_ref_signal_measurement(Dev, &peakSignalRateRef);

      if ((Error == VL53L0X_ERROR_NONE) &&
          (peakSignalRateRef > targetRefRate)) {
        /* Signal rate still too high after
         * setting the minimum number of
         * APERTURE spads. Can do no more
         * therefore set the min number of
         * aperture spads as the result.
         */
        isApertureSpads_int = 1;
        refSpadCount_int = minimumSpadCount;
      }
    }
  } else {
    needAptSpads = 0;
  }
  ERROR_OUT;
  if (peakSignalRateRef < targetRefRate) {
/* At this point, the minimum number of either aperture
 * or non-aperture spads have been set. Proceed to add
 * spads and perform measurements until the target
 * reference is reached.
 */
    isApertureSpads_int = needAptSpads;
    refSpadCount_int = minimumSpadCount;

    uint8_t lastSpadArray[6];
    memcpy(lastSpadArray, Dev->Data.SpadData.RefSpadEnables, spadArraySize);

    uint32_t lastSignalRateDiff = abs(peakSignalRateRef - targetRefRate);
    uint8_t complete = 0;
    while (!complete) {
      int32_t nextGoodSpad = 0;
      get_next_good_spad(Dev->Data.SpadData.RefGoodSpadMap, spadArraySize, currentSpadIndex, &nextGoodSpad);
      if (nextGoodSpad == -1) {
        return VL53L0X_ERROR_REF_SPAD_INIT;
      }
      ++refSpadCount_int;
/* Cannot combine Aperture and Non-Aperture spads, so
 * ensure the current spad is of the correct type.
 */
      if (is_aperture((uint32_t) startSelect + nextGoodSpad) != needAptSpads) {
        return VL53L0X_ERROR_REF_SPAD_INIT;
      }

      currentSpadIndex = nextGoodSpad;
      Error = enable_spad_bit(Dev->Data.SpadData.RefSpadEnables, spadArraySize, currentSpadIndex);
      ERROR_OUT;
      currentSpadIndex++;
/* Proceed to apply the additional spad and
 * perform measurement. */
      Error = set_ref_spad_map(Dev, Dev->Data.SpadData.RefSpadEnables);

      ERROR_OUT;
      Error = perform_ref_signal_measurement(Dev, &peakSignalRateRef);
      ERROR_OUT;
      uint32_t signalRateDiff = abs(peakSignalRateRef - targetRefRate);

      if (peakSignalRateRef > targetRefRate) { /* Select the spad map that provides the measurement closest to the target rate, either above or below it. */
        if (signalRateDiff > lastSignalRateDiff) { /* Previous spad map produced a closer measurement, so choose this. */
          Error = set_ref_spad_map(Dev, lastSpadArray);
          memcpy(Dev->Data.SpadData.RefSpadEnables, lastSpadArray, spadArraySize);
          --refSpadCount_int;
        }
        complete = 1;
      } else { /* Continue to add spads */
        lastSignalRateDiff = signalRateDiff;
        memcpy(lastSpadArray, Dev->Data.SpadData.RefSpadEnables, spadArraySize);
      }
    } /* while */
  }

  ERROR_OUT;
  *refSpadCount = refSpadCount_int;
  *isApertureSpads = isApertureSpads_int;

  VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, RefSpadsInitialised, 1);
  VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, ReferenceSpadCount, (uint8_t) (*refSpadCount));
  VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, ReferenceSpadType, *isApertureSpads);

  return VL53L0X_ERROR_NONE;
} // VL53L0X_perform_ref_spad_management

VL53L0X_Error VL53L0X_set_reference_spads(VL53L0X_DEV Dev, uint32_t count, uint8_t isApertureSpads) {
  //code moved here as it executed regardless of error in the I2C writes
  for (unsigned index = 0; index < spadArraySize; index++) {
    Dev->Data.SpadData.RefSpadEnables[index] = 0;
  }

  uint32_t currentSpadIndex = 0;
  if (isApertureSpads) {
    /* Increment to the first APERTURE spad */
    while ((is_aperture(startSelect + currentSpadIndex) == 0) && (currentSpadIndex < maxSpadCount)) {
      ++currentSpadIndex;
    }
  }

  /*
   * This function applies a requested number of reference spads, either aperture or non-aperture, as requested.
   * The good spad map will be applied.
   */

  VL53L0X_Error Error = VL53L0X_WrByte(Dev, 0xFF, 0x01);
  if (!Error) {
    Error = VL53L0X_WrByte(Dev, VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    if (!Error) {
      Error = VL53L0X_WrByte(Dev, VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
      if (!Error) {
        Error = VL53L0X_WrByte(Dev, 0xFF, 0x00);
        if (!Error) {
          Error = VL53L0X_WrByte(Dev, VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT, startSelect);
        }
      }
    }
  }
  //ick: lots of errors but we enable the ref spads anyway? If we shouldn't then we can apply ERROR_OUT above.
  //note: all of the above errors will never happen due to all errors being suppressed in the present i2c interface.
  //ick: we lose all the errors in the above if we proceed.
  uint32_t lastSpadIndex;
  Error = enable_ref_spads(Dev, isApertureSpads, Dev->Data.SpadData.RefGoodSpadMap, Dev->Data.SpadData.RefSpadEnables, spadArraySize, startSelect, currentSpadIndex, count, &lastSpadIndex);
  ERROR_OUT;
  VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, RefSpadsInitialised, 1);
  VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, ReferenceSpadCount, (uint8_t) (count));
  VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, ReferenceSpadType, isApertureSpads);

  return VL53L0X_ERROR_NONE;
} // VL53L0X_set_reference_spads

VL53L0X_Error VL53L0X_get_reference_spads(VL53L0X_DEV Dev, uint32_t *pSpadCount, uint8_t *pIsApertureSpads) {
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  uint8_t refSpadsInitialised;
  uint8_t refSpadArray[6];

  uint32_t spadsEnabled;
  uint8_t isApertureSpads = 0;

  refSpadsInitialised = VL53L0X_GETDEVICESPECIFICPARAMETER(Dev, RefSpadsInitialised);

  if (refSpadsInitialised == 1) {
    *pSpadCount = (uint32_t) VL53L0X_GETDEVICESPECIFICPARAMETER(Dev, ReferenceSpadCount);
    *pIsApertureSpads = VL53L0X_GETDEVICESPECIFICPARAMETER(Dev, ReferenceSpadType);
  } else {
    /* obtain spad info from device.*/
    Status = get_ref_spad_map(Dev, refSpadArray);

    if (Status == VL53L0X_ERROR_NONE) {
      /* count enabled spads within spad map array and
       * determine if Aperture or Non-Aperture.
       */
      Status = count_enabled_spads(refSpadArray, cSpadArraySize, cMaxSpadCount, &spadsEnabled, &isApertureSpads);

      if (Status == VL53L0X_ERROR_NONE) {
        *pSpadCount = spadsEnabled;
        *pIsApertureSpads = isApertureSpads;
        VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, RefSpadsInitialised, 1);
        VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, ReferenceSpadCount, (uint8_t) spadsEnabled);
        VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, ReferenceSpadType, isApertureSpads);
      }
    }
  }

  return Status;
} // VL53L0X_get_reference_spads

VL53L0X_Error VL53L0X_perform_single_ref_calibration(VL53L0X_DEV Dev, uint8_t vhv_init_byte) {
  VL53L0X_Error Error = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_START_STOP | vhv_init_byte);
  ERROR_OUT;
  Error = VL53L0X_measurement_poll_for_completion(Dev);
  ERROR_OUT;
  Error = VL53L0X_ClearInterruptMask(Dev, 0);
  ERROR_OUT;
  return VL53L0X_WrByte(Dev, VL53L0X_REG_SYSRANGE_START, 0x00);
} // VL53L0X_perform_single_ref_calibration

VL53L0X_Error VL53L0X_ref_calibration_io(VL53L0X_DEV Dev, uint8_t read_not_write, uint8_t VhvSettings, uint8_t PhaseCal, uint8_t *pVhvSettings, uint8_t *pPhaseCal, const uint8_t vhv_enable, const uint8_t phase_enable) {
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;


  /* Read VHV from device */
  Status |= VL53L0X_WrByte(Dev, 0xFF, 0x01);
  Status |= VL53L0X_WrByte(Dev, 0x00, 0x00);
  Status |= VL53L0X_WrByte(Dev, 0xFF, 0x00);
  uint8_t PhaseCalint = 0;
  if (read_not_write) {
    if (vhv_enable) {
      Status |= VL53L0X_RdByte(Dev, 0xCB, pVhvSettings);
    }
    if (phase_enable) {
      Status |= VL53L0X_RdByte(Dev, 0xEE, &PhaseCalint);
    }
  } else {
    if (vhv_enable) {
      Status |= VL53L0X_WrByte(Dev, 0xCB, VhvSettings);
    }
    if (phase_enable) {
      Status |= VL53L0X_UpdateByte(Dev, 0xEE, 0x80, PhaseCal);
    }
  }

  Status |= VL53L0X_WrByte(Dev, 0xFF, 0x01);
  Status |= VL53L0X_WrByte(Dev, 0x00, 0x01);
  Status |= VL53L0X_WrByte(Dev, 0xFF, 0x00);

  *pPhaseCal = (uint8_t) (PhaseCalint & 0xEF);

  return Status;
} // VL53L0X_ref_calibration_io

VL53L0X_Error VL53L0X_perform_vhv_calibration(VL53L0X_DEV Dev, uint8_t *pVhvSettings, const uint8_t get_data_enable, const uint8_t restore_config) {
  /* store the value of the sequence config, this will be reset before the end of the function */
  uint8_t SequenceConfig = restore_config ? PALDevDataGet(Dev, SequenceConfig) : 0;
  /* Run VHV */
  VL53L0X_Error Error = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x01);

  if (!Error) {
    Error = VL53L0X_perform_single_ref_calibration(Dev, 0x40);
  }

  /* Read VHV from device */
  if (!Error && (get_data_enable == 1)) {
    uint8_t PhaseCalInt = 0;//ick: read and ignored
    uint8_t PhaseCal = 0;//ick: read and ignored
    Error = VL53L0X_ref_calibration_io(Dev, 1, 0, PhaseCal /* Not used here */, pVhvSettings, &PhaseCalInt, 1, 0);
  } else {
    *pVhvSettings = 0;
  }

  if (!Error && restore_config) {
    /* restore the previous Sequence Config */
    Error = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, SequenceConfig);
    if (!Error) {
      PALDevDataSet(Dev, SequenceConfig, SequenceConfig);
    }
  }

  return Error;
} // VL53L0X_perform_vhv_calibration

VL53L0X_Error VL53L0X_perform_phase_calibration(VL53L0X_DEV Dev, uint8_t *pPhaseCal, const uint8_t get_data_enable, const uint8_t restore_config) {

  /* store the value of the sequence config, this will be reset before the end of the function */
  uint8_t SequenceConfig = restore_config ? PALDevDataGet(Dev, SequenceConfig) : 0;

  /* Run PhaseCal */
  VL53L0X_Error Status = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02);

  if (Status == VL53L0X_ERROR_NONE) {
    Status = VL53L0X_perform_single_ref_calibration(Dev, 0x0);
  }

  /* Read PhaseCal from device */
  if ((Status == VL53L0X_ERROR_NONE) && (get_data_enable == 1)) {
    uint8_t VhvSettingsint;//ick: ignored
    Status = VL53L0X_ref_calibration_io(Dev, 1, 0, 0, &VhvSettingsint, pPhaseCal, 0, 1);
  } else {
    *pPhaseCal = 0;
  }

  if ((Status == VL53L0X_ERROR_NONE) && restore_config) {
    /* restore the previous Sequence Config */
    Status = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, SequenceConfig);
    if (Status == VL53L0X_ERROR_NONE) {
      PALDevDataSet(Dev, SequenceConfig, SequenceConfig);
    }
  }

  return Status;
} // VL53L0X_perform_phase_calibration

VL53L0X_Error VL53L0X_perform_ref_calibration(VL53L0X_DEV Dev, uint8_t *pVhvSettings, uint8_t *pPhaseCal, uint8_t get_data_enable) {
  /* store the value of the sequence config,
   * this will be reset before the end of the function
   */
  uint8_t SequenceConfig  = PALDevDataGet(Dev, SequenceConfig);

  /* In the following function we don't save the config to optimize
   * writes on device. Config is saved and restored only once. */
  VL53L0X_Error Status = VL53L0X_perform_vhv_calibration(Dev, pVhvSettings, get_data_enable, 0);

  if (Status == VL53L0X_ERROR_NONE) {
    Status = VL53L0X_perform_phase_calibration(Dev, pPhaseCal, get_data_enable, 0);
  }

  if (Status == VL53L0X_ERROR_NONE) {
    /* restore the previous Sequence Config */
    Status = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, SequenceConfig);
    if (Status == VL53L0X_ERROR_NONE) {
      PALDevDataSet(Dev, SequenceConfig, SequenceConfig);
    }
  }

  return Status;
} // VL53L0X_perform_ref_calibration

VL53L0X_Error VL53L0X_set_ref_calibration(VL53L0X_DEV Dev, uint8_t VhvSettings, uint8_t PhaseCal) {
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  uint8_t pVhvSettings;//ick: ignored
  uint8_t pPhaseCal;//ick: ignored
  return VL53L0X_ref_calibration_io(Dev, 0, VhvSettings, PhaseCal, &pVhvSettings, &pPhaseCal, 1, 1);
}

VL53L0X_Error VL53L0X_get_ref_calibration(VL53L0X_DEV Dev, uint8_t *pVhvSettings, uint8_t *pPhaseCal) {
  return VL53L0X_ref_calibration_io(Dev, 1, 0, 0, pVhvSettings, pPhaseCal, 1, 1);
}
