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

//#include "vl53l0x_api_calibration.h"
//still somewhat tangled but at least we can kill this part since apparently the layer violation was to get time tracing:  #include "vl53l0x_api.h"
#include "vl53l0x_api.h" //private members of the class, split across files due to size of files.

#include "log_api.h"
#include "vl53l0x_spadarray.h"

//ick: use enums to constrain the range of an integer, not typeless symbols
#define REF_ARRAY_SPAD_0 0
#define REF_ARRAY_SPAD_5 5
#define REF_ARRAY_SPAD_10 10

//there are 64 spads per quadrant. This array seems to be where the aperture is in the related quadrant
const uint32_t refArrayQuadrants[4] = {REF_ARRAY_SPAD_10, REF_ARRAY_SPAD_5, REF_ARRAY_SPAD_0, REF_ARRAY_SPAD_5};

static const unsigned startSelect = 180;// was 0xB4 but is not a bit pattern, rather it is a decimal number

static const unsigned minimumSpadCount = 3;

namespace VL53L0X {

  Error Api::perform_xtalk_calibration(FixPoint1616_t XTalkCalDistance, FixPoint1616_t &pXTalkCompensationRateMegaCps) {
    if (XTalkCalDistance.raw <= 0) {//ICK: type was unsigned, and so this is a compare to zero.
      return ERROR_INVALID_PARAMS;
    }
    /* Disable the XTalk compensation */
    Error Error = SetXTalkCompensationEnable( 0);
    ERROR_OUT;

    /* Disable the RIT */
    Error = SetLimitCheckEnable( CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);
    ERROR_OUT;

    /* Perform 50 measurements and compute the averages */
    unsigned sum_ranging = 0;//bug: former use of 16 bit begs for integer overflow
    unsigned sum_spads = 0;//... which we will tolerate only on processors whose natural int is 16 bits
    FixPoint1616_t sum_signalRate = 0;
    FixPoint1616_t total_count = 0;
    for (uint8_t xtalk_meas = 0; xtalk_meas < 50; xtalk_meas++) {
      RangingMeasurementData_t RangingMeasurementData;
      Error =PerformSingleRangingMeasurement( &RangingMeasurementData);

      if (Error != ERROR_NONE) {
        return total_count == 0 ? ERROR_RANGE_ERROR : Error;
      }

      /* The range is valid when RangeStatus = 0 */
      if (RangingMeasurementData.RangeStatus == 0) {
        sum_ranging += RangingMeasurementData.RangeMilliMeter;
        sum_signalRate.raw += RangingMeasurementData.SignalRateRtnMegaCps.raw;
        sum_spads += RangingMeasurementData.EffectiveSpadRtnCount / 256;
        ++total_count.raw;
      }
    }
    /* no valid values found */
    if (total_count.raw == 0) {
      return ERROR_RANGE_ERROR;
    }

    /* FixPoint1616_t / uint16_t = FixPoint1616_t */
    FixPoint1616_t xTalkStoredMeanSignalRate = sum_signalRate.raw / total_count.raw;//ick: not rounded
    FixPoint1616_t xTalkStoredMeanRange = (FixPoint1616_t) ( (sum_ranging << 16) / total_count);
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
    Error = SetXTalkCompensationEnable(1);
    ERROR_OUT;

    /* Enable the XTalk compensation */
    return SetXTalkCompensationRateMegaCps(XTalkCompensationRateMegaCps);
  } // VL53L0X_perform_xtalk_calibration

  Error Api::perform_offset_calibration( FixPoint1616_t CalDistanceMilliMeter, int32_t *pOffsetMicroMeter) {
    if (CalDistanceMilliMeter.raw <= 0) {
      return ERROR_INVALID_PARAMS;
    }
    Error Error = SetOffsetCalibrationDataMicroMeter( 0);
    ERROR_OUT;
    /* Get the value of the TCC */
    bool SequenceStepEnabled= get_sequence_() VL53L0X_GetSequenceStepEnable(Dev, SEQUENCESTEP_TCC, &SequenceStepEnabled);
    ERROR_OUT;
    /* Disable the TCC */
    Error = VL53L0X_SetSequenceStepEnable(Dev, SEQUENCESTEP_TCC, 0);
    ERROR_OUT;
    /* Disable the RIT */
    Error = VL53L0X_SetLimitCheckEnable(Dev, CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);
    ERROR_OUT;
    /* Perform 50 measurements and compute the averages */
    uint16_t sum_ranging = 0;
    FixPoint1616_t total_count = 0;
    for (int meas = 0; meas < 50; meas++) {
      RangingMeasurementData_t RangingMeasurementData;
      Error = VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingMeasurementData);
      if (Error != ERROR_NONE) {
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
      return ERROR_RANGE_ERROR;
    }
    /* FixPoint1616_t / uint16_t = FixPoint1616_t */
    FixPoint1616_t StoredMeanRange = (FixPoint1616_t) ((uint32_t) (boost(sum_ranging, 16) / total_count.raw);
    uint32_t StoredMeanRangeAsInt = StoredMeanRange.shrink(16);

    /* Round Cal Distance to Whole Number.
     * Note that the cal distance is in mm, therefore no resolution
     * is lost.*/
    uint32_t CalDistanceAsInt_mm = CalDistanceMilliMeter.shrink(16);
    *pOffsetMicroMeter = (CalDistanceAsInt_mm - StoredMeanRangeAsInt) * 1000;

    /* Apply the calculated offset */
    VL53L0X_SETPARAMETERFIELD(this, RangeOffsetMicroMeters, *pOffsetMicroMeter);
    Error = SetOffsetCalibrationDataMicroMeter(Dev, *pOffsetMicroMeter);
    ERROR_OUT;

    /* Restore the TCC */
    if (SequenceStepEnabled != 0) {
      return VL53L0X_SetSequenceStepEnable(Dev, SEQUENCESTEP_TCC, 1);
    }
    return ERROR_NONE;
  } // VL53L0X_perform_offset_calibration

  Error VL53L0X_set_offset_calibration_data_micro_meter( int32_t OffsetCalibrationDataMicroMeter) {
    const int32_t cMaxOffsetMicroMeter = 511000;
    const int32_t cMinOffsetMicroMeter = -512000;

    LOG_FUNCTION_START
    ("");
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
    Error Status = VL53L0X_WrWord(Dev, REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM, encodedOffsetVal);

    return Status;
  } // VL53L0X_set_offset_calibration_data_micro_meter

  Error Api::get_offset_calibration_data_micro_meter( int32_t *pOffsetCalibrationDataMicroMeter) {
    const int16_t cMaxOffset = 2047;
    const int16_t cOffsetRange = 4096;

    /* Note that offset has 10.2 format */
    uint16_t RangeOffsetRegister;
    Error Error = ERROR_NONE;

    Error = VL53L0X_RdWord(Dev, REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM, &RangeOffsetRegister);
    ERROR_OUT;
    RangeOffsetRegister &= 0x0fff;
    /* Apply 12 bit 2's compliment conversion */
    *pOffsetCalibrationDataMicroMeter = 250 * (int16_t) (RangeOffsetRegister - RangeOffsetRegister > cMaxOffset ? cOffsetRange : 0);
    return ERROR_NONE;
  } // VL53L0X_get_offset_calibration_data_micro_meter

  Error Api::apply_offset_adjustment() {
    /* if we run on this function we can read all the NVM info used by the API */
    Error Error = VL53L0X_get_info_from_device(Dev, 7);
    ERROR_OUT;
    /* Read back current device offset */
    Erroneous<int32_t> CurrentOffsetMicroMeters = GetOffsetCalibrationDataMicroMeter();

    ERROR_OUT;
    /* Apply Offset Adjustment derived from 400mm measurements */
    /* Store initial device offset */
    PALDevDataSet(Dev, Part2PartOffsetNVMMicroMeter, CurrentOffsetMicroMeters);
    int32_t CorrectedOffsetMicroMeters = CurrentOffsetMicroMeters + (int32_t) PALDevDataGet(this, Part2PartOffsetAdjustmentNVMMicroMeter);

    Error = SetOffsetCalibrationDataMicroMeter(Dev, CorrectedOffsetMicroMeters);
    ERROR_OUT;
    /* store current, adjusted offset */
    VL53L0X_SETPARAMETERFIELD(Dev, RangeOffsetMicroMeters, CorrectedOffsetMicroMeters);

    return ERROR_NONE;
  } // VL53L0X_apply_offset_adjustment

  void Api::get_next_good_spad(uint8_t goodSpadArray[], uint32_t size, uint32_t curr, int32_t *next) {
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

  bool is_aperture(uint32_t spadIndex) {
    /*
     * This function reports if a given spad index is an aperture SPAD by
     * deriving the quadrant.
     */
    return refArrayQuadrants[spadIndex >> 6] != REF_ARRAY_SPAD_0;
  }




  Error enable_ref_spads( uint8_t apertureSpads, uint8_t goodSpadArray[], uint8_t spadArray[], uint32_t size, uint32_t start, uint32_t offset, uint32_t spadCount, uint32_t *lastSpad) {


    /*
     * This function takes in a spad array which may or may not have SPADS
     * already enabled and appends from a given offset a requested number
     * of new SPAD enables. The 'good spad map' is applied to
     * determine the next SPADs to enable.
     *
     * This function applies to only aperture or only non-aperture spads.
     * Checks are performed to ensure this.
     */
    Error Error = ERROR_NONE;
    uint32_t currentSpad = offset;
    for (uint32_t index = 0; index < spadCount; index++) {
      int32_t nextGoodSpad;
      get_next_good_spad(goodSpadArray, size, currentSpad, &nextGoodSpad);

      if (nextGoodSpad == -1) {
        Error = ERROR_REF_SPAD_INIT;
        break;
      }

      /* Confirm that the next good SPAD is non-aperture */
      if (is_aperture(start + nextGoodSpad) != apertureSpads) {
        /* if we can't get the required number of good aperture
         * spads from the current quadrant then this is an error
         */
        Error = ERROR_REF_SPAD_INIT;
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

    SpadArray checkSpadArray;
    Error = get_ref_spad_map( checkSpadArray);
    /* Compare spad maps. If not equal report error. */
   if( spadArray != checkSpadArray) {
        return ERROR_REF_SPAD_INIT;

    }

    return ERROR_NONE;
  } // enable_ref_spads

  Error perform_ref_signal_measurement( uint16_t *refSignalRate) {
    /* store the value of the sequence config,
     * this will be reset before the end of the function
     */
    uint8_t SequenceConfig = PALDevDataGet(this, SequenceConfig);
    /*
     * This function performs a reference signal rate measurement.
     */
    Error Error = comm.WrByte( REG_SYSTEM_SEQUENCE_CONFIG, 0xC0);
    ERROR_OUT;

    RangingMeasurementData_t rangingMeasurementData;
    Error = VL53L0X_PerformSingleRangingMeasurement(Dev, &rangingMeasurementData);
    //BUG: object ignored above.
    ERROR_OUT;
    Error = comm.WrByte( 0xFF, 0x01);
    ERROR_OUT;
    Error = VL53L0X_RdWord(Dev, REG_RESULT_PEAK_SIGNAL_RATE_REF, refSignalRate);
    ERROR_OUT;
    Error = comm.WrByte( 0xFF, 0x00);
    ERROR_OUT;
    /* restore the previous Sequence Config */
    Error = comm.WrByte( REG_SYSTEM_SEQUENCE_CONFIG, SequenceConfig);
    ERROR_OUT;
    PALDevDataSet(Dev, SequenceConfig, SequenceConfig);

    return Error;
  } // perform_ref_signal_measurement


  Error Api::set_reference_spads( uint32_t count, uint8_t isApertureSpads) {
    //code moved here as it executed regardless of error in the I2C writes
      Data.SpadData.RefSpadEnables.clear();

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

    Error Error = comm.WrByte( 0xFF, 0x01);
    if (!Error) {
      Error = comm.WrByte( REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
      if (!Error) {
        Error = comm.WrByte( REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
        if (!Error) {
          Error = comm.WrByte( 0xFF, 0x00);
          if (!Error) {
            Error = comm.WrByte( REG_GLOBAL_CONFIG_REF_EN_START_SELECT, startSelect);
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

    return ERROR_NONE;
  } // VL53L0X_set_reference_spads

  Error Api::get_reference_spads( uint32_t *pSpadCount, uint8_t *pIsApertureSpads) {
    Error Status = ERROR_NONE;


    uint32_t spadsEnabled;

    bool refSpadsInitialised = VL53L0X_GETDEVICESPECIFICPARAMETER(this, RefSpadsInitialised);

    if (refSpadsInitialised) {
      *pSpadCount = (uint32_t) VL53L0X_GETDEVICESPECIFICPARAMETER(this, ReferenceSpadCount);
      *pIsApertureSpads = VL53L0X_GETDEVICESPECIFICPARAMETER(this, ReferenceSpadType);
    } else {
      /* obtain spad info from device.*/

      SpadArray refSpadArray;
      Status = get_ref_spad_map( refSpadArray);

      if (Status == ERROR_NONE) {
        /* count enabled spads within spad map array and
         * determine if Aperture or Non-Aperture.
         */

        bool isApertureSpads = false;
        spadsEnabled = refSpadArray.count_enabled(  &isApertureSpads);


          *pSpadCount = spadsEnabled;
          *pIsApertureSpads = isApertureSpads;
          VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, RefSpadsInitialised, 1);
          VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, ReferenceSpadCount, (uint8_t) spadsEnabled);
          VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, ReferenceSpadType, isApertureSpads);

      }
    }

    return Status;
  } // VL53L0X_get_reference_spads

  Error VL53L0X_perform_single_ref_calibration( uint8_t vhv_init_byte) {
    Error Error = comm.WrByte( REG_SYSRANGE_START, REG_SYSRANGE_MODE_START_STOP | vhv_init_byte);
    ERROR_OUT;
    Error = VL53L0X_measurement_poll_for_completion(Dev);
    ERROR_OUT;
    Error = VL53L0X_ClearInterruptMask(Dev, 0);
    ERROR_OUT;
    return comm.WrByte( REG_SYSRANGE_START, 0x00);
  } // VL53L0X_perform_single_ref_calibration

  Error Api::ref_calibration_io( uint8_t read_not_write, uint8_t VhvSettings, uint8_t PhaseCal, uint8_t *pVhvSettings, uint8_t *pPhaseCal, const uint8_t vhv_enable, const uint8_t phase_enable) {
    Error Status = ERROR_NONE;


    /* Read VHV from device */
    Status |= comm.WrByte( 0xFF, 0x01);
    Status |= comm.WrByte( 0x00, 0x00);
    Status |= comm.WrByte( 0xFF, 0x00);
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
        Status |= comm.WrByte( 0xCB, VhvSettings);
      }
      if (phase_enable) {
        Status |= VL53L0X_UpdateByte(Dev, 0xEE, 0x80, PhaseCal);
      }
    }

    Status |= comm.WrByte( 0xFF, 0x01);
    Status |= comm.WrByte( 0x00, 0x01);
    Status |= comm.WrByte( 0xFF, 0x00);

    *pPhaseCal = (uint8_t) (PhaseCalint & 0xEF);

    return Status;
  } // VL53L0X_ref_calibration_io

  Error VL53L0X_perform_vhv_calibration( uint8_t *pVhvSettings, const uint8_t get_data_enable, const uint8_t restore_config) {
    /* store the value of the sequence config, this will be reset before the end of the function */
    uint8_t SequenceConfig = restore_config ? PALDevDataGet(this, SequenceConfig) : 0;
    /* Run VHV */
    Error Error = comm.WrByte( REG_SYSTEM_SEQUENCE_CONFIG, 0x01);

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
      Error = comm.WrByte( REG_SYSTEM_SEQUENCE_CONFIG, SequenceConfig);
      if (!Error) {
        PALDevDataSet(Dev, SequenceConfig, SequenceConfig);
      }
    }

    return Error;
  } // VL53L0X_perform_vhv_calibration

  Error VL53L0X_perform_phase_calibration( uint8_t *pPhaseCal, const uint8_t get_data_enable, const uint8_t restore_config) {

    /* store the value of the sequence config, this will be reset before the end of the function */
    uint8_t SequenceConfig = restore_config ? PALDevDataGet(this, SequenceConfig) : 0;

    /* Run PhaseCal */
    Error Status = comm.WrByte( REG_SYSTEM_SEQUENCE_CONFIG, 0x02);

    if (Status == ERROR_NONE) {
      Status = VL53L0X_perform_single_ref_calibration(Dev, 0x0);
    }

    /* Read PhaseCal from device */
    if ((Status == ERROR_NONE) && (get_data_enable == 1)) {
      uint8_t VhvSettingsint;//ick: ignored
      Status = VL53L0X_ref_calibration_io(Dev, 1, 0, 0, &VhvSettingsint, pPhaseCal, 0, 1);
    } else {
      *pPhaseCal = 0;
    }

    if ((Status == ERROR_NONE) && restore_config) {
      /* restore the previous Sequence Config */
      Status = comm.WrByte( REG_SYSTEM_SEQUENCE_CONFIG, SequenceConfig);
      if (Status == ERROR_NONE) {
        PALDevDataSet(Dev, SequenceConfig, SequenceConfig);
      }
    }

    return Status;
  } // VL53L0X_perform_phase_calibration

  Error Api::perform_ref_calibration( uint8_t *pVhvSettings, uint8_t *pPhaseCal, uint8_t get_data_enable) {
    /* store the value of the sequence config,
     * this will be reset before the end of the function
     */
    uint8_t SequenceConfig = PALDevDataGet(this, SequenceConfig);

    /* In the following function we don't save the config to optimize
     * writes on device. Config is saved and restored only once. */
    Error Status = Api::perform_vhv_calibration( pVhvSettings, get_data_enable, 0);

    if (Status == ERROR_NONE) {
      Status = Api::perform_phase_calibration( pPhaseCal, get_data_enable, 0);
    }

    if (Status == ERROR_NONE) {
      /* restore the previous Sequence Config */
      Status = comm.WrByte( REG_SYSTEM_SEQUENCE_CONFIG, SequenceConfig);
      if (Status == ERROR_NONE) {
        PALDevDataSet(this, SequenceConfig, SequenceConfig);
      }
    }

    return Status;
  } // VL53L0X_perform_ref_calibration

  Error VL53L0X_set_ref_calibration( uint8_t VhvSettings, uint8_t PhaseCal) {
    Error Status = ERROR_NONE;
    uint8_t pVhvSettings;//ick: ignored
    uint8_t pPhaseCal;//ick: ignored
    return VL53L0X_ref_calibration_io(Dev, 0, VhvSettings, PhaseCal, &pVhvSettings, &pPhaseCal, 1, 1);
  }

  Error VL53L0X_get_ref_calibration( uint8_t *pVhvSettings, uint8_t *pPhaseCal) {
    return VL53L0X_ref_calibration_io(Dev, 1, 0, 0, pVhvSettings, pPhaseCal, 1, 1);
  }
}
