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

  //offset field is 12 bits and signed.
  const int16_t cOffsetMask = (1 << 12)-1; //bit field 11=>0
  const int16_t cOffsetMax = cOffsetMask>>1;//max positive value of 12 bits
  const int16_t cOffsetMin = -(cOffsetMask+1);//most negative value


  Error Api::perform_xtalk_calibration(FixPoint1616_t XTalkCalDistance, FixPoint1616_t &pXTalkCompensationRateMegaCps) {
    if (XTalkCalDistance.raw <= 0) {//ICK: type was unsigned, and so this is a compare to zero.
      return ERROR_INVALID_PARAMS;
    }
    /* Disable the XTalk compensation */
    ErrorAccumulator Error = SetXTalkCompensationEnable( 0);
    ERROR_OUT;

    /* Disable the RIT */
    Error = SetLimitCheckEnable( CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);
    ERROR_OUT;

    /* Perform 50 measurements and compute the averages */
    unsigned sum_ranging = 0;//bug: former use of 16 bit begs for integer overflow
    unsigned sum_spads = 0;//... which we will tolerate only on processors whose natural int is 16 bits
    FixPoint1616_t sum_signalRate = 0;
    uint32_t total_count = 0;//unsigned is probably adequate
    for (uint8_t xtalk_meas = 0; xtalk_meas < 50; xtalk_meas++) {//ick: buried constant
      RangingMeasurementData_t RangingMeasurementData;
      Error =PerformSingleRangingMeasurement( &RangingMeasurementData);

      if (Error != ERROR_NONE) {
        return total_count == 0 ? ERROR_RANGE_ERROR : Error.sum;//ick: could use some refinement why error is overridden if first measurement fails for any reason.
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
      return ERROR_RANGE_ERROR;
    }

    /* FixPoint1616_t / uint16_t = FixPoint1616_t */
    FixPoint1616_t xTalkStoredMeanSignalRate ( sum_signalRate,total_count,0);//ick: was not rounded prior to 980f
    FixPoint1616_t xTalkStoredMeanRange (  sum_ranging, total_count);//round the division
    FixPoint1616_t xTalkStoredMeanRtnSpads (sum_spads, total_count);

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
    uint32_t xTalkStoredMeanRtnSpadsAsInt = roundedScale(xTalkStoredMeanRtnSpads , 16);

    /* Round Cal Distance to Whole Number.
     * Note that the cal distance is in mm, therefore no resolution
     * is lost.*/
    uint32_t xTalkCalDistanceAsInt = roundedScale(XTalkCalDistance, 16);
    FixPoint1616_t XTalkCompensationRateMegaCps;

    if (xTalkStoredMeanRtnSpadsAsInt == 0 || xTalkCalDistanceAsInt == 0 || xTalkStoredMeanRange >= XTalkCalDistance) {
      XTalkCompensationRateMegaCps = 0.0F;
    } else {
      /* Round Cal Distance to Whole Number.
       *  Note that the cal distance is in mm, therefore no
       *  resolution is lost.*/
      xTalkCalDistanceAsInt = roundedScale(XTalkCalDistance , 16);

      /* Apply division by mean spad count early in the
       * calculation to keep the numbers small.
       * This ensures we can maintain a 32bit calculation.
       * Fixed1616 / int := Fixed1616 */
      uint32_t signalXTalkTotalPerSpad = (xTalkStoredMeanSignalRate) / xTalkStoredMeanRtnSpadsAsInt;//ick: round divide

      /* Complete the calculation for total Signal XTalk per
       * SPAD
       * Fixed1616 * (Fixed1616 - Fixed1616/int) :=
       * (2^16 * Fixed1616)
       */
      signalXTalkTotalPerSpad *= (Unity.raw - (xTalkStoredMeanRange / xTalkCalDistanceAsInt));//ick: round divide

      /* Round from 2^16 * Fixed1616, to Fixed1616. */
      XTalkCompensationRateMegaCps = roundedScale(signalXTalkTotalPerSpad, 16);
    }

    pXTalkCompensationRateMegaCps = XTalkCompensationRateMegaCps;
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
    ErrorAccumulator Error = SetOffsetCalibrationDataMicroMeter( 0);
    ERROR_OUT;
    /* Get the value of the TCC */
    bool SequenceStepEnabled= GetSequenceStepEnable( SEQUENCESTEP_TCC);
    ERROR_OUT;
    /* Disable the TCC */
    Error = SetSequenceStepEnable( SEQUENCESTEP_TCC, 0);
    ERROR_OUT;
    /* Disable the RIT */
    Error = SetLimitCheckEnable( CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);
    ERROR_OUT;
    /* Perform 50 measurements and compute the averages */
    uint16_t sum_ranging = 0;
    uint32_t total_count = 0;
    for (int meas = 0; meas < 50; meas++) {
      RangingMeasurementData_t RangingMeasurementData;
      Error = PerformSingleRangingMeasurement( &RangingMeasurementData);
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
    FixPoint1616_t StoredMeanRange  (sum_ranging,total_count);
    uint32_t StoredMeanRangeAsInt = StoredMeanRange.shrink(16);

    /* Round Cal Distance to Whole Number.
     * Note that the cal distance is in mm, therefore no resolution
     * is lost.*/
    uint32_t CalDistanceAsInt_mm = CalDistanceMilliMeter.shrink(16);
    *pOffsetMicroMeter = (CalDistanceAsInt_mm - StoredMeanRangeAsInt) * 1000;

    /* Apply the calculated offset */
    VL53L0X_SETPARAMETERFIELD(  RangeOffsetMicroMeters, *pOffsetMicroMeter);
    Error = SetOffsetCalibrationDataMicroMeter( *pOffsetMicroMeter);
    ERROR_OUT;

    /* Restore the TCC */
    if (SequenceStepEnabled != 0) {
      return SetSequenceStepEnable( SEQUENCESTEP_TCC, 1);
    }
    return ERROR_NONE;
  } // VL53L0X_perform_offset_calibration

  Error Api::set_offset_calibration_data_micro_meter( int32_t OffsetCalibrationDataMicroMeter) {
//BUG:    const int32_t cMaxOffsetMicroMeter = 511000;//effectively 2044 rather than 2047 for max positive value.
    LOG_FUNCTION_START;
    /* The offset register is 10.2 format and units are mm
     * therefore conversion is applied by a division of
     * 250.  (1000/4)
     */
    OffsetCalibrationDataMicroMeter/=250; //divide by 1000 and multiply by 4
    if(OffsetCalibrationDataMicroMeter > cOffsetMax) {
      OffsetCalibrationDataMicroMeter = cOffsetMax;
    } else if (OffsetCalibrationDataMicroMeter < cOffsetMin) {
      OffsetCalibrationDataMicroMeter = cOffsetMin;
    }

    return  comm.WrWord( REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM, OffsetCalibrationDataMicroMeter&cOffsetMask);
  } // VL53L0X_set_offset_calibration_data_micro_meter

  Erroneous<int32_t > Api::get_offset_calibration_data_micro_meter( ) {
    Error Error = ERROR_NONE;
    Erroneous<uint16_t> RangeOffsetRegister;
    if(fetch(RangeOffsetRegister, REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM)){
//      int32_t fluffer=;//signed in 12 lsbs, need to sign extend
      /* Apply 12 bit 2's compliment conversion */
      bool isNegative=getBit<11>(RangeOffsetRegister.wrapped);
      if(isNegative){
        RangeOffsetRegister|= -(1<<12);
      }
      return  RangeOffsetRegister*250;
    } else {
      return {RangeOffsetRegister.error};
    }
  } // VL53L0X_get_offset_calibration_data_micro_meter

  Error Api::apply_offset_adjustment() {
    /* if we run on this function we can read all the NVM info used by the API */
    ErrorAccumulator Error = get_info_from_device( 7);
    ERROR_OUT;
    /* Read back current device offset */
    Erroneous<int32_t> CurrentOffsetMicroMeters = GetOffsetCalibrationDataMicroMeter();
    ERROR_ON(CurrentOffsetMicroMeters);
    /* Apply Offset Adjustment derived from 400mm measurements */
    /* Store initial device offset */
    PALDevDataSet( Part2PartOffsetNVMMicroMeter, CurrentOffsetMicroMeters);
    int32_t CorrectedOffsetMicroMeters = CurrentOffsetMicroMeters + (int32_t) PALDevDataGet(  Part2PartOffsetAdjustmentNVMMicroMeter);

    Error = SetOffsetCalibrationDataMicroMeter( CorrectedOffsetMicroMeters);
    ERROR_OUT;
    /* store current, adjusted offset */
    VL53L0X_SETPARAMETERFIELD( RangeOffsetMicroMeters, CorrectedOffsetMicroMeters);

    return ERROR_NONE;
  } // VL53L0X_apply_offset_adjustment

  SpadArray::Index Api::get_next_good_spad(SpadArray goodSpadArray,  SpadArray::Index curr) {
    for(;curr.isValid();++curr) {
      if(goodSpadArray.get(curr)) {
        return curr;
      }
    }
    return ~0;//canonical ! isValid()
  } // get_next_good_spad

  bool is_aperture(unsigned int spadIndex) {
    /*
     * This function reports if a given spad index is an aperture SPAD by
     * deriving the quadrant.
     */
    return refArrayQuadrants[spadIndex >> 6] != REF_ARRAY_SPAD_0;
  }




  Error Api::enable_ref_spads( bool apertureSpads, SpadArray goodSpadArray, SpadArray spadArray,  unsigned start, unsigned offset, unsigned spadCount, unsigned *lastSpad) {
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
    unsigned  currentSpad = offset;
    for (uint32_t index = 0; index < spadCount; index++) {
      unsigned nextGoodSpad= get_next_good_spad(goodSpadArray,  currentSpad);
      if (nextGoodSpad == ~0) {
        return ERROR_REF_SPAD_INIT;
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
      currentSpad = nextGoodSpad;
      enable_spad_bit(spadArray, currentSpad);
      ++currentSpad;
    }

    *lastSpad = currentSpad;
    Error = set_ref_spad_map( spadArray);

    SpadArray checkSpadArray;
    Error = get_ref_spad_map( checkSpadArray);
    /* Compare spad maps. If not equal report error. */
   if( spadArray != checkSpadArray) {
        return ERROR_REF_SPAD_INIT;
    }

    return ERROR_NONE;
  } // enable_ref_spads

  Erroneous<uint16_t> Api::perform_ref_signal_measurement( ) {
    /* store the value of the sequence config,
     * this will be reset before the end of the function
     */
    uint8_t SequenceConfig = PALDevDataGet(  SequenceConfig);
    /*
     * This function performs a reference signal rate measurement.
     */
    ErrorAccumulator Error = set_SequenceConfig(0xC0, true);//908f: now also sets PALDevData
    ERROR_OUT;

    RangingMeasurementData_t rangingMeasurementData;
    Error = PerformSingleRangingMeasurement( &rangingMeasurementData);
    //ick: object ignored above, will fetch result from registers, that apparently isn't part of RMD_t
    ERROR_OUT; //BUG: on any error other than the first the  REG_SYSTEM_SEQUENCE_CONFIG is not restored!

    auto refSignalRate =FFread<uint16_t>(REG_RESULT_PEAK_SIGNAL_RATE_REF);
    EXIT_ON(refSignalRate);
    /* restore the previous Sequence Config */
    Error= set_SequenceConfig(SequenceConfig, true);
    ERROR_OUT;//loses refSignalRate which might be fine.

    return refSignalRate;
  } // perform_ref_signal_measurement


  Error Api::set_reference_spads( uint32_t count, uint8_t isApertureSpads) {
    //code moved here as it executed regardless of error in the I2C writes
      Data.SpadData.RefSpadEnables.clear();

    unsigned currentSpadIndex = 0;
    if (isApertureSpads) {
      /* Increment to the first APERTURE spad */
      while ((is_aperture(startSelect + currentSpadIndex) == 0) && (currentSpadIndex < SpadArray::MaxCount)) {
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

    Erroneous<unsigned > ignoredvalue= enable_ref_spads( isApertureSpads, Data.SpadData.RefGoodSpadMap,Data.SpadData.RefSpadEnables, currentSpadIndex, count, &lastSpadIndex);
    ERROR_ON(ignoredvalue);
    VL53L0X_SETDEVICESPECIFICPARAMETER( RefSpadsInitialised, true);
    VL53L0X_SETDEVICESPECIFICPARAMETER( ReferenceSpadCount, count);
    VL53L0X_SETDEVICESPECIFICPARAMETER( ReferenceSpadType, isApertureSpads);
    return ERROR_NONE;
  } // VL53L0X_set_reference_spads

  Error Api::get_reference_spads( uint32_t *pSpadCount, uint8_t *pIsApertureSpads) {
    Error Error = ERROR_NONE;


    uint32_t spadsEnabled;

    bool refSpadsInitialised = VL53L0X_GETDEVICESPECIFICPARAMETER(  RefSpadsInitialised);

    if (refSpadsInitialised) {
      *pSpadCount = (uint32_t) VL53L0X_GETDEVICESPECIFICPARAMETER(  ReferenceSpadCount);
      *pIsApertureSpads = VL53L0X_GETDEVICESPECIFICPARAMETER(  ReferenceSpadType);
    } else {
      /* obtain spad info from device.*/

      SpadArray refSpadArray;
      Error = get_ref_spad_map( refSpadArray);

      if (Error == ERROR_NONE) {
        /* count enabled spads within spad map array and
         * determine if Aperture or Non-Aperture.
         */

        bool isApertureSpads = false;
        spadsEnabled = refSpadArray.count_enabled(  &isApertureSpads);


          *pSpadCount = spadsEnabled;
          *pIsApertureSpads = isApertureSpads;
          VL53L0X_SETDEVICESPECIFICPARAMETER( RefSpadsInitialised, 1);
          VL53L0X_SETDEVICESPECIFICPARAMETER( ReferenceSpadCount, (uint8_t) spadsEnabled);
          VL53L0X_SETDEVICESPECIFICPARAMETER( ReferenceSpadType, isApertureSpads);

      }
    }

    return Error;
  } // VL53L0X_get_reference_spads

  Error Api::perform_single_ref_calibration( uint8_t vhv_init_byte) {
    ErrorAccumulator Error = comm.WrByte( REG_SYSRANGE_START, REG_SYSRANGE_MODE_START_STOP | vhv_init_byte);
    ERROR_OUT;
    Error |= measurement_poll_for_completion();
    ERROR_OUT;
    Error = ClearInterruptMask( 0);
    ERROR_OUT;
    return comm.WrByte( REG_SYSRANGE_START, 0x00);
  } // VL53L0X_perform_single_ref_calibration

  Error Api::ref_calibration_io( bool read_else_write, uint8_t VhvSettings, uint8_t PhaseCal, uint8_t *pVhvSettings, uint8_t *pPhaseCal, const uint8_t vhv_enable, const uint8_t phase_enable) {
    ErrorAccumulator Error = ERROR_NONE;


    /* Read VHV from device */
    Error |= comm.WrByte(0xFF, 0x01);
    Error |= comm.WrByte(0x00, 0x00);
    Error |= comm.WrByte(0xFF, 0x00);
    uint8_t PhaseCalint = 0;
    if (read_not_write) {
      if (vhv_enable) {
        Error |= comm.RdByte(0xCB, pVhvSettings);
      }
      if (phase_enable) {
        Error |= comm.RdByte(0xEE, &PhaseCalint);
      }
    } else {
      if (vhv_enable) {
        Error |= comm.WrByte(0xCB, VhvSettings);
      }
      if (phase_enable) {
        Error |= comm.UpdateByte(0xEE, 0x80, PhaseCal);
      }
    }

    Error |= comm.WrByte(0xFF, 0x01);
    Error |= comm.WrByte(0x00, 0x01);
    Error |= comm.WrByte(0xFF, 0x00);

    *pPhaseCal = (uint8_t) (PhaseCalint & 0xEF);

    return Error;
  } // VL53L0X_ref_calibration_io

  Error Api::perform_vhv_calibration( uint8_t *pVhvSettings, const bool get_data_enable, const bool restore_config) {
    /* store the value of the sequence config, this will be reset before the end of the function */
    uint8_t SequenceConfig = restore_config ? PALDevDataGet(  SequenceConfig) : 0;
    /* Run VHV */
    ErrorAccumulator Error = comm.WrByte( REG_SYSTEM_SEQUENCE_CONFIG, 0x01);
    ERROR_OUT;

      Error = perform_single_ref_calibration( 1<<6);
    ERROR_OUT;


    /* Read VHV from device */
    if (get_data_enable ) {
      uint8_t PhaseCalInt = 0;//ick: read and ignored
      uint8_t PhaseCal = 0;//ick: read and ignored
      Error = ref_calibration_io( 1, 0, PhaseCal /* Not used here */, pVhvSettings, &PhaseCalInt, 1, 0);
      ERROR_OUT;
    } else {
      *pVhvSettings = 0;
    }

    if ( restore_config) {
      /* restore the previous Sequence Config */
      Error = set_SequenceConfig(SequenceConfig, true);
    }

    return Error;
  } // VL53L0X_perform_vhv_calibration

  Error Api::perform_phase_calibration( uint8_t *pPhaseCal, const uint8_t get_data_enable, const uint8_t restore_config) {

    /* store the value of the sequence config, this will be reset before the end of the function */
    uint8_t SequenceConfig = restore_config ? PALDevDataGet( SequenceConfig) : 0;

    /* Run PhaseCal */
    Error Error = set_SequenceConfig(0x02, true);//980f: now updates PALDevData

    if (Error == ERROR_NONE) {
      Error = perform_single_ref_calibration( 0x0);
    }

    /* Read PhaseCal from device */
    if ((Error == ERROR_NONE) && (get_data_enable == 1)) {
      uint8_t VhvSettingsint;//ick: ignored
      Error = ref_calibration_io( 1, 0, 0, &VhvSettingsint, pPhaseCal, 0, 1);
    } else {
      *pPhaseCal = 0;
    }

    if ((Error == ERROR_NONE) && restore_config) {
      /* restore the previous Sequence Config */
      Error = set_SequenceConfig(SequenceConfig, true);
    }

    return Error;
  } // VL53L0X_perform_phase_calibration

  Error Api::perform_ref_calibration( uint8_t *pVhvSettings, uint8_t *pPhaseCal, uint8_t get_data_enable) {
    /* store the value of the sequence config,
     * this will be reset before the end of the function
     */
    uint8_t SequenceConfig = PALDevDataGet( SequenceConfig);

    /* In the following function we don't save the config to optimize
     * writes on device. Config is saved and restored only once. */
    Error Error = perform_vhv_calibration( pVhvSettings, get_data_enable, 0);

    if (Error == ERROR_NONE) {
      Error = Api::perform_phase_calibration( pPhaseCal, get_data_enable, 0);
    }

    if (Error == ERROR_NONE) {
      /* restore the previous Sequence Config */
      Error = comm.WrByte( REG_SYSTEM_SEQUENCE_CONFIG, SequenceConfig);
      if (Error == ERROR_NONE) {
        PALDevDataSet(SequenceConfig, SequenceConfig);
      }
    }

    return Error;
  } // VL53L0X_perform_ref_calibration

  Error Api::set_ref_calibration( uint8_t VhvSettings, uint8_t PhaseCal) {
    uint8_t pVhvSettings;//ick: ignored
    uint8_t pPhaseCal;//ick: ignored
    return ref_calibration_io( 0, VhvSettings, PhaseCal, &pVhvSettings, &pPhaseCal, 1, 1);
  }

  Error Api::get_ref_calibration( uint8_t *pVhvSettings, uint8_t *pPhaseCal) {
    return ref_calibration_io( 1, 0, 0, pVhvSettings, pPhaseCal, 1, 1);
  }
}
