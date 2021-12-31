/*******************************************************************************
Copyright 2021 by Andy Heilveil, github/980f via near total rewrite of source
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

/**
 * @file VL53L0X_def.h
 *
 * @brief Type definitions for VL53L0X API.
 *
 */

#ifndef _VL53L0X_DEF_H_
#define _VL53L0X_DEF_H_

#include "build.h"  //options for a particular build/localization for a platform

/** @defgroup VL53L0X_globaldefine_group VL53L0X Defines
 *	@brief	  VL53L0X Defines
 *	@{
 */

//there is no real basis for this value:
#define VL53L0X_DEFAULT_MAX_LOOP 200

//ick: the product id is at most 20 chars, where does the 32 come into play?
#define VL53L0X_MAX_STRING_LENGTH 32

#include "vl53l0x_device.h"
#include "vl53l0x_types.h"
#include "vl53l0x_spadarray.h"
#include "bitmanipulators.h"
#include "vl53l0x_error.h"

namespace VL53L0X {

  struct SemverLite {
    uint8_t major;   /*!< Product revision major */
    uint8_t minor;/*!< Product revision minor */
    constexpr bool operator==(SemverLite other) const {
      return major == other.major && minor == other.minor;
    }
  };

  /** @brief Defines the parameters of the Get Version Functions
 */
  struct Version_t {
    SemverLite ver;
    uint8_t build;     /*!< build number */
    uint32_t revision; /*!< revision number */

    /** equality compare omits revision.
     * If you wish to also omit build then compare the ver members.
     * */
    constexpr bool operator==(const Version_t other) const {
      return ver == other.ver && build == other.build;
    }
  };

  using InfoText = const char *;//formerly expensive: char [VL53L0X_MAX_STRING_LENGTH];

/** @brief Defines the parameters of the Get Device Info Functions
 */
  struct DeviceInfo_t {
    InfoText Name;     /*!< Name of the Device e.g. Left_Distance */
    InfoText Type;     /*!< Type of the Device e.g VL53L0X */
    InfoText ProductId;    /*!< Product Identifier String	(//ick:limited to 17+null in places) */
    uint8_t ProductType;     /*!< Product Type, VL53L0X = 1, VL53L1 = 2 */
    struct SemverLite ProductRevision;
  };

/** @defgroup VL53L0X_define_DeviceModes_group Defines Device modes
 *	Defines all possible modes for the device
 *	@{
 */
  enum DeviceModes : uint8_t {
    DEVICEMODE_SINGLE_RANGING = 0
    , DEVICEMODE_CONTINUOUS_RANGING
    , DEVICEMODE_SINGLE_HISTOGRAM
    , DEVICEMODE_CONTINUOUS_TIMED_RANGING

    , DEVICEMODE_SINGLE_ALS = 10
    , DEVICEMODE_GPIO_DRIVE = 20
    , DEVICEMODE_GPIO_OSC

/* ... Modes to be added depending on device */
/** @} VL53L0X_define_DeviceModes_group */
  };

  /** these are paired in many places, having a struct lets us set a universal default  */
  struct GpioConfiguration {
    DeviceModes devMode = DEVICEMODE_GPIO_DRIVE;//looks like two enums are in one space.
    GpioFunctionality function = GPIOFUNCTIONALITY_OFF;
    InterruptPolarity polarity = INTERRUPTPOLARITY_LOW;
  };

  constexpr bool isValid(DeviceModes modes) {//todo: review old code, may need some context here.
    switch (modes) {
      case DEVICEMODE_SINGLE_RANGING:
        return true;
      case DEVICEMODE_CONTINUOUS_RANGING:
        return true;
      case DEVICEMODE_SINGLE_HISTOGRAM:
        return false;//NYI
      case DEVICEMODE_CONTINUOUS_TIMED_RANGING:
        return true;
      case DEVICEMODE_SINGLE_ALS:
        return false; //NYI
      case DEVICEMODE_GPIO_DRIVE:
        return true;
      case DEVICEMODE_GPIO_OSC:
        return true;
      default:
        return false;
    }
  }

#if IncludeHistogramming
/** @defgroup VL53L0X_define_HistogramModes_group Defines Histogram modes
 *	Defines all possible Histogram modes for the device
 *	@{
 */
  enum HistogramModes : uint8_t {
    HISTOGRAMMODE_DISABLED = 0 /*!< Histogram Disabled */
    , HISTOGRAMMODE_REFERENCE_ONLY /*!< Histogram Reference array only */
    , HISTOGRAMMODE_RETURN_ONLY /*!< Histogram Return array only */
    , HISTOGRAMMODE_BOTH  /*!< Histogram both Reference and Return Arrays */

/* ... Modes to be added depending on device */
/** @} VL53L0X_define_HistogramModes_group */
  };

#endif

/** @defgroup VL53L0X_define_PowerModes_group List of available Power Modes
 *	List of available Power Modes
 *	@{
 */

/* 980F: reordered to match value from device, and to get compiler to validate values for anyone who doesn't cast their way around the check */
  enum PowerModes : uint8_t {//two bits: IdleElseStandby and an unused level
    POWERMODE_STANDBY_LEVEL1 = 0 /*!< Standby level 1 */
    , POWERMODE_IDLE_LEVEL1  /*!< Idle level 1 */
    //using #def to get compile time checking on parameters
#if VL53L0X_POWERMODES > 1
    , POWERMODE_STANDBY_LEVEL2 /*!< Standby level 2 */
    , POWERMODE_IDLE_LEVEL2 /*!< Idle level 2 */
#endif
/** @} VL53L0X_define_PowerModes_group */
  };


  using MegaCps =FixPoint1616_t;
  /** a limit might be disabled but still retain its value */
  struct LimitTuple {
    bool enable= false;
    FixPoint<9, 7> value;
  };

/** @brief Defines all parameters for the device
 */
  struct DeviceParameters_t {
    DeviceModes DeviceMode; /*!< Defines type of measurement to be done for the next measure */
#if IncludeHistogramming
    HistogramModes HistogramMode;/*!< Defines type of histogram measurement to be done for the next measure */
#endif
    uint32_t MeasurementTimingBudgetMicroSeconds; /*!< Defines the allowed total time for a single measurement */
    uint32_t InterMeasurementPeriodMilliSeconds; /*!< Defines time between two consecutive measurements (between two	measurement starts). If set to 0 means back-to-back mode */
    bool XTalkCompensationEnable; /*!< Tells if Crosstalk compensation shall be enable or not	 */
    uint16_t XTalkCompensationRangeMilliMeter;  /*!< CrossTalk compensation range in millimeter	 */
    MegaCps XTalkCompensationRateMegaCps; /*!< CrossTalk compensation rate in Mega counts per seconds.	*/
    int32_t RangeOffsetMicroMeters; /*!< Range offset adjustment (mm).	*/

    //todo: struct and single array.
    LimitTuple LimitChecks[CHECKENABLE_NUMBER_OF_CHECKS];     /*!< This Array store all the Limit Check value for this device */
    bool LimitChecksStatus[CHECKENABLE_NUMBER_OF_CHECKS]; /*!< This Array store all the Error of the check linked to last measurement. */

    bool WrapAroundCheckEnable; /*!< Tells if Wrap Around Check shall be enable or not */
  };

/** @defgroup VL53L0X_define_State_group Defines the current status of the
 *device Defines the current status of the device
 *	@{
 */

  enum State : uint8_t {
    STATE_POWERDOWN = 0 /*!< Device is in HW reset	*/
    , STATE_WAIT_STATICINIT /*!< Device is initialized and wait for static initialization  */
    , STATE_STANDBY /*!< Device is in Low power Standby mode   */
    , STATE_IDLE /*!< Device has been initialized and ready to do measurements  */
    , STATE_RUNNING /*!< Device is performing measurement */
    , STATE_UNKNOWN = 98   /*!< Device is in unknown state and need to be rebooted	 */
    , STATE_ERROR /*!< Device is in error state and need to be rebooted  */

/** @} VL53L0X_define_State_group */
  };

/** @brief Structure containing the Dmax computation parameters and data
 */
  struct DMaxData_t {
    int32_t AmbTuningWindowFactor_K;    /*!<  internal algo tuning (*1000) */
    int32_t RetSignalAt0mm;    /*!< intermediate dmax computation value caching */
  };

/**
 * @struct VL53L0X_RangeData_t
 * @brief Range measurement data.
 */
  struct RangingMeasurementData_t {
    uint32_t TimeStamp; /*!< 32-bit time stamp. */ //todo: actually set a timestamp value into this!
    uint32_t MeasurementTimeUsec;     /*!< Give the Measurement time needed by the device to do the measurement.*/

    uint16_t RangeMilliMeter; /*!< range distance in millimeter. */

    uint16_t RangeDMaxMilliMeter;  /*!< Tells what is the maximum detection distance of the device in current setup and environment conditions (Filled when applicable) */

    MegaCps SignalRateRtnMegaCps; /*!< Return signal rate (MCPS)\n these is a 16.16 fix point value, which is effectively a measure of target reflectance.*/
    MegaCps AmbientRateRtnMegaCps;  /*!< Return ambient rate (MCPS)\n these is a 16.16 fix point value, which is effectively a measure of the ambient light.*/

    FixPoint<8, 8> EffectiveSpadRtnCount;  /*!< Return the effective SPAD count for the return signal. To obtain Real value it should be divided by 256  (call .rounded()) */

    uint8_t ZoneId;  /*!< Denotes which zone and range scheduler stage the range data relates to. */

    uint8_t RangeFractionalPart; /*!< Fractional part of range distance. Final value is a FixPoint168 value. */
    RangeStatus rangeError;  /*!< Range Error for the current measurement. This is device dependent. Value = 0 means value is valid. 	See \ref RangeStatusPage */
  };

#if IncludeHistogramming
#define VL53L0X_HISTOGRAM_BUFFER_SIZE 24

/**
 * @struct VL53L0X_HistogramData_t
 * @brief Histogram measurement data.
 */
  struct HistogramMeasurementData_t {  /* Histogram Measurement data */
    uint32_t HistogramData[VL53L0X_HISTOGRAM_BUFFER_SIZE];  /*!< Histogram data */
    uint8_t HistogramType; /*!< Indicate the types of histogram data :  Return only, Reference only, both Return and Reference */
    uint8_t FirstBin;      /*!< First Bin value */
    uint8_t BufferSize;    /*!< Buffer Size - Set by the user.*/
    uint8_t NumberOfBins;  /*!< Number of bins filled by the histogram measurement */

    DeviceError ErrorStatus;  /*!< Error status of the current measurement. \n
    see @a ::VL53L0X_DeviceError @a VL53L0X_GetStatusErrorString() */
  };
#endif

  /**
 * @struct VL53L0X_SpadData_t
 * @brief Spad Configuration Data.
 */
  struct SpadData_t {
    SpadArray enables;
    SpadArray goodones;
  };

  struct SigmaEstimates {
    uint16_t RefArray = 0; /*!< Reference array sigma value in 1/100th of [mm] e.g. 100 = 1mm */
    uint16_t EffPulseWidth = 0;  /*!< Effective Pulse width for sigma estimate in 1/100th of ns e.g. 900 = 9.0ns */
    uint16_t EffAmbWidth = 0;     /*!< Effective Ambient width for sigma estimate in 1/100th of ns e.g. 500 = 5.0ns */
  };

  struct DeviceSpecificParameters_t {
    FixPoint1616_t OscFrequencyMHz; /* Frequency used */

    uint16_t LastEncodedTimeout = 0;/* last encoded Time out used for timing budget*/

    GpioFunctionality Pin0GpioFunctionality = GPIOFUNCTIONALITY_OFF;/* store the functionality of the GPIO: pin0 */

    struct RangeSetting {
      uint32_t TimeoutMicroSecs = 0;/*!< Execution time of the final range*/
      uint8_t VcselPulsePeriod = 0;/*!< Vcsel pulse period (pll clocks) for the final range measurement*/
    };
    RangeSetting FinalRange;/*!< Execution &Vcsel time of the final range*/
    RangeSetting PreRange;/*!< Execution time of the pre-range range (ST had rogue tile in this comment) */

    SigmaEstimates SigmaEst;

    uint8_t ReadDataFromDeviceDone = 0; /* 3bits for subsets of device data having been read */

    uint8_t ModuleId = 0;               /* Module ID */
    uint8_t Revision = 0;               /* test Revision */
    char ProductId[VL53L0X_MAX_STRING_LENGTH] = "";/* Product Identifier String  */

    SpadCount ReferenceSpad;
    bool RefSpadsInitialised = false; /* reports if ref spads are initialised. */

    struct PartUID_t {
      uint32_t Upper = 0;       /*!< Unique Part ID Upper */
      uint32_t Lower = 0;       /*!< Unique Part ID Lower */
    } PartUID;
    MegaCps SignalRateMeasFixed400mm; /*!< Peek Signal rate at 400 mm*/
  };

  using Tunings = const uint8_t *;

  const uint16_t UnityGain = 1000;//used with LinearityCorrectiveGain

  /** @returns whether @param gain is a legal value for the hardware.
   * By using unsigned type we don't have to check signed values for being positive, the bounds check in here will throw those out.
   * If the compiler warns you about narrowing conversion etc then you need to add an explicit check in your code before calling this.
   * */
  constexpr bool validGain(uint16_t gain) {
    return gain <= UnityGain;
  }

  /**
 * @struct VL53L0X_DevData_t
 *
 * @brief VL53L0X PAL device ST private data structure \n
 * End user should never access any of these field directly
 *
 * These must never access directly but only via macro
 */
  struct DevData_t {
//not used, appears to be for debug of some calculation     DMaxData_t DMaxData;     /*!< Dmax Data */
    int32_t Part2PartOffsetNVMMicroMeter;     /*!< backed up NVM value */
    int32_t Part2PartOffsetAdjustmentNVMMicroMeter;     /*!< backed up NVM value representing additional offset adjustment */
    DeviceParameters_t CurrentParameters;     /*!< Current Device Parameter */
    RangingMeasurementData_t LastRangeMeasure;     /*!< Ranging Data */
#if IncludeHistogramming
    HistogramMeasurementData_t LastHistogramMeasure;     /*!< Histogram Data */
#endif
    DeviceSpecificParameters_t DeviceSpecificParameters;     /*!< Parameters specific to the device */
    SpadData_t SpadData;    /*!< Spad Data */
    uint8_t SequenceConfig;    /*!< Internal value for the sequence config */
    bool RangeFractionalEnable;    /*!< Enable/Disable fractional part of ranging data */
    State PalState;    /*!< Current state of the PAL for this device */
    PowerModes PowerMode;/*!< Current Power Mode	 */

    SigmaEstimates SigmaEst;

    uint8_t StopVariable;     /*!< StopVariable used during the stop sequence */
    FixPoint<9, 7> targetRefRate;     /*!< Target Ambient Rate for Ref spad management */
    FixPoint1616_t SigmaEstimate;     /*!< Sigma Estimate - based on ambient & VCSEL rates and signal_total_events */
    FixPoint1616_t SignalEstimate;     /*!< Signal Estimate - based on ambient & VCSEL rates and cross talk */
    MegaCps LastSignalRefMcps;     /*!< Latest Signal ref in Mcps */
    Tunings pTuningSettingsPointer;     /*!< Pointer for Tuning Settings table */
    bool UseInternalTuningSettings;     /*!< Indicate if we use	 Tuning Settings table */
    uint16_t LinearityCorrectiveGain;     /*!< Linearity Corrective Gain value in x1000 */
    bool unityGain() const {
      return LinearityCorrectiveGain == UnityGain;
    }

    struct DmaxCal {
      uint16_t RangeMilliMeter = 0;     /*!< Dmax Calibration Range millimeter */
      MegaCps SignalRateRtnMegaCps;     /*!< Dmax Calibration Signal Rate Return MegaCps */
    } dmaxCal;
  };



/** @} VL53L0X_define_InterruptPolarity_group */

/** @defgroup VL53L0X_define_VcselPeriod_group Vcsel Period Defines
 *	Defines the range measurement for which to access the vcsel period.
 *	@{
 */
  enum VcselPeriod : uint8_t {
    VCSEL_PERIOD_PRE_RANGE = 0 /*!<Identifies the pre-range vcsel period. */
    , VCSEL_PERIOD_FINAL_RANGE /*!<Identifies the final range vcsel period. */
/** @} VL53L0X_define_VcselPeriod_group */
  };

/** @} VL53L0X_define_SchedulerSequence_group */

/** @defgroup VL53L0X_define_SequenceStepId_group Defines the Polarity
 *	of the Interrupt
 *	Defines the the sequence steps performed during ranging..
 *	@{
 */
  enum SequenceStepId : uint8_t { //formerly cast to (VL53L0X_VcselPeriod)
    SEQUENCESTEP_TCC = 0 /*!<Target CentreCheck identifier. */
    , SEQUENCESTEP_DSS /*!<Dynamic Spad Selection function Identifier. */
    , SEQUENCESTEP_MSRC /*!<Minimum Signal Rate Check function Identifier. */
    , SEQUENCESTEP_PRE_RANGE /*!<Pre-Range check Identifier. */
    , SEQUENCESTEP_FINAL_RANGE /*!<Final Range Check Identifier. */

    , SEQUENCESTEP_NUMBER_OF_CHECKS /*!<Number of Sequence Step Managed by the API. */
  };

  /** @returns the bit within the packed SequenceStep control byte for the given @param sid  choice */
  constexpr unsigned bitFor(SequenceStepId sid) {
    switch (sid) {
      case SEQUENCESTEP_TCC:
        return 4;
      case SEQUENCESTEP_DSS:
        return 3;
      case SEQUENCESTEP_MSRC:
        return 2;
      case SEQUENCESTEP_PRE_RANGE:
        return 6;
        break;
      case SEQUENCESTEP_FINAL_RANGE:
        return 7;  //ick: this seems to overlap with wraparound enable
      default:
//no route        THROW(ERROR_INVALID_PARAMS);
        return ~0;//todo: deal with lack of access to throw
    } // switch
  }

/** @defgroup VL53L0X_define_SchedulerSequence_group Defines the steps
 * carried out by the scheduler during a range measurement.
 *	@{
 *	Defines the states of all the steps in the scheduler
 *	i.e. enabled/disabled.
 *
 *	this would be a lot easier to manage as bit fields
 *
 */
  struct SchedulerSequenceSteps_t {
    bool TccOn = false;       /*!<Reports if Target Centre Check On  */
    bool MsrcOn = false;       /*!<Reports if MSRC On  */
    bool DssOn = false;        /*!<Reports if DSS On  */
    bool PreRangeOn = false;   /*!<Reports if Pre-Range On	*/
    bool FinalRangeOn = false; /*!<Reports if Final-Range On  */

    /** @returns whether one of the three that share a timeout are set */
    bool anyOfMsrcDccTcc() const {
      return TccOn || MsrcOn || DssOn;
    }

    /** SequenceConfig byte bit assignments*/
    SchedulerSequenceSteps_t &unpack(uint8_t devicebyte) {
      TccOn = getBit<bitFor(SEQUENCESTEP_TCC)>(devicebyte);
      DssOn = getBit<bitFor(SEQUENCESTEP_DSS)>(devicebyte);
      MsrcOn = getBit<bitFor(SEQUENCESTEP_MSRC)>(devicebyte);
      PreRangeOn = getBit<bitFor(SEQUENCESTEP_PRE_RANGE)>(devicebyte);
      FinalRangeOn = getBit<bitFor(SEQUENCESTEP_FINAL_RANGE)>(devicebyte);
      return *this;
    }

    SchedulerSequenceSteps_t(uint8_t devicebyte) {
      unpack(devicebyte);
    }
  };

/** @} VL53L0X_define_SequenceStepId_group */

/* MACRO Definitions */
/** @defgroup VL53L0X_define_GeneralMacro_group General Macro Defines
 *	General Macro Defines
 *	@{
 */

/* Defines */
#define VL53L0X_SETPARAMETERFIELD(field, value) PALDevDataSet( CurrentParameters.field, value)

#define VL53L0X_GETPARAMETERFIELD(field)  PALDevDataGet( CurrentParameters).field

#define VL53L0X_SETARRAYPARAMETERFIELD(field, index, value)   PALDevDataSet( CurrentParameters.field[index], value)

#define VL53L0X_GETARRAYPARAMETERFIELD(field, index)  PALDevDataGet( CurrentParameters).field[index]

#define VL53L0X_SETDEVICESPECIFICPARAMETER(field, value) PALDevDataSet( DeviceSpecificParameters.field, value)

#define VL53L0X_GETDEVICESPECIFICPARAMETER(field)  PALDevDataGet( DeviceSpecificParameters).field

////todo: remove all these via FixPoint types
//#define VL53L0X_FIXPOINT1616TOFIXPOINT97(Value)  (uint16_t)((Value >> 9) & 0xFFFF)
////BUG: (UB) need to cast/convert before shrink, not after
//#define VL53L0X_FIXPOINT97TOFIXPOINT1616(Value) (FixPoint1616_t)((Value) << 9)
//
//#define VL53L0X_FIXPOINT1616TOFIXPOINT88(Value)  (uint16_t)((Value >> 8) & 0xFFFF)
////BUG: (UB) need to cast/convert before shrink, not after
//#define VL53L0X_FIXPOINT88TOFIXPOINT1616(Value) (FixPoint1616_t)(Value << 8)
//
//#define VL53L0X_FIXPOINT1616TOFIXPOINT412(Value) (uint16_t)((Value >> 4) & 0xFFFF)
//#define VL53L0X_FIXPOINT412TOFIXPOINT1616(Value) (FixPoint1616_t)(Value << 4)
//
//#define VL53L0X_FIXPOINT1616TOFIXPOINT313(Value) (FixPoint<3,13>(Value))
////BUG: (UB) need to cast/convert before shrink, not after
//#define VL53L0X_FIXPOINT313TOFIXPOINT1616(Value) (FixPoint1616_t)(Value << 3)
//
//#define VL53L0X_FIXPOINT1616TOFIXPOINT08(Value) (uint8_t)((Value >> 8) & 0x00FF)
////BUG: (UB) need to cast/convert before shrink, not after
//#define VL53L0X_FIXPOINT08TOFIXPOINT1616(Value) (FixPoint1616_t)(Value << 8)
//
//#define VL53L0X_FIXPOINT1616TOFIXPOINT53(Value)  (uint8_t)((Value >> 13) & 0x00FF)
////BUG: (UB) need to cast/convert before shrink, not after
//#define VL53L0X_FIXPOINT53TOFIXPOINT1616(Value) (FixPoint1616_t)(Value << 13)
//
////BUG: rogue tile, shrink should be 12 not 14.
//#define VL53L0X_FIXPOINT1616TOFIXPOINT102(Value)  (uint16_t)((Value >> 14) & 0x0FFF)
//#define VL53L0X_FIXPOINT102TOFIXPOINT1616(Value) (FixPoint1616_t)(Value << 12)

  constexpr uint16_t MAKEUINT16(uint8_t lsb, uint8_t msb) {
    return (uint16_t(msb) << 8) + uint16_t(lsb);
  }

/** @} VL53L0X_define_GeneralMacro_group */

/** @} VL53L0X_globaldefine_group */
}; //end namespace
#endif /* _VL53L0X_DEF_H_ */
