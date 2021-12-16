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

/** @defgroup VL53L0X_globaldefine_group VL53L0X Defines
 *	@brief	  VL53L0X Defines
 *	@{
 */

//todo:0 hide these values via const objects that the compiler can use to trim code versus forcing the trim with #ifdefs
//... that loses the ability to alter data structures per version of API so maybe we won't.

/** PAL SPECIFICATION major version */
#define VL53L0X10_SPECIFICATION_VER_MAJOR 1
/** PAL SPECIFICATION minor version */
#define VL53L0X10_SPECIFICATION_VER_MINOR 2
/** PAL SPECIFICATION sub version */
#define VL53L0X10_SPECIFICATION_VER_SUB 7
/** PAL SPECIFICATION sub version */
#define VL53L0X10_SPECIFICATION_VER_REVISION 1440

/** VL53L0X PAL IMPLEMENTATION major version */
#define VL53L0X10_IMPLEMENTATION_VER_MAJOR 1
/** VL53L0X PAL IMPLEMENTATION minor version */
#define VL53L0X10_IMPLEMENTATION_VER_MINOR 0
/** VL53L0X PAL IMPLEMENTATION sub version */
#define VL53L0X10_IMPLEMENTATION_VER_SUB 9
/** VL53L0X PAL IMPLEMENTATION sub version */
#define VL53L0X10_IMPLEMENTATION_VER_REVISION 3673

//ick: are teh above and below really two sets of different versioning or just badly maintained aliases?

/** PAL SPECIFICATION major version */
#define VL53L0X_SPECIFICATION_VER_MAJOR 1
/** PAL SPECIFICATION minor version */
#define VL53L0X_SPECIFICATION_VER_MINOR 2
/** PAL SPECIFICATION sub version */
#define VL53L0X_SPECIFICATION_VER_SUB 7
/** PAL SPECIFICATION sub version */
#define VL53L0X_SPECIFICATION_VER_REVISION 1440

/** VL53L0X PAL IMPLEMENTATION major version */
#define VL53L0X_IMPLEMENTATION_VER_MAJOR 1
/** VL53L0X PAL IMPLEMENTATION minor version */
#define VL53L0X_IMPLEMENTATION_VER_MINOR 0
/** VL53L0X PAL IMPLEMENTATION sub version */
#define VL53L0X_IMPLEMENTATION_VER_SUB 1
/** VL53L0X PAL IMPLEMENTATION sub version */
#define VL53L0X_IMPLEMENTATION_VER_REVISION 4606

#define VL53L0X_DEFAULT_MAX_LOOP 200
#define VL53L0X_MAX_STRING_LENGTH 32

#include "vl53l0x_device.h"
#include "vl53l0x_types.h"

namespace VL53L0X {

/****************************************
 * PRIVATE define do not edit
 ****************************************/

/** @brief Defines the parameters of the Get Version Functions
 */
  struct Version_t {
    uint32_t revision; /*!< revision number */
    uint8_t major;     /*!< major number */
    uint8_t minor;     /*!< minor number */
    uint8_t build;     /*!< build number */
  };

/** @brief Defines the parameters of the Get Device Info Functions
 */
  struct DeviceInfo_t {
    char Name[VL53L0X_MAX_STRING_LENGTH];     /*!< Name of the Device e.g. Left_Distance */
    char Type[VL53L0X_MAX_STRING_LENGTH];     /*!< Type of the Device e.g VL53L0X */
    char ProductId[VL53L0X_MAX_STRING_LENGTH];    /*!< Product Identifier String	*/
    uint8_t ProductType;     /*!< Product Type, VL53L0X = 1, VL53L1 = 2 */
    struct ProductRevision {
      uint8_t Major;   /*!< Product revision major */
      uint8_t Minor;/*!< Product revision minor */
    };
  };

/** @defgroup VL53L0X_define_Error_group Error and Warning code returned by API
 *	The following DEFINE are used to identify the PAL ERROR
 *	@{
 */

//this enum is the negative of the C version, we will restore the negation in C wrappers.
  enum Error : int8_t {
    ERROR_NONE = 0
    , ERROR_CALIBRATION_WARNING /*!< Warning invalid calibration data may be in use
        \a	VL53L0X_InitData()
        \a VL53L0X_GetOffsetCalibrationData
        \a VL53L0X_SetOffsetCalibrationData */
    , ERROR_MIN_CLIPPED /*!< Warning parameter passed was clipped to min before to be applied */
    , ERROR_UNDEFINED /*!< Unqualified error */
    , ERROR_INVALID_PARAMS  /*!< Parameter passed is invalid or out of range */
    , ERROR_NOT_SUPPORTED  /*!< Function is not supported in current mode or configuration */
    , ERROR_RANGE_ERROR  /*!< Device report a ranging error interrupt status */
    , ERROR_TIME_OUT  /*!< Aborted due to time out */
    , ERROR_MODE_NOT_SUPPORTED  /*!< Asked mode is not supported by the device */
    , ERROR_BUFFER_TOO_SMALL  /*!< ... */
    , ERROR_GPIO_NOT_EXISTING  /*!< User tried to setup a non-existing GPIO pin */
    , ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED  /*!< unsupported GPIO functionality */
    , ERROR_INTERRUPT_NOT_CLEARED  /*!< Error during interrupt clear */

//pointless jump in numbers, due to mashing different sets of error into one enum.
    , ERROR_CONTROL_INTERFACE = 20 /*!< error reported from IO functions */
    , ERROR_INVALID_COMMAND = 30 /*!< The command is not allowed in the current device state (power down) */
    , ERROR_DIVISION_BY_ZERO = 40 /*!< In the function a division by zero occurs */
    , ERROR_REF_SPAD_INIT = 50 /*!< Error during reference SPAD initialization */
    , ERROR_NOT_IMPLEMENTED = 99  /*!< Tells requested functionality has not been implemented yet or not compatible with the device */

  };

/** base class for returning values from a device, which operation might fail.
 * frequently such errors are ignored, tacking them onto the results of the failed operation makes after the fact reporting cheaper for those who care.*/
  template<typename Wrapped> class Erroneous {
  public:
    Wrapped wrapped;
    Error error;

    Erroneous(Wrapped wrapped, Error error = ERROR_NONE) : wrapped(wrapped), error(error) {
    }

    /** @returns whether value is GOOD. If not you can inspect Error for what went wrong */
    bool isOk() const {
      return error == 0;
    }

    /** syntactic sugar: rarely used monadic function for interesting boolean fact.*/
    bool operator~() const {
      return error != 0;
    }

    /** this should allow transparent access as the type */
    operator Wrapped &() {
      return wrapped;
    }

    operator Wrapped *(){
      return &wrapped;
    }
  };

/** ERROR_OUT is useful for when there will be no common exit code on an error that terminates a sequence of steps other than returning the first such error.
 * To use it name the VL53L0X_Error member Error (instead of Status), which also allows for if(!Error) which is more readable than if(Status != VL53L0X_ERROR_NONE)
 * */
#define ERROR_OUT if(Error) return Error

/** @} VL53L0X_define_Error_group */

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
/** @defgroup VL53L0X_define_PowerModes_group List of available Power Modes
 *	List of available Power Modes
 *	@{
 */

  enum PowerModes : uint8_t {
    POWERMODE_STANDBY_LEVEL1 = 0 /*!< Standby level 1 */
    , POWERMODE_STANDBY_LEVEL2 /*!< Standby level 2 */
    , POWERMODE_IDLE_LEVEL1  /*!< Idle level 1 */
    , POWERMODE_IDLE_LEVEL2 /*!< Idle level 2 */

/** @} VL53L0X_define_PowerModes_group */
  };
/** @brief Defines all parameters for the device
 */
  struct DeviceParameters_t {
    DeviceModes DeviceMode; /*!< Defines type of measurement to be done for the next measure */
    HistogramModes HistogramMode;/*!< Defines type of histogram measurement to be done for the next measure */
    uint32_t MeasurementTimingBudgetMicroSeconds; /*!< Defines the allowed total time for a single measurement */
    uint32_t InterMeasurementPeriodMilliSeconds; /*!< Defines time between two consecutive measurements (between two	measurement starts). If set to 0 means back-to-back mode */
    uint8_t XTalkCompensationEnable; /*!< Tells if Crosstalk compensation shall be enable or not	 */
    uint16_t XTalkCompensationRangeMilliMeter;  /*!< CrossTalk compensation range in millimeter	 */
    FixPoint1616_t XTalkCompensationRateMegaCps; /*!< CrossTalk compensation rate in Mega counts per seconds.	*/
    int32_t RangeOffsetMicroMeters; /*!< Range offset adjustment (mm).	*/

    uint8_t LimitChecksEnable[CHECKENABLE_NUMBER_OF_CHECKS]; /*!< This Array store all the Limit Check enable for this device. */
    uint8_t LimitChecksStatus[CHECKENABLE_NUMBER_OF_CHECKS]; /*!< This Array store all the Status of the check linked to last measurement. */
    FixPoint1616_t LimitChecksValue[CHECKENABLE_NUMBER_OF_CHECKS];     /*!< This Array store all the Limit Check value for this device */

    uint8_t WrapAroundCheckEnable; /*!< Tells if Wrap Around Check shall be enable or not */
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
    int32_t AmbTuningWindowFactor_K;
    /*!<  internal algo tuning (*1000) */
    int32_t RetSignalAt0mm;
    /*!< intermediate dmax computation value caching */
  };

/**
 * @struct VL53L0X_RangeData_t
 * @brief Range measurement data.
 */
  struct RangingMeasurementData_t {
    uint32_t TimeStamp; /*!< 32-bit time stamp. */
    uint32_t MeasurementTimeUsec;     /*!< Give the Measurement time needed by the device to do the measurement.*/

    uint16_t RangeMilliMeter; /*!< range distance in millimeter. */

    uint16_t RangeDMaxMilliMeter;  /*!< Tells what is the maximum detection distance of the device in current setup and environment conditions (Filled when applicable) */

    FixPoint1616_t SignalRateRtnMegaCps; /*!< Return signal rate (MCPS)\n these is a 16.16 fix point value, which is effectively a measure of target reflectance.*/
    FixPoint1616_t AmbientRateRtnMegaCps;  /*!< Return ambient rate (MCPS)\n these is a 16.16 fix point value, which is effectively a measure of the ambient light.*/

    uint16_t EffectiveSpadRtnCount;  /*!< Return the effective SPAD count for the return signal. To obtain Real value it should be divided by 256 */

    uint8_t ZoneId;  /*!< Denotes which zone and range scheduler stage the range data relates to. */
    uint8_t RangeFractionalPart; /*!< Fractional part of range distance. Final value is a FixPoint168 value. */
    uint8_t RangeStatus;  /*!< Range Status for the current measurement. This is device dependent. Value = 0 means value is valid. 	See \ref RangeStatusPage */
  };

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

    DeviceError ErrorStatus;
    /*!< Error status of the current measurement. \n
    see @a ::VL53L0X_DeviceError @a VL53L0X_GetStatusErrorString() */
  };

  //appears as a const int in a few places:
#define REF_SPAD_BUFFER_SIZE 6
  using SpadArray = uint8_t[REF_SPAD_BUFFER_SIZE];

  /**
 * @struct VL53L0X_SpadData_t
 * @brief Spad Configuration Data.
 */
  struct SpadData_t {
    SpadArray RefSpadEnables;
    /*!< Reference Spad Enables */
    SpadArray RefGoodSpadMap;
    /*!< Reference Spad Good Spad Map */
  };

  struct SigmaEstimates {
    uint16_t RefArray; /*!< Reference array sigma value in 1/100th of [mm] e.g. 100 = 1mm */
    uint16_t EffPulseWidth;  /*!< Effective Pulse width for sigma estimate in 1/100th of ns e.g. 900 = 9.0ns */
    uint16_t EffAmbWidth;     /*!< Effective Ambient width for sigma estimate in 1/100th of ns e.g. 500 = 5.0ns */
  };

  struct DeviceSpecificParameters_t {
    FixPoint1616_t OscFrequencyMHz; /* Frequency used */

    uint16_t LastEncodedTimeout;/* last encoded Time out used for timing budget*/

    GpioFunctionality Pin0GpioFunctionality;/* store the functionality of the GPIO: pin0 */

    struct RangeSetting {
      uint32_t TimeoutMicroSecs;/*!< Execution time of the final range*/
      uint8_t VcselPulsePeriod;/*!< Vcsel pulse period (pll clocks) for the final range measurement*/
    };
    RangeSetting FinalRange;/*!< Execution &Vcsel time of the final range*/
    RangeSetting PreRange;/*!< Execution time of the pre-range range (ST had rogue tile in this comment) */

    SigmaEstimates SigmaEst;

    uint8_t ReadDataFromDeviceDone; /* Indicate if read from device has been done (==1) or not (==0) */
    uint8_t ModuleId;               /* Module ID */
    uint8_t Revision;               /* test Revision */
    char ProductId[VL53L0X_MAX_STRING_LENGTH];/* Product Identifier String  */

    uint8_t ReferenceSpadCount;  /* used for ref spad management */
    uint8_t ReferenceSpadType;   /* used for ref spad management */
    uint8_t RefSpadsInitialised; /* reports if ref spads are initialised. */
    uint32_t PartUIDUpper;       /*!< Unique Part ID Upper */
    uint32_t PartUIDLower;       /*!< Unique Part ID Lower */
    FixPoint1616_t SignalRateMeasFixed400mm; /*!< Peek Signal rate at 400 mm*/
  };

/**
 * @struct VL53L0X_DevData_t
 *
 * @brief VL53L0X PAL device ST private data structure \n
 * End user should never access any of these field directly
 *
 * These must never access directly but only via macro
 */
  struct DevData_t {
    DMaxData_t DMaxData;     /*!< Dmax Data */
    int32_t Part2PartOffsetNVMMicroMeter;     /*!< backed up NVM value */
    int32_t Part2PartOffsetAdjustmentNVMMicroMeter;     /*!< backed up NVM value representing additional offset adjustment */
    DeviceParameters_t CurrentParameters;     /*!< Current Device Parameter */
    RangingMeasurementData_t LastRangeMeasure;     /*!< Ranging Data */
    HistogramMeasurementData_t LastHistogramMeasure;     /*!< Histogram Data */
    DeviceSpecificParameters_t DeviceSpecificParameters;     /*!< Parameters specific to the device */
    SpadData_t SpadData;    /*!< Spad Data */
    uint8_t SequenceConfig;    /*!< Internal value for the sequence config */
    uint8_t RangeFractionalEnable;    /*!< Enable/Disable fractional part of ranging data */
    State PalState;    /*!< Current state of the PAL for this device */
    PowerModes PowerMode;/*!< Current Power Mode	 */

    SigmaEstimates SigmaEst;

    /*!< Effective Ambient width for sigma estimate in 1/100th of ns
     * e.g. 500 = 5.0ns */

    uint8_t StopVariable;     /*!< StopVariable used during the stop sequence */
    uint16_t targetRefRate;     /*!< Target Ambient Rate for Ref spad management */
    FixPoint1616_t SigmaEstimate;     /*!< Sigma Estimate - based on ambient & VCSEL rates and signal_total_events */
    FixPoint1616_t SignalEstimate;     /*!< Signal Estimate - based on ambient & VCSEL rates and cross talk */
    FixPoint1616_t LastSignalRefMcps;     /*!< Latest Signal ref in Mcps */
    uint8_t *pTuningSettingsPointer;     /*!< Pointer for Tuning Settings table */
    uint8_t UseInternalTuningSettings;     /*!< Indicate if we use	 Tuning Settings table */
    uint16_t LinearityCorrectiveGain;     /*!< Linearity Corrective Gain value in x1000 */
    uint16_t DmaxCalRangeMilliMeter;     /*!< Dmax Calibration Range millimeter */
    FixPoint1616_t DmaxCalSignalRateRtnMegaCps;     /*!< Dmax Calibration Signal Rate Return MegaCps */

  };

/** @defgroup VL53L0X_define_InterruptPolarity_group Defines the Polarity
 * of the Interrupt
 *	Defines the Polarity of the Interrupt
 *	@{
 *
 *	a trivial enum
 */
  enum InterruptPolarity : uint8_t {
    INTERRUPTPOLARITY_LOW = 0 /*!< Set active low polarity best setup for falling edge. */
    , INTERRUPTPOLARITY_HIGH /*!< Set active high polarity best setup for rising edge. */
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

/** @defgroup VL53L0X_define_SchedulerSequence_group Defines the steps
 * carried out by the scheduler during a range measurement.
 *	@{
 *	Defines the states of all the steps in the scheduler
 *	i.e. enabled/disabled.
 */
  struct SchedulerSequenceSteps_t {
    uint8_t TccOn;        /*!<Reports if Target Centre Check On  */
    uint8_t MsrcOn;       /*!<Reports if MSRC On  */
    uint8_t DssOn;        /*!<Reports if DSS On  */
    uint8_t PreRangeOn;   /*!<Reports if Pre-Range On	*/
    uint8_t FinalRangeOn; /*!<Reports if Final-Range On  */
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


/** @} VL53L0X_define_SequenceStepId_group */

/* MACRO Definitions */
/** @defgroup VL53L0X_define_GeneralMacro_group General Macro Defines
 *	General Macro Defines
 *	@{
 */

/* Defines */
#define VL53L0X_SETPARAMETERFIELD(Dev, field, value) PALDevDataSet(Dev, CurrentParameters.field, value)

#define VL53L0X_GETPARAMETERFIELD(Dev, field, variable) variable = PALDevDataGet(Dev, CurrentParameters).field

#define VL53L0X_SETARRAYPARAMETERFIELD(Dev, field, index, value)   PALDevDataSet(Dev, CurrentParameters.field[index], value)

#define VL53L0X_GETARRAYPARAMETERFIELD(Dev, field, index, variable)  variable = PALDevDataGet(Dev, CurrentParameters).field[index]

#define VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, field, value) PALDevDataSet(Dev, DeviceSpecificParameters.field, value)

#define VL53L0X_GETDEVICESPECIFICPARAMETER(Dev, field)  PALDevDataGet(Dev, DeviceSpecificParameters).field

#define VL53L0X_FIXPOINT1616TOFIXPOINT97(Value)  (uint16_t)((Value >> 9) & 0xFFFF)
#define VL53L0X_FIXPOINT97TOFIXPOINT1616(Value) (FixPoint1616_t)(Value << 9)

#define VL53L0X_FIXPOINT1616TOFIXPOINT88(Value)  (uint16_t)((Value >> 8) & 0xFFFF)
#define VL53L0X_FIXPOINT88TOFIXPOINT1616(Value) (FixPoint1616_t)(Value << 8)

#define VL53L0X_FIXPOINT1616TOFIXPOINT412(Value) (uint16_t)((Value >> 4) & 0xFFFF)
#define VL53L0X_FIXPOINT412TOFIXPOINT1616(Value) (FixPoint1616_t)(Value << 4)

#define VL53L0X_FIXPOINT1616TOFIXPOINT313(Value) (uint16_t)((Value >> 3) & 0xFFFF)
#define VL53L0X_FIXPOINT313TOFIXPOINT1616(Value) (FixPoint1616_t)(Value << 3)

#define VL53L0X_FIXPOINT1616TOFIXPOINT08(Value) (uint8_t)((Value >> 8) & 0x00FF)
#define VL53L0X_FIXPOINT08TOFIXPOINT1616(Value) (FixPoint1616_t)(Value << 8)

#define VL53L0X_FIXPOINT1616TOFIXPOINT53(Value)  (uint8_t)((Value >> 13) & 0x00FF)
#define VL53L0X_FIXPOINT53TOFIXPOINT1616(Value) (FixPoint1616_t)(Value << 13)

//BUG: rogue tile, shift should be 12 not 14.
#define VL53L0X_FIXPOINT1616TOFIXPOINT102(Value)  (uint16_t)((Value >> 14) & 0x0FFF)
#define VL53L0X_FIXPOINT102TOFIXPOINT1616(Value) (FixPoint1616_t)(Value << 12)

#define VL53L0X_MAKEUINT16(lsb, msb)  (uint16_t)((((uint16_t)msb) << 8) + (uint16_t)lsb)

/** @} VL53L0X_define_GeneralMacro_group */

/** @} VL53L0X_globaldefine_group */
}; //end namespace
#endif /* _VL53L0X_DEF_H_ */
