/*******************************************************************************
 Copyright 2021 Andy Heilveil, github/980f starting with source
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
 *****************************************************************************/

#ifndef VL53L0X__api_H_
#define VL53L0X__api_H_

#include "vl53l0x_api_core.h" //extends this

namespace VL53L0X {
  //for runtime version decisions
  extern const Version_t ImplementationVersion;
  extern const Version_t PalSpecVersion;

  /** division of Api and Core seems arbitrary. Might make Core a private base to hide it if that was the intent of the orignal API developers. */
  class Api : public Core {
  public:
    /** once the comm member of the Physical interface is given an abstract interface we can build one and pass it in here instead of passing in its constructor args.
     * before that we can make a macro for the arg list so that we don't have to change it at the three levels ...*/
    Api(Arg &&args) : Core(std::forward<Arg>(args)) {
      //do nothing so that we may static construct.
    }

/** @defgroup cut11_group VL53L0X cut1.1 Function Definition
 *  @brief    VL53L0X cut1.1 Function Definition
 *  @{
 */


/** @defgroup general_group VL53L0X General Functions
 *  @brief    General functions and definitions
 *  @{
 */


/**
 * @brief Reads the Device information for given Device
 *
 * @note This function Access to the device
 *
 * @param   pDeviceInfo  Pointer to current device info for the Device
 * @return  ERROR_NONE   Success
 * @return  "Other error code"  See ::Error
 */
    bool GetDeviceInfo(DeviceInfo_t &pDeviceInfo);

/**
 * @brief Read current status of the error register for the selected device
 *
 * @note This function Access to the device
 * @return  value read from device
 */
    DeviceError GetDeviceErrorStatus();

/**
 * @brief Human readable Range Error string for a given rangeError
 *
 * @note This function doesn't access to the device
 *
 * @param   rangeStatus         The rangeError code as stored on a RangingMeasurementData_t
 * @return  pointer to const text.
 */
    const char *GetRangeStatusString(RangeStatus rangeStatus);

/**
 * @brief Human readable error string for a given Error Code

 * @note This function doesn't access to the device
 *
 * @param   ErrorCode           The error code as stored on  ::DeviceError
 * @return  pointer to const string
 */
    const char *GetDeviceErrorString(DeviceError ErrorCode);

/**
 * @brief Human readable error string for current PAL error status
 *
 * @note This function doesn't access to the device
 *
 * @param   PalErrorCode       The error code as stored on @a Error
 * @return  The error string corresponding to the PalErrorCode
 */
    const char *GetPalErrorString(Error PalErrorCode);

/**
 * @brief Human readable PAL State string
 *
 * @note This function doesn't access to the device
 *
 * @param   PalStateCode          The State code as stored on @a State
 * PalStateCode
 * @return pointer to const text.
 */
    const char *GetPalStateString(State PalStateCode);

/**
 * @brief Reads the internal state of the PAL for a given Device
 *
 * @note This function doesn't access to the device
 *
 * @return  current state of the PAL for a given Device
 */
    State GetPalState();

/**
 * @brief Set the power mode for a given Device
 * The power mode can be Standby or Idle.
 * Different level of both Standby and Idle can exist i some models.
 * This function should not be used when device is in Ranging state.
 *
 * @note This function Access to the device
 *

 * @param   PowerMode             The value of the power mode to set.
 * see ::PowerModes
 *                                Valid values are:
 *                                POWERMODE_STANDBY_LEVEL1,
 *                                POWERMODE_IDLE_LEVEL1
 * @return  Success, will fail if you choose an invalid enum value
 */
    bool SetPowerMode(PowerModes PowerMode);

/**
 * @brief Get the power mode of the Device
 *
 * @note This function Access to the device
 *
 * @param   pPowerMode            Pointer to the current value of the power
 * mode. see ::PowerModes
 * @return  powerMode
 */
    PowerModes GetPowerMode();

/**
 * Set or over-hide part to part calibration offset
 * \sa DataInit()   GetOffsetCalibrationDataMicroMeter()
 *
 * @note This function Access to the device
 *
 * @param   OffsetCalibrationDataMicroMeter    Offset (microns)
 */
    void SetOffsetCalibrationDataMicroMeter(int32_t OffsetCalibrationDataMicroMeter);

/**
 * @brief Get part to part calibration offset
 *
 * @par Function Description
 * Should only be used after a successful call to @a DataInit to backup device NVM value
 *
 * @note This function Access to the device
 *
 * @return  part to part calibration offset from device (microns)
 */
    int32_t GetOffsetCalibrationDataMicroMeter();

/**
 * Set the linearity corrective gain
 *
 * @note This function Access to the device
 *
 * @param   LinearityCorrectiveGain   Linearity corrective gain in x1000
 * if value is 1000 then no modification is applied.
 *
 * 980F changed to unsigned type as underlying value is unsigned and any signed value when interpreted as unsigned will be above the limit that this item has
 * @return  Success
 */
    bool SetLinearityCorrectiveGain(uint16_t LinearityCorrectiveGain);

/**
 * @brief Get the linearity corrective gain
 *
 * @par Function Description
 * Should only be used after a successful call to @a DataInit to cache device NVM value
 *
 * @note This function did NOT Access to the device
 *
 * @return linearity  corrective gain in x1000 (1000 == 1.0)
 */
    uint16_t GetLinearityCorrectiveGain();

/**
 * Set Group parameter Hold state
 *
 * @par Function Description
 * Set or remove device internal group parameter hold
 *
 * @note This function does nothing
 *
 * @param   GroupParamHold   Group parameter Hold state to be set (on/off)
 * @return  false
 */
    bool SetGroupParamHold(uint8_t GroupParamHold);

/**
 * @brief Get the maximal distance for actual setup
 * @par Function Description
 * Device must be initialized through @a SetParameters() prior calling
 * this function.
 *
 * Any range value more than the value returned is to be considered as
 * "no target detected" or
 * "no target in detectable range"\n
 * @warning The maximal distance depends on the setup
 *
 * @note This function does nothing
 *
 * @return  extreme value for type
 */
    uint16_t GetUpperLimitMilliMeter();

/**
 * @brief Get the Total Signal Rate
 * @par Function Description
 * This function will return the Total Signal Rate after a good ranging is done.
 *
 * @note This function access to Device
 *
 * @return  Total Signal Rate value in Mega count per second
 */
    MegaCps GetTotalSignalRate();

/** @} general_group */

/** @defgroup init_group VL53L0X Init Functions
 *  @brief    VL53L0X Init Functions
 *  @{
 */

/**
 * @brief Set new device address
 *
 * After completion the device will answer to the new address programmed.
 * This function should be called when several devices are used in parallel
 * before start programming the sensor.
 * When a single device is used, there is no need to use this functionality.
 *
 * @note This function Access to the device
 *
 * @param   _8bitAddress         The new Device address
 * @return       Success
 */
    bool SetDeviceAddress(uint8_t _8bitAddress);

/**
 *
 * @brief One time device initialization
 *
 * To be called once and only once after device is brought out of reset
 * (Chip enable) and booted see @a WaitDeviceBooted()
 *
 * @par Function Description
 * When not used after a fresh device "power up" or reset, it may return
 * @a #ERROR_CALIBRATION_WARNING meaning wrong calibration data
 * may have been fetched from device that can result in ranging offset error\n
 * If application cannot execute device reset or need to run DataInit
 * multiple time then it  must ensure proper offset calibration saving and
 * restore on its own by using @a GetOffsetCalibrationData() on first
 * power up and then @a SetOffsetCalibrationData() in all subsequent
 * init This function will change the State from STATE_POWERDOWN
 * to STATE_WAIT_STATICINIT.
 *
 * @note This function Access to the device
 * @note This function takes more than a millisecond but otherwise does not block
 *
 */
    void DataInit();

/**
 * @brief Set the tuning settings pointer
 *
 * This function is used to specify the Tuning settings buffer to be used
 * for a given device. The buffer contains all the necessary data to permit
 * the API to write tuning settings.
 * This function permit to force the usage of either external or internal
 * tuning settings.
 *
 * @note This function Access to the device
 *

 * @param   pTuningSettingBuffer            Pointer to tuning settings buffer.
 * @param   UseInternalTuningSettings       Use internal tuning settings value.
 * @return  ERROR_NONE     Success
 * @return  "Other error code"    See ::Error
 */
    bool SetTuningSettingBuffer(Tunings pTuningSettingBuffer, bool UseInternalTuningSettings);

/**
 * @brief Get the tuning settings pointer and the internal external switch value.
 *
 * This function is used to get the Tuning settings buffer pointer if not set to internal.
 *
 * @note This function DOES NOT Access to the device
 *
 * @return  if @param theInternal is true then the default/internal table else the pointer which will be used which might be the default one or one set via SetTuningSettingBuffer
 */
    Tunings GetTuningSettingBuffer(bool theInternal = false);

/**
 * @brief Do basic device init (and eventually patch loading)
 * This function will change the State from
 * STATE_WAIT_STATICINIT to STATE_IDLE.
 * In this stage all default setting will be applied.
 *
 * @note This function Access to the device
 *
 * @return      Success
 */
    bool StaticInit();

/**
 * @brief Wait for device booted after chip enable (hardware standby)
 * This function can be run only when State is STATE_POWERDOWN.
 *
 * @note This function does nothing
 *
 * @return  false
 *
 * todo: can use i2c address probe to detect device ready to communicate.
 */
    bool WaitDeviceBooted();

/**
 * @brief Do an hard reset or soft reset (depending on implementation) of the
 * device \nAfter call of this function, device must be in same state as right
 * after a power-up sequence.This function will change the State to
 * STATE_POWERDOWN.
 *
 * @note This function Access to the device
 * @note  This function blacks until reset is successful.
 *
 * @return Success
 */
    bool ResetDevice();

/** @} init_group */

/** @defgroup parameters_group VL53L0X Parameters Functions
 *  @brief    Functions used to prepare and setup the device
 *  @{
 */

/**
 * @brief  Prepare device for operation
 * @par Function Description
 * Update device with provided parameters
 * @li Then start ranging operation.
 *
 * @note This function Access to the device
 *
 * @param   pDeviceParameters     Pointer to store current device parameters.
 */
    void SetDeviceParameters(const DeviceParameters_t &pDeviceParameters);

/**
 * @brief  Retrieve current device parameters
 * @par Function Description
 * Get actual parameters of the device
 * @li Then start ranging operation.
 *
 * @note This function Access to the device
 *
 * @param   pDeviceParameters     Pointer to store
 * @return  current device parameters.
 */
    DeviceParameters_t GetDeviceParameters();

/**
 * @brief  Set a new device mode
 * @par Function Description
 * Set device to a new mode (ranging, histogram ...)
 *
 * @note This function doesn't Access to the device
 *

 * @param   DeviceMode            see DeviceModes enum, although some might not be allowed on some devices
 *
 *
 * @return   Success
 */
    bool SetDeviceMode(DeviceModes DeviceMode);

/**
 * @brief  Get current new device mode
 * @par Function Description
 * Get last requested mode of the device(ranging, histogram ...)
 *
 * @note This function doesn't Access to the device
 *
 * @return  see DeviceModes enum
 */
    DeviceModes GetDeviceMode();

/**
 * @brief  Sets the resolution of range measurements.
 * @par Function Description
 * Set resolution of range measurements to either 0.25mm if fraction enabled or 1mm if not enabled.
 *
 * @note This function Accesses the device
 *
 * @param   Enable            Enable high resolution (1/4 mm)
 *
 */
    void SetRangeFractionEnable(bool Enable);

/**
 * @brief  Gets the fraction enable parameter indicating the resolution of
 * range measurements.
 *
 * @par Function Description
 * Gets the fraction enable state, which translates to the resolution of
 * range measurements as follows :Enabled:=0.25mm resolution,
 * Not Enabled:=1mm resolution.
 *
 * @note This function Accesses the device
 *
 * @return  whether resolution is 1/4 mm or 1 mm
 */
    bool GetFractionEnable();

/**
 * @brief Set Ranging Timing Budget in microseconds
 *
 * @par Function Description
 * Defines the maximum time allowed by the user to the device to run a
 * full ranging sequence for the current mode (ranging, histogram, ASL ...)
 *
 * @note This function Access to the device
 *

 * @param MeasurementTimingBudgetMicroSeconds  Max measurement time in
 * microseconds.
 *                                   Valid values are:
 *                                   >= 17000 microsecs when wraparound enabled
 *                                   >= 12000 microsecs when wraparound disabled
 *                                   (bug: 20000 enforced by present code!)
 * @return  Success
 */
    bool SetMeasurementTimingBudgetMicroSeconds(uint32_t MeasurementTimingBudgetMicroSeconds) {
      return set_measurement_timing_budget_micro_seconds(MeasurementTimingBudgetMicroSeconds);
    }

/**
 * @brief Get Ranging Timing Budget in microseconds
 *
 * @par Function Description
 * Returns the programmed the maximum time allowed by the user to the
 * device to run a full ranging sequence for the current mode
 * (ranging, histogram, ASL ...)
 *
 * @note This function Access to the device
 *
 * @return   Max measurement time in microseconds.
 */
    uint32_t GetMeasurementTimingBudgetMicroSeconds();

/**
 * @brief Gets the VCSEL pulse period.
 *
 * @par Function Description
 * This function retrieves the VCSEL pulse period for the given period type.
 *
 * @note This function Accesses the device
 *
 * @param   VcselPeriodType          VCSEL period identifier (pre-range|final).
 * @return  period in apparently useconds
 */
    uint8_t GetVcselPulsePeriod(VcselPeriod VcselPeriodType);

/**
 * @brief Sets the VCSEL pulse period.
 *
 * @par Function Description
 * This function sets the VCSEL pulse period for the given period type.
 * Sometime after this and  before measurements you must call  perform_phase_calibration
 *
 *
 * @note This function Accesses the device
 *
 * @param   VcselPeriodType	      VCSEL period identifier @see VcselPeriod enum
 * @param   VCSELPulsePeriod      VCSEL period value
 * @return              Success
 */
    bool SetVcselPulsePeriod(VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriod);

/**
 * @brief Sets the (on/off) state of a requested sequence step.
 *
 * @par Function Description
 * This function enables/disables a requested sequence step.
 *
 * @note This function Accesses the device
 *
 * @param   SequenceStepId	         Sequence step identifier.
 * @param   SequenceStepEnabled          Demanded state {0=Off,1=On}
 */
    void SetSequenceStepEnable(SequenceStepId SequenceStepId, bool SequenceStepEnabled);

/**
 * @brief Sets the timeout of a requested sequence step.
 *
 * @par Function Description
 * This function sets the timeout of a requested sequence step.
 *
 * @note This function Accesses the device
 *
 * @param   SequenceStepId               Sequence step identifier.
 * @param   TimeOutMilliSecs             Demanded timeout
 * @return  Success
 */
    bool SetSequenceStepTimeout(SequenceStepId SequenceStepId, FixPoint1616_t TimeOutMilliSecs);

/**
 * @brief Gets the timeout of a requested sequence step.
 *
 * @par Function Description
 * This function retrieves the timeout of a requested sequence step.
 *
 * @note This function Accesses the device
 *
 * @param   SequenceStepId               Sequence step identifier.
 * @return  MilliSecs            Timeout value.
 */
    FixPoint1616_t GetSequenceStepTimeout(SequenceStepId SequenceStepId);

/**
 * @brief Gets number of sequence steps managed by the API.
 *
 * @par Function Description
 * This function retrieves the number of sequence steps currently managed
 * by the API
 * use the SequenceStepId enum and you don't have to check this.
 *
 * @note This function Accesses the device
 * @return  max sequence step id
 */
    static SequenceStepId GetNumberOfSequenceSteps();

/**
 * @brief Gets the name of a given sequence step.
 *
 * @par Function Description
 * This function retrieves the name of sequence steps corresponding to
 * SequenceStepId.
 *
 * @note This function doesn't Accesses the device
 *
 * @param   SequenceStepId               Sequence step identifier.
 * @param   pSequenceStepsString         Pointer to Info string
 *
 * @return  ERROR_NONE            Success
 * @return  "Other error code"           See ::Error
 */
    const char *GetSequenceStepsInfo(SequenceStepId SequenceStepId);

/**
 * Program continuous mode Inter-Measurement period in milliseconds
 *
 * @par Function Description
 * When trying to set too short time return  INVALID_PARAMS minimal value
 *
 * @note This function Access to the device
 *
 * @param   InterMeasurementPeriodMilliSeconds   Inter-Measurement Period in ms.
 */
    void SetInterMeasurementPeriodMilliSeconds(unsigned int InterMeasurementPeriodMilliSeconds);

/**
 * Get continuous mode Inter-Measurement period in milliseconds
 * @note This function Access to the device
 *
 * @return  Inter-Measurement Period in milliseconds.
 */
    uint32_t GetInterMeasurementPeriodMilliSeconds();

    /** these guys are often manipulate at the same time so we make them a pair.
     * They are small enough to pass the struct in a register so copying is cheap. */
    struct CalibrationParameters {
      uint8_t VhvSettings = 0; //zero init for when we are only using one field
      uint8_t PhaseCal = 0;
    };

/**
 * @brief Set Reference Calibration Parameters
 *
 * @par Function Description
 * Set Reference Calibration Parameters.
 *
 * @note This function Access to the device
 *
 * @param   Parameter for VHV and PhaseCal
 */
    void SetRefCalibration(CalibrationParameters refParams);

/**
 * @brief Get Reference Calibration Parameters
 *
 * @par Function Description
 * Get Reference Calibration Parameters.
 *
 * @note This function Access to the device
 *
 * @return  vhv phase pair
 */
    CalibrationParameters GetRefCalibration();

/**
 * @brief  Get the number of the check limit managed by a given Device
 *
 * @par Function Description
 * This function give the number of the check limit managed by the Device.
 * Use the CheckEnable enum and you will never need to reference this.
 *
 *   (0<= LimitCheckId < GetNumberOfLimitCheck() ).
 *
 * @note This function doesn't Access to the device
 * @return  check value for limitId.
 */
    static CheckEnable GetNumberOfLimitCheck();

/**
 * @brief  Return a description string for a given limit check number
 *
 * @par Function Description
 * This function returns a description string for a given limit check number.
 * The limit check is identified with the LimitCheckId.
 *
 * @note This function doesn't Access to the device
 *
 * @param   LimitCheckId                  Limit Check ID
 * @return  description of limit checked
 */
    static const char *GetLimitCheckInfo(CheckEnable LimitCheckId);

/**
 * @brief  Return a the Error of the specified check limit
 *
 * @par Function Description
 * This function returns the Error of the specified check limit.
 * The value indicate if the check is fail or not.
 * The limit check is identified with the LimitCheckId.
 *
 * @note This function doesn't Access to the device
 *
 * @param   LimitCheckId                  Limit Check ID

 * @return  whether limit fired, IE limit was exceeded
 */
    bool GetLimitCheckStatus(CheckEnable LimitCheckId);

/**
 * @brief  Enable/Disable a specific limit check
 *
 * @par Function Description
 * This function Enable/Disable a specific limit check.
 *
 * @note This function doesn't Access to the device
 *
 * @param   LimitCheckId                  Limit Check ID
 * @param   LimitCheckEnable              whether to check limit corresponding to LimitCheckId
 * @return  Success
 */
    void SetLimitCheckEnable(CheckEnable LimitCheckId, bool LimitCheckEnable);

/**
 * @brief  Set a specific limit check value
 *
 * @par Function Description
 * This function set a specific limit check value.
 * The limit check is identified with the LimitCheckId.
 *
 * @note This function Access to the device
 *
 * @param   LimitCheckId                  Limit Check ID
 * @param   LimitCheckValue               Limit check Value for given LimitCheckId
 */
    void SetLimitCheckValue(CheckEnable LimitCheckId, Cps16 LimitCheckValue);

    /** convenience to set limit enable and value in one call.
     * we might remove the individual setters.
     *
     * use SetLimitCheck(CheckSomething, { enable, value });
     * */
    void SetLimitCheck(CheckEnable LimitCheckId, LimitTuple limit);

/**
 * @brief  Get the current value of the signal used for the limit check
 *
 * @par Function Description
 * This function gets the current value of the signal used for the limit check.
 * To obtain the latest value you should run a ranging before.
 * The value reported is linked to the limit check identified with the LimitCheckId.
 *
 * @note This function Access to the device
 *
 * @param   LimitCheckId                  Limit Check ID
 * @return  value limit is checked against
 *
 * @throws ERROR_ILLEGAL_PARAMS if you don't use the enum for the CheckId
 */
    MegaCps GetLimitCheckCurrent(CheckEnable LimitCheckId);

/**
 * @brief  Enable (or disable) Wrap around Check
 *
 * @note This function Access to the device
 *
 * @param   WrapAroundCheckEnable  whether to perform Wrap around Check
 **/
    void SetWrapAroundCheckEnable(bool WrapAroundCheckEnable);

/**
 * @brief  Get setup of Wrap around Check
 *
 * @par Function Description
 * This function get the wrapAround check enable parameters
 *
 * @note This function Access to the device
 *
 * @return  whether wrap around check is enabled
 */
    bool GetWrapAroundCheckEnable();

/**
 * @brief   Set Dmax Calibration Parameters for a given device
 * When one of the parameter is zero, this function will get parameter
 * from NVM.
 * @note This function doesn't Access to the device
 *
 * @param   RangeMilliMeter        Calibration Distance and rate read at that distance
 * @return   Success
 */
    bool SetDmaxCalParameters(const DevData_t::DmaxCal &p);

/**
 * @brief  Get Dmax Calibration Parameters for a given device
 *
 * @note This function Access to the device
 *
 * @return   pRangeMilliMeter        Calibration Distance and its rate
 */
    DevData_t::DmaxCal GetDmaxCalParameters();

/** @} parameters_group */

/** @defgroup measurement_group VL53L0X Measurement Functions
 *  @brief    Functions used for the measurements
 *  @{
 */

/**
 * @brief Single shot measurement.
 *
 * @par Function Description
 * Perform simple measurement sequence (Start measure, Wait measure to end,
 * and returns when measurement is done).
 * Once function returns, user can get valid data by calling
 * GetRangingMeasurement or GetHistogramMeasurement
 * depending on defined measurement mode
 * User should Clear the interrupt in case this are enabled by using the
 * function ClearInterruptMask().
 *
 * @warning This function is a blocking function
 *
 * @note This function Access to the device
 *

 * @return  Success  , @see GetRangingMeasurementData for results
 */
    bool PerformSingleMeasurement();

/**
 * @brief Perform Reference Calibration
 *
 * @details Perform a reference calibration of the Device.
 * This function should be run from time to time before doing
 * a ranging measurement.
 * This function will launch a special ranging measurement, so
 * if interrupt are enable an interrupt will be done.
 * This function will clear the interrupt generated automatically.
 *
 * @warning This function is blocking
 *
 * @note This function Access to the device
 *
 * @return   Success
 */
    bool PerformRefCalibration();

/**
 * @brief Perform XTalk Calibration
 *
 * @details Perform a XTalk calibration of the Device.
 * This function will launch a ranging measurement, if interrupts
 * are enabled an interrupt will be done.
 * This function will clear the interrupt generated automatically.
 * This function will program a new value for the XTalk compensation
 * and it will enable the cross talk before exit.
 * @note This function disables the CHECKENABLE_RANGE_IGNORE_THRESHOLD.
 *
 * @note This function change the device mode to DEVICEMODE_SINGLE_RANGING
 *
 * @warning This function is blocking
 *
 * @note This function Access to the device
 *
 *
 * @param   XTalkCalDistance     CalDistanceMilliMeter value used for the XTalk measurement
 * @return    Success
 */
    bool PerformXTalkCalibration(FixPoint1616_t XTalkCalDistance);

/**
 * @brief Perform Offset Calibration
 *
 * @details Perform a Offset calibration of the Device.
 * This function will launch a ranging measurement, if interrupts are
 * enabled an interrupt will be done.
 * This function will clear the interrupt generated automatically.
 * This function will program a new value for the Offset calibration value
 * @note  This function will disable the CHECKENABLE_RANGE_IGNORE_THRESHOLD.
 *
 * @warning This function is blocking
 *
 * @note This function Access to the device
 *
 * @note This function does not change the device mode.
 *
 * @param   CalDistanceMilliMeter     Calibration distance value used for the offset compensation.

 * @return  whether process completed.
 * if so then the value of interest will be in VL53L0X_GETPARAMETERFIELD(RangeOffsetMicroMeters)
 */
    bool PerformOffsetCalibration(FixPoint1616_t CalDistanceMilliMeter);

/**
 * @brief Start device measurement
 *
 * @details Started measurement will depend on device parameters set through
 * @a SetParameters()
 * This is a non-blocking function.
 * This function will change the State from STATE_IDLE to
 * STATE_RUNNING.
 *
 * @note This function Access to the device
 *
 *
 * @return Success
 * */
    bool StartMeasurement();

/**
 * @brief Stop device measurement
 *
 * @details Will set the device in standby mode at end of current measurement\n
 *          Not necessary in single mode as device shall return automatically
 *          in standby mode at end of measurement.
 *          This function will change the State from STATE_RUNNING to STATE_IDLE.
 *
 * @note This function Access to the device
 *
 * @return    Success
 */
    bool StopMeasurement();

/**
 * @brief Return Measurement Data Ready
 *
 * @par Function Description
 * This function indicate that a measurement data is ready.
 * This function check if interrupt mode is used then check is done accordingly.
 * If perform function clear the interrupt, this function will not work,
 * like in case of @a PerformSingleRangingMeasurement().
 * WaitDeviceReadyForNewMeasurement is blocking function, GetMeasurementDataReady
 * is used for non-blocking capture.
 *
 * @note This function Access to the device
 *
 * @return whether we have a definite indication of data being ready.
 */
    bool GetMeasurementDataReady();

    /** formerly declared in core.h but implemented in api.cpp
     * @returns false on timeout */
    bool measurement_poll_for_completion();

    /**
 * @brief Wait for device ready for a new measurement command.
 * Blocking function.
 *
 * @note This function is not Implemented
 *
 * @param   MaxLoop    Max Number of polling loop (timeout).
 * @return  false
 */
    bool WaitDeviceReadyForNewMeasurement(unsigned MaxLoop);

/**
 * @brief Retrieve the Reference Signal after a measurements
 *
 * @par Function Description
 * Get Reference Signal from last successful Ranging measurement
 * This function return a valid value after that you call the
 * @a GetRangingMeasurementData().
 *
 * @note This function Access to the device
 *
 * @return  signal rate of last successful measurement
 */
    MegaCps GetMeasurementRefSignal();

/**
 * @brief Retrieve the measurements from device for a given setup
 *
 * @par Function Description
 * Get data from last successful Ranging measurement
 * @warning USER should take care about  @a GetNumberOfROIZones()
 * before get data.
 * PAL will fill a NumberOfROIZones times the corresponding data
 * structure used in the measurement function.
 *
 * @note This function Access to the device
 *

 * @param   pRangingMeasurementData  Pointer to the data structure to fill up.
 * @return  Success.  on failure the object may be partially altered. If successful a copy is made on the PALData object
 */
    bool GetRangingMeasurementData(RangingMeasurementData_t &pRangingMeasurementData);
#if IncludeHistogramming
    /**
     * @brief  Set a new Histogram mode
     * @par Function Description
     * Set device to a new Histogram mode
     *
     * @note This function doesn't Access to the device
     *

     * @param   HistogramMode         New device mode to apply
     *                                Valid values are:
     *                                HISTOGRAMMODE_DISABLED
     *                                DEVICEMODE_SINGLE_HISTOGRAM
     *                                HISTOGRAMMODE_REFERENCE_ONLY
     *                                HISTOGRAMMODE_RETURN_ONLY
     *                                HISTOGRAMMODE_BOTH
     *
     * @return  false
     */
        bool SetHistogramMode(HistogramModes HistogramMode) {
          VL53L0X_NYI(false);
        }

    /**
     * @brief  Get current new device mode
     * @par Function Description
     * Get current Histogram mode of a Device
     *
     * @note This function doesn't Access to the device
     *
     * @return  histogram mode, which at present is always disabled
     */
        HistogramModes GetHistogramMode() {
          return HISTOGRAMMODE_DISABLED;//NYI
        }

    /**
     * @brief Retrieve the measurements from device for a given setup
     *
     * @par Function Description
     * Get data from last successful Histogram measurement
     * @warning USER should take care about  @a GetNumberOfROIZones()
     * before get data.
     * PAL will fill a NumberOfROIZones times the corresponding data structure
     * used in the measurement function.
     *
     * @note This function is not Implemented
     *
     * @param   pHistogramMeasurementData   Pointer to the histogram data structure.
     * @return  false
     */
        bool GetHistogramMeasurementData(HistogramMeasurementData_t &pHistogramMeasurementData) {
          VL53L0X_NYI(false);
        }

    /**
     * @brief Performs a single histogram measurement and retrieve the histogram measurement data
     *   Is equivalent to PerformSingleMeasurement +  GetHistogramMeasurementData
     *
     * @par Function Description
     * Get data from last successful Ranging measurement.
     * This function will clear the interrupt in case of these are enabled.
     *
     * @note This function is not Implemented
     *
     * @param   pHistogramMeasurementData  Pointer to the data structure to fill up.
     * @return  false
     */
        bool PerformSingleHistogramMeasurement(HistogramMeasurementData_t &pHistogramMeasurementData) {
          VL53L0X_NYI(false)
        }

    /**
     * @brief Perform XTalk Measurement
     *
     * @details Measures the current cross talk from glass in front
     * of the sensor.
     * This functions performs a histogram measurement and uses the results
     * to measure the crosstalk. For the function to be successful, there
     * must be no target in front of the sensor.
     *
     * @warning This function is a blocking function
     *
     * @warning This function is not supported when the final range
     * vcsel clock period is set below 10 PCLKS.
     *
     * @note This function Access to the device
     *
     * @param   TimeoutMs            Histogram measurement duration.
     * @param   pXtalkPerSpad        Output parameter containing the crosstalk
     * measurement result, in MCPS/Spad. Format fixpoint 16:16.
     * @param   pAmbientTooHigh      Output parameter which indicate that
     * pXtalkPerSpad is not good if the Ambient is too high.
     * @return  ERROR_NONE    Success
     * @return  ERROR_INVALID_PARAMS vcsel clock period not supported
     * for this operation. Must not be less than 10PCLKS.
     * @return  "Other error code"   See ::Error
     */
        bool PerformXTalkMeasurement(uint32_t TimeoutMs, FixPoint1616_t *pXtalkPerSpad, uint8_t *pAmbientTooHigh) {
          VL53L0X_NYI(false);
    //similar name but quite different technique from      perform_xtalk_calibration()
        }
#endif
/**
 * @brief Performs a single ranging measurement and retrieve the ranging measurement data
 *
 * @par Function Description
 * This function will change the device mode to
 * DEVICEMODE_SINGLE_RANGING with @a SetDeviceMode(), It
 * performs measurement with @a PerformSingleMeasurement() It get data
 * from last successful Ranging measurement with
 * @a GetRangingMeasurementData.
 * Finally it clear the interrupt with @a ClearInterruptMask().
 *
 * @note This function Access to the device
 *
 * @note This function change the device mode to DEVICEMODE_SINGLE_RANGING
 *
 * @warning blocking
 *
 * @param   pRangingMeasurementData   Pointer to the data structure to fill up.
 * @return   Success
 */
    bool PerformSingleRangingMeasurement(RangingMeasurementData_t &pRangingMeasurementData);

#if HaveRoiZones
    /**
     * @brief Set the number of ROI Zones to be used for a specific Device
     *
     * @par Function Description
     * Set the number of ROI Zones to be used for a specific Device.
     * The programmed value should be less than the max number of ROI Zones given
     * with @a GetMaxNumberOfROIZones().
     * This version of API manage only one zone.
     *

     * @param   NumberOfROIZones              Number of ROI Zones to be used for a
     *  specific Device.
     * @return  ERROR_NONE             Success
     * @return  ERROR_INVALID_PARAMS   This error is returned if
     * NumberOfROIZones != 1
     */
        bool SetNumberOfROIZones(uint8_t NumberOfROIZones);

    /**
     * @brief Get the number of ROI Zones managed by the Device
     *
     * @par Function Description
     * Get number of ROI Zones managed by the Device
     * USER should take care about  @a GetNumberOfROIZones()
     * before get data after a perform measurement.
     * PAL will fill a NumberOfROIZones times the corresponding data
     * structure used in the measurement function.
     *
     * @note This function doesn't Access to the device
     *

     * @param   pNumberOfROIZones     Pointer to the Number of ROI Zones value.
     * @return  ERROR_NONE     Success
     */
        unsigned GetNumberOfROIZones();

    /**
     * @brief Get the Maximum number of ROI Zones managed by the Device
     *
     * @par Function Description
     * Get Maximum number of ROI Zones managed by the Device.
     *
     * @note This function doesn't Access to the device
     *

     * @param   pMaxNumberOfROIZones   Pointer to the Maximum Number
     *  of ROI Zones value.
     * @return  ERROR_NONE      Success
     */
        unsigned GetMaxNumberOfROIZones();
#endif

/** @} measurement_group */

/** @defgroup interrupt_group VL53L0X Interrupt Functions
 *  @brief    Functions used for interrupt managements
 *  @{
 */

/**
 * @brief Set the configuration of GPIO pin for a given device
 *
 * @note This function Access to the device
 *

 * @param   Pin                   ID of the GPIO Pin MUST BE 0 at present
 * gpioConfig containes
 * Select Pin functionality.
 * Device Mode associated to the Gpio.
 * Active high or active low interrupt see ::InterruptPolarity
 *
 * @return    Success
 *
 * @throws  ERROR_GPIO_NOT_EXISTING               Only Pin=0 is accepted.
 * @throws  ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED    when Functionality programmed is not in the supported list:
 *                             Supported value are:
 *                             GPIOFUNCTIONALITY_OFF,
 *                             GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW,
 *                             GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH,
 *                             GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT,
 *                             GPIOFUNCTIONALITY_NEW_MEASURE_READY
 */
    bool SetGpioConfig(uint8_t Pin, GpioConfiguration gpioConfig);

/**
 * @brief Get current configuration for GPIO pin for a given device
 *
 * @note This function Access to the device
 *
 * @return  collection of values read from the device
 */
    GpioConfiguration GetGpioConfig(uint8_t Pin = 0);

    /** yet another handy tuple. May import a range class from other libraries,*/
    struct RangeWindow {
      FixPoint1616_t Low;
      FixPoint1616_t High;
    };

/**
 * @brief Set low and high Interrupt thresholds for a given mode
 * (ranging, ALS, ...) for a given device
 *
 * @par Function Description
 * Set low and high Interrupt thresholds for a given mode (ranging, ALS, ...)
 *
 * @note This function Access to the device
 *
 * @note DeviceMode is ignored for the current device
 *
 * @param   DeviceMode       ignored on present device
 * @param   Threshold    Low and high for compare to undocumented value, presumably mm
 */
    void SetInterruptThresholds(DeviceModes DeviceMode, RangeWindow Threshold);

/**
 * @brief  Get high and low Interrupt thresholds for a given mode
 *  (ranging, ALS, ...) for a given device
 *
 * @par Function Description
 * Get high and low Interrupt thresholds for a given mode (ranging, ALS, ...)
 * for a given device
 *
 * @note This function Access to the device
 *
 * @note DeviceMode is ignored for the current device
 *
 * @param   DeviceMode       ignored, arbitrary default added as most likely choice associated with feature.
 * @return  pair of threshold values
 */
    RangeWindow GetInterruptThresholds(DeviceModes DeviceMode = DeviceModes::DEVICEMODE_CONTINUOUS_RANGING);

/**
 * @brief Return device stop completion status
 *
 * @par Function Description
 * Returns stop completion status.
 * User shall call this function after a stop command
 *
 * @note This function Access to the device
 *
 * @return  undocumented stop value.
 */
    uint8_t GetStopCompletedStatus();

/**
 * @brief Clear given system interrupt condition
 *
 * @par Function Description
 * Clear given interrupt(s).
 *
 * @note This function Access to the device
 *
 * @param   ignored_InterruptMask        Mask of interrupts to clear
 * @return  Success of issuing command
 */
    bool ClearInterruptMask(unsigned int ignored_InterruptMask);

/**
 * @brief Return device interrupt status
 *
 * @par Function Description
 * Returns currently raised interrupts by the device.
 * User shall be able to activate/deactivate interrupts through
 * @a SetGpioConfig()
 *
 * @note This function Access to the device
 *
 * @note C code cleared bit 4 and 3, we added a boolean to throw if they are not zero
 *
 */
    uint8_t GetInterruptMaskStatus(bool throwRangeErrors);

/**
 * @brief Configure ranging interrupt reported to system
 *
 * @note This function is not Implemented
 *
 * @param   InterruptMask         Mask of interrupt to Enable/disable
 *  (0:interrupt disabled or 1: interrupt enabled)
 * @return  ERROR_NOT_IMPLEMENTED   Not implemented
 */
    bool EnableInterruptMask(uint8_t InterruptMask);

/** @} interrupt_group */

/** @defgroup SPADfunctions_group VL53L0X SPAD Functions
 *  @brief    Functions used for SPAD managements
 *  @{
 */

/**
 * @brief  Set the SPAD Ambient Damper Threshold value
 *
 * @par Function Description
 * This function set the SPAD Ambient Damper Threshold value
 *
 * @note This function Access to the device
 *
 * @param   SpadAmbientDamperThreshold    SPAD Ambient Damper Threshold value
 */
    void SetSpadAmbientDamperThreshold(uint16_t SpadAmbientDamperThreshold);

/**
 * @brief  Get the current SPAD Ambient Damper Threshold value
 *
 * @par Function Description
 * This function get the SPAD Ambient Damper Threshold value
 *
 * @note This function Access to the device
 *
 * @return  programmed SPAD Ambient Damper Threshold value
 */
    uint16_t GetSpadAmbientDamperThreshold();

/**
 * @brief  Set the SPAD Ambient Damper Factor value
 *
 * @par Function Description
 * This function set the SPAD Ambient Damper Factor value
 *
 * @note This function Access to the device
 *
 * @param   SpadAmbientDamperFactor       SPAD Ambient Damper Factor value
 */
    void SetSpadAmbientDamperFactor(uint16_t SpadAmbientDamperFactor);

/**
 * @brief  Get the current SPAD Ambient Damper Factor value
 *
 * @par Function Description
 * This function get the SPAD Ambient Damper Factor value
 *
 * @note This function Access to the device
 *
 * @param   pSpadAmbientDamperFactor      Pointer to programmed SPAD Ambient
 * Damper Factor value
 * @return  ERROR_NONE             Success
 * @return  "Other error code"            See ::Error
 */
    uint8_t GetSpadAmbientDamperFactor(); //ick: former code expanded to 16 bits locally. It is just 8 in the device, let us not hide that.

/**
 * @brief Performs Reference Spad Management
 *
 * @par Function Description
 * The reference SPAD initialization procedure determines the minimum amount
 * of reference spads to be enables to achieve a target reference signal rate
 * and should be performed once during initialization.
 *
 * @note This function Access to the device
 *
 * @note This function change the device mode to  DEVICEMODE_SINGLE_RANGING
 *
 * @return Success. Values are available in the PALData
 */
    bool PerformRefSpadManagement();

/**
 * @brief Applies Reference SPAD configuration
 *
 * @par Function Description
 * This function applies a given number of reference spads, identified as either Aperture or Non-Aperture.
 * The requested spad count and type are stored within the device specific parameters data for access by the host.
 *
 * @note This function does NOT access the device
 *
 * @param   refSpadCount                 Number of ref spads and isAperture
 */
    void SetReferenceSpads(SpadCount spad);

/**
 * @brief Retrieves SPAD configuration
 *
 * @par Function Description
 * This function retrieves the current number of applied reference spads
 * and also their type : Aperture or Non-Aperture.
 *
 * @note This function Access to the device
 *
 * @return  spad count and type
 */
    SpadCount GetReferenceSpads();

/** @} SPADfunctions_group */

/** @} cut11_group */

    bool check_part_used(uint8_t &Revision, DeviceInfo_t &pDeviceInfo);
    bool get_device_info(DeviceInfo_t &pDeviceInfo);

    bool perform_ref_spad_management();//#staying with reference parameter for error handling ease in one place

    SpadCount get_reference_spads();
    Api::CalibrationParameters get_ref_calibration();

  private: //calibration.h was high level actions hidden from direct use in api

    bool perform_offset_calibration(FixPoint1616_t CalDistanceMilliMeter);

    void set_offset_calibration_data_micro_meter(int32_t OffsetCalibrationDataMicroMeter);
    int32_t get_offset_calibration_data_micro_meter();

    bool apply_offset_adjustment();

    bool set_reference_spads(SpadCount ref);

    bool perform_phase_calibration(bool restore_config);
    bool perform_ref_calibration();

    void set_ref_calibration(CalibrationParameters p, bool setv, bool setp);

  protected:
    bool waitOnResetIndicator(bool disappear);

    static const SpadArray::Index startSelect;// was 0xB4 but is not a bit pattern, rather it is a decimal number

    static const unsigned minimumSpadCount = 3;

    void initRanger(VcselPeriod periodType, SequenceStepId stepId, DeviceSpecificParameters_t::RangeSetting &ranger);
    SpadArray::Index enable_ref_spads(const SpadCount &req, const SpadArray &goodSpadArray, SpadArray &spadArray, SpadArray::Index offset);

    bool CheckAndLoadInterruptSettings(bool StartNotStopFlag);//move to core?

    /** the factoring and rework relies upon registers CB and EE not being affected by the cal procedure for the other.
     * if that is not the case then their values will have to be captured between the acquisitions which trigger their updates.
     *
     * */
    bool perform_single_ref_calibration(uint8_t vhv_init_byte);

    bool perform_vhv_calibration(bool restore_config);
    /** @returns Cal parameters after running a calibration of one type or the other. Only one param will be meaningful */
    bool perform_item_calibration(bool vElseP, bool restore_config);

    /** @returns the rate if measurement succeeds else @param iffails (not sure what makes a good sentinal, 0 or ~0 ) */
    uint16_t perform_ref_signal_measurement(uint16_t iffails);
    bool perform_xtalk_calibration(FixPoint1616_t XTalkCalDistance);

    void set_threshold(RegSystem index, FixPoint1616_t ThresholdLow);
    FixPoint1616_t get_threshold(RegSystem index);
  };
}//end namespace
#endif /* __H_ */
