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


//if the core isn't using it then move it to this class #include "vl53l0x_def.h"
//the platform is below the core #include "vl53l0x_platform.h"
#include "vl53l0x_api_core.h" //extends this
#include "vl53l0x_platform_log.h"

#define  VL53L0X_NYI   return LOG_ERROR(ERROR_NOT_IMPLEMENTED);

namespace VL53L0X {

  /** division of Api and Core seems arbitrary. Might make Core a member to hide it if that was the intent of the orignal API developers. */
  class Api : public Core {
  public:
    /** once the comm member of the Physical interface is given an abstract interface we can build one and pass it in here instead of passing in its constructor args.
     * before that we can make a macro for the arg list so that we don't have to change it at the three levels ...*/
    //instead of passing the device handle into (nearly) every function place it on the object, reduces visual clutter. 
    Api(Arg &&args) : Core(std::forward<Arg>(args)) {
      //do nothing so that we may static construct.
    }

/**
 * @brief Return the VL53L0X PAL Implementation Version
 *
 * @note This function doesn't access to the device
*/
    static const Version_t ImplementationVersion;

    static const Version_t PalSpecVersion;


/** @defgroup cut11_group VL53L0X cut1.1 Function Definition
 *  @brief    VL53L0X cut1.1 Function Definition
 *  @{
 */

/** @defgroup general_group VL53L0X General Functions
 *  @brief    General functions and definitions
 *  @{
 */

/**
 * @brief Reads the Product Revision for a for given Device
 * This function can be used to distinguish cut1.0 from cut1.1.
 *
 * @note This function Access to the device
 *
 * @param   Dev                 Device Handle
 * @param   pProductRevisionMajor  Pointer to Product Revision major
 * for a given Device
 * @param   pProductRevisionMinor  Pointer to Product Revision minor
 * for a given Device
 * @return  ERROR_NONE      Success
 * @return  "Other error code"  See ::Error
 */

/**
 * @brief Reads the Device information for given Device
 *
 * @note This function Access to the device
 *
 * @param   Dev                 Device Handle
 * @param   pDeviceInfo  Pointer to current device info for a given
 *  Device
 * @return  ERROR_NONE   Success
 * @return  "Other error code"  See ::Error
 */
    Error GetDeviceInfo(DeviceInfo_t *pDeviceInfo);

/**
 * @brief Read current status of the error register for the selected device
 *
 * @note This function Access to the device
 *
 * @param   Dev                   Device Handle
 * @param   pDeviceErrorStatus    Pointer to current error code of the device
 * @return  ERROR_NONE     Success
 * @return  "Other error code"    See ::Error
 */
    Erroneous<DeviceError> GetDeviceErrorStatus();

/**
 * @brief Human readable Range Error string for a given RangeStatus
 *
 * @note This function doesn't access to the device
 *
 * @param   RangeStatus         The RangeStatus code as stored on a RangingMeasurementData_t
 * @return  pointer to const text.
 */

    const char *GetRangeStatusString(uint8_t RangeStatus);

/**
 * @brief Human readable error string for a given Error Code

 * @note This function doesn't access to the device
 *
 * @param   ErrorCode           The error code as stored on  ::DeviceError
 * @return  pointer to const string
 */
    const char * GetDeviceErrorString(DeviceError ErrorCode);

/**
 * @brief Human readable error string for current PAL error status
 *
 * @note This function doesn't access to the device
 *
 * @param   PalErrorCode       The error code as stored on @a Error
 * @return  The error string corresponding to the PalErrorCode
 */
    const char * GetPalErrorString(Error PalErrorCode);

/**
 * @brief Human readable PAL State string
 *
 * @note This function doesn't access to the device
 *
 * @param   PalStateCode          The State code as stored on @a State
 * PalStateCode
 * @return pointer to const text.
 */
    const char * GetPalStateString(State PalStateCode);

/**
 * @brief Reads the internal state of the PAL for a given Device
 *
 * @note This function doesn't access to the device
 *
 * @param   Dev                   Device Handle
 * @param   pPalState             Pointer to current state of the PAL for a given Device
 * @return  ERROR_NONE     Success
 * @return  "Other error code"    See ::Error
 */
    State GetPalState();

/**
 * @brief Set the power mode for a given Device
 * The power mode can be Standby or Idle. Different level of both Standby and
 * Idle can exists.
 * This function should not be used when device is in Ranging state.
 *
 * @note This function Access to the device
 *
 * @param   Dev                   Device Handle
 * @param   PowerMode             The value of the power mode to set.
 * see ::PowerModes
 *                                Valid values are:
 *                                POWERMODE_STANDBY_LEVEL1,
 *                                POWERMODE_IDLE_LEVEL1
 * @return  ERROR_NONE                  Success
 * @return  ERROR_MODE_NOT_SUPPORTED    This error occurs when PowerMode
 * is not in the supported list
 * @return  "Other error code"    See ::Error
 */
    Error SetPowerMode(PowerModes PowerMode);

/**
 * @brief Get the power mode for a given Device
 *
 * @note This function Access to the device
 *
 * @param   Dev                   Device Handle
 * @param   pPowerMode            Pointer to the current value of the power
 * mode. see ::PowerModes
 *                                Valid values are:
 *                                POWERMODE_STANDBY_LEVEL1,
 *                                POWERMODE_IDLE_LEVEL1
 * @return  ERROR_NONE     Success
 * @return  "Other error code"    See ::Error
 */
    Erroneous<PowerModes> GetPowerMode( );

/**
 * Set or over-hide part to part calibration offset
 * \sa DataInit()   GetOffsetCalibrationDataMicroMeter()
 *
 * @note This function Access to the device
 *
 * @param   Dev                                Device Handle
 * @param   OffsetCalibrationDataMicroMeter    Offset (microns)
 * @return  ERROR_NONE                  Success
 * @return  "Other error code"                 See ::Error
 */
    Error SetOffsetCalibrationDataMicroMeter(int32_t OffsetCalibrationDataMicroMeter);

/**
 * @brief Get part to part calibration offset
 *
 * @par Function Description
 * Should only be used after a successful call to @a DataInit to backup
 * device NVM value
 *
 * @note This function Access to the device
 *
 * @param   Dev                                Device Handle
 * @param   pOffsetCalibrationDataMicroMeter   Return part to part
 * calibration offset from device (microns)
 * @return  ERROR_NONE                  Success
 * @return  "Other error code"                 See ::Error
 */
    Erroneous<int32_t> GetOffsetCalibrationDataMicroMeter( );

/**
 * Set the linearity corrective gain
 *
 * @note This function Access to the device
 *
 * @param   Dev                                Device Handle
 * @param   LinearityCorrectiveGain            Linearity corrective
 * gain in x1000
 * if value is 1000 then no modification is applied.
 * @return  ERROR_NONE                  Success
 * @return  "Other error code"                 See ::Error
 */
    Error SetLinearityCorrectiveGain(int16_t LinearityCorrectiveGain);

/**
 * @brief Get the linearity corrective gain
 *
 * @par Function Description
 * Should only be used after a successful call to @a DataInit to cache device NVM value
 *
 * @note This function did NOT Access to the device
 *
 * @return linearity  corrective gain in x1000
 */
    uint16_t  GetLinearityCorrectiveGain();

/**
 * Set Group parameter Hold state
 *
 * @par Function Description
 * Set or remove device internal group parameter hold
 *
 * @note This function is not Implemented
 *
 * @param   Dev      Device Handle
 * @param   GroupParamHold   Group parameter Hold state to be set (on/off)
 * @return  ERROR_NOT_IMPLEMENTED        Not implemented
 */
    Error SetGroupParamHold(uint8_t GroupParamHold);

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
 * @note This function is not Implemented
 *
 * @param   Dev      Device Handle
 * @param   pUpperLimitMilliMeter   The maximal range limit for actual setup
 * (in millimeter)
 * @return  ERROR_NOT_IMPLEMENTED        Not implemented
 */
    Error GetUpperLimitMilliMeter(uint16_t *pUpperLimitMilliMeter);

/**
 * @brief Get the Total Signal Rate
 * @par Function Description
 * This function will return the Total Signal Rate after a good ranging is done.
 *
 * @note This function access to Device
 *
 * @param   Dev      Device Handle
 * @param   pTotalSignalRate   Total Signal Rate value in Mega count per second
 * @return  ERROR_NONE     Success
 * @return  "Other error code"    See ::Error
 */
    Error GetTotalSignalRate(FixPoint1616_t *pTotalSignalRate);

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
 * When a single device us used, there is no need to call this function.
 *
 * @note This function Access to the device
 *
 * @param   Dev                   Device Handle
 * @param   DeviceAddress         The new Device address
 * @return  ERROR_NONE     Success
 * @return  "Other error code"    See ::Error
 */
    Error SetDeviceAddress(uint8_t DeviceAddress);

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
 *
 * @param   Dev                   Device Handle
 * @return  ERROR_NONE     Success
 * @return  "Other error code"    See ::Error
 */
    Error DataInit();

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
 * @param   Dev                             Device Handle
 * @param   pTuningSettingBuffer            Pointer to tuning settings buffer.
 * @param   UseInternalTuningSettings       Use internal tuning settings value.
 * @return  ERROR_NONE     Success
 * @return  "Other error code"    See ::Error
 */
    Error SetTuningSettingBuffer(const uint8_t *pTuningSettingBuffer, bool UseInternalTuningSettings);

/**
 * @brief Get the tuning settings pointer and the internal external switch value.
 *
 * This function is used to get the Tuning settings buffer pointer if not set to internal.
 *
 * @note This function DOES NOT Access to the device
 *
 * @return  pointer if not 'use internal' else nullptr
 */
    const uint8_t *GetTuningSettingBuffer();//todo: bool to get the internal one regardless of enable

/**
 * @brief Do basic device init (and eventually patch loading)
 * This function will change the State from
 * STATE_WAIT_STATICINIT to STATE_IDLE.
 * In this stage all default setting will be applied.
 *
 * @note This function Access to the device
 *
 * @param   Dev                   Device Handle
 * @return  ERROR_NONE     Success
 * @return  "Other error code"    See ::Error
 */
    Error StaticInit();

/**
 * @brief Wait for device booted after chip enable (hardware standby)
 * This function can be run only when State is STATE_POWERDOWN.
 *
 * @note This function is not Implemented
 *
 * @param   Dev      Device Handle
 * @return  ERROR_NOT_IMPLEMENTED Not implemented
 *
 */
    Error WaitDeviceBooted(){
      VL53L0X_NYI;
    }

/**
 * @brief Do an hard reset or soft reset (depending on implementation) of the
 * device \nAfter call of this function, device must be in same state as right
 * after a power-up sequence.This function will change the State to
 * STATE_POWERDOWN.
 *
 * @note This function Access to the device
 * @note  This function blacks until reset is successful.
 *
 * @param   Dev                   Device Handle
 * @return  ERROR_NONE     Success
 * @return  "Other error code"    See ::Error
 */
    Error ResetDevice();

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
 * @param   Dev                   Device Handle
 * @param   pDeviceParameters     Pointer to store current device parameters.
 * @return  ERROR_NONE     Success
 * @return  "Other error code"    See ::Error
 */
    Error SetDeviceParameters(const DeviceParameters_t &pDeviceParameters);

/**
 * @brief  Retrieve current device parameters
 * @par Function Description
 * Get actual parameters of the device
 * @li Then start ranging operation.
 *
 * @note This function Access to the device
 *
 * @param   Dev                   Device Handle
 * @param   pDeviceParameters     Pointer to store current device parameters.
 * @return  ERROR_NONE     Success
 * @return  "Other error code"    See ::Error
 */
    Erroneous<DeviceParameters_t> GetDeviceParameters( );

/**
 * @brief  Set a new device mode
 * @par Function Description
 * Set device to a new mode (ranging, histogram ...)
 *
 * @note This function doesn't Access to the device
 *
 * @param   Dev                   Device Handle
 * @param   DeviceMode            New device mode to apply
 *                                Valid values are:
 *                                DEVICEMODE_SINGLE_RANGING
 *                                DEVICEMODE_CONTINUOUS_RANGING
 *                                DEVICEMODE_CONTINUOUS_TIMED_RANGING
 *                                DEVICEMODE_SINGLE_HISTOGRAM
 *                                HISTOGRAMMODE_REFERENCE_ONLY
 *                                HISTOGRAMMODE_RETURN_ONLY
 *                                HISTOGRAMMODE_BOTH
 *
 *
 * @return  ERROR_NONE               Success
 * @return  ERROR_MODE_NOT_SUPPORTED This error occurs when DeviceMode
 * is not in the supported list
 */
    Error SetDeviceMode(DeviceModes DeviceMode);

/**
 * @brief  Get current new device mode
 * @par Function Description
 * Get actual mode of the device(ranging, histogram ...)
 *
 * @note This function doesn't Access to the device
 *
 * @param   Dev                   Device Handle
 * @param   pDeviceMode           Pointer to current apply mode value
 *                                Valid values are:
 *                                DEVICEMODE_SINGLE_RANGING
 *                                DEVICEMODE_CONTINUOUS_RANGING
 *                                DEVICEMODE_CONTINUOUS_TIMED_RANGING
 *                                DEVICEMODE_SINGLE_HISTOGRAM
 *                                HISTOGRAMMODE_REFERENCE_ONLY
 *                                HISTOGRAMMODE_RETURN_ONLY
 *                                HISTOGRAMMODE_BOTH
 *
 * @return  ERROR_NONE                   Success
 * @return  ERROR_MODE_NOT_SUPPORTED     This error occurs when
 * DeviceMode is not in the supported list
 */
    DeviceModes GetDeviceMode();

/**
 * @brief  Sets the resolution of range measurements.
 * @par Function Description
 * Set resolution of range measurements to either 0.25mm if
 * fraction enabled or 1mm if not enabled.
 *
 * @note This function Accesses the device
 *
 * @param   Dev               Device Handle
 * @param   Enable            Enable high resolution
 *
 * @return  ERROR_NONE               Success
 * @return  "Other error code"              See ::Error
 */
    Error SetRangeFractionEnable(bool Enable);

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
 * @param   Dev               Device Handle
 * @param   pEnable           Output Parameter reporting the fraction enable
 * state.
 *
 * @return  ERROR_NONE                   Success
 * @return  "Other error code"                  See ::Error
 */
    Erroneous<bool> GetFractionEnable();

/**
 * @brief  Set a new Histogram mode
 * @par Function Description
 * Set device to a new Histogram mode
 *
 * @note This function doesn't Access to the device
 *
 * @param   Dev                   Device Handle
 * @param   HistogramMode         New device mode to apply
 *                                Valid values are:
 *                                HISTOGRAMMODE_DISABLED
 *                                DEVICEMODE_SINGLE_HISTOGRAM
 *                                HISTOGRAMMODE_REFERENCE_ONLY
 *                                HISTOGRAMMODE_RETURN_ONLY
 *                                HISTOGRAMMODE_BOTH
 *
 * @return  ERROR_NONE                   Success
 * @return  ERROR_MODE_NOT_SUPPORTED     This error occurs when
 * HistogramMode is not in the supported list
 * @return  "Other error code"    See ::Error
 */
    Error SetHistogramMode(HistogramModes HistogramMode){
      VL53L0X_NYI
    }

/**
 * @brief  Get current new device mode
 * @par Function Description
 * Get current Histogram mode of a Device
 *
 * @note This function doesn't Access to the device
 *
 * @param   Dev                   Device Handle
 * @param   pHistogramMode        Pointer to current Histogram Mode value
 *                                Valid values are:
 *                                HISTOGRAMMODE_DISABLED
 *                                DEVICEMODE_SINGLE_HISTOGRAM
 *                                HISTOGRAMMODE_REFERENCE_ONLY
 *                                HISTOGRAMMODE_RETURN_ONLY
 *                                HISTOGRAMMODE_BOTH
 * @return  ERROR_NONE     Success
 * @return  "Other error code"    See ::Error
 */
    HistogramModes GetHistogramMode(){
      return HISTOGRAMMODE_DISABLED;//NYI
    }

/**
 * @brief Set Ranging Timing Budget in microseconds
 *
 * @par Function Description
 * Defines the maximum time allowed by the user to the device to run a
 * full ranging sequence for the current mode (ranging, histogram, ASL ...)
 *
 * @note This function Access to the device
 *
 * @param   Dev                                Device Handle
 * @param MeasurementTimingBudgetMicroSeconds  Max measurement time in
 * microseconds.
 *                                   Valid values are:
 *                                   >= 17000 microsecs when wraparound enabled
 *                                   >= 12000 microsecs when wraparound disabled
 * @return  ERROR_NONE             Success
 * @return  ERROR_INVALID_PARAMS   This error is returned if
 *  MeasurementTimingBudgetMicroSeconds out of range
 * @return  "Other error code"            See ::Error
 */
    Error SetMeasurementTimingBudgetMicroSeconds(uint32_t MeasurementTimingBudgetMicroSeconds){
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
 * @param   Dev                                    Device Handle
 * @param   pMeasurementTimingBudgetMicroSeconds   Max measurement time in
 * microseconds.
 *                                   Valid values are:
 *                                   >= 17000 microsecs when wraparound enabled
 *                                   >= 12000 microsecs when wraparound disabled
 * @return  ERROR_NONE                      Success
 * @return  "Other error code"                     See ::Error
 */
    Erroneous<uint32_t> GetMeasurementTimingBudgetMicroSeconds( );

/**
 * @brief Gets the VCSEL pulse period.
 *
 * @par Function Description
 * This function retrieves the VCSEL pulse period for the given period type.
 *
 * @note This function Accesses the device
 *
 * @param   Dev                      Device Handle
 * @param   VcselPeriodType          VCSEL period identifier (pre-range|final).
 * @param   pVCSELPulsePeriod        Pointer to VCSEL period value.
 * @return  ERROR_NONE        Success
 * @return  ERROR_INVALID_PARAMS  Error VcselPeriodType parameter not
 *                                       supported.
 * @return  "Other error code"           See ::Error
 */
    Erroneous<uint8_t> GetVcselPulsePeriod(VcselPeriod VcselPeriodType);

/**
 * @brief Sets the VCSEL pulse period.
 *
 * @par Function Description
 * This function retrieves the VCSEL pulse period for the given period type.
 *
 * @note This function Accesses the device
 *
 * @param   Dev                       Device Handle
 * @param   VcselPeriodType	      VCSEL period identifier (pre-range|final).
 * @param   VCSELPulsePeriod          VCSEL period value
 * @return  ERROR_NONE            Success
 * @return  ERROR_INVALID_PARAMS  Error VcselPeriodType parameter not
 *                                       supported.
 * @return  "Other error code"           See ::Error
 */
    Error SetVcselPulsePeriod(VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriod);

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
 *                                       is enabled.
 * @return  ERROR_NONE            Success
 * @return  ERROR_INVALID_PARAMS  Error SequenceStepId parameter not
 *                                       supported.
 * @return  "Other error code"           See ::Error
 */
    Error SetSequenceStepEnable(SequenceStepId SequenceStepId, bool SequenceStepEnabled);

/**
 * @brief Gets the (on/off) state of a requested sequence step.
 *
 * @par Function Description
 * This function retrieves the state of a requested sequence step, i.e. on/off.
 *
 * @note This function Accesses the device
 *
 * @param   Dev                    Device Handle
 * @param   SequenceStepId         Sequence step identifier.
 * @param   pSequenceStepEnabled   Out parameter reporting if the sequence step
 *                                 is enabled {0=Off,1=On}.
 * @return  ERROR_NONE            Success
 * @return  ERROR_INVALID_PARAMS  Error SequenceStepId parameter not
 *                                       supported.
 * @return  "Other error code"           See ::Error
 */
    Erroneous<bool> GetSequenceStepEnable(SequenceStepId StepId);

/**
 * @brief Gets the (on/off) state of all sequence steps.
 *
 * @par Function Description
 * This function retrieves the state of all sequence step in the scheduler.
 *
 * @note This function Accesses the device
 *
 * @param   Dev                          Device Handle
 * @param   pSchedulerSequenceSteps      Pointer to struct containing result.
 * @return  ERROR_NONE            Success
 * @return  "Other error code"           See ::Error
 */
    Erroneous<SchedulerSequenceSteps_t> GetSequenceStepEnables();

/**
 * @brief Sets the timeout of a requested sequence step.
 *
 * @par Function Description
 * This function sets the timeout of a requested sequence step.
 *
 * @note This function Accesses the device
 *
 * @param   Dev                          Device Handle
 * @param   SequenceStepId               Sequence step identifier.
 * @param   TimeOutMilliSecs             Demanded timeout
 * @return  ERROR_NONE            Success
 * @return  ERROR_INVALID_PARAMS  Error SequenceStepId parameter not
 *                                       supported.
 * @return  "Other error code"           See ::Error
 */
    Error SetSequenceStepTimeout(SequenceStepId SequenceStepId, FixPoint1616_t TimeOutMilliSecs);

/**
 * @brief Gets the timeout of a requested sequence step.
 *
 * @par Function Description
 * This function retrieves the timeout of a requested sequence step.
 *
 * @note This function Accesses the device
 *
 * @param   Dev                          Device Handle
 * @param   SequenceStepId               Sequence step identifier.
 * @param   pTimeOutMilliSecs            Timeout value.
 * @return  ERROR_NONE            Success
 * @return  ERROR_INVALID_PARAMS  Error SequenceStepId parameter not
 *                                       supported.
 * @return  "Other error code"           See ::Error
 */
    Erroneous<FixPoint1616_t> GetSequenceStepTimeout(SequenceStepId SequenceStepId);

/**
 * @brief Gets number of sequence steps managed by the API.
 *
 * @par Function Description
 * This function retrieves the number of sequence steps currently managed
 * by the API
 *
 * @note This function Accesses the device
 *
 * @param   Dev                          Device Handle
 * @param   pNumberOfSequenceSteps       Out parameter reporting the number of
 *                                       sequence steps.
 * @return  ERROR_NONE            Success
 * @return  "Other error code"           See ::Error
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
    const char * GetSequenceStepsInfo(SequenceStepId SequenceStepId);

/**
 * Program continuous mode Inter-Measurement period in milliseconds
 *
 * @par Function Description
 * When trying to set too short time return  INVALID_PARAMS minimal value
 *
 * @note This function Access to the device
 *
 * @param   Dev                                  Device Handle
 * @param   InterMeasurementPeriodMilliSeconds   Inter-Measurement Period in ms.
 * @return  ERROR_NONE                    Success
 * @return  "Other error code"                   See ::Error
 */
    Error SetInterMeasurementPeriodMilliSeconds(uint32_t InterMeasurementPeriodMilliSeconds);

/**
 * Get continuous mode Inter-Measurement period in milliseconds
 *
 * @par Function Description
 * When trying to set too short time return  INVALID_PARAMS minimal value
 *
 * @note This function Access to the device
 *
 * @param   Dev                                  Device Handle
 * @param   pInterMeasurementPeriodMilliSeconds  Pointer to programmed
 *  Inter-Measurement Period in milliseconds.
 * @return  ERROR_NONE                    Success
 * @return  "Other error code"                   See ::Error
 */
    Erroneous<uint32_t> GetInterMeasurementPeriodMilliSeconds();

/**
 * @brief Enable/Disable Cross talk compensation feature
 *
 * @note This function is not Implemented.
 * Enable/Disable Cross Talk by set to zero the Cross Talk value
 * by using @a SetXTalkCompensationRateMegaCps().
 *
 * @param   Dev                       Device Handle
 * @param   XTalkCompensationEnable   Cross talk compensation
 *  to be set 0=disabled else = enabled
 * @return  ERROR_NOT_IMPLEMENTED   Not implemented
 */
    Error SetXTalkCompensationEnable(bool XTalkCompensationEnable);

/**
 * @brief Get Cross talk compensation rate
 *
 * @note This function is not Implemented. (Says who!) //BUG: stale documentation
 * Enable/Disable Cross Talk by set to zero the Cross Talk value by
 * using @a SetXTalkCompensationRateMegaCps().
 *
 * @param   Dev                        Device Handle
 * @param   pXTalkCompensationEnable   Pointer to the Cross talk compensation
 *  state 0=disabled or 1 = enabled
 * @return  ERROR_NOT_IMPLEMENTED   Not implemented
 */
    bool GetXTalkCompensationEnable();

/**
 * @brief Set Cross talk compensation rate
 *
 * @par Function Description
 * Set Cross talk compensation rate.
 *
 * @note This function Access to the device
 *
 * @param   Dev                            Device Handle
 * @param   XTalkCompensationRateMegaCps   Compensation rate in
 *  Mega counts per second (16.16 fix point) see datasheet for details
 * @return  ERROR_NONE              Success
 * @return  "Other error code"             See ::Error
 */
    Error SetXTalkCompensationRateMegaCps(FixPoint1616_t XTalkCompensationRateMegaCps);

/**
 * @brief Get Cross talk compensation rate
 *
 * @par Function Description
 * Get Cross talk compensation rate.
 *
 * @note This function Access to the device
 *
 * @param   Dev                            Device Handle
 * @param   pXTalkCompensationRateMegaCps  Pointer to Compensation rate
 *  in Mega counts per second (16.16 fix point) see datasheet for details
 * @return  ERROR_NONE              Success
 * @return  "Other error code"             See ::Error
 */
    Erroneous<FixPoint1616_t> GetXTalkCompensationRateMegaCps( );

    struct CalibrationParameters{
      uint8_t VhvSettings;
      uint8_t PhaseCal;
    };

/**
 * @brief Set Reference Calibration Parameters
 *
 * @par Function Description
 * Set Reference Calibration Parameters.
 *
 * @note This function Access to the device
 *
 * @param   Dev                            Device Handle
 * @param   VhvSettings                    Parameter for VHV
 * @param   PhaseCal                       Parameter for PhaseCal
 * @return  ERROR_NONE              Success
 * @return  "Other error code"             See ::Error
 */
    Error SetRefCalibration(CalibrationParameters refParams);

/**
 * @brief Get Reference Calibration Parameters
 *
 * @par Function Description
 * Get Reference Calibration Parameters.
 *
 * @note This function Access to the device
 *
 * @param   Dev                            Device Handle
 * @param   pVhvSettings                   Pointer to VHV parameter
 * @param   pPhaseCal                      Pointer to PhaseCal Parameter
 * @return  ERROR_NONE              Success
 * @return  "Other error code"             See ::Error
 */
    Erroneous<CalibrationParameters> GetRefCalibration();

/**
 * @brief  Get the number of the check limit managed by a given Device
 *
 * @par Function Description
 * This function give the number of the check limit managed by the Device
 *
 * @note This function doesn't Access to the device
 *
 * @param   pNumberOfLimitCheck           Pointer to the number of check limit.
 * @return  ERROR_NONE             Success
 * @return  "Other error code"            See ::Error
 */
    CheckEnable GetNumberOfLimitCheck();

/**
 * @brief  Return a description string for a given limit check number
 *
 * @par Function Description
 * This function returns a description string for a given limit check number.
 * The limit check is identified with the LimitCheckId.
 *
 * @note This function doesn't Access to the device
 *
 * @param   Dev                           Device Handle
 * @param   LimitCheckId                  Limit Check ID
 *  (0<= LimitCheckId < GetNumberOfLimitCheck() ).
 * @param   pLimitCheckString             Pointer to the
 *  description string of the given check limit.
 * @return  ERROR_NONE             Success
 * @return  ERROR_INVALID_PARAMS   This error is
 *  returned when LimitCheckId value is out of range.
 * @return  "Other error code"            See ::Error
 */
    const char * GetLimitCheckInfo(CheckEnable LimitCheckId);

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
 * @param   Dev                           Device Handle
 * @param   LimitCheckId                  Limit Check ID
 *  (0<= LimitCheckId < GetNumberOfLimitCheck() ).
 * @param   pLimitCheckStatus             Pointer to the
 *  Limit Check Error of the given check limit.
 * LimitCheckStatus :
 * 0 the check is not fail
 * 1 the check if fail or not enabled
 *
 * @return  ERROR_NONE             Success
 * @return  ERROR_INVALID_PARAMS   This error is
 *  returned when LimitCheckId value is out of range.
 * @return  "Other error code"            See ::Error
 */
    Erroneous<bool> GetLimitCheckStatus(CheckEnable LimitCheckId);

/**
 * @brief  Enable/Disable a specific limit check
 *
 * @par Function Description
 * This function Enable/Disable a specific limit check.
 * The limit check is identified with the LimitCheckId.
 *
 * @note This function doesn't Access to the device
 *
 * @param   Dev                           Device Handle
 * @param   LimitCheckId                  Limit Check ID
 *  (0<= LimitCheckId < GetNumberOfLimitCheck() ).
 * @param   LimitCheckEnable              if 1 the check limit
 *  corresponding to LimitCheckId is Enabled
 *                                        if 0 the check limit
 *  corresponding to LimitCheckId is disabled
 * @return  ERROR_NONE             Success
 * @return  ERROR_INVALID_PARAMS   This error is returned
 *  when LimitCheckId value is out of range.
 * @return  "Other error code"            See ::Error
 */
    Error SetLimitCheckEnable(CheckEnable LimitCheckId, bool LimitCheckEnable);


/**
 * @brief  Set a specific limit check value
 *
 * @par Function Description
 * This function set a specific limit check value.
 * The limit check is identified with the LimitCheckId.
 *
 * @note This function Access to the device
 *
 * @param   Dev                           Device Handle
 * @param   LimitCheckId                  Limit Check ID
 *  (0<= LimitCheckId < GetNumberOfLimitCheck() ).
 * @param   LimitCheckValue               Limit check Value for a given
 * LimitCheckId
 * @return  ERROR_NONE             Success
 * @return  ERROR_INVALID_PARAMS   This error is returned when either
 *  LimitCheckId or LimitCheckValue value is out of range.
 * @return  "Other error code"            See ::Error
 */
    Error SetLimitCheckValue(CheckEnable LimitCheckId, FixPoint1616_t LimitCheckValue);



/**
 * @brief  Get the current value of the signal used for the limit check
 *
 * @par Function Description
 * This function get a the current value of the signal used for the limit check.
 * To obtain the latest value you should run a ranging before.
 * The value reported is linked to the limit check identified with the
 * LimitCheckId.
 *
 * @note This function Access to the device
 *
 * @param   Dev                           Device Handle
 * @param   LimitCheckId                  Limit Check ID
 *  (0<= LimitCheckId < GetNumberOfLimitCheck() ).
 * @param   pLimitCheckCurrent            Pointer to current Value for a
 * given LimitCheckId.
 * @return  ERROR_NONE             Success
 * @return  ERROR_INVALID_PARAMS   This error is returned when
 * LimitCheckId value is out of range.
 * @return  "Other error code"            See ::Error
 */
    Error GetLimitCheckCurrent(uint16_t LimitCheckId, FixPoint1616_t *pLimitCheckCurrent);

/**
 * @brief  Enable (or disable) Wrap around Check
 *
 * @note This function Access to the device
 *
 * @param   Dev                    Device Handle
 * @param   WrapAroundCheckEnable  Wrap around Check to be set
 *                                 0=disabled, other = enabled
 * @return  ERROR_NONE      Success
 * @return  "Other error code"     See ::Error
 */
    Error SetWrapAroundCheckEnable(bool WrapAroundCheckEnable);

/**
 * @brief  Get setup of Wrap around Check
 *
 * @par Function Description
 * This function get the wrapAround check enable parameters
 *
 * @note This function Access to the device
 *
 * @param   Dev                     Device Handle
 * @param   pWrapAroundCheckEnable  Pointer to the Wrap around Check state
 *                                  0=disabled or 1 = enabled
 * @return  ERROR_NONE       Success
 * @return  "Other error code"      See ::Error
 */
    Erroneous<bool> GetWrapAroundCheckEnable();

/**
 * @brief   Set Dmax Calibration Parameters for a given device
 * When one of the parameter is zero, this function will get parameter
 * from NVM.
 * @note This function doesn't Access to the device
 *
 * @param   Dev                    Device Handle
 * @param   RangeMilliMeter        Calibration Distance
 * @param   SignalRateRtnMegaCps   Signal rate return read at CalDistance
 * @return  ERROR_NONE      Success
 * @return  "Other error code"     See ::Error
 */
    Error SetDmaxCalParameters(const DevData_t::DmaxCal &p);

/**
 * @brief  Get Dmax Calibration Parameters for a given device
 *
 *
 * @note This function Access to the device
 *
 * @param   Dev                     Device Handle
 * @param   pRangeMilliMeter        Pointer to Calibration Distance
 * @param   pSignalRateRtnMegaCps   Pointer to Signal rate return
 * @return  ERROR_NONE       Success
 * @return  "Other error code"      See ::Error
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
 * @param   Dev                  Device Handle
 * @return  ERROR_NONE    Success
 * @return  "Other error code"   See ::Error
 */
    Error PerformSingleMeasurement();

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
 * @warning This function is a blocking function
 *
 * @note This function Access to the device
 *
 * @param   Dev                  Device Handle
 * @param   pVhvSettings         Pointer to vhv settings parameter.
 * @param   pPhaseCal            Pointer to PhaseCal parameter.
 * @return  ERROR_NONE    Success
 * @return  "Other error code"   See ::Error
 */
    Error PerformRefCalibration(CalibrationParameters *c);

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
 * @param   Dev                  Device Handle
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
    Error PerformXTalkMeasurement(uint32_t TimeoutMs, FixPoint1616_t *pXtalkPerSpad, uint8_t *pAmbientTooHigh);

/**
 * @brief Perform XTalk Calibration
 *
 * @details Perform a XTalk calibration of the Device.
 * This function will launch a ranging measurement, if interrupts
 * are enabled an interrupt will be done.
 * This function will clear the interrupt generated automatically.
 * This function will program a new value for the XTalk compensation
 * and it will enable the cross talk before exit.
 * This function will disable the CHECKENABLE_RANGE_IGNORE_THRESHOLD.
 *
 * @warning This function is a blocking function
 *
 * @note This function Access to the device
 *
 * @note This function change the device mode to
 * DEVICEMODE_SINGLE_RANGING
 *
 * @param   Dev                  Device Handle
 * @param   XTalkCalDistance     XTalkCalDistance value used for the XTalk
 * computation.
 * @param   pXTalkCompensationRateMegaCps  Pointer to new
 * XTalkCompensation value.
 * @return  ERROR_NONE    Success
 * @return  "Other error code"   See ::Error
 */
    Error PerformXTalkCalibration(FixPoint1616_t XTalkCalDistance, FixPoint1616_t *pXTalkCompensationRateMegaCps);

/**
 * @brief Perform Offset Calibration
 *
 * @details Perform a Offset calibration of the Device.
 * This function will launch a ranging measurement, if interrupts are
 * enabled an interrupt will be done.
 * This function will clear the interrupt generated automatically.
 * This function will program a new value for the Offset calibration value
 * This function will disable the CHECKENABLE_RANGE_IGNORE_THRESHOLD.
 *
 * @warning This function is a blocking function
 *
 * @note This function Access to the device
 *
 * @note This function does not change the device mode.
 *
 * @param   Dev                  Device Handle
 * @param   CalDistanceMilliMeter     Calibration distance value used for the
 * offset compensation.
 * @param   pOffsetMicroMeter  Pointer to new Offset value computed by the
 * function.
 *
 * @return  ERROR_NONE    Success
 * @return  "Other error code"   See ::Error
 */
    Error PerformOffsetCalibration(FixPoint1616_t CalDistanceMilliMeter, int32_t *pOffsetMicroMeter);

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
 * @param   Dev                  Device Handle
 * @return  ERROR_NONE                  Success
 * @return  ERROR_MODE_NOT_SUPPORTED    This error occurs when
 * DeviceMode programmed with @a SetDeviceMode is not in the supported
 * list:
 *                                   Supported mode are:
 *                                   DEVICEMODE_SINGLE_RANGING,
 *                                   DEVICEMODE_CONTINUOUS_RANGING,
 *                                   DEVICEMODE_CONTINUOUS_TIMED_RANGING
 * @return  ERROR_TIME_OUT    Time out on start measurement
 * @return  "Other error code"   See ::Error
 */
    Error StartMeasurement();

/**
 * @brief Stop device measurement
 *
 * @details Will set the device in standby mode at end of current measurement\n
 *          Not necessary in single mode as device shall return automatically
 *          in standby mode at end of measurement.
 *          This function will change the State from
 * STATE_RUNNING to STATE_IDLE.
 *
 * @note This function Access to the device
 *
 * @param   Dev                  Device Handle
 * @return  ERROR_NONE    Success
 * @return  "Other error code"   See ::Error
 */
    Error StopMeasurement();

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
    Erroneous<bool> GetMeasurementDataReady();


    /** formerly declared in core.h but implemented in api.cpp
     * @returns false on timeout */
    Erroneous<bool> measurement_poll_for_completion();


    /**
 * @brief Wait for device ready for a new measurement command.
 * Blocking function.
 *
 * @note This function is not Implemented
 *
 * @param   Dev      Device Handle
 * @param   MaxLoop    Max Number of polling loop (timeout).
 * @return  ERROR_NOT_IMPLEMENTED   Not implemented
 */
    Error WaitDeviceReadyForNewMeasurement(uint32_t MaxLoop);

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
 * @param   Dev                      Device Handle
 * @param   pMeasurementRefSignal    Pointer to the Ref Signal to fill up.
 * @return  ERROR_NONE        Success
 * @return  "Other error code"       See ::Error
 */
    Error GetMeasurementRefSignal(FixPoint1616_t *pMeasurementRefSignal);

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
 * @param   Dev                      Device Handle
 * @param   pRangingMeasurementData  Pointer to the data structure to fill up.
 * @return  ERROR_NONE        Success
 * @return  "Other error code"       See ::Error
 */
    Error GetRangingMeasurementData(RangingMeasurementData_t *pRangingMeasurementData);

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
 * @param   Dev                         Device Handle
 * @param   pHistogramMeasurementData   Pointer to the histogram data structure.
 * @return  ERROR_NOT_IMPLEMENTED   Not implemented
 */
    Error GetHistogramMeasurementData(HistogramMeasurementData_t *pHistogramMeasurementData);

/**
 * @brief Performs a single ranging measurement and retrieve the ranging
 * measurement data
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
 * @note This function change the device mode to
 * DEVICEMODE_SINGLE_RANGING
 *
 * @param   Dev                       Device Handle
 * @param   pRangingMeasurementData   Pointer to the data structure to fill up.
 * @return  ERROR_NONE         Success
 * @return  "Other error code"        See ::Error
 */
    Error PerformSingleRangingMeasurement(RangingMeasurementData_t *pRangingMeasurementData);

/**
 * @brief Performs a single histogram measurement and retrieve the histogram
 * measurement data
 *   Is equivalent to PerformSingleMeasurement +
 *   GetHistogramMeasurementData
 *
 * @par Function Description
 * Get data from last successful Ranging measurement.
 * This function will clear the interrupt in case of these are enabled.
 *
 * @note This function is not Implemented
 *
 * @param   Dev                        Device Handle
 * @param   pHistogramMeasurementData  Pointer to the data structure to fill up.
 * @return  ERROR_NOT_IMPLEMENTED   Not implemented
 */
    Error PerformSingleHistogramMeasurement(HistogramMeasurementData_t *pHistogramMeasurementData);

/**
 * @brief Set the number of ROI Zones to be used for a specific Device
 *
 * @par Function Description
 * Set the number of ROI Zones to be used for a specific Device.
 * The programmed value should be less than the max number of ROI Zones given
 * with @a GetMaxNumberOfROIZones().
 * This version of API manage only one zone.
 *
 * @param   Dev                           Device Handle
 * @param   NumberOfROIZones              Number of ROI Zones to be used for a
 *  specific Device.
 * @return  ERROR_NONE             Success
 * @return  ERROR_INVALID_PARAMS   This error is returned if
 * NumberOfROIZones != 1
 */
    Error SetNumberOfROIZones(uint8_t NumberOfROIZones);

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
 * @param   Dev                   Device Handle
 * @param   pNumberOfROIZones     Pointer to the Number of ROI Zones value.
 * @return  ERROR_NONE     Success
 */
    Error GetNumberOfROIZones(uint8_t *pNumberOfROIZones);

/**
 * @brief Get the Maximum number of ROI Zones managed by the Device
 *
 * @par Function Description
 * Get Maximum number of ROI Zones managed by the Device.
 *
 * @note This function doesn't Access to the device
 *
 * @param   Dev                    Device Handle
 * @param   pMaxNumberOfROIZones   Pointer to the Maximum Number
 *  of ROI Zones value.
 * @return  ERROR_NONE      Success
 */
    Error GetMaxNumberOfROIZones(uint8_t *pMaxNumberOfROIZones);

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
 * @param   Dev                   Device Handle
 * @param   Pin                   ID of the GPIO Pin
 * @param   Functionality         Select Pin functionality.
 *  Refer to ::GpioFunctionality
 * @param   DeviceMode            Device Mode associated to the Gpio.
 * @param   Polarity              Set interrupt polarity. Active high
 *   or active low see ::InterruptPolarity
 * @return  ERROR_NONE                            Success
 * @return  ERROR_GPIO_NOT_EXISTING               Only Pin=0 is
 *  accepted.
 * @return  ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED    This error occurs
 * when Functionality programmed is not in the supported list:
 *                             Supported value are:
 *                             GPIOFUNCTIONALITY_OFF,
 *                             GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW,
 *                             GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH,
 *  GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT,
 *                               GPIOFUNCTIONALITY_NEW_MEASURE_READY
 * @return  "Other error code"    See ::Error
 */
    Error SetGpioConfig(uint8_t Pin, DeviceModes DeviceMode, GpioFunctionality Functionality, InterruptPolarity Polarity);

/**
 * @brief Get current configuration for GPIO pin for a given device
 *
 * @note This function Access to the device
 *
 * @param   Dev                   Device Handle
 * @param   Pin                   ID of the GPIO Pin
 * @param   pDeviceMode           Pointer to Device Mode associated to the Gpio.
 * @param   pFunctionality        Pointer to Pin functionality.
 *  Refer to ::GpioFunctionality
 * @param   pPolarity             Pointer to interrupt polarity.
 *  Active high or active low see ::InterruptPolarity
 * @return  ERROR_NONE                            Success
 * @return  ERROR_GPIO_NOT_EXISTING               Only Pin=0 is
 * accepted.
 * @return  ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED   This error occurs
 * when Functionality programmed is not in the supported list:
 *                      Supported value are:
 *                      GPIOFUNCTIONALITY_OFF,
 *                      GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW,
 *                      GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH,
 *                      GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT,
 *                      GPIOFUNCTIONALITY_NEW_MEASURE_READY
 * @return  "Other error code"    See ::Error
 */
    Error GetGpioConfig(uint8_t Pin, DeviceModes *pDeviceMode, GpioFunctionality *pFunctionality, InterruptPolarity *pPolarity);

/**
 * @brief Set low and high Interrupt thresholds for a given mode
 * (ranging, ALS, ...) for a given device
 *
 * @par Function Description
 * Set low and high Interrupt thresholds for a given mode (ranging, ALS, ...)
 * for a given device
 *
 * @note This function Access to the device
 *
 * @note DeviceMode is ignored for the current device
 *
 * @param   Dev              Device Handle
 * @param   DeviceMode       Device Mode for which change thresholds
 * @param   ThresholdLow     Low threshold (mm, lux ..., depending on the mode)
 * @param   ThresholdHigh    High threshold (mm, lux ..., depending on the mode)
 * @return  ERROR_NONE    Success
 * @return  "Other error code"   See ::Error
 */
    Error SetInterruptThresholds(DeviceModes DeviceMode, FixPoint1616_t ThresholdLow, FixPoint1616_t ThresholdHigh);

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
 * @param   Dev              Device Handle
 * @param   DeviceMode       Device Mode from which read thresholds
 * @param   pThresholdLow    Low threshold (mm, lux ..., depending on the mode)
 * @param   pThresholdHigh   High threshold (mm, lux ..., depending on the mode)
 * @return  ERROR_NONE   Success
 * @return  "Other error code"  See ::Error
 */
    Error GetInterruptThresholds(DeviceModes DeviceMode, FixPoint1616_t *pThresholdLow, FixPoint1616_t *pThresholdHigh);

/**
 * @brief Return device stop completion status
 *
 * @par Function Description
 * Returns stop completiob status.
 * User shall call this function after a stop command
 *
 * @note This function Access to the device
 *
 * @param   Dev                    Device Handle
 * @param   pStopStatus            Pointer to status variable to update
 * @return  ERROR_NONE      Success
 * @return  "Other error code"     See ::Error
 */
    Error GetStopCompletedStatus(uint32_t *pStopStatus);

/**
 * @brief Clear given system interrupt condition
 *
 * @par Function Description
 * Clear given interrupt(s).
 *
 * @note This function Access to the device
 *
 * @param   Dev                  Device Handle
 * @param   InterruptMask        Mask of interrupts to clear
 * @return  ERROR_NONE    Success
 * @return  ERROR_INTERRUPT_NOT_CLEARED    Cannot clear interrupts
 *
 * @return  "Other error code"   See ::Error
 */
    Error ClearInterruptMask(uint32_t InterruptMask);

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
 * @param   Dev                    Device Handle
 * @param   pInterruptMaskStatus   Pointer to status variable to update
 * @return  ERROR_NONE      Success
 * @return  "Other error code"     See ::Error
 */
    Erroneous<uint8_t> GetInterruptMaskStatus();

/**
 * @brief Configure ranging interrupt reported to system
 *
 * @note This function is not Implemented
 *
 * @param   Dev                  Device Handle
 * @param   InterruptMask         Mask of interrupt to Enable/disable
 *  (0:interrupt disabled or 1: interrupt enabled)
 * @return  ERROR_NOT_IMPLEMENTED   Not implemented
 */
    Error EnableInterruptMask(uint32_t InterruptMask);

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
 * @param   Dev                           Device Handle
 * @param   SpadAmbientDamperThreshold    SPAD Ambient Damper Threshold value
 * @return  ERROR_NONE             Success
 * @return  "Other error code"            See ::Error
 */
    Error SetSpadAmbientDamperThreshold(uint16_t SpadAmbientDamperThreshold);

/**
 * @brief  Get the current SPAD Ambient Damper Threshold value
 *
 * @par Function Description
 * This function get the SPAD Ambient Damper Threshold value
 *
 * @note This function Access to the device
 *
 * @param   Dev                           Device Handle
 * @param   pSpadAmbientDamperThreshold   Pointer to programmed
 *                                        SPAD Ambient Damper Threshold value
 * @return  ERROR_NONE             Success
 * @return  "Other error code"            See ::Error
 */
    Error GetSpadAmbientDamperThreshold(uint16_t *pSpadAmbientDamperThreshold);

/**
 * @brief  Set the SPAD Ambient Damper Factor value
 *
 * @par Function Description
 * This function set the SPAD Ambient Damper Factor value
 *
 * @note This function Access to the device
 *
 * @param   Dev                           Device Handle
 * @param   SpadAmbientDamperFactor       SPAD Ambient Damper Factor value
 * @return  ERROR_NONE             Success
 * @return  "Other error code"            See ::Error
 */
    Error SetSpadAmbientDamperFactor(uint16_t SpadAmbientDamperFactor);

/**
 * @brief  Get the current SPAD Ambient Damper Factor value
 *
 * @par Function Description
 * This function get the SPAD Ambient Damper Factor value
 *
 * @note This function Access to the device
 *
 * @param   Dev                           Device Handle
 * @param   pSpadAmbientDamperFactor      Pointer to programmed SPAD Ambient
 * Damper Factor value
 * @return  ERROR_NONE             Success
 * @return  "Other error code"            See ::Error
 */
    Error GetSpadAmbientDamperFactor(uint16_t *pSpadAmbientDamperFactor);

    struct SpadInfo {
      unsigned count;
      bool isAperture;//are aperture?
    };

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
 * @note This function change the device mode to
 * DEVICEMODE_SINGLE_RANGING
 *
 * @param   Dev                          Device Handle
 * @param   refSpadCount                 Reports ref Spad Count
 * @param   isApertureSpads              Reports if spads are of type
 *                                       aperture or non-aperture.
 *                                       1:=aperture, 0:=Non-Aperture
 * @return  ERROR_NONE            Success
 * @return  ERROR_REF_SPAD_INIT   Error in the Ref Spad procedure.
 * @return  "Other error code"           See ::Error
 */
    Erroneous<SpadInfo> PerformRefSpadManagement();

/**
 * @brief Applies Reference SPAD configuration
 *
 * @par Function Description
 * This function applies a given number of reference spads, identified as
 * either Aperture or Non-Aperture.
 * The requested spad count and type are stored within the device specific
 * parameters data for access by the host.
 *
 * @note This function Access to the device
 *
 * @param   Dev                          Device Handle
 * @param   refSpadCount                 Number of ref spads.
 * @param   isApertureSpads              Defines if spads are of type
 *                                       aperture or non-aperture.
 *                                       1:=aperture, 0:=Non-Aperture
 * @return  ERROR_NONE            Success
 * @return  ERROR_REF_SPAD_INIT   Error in the in the reference
 *                                       spad configuration.
 * @return  "Other error code"           See ::Error
 */
    Error SetReferenceSpads(SpadInfo spad);

/**
 * @brief Retrieves SPAD configuration
 *
 * @par Function Description
 * This function retrieves the current number of applied reference spads
 * and also their type : Aperture or Non-Aperture.
 *
 * @note This function Access to the device
 *
 * @param   Dev                          Device Handle
 * @param   refSpadCount                 Number ref Spad Count
 * @param   isApertureSpads              Reports if spads are of type
 *                                       aperture or non-aperture.
 *                                       1:=aperture, 0:=Non-Aperture
 * @return  ERROR_NONE            Success
 * @return  ERROR_REF_SPAD_INIT   Error in the in the reference
 *                                       spad configuration.
 * @return  "Other error code"           See ::Error
 */
    Erroneous<SpadInfo> GetReferenceSpads();

/** @} SPADfunctions_group */

/** @} cut11_group */

    Erroneous <uint8_t> GetStopCompletedStatus();
    Error check_part_used(uint8_t &Revision, DeviceInfo_t &pDeviceInfo);
    Error get_device_info(DeviceInfo_t &pDeviceInfo);


    Error   perform_ref_spad_management(unsigned &refSpadCount, bool &isApertureSpads);

  private: //calibration.h was high level actions hidden from direct use in api
    Error perform_xtalk_calibration(FixPoint1616_t XTalkCalDistance, FixPoint1616_t &pXTalkCompensationRateMegaCps);

    Error perform_offset_calibration(FixPoint1616_t CalDistanceMilliMeter, int32_t *pOffsetMicroMeter);

    Error set_offset_calibration_data_micro_meter(int32_t OffsetCalibrationDataMicroMeter);
    Erroneous<int32_t> get_offset_calibration_data_micro_meter();

    Error apply_offset_adjustment();

    Error set_reference_spads(uint32_t count, uint8_t isApertureSpads);

    Error get_reference_spads(uint32_t *pSpadCount, uint8_t *pIsApertureSpads);

    Error perform_phase_calibration(uint8_t *pPhaseCal, const bool get_data_enable, const bool restore_config);

    Error perform_ref_calibration(CalibrationParameters &p, bool get_data_enable);

    Error set_ref_calibration(CalibrationParameters p);

    Erroneous<CalibrationParameters> get_ref_calibration();

    Erroneous <FixPoint1616_t> GetTotalSignalRate();
    Error waitOnResetIndicator( bool disappear);

    unsigned int GetMaxNumberOfROIZones();
    unsigned int GetNumberOfROIZones();

    static const unsigned startSelect = 180;// was 0xB4 but is not a bit pattern, rather it is a decimal number

    static const unsigned minimumSpadCount = 3;

    SpadArray::Index get_next_good_spad(SpadArray goodSpadArray, SpadArray::Index curr);

    Error initRanger(VcselPeriod periodType, SequenceStepId stepId, DeviceSpecificParameters_t::RangeSetting &ranger);
    Erroneous <SpadArray::Index> enable_ref_spads(bool apertureSpads, SpadArray goodSpadArray, SpadArray spadArray, SpadArray::Index start, SpadArray::Index offset, unsigned int spadCount);

    Error CheckAndLoadInterruptSettings(bool StartNotStopFlag);//move to core?
  };
}//end namespace
#endif /* __H_ */
