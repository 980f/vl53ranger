#pragma once

namespace VL53L0X{
/** @defgroup VL53L0X_define_Error_group Error and Warning code returned by API
 *	The following DEFINE are used to identify the PAL ERROR
 *	@{
 */

//this enum is the negative of the C version, we may restore the negation in C wrappers.
  enum Error {
    ERROR_NONE = 0
    , ERROR_CALIBRATION_WARNING /*!< Warning invalid calibration data may be in use
        \a	VL53L0X_InitData()
        \a VL53L0X_GetOffsetCalibrationData
        \a VL53L0X_SetOffsetCalibrationData */
    , ERROR_MIN_CLIPPED /*!< Warning parameter passed was clipped to min before to be applied */
    , ERROR_UNDEFINED /*!< Unqualified error */
    , ERROR_INVALID_PARAMS  /*!< Parameter passed is invalid or out of range todo: split into illegal and out of range */
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

/** @} VL53L0X_define_Error_group */
}
