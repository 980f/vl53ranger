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

//  /** C++ is too strict for the legacy weirdness Error |= Error to compile, so:*/
//  struct ErrorAccumulator {
//    Error sum;
//
//    ErrorAccumulator(Error first = ERROR_NONE) : sum(first) {
//    }
//
//    /** ST's code did this with the raw values, which would have produced garbage if any of the calls actually produced an error, except when all the calls can only produce the I2C failure code */
//    ErrorAccumulator &operator|=(Error other) {
//      if (other != ERROR_NONE) {
//        sum = other;
//      }
//      return *this;
//    }
//
//    ErrorAccumulator &operator=(Error other) {
//      sum = other;
//      return *this;
//    }
//
//    operator Error() const {
//      return sum;
//    }
//
//    /** @returns whether there is an error, made to match Erroneous usage */
//    bool operator~() const {
//      return sum != ERROR_NONE;
//    }
//  };

  /** base class for returning values from a device, which operation might fail.*/
  template<typename Wrapped> class Erroneous {
  public:
    Wrapped wrapped;
    bool error;

    Erroneous(Wrapped wrapped, bool error = false) : wrapped(wrapped), error(error) {
    }

    Erroneous(bool error = false) : wrapped(), error(error) {
    }

    /** @returns whether value is GOOD. If not you can inspect Error for what went wrong */
    bool isOk() const {
      return error == false;
    }

    /** @returns "value not present" which is determined by the error not being 'none'
     * syntactic sugar: rarely used monadic function for interesting boolean fact.*/
    bool operator~() const {
      return error;
    }

    void operator|=(Error other) {
      error |= other!=ERROR_NONE;//todo: this is sloppy
    }

    /** this should allow transparent access as the type */
    operator Wrapped &() {
      return wrapped;
    }

    operator Wrapped *() {
      return &wrapped;
    }
  };

/** ERROR_OUT is useful for when there will be no common exit code on an error that terminates a sequence of steps other than returning the first such error.
 * To use it name the VL53L0X_Error member Error (instead of Error), which also allows for if(!Error) which is more readable than if(Error != VL53L0X_ERROR_NONE)
 *
 * Error is now an object with an operator~ which is true when there is an error.
 * */
#define ERROR_OUT if(~Error) return {Error}
#define EXIT_ON(erroneous)     if(~erroneous){  return erroneous; }
#define ERROR_ON(erroneous)     if(~erroneous){  return erroneous.error; }

//tempt while editing
#define THROW(err)
/** @} VL53L0X_define_Error_group */
}
