/*******************************************************************************
 *  Copyright 2015, STMicroelectronics International N.V.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 * Neither the name of STMicroelectronics nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
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
 ********************************************************************************/

#ifndef _VL53L0X_PLATFORM_H_
#define _VL53L0X_PLATFORM_H_

#include "vl53l0x_def.h"  //DevData
#include "vl53l0x_error.h"
#include "vl53l0x_i2c_platform.h"

namespace VL53L0X {

/**
 * @file vl53l0_platform.h
 *
 * @brief All end user OS/platform/application porting
 */

/**
 * @defgroup VL53L0X_platform_group VL53L0 Platform Functions
 * @brief    VL53L0 Platform Functions
 *  @{
 */

/**
 * @struct  VL53L0X_Dev_t
 * @brief    Generic PAL device type that does link between API and platform
 * abstraction layer
 *
 */
/** In ST's work this guy's fields were embedded in the Device causing unnecessary sharing of information. */
  struct Physical {

    /*!< user specific field */
    ArduinoWirer wirer;

    Physical(Arg &&args) :wirer(std::forward<Arg>(args)){
      //do nothing so that we can have statically constructed instances.
    }

    /** it appears that applications typically have done this init themselves instead of calling this one. */
    bool init() {
      wirer.i2c_init();
      return true;
    }

/**
 * Lock comms interface to serialize all commands to a shared I2C interface.
 * ST allowed useless context, only the I2C master involved should be relevant here.
 * There are no good reasons for multiple OS threads to be talking directly to a sensor,
 * the user app should use message queues for other apps to share use of the sensor.
 * Anything else is unlikely to be sane, the number of failure modes is uncountable.
 *
 * A thread interrupting itself to do other sensor operations is begging for a dead-lock.
 *
 * @return  whether lock was achieved
 */
    bool LockSequenceAccess() {
      return true;//NYI
    }

/**
 * Unlock comms interface to serialize all commands to a shared I2C interface
 * for a specific device
 * @return whether unlock worked.
 */
    bool UnlockSequenceAccess() {
      return true;//NYI
    }

/**
 * Writes the supplied byte buffer to the device
 * @param   index     The register index
 * @param   pdata     Pointer to uint8_t buffer containing the data to be
 * written
 * @param   count     Number of bytes in the supplied byte buffer
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  ERROR_CONTROL_INTERFACE i2c tx buffer overflow
 * @returns ERROR_BAD_PARAMS count>buffer size.
 */
    Error WriteMulti(uint8_t index, const uint8_t *pdata, int count);

/**
 * Reads the requested number of bytes from the device
 * @param   index     The register index
 * @param   pdata     Pointer to the uint8_t buffer to store read data
 * @param   count     Number of uint8_t's to read
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
    Error ReadMulti(uint8_t index, uint8_t *pdata, int count);

    /**  */
    template<typename Scalar> Error Write(uint8_t index, Scalar data, bool swapendians = false) {
      return WriteMulti(index, reinterpret_cast<uint8_t *>(&data), sizeof(Scalar) * (swapendians ? -1 : 1));
    }

    /** reads into variable passed by address */
    template<typename Scalar> Error Read(uint8_t index, Scalar &data, bool swapendians = false) {
      return ReadMulti(index, reinterpret_cast<uint8_t *>(data), sizeof(Scalar) * (swapendians ? -1 : 1));
    }

/** update paired value and 'Error' of reading that value.
 * This is a macro as we aren't ready to make Erroneous<> pervasive quite yet. */
#define Fetch( erroneousThing, regcode) erroneousThing.error = comm.Read( regcode, erroneousThing.wrapped);

    /**
 * Write single byte register
 * @param   index     The register index
 * @param   data      8 bit register data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
    Error WrByte(uint8_t index, uint8_t data);

/**
 * Write word register
 * @param   index     The register index
 * @param   data      16 bit register data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
    Error WrWord(uint8_t index, uint16_t data);

/**
 * Write double word (4 byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      32 bit register data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
    Error WrDWord(uint8_t index, uint32_t data);

/**
 * Read single byte register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 8 bit data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
    Error RdByte(uint8_t index, uint8_t *data);

/**
 * Read word (2byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 16 bit data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
    Error RdWord(uint8_t index, uint16_t *data);

/**
 * Read dword (4byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 32 bit data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
    Error RdDWord(uint8_t index, uint32_t *data);

/**
 * Threat safe Update (read/modify/write) single byte register
 *
 * Final_reg = (Initial_reg & and_data) |or_data
 *
 * @param   Dev        Device Handle
 * @param   index      The register index
 * @param   AndData    8 bit and data
 * @param   OrData     8 bit or data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
    Error UpdateByte(uint8_t index, uint8_t AndData, uint8_t OrData);

    Error UpdateBit(const uint8_t index, const uint8_t bitnumber, const bool value){
      return UpdateByte(index,~(value ? 0 : Bitter(bitnumber)), value ? Bitter(bitnumber) : 0);
    }

/** @} end of VL53L0X_registerAccess_group */
  private:
    /** @returns an api error code for a non-zero return from an i2c function. At present all such return 0. */
    static Error recode(int i2creturn) {
      return i2creturn ? ERROR_CONTROL_INTERFACE : ERROR_NONE;
    }

  };

  struct Dev_t { //gets extended  to Core and that to Api
    DevData_t Data; /*!< embed ST Ewok Dev  data as "Data"*/
    Physical comm; //not a base as eventually we will pass in a reference to a baser class

    Dev_t(Arg &&args) : comm(std::forward<Arg>(args)) {
      //do nothing so that we may static construct.
    }

    SemverLite ProductRevision;

    /**
 * @brief execute delay in all polling API call
 *
 * 980f: this sounds like it should be called 'yield()', allow other tasks to run.
 *
 * A typical multi-thread or RTOs implementation is to sleep the task for some
 * 5ms (with 100Hz max rate faster polling is not needed) if nothing specific is
 * need you can define it as an empty/void macro
 * @code
 * #define VL53L0X_PollingDelay(...) (void)0
 * @endcode
 * @param Dev       Device Handle
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
    Error PollingDelay(); /* usually best implemented as a real function */

    /** RAII device to ensure that certain device registers are restored before leaving a chunk of code. It appears that the interface has many more values than can be indexed by one byte of address and so there is a page register, but most accesses presume page 0.
     * Another instance appears to be an update disable to ensure that a block of many reads does not have content altered during reading. Having a snapshot rather than freeze-thaw would have been kind by the device engineers. */
    class FrameEnder {
      Physical &comm;
      const RegSystem reg;
      const uint8_t ender;
      Error error;

    public:
      operator Error () const {
        return error;
      }

      FrameEnder(Physical &comm, RegSystem reg, uint8_t starter, uint8_t ender) : comm(comm), reg(reg), ender(ender) {
        error = comm.WrByte(reg, starter);
      }

      ~FrameEnder() {
        //todo: debate whether this is conditional on constructor success. It appears to not be checked in much of the ST code.
        error = comm.WrByte(reg, ender);
        //but we have no means of reporting failure here, but if it fails here recovery is hopeless.
        //that is why the error should be a thread_local and not returned everywhere.
      }

      /** @returns whether starter was successfully applied */
      operator bool() const {
        return error == ERROR_NONE;
      }

    };

    FrameEnder autoCloser(RegSystem reg, uint8_t starter, uint8_t ender) {
      return {comm, reg, starter, ender};
    }
  };

/**
 * @brief   Declare the device Handle as a pointer of the structure @a
 * VL53L0X_Dev_t.
 *
 */
//  typedef VL53L0X_Dev_t *VL53L0X_DEV;

/**
 * @def PALDevDataGet
 * @brief Get ST private structure @a VL53L0X_DevData_t data access
 *
 * @param Dev       Device Handle
 * @param field     ST structure field name
 * It maybe used and as real data "ref" not just as "get" for sub-structure item
 * like PALDevDataGet(FilterData.field)[i] or
 * PALDevDataGet(FilterData.MeasurementIndex)++
 */
#define PALDevDataGet(field) Data.field

/**
 * @def PALDevDataSet( field, data)
 * @brief  Set ST private structure @a VL53L0X_DevData_t data field
 * @param Dev       Device Handle
 * @param field     ST structure field name
 * @param data      Data to be set
 */
#define PALDevDataSet(field, data) Data.field = data

/**
 * @defgroup VL53L0X_registerAccess_group PAL Register Access Functions
 * @brief    PAL Register Access Functions
 *  @{
 */


/** @} end of VL53L0X_platform_group */
/** should be in std lib somewhere ... */
template <typename Target,typename Source> Target saturated(Source bigger){
  if(bigger> std::numeric_limits<Target>::max()){
    return  std::numeric_limits<Target>::max();
  }
  if(std::numeric_limits<Target>::is_signed) {
    if (bigger < std::numeric_limits<Target>::min()) {
      return std::numeric_limits<Target>::min();
    }
  }
  return static_cast<Target>(bigger);
}

}//end namespace
#endif /* _VL53L0X_PLATFORM_H_ */
