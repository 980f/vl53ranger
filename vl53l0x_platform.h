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

#include "vl53l0x_def.h"
#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_platform_log.h"

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
    uint8_t I2cDevAddr; /*!< i2c device address user specific field */
//senseless parameter until we have a base class instead of explicit TwoWire    uint8_t comms_type;   /*!< Type of comms : VL53L0X_COMMS_I2C or VL53L0X_COMMS_SPI */
    uint16_t comms_speed_khz; /*!< Comms speed [kHz] : typically 400kHz for I2C */

    TwoWire &i2c;
    Physical(TwoWire &i2c,uint8_t I2cDevAddr,uint16_t comms_speed_khz=400):
    i2c(i2c),I2cDevAddr(I2cDevAddr),comms_speed_khz(comms_speed_khz){
      //do nothing so that we can have statically constructed instances.
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
    bool LockSequenceAccess(){}

/**
 * Unlock comms interface to serialize all commands to a shared I2C interface
 * for a specific device
 * @return whether unlock worked.
 */
    bool UnlockSequenceAccess(){}

/**
 * Writes the supplied byte buffer to the device
 * @param   index     The register index
 * @param   pdata     Pointer to uint8_t buffer containing the data to be
 * written
 * @param   count     Number of bytes in the supplied byte buffer
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
    Error WriteMulti( uint8_t index, uint8_t *pdata, uint32_t count);

/**
 * Reads the requested number of bytes from the device
 * @param   index     The register index
 * @param   pdata     Pointer to the uint8_t buffer to store read data
 * @param   count     Number of uint8_t's to read
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
    Error ReadMulti( uint8_t index, uint8_t *pdata, uint32_t count);


/**
 * Write single byte register
 * @param   index     The register index
 * @param   data      8 bit register data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
    Error WrByte( uint8_t index, uint8_t data);

/**
 * Write word register
 * @param   index     The register index
 * @param   data      16 bit register data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
    Error WrWord( uint8_t index, uint16_t data);

/**
 * Write double word (4 byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      32 bit register data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
    Error WrDWord( uint8_t index, uint32_t data);

/**
 * Read single byte register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 8 bit data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
    Error RdByte( uint8_t index, uint8_t *data);

/**
 * Read word (2byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 16 bit data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
    Error RdWord( uint8_t index, uint16_t *data);

/**
 * Read dword (4byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 32 bit data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
    Error RdDWord( uint8_t index, uint32_t *data);

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
    Error UpdateByte( uint8_t index, uint8_t AndData, uint8_t OrData);

/** @} end of VL53L0X_registerAccess_group */

  };

  struct Dev_t {
    DevData_t Data; /*!< embed ST Ewok Dev  data as "Data"*/
    Physical comm;

    struct ProductRevision {
      uint8_t Major;
      uint8_t Minor;
    };

    Erroneous<ProductRevision> GetProductRevision();
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
#define PALDevDataGet(Dev, field) (Dev->Data.field)

/**
 * @def PALDevDataSet(Dev, field, data)
 * @brief  Set ST private structure @a VL53L0X_DevData_t data field
 * @param Dev       Device Handle
 * @param field     ST structure field name
 * @param data      Data to be set
 */
#define PALDevDataSet(Dev, field, data) (Dev->Data.field) = (data)

/**
 * @defgroup VL53L0X_registerAccess_group PAL Register Access Functions
 * @brief    PAL Register Access Functions
 *  @{
 */

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
  Error PollingDelay(Physical &bus); /* usually best implemented as a real function */

/** @} end of VL53L0X_platform_group */

}//end namespace
#endif /* _VL53L0X_PLATFORM_H_ */
