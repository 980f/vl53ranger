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

/**
 * @file VL53L0X_i2c.c
 *
 * Copyright (C) 2014 ST MicroElectronics
 *
 * provide variable word size byte/Word/dword VL6180x register access via i2c
 *
 */
#include "vl53l0x_platform.h"
#include "vl53l0x_i2c_platform.h"

#include "log_platform.h"
namespace VL53L0X {
/**
 * @def I2C_BUFFER_CONFIG
 *
 * @brief Configure Device register I2C access
 *
 * @li 0 : one GLOBAL buffer \n
 *   Use one global buffer of MAX_I2C_XFER_SIZE byte in data space \n
 *   This solution is not multi-Device compliant nor multi-thread cpu safe \n
 *   It can be the best option for small 8/16 bit MCU without stack and limited
 * ram  (STM8s, 80C51 ...)
 *
 * @li 1 : ON_STACK/local \n
 *   Use local variable (on stack) buffer \n
 *   This solution is multi-thread with use of i2c resource lock or mutex see
 * VL6180x_GetI2CAccess() \n
 *
 * @li 2 : User defined \n
 *    Per Device potentially dynamic allocated. Requires VL6180x_GetI2cBuffer()  //ick: cut and paste error, one that would never have happened with C++ namespaces
 * to be implemented.
 * @ingroup Configuration
 */
#define I2C_BUFFER_CONFIG 1
/** Maximum buffer size to be used in i2c */
#define VL53L0X_MAX_I2C_XFER_SIZE    64

//unused concept:
//#if I2C_BUFFER_CONFIG == 0
///* GLOBAL config buffer */
//uint8_t i2c_global_buffer[VL53L0X_MAX_I2C_XFER_SIZE];
//
//#define DECL_I2C_BUFFER
//#define VL53L0X_GetLocalBuffer( n_byte) i2c_global_buffer
//
//#elif I2C_BUFFER_CONFIG == 1
///* ON STACK */
//#define DECL_I2C_BUFFER uint8_t LocBuffer[VL53L0X_MAX_I2C_XFER_SIZE];
//#define VL53L0X_GetLocalBuffer( n_byte) LocBuffer
//#elif I2C_BUFFER_CONFIG == 2
///* user define buffer type declare DECL_I2C_BUFFER  as access  via
// * VL53L0X_GetLocalBuffer */
//#define DECL_I2C_BUFFER
//#else
//#error "invalid I2C_BUFFER_CONFIG "
//#endif

#define VL53L0X_I2C_USER_VAR /* none but could be for a flag var to get/pass to mutex interruptible  return flags and try again */
#define VL53L0X_GetI2CAccess(Dev) /* todo mutex acquire */
#define VL53L0X_DoneI2CAcces(Dev) /* todo mutex release */
//
//Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev) {
//  return Error_NONE;
//}
//
//Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev) {
//  return Error_NONE;
//}


  Error Physical::WriteMulti(uint8_t index, const uint8_t *pdata, int count) {
    if (count >= VL53L0X_MAX_I2C_XFER_SIZE) {
      return ERROR_INVALID_PARAMS;//BUG: formerly went ahead and asked for invalid transfer
    }
    return recode(wirer.write_multi( index, pdata, count));
  }

  Error Physical::ReadMulti( uint8_t index, uint8_t *pdata, int count) {
    VL53L0X_I2C_USER_VAR
    if (count >= VL53L0X_MAX_I2C_XFER_SIZE) {
      return ERROR_INVALID_PARAMS;//BUG: formerly went ahead and sent truncated data
    }
    return recode(wirer.read_multi( index, pdata, count));
  } // VL53L0X_ReadMulti

  Error Physical::WrByte( uint8_t index, uint8_t data) {
    //BUG?: not locked like the ReadMulti was, why not?
    return recode(wirer.Write( index, data));
  } // VL53L0X_WrByte

  Error Physical::WrWord( uint8_t index, uint16_t data) {
    return recode(wirer.Write( index, data));
  } // comm.WrWord

  Error Physical::WrDWord(uint8_t index, uint32_t data) {
    return recode(wirer.Write( index, data));
  } // comm.WrDWord

  Error Physical::RdByte( uint8_t index, uint8_t *data) {
    return recode(wirer.Read(index, data));
  } // comm.RdByte

  Error Physical::RdWord( uint8_t index, uint16_t *data) {
    return recode(wirer.Read( index, data));
  } // comm.RdWord

  Error Physical::RdDWord(uint8_t index, uint32_t *data) {
    return recode(wirer.Read( index, data) != 0);
  } // comm.RdDWord

  Error Physical::UpdateByte(uint8_t index, uint8_t AndData, uint8_t OrData) {
    uint8_t data;
    if (wirer.Read( index, data)) {
      return ERROR_CONTROL_INTERFACE;
    }
    data = (data & AndData) | OrData;
    return recode(wirer.Write( index, data));
  }

//ick: below is a parameter that must be tuned per platform, but is buried deep in the source:
#define VL53L0X_POLLINGDELAY_LOOPNB 250
  Error Dev_t::PollingDelay() {
    LOG_FUNCTION_START    ;
    for (volatile unsigned i = VL53L0X_POLLINGDELAY_LOOPNB; i-- > 0;) {
      // Do nothing, except keep compiler from dropping loop due to no side-effects!
      asm ("nop");
    }
    return ERROR_NONE;
  } // VL53L0X_PollingDelay

} //end namespace
