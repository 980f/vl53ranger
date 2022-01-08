/*******************************************************************************
 * Copyright 2021 Andrew Heilveil, github/980f massively rewritten starting with content
 *  Copyright 2014,2015, STMicroelectronics International N.V.
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

#include "vl53l0x_platform.h"
#include "vl53l0x_i2c_platform.h"

#include "vl53l0x_platform_log.h"

#include "trynester.h"

namespace VL53L0X {

/** Maximum buffer size to be used in i2c
 *
 * 32 bit ints: 4
 * SpadArray:   6
 * Measurement result wad 12
 * */
#define VL53L0X_MAX_I2C_XFER_SIZE    12

  void Physical::WriteMulti(uint8_t index, const uint8_t *pdata, int count) {
    if (count >= VL53L0X_MAX_I2C_XFER_SIZE) {
      THROW(ERROR_INVALID_PARAMS);//unlikely: unreasonably large item
    }
    wirer.write_multi( index, pdata, count);
  }

  void Physical::ReadMulti( uint8_t index, uint8_t *pdata, int count) {
    if (count >= VL53L0X_MAX_I2C_XFER_SIZE) {
      THROW( ERROR_INVALID_PARAMS);//unlikely: unreasonably large item
    }
    wirer.read_multi( index, pdata, count);
  } // VL53L0X_ReadMulti

  void Physical::WrByte( uint8_t index, uint8_t data) {
    wirer.Write( index, data);
  } // VL53L0X_WrByte

  void Physical::WrWord( uint8_t index, uint16_t data) {
    wirer.Write( index, data,true);
  } // comm.WrWord

  void Physical::WrDWord(uint8_t index, uint32_t data) {
    wirer.Write( index, data,true);
  } // comm.WrDWord

  void Physical::RdByte( uint8_t index, uint8_t *data) {
    wirer.Read(index, data);
  } // comm.RdByte

  void Physical::RdWord( uint8_t index, uint16_t *data) {
    wirer.Read( index, data,true);
  } // comm.RdWord

  void Physical::RdDWord(uint8_t index, uint32_t *data) {
    wirer.Read( index, data, true) ;
  } // comm.RdDWord

  void Physical::UpdateByte(uint8_t index, uint8_t AndData, uint8_t OrData) {
    //todo:T for multithreading must find wire support classes to mutex Read vs Write via repeated start and no stop.
    //the goal: // start address index repeat-start address read_byte hold repeat-start address index data stop.
    uint8_t data;
    wirer.Read( index, data);
    data = (data & AndData) | OrData;
    wirer.Write( index, data);
  }

//ick: below is a parameter that must be tuned per platform, but is buried deep in the source:
#ifndef VL53L0X_POLLINGDELAY_LOOPNB
#define VL53L0X_POLLINGDELAY_LOOPNB 250
#endif

  void Dev_t::PollingDelay() {
    for (volatile unsigned i = VL53L0X_POLLINGDELAY_LOOPNB; i-- > 0;) {
      // Do nothing, except keep compiler from dropping loop due to no side-effects!
      asm ("nop");
    }
  } // VL53L0X_PollingDelay

} //end namespace
