/*******************************************************************************
Copyright 2021, Andy Heilveil, github/980f via major editing of source
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
 * Device specific defines. To be adapted by chip firmware implementer for the targeted device.
 */

#ifndef _VL53L0X_DEVICE_H_
#define _VL53L0X_DEVICE_H_

#include "vl53l0x_types.h"
#include "bitmanipulators.h" //some enums are bit fields.

#define VL53L0X_I2C_ADDR 0x52 //< Default sensor I2C address in 8 Bit format

namespace VL53L0X {
/** @defgroup VL53L0X_DevSpecDefines_group VL53L0X cut1.1 Device Specific
 * Defines
 *  @brief VL53L0X cut1.1 Device Specific Defines
 *  @{
 */

/** @defgroup VL53L0X_DeviceError_group Device Error
 *  @brief Device Error code
 *
 *  This enum is Device specific it should be updated in the implementation
 *  Use @a VL53L0X_GetStatusErrorString() to get the string.
 *  It is related to Error Register of the Device.
 *  @{
 */
  enum DeviceError : uint8_t {
    DEVICEERROR_NONE = 0
    , DEVICEERROR_VCSELCONTINUITYTESTFAILURE
    , DEVICEERROR_VCSELWATCHDOGTESTFAILURE
    , DEVICEERROR_NOVHVVALUEFOUND
    , DEVICEERROR_MSRCNOTARGET
    , DEVICEERROR_SNRCHECK
    , DEVICEERROR_RANGEPHASECHECK
    , DEVICEERROR_SIGMATHRESHOLDCHECK
    , DEVICEERROR_TCC
    , DEVICEERROR_PHASECONSISTENCY
    , DEVICEERROR_MINCLIP
    , DEVICEERROR_RANGECOMPLETE
    , DEVICEERROR_ALGOUNDERFLOW
    , DEVICEERROR_ALGOOVERFLOW
    , DEVICEERROR_RANGEIGNORETHRESHOLD
/** @} end of VL53L0X_DeviceError_group */
  };

  /** this enum extracted from comments in source code, then applied to where they came from */
  enum RangeStatus : uint8_t {
    Range_Valid = 0
    , Sigma_Fail = 1
    , Signal_Fail = 2
    , Min_range = 3
    , Phase_fail = 4
    , HW_fail = 5
    , Not_Set = 255
  };

/** @defgroup VL53L0X_CheckEnable_group Check Enable list
 *  @brief Check Enable code
 *
 *  Define used to specify the LimitCheckId.
 *  Use @a VL53L0X_GetLimitCheckInfo() to get the string.
 *  @{
 */
  enum CheckEnable {
    CHECKENABLE_SIGMA_FINAL_RANGE = 0
    , CHECKENABLE_SIGNAL_RATE_FINAL_RANGE
    , CHECKENABLE_SIGNAL_REF_CLIP
    , CHECKENABLE_RANGE_IGNORE_THRESHOLD
    , CHECKENABLE_SIGNAL_RATE_MSRC
    , CHECKENABLE_SIGNAL_RATE_PRE_RANGE
    , //future checks go here
    CHECKENABLE_NUMBER_OF_CHECKS
  };
/** @}  end of VL53L0X_CheckEnable_group */

/** @defgroup VL53L0X_GpioFunctionality_group Gpio Functionality
 *  @brief Defines the different functionalities for the device GPIO(s)
 *  @{
 */
  enum GpioFunctionality : uint8_t {
    GPIOFUNCTIONALITY_OFF = 0 /*!< NO Interrupt  */
    , GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW /*!< Level Low (value < thresh_low)  */
    , GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH /*!< Level High (value > thresh_high) */
    , GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT /*!< Out Of Window (value < thresh_low OR value > thresh_high)  */
    , GPIOFUNCTIONALITY_NEW_MEASURE_READY /*!< New Sample Ready  */

  };

constexpr  bool valid(GpioFunctionality functionality) {
    return functionality <= GPIOFUNCTIONALITY_NEW_MEASURE_READY;
  }

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

#if 0  //the following were redundant definitions for the GpioFunctionality
#define REG_SYSTEM_INTERRUPT_GPIO_DISABLED 0x00
#define REG_SYSTEM_INTERRUPT_GPIO_LEVEL_LOW 0x01
#define REG_SYSTEM_INTERRUPT_GPIO_LEVEL_HIGH 0x02
#define REG_SYSTEM_INTERRUPT_GPIO_OUT_OF_WINDOW 0x03
#define REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY 0x04
#endif
/** @} end of VL53L0X_GpioFunctionality_group */

/* Device register map */

  enum SysRange {
    /** bit 0 in #REG_SYSRANGE_START write 1 toggles state in continuous mode and arms next shot in single shot mode */
    REG_SYSRANGE_MODE_START_STOP = (1 << 0)
    ,/** bit 1 write 0 in #REG_SYSRANGE_START set single shot mode */
    REG_SYSRANGE_MODE_SINGLESHOT = (0 << 1)
    , /** bit 1 write 1 in #REG_SYSRANGE_START set back-to-back operation mode */
    REG_SYSRANGE_MODE_BACKTOBACK = (1 << 1)
    , /** bit 2 write 1 in #REG_SYSRANGE_START set timed operation mode */
    REG_SYSRANGE_MODE_TIMED = (1 << 2)
    , /** bit 3 write 1 in #REG_SYSRANGE_START set histogram operation mode */
    REG_SYSRANGE_MODE_HISTOGRAM = (1 << 3)
    , /** mask existing bit in #REG_SYSRANGE_START*/
    REG_SYSRANGE_MODE_MASK = Mask<3, 0>::places

    //todo: document the other 4 bits in the register. Since the mask is unused one presumes the bits are as well.
    //bit 6 seems to be 'vhv cal'
  };

/** @defgroup VL53L0X_DefineRegisters_group Define Registers
 *  @brief List of all the defined registers
 *  @{
 *
 *  there are more than 256 registers, there is a page select register for selecting which bank.
 *  symbols like the following indicate you must remember to select page 1.
 *  = 0x4E  / * 0x14E * /
 */
  enum RegSystem : uint8_t {//the original defines had 16 bit constants, but hardware only has 8.
    REG_SYSRANGE_START = 0
      ,REG_SYSRANGE_stopper = 0x91      //980f guess, and needs MagicTrio wrapper
    , REG_SYSTEM_SEQUENCE_CONFIG = 0x01  //msb is wrap around enable
    , REG_SYSTEM_THRESH_HIGH = 0x0C
    , REG_SYSTEM_THRESH_LOW = 0x0E
    , REG_SYSTEM_RANGE_CONFIG = 0x09
    , REG_SYSTEM_INTERMEASUREMENT_PERIOD = 0x04
    , REG_SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A
    , REG_GPIO_HV_MUX_ACTIVE_HIGH = 0x84
    , REG_SYSTEM_INTERRUPT_CLEAR = 0x0B
    /* Result registers */
    , REG_RESULT_INTERRUPT_STATUS = 0x13
    , REG_RESULT_RANGE_STATUS = 0x14
    , REG_RESULT_CORE_PAGE = 1
    , REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC
    , REG_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0
    , REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0
    , REG_RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4
    , REG_RESULT_PEAK_SIGNAL_RATE_REF = 0xB6
    /* Algo register */
    , REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28
    , REG_I2C_SLAVE_DEVICE_ADDRESS = 0x8a
    /* Check Limit registers */
    , REG_MSRC_CONFIG_CONTROL = 0x60
    , REG_PRE_RANGE_CONFIG_MIN_SNR = 0x27
    //ick: an exception to the bigendianess seen elsewhere?:
    , REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56
    , REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57
    , REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64
    , REG_FINAL_RANGE_CONFIG_MIN_SNR = 0x67
    //ick: another little endian!
    , REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47
    , REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48
    , REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44
    , REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61
    , REG_PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62
    , REG_PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50
    , REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51
    , REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52
    , REG_SYSTEM_HISTOGRAM_BIN = 0x81
    , REG_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33
    , REG_HISTOGRAM_CONFIG_READOUT_CTRL = 0x55
    , REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70
    , REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71
    , REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72
    , REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20 // fp3.13
    , REG_MSRC_CONFIG_TIMEOUT_MACROP = 0x46
    , REG_SOFT_RESET_GO2_SOFT_RESET_N = 0xbf
    , REG_IDENTIFICATION_MODEL_ID = 0xc0
    , REG_IDENTIFICATION_REVISION_ID = 0xc2
    , REG_OSC_CALIBRATE_VAL = 0xf8
    , REG_GLOBAL_CONFIG_VCSEL_WIDTH = 0x32 //register or value?
    , REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_BASE = 0xB0 //note: there were 6 defines with a numerical suffix, this is the base for a function that generates them
    , REG_GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6  //first spad to pay any attention to, hardcoded to 180 as of this code rewrite
    , REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E  /* 0x14E */
    , REG_DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F /* 0x14F */
    , REG_POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80
    , REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89
    , REG_ALGO_PHASECAL_LIM = 0x30 /* 0x130 */
    , REG_ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30

    //formerly hidden inside code, naming for documentation purposes
    , Private_Strober = 0x83  /** strobe related */
    , Private_04 = 0x04 /** ?completion status */
    , Private_PowerMode = 0x80
    , Private_Pager = 0xFF  /** guessing a page select, or freeze for atomic update */
  };

  /** the following defines were dropped in favor of a function:
//#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 0x0B0
//#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_1 0x0B1
//#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_2 0x0B2
//#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_3 0x0B3
//#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_4 0x0B4
//#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_5 0x0B5 */
  constexpr uint8_t REG_GLOBAL_CONFIG_SPAD_ENABLES_REF(unsigned which) {
    return (REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_BASE + which);
  }

/** @} VL53L0X_DefineRegisters_group */

/** @} VL53L0X_DevSpecDefines_group */


  /* Speed of light in um per 1E-10 Seconds */
  const unsigned SPEED_OF_LIGHT_IN_AIR = 2997;

  const unsigned SIGMA_ESTIMATE_MAX_VALUE = 65535; /* equivalent to a range sigma of 655.35mm */
};//end namespace

#endif /* _VL53L0X_DEVICE_H_ */
