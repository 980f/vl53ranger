/*******************************************************************************
Copyright 2015, STMicroelectronics International N.V.
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
********************************************************************************/
/**
 * @file  vl53l0_types.h
 * @brief VL53L0 types definition
 */

#ifndef VL53L0X_TYPES_H_
#define VL53L0X_TYPES_H_

/** @defgroup porting_type  Basic type definition
 *  @ingroup  VL53L0X_platform_group
 *
 *  @brief  file vl53l0_types.h files hold basic type definition that may
 * requires porting
 *
 *  contains type that must be defined for the platform\n
 *  when target platform and compiler provide stdint.h and stddef.h it is enough
 * to include it.\n If stdint.h is not available review and adapt all signed and
 * unsigned 8/16/32 bits basic types. \n If stddef.h is not available review and
 * adapt NULL definition .
 */
#include <cstddef>
#include <cstdint>
#include <cmath>

#ifndef NULL
#error "Error NULL definition should be done. Please add required include "
#endif

#if !defined(STDINT_H) && !defined(_STDINT_H) && !defined(_GCC_STDINT_H) && !defined(__STDINT_DECLS) && !defined(_GCC_WRAP_STDINT_H)

#pragma message( "Please review  type definition of STDINT define for your platform and add to list above ")

/*
 *  target platform do not provide stdint or use a different #define than above
 *  to avoid seeing the message below addapt the #define list above or implement
 *  all type and delete these pragma
 */

/** \ingroup VL53L0X_portingType_group
 * @{
 */

typedef unsigned long long uint64_t;

/** @brief Typedef defining 32 bit unsigned int type.\n
 * The developer should modify this to suit the platform being deployed.
 */
typedef unsigned int uint32_t;

/** @brief Typedef defining 32 bit int type.\n
 * The developer should modify this to suit the platform being deployed.
 */
typedef int int32_t;

/** @brief Typedef defining 16 bit unsigned short type.\n
 * The developer should modify this to suit the platform being deployed.
 */
typedef unsigned short uint16_t;

/** @brief Typedef defining 16 bit short type.\n
 * The developer should modify this to suit the platform being deployed.
 */
typedef short int16_t;

/** @brief Typedef defining 8 bit unsigned char type.\n
 * The developer should modify this to suit the platform being deployed.
 */
typedef unsigned char uint8_t;

/** @brief Typedef defining 8 bit char type.\n
 * The developer should modify this to suit the platform being deployed.
 */
typedef signed char int8_t;

/** @}  */
#endif /* _STDINT_H */

template<typename Intish> constexpr bool getBit(const unsigned bitnum, const Intish data) {
  if (bitnum / 8 > sizeof(Intish)) {
    //todo: compiletime error if bitnum/8 is greater than bytes in the data
    return false;
  }
  return (data & (1 << bitnum)) != 0;
}

template<unsigned bitnum, typename Intish> constexpr bool getBit(const Intish data) {
  static_assert(bitnum / 8 < sizeof(Intish));
  return (data & (1 << bitnum)) != 0;
}

template<unsigned msbit,unsigned lsbit, typename Intish> constexpr bool getBits(const Intish data) {
  static_assert(msbit / 8 < sizeof(Intish));
  static_assert(lsbit / 8 < sizeof(Intish));
  static_assert(msbit>=lsbit);
  //to generate a mask for bit MS start with 10000 with the 1 to the left of the msbit then subtract some power of two from that.
  return (data & ((1 << msbit+1)-(1<<lsbit))) >>lsbit;
}

template<typename Intish> constexpr uint8_t getByte(const unsigned bytenum, const Intish data) {
  if (bytenum > sizeof(Intish)) {
    return false;
  }
  return data >> (8 * bytenum);
}

template<unsigned bytenum, typename Intish> constexpr uint8_t getByte(const Intish data) {
  static_assert(bytenum < sizeof(Intish), "operand doesn't have that many bytes");
  return data >> (8 * bytenum);
}

/** @returns @param num divided by @param denom, rounded */
template<typename IntishOver, typename IntishUnder> IntishOver roundedDivide(IntishOver num, IntishUnder denom) {
  return (denom != 0) ? (num + (denom >> 1)) / denom : 0;//using 0 for divide by zero as a local preference. IE the first places that actually checked for /0 used 0 as the ratio.
}

/** @returns value divided by 2^ @param powerof2 rounded rather than truncated*/
template<typename IntishOver> IntishOver roundedScale(IntishOver num, unsigned powerof2) {
  return (num + (1 << (powerof2 - 1))) >> powerof2;//# fully parenthesized for clarity ;)
}

/** @returns @param num divided by 1000, rounded.
 * this is somewhat the reverse of FixPoint millis which multiplies the nominal value by 1000 and rounds to integer */
template<typename IntishOver> IntishOver kilo(IntishOver num) {
  return roundedDivide(num, 1000);
}

/** @returns @param value multiplied by @param shift power of 2*/
template<typename Intish> constexpr Intish boost(unsigned shift, Intish value) {
  return value << shift;
}

/** alters @param value to be no greater than @param max */
template<typename Intish> constexpr void lessen(Intish &value, Intish max) {
  if (value > max) {
    /* Clip to prevent overflow. Will ensure safe max result. */
    value = max;
  }
}

/** @returns value squared
 * might add saturation which is rarely checked at present */
template<typename Intish> Intish squared(Intish num) {
  return num * num;
}

#include "vl53l0x_fixpoint.h"

using FixPoint1616_t = FixPoint<16, 16>;

/** 0x10000 which is one greater than 0x0000FFFF */
const FixPoint1616_t Unity {1.0};



//types seen in macros that used to do the conversions:
using FixPoint97_t = FixPoint<9, 7>;
using FixPoint88_t = FixPoint<8, 8>;
using FixPoint412_t = FixPoint<4, 12>;
using FixPoint313_t = FixPoint<3, 13>;
using FixPoint102_t = FixPoint<3, 13>;

using FixPoint08_t = FixPoint<0, 8>;
using FixPoint53_t = FixPoint<5, 3>;

#endif /* VL53L0X_TYPES_H_ */
