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

/** use where fractional values are expected
 *
 * Given a floating point value f it's .16 bit point is (int)(f*(1<<16))*/
template <unsigned whole, unsigned fract,typename RawType= uint32_t> struct FixPoint {
  //todo:2 compute Rawtype from whole and fract.
  RawType raw;
  enum {
    mask= (1<<fract)-1,
    size=(fract+whole)/2,
    powerShifter=1<<fract
  };
  operator RawType  () const {
    return raw;
  }
  operator float () const {
    return float(raw>>fract)+ float(raw&mask)/float(powerShifter);
  }
  FixPoint & operator = (float eff){
    if(eff<0){
      //need to see if this ever occurs
      raw=0;
    } else if(eff==0.0){//frequent enough to special case
      raw=0;
    } else {
#if 0  //need to see which is better on processor without hardware floating point. modff should be faster
      raw= RawType(modff(eff,&eff))<<fract;
      raw|= RawType(eff*(1<<fract));
#else
      raw = RawType(eff*powerShifter);
#endif
    }
    return *this;
  }
public:

  template<unsigned other_whole, unsigned other_fract>
  FixPoint &operator=(FixPoint<other_whole, other_fract> other) {
//e.g: <16,16>other  to <9,7> this (uint16_t)((Value >> 9) & 0xFFFF)
    const int bitdiff = fract - other_fract;
    if (size >= other.size) {//we are expanding so inflate and shift
      raw = other.raw;//radix point is still at other_fract position
      if (bitdiff < 0) {//truncating some bits on the ls end
        raw >>= (-bitdiff);
      } else { //##do not convert this to a ternary until use is debugged
        raw <<= (bitdiff);
      }
    } else { //shrinking so must align before truncating
      if (bitdiff < 0) {//truncating some bits on the ls end
        other.raw >>= (-bitdiff);
      } else { //##do not convert this to a ternary until use is debugged
        other.raw <<= (bitdiff);
      }
      raw = other.raw;//radix point is still at other_fract position
    }
  }
  };
using FixPoint1616_t = FixPoint<16,16,uint32_t>;
//types seen in macros that used to do the conversions.
using FixPoint97_t = FixPoint<9,7,uint16_t>;
using FixPoint88_t = FixPoint<8,8,uint16_t>;
using FixPoint412_t = FixPoint<4,12,uint16_t>;
using FixPoint313_t = FixPoint<3,13,uint16_t>;
using FixPoint102_t = FixPoint<3,13,uint16_t>;

using FixPoint08_t = FixPoint<0,8,uint8_t>;
using FixPoint53_t = FixPoint<5,3,uint8_t>;

#endif /* VL53L0X_TYPES_H_ */
