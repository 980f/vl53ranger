//
// Copyright 2021 by Andy Heilveil (github/980f) created on 12/20/21.
//

#ifndef VL53_BITMANIPULATORS_H
#define VL53_BITMANIPULATORS_H

#if __has_include(<cstdint>)

#include <cstdint>  //uint8_t in a few places

#else
#include "stdint.h"
#endif

#include "intsizer.h"

/** bit of a byte.  A template allowing this to work for larger int types is too tedious to use
 * strange name as too many bit() functions are flying around at the moment, should rename once we get rid of such
 * */
template<typename Intish=uint8_t> constexpr Intish Bitter(unsigned bitnum) {
  return Intish(1) << bitnum;
}

/** annoying to use template for bit of something other than a byte.
 * use: Bitof<uint16_t>(12)  => 1<<12.
 * */
template<typename Intish> constexpr Intish Bitof(const unsigned bitnum) {
  if (bitnum / 8 >= sizeof(Intish)) {
    return 0;
  }
  return 1 << bitnum;
}

template<typename Intish, unsigned bitnum> constexpr Intish Bit() {
  static_assert(bitnum / 8 < sizeof(Intish), "type does not have that many bits");
  return 1 << bitnum;
}

template<typename Intish> constexpr bool getBit(const unsigned bitnum, const Intish data) {
  if (bitnum / 8 > sizeof(Intish)) {
    return false;
  }
  return (data & Bitof<Intish>(bitnum)) != 0;
}

template<unsigned bitnum, typename Intish> constexpr bool getBit(const Intish data) {
  return (data & Bit<Intish, bitnum>()) != 0;
}

template<typename Intish> constexpr void setBit(const unsigned bitnum, Intish &data, bool setit) {
  if (bitnum / 8 <= sizeof(Intish)) {
    if (setit) {
      data |= Bitter<Intish>(bitnum);
    } else {
      data &= ~Bitter<Intish>(bitnum);
    }
  }
}

template<unsigned bitnum, typename Intish> constexpr void setBit(Intish &data, bool setit) {
  static_assert(bitnum / 8 < sizeof(Intish));
  if (bitnum / 8 <= sizeof(Intish)) {
    if (setit) {
      data |= Bitter<Intish>(bitnum);
    } else {
      data &= ~Bitter<Intish>(bitnum);
    }
  }
}

/**
 * e.g. for bits 6..3 0x78   Mask<6,3>::places
 * for masking after shifting field to line up lsb use Mask<4,3>::shifted  which will be 0x03
 * optional 3rd template argument must sometimes be provided by user to stifle compiler pickiness
 * */
template<unsigned msbit, unsigned lsbit = msbit, unsigned outof = msbit + 1> struct Mask {
  static_assert(msbit >= lsbit, "operand order is msbit,lsbit ");//require ordered to simplify the code
  //by using unsigned we don't need to check for negatives
  //we use unsigned as very few of the numbers are signed and the math really does not depend upon signedness
  using RawType = typename Unsigned<outof>::type;
  enum : RawType {
    width = msbit - lsbit + 1
    ,    //check: needs to be 1 when msbit==lsbit
    shifted = ~(~0U << width)
    ,  //check: should be 1 == 1<<0 when msbit==lsbit, width =1 1<<1 = 2, -1 = 1, check should be -1 when msbit=numbits in type-1, and lsbit=0
    places = shifted << lsbit // suitable for field insertion mask
  };
};

/** @returns a substring of bits from @param data, returning the same sized value even if it could be a smaller type
 * this version only works for unsigned data. If you want to extract a signed field use a struct with bit fields instead of this.
 * */
template<unsigned msbit, unsigned lsbit, typename Intish> constexpr Intish getBits(const Intish data) {
  static_assert(msbit / 8 < sizeof(Intish));//does the data type even have that many bits?

  return (data >> lsbit) & Mask<msbit, lsbit>::shifted;
}

template<typename Intish> constexpr uint8_t getByte(const unsigned bytenum, const Intish data) {
  if (bytenum > sizeof(Intish)) {
    return 0;//shifted into nothingness
  }
  return data >> (8 * bytenum);
}

template<unsigned bytenum, typename Intish> constexpr uint8_t getByte(const Intish data) {
  static_assert(bytenum < sizeof(Intish), "operand doesn't have that many bytes");
  return data >> (8 * bytenum);
}

/** read and writes like a bit */
class BitAlias {
  const unsigned offset;
  uint8_t &wrapped;
public:

  BitAlias &operator=(bool setme) {
    setBit(offset, wrapped, setme);
    return *this;
  }

  operator bool() const {
    return getBit(offset, wrapped);
  }

  BitAlias(unsigned char &datum, unsigned bitnum) : offset(bitnum), wrapped(datum) {
  }
};

#endif //VL53_BITMANIPULATORS_H
