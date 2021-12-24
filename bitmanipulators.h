//
// Created by andyh on 12/20/21.
//

#ifndef VL53_BITMANIPULATORS_H
#define VL53_BITMANIPULATORS_H

template<typename Intish> constexpr bool getBit(const unsigned bitnum, const Intish data) {
  if (bitnum / 8 > sizeof(Intish)) {
    return false;
  }
  return (data & (1 << bitnum)) != 0;
}

template<unsigned bitnum, typename Intish> constexpr bool getBit(const Intish data) {
  static_assert(bitnum / 8 < sizeof(Intish));
  return (data & (1 << bitnum)) != 0;
}

template<typename Intish> constexpr void setBit(const unsigned bitnum, Intish &data, bool setit) {
  if (bitnum / 8 <= sizeof(Intish)) {
    if (setit) {
      data |= 1 << bitnum;
    } else {
      data &= ~(1 << bitnum);
    }
  }
}

template<unsigned bitnum, typename Intish> constexpr void setBit(Intish &data, bool setit) {
  static_assert(bitnum / 8 < sizeof(Intish));
  if (bitnum / 8 <= sizeof(Intish)) {
    if (setit) {
      data |= 1 << bitnum;
    } else {
      data &= ~(1 << bitnum);
    }
  }
}

/**
 * e.g. for bits 6..3 0xE8   Mask<6,3>::places
 *
 * */
template<unsigned msbit, unsigned lsbit> struct Mask {
  static_assert(msbit >= lsbit, "operand order is msbit,lsbit ");//require ordered to simplify the code
  //by checking msbit> lsbit we don't need to test lsbit.
  //by using unsigned we don't need to check for negatives
  enum {
    width = msbit - lsbit + 1    //check: needs to be 1 when msbit==lsbit
    , places = (1 << (msbit + 1)) - (1 << lsbit)
    , shifted = (1 << width) - 1  //should be 1 == 1<<0 when msbit==lsbit
  };
};

/** @returns a substring of bits from @param data, returning the same sized value even if it could be a smaller type
 * this version only works for unsigned data. If you want to extract a signed field use a struct with bit fields instead of this.
 * */
template<unsigned msbit, unsigned lsbit, typename Intish> constexpr Intish getBits(const Intish data) {
  static_assert(msbit / 8 < sizeof(Intish));//does the data type even have that many bits?

  return (data >> lsbit) & Mask<msbit,lsbit>::shifted;
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
