//
// Copyright 2021 by Andy Heilveil, github/980f, on 12/19/21.
//

#pragma once

#include "bitmanipulators.h"

/** provides type at least large enough for the given bits.
 * This is somewhere in the c++ std library but I am in a hurry
 * */
template <unsigned size> struct Unsigned {
  using type=
  typename std::conditional<size <= 8, uint8_t,
    typename std::conditional<size <= 16, uint16_t,
      typename std::conditional<size <= 32, uint32_t,
        uint64_t
      >::type
    >::type
  >::type;
};

/** use where fractional values are expected
 * Formerly the gyrations in here were explicitly repeated in very many places, each having to be checked for rogue tile error.
 * Now we know that adding half the denominator before dividing in order to round always uses the same value for that halving, in one spot it did not!
 * Given a floating point value f it's .16 bit point is (int)(f*(1<<16))
 * */
template<unsigned whole, unsigned fract> struct FixPoint {

  enum {
    mask = Mask<fract, 0>::shifted   //to extract fraction bits from lsbs
    , unity = Bitter(fract) //equals 1.0F
    , half = Bitter(fract - 1)  // 0.5
    , size = (fract + whole)  //total number of bits needed to represent the value, used to pick the underlying int type.
  };

  using RawType = typename Unsigned<size>::type; //we use unsigned as very few of the numbers are signed and the math really does not depend upon signedness

  //for rounding to integer:
  using WholeType =  typename Unsigned<whole>::type;

  ////////////////
  RawType raw;

  operator RawType() const {
    return raw;
  }

  operator RawType &() {
    return raw;
  }

  /** convert to a float */
  explicit operator float() const {
    return float(raw >> fract) + float(raw & mask) / float(unity);
  }

  //todo: this is logically dangerous but matches legacy use. Logically one would expect to assign to the whole part, ie shift up by fract.
  FixPoint &operator=(RawType pattern) {
    raw = pattern;
    return *this;
  }

  FixPoint() : raw(0) {
  }

  FixPoint &operator=(float eff) {
    raw = FixPoint(eff).raw;//borrow constructor
    return *this;
  }

  template<typename Intish> explicit FixPoint(Intish stuff) {
    raw = stuff;
  }

  /** two ints init item to ratio, with boosted of numerator by @param boostit power of two which defaults to fract, which treats num  as an integer value */
  template<typename IntishUp, typename IntishDown>
  constexpr FixPoint(IntishUp num, IntishDown denom, unsigned boostit = fract) {
    raw = num;//expand to 32 bits asap.
    boosted(boostit);
    if (denom != 1) {//do not round if denom is 1, which would only make a difference if boostit != fract
      divideby(denom);
    }
  }

  constexpr FixPoint(float eff) {
    if (eff < 0) { //need to see if this ever occurs or if all entities are strictily positive or checked by app.
      raw = 0;
    } else if (eff == 0.0) {//frequent enough to special case
      raw = 0;
    } else {
      raw= RawType(eff*unity);//todo: pick bits rather than actually multiply
    }
  }

  /** @returns nearest integer to nominal value */
  WholeType rounded() const {
    return (raw + half) >> fract;
  }

  /** @returns the raw value of this shrink by 2^ @param  bits, roundeding */
  RawType shrink(unsigned bits) const {
    return roundedScale(raw, bits);
  }

  /** @returns this after dividing by 2^ @param bits, rounding */
  FixPoint &shrunk(unsigned bits=fract) {
    raw = this->shrink(bits);
    return *this;
  }

  /** @returns value of this after multiplying by 2^ @param bits */
  RawType shiftup(unsigned bits) const {
    return raw << bits;
  }

  /** @returns this after dividing by 2^ @param bits, rounding */
  FixPoint &boosted(unsigned bits) {
    raw <<= bits;
    return *this;
  }

  RawType squared(unsigned reducebits=0) const {
    return roundedScale(::squared(raw),reducebits);
  }

  FixPoint &square(unsigned reducebits=0) {
    raw = squared(reducebits);
    return *this;
  }

  /** @returns this after ensuring that it is no greater than @param max */
  FixPoint &lessen(FixPoint max) {
    if (raw > max.raw) {
      /* Clip to prevent overflow. Will ensure safe max result. */
      raw = max.raw;
    }
    return *this;
  }

  /** @returns integer part after times 1000, rounded to integer. Might overflow. */
  RawType millis() const {
    return ((raw * 1000) + half) >> fract;
  }

  bool operator<(const FixPoint &rhs) const {
    return raw < rhs.raw;
  }

  bool operator>(const FixPoint &rhs) const {
    return rhs < *this;
  }

  bool operator<=(const FixPoint &rhs) const {
    return !(rhs < *this);
  }

  bool operator>=(const FixPoint &rhs) const {
    return !(*this < rhs);
  }

  bool operator==(const FixPoint &rhs) const {
    return raw == rhs.raw;
  }

  bool operator!=(const FixPoint &rhs) const {
    return !(rhs == *this);
  }

  RawType operator+(const FixPoint &rhs) {
    return raw + rhs.raw;
  }

  RawType operator-(const FixPoint &rhs) {
    return raw - rhs.raw;
  }

  RawType operator*(const FixPoint &rhs) {
    return raw * rhs.raw;
  }
  //force explicit roundedDivided

  FixPoint &operator+=(const FixPoint &rhs) {
    raw += rhs.raw;
    return *this;
  }

  FixPoint &operator-=(const FixPoint &rhs) {
    raw -= rhs.raw;
    return *this;
  }

  FixPoint &operator*=(const FixPoint &rhs) {
    raw *= rhs.raw;
    return *this;
  }
//force explicit roundedDivided

  RawType operator+(RawType raw_) {
    return raw + raw_;
  }

  RawType operator-(RawType raw_) {
    return raw - raw_;
  }

  RawType operator*(RawType raw_) {
    return raw * raw_;
  }
  //force explicit roundedDivided

  FixPoint &operator+=(RawType raw_) {
    raw += raw_;
    return *this;
  }

  FixPoint &operator-=(RawType raw_) {
    raw -= raw_;
    return *this;
  }

  FixPoint &operator*=(RawType raw_) {
    raw *= raw_;
    return *this;
  }
//force explicit roundedDivided

  /**
   * assigning with shifting and truncation or expansion as needed
   * */
  template<unsigned other_whole, unsigned other_fract> FixPoint(FixPoint<other_whole, other_fract> other) {
//e.g: <16,16>other  to <9,7> this (uint16_t)((Value >> 9) & 0xFFFF)
    const int bitdiff = fract - other_fract;
    if constexpr (unsigned(size) >= unsigned(other.size)) {//we are expanding so inflate and shrunk
      raw = other.raw;//radix point is still at other_fract position
      if constexpr (bitdiff < 0) {//truncating some bits on the ls end
        raw >>= (-bitdiff);
      } else if constexpr (bitdiff > 0) {
        raw <<= (bitdiff);
      }
    } else { //shrinking so must align before truncating
      if constexpr (bitdiff < 0) {//truncating some bits on the ls end
        other.raw >>= (-bitdiff);
      } else if constexpr(bitdiff > 0) {
        other.raw <<= (bitdiff);
      }
      raw = other.raw;
    }
  }

  template<unsigned other_whole, unsigned other_fract> FixPoint &operator=(FixPoint<other_whole, other_fract> other) {
    raw = static_cast<FixPoint>(other);//use constructor to do conversion as the vast majority were inline. Defining the targets as the appropriate type will let us drop macros that use the constructor.
    return *this;
  }

  /** rounding /= */
  template<typename Intish> FixPoint &divideby(Intish denom) {
    raw = roundedDivide(raw, denom);
    return *this;
  }
};

/** @returns value divided by 2^ @param powerof2 rounded rather than truncated*/
template<unsigned whole, unsigned fract> auto roundedScale(FixPoint<whole, fract> num, unsigned powerof2) -> typename decltype(num)::RawType {
  return (num.raw + (1 << (powerof2 - 1))) >> powerof2;//# fully parenthesized for clarity ;)
}

/** @returns @param num divided by @param denom, rounded */
template<unsigned whole, unsigned fract, typename IntishUnder>
auto roundedDivide(FixPoint<whole, fract> num, IntishUnder denom) -> typename decltype(num)::RawType {
  return (denom != 0) ? (num.raw + (denom >> 1)) / denom : 0;//using 0 for divide by zero as a local preference. IE the first places that actually checked for /0 used 0 as the ratio.
}
