//
// Copyright 2021 by Andy Heilveil, github/980f, on 12/19/21.
//

#ifndef VL53_VL53L0X_FIXPOINT_H
#define VL53_VL53L0X_FIXPOINT_H


/** use where fractional values are expected
 * Formerly the gyration in here were explicitly repeated in very many places, each having to be checked for rogue tile error.
 * Now we know that adding half the denominator before dividing in order to round always uses the same value for that halving, in one spot it did not!
 * Given a floating point value f it's .16 bit point is (int)(f*(1<<16))
 *
 * todo: automatically generate the RawType based in whole+fract
 *
 * */
template<unsigned whole, unsigned fract, typename RawType= uint32_t> struct FixPoint {
  RawType raw;
  enum {
    mask = (1 << fract) - 1
    , size = (fract + whole) / 2
    , powerShifter = 1 << fract
    , half = 1 << (fract - 1)
  };

  operator RawType() const {
    return raw;
  }

  operator RawType &()  {
    return raw;
  }

  operator float() const {
    return float(raw >> fract) + float(raw & mask) / float(powerShifter);
  }

  //todo: this is logically dangerous but matches legacy use. Logically one would expect to assign to the whole part, ie shrink by fract.
  FixPoint &operator=(RawType pattern) {
    raw = pattern;
    return *this;
  }

  FixPoint() : raw(0) {
  }

  FixPoint &operator=(float eff) {
    if (eff < 0) { //need to see if this ever occurs or if all entities are strictily positive or checked by app.
      raw = 0;
    } else if (eff == 0.0) {//frequent enough to special case
      raw = 0;
    } else {
#if 0  //need to see which is better on processor without hardware floating point. modff should be faster
      raw= RawType(modff(eff,&eff))<<fract;
      raw|= RawType(eff*(1<<fract));
#else
      raw = RawType(eff * powerShifter);
#endif
    }
    return *this;
  }

  FixPoint(float eff) {
    *this = eff;
  }

  /** @returns nearest integer to nominal value */
  RawType rounded() const {
    return (raw + half) >> fract;
  }

  /** @returns the raw value of this scaled by 2^ @param  bits, roundeding */
  RawType scaled(unsigned bits) const {
    return roundedScale(raw, bits);
  }

  /** @returns this after dividing by 2^ @param bits, rounding */
  FixPoint &shrink(unsigned bits) {
    raw = this->scaled(bits);
    return *this;
  }

  /** @returns this after dividing by 2^ @param bits, rounding */
  RawType boosted(unsigned bits) const {
    return raw << bits;
  }

  /** @returns this after dividing by 2^ @param bits, rounding */
  FixPoint &boost(unsigned bits) {
    raw <<= bits;
    return *this;
  }

  RawType squared() const {//todo: add power of 2 divider
    return squared(raw);
  }

  FixPoint &square() {
    raw *= raw;
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

  RawType operator +(const FixPoint &rhs) {
    return raw+rhs.raw;
  }
  RawType operator -(const FixPoint &rhs) {
    return raw-rhs.raw;
  }
  RawType operator *(const FixPoint &rhs) {
    return raw*rhs.raw;
  }
  //force explicit roundedDivided

  FixPoint & operator +=(const FixPoint &rhs) {
    raw+=rhs.raw;
    return *this;
  }
  FixPoint & operator -=(const FixPoint &rhs) {
    raw-=rhs.raw;
    return *this;
  }
  FixPoint & operator *=(const FixPoint &rhs) {
     raw*=rhs.raw;
     return *this;
  }
//force explicit roundedDivided

  RawType operator +(RawType rhs) {
    return raw+rhs.raw;
  }
  RawType operator -(RawType rhs) {
    return raw-rhs.raw;
  }
  RawType operator *(RawType rhs) {
    return raw*rhs.raw;
  }
  //force explicit roundedDivided

  FixPoint & operator +=(RawType rhs) {
    raw+=rhs.raw;
    return *this;
  }
  FixPoint & operator -=(RawType rhs) {
    raw-=rhs.raw;
    return *this;
  }
  FixPoint & operator *=(RawType rhs) {
    raw*=rhs.raw;
    return *this;
  }
//force explicit roundedDivided

  /**
   * assinging with shifting and truncation or expansion as needed
   * */
  template<unsigned other_whole, unsigned other_fract> FixPoint &operator=(FixPoint<other_whole, other_fract> other) {
//e.g: <16,16>other  to <9,7> this (uint16_t)((Value >> 9) & 0xFFFF)
    const int bitdiff = fract - other_fract;
    if constexpr (size >= other.size) {//we are expanding so inflate and shrink
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
    return *this;
  }

  /** rounding /= */
  template<typename Intish> FixPoint &divideby(Intish denom) {
    raw = roundedDivide(raw, denom);
    return *this;
  }
};

#endif //VL53_VL53L0X_FIXPOINT_H