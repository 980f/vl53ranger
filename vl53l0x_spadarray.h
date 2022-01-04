//
// Copyright 2021 by Andy Heilveil (github/980f)
// Created by andyh on 12/19/21.
//

#ifndef VL53_VL53L0X_SPADARRAY_H
#define VL53_VL53L0X_SPADARRAY_H

#include "bitmanipulators.h"

/** legacy combination. */
struct SpadCount {
  uint8_t quantity = 0;//ReferenceSpadCount;  /* used for ref spad management */
  bool isAperture = false;//ReferenceSpadType;   /* used for ref spad management */

  bool isValid() const {
    return quantity > 0; //not this guy's job to enforce minSpadCount
  }
};

/** spad arrays are arrays of bits of whether a spad exists or is enabled
 * If we were willing to commit to a more recent c++ standard we would use a bitarray inside this class. */
class SpadArray {

public:
  enum {
    MaxCount = 44    //this also appeared in one place as 0x2C
    , NumberOfBytes = (MaxCount + 7) / 8
  };

  uint8_t raw[NumberOfBytes];

public:
  struct Index {
    //using 8 bit ints to urge compiler to pass the whole object in a register.
    uint8_t coarse;
    uint8_t fine;

    /** */
    constexpr Index(unsigned bitnumber) : coarse(bitnumber / 8), fine(bitnumber % 8) {
    }

    /** default value creates a bad index, however if  you ++ it it becomes the 0th which is valid. */
    Index() : coarse(~0), fine(7) {
    }

    bool operator==(const Index &rhs) const {
      return coarse == rhs.coarse && fine == rhs.fine;
    }

    bool operator!=(const Index &rhs) const {
      return !(rhs == *this);
    }

    Index &operator++() {
      if (++fine >= 8) {
        fine = 0;
        ++coarse;
      }
      return *this;
    }

    Index &operator=(Index other) {
      coarse = other.coarse;
      fine = other.fine;
      return *this;
    }

    unsigned absolute() const {
      return (coarse << 3) + fine;
    }

    Index operator+(const Index &other) const {
      return absolute() + other.absolute();
    }

    bool isValid() const {
      return coarse < NumberOfBytes && (coarse < (NumberOfBytes - 1) || fine < MaxCount % 8);
    }

/** @returns whether  a given spad index is an aperture SPAD by checking the quadrant.        */
    bool is_aperture() const {
//there are 64 spads per quadrant. quadrant 2 seems to be aperture, the rest not.
      static const uint32_t refArrayQuadrants[4] = {10, 5, 0, 5};
      return coarse < 32 && refArrayQuadrants[(coarse >> 3)] != 0;
    }
  };

public:
  static const Index badSpad;//sentinel return value
  /** zero init is excessive since all instances load before use, but ensures that the excess bits are zero*/
  SpadArray() {
    clear();
  }

  /** sometimes it is an array of 6 integers */
  uint8_t &operator[](unsigned coarseindex) {
    return raw[coarseindex];//vulnerable to bad index just like raw array code was.
  }

  void clear() {
    for (unsigned char &i: raw) {
      i = 0;
    }
  }

  bool get(Index bitly) const {
    return getBit(bitly.fine, raw[bitly.coarse]);
  }

  void set(Index bitly, bool enableit) {
    BitAlias(raw[bitly.coarse], bitly.fine) = enableit;
  }

  void enable(Index spadIndex) {
    set(spadIndex, true);
  }

  SpadCount count_enabled();

  bool operator==(const SpadArray &rhs) const;

  bool operator!=(const SpadArray &rhs) const {
    return !((*this) == (rhs));
  }

  /** @returns the index of the next bit that is set, which may the value passed in. */
  Index nextSet(Index curr) const;

  /** wraps what was once called enable_ref_spad */
  class Scanner {
    const SpadArray &goodSpadArray;
    SpadArray::Index scanner;
  public:
    SpadArray::Index lastSet;//because this is the cheapest way to undo the last set
    SpadArray &spadArray;

    void restart() {
      scanner = 0;
    }

    Scanner(const SpadArray &goodSpadArray, SpadArray &spadArray);

    /** move pointer based on inherent type, ignoring the arrays*/
    bool moveto(bool isAperature, SpadArray::Index bias);

    /** set req.quantity spads of type req.isAperture in the wrapped spadArray */
    bool operator()(SpadCount req);

    /** clear/disable the last bit enabled, making no other changes */
    void undoLast() {
      spadArray.set(lastSet, false);
      //and what do we do to guard against ill use? nothing, leave this idempotent
    }
  };
};

#endif //VL53_VL53L0X_SPADARRAY_H
