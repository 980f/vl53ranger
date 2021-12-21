//
// Created by andyh on 12/19/21.
//

#ifndef VL53_VL53L0X_SPADARRAY_H
#define VL53_VL53L0X_SPADARRAY_H

#include "bitmanipulators.h"

/** spad arrays are arrays of bits of whether a spad exists or is enabled
 * If we were willing to commit to a more recent c++ standard we would use a bitarray inside this class. */
class SpadArray {

public:
  enum {
    NumberOfBytes = 6
    , MaxCount = 44
  };

  uint8_t raw[NumberOfBytes];

  struct Index {
    //using 8 bit ints to urge compiler to pass the whole object in a register.
    uint8_t coarse;
    uint8_t fine;

    Index(unsigned bitnumber) : coarse(bitnumber / 8), fine(bitnumber % 8) {
    }

    Index &operator++() {
      if (++fine >= 8) {
        fine = 0;
        ++coarse;
      }
      return *this;
    }

    bool isValid() const {
      return coarse < NumberOfBytes && (coarse < (NumberOfBytes - 1) || fine < MaxCount % 8);
    }
  };

public:
  /** sometimes it is an array of 6 integers */
  uint8_t &operator[](unsigned coarseindex) {
    return raw[coarseindex];//vulnerable to bad index just like raw array code was.
  }

  void clear() {
    for (unsigned i = 0; i < NumberOfBytes; ++i) {
      raw[i] = 0;
    }
  }

  bool get(Index bitly) {
    return getBit(bitly.fine, raw[bitly.coarse]);
  }

  void set(Index bitly, bool enableit) {
    if (enableit) {
      raw[bitly.coarse] |= 1 << bitly.fine;
    } else {
      raw[bitly.coarse] &= ~(1 << bitly.fine);
    }
  }

  void enable(unsigned spadIndex) {
    set(spadIndex, 1);
  }

  unsigned count_enabled(bool *pIsAperture);

  bool operator==(const SpadArray &rhs) const {
    for (unsigned i = 0; i < NumberOfBytes; ++i) {
      if (raw[i] != rhs.raw[i]) {
        return false;
      }
    }
    return true;
  }

  class Pointer{
    SpadArray &storage;
    SpadArray::Index index;
  public:
    Pointer(SpadArray&wrapped):storage(wrapped), index(0){}

    Pointer &operator=(unsigned absolute){
      index=absolute;
      return *this;
    }

    BitAlias operator *(){
      return BitAlias(storage[index.coarse],index.fine);
    }

    Pointer &operator ++(){
      ++index;
      return *this;
    }

//    /** post increment is a touch expensive, returns a copy of this which is about 8 bytes with a weird lifetime */
//    Pointer &operator ++(int ){
//      Pointer post=*this;
//      ++index;
//      return post;
//    }

  };

};

#endif //VL53_VL53L0X_SPADARRAY_H
