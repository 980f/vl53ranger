//
// Copyright 2021 by Andy Heilveil (github/980f)
// Created by andyh on 12/19/21.
//

#include "vl53l0x_spadarray.h"

SpadCount SpadArray::count_enabled( ) {
  SpadCount sc;//inits to 0 and false.
  /* This seems simple compared to legacy because the ST code screwed up the counting of the last byte, which was always pointless.
   * We init the array to all zeroes and looking at 4 unused bits cost a few extra iterations, which costs less than starting up a second loop.
   */
  for (unsigned byteIndex = 0; byteIndex < SpadArray::NumberOfBytes; ++byteIndex) {
    uint8_t tempByte = raw[byteIndex];
    for (unsigned bitIndex = 0; bitIndex <= 8; ++bitIndex) {
      if (getBit(bitIndex,tempByte)) {
        if (!sc.quantity++) {//if we found first one then its index tells us its type:
          sc.isAperture =  (byteIndex >= 2) || (bitIndex >= 4);
        }
      }
    }
  }
  return sc;
}

bool SpadArray::operator==(const SpadArray &rhs) const {
  for (unsigned i = 0; i < NumberOfBytes; ++i) {
    if (raw[i] != rhs.raw[i]) {
      return false;
    }
  }
  return true;
}
