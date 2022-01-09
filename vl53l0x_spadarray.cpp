//
// Copyright 2021 by Andy Heilveil (github/980f)
// Created by andyh on 12/19/21.
//

#include "vl53l0x_spadarray.h"
const SpadArray::Index SpadArray::badSpad;//refused to accept init in header despite const and constexpr constructors

SpadCount SpadArray::count_enabled( ) {
  SpadCount sc;//inits to 0 and false.
  /* This seems simple compared to legacy because the ST code screwed up the counting of the last byte, which was always pointless.
   * We init the array to all zeroes and looking at 4 unused bits cost a few extra iterations, which costs less than starting up a second loop.
   */
  for (unsigned byteIndex = 0; byteIndex < SpadArray::NumberOfBytes; ++byteIndex) {
    uint8_t onebyte = raw[byteIndex];
    for (unsigned bitIndex = 0; bitIndex <= 8; ++bitIndex) {
      if (getBit(bitIndex, onebyte)) {
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

SpadArray::Index SpadArray::nextSet(SpadArray::Index curr) const {
  while (curr.isValid()) {
    if (get(curr)) {
      return curr;
    }
    ++curr;
  }
  return badSpad;//canonical ! isValid()
}

bool SpadArray::Scanner::moveto(bool isAperature, SpadArray::Index bias) {
  while (scanner.isValid() && (bias + scanner).is_aperture()!=isAperature) {
    ++scanner;
  }
  return scanner.isValid();
}

bool SpadArray::Scanner::operator()(SpadCount req) {
  while (req.quantity>0) {
    if(!scanner.isValid()){
      return false;
    }
    auto nextGoodSpad = goodSpadArray.nextSet(scanner);
    if (!nextGoodSpad.isValid()) {
      return false;//seems excessive, we just went past the last
    }
    /* Confirm that the next good SPAD is of the desired aperture type */
    if (nextGoodSpad.is_aperture() != req.isAperture) {
      /* if we can't get the required number of good spads from the current quadrant then this is an error */
      return false;
    }
    lastSet=nextGoodSpad;
    spadArray.enable(lastSet);
    scanner = ++nextGoodSpad;//without the incr we would spin forever
  }
  return true;
}

SpadArray::Scanner::Scanner(const SpadArray &goodSpadArray, SpadArray &spadArray) : goodSpadArray(goodSpadArray), spadArray(spadArray) {
  //default scanner is NOT valid, call restart before every use.
}
