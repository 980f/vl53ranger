//
// Created by andyh on 12/19/21.
//

#include "vl53l0x_spadarray.h"

unsigned SpadArray::count_enabled( bool *pIsAperture) {

  bool spadTypeIdentified = (pIsAperture==nullptr);//we only write if and when we find the first enable spad, if no pointer then we don't even look.

  /* The entire array will not be used for spads, therefore the last
   * byte and last bit is determined from the max spads value.
   * Which we can simply accommodate by having the unused bits always zero which we can presume in most places.
   * The ST code screwed up the counting of the last byte, which was always pointless.
   */
  unsigned enabled = 0;

  for (unsigned byteIndex = 0; byteIndex < SpadArray::NumberOfBytes; ++byteIndex) {
    uint8_t tempByte = raw[byteIndex];
    for (unsigned bitIndex = 0; bitIndex <= 8; ++bitIndex) {
      if (getBit(bitIndex,tempByte)) {
        ++enabled;
        if (!spadTypeIdentified ) {
          *pIsAperture =  (byteIndex >= 2) || (bitIndex >= 4);
          spadTypeIdentified = 1;
        }
      }
    }
  }
  return enabled;
}
