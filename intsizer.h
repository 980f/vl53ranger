#pragma once


namespace xstd { //missing on one platform, and was getting tedious to figure out how to add this.

template<bool Condition, typename ForTrue, typename ForFalse> struct conditional {
  typedef ForTrue type;
};

template< typename ForTrue, typename ForFalse> struct conditional<false, ForTrue, ForFalse> {
  typedef ForFalse type;
};

}


/** provides type at least large enough for the given bits.
   This is somewhere in the c++ std library but I am in a hurry
 * */
template<unsigned size> struct Unsigned {
  using type =
    typename xstd::conditional < size <= 8, uint8_t,
    typename xstd::conditional < size <= 16, uint16_t,
    typename xstd::conditional<size <= 32, uint32_t,
    uint64_t
    >::type
    >::type
    >::type;
};

template<unsigned size> struct Signed {
  using type =
    typename xstd::conditional < size <= 8, int8_t,
    typename xstd::conditional < size <= 16, int16_t,
    typename xstd::conditional<size <= 32, int32_t,
    int64_t
    >::type
    >::type
    >::type;
};
