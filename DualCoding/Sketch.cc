//-*-c++-*-

#include "Sketch.h"

namespace DualCoding {

// These functions must go here instead of in Sketch.h because
// they are not templated.

#define DEF_MATHOPS_CC(_T1, _T2, _Result) \
  DEF_MATHOP_CC( +, _T1, _T2, _Result ) \
  DEF_MATHOP_CC( -, _T1, _T2, _Result ) \
  DEF_MATHOP_CC( *, _T1, _T2, _Result ) \
  DEF_MATHOP_CC( /, _T1, _T2, _Result )

#define DEF_MATHOP_CC(_Op, _T1, _T2, _Result) \
Sketch<_Result> operator _Op (const Sketch<_T1> &lhs, const Sketch<_T2> &rhs) { \
  Sketch<_Result> result(lhs->getName() + #_Op + rhs->getName(), lhs); \
  _Result* dest = &(*result.pixels)[0]; \
  const _T1* src1 = &(*lhs.pixels)[0]; \
  const _T1* end1 = &(*lhs.pixels)[lhs->getNumPixels()]; \
  const _T2* src2 = &(*rhs.pixels)[0]; \
  while ( src1 != end1 ) \
    *dest++ = *src1++ _Op *src2++; \
  return result; \
} \
/* continued... */ \
Sketch<_Result> operator _Op (const Sketch<_T1> &lhs, const _T2 value) { \
  Sketch<_Result> result(lhs->getName() + #_Op + "scalar", lhs); \
  _Result* dest = &(*result.pixels)[0]; \
  const _T1* src1 = &(*lhs.pixels)[0]; \
  const _T1* end1 = &(*lhs.pixels)[lhs->getNumPixels()]; \
  while ( src1 != end1 ) \
    *dest++ = *src1++ _Op (_Result)value; \
  return result; \
}

// DEF_MATHOPS(bool, bool, bool) disallowed because valarray<bool> doesn't provide arithmetic
DEF_MATHOPS_CC(bool, uchar, uchar)
DEF_MATHOPS_CC(bool, uint, uint)
DEF_MATHOPS_CC(bool, float, float)

DEF_MATHOPS_CC(uchar, bool, uchar)
DEF_MATHOPS_CC(uchar, uchar, uchar)
DEF_MATHOPS_CC(uchar, uint, uint)
DEF_MATHOPS_CC(uchar, float, float)

DEF_MATHOPS_CC(usint, bool, usint)
DEF_MATHOPS_CC(usint, uchar, usint)
DEF_MATHOPS_CC(usint, usint, usint)
DEF_MATHOPS_CC(usint, float, float)

DEF_MATHOPS_CC(uint, bool, uint)
DEF_MATHOPS_CC(uint, uchar, uint)
DEF_MATHOPS_CC(uint, uint, uint)
DEF_MATHOPS_CC(uint, float, float)

DEF_MATHOPS_CC(float, bool, float)
DEF_MATHOPS_CC(float, uchar, float)
DEF_MATHOPS_CC(float, uint, float)
DEF_MATHOPS_CC(float, float, float)

#undef DEF_MATHOPS_CC
#undef DEF_MATHOP_CC

#define DEF_MATHOPS_INT_CC(_T1) \
  DEF_MATHOP_INT_CC( +, _T1) \
  DEF_MATHOP_INT_CC( -, _T1) \
  DEF_MATHOP_INT_CC( *, _T1) \
  DEF_MATHOP_INT_CC( /, _T1)

#define DEF_MATHOP_INT_CC(_Op, _T1) \
Sketch<_T1> operator _Op (const Sketch<_T1>& lhs, const int value) { \
  Sketch<_T1> result(lhs->getName() + #_Op + "scalar", lhs); \
  *result.pixels = *lhs.pixels _Op (_T1)(value); \
  return result; \
}

//DEF_MATHOPS_INT(bool, uchar) disallowed because valarray won't mix types
DEF_MATHOPS_INT_CC(uchar)
DEF_MATHOPS_INT_CC(usint)
DEF_MATHOPS_INT_CC(uint)
DEF_MATHOPS_INT_CC(float)

#undef DEF_MATHOPS_INT_CC
#undef DEF_MATHOP_INT_CC

// Define a special version of int math ops on Sketch<bool> that converts to Sketch<uchar>

#define DEF_MATHBOOL_INT_CC(_Op) \
Sketch<uchar> operator _Op (const Sketch<bool>& lhs, const int value) { \
  Sketch<uchar> result(lhs->getName() + #_Op + "scalar", lhs); \
  uchar* dest = &(*result.pixels)[0]; \
  const bool* src1 = &(*lhs.pixels)[0]; \
  const bool* end1 = &(*lhs.pixels)[lhs->getNumPixels()]; \
  while ( src1 != end1 ) \
    *dest++ = *src1++ _Op (uchar)value; \
  return result; \
}

DEF_MATHBOOL_INT_CC( + )
DEF_MATHBOOL_INT_CC( - )
DEF_MATHBOOL_INT_CC( * )
DEF_MATHBOOL_INT_CC( / )

#undef DEF_MATHBOOL_INT_CC

template<>
Sketch<bool>::operator Sketch<uchar>() const {
  /*
    std::cout << "Converting " << this << " '" << getName() << "'"
    << " id=" << getId() << ",parent=" << getParentId() << ",refcount=" << data->refcount
    << " to Sketch<uchar>\n";
  */
  Sketch<uchar> converted("uchar("+(*this)->getName()+")", *this);
  copyPixels(converted, *this);
  return converted;
}

template<>
Sketch<uchar>::operator Sketch<bool>() const {
  /*
    std::cout << "Converting " << this << " '" << getName() << "'"
    << " id=" << getId() << ",parent=" << getParentId() << ",refcount=" << data->refcount
    << " to Sketch<bool>\n";
  */
  Sketch<bool> converted("bool(" + (*this)->getName() + ")", *this);
  copyPixels(converted, *this);
  return converted;
}

Sketch<bool>& operator&= (Sketch<bool>& arg1, Sketch<bool> const& arg2) {
  *(arg1.pixels) &= *(arg2.pixels); 
  return arg1;
}

Sketch<bool>& operator|= (Sketch<bool>& arg1, Sketch<bool> const& arg2) {
  *(arg1.pixels) |= *(arg2.pixels); 
  return arg1;
}

Sketch<bool>& operator^= (Sketch<bool>& arg1, Sketch<bool> const& arg2) {
  *(arg1.pixels) ^= *(arg2.pixels); 
  return arg1;
}

} // namespace
