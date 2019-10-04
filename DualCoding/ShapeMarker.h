//-*-c++-*-
#ifndef _SHAPEMARKER_H_
#define _SHAPEMARKER_H_

#include "ShapeRoot.h"
#include "MarkerData.h"

namespace DualCoding {

class ShapeSpace;

template<>
class Shape<MarkerData> : public ShapeRoot {
public:
  SHAPESTUFF_H(MarkerData);   // defined in ShapeRoot.h
};

} // namespace

#endif // _SHAPEMARKER_H_
