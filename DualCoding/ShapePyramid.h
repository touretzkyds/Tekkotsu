//-*-c++-*-
#ifndef _SHAPEPYRAMID_H_
#define _SHAPEPYRAMID_H_

#include "ShapeRoot.h"
#include "PyramidData.h"

namespace DualCoding {

class ShapeSpace;
class Point;

template<>
class Shape<PyramidData> : public ShapeRoot {
public:
  SHAPESTUFF_H(PyramidData);   // defined in ShapeRoot.h

  Shape<PyramidData>(ShapeSpace &s, Point &FL, Point &FR, Point &BL, Point &BR, Point &Top)
    : ShapeRoot(addShape(new PyramidData(s, FL, FR, BL, BR, Top))) {};
	
};

} // namespace

#endif
