//-*-c++-*-
#ifndef _SHAPEELLIPSE_H_
#define _SHAPEELLIPSE_H_

#include "ShapeRoot.h"
#include "EllipseData.h"

namespace DualCoding {

class ShapeSpace;
class Point;
class Region;

//! Smart pointer to an @a EllipseData object
template<>
class Shape<EllipseData> : public ShapeRoot {
public:
  SHAPESTUFF_H(EllipseData);   // defined in ShapeRoot.h

  Shape<EllipseData>(ShapeSpace &s, Point &centerval)
    : ShapeRoot(addShape(new EllipseData(s,centerval))) {};
	
  Shape<EllipseData>(Region& region);

};

} // namespace

#endif
