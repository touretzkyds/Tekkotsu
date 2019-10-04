//-*-c++-*-
#ifndef _SHAPEPOINT_H_
#define _SHAPEPOINT_H_

#include "ShapeRoot.h"
#include "PointData.h"
#include "EndPoint.h"

namespace DualCoding {

class ShapeSpace;
class Point;

template<>
class Shape<PointData> : public ShapeRoot {
public:
  SHAPESTUFF_H(PointData);   // defined in ShapeRoot.h

  explicit Shape<PointData>(ShapeSpace &s, const Point &pt)
    : ShapeRoot(addShape(new PointData(s,pt))) {};

  explicit Shape<PointData>(const PointData& newData) : 
    ShapeRoot(addShape(new PointData(newData))) {};
};

} // namespace

#endif
