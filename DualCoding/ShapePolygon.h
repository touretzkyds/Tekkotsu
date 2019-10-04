//-*-c++-*-
#ifndef _SHAPEPOLYGON_H_
#define _SHAPEPOLYGON_H_

#include "ShapeRoot.h"
#include "ShapeLine.h"
#include "PolygonData.h"

namespace DualCoding {

class ShapeSpace;
class Point;
class Region;

template<>
class Shape<PolygonData> : public ShapeRoot {
public:
  SHAPESTUFF_H(PolygonData);   // defined in ShapeRoot.h

  Shape<PolygonData>(const LineData& side) : ShapeRoot(addShape(new PolygonData(side))) {};

  Shape<PolygonData>(const Shape<LineData>& side)
    : ShapeRoot(addShape(new PolygonData(LineData(side.getSpace(), side->end1Pt(), side->end2Pt())))) {};
  
};

} // namespace

#endif
