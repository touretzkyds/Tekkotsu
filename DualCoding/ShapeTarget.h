//-*-c++-*-
#ifndef _SHAPETARGET_H_
#define _SHAPETARGET_H_

#include "ShapeRoot.h"
#include "TargetData.h"

namespace DualCoding {

class ShapeSpace;
class Point;
class EndPoint;

template<>
class Shape<TargetData> : public ShapeRoot {
public:
  SHAPESTUFF_H(TargetData);  // defined in ShapeRoot.h

  //! Make a target from a point, two end points, and a height
  Shape<TargetData>(ShapeSpace &s, const EndPoint &frontLeftPt, const EndPoint &frontRightPt, const EndPoint &backLeftPt, const EndPoint &backRightPt, const EndPoint &frontIntersect, const EndPoint &backIntersect, const float height)
    : ShapeRoot(addShape(new TargetData(s, frontLeftPt, frontRightPt, backLeftPt, backRightPt, frontIntersect, backIntersect, height))) {}
	
  //! Copy constructor.
  Shape<TargetData>(const TargetData& newData) : ShapeRoot(addShape(new TargetData(newData))) {}

};

} // namespace

#endif
