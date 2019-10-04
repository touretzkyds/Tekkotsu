//-*-c++-*-
#ifndef _SHAPELINE_H_
#define _SHAPELINE_H_

#include "Shared/Measures.h"
#include "ShapeRoot.h"
#include "LineData.h"

namespace DualCoding {

class ShapeSpace;
class Point;

//! Smart pointer to a @a LineData object
template<> class Shape<LineData> : public ShapeRoot {
public:

  SHAPESTUFF_H(LineData);  // defined in ShapeRoot.h

  //! Make a line from two points.
  Shape<LineData>(ShapeSpace &s, const Point &end1pt, const Point &end2pt)
    : ShapeRoot(addShape(new LineData(s, end1pt, end2pt))) { };
	
  //! Make a line from a point and an orientation.
  Shape<LineData>(ShapeSpace &s, const Point &colinear_pt, AngPi orientation);

  //! Make a line from a point and a slope.
  Shape<LineData>(ShapeSpace &s, const Point &colinear_pt, Slope slope);

  //! Copy constructor.
  explicit Shape<LineData>(const LineData& newData) : ShapeRoot(addShape(new LineData(newData))) {};

};

} // namespace

#endif
