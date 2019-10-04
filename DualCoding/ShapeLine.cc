//-*-c++-*-
#include <math.h>
#include <vector>

#include "ShapeLine.h"

namespace DualCoding {

SHAPESTUFF_CC(LineData);   // defined in ShapeRoot.h

Shape<LineData>::Shape(ShapeSpace &s, const Point &colinear_pt, AngPi orientation)
  : ShapeRoot(addShape(new LineData(s, colinear_pt,
				    Point(colinear_pt.coordX() + 10 * std::cos((orientation_t)orientation),
				    colinear_pt.coordY() + 10 * std::sin((orientation_t)orientation)))))
{
	(*this)->end1_pt.setActive(false);
	(*this)->end1_pt.setValid(false);

	(*this)->end2_pt.setActive(false);
	(*this)->end2_pt.setValid(false);
}


Shape<LineData>::Shape(ShapeSpace &s, const Point &colinear_pt, Slope slope) 
  : ShapeRoot(addShape(new LineData(s, colinear_pt,
				    Point(colinear_pt.coordX()+10,
					  slope * (colinear_pt.coordX()+10) +
					  (colinear_pt.coordY()-slope*colinear_pt.coordX())))))
{
	(*this)->end1_pt.setActive(false);
	(*this)->end1_pt.setValid(false);

	(*this)->end2_pt.setActive(false);
	(*this)->end2_pt.setValid(false);
}

} // namespace
