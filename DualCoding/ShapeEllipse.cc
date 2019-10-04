//-*-c++-*-
#include <iostream>

using namespace std;

#include "SketchSpace.h"
#include "Region.h"

#include "ShapeEllipse.h"

namespace DualCoding {

SHAPESTUFF_CC(EllipseData);
	
Shape<EllipseData>::Shape(Region& region)
  : ShapeRoot(addShape(new EllipseData(region.getSpace().getDualSpace(),
				       region.findCentroid())))
{ 
	(*this)->setSemimajor(region.findSemiMajorAxisLength());
	(*this)->setSemiminor(region.findSemiMinorAxisLength());
	(*this)->setOrientation(region.findPrincipalAxisOrientation());
};


} // namespace
