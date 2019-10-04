#include <iostream>

using namespace std;

#include "SketchSpace.h"
#include "Region.h"

#include "ShapeSphere.h"

namespace DualCoding {

SHAPESTUFF_CC(SphereData);
	
Shape<SphereData>::Shape(Region& region)
  : ShapeRoot(addShape(new SphereData(region.getSpace().getDualSpace(), 
				      region.findCentroid(), region.findRadius()))) {};

} // namespace
