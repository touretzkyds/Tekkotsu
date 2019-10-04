#ifndef _SHAPENAUGHT_H_
#define _SHAPENAUGHT_H_

#include "ShapeRoot.h"
#include "NaughtData.h"
#include "Region.h"

namespace DualCoding {

class ShapeSpace;

template<>
class Shape<NaughtData> : public ShapeRoot {
 public:

  SHAPESTUFF_H(NaughtData);   // defined in ShapeRoot.h

	Shape<NaughtData>(ShapeSpace& space, Point& centroid, float height, float radius)
		: ShapeRoot(addShape(new NaughtData(space, centroid, height, radius))) {}

};

} // namespace

#endif
