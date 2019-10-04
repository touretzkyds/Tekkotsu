#ifndef _SHAPECROSS_H_
#define _SHAPECROSS_H_

#include "ShapeRoot.h"
#include "CrossData.h"

namespace DualCoding {

class ShapeSpace;

template<>
class Shape<CrossData> : public ShapeRoot {
 public:
  SHAPESTUFF_H(CrossData);   // defined in ShapeRoot.h

	Shape<CrossData>(ShapeSpace& space, const Point& center, const LineData line1, const LineData line2,
						 const fmat::Column<3>& extents)
		: ShapeRoot(addShape(new CrossData(space, line1, line2, extents))) {}
};

} // namespace

#endif
