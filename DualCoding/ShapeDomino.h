#ifndef _SHAPEDOMINO_H_
#define _SHAPEDOMINO_H_

#include "ShapeRoot.h"
#include "DominoData.h"

namespace DualCoding {

class ShapeSpace;

template<>
class Shape<DominoData> : public ShapeRoot {
 public:
  SHAPESTUFF_H(DominoData);   // defined in ShapeRoot.h

	Shape<DominoData>(ShapeSpace& space, unsigned char lowValue, unsigned char highValue,
										const fmat::SubVector<3,const fmat::fmatReal>& centroid,
										fmat::Column<3> _extents,
										const fmat::SubMatrix<3,3,const fmat::fmatReal>& _orient)
		: ShapeRoot(addShape(new DominoData(space, lowValue, highValue, centroid, _extents, _orient))) {}
};

} // namespace

#endif
