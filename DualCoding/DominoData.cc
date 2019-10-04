#include "DominoData.h"
#include "ShapeDomino.h"

namespace DualCoding {

DATASTUFF_CC(DominoData);

void DominoData::flipLeftRight() {
	std::swap(GFL,GBR);
	std::swap(GFR,GBL);
	std::swap(TFL,TBR);
	std::swap(TFR,TBL);
	orientation = fmat::Quaternion::aboutZ(M_PI) * orientation;
}

bool DominoData::isMatchFor(const ShapeRoot& other) const {
  if (!(isSameTypeAs(other) && isSameColorAs(other)))
    return false;
  const Shape<DominoData>& other_domino = ShapeRootTypeConst(other,DominoData);
	if ( lowValue != other_domino->getLowValue() ||
			 highValue != other_domino->getHighValue() )
		return false;
  float dist = getCentroid().distanceFrom(other_domino->getCentroid());
  return dist < 100;
}

void DominoData::computeGraspPoint(const ObjectFeature pos,
																	 Point &location,
																	 AngTwoPi &orient) const {
	if ( pos == center )
		location = getCentroid();
	else if ( pos == highEnd )
		location = LineData(getSpace(),getTFR(),getGBR()).getCentroid();
	else if ( pos == lowEnd )
		location = LineData(getSpace(),getTFL(),getGBL()).getCentroid();

	orient = (getTFR() - getTFL()).atanYX() + AngTwoPi(pos == highEnd ? M_PI : 0.f);
}

void DominoData::computeTargetPosition(const ObjectFeature pos,
																			 Point &location,
																			 AngTwoPi &orient) const {
	orient = (getTFR() - getTFL()).atanYX() + (AngTwoPi)(pos == highEnd) ? M_PI : 0.f;
	switch (pos) {
	case none:
	case center:
	case lowEnd:
	case highEnd:
		computeGraspPoint(pos,location,orient);
		location = ((location - getCentroid()) * 2) + getCentroid();  // *** what is this doing???
		break;
	case rightSide:
		location = (LineData(getSpace(),getTBR(),getGBL()).getCentroid() - 
								getCentroid()).unitVector()*((75./2) + (200./2)) + getCentroid();
		orient += (AngTwoPi)(M_PI/2);
		break;
	case leftSide:
		location = (LineData(getSpace(),getTFR(),getGFL()).getCentroid() -
								getCentroid()).unitVector()*((75./2) + (200./2)) + getCentroid();
		orient -= (AngTwoPi)(M_PI/2);
		break;
	}
}


} // namespace
