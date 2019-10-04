#ifndef _DOMINODATA_H_
#define _DOMINODATA_H_

#include "BrickData.h"

namespace DualCoding {

//! Subclass of Brick that describes a Domino
class DominoData : public BrickData {

public:
  DominoData(ShapeSpace& _space, int _lowValue, int _highValue,
						 const fmat::SubVector<3,const fmat::fmatReal>& _centroid,
						 const fmat::Column<3> _extents,
						 const fmat::SubMatrix<3,3,const fmat::fmatReal> &_orient) :
		BrickData(_space, _centroid, _extents, _orient), lowValue(_lowValue), highValue(_highValue),
		length(2*_extents[0]), width(2*_extents[1]), height(2*_extents[2]), lineColor()
		{
			type = dominoDataType;
		}

  DATASTUFF_H(DominoData);

  static ShapeType_t getStaticType() { return dominoDataType; }

	int getLowValue() const { return lowValue; }
	int getHighValue() const { return highValue; }
	void setValues(int low, int high) { lowValue=low; highValue=high; }

	float getLength() const { return length; }
	float getWidth()  const { return width; }
	float getHeight() const { return height; }

	rgb getLineColor() const { return lineColor; }
  void setLineColor(rgb _lineColor) { lineColor = _lineColor; }

  //! Match dominoes based on their parameters.  (Virtual in BaseData.)
  virtual bool isMatchFor(const ShapeRoot& other) const;

	void flipLeftRight(); //! Helper function for domino construction

  enum ObjectFeature { none=0, center, lowEnd, highEnd, leftSide, rightSide };

	//! Compute an object grasp point based on some feature of the object, e.g., the low end of a domino.
  void computeGraspPoint(const ObjectFeature pos,
												 Point &location,
												 AngTwoPi &orient) const;

	//! Compute a point on the destination target based on some feature of the target, e.g., the left side of a domino.
  void computeTargetPosition(const ObjectFeature pos,
                             Point &location,
                             AngTwoPi &orient) const;

protected:
	int lowValue;   //!< number of dots on low side
	int highValue;  //!< number of dots on high side
	float length; //!< length of the domino (longest dimension)
	float width; //!< width of the domino (smaller than length);
	float height; //!< height of the domino
  rgb lineColor; //!< color of the dividing line
};

} // namespace

#endif
