#ifndef _NAUGHT_DATA_
#define _NAUGHT_DATA_

#include "CylinderData.h"
#include "Crew/MapBuilderRequest.h"
#include "BaseData.h"
#include "Point.h"
#include "ShapeTypes.h"
#include "SketchTypes.h"   // uchar
#include <map>

namespace DualCoding {

class NaughtData : public BaseData {
private:
  Point centroid;
  float height;
  float radius;

public:
  //! Constructor
  NaughtData(ShapeSpace& _space, const Point& _centroid, float _height=0, float _radius=0);

  //! Constructor
  NaughtData(Region &region, float assumedHeight);

  static ShapeType_t getStaticType() { return naughtDataType; }

  DATASTUFF_H(NaughtData);

  //! Centroid. (Virtual in BaseData.)
  Point getCentroid() const { return centroid; }
  void setCentroidPt(const Point& other) { centroid.setCoords(other); }

  float getRadius() const { return radius; }
  float getHeight() const { return height; }

  virtual bool isMatchFor(const ShapeRoot& other) const;

  virtual bool updateParams(const ShapeRoot& other, bool force=false);

  //! Print information about this shape. (Virtual in BaseData.)
  virtual void printParams() const;

  virtual short unsigned int getDimension() const { return 2; }

  //! Transformations. (Virtual in BaseData.)
  void applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref=unspecified);
	virtual void projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane);

  virtual Sketch<bool>* render() const;

	// Extracts naughts using regions, similar to how EllipseData does it
  static std::vector<Shape<NaughtData> > extractNaughts(const Sketch<bool> &sketch, const fmat::Column<3>& dimensions);

};

}

#endif
