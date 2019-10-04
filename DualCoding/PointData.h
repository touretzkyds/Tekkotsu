//-*-c++-*-
#ifndef _POINTDATA_H_
#define _POINTDATA_H_

#include <vector>
#include <iostream>
#include <string>

#include "Shared/fmatSpatial.h"

#include "BaseData.h"    // superclass
#include "Point.h"       // Point data member
#include "ShapeTypes.h"  // pointDataType

namespace DualCoding {

class ShapeRoot;
class SketchSpace;
template<typename T> class Sketch;

class PointData : public BaseData, public Point {
public:
  //! Constructor
  PointData(ShapeSpace& _space, const Point &c);

  static ShapeType_t getStaticType() { return pointDataType; }

  DATASTUFF_H(PointData);
  
  //! Centroid. (Virtual in BaseData.)
  virtual Point getCentroid() const { return *this; }  
  
  BoundingBox2D getBoundingBox() const {
    return BoundingBox2D(coords);
  }

  //! Match points based on their parameters.  (Virtual in BaseData.)
  virtual bool isMatchFor(const ShapeRoot& other) const;

  virtual void mergeWith(const ShapeRoot& other);

  virtual bool updateParams(const ShapeRoot& other, bool force=false);

  //! Print information about this shape. (Virtual in BaseData.)
  virtual void printParams() const;
  
  //! Transformations. (Virtual in BaseData.)
  virtual void applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref=unspecified);
  
  //! Project to ground
  virtual void projectToGround(const fmat::Transform& camToBase,
			       const PlaneEquation& groundplane);

  //! Extraction.
  static std::vector<ShapeRoot> extractPoints(const Sketch<bool>& sketch);

  virtual unsigned short getDimension() const { return 0; }

  PointData& operator=(const PointData&);
  
private:
  //! Render into a sketch space and return reference. (Private.)
  virtual Sketch<bool>* render() const;
  //@}

};

} // namespace

#endif
