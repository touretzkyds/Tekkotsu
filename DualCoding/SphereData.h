//-*-c++-*-
#ifndef _SPHEREDATA_H_
#define _SPHEREDATA_H_

#include <vector>
#include <iostream>

#include "BaseData.h"    // superclass
#include "Point.h"       // Point data member
#include "ShapeTypes.h"  // sphereDataType

namespace DualCoding {

class ShapeRoot;
class SketchSpace;
template<typename T> class Sketch;

#define SPHERE_DATA_MOBILE false

class SphereData : public BaseData {
private:
  Point centroid;
  float radius;

public:

  //! Constructor
  SphereData(ShapeSpace& _space, const Point &c, float r);

  //! Copy constructor
  SphereData(const SphereData& otherData);
  
  static ShapeType_t getStaticType() { return sphereDataType; }

  DATASTUFF_H(SphereData);
  
  friend class Shape<SphereData>;
  
  //! Centroid. (Virtual in BaseData.)
  Point getCentroid() const { return centroid; }  
  
  void setCentroidPt(const Point& pt) { centroid.setCoords(pt); }

  //! Match spheres based on their parameters.  (Virtual in BaseData.)
  virtual bool isMatchFor(const ShapeRoot& other) const;

  virtual void mergeWith(const ShapeRoot& other);

  virtual bool isAdmissible() const {
    return (radius >= 9.0 );  // **DST Hack** minimum size for a sphere to be added to local map
  }

  virtual bool updateParams(const ShapeRoot& other, bool force=false);

  //! Print information about this shape. (Virtual in BaseData.)
  virtual void printParams() const;
  
  //! Transformations. (Virtual in BaseData.)
  void applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref=unspecified);
  
  //! Project to ground
  virtual void projectToGround(const fmat::Transform& camToBase,
			       const PlaneEquation& groundplane);

  virtual bool isInside(const Point& pt) const;
  //! Center point access function.
  const Point& centerPt() const { return centroid; }
  
  virtual unsigned short getDimension() const { return 3; }

  //! Properties functions.
  //@{
  float getRadius() const { return radius; }
  //@}
  
  
  //! Set properties.
  //@{
  void setRadius(float _radius);
  //@}
  
  // ==================================================
  // BEGIN SKETCH MANIPULATION AND SPHERE EXTRACTION CODE
  // ==================================================
  
  
  //! Extraction.
  //@{
  static std::vector<Shape<SphereData> > extractSpheres(const Sketch<bool>& sketch);
  static std::vector<Shape<SphereData> > get_spheres(const Sketch<CMVision::uchar>& cam);
  static std::vector<Shape<SphereData> > get_spheres(const Sketch<CMVision::uchar>& cam,
					     std::vector<bool>& Valid_Colors);
  //@}
  
  
private:
  //! Render into a sketch space and return pointer. (Private.)
  Sketch<bool>* render() const;

  SphereData& operator=(const SphereData&); //!< don't call
};

} // namespace

#endif
