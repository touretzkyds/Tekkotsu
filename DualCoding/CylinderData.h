#ifndef _CYLINDER_DATA_
#define _CYLINDER_DATA_

#include "Crew/MapBuilderRequest.h"
#include "DualCoding/BaseData.h"
#include "DualCoding/Point.h"
#include "DualCoding/ShapeTypes.h"
#include "SketchTypes.h"   // uchar
#include <map>

namespace DualCoding {
	
//!	Vertical standing cylinder
class CylinderData: public BaseData {
protected:
  Point centroid;
  float height;
  float radius;
  fmat::Quaternion orientation;
 	
public:
  //! Constructor
  CylinderData(ShapeSpace& _space, Point _centroid, float _height=0, float _radius=0, fmat::Quaternion _orientation=fmat::Quaternion());

  float getHeight() const { return height; }
  float getRadius() const { return radius; }
  const fmat::Quaternion getOrientation() const { return orientation; }
  
  static ShapeType_t getStaticType() { return cylinderDataType; }
  DATASTUFF_H(CylinderData);
  
  virtual bool isMatchFor(const ShapeRoot&) const;
  //! return the centroid of the shape in point format
  virtual Point getCentroid() const;
  virtual bool updateParams(const ShapeRoot&, bool);

  //! Returns the bounding box of the cylinder
  virtual BoundingBox2D getBoundingBox() const;

  virtual short unsigned int getDimension() const { return 3; }
  virtual void printParams() const;
  virtual void applyTransform(const fmat::Transform&, ReferenceFrameType_t);
  virtual void projectToGround(const fmat::Transform&, const PlaneEquation&);
  virtual Sketch<bool>* render() const;
  
  //! Import cylinders of specified colors from Sketch<uchar> as a vector of Shape<CylinderData>
  static std::vector<Shape<CylinderData> >
  extractCylinders(const Sketch<uchar> &sketch, 
									 const std::set<color_index>& colors,
									 const std::map<color_index,coordinate_t>& assumedHeights,
									 const std::map<color_index,int>& minCylinderAreas,
									 int maxcylinders,
									 std::vector<GazePoint> &addGazePts);

};
  
}

#endif
