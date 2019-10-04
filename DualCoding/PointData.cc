//-*-c++-*-

#include <iostream>
#include <vector>

#include "BaseData.h"    // superclass
#include "Point.h"       // Point data member
#include "ShapeTypes.h"  // pointDataType

#include "SketchSpace.h"
#include "Sketch.h"
#include "visops.h"

#include "ShapeSpace.h"  // required by DATASTUFF_CC
#include "ShapeRoot.h"   // required by DATASTUFF_CC

#include "PointData.h"
#include "ShapePoint.h"

using namespace std;

namespace DualCoding {

PointData::PointData(ShapeSpace& _space, const Point &c) 
  : BaseData(_space,getStaticType()), Point(c) {
  refFrameType = space->getRefFrameType();
}
  
DATASTUFF_CC(PointData);

bool PointData::isMatchFor(const ShapeRoot& other) const {
  if (!(isSameTypeAs(other) && isSameColorAs(other)))
    return false;
  const Shape<PointData>& other_point = ShapeRootTypeConst(other,PointData);
  float dist = distanceFrom(other_point->getCentroid());
  return dist < 20; // *** DST hack
}

void PointData::mergeWith(const ShapeRoot& other) {
  const Shape<PointData>& other_point = ShapeRootTypeConst(other,PointData);
  if (other_point->confidence <= 0)
    return;
  const int other_conf = other_point->confidence;
  confidence += other_conf;
	Point::operator=(((*this)*confidence + other_point->getCentroid()*other_conf) / (confidence+other_conf));
}

bool PointData::updateParams(const ShapeRoot& other, bool) {
  const Shape<PointData>& other_point = *static_cast<const Shape<PointData>*>(&other);
  ++confidence;
	Point::operator=(((*this)*(confidence-1) + other_point->getCentroid())/confidence);
  deleteRendering();
  return true;
}

void
PointData::printParams() const {
  cout << "Type = " << getTypeName();
  cout << "Shape ID = " << getId() << endl;
  cout << "Parent ID = " << getParentId() << endl;
  
  // Print critical points.
  cout << endl;
  cout << "center{" << getCentroid().coordX() << ", " << getCentroid().coordY() << "}" << endl;
  printf("color = %d %d %d\n",getColor().red,getColor().green,getColor().blue);
  cout << "viewable = " << isViewable() << endl;
}


void PointData::applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref) {
  Point::applyTransform(Tmat,newref);
}

void PointData::projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane) {
	Point::projectToGround(camToBase,groundplane);  // call Point's method
}

// ==================================================
// BEGIN SKETCH MANIPULATION AND POINT EXTRACTION CODE
// ==================================================


//! Point extraction.

std::vector<ShapeRoot> PointData::extractPoints(const Sketch<bool>& sketch)
{
  SketchSpace &SkS = sketch->getSpace();
  ShapeSpace &ShS = SkS.getDualSpace();
  std::vector<ShapeRoot> result;
  size_t num_pixels = sketch.width * sketch.height;
  for ( size_t i=0; i < num_pixels; i++ )
    if ( sketch[i] ) {
      int const y = int(i/sketch.width);
      int const x = i - y*sketch.width;
      Point pt(x,y,0);
      SkS.applyTmat(pt.getCoords());
      Shape<PointData> new_point_shape(new PointData(ShS,pt));
      new_point_shape->inheritFrom(*sketch.rootGetData());
      result.push_back(new_point_shape);
    }
  return result;
}


//! Render into a sketch space and return a pointer. (Private.)
Sketch<bool>* PointData::render() const {
  SketchSpace &SkS = space->getDualSpace();
  fmat::Column<3> ctr(getCentroid().getCoords());
  SkS.applyTmat(ctr);
  int const cx = int(ctr[0]);
  int const cy = int(ctr[1]);
  Sketch<bool>& draw_result = 
    *new Sketch<bool>(SkS, "render("+getName()+")");
  draw_result = false;
  draw_result(cx,cy) = true;  
  return &draw_result;
}

PointData& PointData::operator=(const PointData& other) {
  if (&other == this)
    return *this;
  BaseData::operator=(other);
	Point::operator=(other);
  return *this;
}

} // namespace
