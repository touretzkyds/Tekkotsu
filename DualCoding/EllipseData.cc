//-*-c++-*-
#include <iostream>
#include <vector>
#include <list>
#include <math.h>

#include "SketchSpace.h"
#include "Sketch.h"
#include "Region.h"
#include "visops.h"

#include "ShapeSpace.h"  // required by DATASTUFF_CC
#include "ShapeRoot.h"   // required by DATASTUFF_CC

#include "EllipseData.h"
#include "ShapeEllipse.h"

#include "Crew/MapBuilder.h"
#include "VRmixin.h"

using namespace std;

namespace DualCoding {

inline int round(float x) { return (int) ceil((double)x-0.5f); }

EllipseData::EllipseData(ShapeSpace& _space, const Point &c, 
			 const float _semimajor, const float _semiminor, const float _orientation) 
  : BaseData(_space, getStaticType()),
    center_pt(c), semimajor(_semimajor), semiminor(_semiminor), orientation(_orientation)
{ if ( semimajor < semiminor ) {
    swap(semimajor,semiminor);
    orientation = AngPi(float(orientation) + M_PI/2);
  }
  center_pt.setRefFrameType(getRefFrameType());
  mobile = ELLIPSE_DATA_MOBILE;
}
  
DATASTUFF_CC(EllipseData);

BoundingBox2D EllipseData::getBoundingBox() const {
  float t_x = atan(-semiminor * tan(orientation) / semimajor);
  float t_y = atan( semiminor / tan(orientation) / semimajor);
  BoundingBox2D b;
  // derived from parametrization of the ellipse
  float o_sin = sin(orientation), t_x_sin = sin(t_x), t_y_sin = sin(t_y);
  float o_cos = cos(orientation), t_x_cos = cos(t_x), t_y_cos = cos(t_y);
  b.expand(fmat::pack(center_pt.coordX() + semimajor*t_x_cos*o_cos - semiminor*t_x_sin*o_sin,
                      center_pt.coordY() + semiminor*t_y_sin*o_cos + semimajor*t_y_cos*o_sin));
  // shift t by PI for both x and y
  t_x += M_PI; t_y += M_PI;
  t_x_sin = -t_x_sin;
  t_x_cos = -t_x_cos;
  t_y_sin = -t_y_sin;
  t_y_cos = -t_y_cos;
  b.expand(fmat::pack(center_pt.coordX() + semimajor*t_x_cos*o_cos - semiminor*t_x_sin*o_sin,
                      center_pt.coordY() + semiminor*t_y_sin*o_cos + semimajor*t_y_cos*o_sin));
  return b;
}

bool EllipseData::isMatchFor(const ShapeRoot& other) const {
  if (!(isSameTypeAs(other) && isSameColorAs(other)))
    return false;
  const Shape<EllipseData>& other_ellipse = ShapeRootTypeConst(other,EllipseData);
  float dist = center_pt.distanceFrom(other_ellipse->centerPt());
  return dist < 2*max(semimajor,other_ellipse->semimajor); // *** DST hack
}

bool EllipseData::isAdmissible() const {
	return (semimajor >= VRmixin::mapBuilder->curReq->minEllipseSemiMajor);
}

bool EllipseData::updateParams(const ShapeRoot& other, bool) {
  const Shape<EllipseData>& other_ellipse = ShapeRootTypeConst(other,EllipseData);
  if (other_ellipse->confidence <= 0)
    return false;
  const int other_conf = other_ellipse->confidence;
  center_pt = (center_pt*confidence + other_ellipse->centerPt()*other_conf) / (confidence+other_conf);
  semimajor = (semimajor*confidence + other_ellipse->getSemimajor()*other_conf) / (confidence+other_conf);
  semiminor = (semiminor*confidence + other_ellipse->getSemiminor()*other_conf) / (confidence+other_conf);
  orientation = orientation*((orientation_t)confidence/(confidence+other_conf))
    + other_ellipse->getOrientation()*((orientation_t)confidence/(confidence+other_conf));
  return true;
}

//! Print information about this shape. (Virtual in BaseData.)
void EllipseData::printParams() const {
  cout << "Type = " << getTypeName() << "  ID=" << getId() << "  ParentID=" << getParentId() << endl;
  printf("  color = %d %d %d\n",getColor().red,getColor().green,getColor().blue);
  cout << "  center = " << centerPt() << endl;
  cout << "  semimajor = " << getSemimajor()
       << ", semiminor = " << getSemiminor()
       << ", orientation = " << getOrientation() << endl;
  cout << "  mobile = " << getMobile() << ", viewable = " << isViewable() << endl;
}

pair<Point,Point> EllipseData::findFeaturePoints() const {
  const AngPi theta1 = getOrientation();
  const float d1 = getSemimajor();
  const fmat::Column<3> from_center1 = fmat::pack(d1*cos(theta1), d1*sin(theta1), 0.f);
  const Point majorPt = Point(from_center1) + center_pt;

  const float d2 = getSemiminor();
  const AngPi theta2 = theta1 + AngPi((orientation_t)M_PI/2);
  const fmat::Column<3> from_center2 = fmat::pack(d2*sin(theta2), d2*cos(theta2), 0.f);
  const Point minorPt = Point(from_center2) + center_pt;

  return pair<Point,Point>(majorPt,minorPt);
}

//! Transformations. (Virtual in BaseData.)
void EllipseData::applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref) {
  pair<Point,Point> featurePts = findFeaturePoints();
  center_pt.applyTransform(Tmat,newref);
  //  orientation = orientation - (AngTwoPi)atan2(Tmat(1,2),Tmat(1,1));
  featurePts.first.applyTransform(Tmat,newref);
  featurePts.second.applyTransform(Tmat,newref);
  updateProperties(featurePts.first, featurePts.second);
}

void EllipseData::updateProperties(const Point& majorPt, const Point& minorPt) {
  setSemiminor(minorPt.xyDistanceFrom(center_pt));
  setSemimajor(majorPt.xyDistanceFrom(center_pt));
  setOrientation(atan2(majorPt.coordY()-center_pt.coordY(), majorPt.coordX()-center_pt.coordX()));
}

void EllipseData::projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane) {
  pair<Point,Point> featurePts = findFeaturePoints();
  center_pt.projectToGround(camToBase,groundplane);
  featurePts.first.projectToGround(camToBase,groundplane);  
  featurePts.second.projectToGround(camToBase,groundplane);
  updateProperties(featurePts.first, featurePts.second);
}

//! Functions to set properties.
//{
void EllipseData::setOrientation(const AngPi _orientation) {
  orientation = AngPi(_orientation);
  deleteRendering();
}

void EllipseData::setSemimajor(float _semimajor) {
  semimajor = _semimajor;
  deleteRendering();
}

void EllipseData::setSemiminor(float _semiminor) {
  semiminor = _semiminor;
  deleteRendering();
}
//}


bool EllipseData::AreaLessThan::operator() (const Shape<EllipseData> &ellipse1, const Shape<EllipseData> &ellipse2) const {
      return ellipse1->getArea() < ellipse2->getArea();
}

// ==================================================
// BEGIN SKETCH MANIPULATION AND ELLIPSE EXTRACTION CODE
// ==================================================


//! Ellipse extraction.

std::vector<Shape<EllipseData> > EllipseData::extractEllipses(const Sketch<bool>& sketch) {
  const float AREA_TOLERANCE = 0.5f;
  const int REGION_THRESH = 25;
  NEW_SKETCH_N(labels,uint,visops::oldlabelcc(sketch,visops::EightWayConnect));
  list<Region> regionlist = Region::extractRegions(labels,REGION_THRESH);
  std::vector<Shape<EllipseData> > ellipses;
  
  if(regionlist.empty())
    return ellipses;
  
  typedef list<Region>::iterator R_IT;
  for (R_IT it = regionlist.begin(); it != regionlist.end(); ++it) {
    float ratio = it->findSemiMajorAxisLength()/(float)(it->findSemiMinorAxisLength());
    if((ratio < 3.0) && (ratio > 1.0f/3.0f)
       && (it->findArea() > M_PI*2.0*(it->findSemiMajorAxisLength())
	   *2.0*(it->findSemiMinorAxisLength())*AREA_TOLERANCE/4.0)) {
      Shape<EllipseData> temp_ellipse(*it);
      temp_ellipse->setParentId(sketch->getViewableId());
      temp_ellipse->setColor(sketch->getColor());
      ellipses.push_back(Shape<EllipseData>(temp_ellipse));
    };
  }
  return ellipses;
}


//! Render into a sketch space and return reference. (Private.)
Sketch<bool>* EllipseData::render() const {
  SketchSpace &SkS = space->getDualSpace();
  fmat::Column<3> ctr(centerPt().getCoords());
  SkS.applyTmat(ctr);
  const float &cx = ctr[0];
  const float &cy = ctr[1];
  const fmat::Transform &Tmat = SkS.getTmat();
  fmat::Column<2> ori;
  ori[0] = cos(orientation);
  ori[1] = sin(orientation);
  fmat::Matrix<2,2> rot;
  rot(0,0) = Tmat(0,0);
  rot(0,1) = Tmat(0,1);
  rot(1,0) = Tmat(1,0);
  rot(1,1) = Tmat(1,1);
  ori = rot * ori;
  const float &cosT = ori[0];
  const float &sinT = ori[1];
  const float xRange = semimajor;  // don't scale this because cosT and sinT are scaled
  const float majorSq = xRange*xRange;
  const float mnrDevMjr = semiminor/semimajor;
  Sketch<bool> result(SkS, "render("+getName()+")");
  result = 0;
  for (float xDist = -xRange; xDist <= xRange; xDist+=0.2f) {
    const float yRange = sqrt(max((float)0, majorSq - xDist*xDist)) * mnrDevMjr;
    for (float yDist = -yRange; yDist <= yRange; yDist+=0.2f) {
      int const px = round(cx+xDist*cosT-yDist*sinT);
      int const py = round(cy+yDist*cosT+xDist*sinT);
      if ( px >= 0 && px < result.width &&
           py >= 0 && py < result.height )
        result(px,py) = true;
    }
  }
  return new Sketch<bool>(result);
}

} // namespace
