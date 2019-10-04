//-*-c++-*-
#include <iostream>
#include <math.h>
#include <vector>
#include <list>
#include "limits.h"

#include "SketchSpace.h"
#include "Sketch.h"
#include "Region.h"
#include "visops.h"

#include "ShapeSpace.h"
#include "ShapeRoot.h"

#include "PointData.h"
#include "LineData.h"
#include "ShapeLine.h"

#include "Crew/MapBuilder.h"
#include "VRmixin.h"

#ifdef PLATFORM_APERIOS
//! this is normally defined in <math.h>, but the OPEN-R cross-compiler isn't configured right
template<typename T> static inline int signbit(T x) { return x<0; }
#endif

using namespace std;

namespace DualCoding {

  DATASTUFF_CC(LineData);

  const float LineData::MIN_FRACTIONAL_LENGTH = 1.0/12;

  float LineData::extractorMinLineLength = 40;
  float LineData::extractorGapTolerance = 15;
  float LineData::minLinesPerpDist = 100;


  LineData::LineData(ShapeSpace& _space, const Point &p1, orientation_t orient)
    : BaseData(_space,getStaticType()), end1_pt(p1), end2_pt(),
      rho_norm(0), theta_norm(0), orientation(0), length(0) {
    int const width = space->getDualSpace().getWidth();
    int const height = space->getDualSpace().getHeight();
    // Use a large offset from p1 to p2 because SketchGUI must calculate
    // the line slope from p1/p2 coords transmitted as strings; we don't
    // transmit the orientation.  But don't use an offset so large that the line
    // goes off-screen.
    float p2x=0, p2y=0;
    if ( fabs(orient-M_PI/2) < 0.001 ) {
      p2x = p1.coordX();
      p2y = p1.coordY() > height/2 ? 0 : height-1;
    } else {
      float slope = tan(orient);
      float intcpt = p1.coordY() - p1.coordX()*slope;
      p2x = p1.coordX() >= width/2 ? 0 : width-1;
      p2y = p2x * slope + intcpt;
      if ( p2y > height-1 ) {
        p2y = height-1;
        p2x = (p2y-intcpt) / slope;
      } else if ( p2y < 0 ) {
        p2y = 0;
        p2x = -intcpt/slope;
      }
    }
    end2_pt = Point(p2x,p2y);
    end1_pt.setValid(false);
    end1_pt.setActive(false);
    end2_pt.setValid(false);
    end2_pt.setActive(false);
    update_derived_properties();
  }

  LineData::LineData(const LineData& other)
    : BaseData(other),
      end1_pt(other.end1_pt), end2_pt(other.end2_pt),
      rho_norm(other.rho_norm), theta_norm(other.theta_norm),
      orientation(other.orientation), length(other.length) {}

  Point LineData::getCentroid() const { return (end1Pt()+end2Pt())*0.5f; }

  void LineData::setInfinite(bool value) {
    end1_pt.setActive(!value);
    end2_pt.setActive(!value);
  }

#define LINE_MATCH_OVERLAP -20

  bool LineData::isMatchFor(const ShapeRoot& other) const {
    if (!(isSameTypeAs(other) && isSameColorAs(other)))
      return false;
    else {
      const Shape<LineData>& other_ln = ShapeRootTypeConst(other,LineData);
      return isMatchFor(other_ln.getData());
    }
  }

  bool LineData::isMatchFor(const LineData& other_line) const {
    AngPi orient_diff = angdist(orientation, other_line.orientation);
    float const linePerpDist = space->getRefFrameType() == camcentric ? 20 : minLinesPerpDist;
    AngPi const lineOrientationMatch =
      space->getRefFrameType() == camcentric ? M_PI/6 : M_PI/6;
    bool result = orient_diff < lineOrientationMatch
      && closestApproachTo(other_line) < linePerpDist
      && isOverlappedWith(other_line,LINE_MATCH_OVERLAP);
    /*
    if ( getId() == 10111 && other_line.getId() == 10043 )
      std::cout << "Line match " << getId() << " " << other_line.getId() << " orient_diff: " << orient_diff
                << " theta=" << theta_norm << " othr_theta=" << other_line.theta_norm
                << " rho=" << rho_norm << " othr_rho=" << other_line.rho_norm
                << " orient=" << orientation << " othr_orient=" << other_line.orientation
                << " closestApproach=" << closestApproachTo(other_line)
                << " : " << linePerpDist
                << " isOverlappedWith: " << isOverlappedWith(other_line,LINE_MATCH_OVERLAP)
                << " intersects=" << intersectsLine(other_line)
                << " result=" << result << std::endl;
    */
    return result;
  }

  Point LineData::projectionFrom(const Point &pt) const {
    fmat::Column<2> v = fmat::pack(end2_pt.coordX() - end1_pt.coordX(),
                                   end2_pt.coordY() - end1_pt.coordY()) / getLength();
    fmat::Column<2> u = fmat::pack(pt.coordX() - end1_pt.coordX(),
                                   pt.coordY() - end1_pt.coordY());
    float d = dotProduct(u,v);
    Point result(end1_pt);
    result.setCoords(end1_pt.coordX()+d*v[0], end1_pt.coordY()+d*v[1]);
    //if ( getId() == 10020 )
    //  std::cout << pt << " u=" << u << " v=" << v << " d=" << d << " result=" << result << std::endl;
    return result;
  }

  int LineData::blankPixelsOnLine(const Point &pt1, const Point &pt2, const Sketch<bool> &sketch) {
    float dist = (pt2-pt1).xyNorm();
    Point dir = (pt2-pt1) / dist;
    int count = 0;
    for (float i=0; i<dist; i++) {
      Point p = pt1 + dir * i;
      if ( ! sketch(int(p.coordX()), int(p.coordY())) )
        ++count;
    }
    return count;
  }

  bool LineData::isParasiteOf(const LineData& other_line, const Sketch<bool> &sketch) const {
    float const tol = 5;
    if ( getLength() < other_line.getLength()/3  &&
         intersectsLine(other_line) &&
         other_line.pointFallsWithin(end1_pt, tol) &&
         other_line.pointFallsWithin(end2_pt, tol) ) {
      Point int1 = other_line.projectionFrom(end1_pt);
      Point int2 = other_line.projectionFrom(end2_pt);
      int const MAX_BLANK_PIXELS = 10;
      if ( blankPixelsOnLine(end1_pt, int1, sketch) <= MAX_BLANK_PIXELS &&
           blankPixelsOnLine(end2_pt, int2, sketch) <= MAX_BLANK_PIXELS ) {
        /*
        if ( other_line.getId() == 10011 || other_line.getId() == 10022 )
          std::cout << "parasite: " << end1_pt << " of " << other_line.getId()
                    << " z1=" << blankPixelsOnLine(end1_pt, int1, sketch) << " " << end1_pt << " " << int1
                    << " z2=" << blankPixelsOnLine(end2_pt, int2, sketch) << " " << end2_pt << " " << int2
                    << std::endl;
        */
        return true;
      }
    }
    return false;
  }

  void LineData::mergeWith(const ShapeRoot& other) {
    const Shape<LineData>& other_line = ShapeRootTypeConst(other,LineData);
    if (other_line->confidence <= 0)
      return;
    //const int other_conf = other_line->confidence;
    //confidence += other_conf;
    // Favor longer lines because they're likely to be more reliable
    float const totalLengthSq = length * length + other_line->length * other_line->length;
    EndPoint new_end1_pt = (end1_pt*length*length/totalLengthSq + other_line->end1Pt()*other_line->length*other_line->length/totalLengthSq);
    EndPoint new_end2_pt = (end2_pt*length*length/totalLengthSq + other_line->end2Pt()*other_line->length*other_line->length/totalLengthSq);
    new_end1_pt.valid = end1_pt.isValid() && other_line->end1Pt().isValid();
    new_end2_pt.valid = end2_pt.isValid() && other_line->end2Pt().isValid();
    end1_pt = new_end1_pt;
    end2_pt = new_end2_pt;
    update_derived_properties();
  }

  bool LineData::isValidUpdate(coordinate_t c1_cur, coordinate_t c2_cur, coordinate_t c1_new, coordinate_t c2_new) {
    const float c1_noise = 10 + std::abs(c1_cur+c1_new) / 20.f; // allow larger error for shapes far from the robot
    const float c2_noise = 10 + std::abs(c2_cur+c2_new) / 20.f;
    return (c1_new-c1_noise < c1_cur && c2_cur < c2_new+c2_noise);
  }


  //! Update a line in the local map with info from a matching line in the ground space.
  bool LineData::updateParams(const ShapeRoot& ground_root, bool force) {
    const Shape<LineData>& ground_line = ShapeRootTypeConst(ground_root,LineData);
    return updateParams(ground_line.getData(), force);
  }

  //! Update a line in the local map with info from a matching line in the ground space.
	bool LineData::updateParams(const LineData &ground_line, bool force) {
    //  cout << "updating local Line " << getId() << " with data from ground line " << ground_line->getId() << ":" << endl;
    //  ground_line->printEnds();
    //  cout << "Update from " << endl;
    //  printEnds();

    const coordinate_t c1_cur = firstPtCoord();
    const coordinate_t c2_cur = secondPtCoord();

    Point _end1_pt = firstPt();
    Point _end2_pt = secondPt();

    updateLinePt(firstPt(), firstPtCoord(), firstPt(ground_line), firstPtCoord(ground_line), -1);
    updateLinePt(secondPt(), secondPtCoord(), secondPt(ground_line), secondPtCoord(ground_line), +1);
    //  cout << "to" << endl;
    //  printEnds();

    const coordinate_t c1_new = firstPtCoord();
    const coordinate_t c2_new = secondPtCoord();

    if (isValidUpdate(c1_cur, c2_cur, c1_new, c2_new) || force){
      //    cout << "was accepted, line updated" << endl;
      //    ++nsamples;
      update_derived_properties();
      return true;
    }

    //  cout << "was denied, line not updated" << endl;
    setEndPts(_end1_pt, _end2_pt);
    return false;
  }

  void LineData::updateLinePt(EndPoint& localPt, coordinate_t local_coord,
                              const EndPoint& groundPt, coordinate_t ground_coord,
                              int sign) {
    if ( groundPt.isValid() ) {
      if ( localPt.isValid() )
        localPt.updateParams(groundPt);
      else
        localPt = groundPt;
    }
    else if ( (ground_coord - local_coord)*sign > 0 )
      localPt = groundPt;
  }

  bool LineData::isAdmissible() const {
    if (end1Pt().isValid() && end2Pt().isValid())
      return length >= VRmixin::mapBuilder->curReq->minLineLength;
    else
      return length >= VRmixin::mapBuilder->curReq->minRayLength;
  }

  //! Print information about this shape. (Virtual in BaseData.)
  void LineData::printParams() const {
    cout << "Type = " << getTypeName() << "  ID=" << getId() << "  ParentID=" << getParentId() << endl;
    cout << "  end1{" << end1Pt().coordX() << ", " << end1Pt().coordY()  << "}"
         << " active=" << end1Pt().isActive()
         << " valid=" << end1Pt().isValid() << endl;

    cout << "  end2{" << end2Pt().coordX() << ", " << end2Pt().coordY() <<  "}"
         << " active=" << end2Pt().isActive()
         << " valid=" << end2Pt().isValid() << std::endl;

    cout << "  rho_norm=" << rho_norm
         << ", theta_norm=" << theta_norm
         << ", orientation=" << getOrientation()
         << ", length=" << getLength() << endl;

    printf("  color = %d %d %d\n",getColor().red,getColor().green,getColor().blue);

    cout << "  mobile=" << getMobile()
         << ", viewable=" << isViewable() << endl;

    vector<float> abc = lineEquation_abc();
    printf("  equ = %f %f %f\n",abc[0],abc[1],abc[2]);
  }

  void LineData::printEnds() const {
    cout << "  end1{" << end1Pt().coordX() << ", " << end1Pt().coordY()  << "}";
    cout << "  active=" << end1Pt().isActive() << ", valid=" << end1Pt().isValid() << endl;
    cout << "  end2{" << end2Pt().coordX() << ", " << end2Pt().coordY() <<  "}";
    cout << "  active=" << end2Pt().isActive() << ", valid=" << end2Pt().isValid() << endl;
  }


  // *** Transformations. *** //

  //! Apply a transformation to this shape.
  void LineData::applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref) {
    end1Pt().applyTransform(Tmat,newref);
    end2Pt().applyTransform(Tmat,newref);
    update_derived_properties();
  }

  void LineData::projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane) {
    end1Pt().projectToGround(camToBase,groundplane);
    end2Pt().projectToGround(camToBase,groundplane);
    update_derived_properties();
  }


  // *** Logical endpoints *** //

  EndPoint& LineData::leftPt() { return end1Pt().isLeftOf(end2Pt()) ? end1_pt : end2_pt; }
  const EndPoint& LineData::leftPt() const { return end1Pt().isLeftOf(end2Pt()) ? end1_pt : end2_pt; }
  EndPoint& LineData::rightPt() { return end1Pt().isLeftOf(end2Pt()) ? end2_pt : end1_pt; }
  const EndPoint& LineData::rightPt() const { return end1Pt().isLeftOf(end2Pt()) ? end2_pt : end1_pt; }
  EndPoint& LineData::topPt() { return end1Pt().isAbove(end2Pt()) ? end1_pt : end2_pt; }
  const EndPoint& LineData::topPt() const { return end1Pt().isAbove(end2Pt()) ? end1_pt : end2_pt; }
  EndPoint& LineData::bottomPt() { return end1Pt().isAbove(end2Pt()) ? end2_pt : end1_pt; }
  const EndPoint& LineData::bottomPt() const { return end1Pt().isAbove(end2Pt()) ? end2_pt : end1_pt; }

  Shape<PointData> LineData::leftPtShape() {
    Shape<PointData> result(new PointData(*space, leftPt()));
    result->setName("leftPt");
    result->inheritFrom(*this);
    result->setViewable(false);
    return result;
  }

  Shape<PointData> LineData::rightPtShape() {
    Shape<PointData> result(new PointData(*space, rightPt()));
    result->setName("rightPt");
    result->inheritFrom(*this);
    result->setViewable(false);
    return result;
  }

  Shape<PointData> LineData::topPtShape() {
    Shape<PointData> result(new PointData(*space, topPt()));
    result->setName("topPt");
    result->inheritFrom(*this);
    result->setViewable(false);
    return result;
  }

  Shape<PointData> LineData::bottomPtShape() {
    Shape<PointData> result(new PointData(*space, bottomPt()));
    result->setName("bottomPt");
    result->inheritFrom(*this);
    result->setViewable(false);
    return result;
  }

  EndPoint& LineData::firstPt() {
    if ( isNotVertical() )
      if ( end1Pt().coordX() < end2Pt().coordX() )
        return end1Pt();
      else return end2Pt();
    else
      if ( end1Pt().coordY() < end2Pt().coordY() )
        return end1Pt();
      else return end2Pt();
  }

  const EndPoint& LineData::firstPt() const {
    if ( isNotVertical() )
      if ( end1Pt().coordX() < end2Pt().coordX() )
        return end1Pt();
      else return end2Pt();
    else
      if ( end1Pt().coordY() < end2Pt().coordY() )
        return end1Pt();
      else return end2Pt();
  }

  const EndPoint& LineData::firstPt(const LineData &otherline) const {
    if ( isNotVertical() )
      if ( otherline.end1Pt().coordX() < otherline.end2Pt().coordX() )
        return otherline.end1Pt();
      else return otherline.end2Pt();
    else
      if ( otherline.end1Pt().coordY() < otherline.end2Pt().coordY() )
        return otherline.end1Pt();
      else return otherline.end2Pt();
  }

  EndPoint& LineData::secondPt() {
    if ( isNotVertical() )
      if ( end1Pt().coordX() > end2Pt().coordX() )
        return end1Pt();
      else return end2Pt();
    else
      if ( end1Pt().coordY() > end2Pt().coordY() )
        return end1Pt();
      else return end2Pt();
  }

  const EndPoint& LineData::secondPt() const {
    if ( isNotVertical() )
      if ( end1Pt().coordX() > end2Pt().coordX() )
        return end1Pt();
      else return end2Pt();
    else
      if ( end1Pt().coordY() > end2Pt().coordY() )
        return end1Pt();
      else return end2Pt();
  }

  const EndPoint& LineData::secondPt(const LineData &otherline) const {
    if ( isNotVertical() )
      if ( otherline.end1Pt().coordX() > otherline.end2Pt().coordX() )
        return otherline.end1Pt();
      else return otherline.end2Pt();
    else
      if ( otherline.end1Pt().coordY() > otherline.end2Pt().coordY() )
        return otherline.end1Pt();
      else return otherline.end2Pt();
  }

  Shape<PointData> LineData::firstPtShape() {
    Shape<PointData> result(new PointData(*space, firstPt()));
    result->setName("firstPt");
    result->inheritFrom(*this);
    result->setViewable(false);
    return result;
  }

  Shape<PointData> LineData::secondPtShape() {
    Shape<PointData> result(new PointData(*space, secondPt()));
    result->setName("secondPt");
    result->inheritFrom(*this);
    result->setViewable(false);
    return result;
  }

  coordinate_t LineData::firstPtCoord() const {
    return  isNotVertical() ? firstPt().coordX() : firstPt().coordY();
  }

  coordinate_t LineData::firstPtCoord(const LineData &otherline) const {
    return  isNotVertical() ?
      firstPt(otherline).coordX() :
      firstPt(otherline).coordY();
  }

  coordinate_t LineData::secondPtCoord() const {
    return  isNotVertical() ? secondPt().coordX() : secondPt().coordY();
  }

  coordinate_t LineData::secondPtCoord(const LineData &otherline) const {
    return  isNotVertical() ?
      secondPt(otherline).coordX() :
      secondPt(otherline).coordY();
  }


  // *** Functions to set endpoints. *** //

  void LineData::setEndPts(const EndPoint& _end1_pt, const EndPoint& _end2_pt) {
    end1_pt.setCoords(_end1_pt);
    end1_pt.setActive(_end1_pt.isActive());
    end1_pt.setValid(_end1_pt.isValid());
    end1_pt.setNumUpdates(_end1_pt.numUpdates());

    end2_pt.setCoords(_end2_pt);
    end2_pt.setActive(_end2_pt.isActive());
    end2_pt.setValid(_end2_pt.isValid());
    end2_pt.setNumUpdates(_end2_pt.numUpdates());

    update_derived_properties();
  }


  // *** Properties functions. *** //

  std::pair<float,float> LineData::lineEquation_mb() const {
    float m;
    if ((fabs(end2Pt().coordX() - end1Pt().coordX()) * BIG_SLOPE)
        <= fabs(end2Pt().coordY() - end2Pt().coordY()))
      m = BIG_SLOPE;
    else
      m = (end2Pt().coordY() - end1Pt().coordY())/(end2Pt().coordX() - end1Pt().coordX());
    float b = end1Pt().coordY() - m * end1Pt().coordX();
    return pair<float,float>(m,b);
  }


  //! Determine parameters a, b, c, d satisfying the equation ax + bz = c.
  std::vector<float> LineData::lineEquation_abc_xz() const {
    float dx = end2Pt().coordX() - end1Pt().coordX();
    float dz = end2Pt().coordZ() - end1Pt().coordZ();

    std::vector<float> abc(3,1);
    float& a = abc[0];
    float& b = abc[1];
    float& c = abc[2];

    // If vertical...b = 0
    if((dx == 0)
       || (dz/dx > BIG_SLOPE)) {
      a = 1;
      b = 0;
      c = end1Pt().coordX();
    }

    // If horizontal...a = 0
    else if((dz == 0)
            || (dx/dz > BIG_SLOPE)) {
      a = 0;
      b = 1;
      c = end1Pt().coordZ();
    }

    // If not horizontal or vertical...a = 1.0
    else {
      a = 1;
      b = (end1Pt().coordX() - end2Pt().coordX())
        / (end2Pt().coordZ() - end1Pt().coordZ());
      c = end1Pt().coordX() + b*end1Pt().coordZ();
    }

    return(abc);

  }

  //! Determine parameters a, b, c satisfying the equation ax + by = c.
  std::vector<float> LineData::lineEquation_abc() const {
    float dx = end2Pt().coordX() - end1Pt().coordX();
    float dy = end2Pt().coordY() - end1Pt().coordY();

    std::vector<float> abc(3,1);
    float& a = abc[0];
    float& b = abc[1];
    float& c = abc[2];

    // If vertical...b = 0
    if( std::abs(dx) < 1.0e-6f || Slope(dy/dx) > BIG_SLOPE) {
      a = 1;
      b = 0;
      c = end1Pt().coordX();
    }

    // If horizontal...a = 0
    else if ( std::abs(dy) < 1.0e-6f || Slope(dx/dy) > BIG_SLOPE ) {
      a = 0;
      b = 1;
      c = end1Pt().coordY();
    }

    // If not horizontal or vertical...a = 1.0
    else {
      a = 1;
      b = -dx / dy;
      c = end1Pt().coordX() + b*end1Pt().coordY();
    }

    return(abc);
  }


  // *** Functions to set values dealing with orientation. *** //

  void LineData::update_derived_properties() {
    static const Point origin_pt = Point(0,0);
    rho_norm = perpendicularDistanceFrom(origin_pt);
    const vector<float> abc = lineEquation_abc();
    const float& a1 = abc[0];
    const float& b1 = abc[1];
    const float& c1 = abc[2];
    const float c1sign = (c1 >= 0) ? 1 : -1;
    theta_norm = atan2(b1*c1sign, a1*c1sign);
    orientation = theta_norm + AngPi((orientation_t)M_PI/2);
    length = end1Pt().distanceFrom(end2Pt());
    const ReferenceFrameType_t ref = getRefFrameType();
    end1_pt.setRefFrameType(ref);
    end2_pt.setRefFrameType(ref);
    deleteRendering();
  }

  bool LineData::isNotVertical() const {
    const AngPi threshold = (orientation_t)M_PI / 3;
    const AngPi orient = getOrientation();
    return (orient <= threshold) || (orient >= (orientation_t)M_PI - threshold);
  }

  /*
    bool LineData::isRoughlyPerpendicularTo(Shape<LineData>& other) {
    AngPi threshold = M_PI_4;
    AngPi orientation_diff = getOrientation() - other->getOrientation();
    if((orientation_diff >= threshold) && (orientation_diff < (M_PI-threshold)))
    return true;
    else
    return false;
    }

    bool LineData::isExactlyPerpendicularTo(Shape<LineData>& other) {
    AngPi orientation_diff = getOrientation() - other->getOrientation();
    return (orientation_diff == M_PI_2);
    }

  */

  //================================================================

  /* There is a standard formula to determine whether a point is to the
     left of a ray.  However, we're using lines, not rays (so there is no
     direction), and we mean "left of" from the viewer's point of view, not
     the ray's.  So this code is a little more complex.  We must decide on
     canonical start and end points for the line (ray) based on the
     comparison we're trying to make.  (To test this, draw two lines
     forming an "X" and make sure the code works correctly for both lines.)
     Also, we want this to work in both camcentric and ego/allocentric
     reference frames, which adds further complexity. -- DST */

  bool LineData::pointIsLeftOf(const Point& pt) const {
    const EndPoint &p1 =
      ( getRefFrameType() == camcentric )
      ? ((end1_pt.coordY() < end2_pt.coordY()) ? end1_pt : end2_pt)
      : ((end1_pt.coordX() < end2_pt.coordX()) ? end1_pt : end2_pt);
    const EndPoint &p2 = (&p1 == &end1_pt) ? end2_pt : end1_pt;
    if ( (getRefFrameType()==camcentric) ? (p1.coordY() == p2.coordY()) : (p1.coordX() == p2.coordX()) )
      return false;
    else
      return ( (p2.coordX() - p1.coordX()) * (pt.coordY() - p1.coordY()) -
               (p2.coordY() - p1.coordY()) * (pt.coordX() - p1.coordX()) ) > 0;
  }

  bool LineData::pointIsRightOf(const Point& pt) const {
    const EndPoint &p1 =
      ( getRefFrameType() == camcentric )
      ? ((end1_pt.coordY() < end2_pt.coordY()) ? end1_pt : end2_pt)
      : ((end1_pt.coordX() < end2_pt.coordX()) ? end1_pt : end2_pt);
    const EndPoint &p2 = (&p1 == &end1_pt) ? end2_pt : end1_pt;
    if ( (getRefFrameType()==camcentric) ? (p1.coordY() == p2.coordY()) : (p1.coordX() == p2.coordX()) )
      return false;
    else
      return ( (p2.coordX() - p1.coordX()) * (pt.coordY() - p1.coordY()) -
               (p2.coordY() - p1.coordY()) * (pt.coordX() - p1.coordX()) ) < 0;
  }

  bool LineData::pointIsAbove(const Point& pt) const {
    const EndPoint &p1 =
      ( getRefFrameType() == camcentric )
      ? ((end1_pt.coordX() < end2_pt.coordX()) ? end1_pt : end2_pt)
      : ((end1_pt.coordY() < end2_pt.coordY()) ? end1_pt : end2_pt);
    const EndPoint &p2 = (&p1 == &end1_pt) ? end2_pt : end1_pt;
    if ( (getRefFrameType()==camcentric) ? (p1.coordX() == p2.coordX()) : (p1.coordY() == p2.coordY()) )
      return false;
    else
      return ( (p2.coordX() - p1.coordX()) * (pt.coordY() - p1.coordY()) -
               (p2.coordY() - p1.coordY()) * (pt.coordX() - p1.coordX()) ) > 0;
  }

  bool LineData::pointIsBelow(const Point& pt) const {
    const EndPoint &p1 =
      ( getRefFrameType() == camcentric )
      ? ((end1_pt.coordX() < end2_pt.coordX()) ? end1_pt : end2_pt)
      : ((end1_pt.coordY() < end2_pt.coordY()) ? end1_pt : end2_pt);
    const EndPoint &p2 = (&p1 == &end1_pt) ? end2_pt : end1_pt;
    if ( (getRefFrameType()==camcentric) ? (p1.coordX() == p2.coordX()) : (p1.coordY() == p2.coordY()) )
      return false;
    else
      return ( (p2.coordX() - p1.coordX()) * (pt.coordY() - p1.coordY()) -
               (p2.coordY() - p1.coordY()) * (pt.coordX() - p1.coordX()) ) < 0;
  }

  // *** These functions are true based on line length. *** //

  bool LineData::isLongerThan(const Shape<LineData>& other) const {
    return length > other->length; }

  bool LineData::isLongerThan(float ref_length) const {
    return length > ref_length; }

  bool LineData::isShorterThan(const Shape<LineData>& other) const {
    return length < other->length; }

  bool LineData::isShorterThan(float ref_length) const {
    return length < ref_length; }

  bool LineData::isBetween(const Point &p, const LineData &other) const {
    if (getOrientation() == other.getOrientation()) { // parallel lines
      float dl = perpendicularDistanceFrom(other.end1Pt()); // distance between the lines
      return (perpendicularDistanceFrom(p) <= dl && other.perpendicularDistanceFrom(p) <= dl);
    }
    else {
      bool b;
      const LineData p_line (*space,  p,  // line from intersection of this and other to p
                             intersectionWithLine(other, b, b));
      const AngPi theta_pline = p_line.getOrientation();
      const AngPi theta_greater =
        (getOrientation() > other.getOrientation()) ? getOrientation() : other.getOrientation();
      const AngPi theta_smaller =
        (getOrientation() < other.getOrientation()) ? getOrientation() : other.getOrientation();
      if (theta_greater - theta_smaller > M_PI/2)
        return (theta_pline >= theta_greater || theta_pline <= theta_smaller);
      else
        return (theta_pline <= theta_greater && theta_pline >= theta_smaller);
    }
  }


  bool LineData::isOverlappedWith(const LineData& otherline, int amount) const {
    fmat::Column<3> thisVec = end2_pt.coords - end1_pt.coords;
    fmat::Column<2> v = fmat::SubMatrix<2,1>(thisVec);
    float theta = atan2(v[1],v[0]);
    fmat::Matrix<2,2> rot = fmat::rotation2D(-theta);
    v = rot * v;
    fmat::Column<3> otherPt1 = otherline.end1_pt.coords - end1_pt.coords;
    fmat::Column<3> otherPt2 = otherline.end2_pt.coords - end1_pt.coords;
    fmat::Column<2> op1 = rot * fmat::SubMatrix<2,1>(otherPt1);
    fmat::Column<2> op2 = rot * fmat::SubMatrix<2,1>(otherPt2);
    if ( v[0] <= 0 ) cout << "*** *** LineData::perpendicularDistanceFrom v[0] = " << v[0] << endl;
    float thisMin = min(0.f, v[0]);
    float thisMax = max(0.f, v[0]);
    float otherMin = min(op1[0], op2[0]);
    float otherMax = max(op1[0], op2[0]);
    return (thisMin < otherMin) ? (otherMin+amount <= thisMax) : (thisMin+amount <= otherMax);
  }

  bool LineData::pointFallsWithin(const Point& pt, float tolerance) const {
    fmat::Column<3> thisVec = end2_pt.coords - end1_pt.coords;
    fmat::Column<2> v = fmat::SubMatrix<2,1>(thisVec);
    float theta = atan2(v[1],v[0]);
    fmat::Matrix<2,2> rot = fmat::rotation2D(-theta);
    v = rot * v;
    fmat::Column<3> pvec = pt.coords - end1_pt.coords;
    fmat::Column<2> prot = rot * fmat::SubMatrix<2,1>(pvec);
    return ( prot[0]+tolerance >= 0.0 && prot[0]-tolerance <= v[0] );
  }

  // ***  Check intersection. *** //

  bool
  LineData::intersectsLine(const Shape<LineData>& other) const {
    return intersectsLine(other.getData());
  }

  bool
  LineData::intersectsLine(const LineData& other) const {
    // Calculate F(x,y) = 0 for this line (x1,y1) to (x2,y2).
    pair<float,float> F = lineEquation_mb();

    // Calculate G(x,y) = 0 for L (x3,y3) to (x4,y4).
    pair<float,float> G = other.lineEquation_mb();

    // NOTE: These tests are assumed to be taking place in the space of
    // "this" line.  Therefore, the limits of line extent (for lines
    // with inactive endpoints) are calculated in the space of "this"
    // line.

    // JJW *** YOU NEED TO TAKE ACCOUNT OF END POINTS BEING TURNED OFF.

    // 	float end1x, end1y, end2x, end2y, other_end1x, other_end1y, other_end2x, other_end2y;
    // 	if(end1Pt().isActive()) {
    // 		end1x = end1Pt().coordX();
    // 		end1y = end1Pt().coordY();
    // 	} else {
    // 		end1x =


    // TEST 1

    // Calculate r3 = F(x3,y3)
    float r3 = F.first * other.end1Pt().coordX() + F.second - other.end1Pt().coordY();

    // Calculate r4 = F(x4,y4)
    float r4 = F.first * other.end2Pt().coordX() + F.second - other.end2Pt().coordY();

    // If r3 != 0...
    // ...AND r4 != 0...
    // ...AND signbit(r3) == signbit(r4)...
    // ...THEN the lines do not intersect.

    if((r3 != 0)
       && (r4 != 0)
       && (signbit(r3) == signbit(r4))
       )
      return false;


    // TEST 2

    // Calculate r1 = G(x1,y1)
    float r1 = G.first * end1Pt().coordX() + G.second - end1Pt().coordY();

    // Calculate r2 = G(x2,y2)
    float r2 = G.first * end2Pt().coordX() + G.second - end2Pt().coordY();

    // If r1 != 0...
    // ...AND r2 != 0...
    // ...AND signbit(r1) == signbit(r2)...
    // ...THEN the lines do not intersect.

    if((r1 != 0)
       && (r2 != 0)
       && (signbit(r1) == signbit(r2))
       )
      return false;

    // Otherwise, the lines DO intersect.
    return true;
  }


  Point LineData::intersectionWithLine(const Shape<LineData>& other,
                                       bool& intersection_on_this,
                                       bool& intersection_on_other) const {
    return intersectionWithLine(other.getData(), intersection_on_this,intersection_on_other);
  }

  Point
  LineData::intersectionWithLine(const LineData& other,
                                 bool& intersection_on_this, bool& intersection_on_other) const {

    // Based upon algorithm written by Paul Bourke, April 1989.
    // http://astronomy.swin.edu/~pbourke/geometry/lineline2d/
    // Accessed July 20th 2004

    // Points 1 and 2 are on "this" line. Points 3 and 4 define the "other" line.
    float x1, x2, x3, x4, y1, y2, y3, y4;
    x1 = end1Pt().coordX();
    x2 = end2Pt().coordX();
    x3 = other.end1Pt().coordX();
    x4 = other.end2Pt().coordX();
    y1 = end1Pt().coordY();
    y2 = end2Pt().coordY();
    y3 = other.end1Pt().coordY();
    y4 = other.end2Pt().coordY();

    // x1 + u_a(x2-x1) = x3 + u_b(x4-x3)
    // y1 + u_a(y2-y1) = y3 + u_b(y4-y3)

    // u_a = (x4-x3)(y1-y3) - (y4-y3)(x1-x3)
    //       -------------------------------
    //       (y4-y3)(x2-x1) - (x4-x3)(y2-y1)

    // u_b = (x2-x1)(y1-y3) - (y2-y1)(x1-x3)
    //       -------------------------------
    //       (y4-y3)(x2-x1) - (x4-x3)(y2-y1)

    float denom = ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1));
    float u_a_numerator = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3));
    float u_b_numerator = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3));

    // If denominators of u_a and u_b are zero, then lines are parallel.
    if (denom == 0.0) {
      if (u_a_numerator == 0.0 && u_b_numerator == 0.0) {
        // cerr << "intersectionWithLine: lines are coincident!\n";
        // printParams();
        // other.printParams();
        return(end1Pt());
      }
      else {
        // cout << x1 << " " << x2 << " " << x3 << " " << x4 << " "
        //      << y1 << " " << y2 << " " << y3 << " " << y4 << endl;
        // cout << "this theta; " << getOrientation() << ", other theta: " << other.getOrientation() << endl;
        // cerr << "ERROR in intersectionWithLine: lines are parallel!\n";
        return(Point(-9999.0f,-99999.0f));
      }
    }

    else {
      float u_a = u_a_numerator / denom;
      float u_b = u_b_numerator / denom;

      // If 0 <= u_a <=1  then intersection point is on segment a.
      if(0<=u_a && u_a<=1)
        intersection_on_this = true;
      else
        intersection_on_this = false;

      // If 0 <= u_b <=1  then intersection point is on segment b.
      if(0<=u_b && u_b<=1)
        intersection_on_other = true;
      else
        intersection_on_other = false;

      return(Point((x1+u_a*(x2-x1)),
                   (y1+u_a*(y2-y1))));
    }
  }


  // *** Distance. *** //

  float LineData::perpendicularDistanceFrom(const Point& otherPt) const {
    // NOTE that this formula is rather slow, as it involves the
    // calculation of the line equation in addition to a square root here.

    // Using the formula from http://www.tpub.com/math2/8.htm
    vector<float> abc = lineEquation_abc();
    const float& a = abc[0];
    const float& b = abc[1];
    const float& c = abc[2];

    // Distance...
    //    (x*A + y*B - C  )
    // abs(-------------  )
    //    (sqrt(a^2 + b^2))
    const float d = fabs((a * otherPt.coordX() + b * otherPt.coordY() - c)/sqrt(a*a + b*b));
    //  cout << "abcd: " << a << ", " << b << ", " << c << ", " << d << endl;
    return(d);
  }

 float LineData::closestApproachTo(const LineData& other) const {
    if ( intersectsLine(other) )
      return 0.0;
    else {
      float d1 = min(end1Pt().distanceFrom(other.end1Pt()),
                     end1Pt().distanceFrom(other.end2Pt()));
      float d2 = min(end2Pt().distanceFrom(other.end1Pt()),
                     end2Pt().distanceFrom(other.end2Pt()));
      float minDist = min(d1,d2);
      if ( pointFallsWithin(other.end1Pt()) )
        minDist = min(minDist, perpendicularDistanceFrom(other.end1Pt()));
      if ( pointFallsWithin(other.end2Pt()) )
        minDist = min(minDist, perpendicularDistanceFrom(other.end2Pt()));
      if ( other.pointFallsWithin(end1Pt()) )
        minDist = min(minDist, other.perpendicularDistanceFrom(end1Pt()));
      if ( other.pointFallsWithin(end2Pt()) )
        minDist = min(minDist, other.perpendicularDistanceFrom(end2Pt()));
      return minDist;
    }
  }

  // ==================================================
  // BEGIN LINE EXTRACTION CODE
  // ==================================================

#define BIG_SLOPE_CS 5000.0

static int const EXTRACT_LINES_MIN_SCORE = 10;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wlarger-than="
  LineData::HoughTally::HoughTally(unsigned short table[TSIZE][RSIZE]) : head(0), free(0) {
  memset(headers, 0, NUMCHAINS*sizeof(unsigned int));
  buildChains(table);
  joinChains();
}
#pragma GCC diagnostic pop

void LineData::HoughTally::buildChains(unsigned short table[TSIZE][RSIZE]) {
  for ( unsigned short t = 0; t<TSIZE; t++ )
    for ( unsigned short r = 0; r<RSIZE; r++ ) {
      unsigned short score = table[t][r];
      if ( score >= EXTRACT_LINES_MIN_SCORE ) {
        ++free;
        if ( headers[score] == 0 ) // this entry will be the first in its chain
          bottoms[score] = free;
        bins[free] = HoughBin(t, r, headers[score]);
        headers[score] = free;
      }
    }
}

void LineData::HoughTally::joinChains() {
  // Link the chains together
  unsigned int prevbottom = 0;
  for ( unsigned short i = NUMCHAINS-1; i > 0; i-- ) {
    if ( headers[i] != 0 ) {
      if ( head == 0 )
        head = headers[i];
      if ( prevbottom != 0 )
        bins[prevbottom].next = headers[i];
      prevbottom = bottoms[i];
    }
  }
}

  vector<Shape<LineData> > LineData::houghExtractLines(Sketch<bool> const& sketch,
                                                       Sketch<bool> const& occluders,
                                                       const int num_lines) {
    unsigned int const width = sketch->getWidth(), height = sketch->getHeight();
    SketchSpace &SkS = sketch->getSpace();
    ShapeSpace &ShS = SkS.getDualSpace();
    NEW_SKETCH_N(skelDist,uint,visops::mdist(sketch));
    // Clean out pixels on image edges
    NEW_SKETCH_N(edges, bool, visops::non_bounds(sketch, 2));

    // Populate the Hough table: rho can be negative, which allows
    // theta to range from 0 to pi instead of 0 to 2pi.  But in the
    // actual LineData structure, theta_norm ranges from 0 to 2pi.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wlarger-than="
    unsigned short hough[TSIZE][RSIZE];    // bin counts for Hough table
#pragma GCC diagnostic pop
    memset(hough, 0, TSIZE*RSIZE*sizeof(unsigned short));
    for (unsigned int x = 0; x < width; x++) {
      for (unsigned int y = 0; y < height; y++) {
        if ( edges(x,y) ) {
          for ( int t = 0; t < TSIZE; t++ ) {
            double theta = M_PI/TSIZE * t;
            int r = int(x*cos(theta) + y*sin(theta) + RSIZE/2);
            hough[t][r]++;
          }
        }
      }
    }
     
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wlarger-than="
    HoughTally tally(hough);
#pragma GCC diagnostic pop
    unsigned int nextBin = tally.head;
    //std::cout << "Tally head=" << tally.head << " free=" << tally.free << std::endl;

    vector<Shape<LineData> > lines_vec;
    while ( nextBin != 0 && (int)lines_vec.size() < num_lines ) {
      int maxT = tally.bins[nextBin].theta;
      int maxR = tally.bins[nextBin].rho;
      //std::cout << ". " << nextBin << " t=" << maxT << " r=" << maxR << std::endl;
      nextBin = tally.bins[nextBin].next;
      //std::cout << ". Score: " << maxScore << ", T: " << maxT << ", R: " << maxR << std::endl;

      // Make some line segments
      float x_normpoint = (maxR - RSIZE/2)*cos(float(maxT)/TSIZE * M_PI);
      float y_normpoint = (maxR - RSIZE/2)*sin(float(maxT)/TSIZE * M_PI);
      // m is slope of this line (not the normal line)
      float m = std::max(std::min(std::tan((maxT * M_PI/TSIZE)+AngPi((orientation_t)M_PI/2)),
                                  BIG_SLOPE_CS),
                         -BIG_SLOPE_CS);
      float b = y_normpoint - m*x_normpoint;
      Point pt(0, b);
      AngPi orientation = atan(m);
      int count = 0;
      for ( int startval = 0;; ) {
        if ( ++count > 6 ) break;  // safety check: can't have more than 6 colinear lines in an image
        LineData extracted_line(ShS, pt, orientation);
        extracted_line.setColor(sketch->getColor()); // needed for IsMatchFor to work
        extracted_line.setParentId(sketch->getViewableId());
        if ( std::abs(m) <= 1 )  // when |slope| <= 1, scan along x, else scan along y
          startval = extracted_line.scanHorizForEndPts(skelDist,occluders,m,b,startval);
        else
          startval = extracted_line.scanVertForEndPts(skelDist,occluders,m,b,startval);
        if ( startval < 0 ) {
          break;
        }
        extracted_line.firstPt().checkValidity(width,height,beg_dist_thresh);
        extracted_line.secondPt().checkValidity(width,height,beg_dist_thresh);
        // Transform from SketchSpace coordinates to ShapeSpace coordinates
        SkS.applyTmatinv(extracted_line.firstPt().getCoords());
        SkS.applyTmatinv(extracted_line.secondPt().getCoords());
        extracted_line.update_derived_properties();
        if ( extracted_line.getLength() < extractorMinLineLength ) {
          //cout << "  Should delete " << extracted_line << " length=" << extracted_line.getLength()
          //     << " score=" << maxScore << " startval=" << startval << " m=" << m << " b=" << b << std::endl;
          continue;
        }
        bool matched = false;
        bool substituted = false;
        for ( vector<Shape<LineData> >::iterator ln_it = lines_vec.begin();
              ln_it != lines_vec.end(); ) { // don't bump iterator here; do it at bottom
          Shape<LineData> &ln = *ln_it;
          if ( extracted_line.isParasiteOf(ln.getData(), occluders) ) {
            matched = true;
            break;
          }
          if ( extracted_line.isMatchFor(ln) ) {
            matched = true;
            if ( extracted_line.getLength() > ln->getLength() ) {
              //std::cout << "Extracted line " << extracted_line->getId()
              //          << " " << extracted_line->firstPt()
              //          << "," << extracted_line->secondPt() << " len " << extracted_line->getLength()
              //          << " replacing " << ln->getId()
              //          << " " << ln->firstPt() << "," << ln->secondPt()
              //          << " len " << ln->getLength() << std::endl;
              ln_it->deleteShape();
              if ( ! substituted ) {
                *ln_it = Shape<LineData>(new LineData(extracted_line));
                substituted = true;
              } else { // we've already replaced one line; from now on just erase them
               ln_it = lines_vec.erase(ln_it);
               continue;
              }
            }
            else {
              //std::cout << "Extracted line " << extracted_line.getId()
              //          << " " << extracted_line.firstPt()
              //          << "," << extracted_line.secondPt() << " len " << extracted_line.getLength()
              //          << " inferior to " << ln->getId()
              //          << " " << ln->firstPt() << "," << ln->secondPt()
              //          << " len " << ln->getLength() << std::endl;
              break;
            }
          }
          ln_it++;  // safe to bump here
        }
        if ( ! matched ) {
          lines_vec.push_back(Shape<LineData>(new LineData(extracted_line)));
          //std::cout << "Extracted new line " << extracted_line.getId()
          //          << " " << extracted_line.firstPt()
          //          << "," << extracted_line.secondPt() << " len " << extracted_line.getLength()
          //          << std::endl;
        }
      }
    } // end of loop to extract lines

    // adjust endpoints to center lines over their supporting pixels
    std::vector<Shape<LineData>> result;
    //Shape<LineData> debugLineSave;
    for (std::vector<Shape<LineData>>::iterator it = lines_vec.begin(); it != lines_vec.end(); it++) {
      float slope = std::max(std::min(double(std::tan((*it)->getOrientation())), BIG_SLOPE_CS), -BIG_SLOPE_CS);
      if (std::abs(slope) <= 1) {
        EndPoint &leftpt = (*it)->leftPt();
        EndPoint &rightpt = (*it)->rightPt();
        int xleft = int(leftpt.coordX());
        int yleft = int(leftpt.coordY());
        int xright = int(rightpt.coordX());
        int yright = int(rightpt.coordY());
        int min_diff = calculateDiff(occluders, xleft, yleft, xright, yright);
        int lefty1, lefty2, diff1, diff2, righty1, righty2;
        for (int i = 0; i < 5; i++) {
          lefty1 = adjustYleft(occluders, xleft, yleft, xright, yright, min_diff, +1);
          lefty2 = adjustYleft(occluders, xleft, yleft, xright, yright, min_diff, -1);
          diff1 = calculateDiff(occluders, xleft, lefty1, xright, yright);
          diff2 = calculateDiff(occluders, xleft, lefty2, xright, yright);
          min_diff = std::min(diff1, diff2);
          yleft = diff1 < diff2 ? lefty1 : lefty2;
          righty1 = adjustYright(occluders, xleft, yleft, xright, yright, min_diff, +1);
          righty2 = adjustYright(occluders, xleft, yleft, xright, yright, min_diff, -1);
          diff1 = calculateDiff(occluders, xleft, yleft, xright, righty1);
          diff2 = calculateDiff(occluders, xleft, yleft, xright, righty2);
          yright = diff1 < diff2 ? righty1 : righty2;
          min_diff = std::min(diff1, diff2);
        }
        leftpt.setCoords(xleft,yleft);
        rightpt.setCoords(xright,yright);
      }	else {
        EndPoint &toppt = (*it)->topPt();
        EndPoint &bottompt = (*it)->bottomPt();
        int xtop = int(toppt.coordX());
         int ytop = int(toppt.coordY());
        int xbottom = int(bottompt.coordX());
        int ybottom = int(bottompt.coordY());
        int min_diff = calculateDiff(occluders, xtop, ytop, xbottom, ybottom);
        int topx1, topx2, diff1, diff2, bottomx1, bottomx2;
        for (int i = 0; i < 5; i++) {
          topx1 = adjustXtop(occluders, xtop, ytop, xbottom, ybottom, min_diff, +1);
          topx2 = adjustXtop(occluders, xtop, ytop, xbottom, ybottom, min_diff, -1);
          diff1 = calculateDiff(occluders, topx1, ytop, xbottom, ybottom);
          diff2 = calculateDiff(occluders, topx2, ytop, xbottom, ybottom);
          min_diff = std::min(diff1, diff2);
          xtop = diff1 < diff2 ? topx1 : topx2;
          bottomx1 = adjustXbot(occluders, xtop, ytop, xbottom, ybottom, min_diff, +1);
          bottomx2 = adjustXbot(occluders, xtop, ytop, xbottom, ybottom, min_diff, -1);
          diff1 = calculateDiff(occluders, xtop, ytop, bottomx1, ybottom);
          diff2 = calculateDiff(occluders, xtop, ytop, bottomx2, ybottom);
          xbottom = diff1 < diff2 ? bottomx1 : bottomx2;
          min_diff = std::min(diff1, diff2);
        }
        toppt.setCoords(xtop,ytop);
        bottompt.setCoords(xbottom,ybottom);
      }
      (*it)->update_derived_properties();
      //if ( (*it)->getId() == 10044 ) debugLineSave = *it;
      //if ( (*it)->getId() == 10045 ) (*it)->isParasiteOf(debugLineSave.getData(),occluders);
      // cout << "extracted line length = " << (*it)->getLength() << " min=" << extractorMinLineLength << endl;
      if ( (*it)->getLength() >= extractorMinLineLength )
        result.push_back(*it);
      else
        it->deleteShape();
    }
    return result;
  }

  int LineData::adjustXtop(Sketch<bool> const& sketch, int xtop, int ytop, int xbottom, int ybottom,
                           int diff, int incr) {
    int bestdiff = diff;
    int ixtop = xtop + incr;
    while (ixtop >= 0 && ixtop < int(sketch->getWidth())) {
      int newdiff = calculateDiff(sketch, ixtop, ytop, xbottom, ybottom);
      if (newdiff >= bestdiff)
        break;
      bestdiff = newdiff;
      ixtop += incr;
    }
    return ixtop - incr;
  }

  int LineData::adjustXbot(Sketch<bool> const& sketch, int xtop, int ytop, int xbottom, int ybottom,
                           int diff, int incr) {
    int bestdiff = diff;
    int ixbottom = xbottom + incr;
    while (ixbottom >= 0 && ixbottom < int(sketch->getWidth())) {
      int newdiff = calculateDiff(sketch, xtop, ytop, ixbottom, ybottom);
      if (newdiff >= bestdiff)
        break;
      bestdiff = newdiff;
      ixbottom += incr;
    }
    return ixbottom - incr;
  }

  int LineData::adjustYleft(Sketch<bool> const& sketch, int xleft, int yleft, int xright, int yright,
                            int diff, int incr) {
    int bestdiff = diff;
    int iyleft = yleft;
    while (iyleft > 0 && iyleft < int(sketch->getHeight())) {
      int newdiff = calculateDiff(sketch, xleft, iyleft, xright, yright);
      if (newdiff >= bestdiff)
        break;
      bestdiff = newdiff;
      iyleft += incr;
    }
    return iyleft - incr;
  }

  int LineData::adjustYright(Sketch<bool> const& sketch, int xleft, int yleft, int xright, int yright,
                             int diff, int incr) {
    int bestdiff = diff;
    int iyright = yright;
    while (iyright > 0 && iyright < int(sketch->getHeight())) {
      int newdiff = calculateDiff(sketch, xleft, yleft, xright, iyright);
      if (newdiff >= bestdiff)
        break;
      bestdiff = newdiff;
      iyright += incr;
    }
    return iyright - incr;
  }

  int LineData::calculateDiff(Sketch<bool> const& sketch, int xleft, int yleft, int xright, int yright) {
    double sl;
    if (xleft != xright)
      sl = std::max(std::min(double((yleft - yright))/(xleft - xright), BIG_SLOPE_CS), -BIG_SLOPE_CS);
    else
      sl = BIG_SLOPE_CS;
    double b = yleft - sl * xleft;
    int diff_sum = 0;
    if (std::abs(sl)<1) {  // when |slope| <= 1, scan along x, else scan along y
      for (int ix = xleft + 5; ix < xright - 5; ix++) {
        int iy1 = sl * ix + b;
        int iy2 = sl * ix + b;
        int iy = ix * sl + b;
        if (!sketch(ix, iy)) {
          diff_sum += 100;
          continue;
        }
        while (sketch(ix, iy1) && ix >= 0 && ix < sketch.width && iy1 >= 0 && iy1 < sketch.height)
          iy1++;
        while (sketch(ix, iy2) && ix >= 0 && ix < sketch.width && iy2 >= 0 && iy2 < sketch.height)
          iy2--;
        diff_sum += std::abs(std::abs(iy1 - iy) - std::abs(iy - iy2));
      }
      return diff_sum;
    }
    else {
      for (int iy = std::min(yleft, yright) + 5; iy < std::max(yleft, yright) - 5; iy++) {
        int ix1 = (iy - b)/sl + 1;
        int ix2 = (iy - b)/sl - 1;
        int ix = (iy - b)/sl;
        //cout << "check x " << ix << " and y " << iy << endl;
        if (!sketch(ix, iy)) {
          diff_sum += 100;
          continue;
        }
        while (sketch(ix1, iy) && ix1 >= 0 && ix1 < sketch.width && iy >= 0 && iy < sketch.height)
          ix1++;
        while (sketch(ix2, iy) && ix2 >= 0 && ix2 < sketch.width && iy >= 0 && iy < sketch.height)
          ix2--;
        diff_sum += std::abs(std::abs(ix1 - ix) - std::abs(ix - ix2));
      }
      return diff_sum;
    }
  }

  Shape<LineData> LineData::extractLine(Sketch<bool>& sketch) {
    //copy sketch as occluders
    NEW_SKETCH_N(occluders, bool, visops::copy(sketch));
    return houghExtractLines(sketch, occluders, 1)[0];

    //return houghExtractLines(sketch, sketch, 1)[0];
  }

  Shape<LineData> LineData::extractLine(Sketch<bool>& sketch, const Sketch<bool>& occlusions) {
    //copy sketch as occluders
    //NEW_SKETCH_N(occluders, bool, visops::copy(sketch));
    //return houghExtractLines(sketch, occluders, 1)[0];

    return houghExtractLines(sketch, occlusions, 1)[0];
  }

  vector<Shape<LineData> > LineData::extractLines(Sketch<bool> const& sketch, const int num_lines) {
    return houghExtractLines(sketch, sketch, num_lines);
  }

  vector<Shape<LineData> > LineData::extractLines(Sketch<bool> const& sketch, 
                                                  Sketch<bool> const& occluders,
                                                  const int num_lines) {
    return houghExtractLines(sketch, occluders, num_lines);
  }

  // Hack to split a non-straight region, by Kei
  Shape<LineData> LineData::splitLine(ShapeSpace &ShS, Region &skelchunk,
                                      Sketch<bool> &skeleton, const Sketch<bool> &occlusions) {
    //cout << "this region is not straight, needs be split into two regions" << endl;
    Point bounds[4] = {skelchunk.findTopBoundPoint(), skelchunk.findLeftBoundPoint(),
                       skelchunk.findRightBoundPoint(), skelchunk.findBotBoundPoint()};
    //  cout << "top(0): " << bounds[0] << endl;
    //  cout << "left(1): " << bounds[1] << endl;
    //  cout << "right(2): " << bounds[2] << endl;
    //  cout << "bottom(3): " << bounds[3] << endl;
    for (int i = 0; i < 4; i++) {
      for (int j = i+1; j < 4; j++) {
        if (bounds[i].distanceFrom(bounds[j]) > 20 && ! skelchunk.isContained((bounds[i]+bounds[j])/2.f, 3)) {
          //	cout << "[" << i << "," << j << "] form a line from which most distant point is where the region should split " << endl;
          LineData ln(ShS,bounds[i],bounds[j]);
          Point most_distant(skelchunk.mostDistantPtFrom(ln));
          ShS.getDualSpace().applyTmat(most_distant.getCoords());
          int const clear_dist = 15;  // was 10
          int x1 = max(0, int(most_distant.coordX() - clear_dist));
          int x2 = min(skeleton.width-1, int(most_distant.coordX() + clear_dist));
          int y1 = max(0, int(most_distant.coordY() - clear_dist));
          int y2 = min(skeleton.height-1, int(most_distant.coordY() + clear_dist));
          for (int x = x1; x <= x2; x++)
            for (int y = y1; y <= y2; y++)
              skeleton(x,y) = false;
          return extractLine(skeleton, occlusions);
        }
      }
    }
    ShapeRoot invalid; // need to define a named variable to avoid warning on next line
    return ShapeRootType(invalid,LineData);
  }

  int LineData::scanHorizForEndPts(const Sketch<uint>& skelDist,
                                   const Sketch<bool>& occlusions,
                                   float m, float b, int xstart) {
    // Scan along the infinite line, looking for a segment in the image.
    // adjust the line's endpoints based on what we find.
    // Return stopping point so we can be called again for additional segments.
    bool on_line = false;  // true if tracing a line skeleton
    int curxstart = -1;    // start of current segment we're examining
    int possxstop = -1;    // end of current segment unless we bridge a gap
    int bestlength = -1;   // length of best segment seen so far
    int goodpixels;        // number of pixels that aren't gaps
    int const xstop = skelDist.width - line_min_length;
    // cout << "horiz:  xstart=" << xstart << " m=" << m << " b=" << b << " ";
    while ( xstart < xstop ) {
      possxstop = -1;
      for (int x = xstart, y, dist=0; x < skelDist.width; x++) {
        y = int(m*x+b);
        if (y < 0 || y >= skelDist.height) continue;  // make sure y is on-screen
        dist = skelDist(x,y);
        if (on_line == false) {   // not currently on a line segment
          if (dist <= beg_dist_thresh) { // start a new segment
            on_line = true;
            goodpixels = 0;
            curxstart = x - dist;
            // if new segment begins at an occluder, back up over the occluder
            int curystart;
            bool backupflag = false;
            while ( curxstart >= 0 && (curystart=int(m*curxstart+b)) >= 0 && curystart < skelDist.height ) {
              if ( occlusions(curxstart,curystart) || skelDist(curxstart,curystart) == 0 ) {
                --curxstart;
                backupflag = true;
              } else { // if we backed up past the occluder or segment, go one step forward
                curxstart += backupflag;
                break;
              }
            }
            if ( curxstart < 0) // occluder extended to left edge of image
              curxstart = 0;
          }
        }
        else {   // on_line == true:  currently on a line segment
          if ( dist <= end_dist_thresh || occlusions(x,y) ) {
            ++goodpixels;
            possxstop = x;
          }
          else if ( dist > extractorGapTolerance ) {
            // we're traversing a gap, and it just got too long
            on_line = false;
            bestlength = possxstop - curxstart;
            if ( bestlength >= line_min_length )
              break; // proceed if we've found a good segment
          }
        }
      }
      bestlength = possxstop - curxstart;
      if ( bestlength <= 0 ) {  // could not find a candidate line; we're done
        return -1;
      }
      if ( bestlength >= line_min_length && float(goodpixels)/bestlength >= 0.8 ) {  // found a good line; return it
        float y1 = m*curxstart + b;
        float y2 = m*possxstop + b;
        setEndPts(Point(curxstart,y1), Point(possxstop,y2));
        return possxstop + end_dist_thresh;
      }
      xstart = max(xstart+1,possxstop) + end_dist_thresh;
    }
    // ran off the end of the screen; we're done
    return -1;
  }

  int LineData::scanVertForEndPts(const Sketch<uint>& skelDist,
                                  const Sketch<bool>& occlusions,
                                  float m, float b, int ystart) {
    // Scan along the infinite line, looking for segments in the image.
    // Adjust the line's endpoints based on what we find.
    // Return stopping point so we can be called again for additional segments.
    bool on_line = false;  // true if currently tracing a line skeleton
    int curystart = -1;    // start of current segment we're examining
    int possystop = -1;    // end of current segment unless we bridge a gap
    int bestlength = -1;   // length of segment seen so far
    int goodpixels = 0;    // number of pixels that aren't gaps
    int const ystop = skelDist.height - line_min_length;
    while ( ystart < ystop ) {
      // scan for a candidate line
      possystop = -1;
      for (int x, y = ystart, dist=0; y < skelDist.height; y++) {
        x = int((y-b)/m);
        if (x < 0 || x >= skelDist.width) continue;  // make sure x is on-screen
        dist = skelDist(x,y);
        if (on_line == false) {   // not currently on a line segment
          if (dist <= beg_dist_thresh) { // start a new segment
            on_line = true;
            goodpixels = 0;
            curystart = y - dist;
            // if new segment begins at an occluder, back up over the occluder
            int curxstart;
            bool backupflag = false;
            while ( curystart >= 0 && (curxstart=int((curystart-b)/m)) >= 0 && curxstart < skelDist.width )
              if ( occlusions(curxstart,curystart) || skelDist(curxstart,curystart) == 0 ) {
                --curystart;
                backupflag = true;
              } else { // if we backed up past the occluder or segment, go one step forward
                curystart += backupflag;
                break;
              }
            if ( curystart < 0) // occluder extended to top edge of image
              curystart = 0;
          }
        }
        else {   // on_line == true:  currently on a line segment
          if ( dist <= end_dist_thresh || occlusions(x,y) ) {
            ++goodpixels;
            possystop = y;
          }
          else if ( dist > extractorGapTolerance ) {
            // we're traversing a gap, and it just got too long
            on_line = false;
            bestlength = possystop - curystart;
            if ( bestlength >= line_min_length )
              break; // proceed if we've found a good setment
          }
        }
      }
      bestlength = possystop - curystart;
      // cout << "  vert bestlength=" << bestlength << "  goodpixels=" << goodpixels << endl;
      if ( bestlength <= 0 )  // could not find a candidate line; we're done
        return -1;
      if ( bestlength >= line_min_length && float(goodpixels)/bestlength >= 0.8 ) {  // found a good line; return it
        float x1 = (curystart-b)/m;
        float x2 = (possystop-b)/m;
        setEndPts(Point(x1,curystart), Point(x2,possystop));
        return possystop + end_dist_thresh;
      }
      // found a candidate but it didn't pan out; look some more
      // cout << " ystart=" << ystart << " curystart=" << curystart << " possystop=" << possystop;
      ystart = max(ystart+1,possystop) + end_dist_thresh;
      // cout << " new ystart=" << ystart << endl;
    }
    // ran off the end of the screen; we're done
    return -1;
  }

  //! Clear out pixels that are on or close to this line.
  void LineData::clearLine(Sketch<bool>& sketch) {
    const Sketch<bool>& line_rendering = getRendering();
    uint const clear_dist = 5;
    Sketch<bool> not_too_close = (visops::mdist(line_rendering) >= clear_dist);
    sketch &= not_too_close;
  }



  // ==================================================
  // BEGIN LINE RENDERING CODE
  // ==================================================

  Sketch<bool>& LineData::getRendering() {
    if ( ! end1Pt().rendering_valid || ! end2Pt().rendering_valid )
      deleteRendering();
    if ( rendering_sketch == NULL )
      rendering_sketch = render();
    return *rendering_sketch;
  }

  //! Render line to SketchSpace and return a pointer to the sketch.
  //! This function does not link the Sketch<bool>* in the shape to the sketch returned.
  Sketch<bool>* LineData::render() const {
    SketchSpace &renderspace = space->getDualSpace();
    Sketch<bool>* draw_result =
      new Sketch<bool>(renderspace, "render("+getName()+")");
    //  (*draw_result)->setParentId(getViewableId());
    //  (*draw_result)->setColor(getColor());
    (*draw_result)->inheritFrom(*this);
    *draw_result = 0;
    renderOnTo(draw_result);
    return draw_result;
  }

  void LineData::renderOnTo(Sketch<bool>* draw_result) const {
    SketchSpace &renderspace = space->getDualSpace();
    int const width = renderspace.getWidth();
    int const height = renderspace.getHeight();
    float x1,y1,x2,y2;
    setDrawCoords(x1, y1, x2, y2, width, height);

    drawline2d(*draw_result, (int)x1, (int)y1, (int)x2, (int)y2);
    end1Pt().rendering_valid = true;
    end2Pt().rendering_valid = true;
  }

  // This function will be called by both LineData and PolygonData renderers
  void LineData::setDrawCoords(float& x1,float& y1, float& x2, float& y2,
                               const int width, const int height) const {
    EndPoint e1(end1Pt());
    EndPoint e2(end2Pt());
    //std::cout << "setDrawCoords: e1=" << e1 << " e2=" << e2 << std::endl;
    space->getDualSpace().applyTmat(e1.getCoords());
    space->getDualSpace().applyTmat(e2.getCoords());
    const EndPoint &left_point = e1.coordX() <= e2.coordX() ? e1 : e2;
    const EndPoint &right_point = e1.coordX() <= e2.coordX() ? e2 : e1;
    //std::cout << "setDrawCoords: left=" << left_point << " right=" << right_point << std::endl;

    // Check if horizontal
    if ( left_point.coordY() == right_point.coordY()) {
      if (!left_point.isActive())
        x1 = 0;
      else x1 = max(0.0f,min(width-1.0f,left_point.coordX()));

      if (!right_point.isActive())
        x2 = width-1;
      else x2 = max(0.0f,min(width-1.0f,right_point.coordX()));

      y1 = left_point.coordY();
      y2 = y1;
    }
    else if ( left_point.coordX() == right_point.coordX()) {  // Check if vertical...

      const EndPoint &top_point =    e1.coordY() <= e2.coordY() ? e1 : e2;
      const EndPoint &bottom_point = e1.coordY() <= e2.coordY() ? e2 : e1;

      //std::cout << "top_point=" << top_point << " bottom_point=" << bottom_point << std::endl;
      if (!top_point.isActive())
        y1 = 0;
      else y1 = max(0.0f,min(height-1.0f,top_point.coordY()));

      if (!bottom_point.isActive())
        y2 = height-1;
      else y2 = max(0.f,min(height-1.0f,bottom_point.coordY()));

      x1 = left_point.coordX();
      x2 = x1;
    }

    else {   // Neither horizontal nor vertical...
      float const m = (right_point.coordY()-left_point.coordY())/(right_point.coordX()-left_point.coordX());
      float const b = left_point.coordY() - m*left_point.coordX();

      // find image edge intersections
      int const i0x = (int)((0-b)/m);
      int const ihx = (int)(((height-1)-b)/m);
      int const i0y = (int)(m*0+b);
      int const iwy = (int)(m*(width-1)+b);

      // If left point is active, set starting x and y accordingly.
      if ( left_point.isActive() ) {
        x1 = left_point.coordX();
        y1 = left_point.coordY();
      }

      // If endpoint 1 extends past image edge...
      else {

        // intersects left edge
        if ( i0y >= 0 && i0y < height ) {
          x1 = 0;
          y1 = i0y;
        }

        // intersects top or bottom edge
        else {

          // intersects top first
          if ( i0x < ihx ) {
            x1 = i0x;
            y1 = 0;
          }

          // intersects bottom first
          else {
            x1 = ihx;
            y1 = height-1;
          }
        }
      }

      // If right point is active, set starting x and y accordingly.
      if ( right_point.isActive() ) {
        x2 = right_point.coordX();
        y2 = right_point.coordY();
      }
      else { // endpoint extends to image edge
        if ( iwy >= 0 && iwy < height ) { // intersects right edge
          x2 = width-1;
          y2 = iwy;
        }
        else { // intersects top or bottom edge
          if ( i0x > ihx ) { // intersects top last
            x2 = i0x;
            y2 = 0;
          }
          else { // intersects bottom last
            x2 = ihx;
            y2 = height-1;
          }
        }
      }
    }
  }

  void LineData::drawline2d(Sketch<bool>& canvas, int x0, int y0, int x1, int y1) {
    int width = canvas->getWidth();
    int height = canvas->getHeight();

    //std::cout << "LineData::drawline2D x0=" << x0 << " y0=" << y0 << " x1=" << x1 << " y1=" << y1 << std::endl;

    // code below from free Graphics Gems repository, graphicsgems.org
    int dx = x1-x0,  ax = abs(dx)<<1,  sx = signbit(dx) ? -1 : 1;
    int dy = y1-y0,  ay = abs(dy)<<1,  sy = signbit(dy) ? -1 : 1;
    int d, x=x0, y=y0;

    if ( ax > ay ) {		// x dominant
      d = ay - (ax>>1);
      for (;;) {
        if (x >= 0 && y >= 0 && x < width && y < height)
          canvas(x,y) = true;
        if (x==x1)
          return;
        if ( d >= 0 ) {
          y += sy;
          d -= ax;
        }
        x += sx;
        d += ay;
      }
    }
    else { // y dominant
      d = ax - (ay>>1);
      for (;;) {
        if (x >= 0 && y >= 0 && x < width && y < height)
          canvas(x,y) = true;
        if ( y == y1 )
          return;
        if ( d >= 0 ) {
          x += sx;
          d -= ay;
        }
        y += sy;
        d += ax;
      }
    }
  }
  //}

  LineData& LineData::operator=(const LineData& other) {
    if (&other == this)
      return *this;
    BaseData::operator=(other);
    end1_pt = other.end1_pt;
    end2_pt = other.end2_pt;
    rho_norm = other.rho_norm;
    theta_norm = other.theta_norm;
    orientation = other.orientation;
    length = other.length;
    return *this;
  }


  // Compute if two points are on the same side of the line
  bool LineData::pointsOnSameSide(const Point& p1, const Point& p2)
  {
    float dx = end2_pt.coordX() - end1_pt.coordX();
    float dy = end2_pt.coordY() - end1_pt.coordY();

    float p1val = (p1.coordY() - end1_pt.coordY())*dx - (p1.coordX() - end1_pt.coordX())*dy;
    float p2val = (p2.coordY() - end1_pt.coordY())*dx - (p2.coordX() - end1_pt.coordX())*dy;

    return (p1val>0) == (p2val>0);
  }

  // Checks if the distance from the line to the point is less than 1 pixel,
  // and checks that the point is within the end points of the line.
  bool LineData::pointOnLine(const Point& p)
  {
    const float BOUNDS_EXTEND = 1;
    float dx = end2_pt.coordX() - end1_pt.coordX();
    float dy = end2_pt.coordY() - end1_pt.coordY();
    float val = (p.coordY() - end1_pt.coordY())*dx - (p.coordX() - end1_pt.coordX())*dy;
    val /= length;
    bool inBounds = (p.coordX() >= (leftPt().coordX() - BOUNDS_EXTEND)) &&
      (p.coordX() <= (rightPt().coordX() + BOUNDS_EXTEND)) &&
      (p.coordY() >= (topPt().coordY() - BOUNDS_EXTEND)) &&
      (p.coordY() <= (bottomPt().coordY() + BOUNDS_EXTEND));
    return (val > -1) && (val < 1) && inBounds;
  }


  // Comparison predicates

  bool LineData::LineDataLengthLessThan::operator() (const LineData &line1, const LineData &line2) const {
    return line1.getLength() < line2.getLength();
  }

  bool LineData::LengthLessThan::operator() (const Shape<LineData> &line1, const Shape<LineData> &line2) const {
    return line1->getLength() < line2->getLength();
  }

  bool LineData::ParallelTest::operator() (const Shape<LineData> &line1, const Shape<LineData> &line2) const {
    return angdist(line1->getOrientation(),line2->getOrientation()) <= tolerance;
  }

  bool LineData::PerpendicularTest::operator() (const Shape<LineData> &line1, const Shape<LineData> &line2) const {
    return angdist(angdist(line1->getOrientation(),line2->getOrientation()), (orientation_t)M_PI/2) <= tolerance;
  }

  bool LineData::ColinearTest::operator() (const Shape<LineData> &line1, const Shape<LineData> &line2) const {
    return ParallelTest(ang_tol)(line1,line2) &&
      abs( line1->getRhoNorm() - line2->getRhoNorm() ) <= dist_tol;
  }

  bool LineData::IsHorizontal::operator() (const Shape<LineData> &line) const {
    const AngPi orient = line->getOrientation();
    return  (orient <= threshold) || (orient >= M_PI - threshold);
  }

  bool LineData::IsVertical::operator() (const Shape<LineData> &line) const {
    const AngPi orient = line->getOrientation();
    return  (orient >= threshold) && (orient <= M_PI - threshold);
  }

} // namespace
