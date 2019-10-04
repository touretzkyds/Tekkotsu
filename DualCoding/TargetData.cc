//-*-c++-*-

#include "BaseData.h"    // superclass
#include "Point.h"       // Point data member
#include "ShapeTypes.h"  // TargetDataType

#include "SketchSpace.h"
#include "Sketch.h"
#include "visops.h"

#include "ShapeSpace.h"  // required by DATASTUFF_CC
#include "ShapeRoot.h"   // required by DATASTUFF_CC

#include "TargetData.h"
#include "ShapeTarget.h"

#include "BlobData.h"
#include "ShapeBlob.h"
#include "VRmixin.h"

using namespace std;

namespace DualCoding {

TargetData::TargetData(ShapeSpace& _space, const EndPoint &_frontLeftPt, const EndPoint &_frontRightPt, const EndPoint &_backLeftPt, const EndPoint &_backRightPt, const EndPoint &_frontIntersect, const EndPoint &_backIntersect, const float _height)
                     : BaseData(_space, getStaticType()),
                       frontLine(_space, _frontLeftPt, _frontRightPt),
                       backLine(_space, _backLeftPt, _backRightPt),
                       frontValid((_frontLeftPt != Point(0, 0, 0)) && (_frontRightPt != Point(0, 0, 0))),
                       backValid((_backLeftPt != Point(0, 0, 0)) && (_backRightPt != Point(0, 0, 0))),
                       frontIntersect(_frontIntersect),
                       backIntersect(_backIntersect),
                       orientation(0),
                       length(0),
                       width(0),
                       height(_height) {
  update_derived_properties();
}

DATASTUFF_CC(TargetData);

BoundingBox2D TargetData::getBoundingBox() const {
	BoundingBox2D result(frontLine.getBoundingBox());
	result.expand(backLine.getBoundingBox());
	return result;
}

bool TargetData::isMatchFor(const ShapeRoot& other) const {
  if (!(isSameTypeAs(other) && isSameColorAs(other))) {
    return false;
  }
  else {
    const Shape<TargetData>& other_target = ShapeRootTypeConst(other,TargetData);
    return isMatchFor(other_target.getData());
  }
}

bool TargetData::isMatchFor(const TargetData& other_target) const {
  // 2 targets match if either of their lines match
  if (frontValid && !frontLine.isMatchFor(other_target.frontLine) && backValid && !backLine.isMatchFor(other_target.backLine))
    return false;
  return true;
}

void TargetData::update_derived_properties() {
  EndPoint &frontLeftPt  = getFrontLeftPt();
  EndPoint &frontRightPt = getFrontRightPt();
  EndPoint &backLeftPt   = getBackLeftPt();
  EndPoint &backRightPt  = getBackRightPt();
  
  // Determine if the front and back lines are valid
  frontValid  = true;
  backValid   = true;
  if ((frontLeftPt == Point(0, 0, 0)) || (frontRightPt == Point(0, 0, 0))) {
    frontValid = false;
  }
  if ((backLeftPt == Point(0, 0, 0)) || (backRightPt == Point(0, 0, 0))) {
    backValid = false;
  }
  if (frontValid && backValid && (getRefFrameType() != camcentric)) {
    // Lines should not be in the negative z plane
    if (frontLine.getCentroid().coordZ() < 0) {
      frontValid = false;
    }
    if (backLine.getCentroid().coordZ() < 0) {
      backValid = false;
    }
    // Remove short lines if the other line is long
    if ((frontLine.getLength() < 50.0f) && backValid && (backLine.getLength() > 50.0f)) {
      frontValid = false;
    }
    else if ((backLine.getLength() < 50.0f) && frontValid && (frontLine.getLength() > 50.0f)) {
      backValid = false;
    }
  }
  
  // Reset the front/back lines if they are invalid
  if (!frontValid) {
    frontLeftPt  = Point(0, 0, 0);
    frontRightPt = Point(0, 0, 0);
    frontLeftPt.setValid(false);
    frontRightPt.setValid(false);
    frontLine.update_derived_properties();
    
    frontIntersect = Point(0, 0, 0);
    frontIntersect.setValid(false);
  }
  if (!backValid) {
    backLeftPt  = Point(0, 0, 0);
    backRightPt = Point(0, 0, 0);
    backLeftPt.setValid(false);
    backRightPt.setValid(false);
    backLine.update_derived_properties();
    
    backIntersect = Point(0, 0, 0);
    backIntersect.setValid(false);
  }
  
  // Determine if the front/back intersection points are valid
  if (frontIntersect == Point(0, 0, 0)) {
    frontIntersect.setValid(false);
  }
  if (backIntersect == Point(0, 0, 0)) {
    backIntersect.setValid(false);
  }
  
  // Check left/right points based on intersection points
  if (frontIntersect.isValid() || backIntersect.isValid()) {
    if (frontIntersect.isValid()) {
      bool swap = false;
      
      // The right point will be closer to the intersection if both points are valid/invalid
      if (frontValid && (frontLeftPt.isValid() == frontRightPt.isValid())
       && (frontLeftPt.distanceFrom(frontIntersect) < frontRightPt.distanceFrom(frontIntersect))) {
        swap = true;
      }
      // Assumption: the right point will be the valid point otherwise
      else if (frontValid && frontLeftPt.isValid() && !frontRightPt.isValid()) {
        swap = true;
      }
      
      
      if (swap) {
        // Front left and right are reversed
        EndPoint temp = frontLeftPt;
        frontLeftPt = frontRightPt;
        frontRightPt = temp;
        frontLine.update_derived_properties();
      }
      
      // If back intersection is invalid, then determine back left/right based on distance from front left/right
      if (backValid && !backIntersect.isValid()) {
        float leftMax  = max(frontLeftPt.distanceFrom(backLeftPt), frontRightPt.distanceFrom(backLeftPt));
        float rightMax = max(frontLeftPt.distanceFrom(backRightPt), frontRightPt.distanceFrom(backRightPt));
        
        // Look at the point that's further away
        swap = false;
        if (leftMax > rightMax) {
          if (frontLeftPt.distanceFrom(backLeftPt) > frontRightPt.distanceFrom(backLeftPt)) {
            swap = true;
          }
        }
        else {
          if (frontRightPt.distanceFrom(backRightPt) > frontLeftPt.distanceFrom(backRightPt)) {
            swap = true;
          }
        }
        
        if (swap) {
          // Back left and right are reversed
          EndPoint temp = backLeftPt;
          backLeftPt = backRightPt;
          backRightPt = temp;
          backLine.update_derived_properties();
        }
      }
    }
    if (backIntersect.isValid()) {
      bool swap = false;
      
      // The right point will be closer to the intersection if both points are valid/invalid
      if (backValid && (backLeftPt.isValid() == backRightPt.isValid())
       && (backLeftPt.distanceFrom(backIntersect) < backRightPt.distanceFrom(backIntersect))) {
        swap = true;
      }
      // Assumption: the right point will be the valid point otherwise
      else if (backValid && backLeftPt.isValid() && !backRightPt.isValid()) {
        swap = true;
      }
      
      
      if (swap) {
        // Back left and right are reversed
        EndPoint temp = backLeftPt;
        backLeftPt = backRightPt;
        backRightPt = temp;
        backLine.update_derived_properties();
      }
      
      // If front intersection is invalid, then determine front left/right based on distance from back left/right
      if (frontValid && !frontIntersect.isValid()) {
        float leftMax  = max(backLeftPt.distanceFrom(frontLeftPt), backRightPt.distanceFrom(frontLeftPt));
        float rightMax = max(backLeftPt.distanceFrom(frontRightPt), backRightPt.distanceFrom(frontRightPt));
        
        // Look at the point that's further away
        swap = false;
        if (leftMax > rightMax) {
          if (backLeftPt.distanceFrom(frontLeftPt) > backRightPt.distanceFrom(frontLeftPt)) {
            swap = true;
          }
        }
        else {
          if (backRightPt.distanceFrom(frontRightPt) > backLeftPt.distanceFrom(frontRightPt)) {
            swap = true;
          }
        }
        
        if (swap) {
          // Front left and right are reversed
          EndPoint temp = frontLeftPt;
          frontLeftPt = frontRightPt;
          frontRightPt = temp;
          frontLine.update_derived_properties();
        }
      }
    }
  }
  
  float front_dx = frontRightPt.coordX() - frontLeftPt.coordX();
  float front_dy = frontRightPt.coordY() - frontLeftPt.coordY();
  float back_dx  = backRightPt.coordX() - backLeftPt.coordX();
  float back_dy  = backRightPt.coordY() - backLeftPt.coordY();
  
  if (frontValid && backValid) {
    // Calculate the length/width
    length = max(frontLine.getLength(), backLine.getLength());
    width  = max(frontLine.perpendicularDistanceFrom( backLine.getCentroid() ), backLine.perpendicularDistanceFrom( frontLine.getCentroid() ));

    // Calculate the orientation based on the front and back line
    AngSignPi front_orientation = AngSignPi(std::atan2(front_dy, front_dx) + (direction_t)M_PI / 2);
    AngSignPi back_orientation  = AngSignPi(std::atan2(back_dy, back_dx) + (direction_t)M_PI / 2);
    
    // Check left/right of line by coming up with a point that's 1000mm away,
    // and if the point is closer to the other line, then orientation should be flipped
    float dist = 1000.0f;
    Point frontPt( getCentroid().coordX() + dist * cos(front_orientation), getCentroid().coordY() + dist * sin(front_orientation), getCentroid().coordZ() );
    if ((width > 50.0f) && (frontLine.perpendicularDistanceFrom( frontPt ) > backLine.perpendicularDistanceFrom( frontPt ))) {
      // Front left and right are reversed
      EndPoint temp = frontLeftPt;
      frontLeftPt = frontRightPt;
      frontRightPt = temp;
      frontLine.update_derived_properties();
      
      front_dx = frontRightPt.coordX() - frontLeftPt.coordX();
      front_dy = frontRightPt.coordY() - frontLeftPt.coordY();
      front_orientation = AngSignPi(std::atan2(front_dy, front_dx) + (direction_t)M_PI / 2);
    }
    
    Point backPt( getBackCentroid().coordX() - dist * cos(back_orientation), getBackCentroid().coordY() - dist * sin(back_orientation), getBackCentroid().coordZ() );
    if ((width > 50.0f) && (backLine.perpendicularDistanceFrom( backPt ) > frontLine.perpendicularDistanceFrom( backPt ))) {
      // Back left and right are reversed
      EndPoint temp = backLeftPt;
      backLeftPt = backRightPt;
      backRightPt = temp;
      backLine.update_derived_properties();
      
      back_dx  = backRightPt.coordX() - backLeftPt.coordX();
      back_dy  = backRightPt.coordY() - backLeftPt.coordY();
      back_orientation  = AngSignPi(std::atan2(back_dy, back_dx) + (direction_t)M_PI / 2);
    }
    
    // Use the orientation of the line with most valid points
    int frontValidCount = 0, backValidCount = 0;
    if (frontLeftPt.isValid()) frontValidCount++;
    if (frontRightPt.isValid()) frontValidCount++;
    if (backLeftPt.isValid()) backValidCount++;
    if (backRightPt.isValid()) backValidCount++;
    
    if (frontValidCount == backValidCount) {
      // Take the average of the 2 orientations
      orientation = AngSignPi(std::atan2(std::sin((direction_t)front_orientation) + std::sin((direction_t)back_orientation), std::cos((direction_t)front_orientation) + std::cos((direction_t)back_orientation)));
    }
    else if (frontValidCount > backValidCount) {
      orientation = front_orientation;
    }
    else {
      orientation = back_orientation;
    }
  }
  // If only one line is valid, obtain the length/orientation from it
  else if (frontValid) {
    length = frontLine.getLength();
    width = 0;
    orientation = AngSignPi(std::atan2(front_dy, front_dx) + (direction_t)M_PI / 2);
  }
  else if (backValid) {
    length = backLine.getLength();
    width = 0;
    orientation = AngSignPi(std::atan2(back_dy, back_dx) + (direction_t)M_PI / 2);
  }
  else {
    length = 0;
    width = 0;
    orientation = 0;
  }
  
}

bool TargetData::updateParams(const ShapeRoot& other, bool force) {
  const Shape<TargetData>& other_target = ShapeRootTypeConst(other,TargetData);
  if (other_target->confidence <= 0)
    return false;
    
  const int other_conf = other_target->confidence;
  confidence += other_conf;
  
  height = (height*confidence + other_target->getHeight()*other_conf) / (confidence+other_conf);
  
  // Update front line
  EndPoint &frontLeftPt  = getFrontLeftPt();
  EndPoint &frontRightPt = getFrontRightPt();
  if (!frontValid) {
    frontLeftPt  = other_target->getFrontLeftPt();
    frontRightPt = other_target->getFrontRightPt();
  }
  else if (other_target->isFrontValid()) {
    NEW_SHAPE(otherFrontLine, LineData, other_target->frontLine);
    frontLine.updateParams(otherFrontLine, force);
    otherFrontLine.getSpace().deleteShape(otherFrontLine);
  }
  
  // Update back line
  EndPoint &backLeftPt  = getBackLeftPt();
  EndPoint &backRightPt = getBackRightPt();
  if (!backValid) {
    backLeftPt  = other_target->getBackLeftPt();
    backRightPt = other_target->getBackRightPt();
  }
  else if (other_target->isBackValid()) {
    NEW_SHAPE(otherBackLine, LineData, other_target->backLine);
    backLine.updateParams(otherBackLine, force);
    otherBackLine.getSpace().deleteShape(otherBackLine);
  }
  
  // Update front intersection point
  if (frontIntersect == Point(0, 0, 0)) {
    frontIntersect = other_target->getFrontIntersect();
  }
  else if (other_target->getFrontIntersect() != Point(0, 0, 0)) {
    if (frontIntersect.isValid() == other_target->getFrontIntersect().isValid()) {
      frontIntersect.updateParams(other_target->getFrontIntersect());
    }
    else if (other_target->getFrontIntersect().isValid()) {
      frontIntersect = other_target->getFrontIntersect();
    }
  }
  
  // Update back intersection point
  if (backIntersect == Point(0, 0, 0)) {
    backIntersect = other_target->getBackIntersect();
  }
  else if (other_target->getBackIntersect() != Point(0, 0, 0)) {
    if (backIntersect.isValid() == other_target->getBackIntersect().isValid()) {
      backIntersect.updateParams(other_target->getBackIntersect());
    }
    else if (other_target->getBackIntersect().isValid()) {
      backIntersect = other_target->getBackIntersect();
    }
  }
  
  update_derived_properties();
  
  return true;
}

void TargetData::mergeWith(const ShapeRoot& other) {
  const Shape<TargetData>& other_target = ShapeRootTypeConst(other,TargetData);
  if (other_target->confidence <= 0)
    return;
  const int other_conf = other_target->confidence;
  confidence += other_conf;
  
  height = (height*confidence + other_target->height*other_conf) / (confidence+other_conf);
  NEW_SHAPE(otherFrontLine, LineData, other_target->frontLine);
  NEW_SHAPE(otherBackLine, LineData, other_target->backLine);
  frontLine.mergeWith(otherFrontLine);
  backLine.mergeWith(otherBackLine);
  otherFrontLine.getSpace().deleteShape(otherFrontLine);
  otherBackLine.getSpace().deleteShape(otherBackLine);
  
  frontIntersect.updateParams(other_target->getFrontIntersect());
  backIntersect.updateParams(other_target->getBackIntersect());
  
  update_derived_properties();
}

void TargetData::printParams() const {
  cout << "Type = " << getTypeName() << endl;
  cout << "Shape ID = " << getId() << endl;
  cout << "Parent ID = " << getParentId() << endl;
  
  // Print all parameters.
  const EndPoint frontLeftPt  = getFrontLeftPt();
  const EndPoint frontRightPt = getFrontRightPt();
  const EndPoint backLeftPt   = getBackLeftPt();
  const EndPoint backRightPt  = getBackRightPt();
  const Point centroid = getCentroid();
  
  cout << endl;
  cout << "centroid = (" << centroid.coordX() << ", " << centroid.coordY() << ", " << centroid.coordZ() << ")" << endl;
  cout << "front left pt = (" << frontLeftPt.coordX() << ", " << frontLeftPt.coordY() << ", " << frontLeftPt.coordZ() << "), valid = " << frontLeftPt.isValid() << endl;
  cout << "front right pt = (" << frontRightPt.coordX() << ", " << frontRightPt.coordY() << ", " << frontRightPt.coordZ() << "), valid = " << frontRightPt.isValid() << endl;
  cout << "back left pt = (" << backLeftPt.coordX() << ", " << backLeftPt.coordY() << ", " << backLeftPt.coordZ() << "), valid = " << backLeftPt.isValid() << endl;
  cout << "back right pt = (" << backRightPt.coordX() << ", " << backRightPt.coordY() << ", " << backRightPt.coordZ() << "), valid = " << backRightPt.isValid() << endl;
  cout << "front intersect = (" << frontIntersect.coordX() << ", " << frontIntersect.coordY() << ", " << frontIntersect.coordZ() << "), valid = " << frontIntersect.isValid() << endl;
  cout << "back intersect = (" << backIntersect.coordX() << ", " << backIntersect.coordY() << ", " << backIntersect.coordZ() << "), valid = " << backIntersect.isValid() << endl;
  cout << "front valid = " << frontValid << endl;
  cout << "back valid = " << backValid << endl;
  cout << "orientation = " << (float)orientation * 180.0 / M_PI << " deg" << endl;
  cout << "length = " << length << endl;
  cout << "width = " << width << endl;
  cout << "height = " << height << endl;
}

void TargetData::applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref) {
  if (frontValid)
    frontLine.applyTransform(Tmat, newref);
  if (backValid)
    backLine.applyTransform(Tmat, newref);
  if (frontIntersect != Point(0, 0, 0))
    frontIntersect.applyTransform(Tmat, newref);
  if (backIntersect != Point(0, 0, 0))
    backIntersect.applyTransform(Tmat, newref);
  update_derived_properties();
}

void TargetData::projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane) {
  // project the lines to elevated ground space
  PlaneEquation target_plane = groundplane;
  float const new_displacement = target_plane.getDisplacement() + target_plane.getZsign()*height;
  target_plane.setDisplacement(new_displacement);
  
  if (frontValid)
    frontLine.projectToGround(camToBase, target_plane);
  if (backValid)
    backLine.projectToGround(camToBase, target_plane);
  if (frontIntersect != Point(0, 0, 0))
    frontIntersect.projectToGround(camToBase, target_plane);
  if (backIntersect != Point(0, 0, 0))
    backIntersect.projectToGround(camToBase, target_plane);
  update_derived_properties();
}

// Doesn't actually render a target to sketch space
Sketch<bool>* TargetData::render() const {
  SketchSpace &SkS = space->getDualSpace();
  
  Sketch<bool>& draw_result = 
    *new Sketch<bool>(SkS, "render("+getName()+")");
  draw_result->setParentId(getViewableId());
  draw_result->setColor(getColor());
  
  fmat::Column<3> ctr(getCentroid().getCoords());
  SkS.applyTmat(ctr);
  int const cx = int(ctr[0]);
  int const cy = int(ctr[1]);
  draw_result = false;
  draw_result(cx, cy) = true;  
  return &draw_result;
}

TargetData& TargetData::operator=(const TargetData& other) {
  if (&other == this)
    return *this;
  BaseData::operator=(other);
  
  frontLine      = other.frontLine;
  backLine       = other.backLine;
  frontValid     = other.frontValid;
  backValid      = other.backValid;
  frontIntersect = other.frontIntersect;
  backIntersect  = other.backIntersect;
  orientation    = other.orientation;
  length         = other.length;
  width          = other.width;
  height         = other.height;
  
  return *this;
}

float TargetData::perpendicularDistanceFrom(Point point) {
  if (frontValid && backValid)
    return min(frontLine.perpendicularDistanceFrom(point), backLine.perpendicularDistanceFrom(point));
  else if (frontValid)
    return frontLine.perpendicularDistanceFrom(point);
  else if (backValid)
    return backLine.perpendicularDistanceFrom(point);
  else
    return 0;
}

Shape<TargetData> TargetData::extractLineTarget(std::string frontColor, std::string backColor, std::string rightColor, std::string occluderColor, const float height) {
  // get the front, back and right stuff
  NEW_SKETCH(camFrame, uchar, VRmixin::sketchFromSeg());    
  NEW_SKETCH(frontStuff, bool, visops::colormask(camFrame, frontColor));
  NEW_SKETCH(backStuff, bool, visops::colormask(camFrame, backColor));
  NEW_SKETCH(rightStuff, bool, visops::colormask(camFrame, rightColor));
  NEW_SKETCH(occluders, bool, visops::colormask(camFrame, occluderColor));

  NEW_SHAPEVEC(frontBlobs, BlobData, BlobData::extractBlobs(frontStuff, 100));
  NEW_SHAPEVEC(backBlobs, BlobData, BlobData::extractBlobs(backStuff, 100));
  NEW_SHAPEVEC(rightBlobs, BlobData, BlobData::extractBlobs(rightStuff, 100));
  
  // assume the biggest blob forms the front line
  NEW_SKETCH(frontSketch, bool, frontBlobs.size() > 0 ? frontBlobs[0]->getRendering() : visops::zeros(camFrame));
  // assume the biggest blob forms the back line
  NEW_SKETCH(backSketch, bool, backBlobs.size() > 0 ? backBlobs[0]->getRendering() : visops::zeros(camFrame));
  // assume the biggest blob forms the right line
  NEW_SKETCH(rightSketch, bool, rightBlobs.size() > 0 ? rightBlobs[0]->getRendering() : visops::zeros(camFrame));
  
  Shape<TargetData> result = extractLineTarget(frontSketch, backSketch, rightSketch, occluders, height);
  
  // delete temporary camera shapes
  //camShS.deleteShapes(horBlobs);
  //camShS.deleteShapes(vertBlobs);
  
  return result;
}

Shape<TargetData> TargetData::extractLineTarget(Sketch<bool>& frontSketch, Sketch<bool>& backSketch, Sketch<bool>& rightSketch, Sketch<bool>& occluders, const float height) {
  NEW_SHAPE(frontLine, LineData, LineData::extractLine(frontSketch, occluders | rightSketch));
  NEW_SHAPE(backLine, LineData, LineData::extractLine(backSketch, occluders | rightSketch));
  NEW_SHAPE(rightLine, LineData, LineData::extractLine(rightSketch, occluders));

  Shape<TargetData> result = extractLineTarget(frontLine, backLine, rightLine, frontSketch->getColor(), height);
  
  // delete temporary camera shapes
  if (frontLine.isValid())
    frontLine.getSpace().deleteShape(frontLine);
  if (backLine.isValid())
    backLine.getSpace().deleteShape(backLine);
  if (rightLine.isValid())
    rightLine.getSpace().deleteShape(rightLine);
  
  return result;
}

Shape<TargetData> TargetData::extractLineTarget(Shape<LineData>& frontLine, Shape<LineData>& backLine, Shape<LineData>& rightLine, rgb color, const float height) {
  
  // create the target using the front and back lines
  if (frontLine.isValid() && backLine.isValid()) {
    EndPoint frontIntersect = Point(0, 0, 0);
    EndPoint backIntersect = Point(0, 0, 0);
    if (rightLine.isValid()) {
      frontIntersect = frontLine->intersectionWithLine(rightLine);
      backIntersect  = backLine->intersectionWithLine(rightLine);
      frontIntersect.setValid(frontLine->pointOnLine(frontIntersect));
      backIntersect.setValid(backLine->pointOnLine(backIntersect));
    }
    else {
      frontIntersect.setValid(false);
      backIntersect.setValid(false);
    }
  
    Shape<TargetData> result(frontLine.getSpace(), frontLine->leftPt(), frontLine->rightPt(), backLine->leftPt(), backLine->rightPt(), frontIntersect, backIntersect, height);
    result->setColor(color);
    return result;
  }
  else if (frontLine.isValid()) {
    EndPoint left = Point(0, 0, 0);
    EndPoint right = Point(0, 0, 0);
    left.setValid(false);
    right.setValid(false);
    
    EndPoint frontIntersect = Point(0, 0, 0);
    EndPoint backIntersect = Point(0, 0, 0);
    frontIntersect.setValid(false);
    backIntersect.setValid(false);
    
    if (rightLine.isValid()) {
      frontIntersect = frontLine->intersectionWithLine(rightLine);
      frontIntersect.setValid(frontLine->pointOnLine(frontIntersect));
    }
    
    Shape<TargetData> result(frontLine.getSpace(), frontLine->rightPt(), frontLine->leftPt(), right, left, frontIntersect, backIntersect, height);
    result->setColor(color);
    return result;
  }
  else if (backLine.isValid()) {
    EndPoint left = Point(0, 0, 0);
    EndPoint right = Point(0, 0, 0);
    left.setValid(false);
    right.setValid(false);
    
    EndPoint frontIntersect = Point(0, 0, 0);
    EndPoint backIntersect = Point(0, 0, 0);
    frontIntersect.setValid(false);
    backIntersect.setValid(false);
    
    if (rightLine.isValid()) {
      backIntersect = backLine->intersectionWithLine(rightLine);
      backIntersect.setValid(backLine->pointOnLine(backIntersect));
    }
    
    Shape<TargetData> result(backLine.getSpace(), right, left, backLine->rightPt(), backLine->leftPt(), frontIntersect, backIntersect, height);
    result->setColor(color);
    return result;
  }
  else {
    ShapeRoot invalid; // need to define a named variable to avoid warning on next line
    return ShapeRootType(invalid, TargetData);
  }

}

} // namespace
