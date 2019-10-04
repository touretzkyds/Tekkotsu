//-*-c++-*-
#ifndef _TARGETDATA_H_
#define _TARGETDATA_H_

#include "BaseData.h"    // superclass
#include "LineData.h"
#include "Point.h"       // Point data member
#include "EndPoint.h"    // EndPoint data member
#include "ShapeTypes.h"  // TargetDataType

namespace DualCoding {

class ShapeRoot;
class SketchSpace;
template<typename T> class Sketch;

class TargetData : public BaseData {
protected:
  LineData frontLine, backLine;
  bool frontValid, backValid;
  EndPoint frontIntersect, backIntersect;
  AngSignPi orientation;
  float length, width, height;
  
public:
  //! Constructor
  TargetData(ShapeSpace& _space, const EndPoint &_frontLeftPt, const EndPoint &_frontRightPt, const EndPoint &_backLeftPt, const EndPoint &_backRightPt, const EndPoint &_frontIntersect, const EndPoint &_backIntersect, const float _height);

  //! Copy constructor
  TargetData(const TargetData& other)
    : BaseData(other),
      frontLine(other.frontLine), backLine(other.backLine),
      frontValid(other.frontValid), backValid(other.backValid),
      frontIntersect(other.frontIntersect), backIntersect(other.backIntersect),
      orientation(other.orientation), length(other.length), width(other.width), height(other.height) {}

  static ShapeType_t getStaticType() { return targetDataType; }

  DATASTUFF_H(TargetData);
  
  //! Centroid. (Virtual in BaseData.)
  Point getFrontCentroid() const { return frontValid ? frontLine.getCentroid() : backLine.getCentroid(); }
  Point getBackCentroid() const { return backValid ? backLine.getCentroid() : frontLine.getCentroid(); }  
  Point getCentroid() const { return (frontValid && backValid) ? (frontLine.getCentroid() + backLine.getCentroid()) / 2: getFrontCentroid(); }
  
  BoundingBox2D getBoundingBox() const;

  //! Match points based on their parameters.  (Virtual in BaseData.)
  virtual bool isMatchFor(const ShapeRoot& other) const;
  bool isMatchFor(const TargetData& other_target) const;

  //! minimum length of a target
  virtual bool isAdmissible() const { return length > 10.0; }

  //! updates orientation and length from feature points
  void update_derived_properties();
  
  virtual bool updateParams(const ShapeRoot& other, bool force=false);

  virtual void mergeWith(const ShapeRoot& other);

  //! Print information about this shape. (Virtual in BaseData.)
  virtual void printParams() const;
  
  //! Transformations. (Virtual in BaseData.)
  void applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref=unspecified);
  
  //! Project the Target onto the target plane (ground plane lifted up by height)
  virtual void projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane);

  virtual unsigned short getDimension() const { return 3; }

  EndPoint& getFrontLeftPt() { return frontLine.end1Pt(); }
  EndPoint& getFrontRightPt() { return frontLine.end2Pt(); }
  const EndPoint& getFrontLeftPt() const { return frontLine.end1Pt(); }
  const EndPoint& getFrontRightPt() const { return frontLine.end2Pt(); }
  
  EndPoint& getBackLeftPt() { return backLine.end1Pt(); }
  EndPoint& getBackRightPt() { return backLine.end2Pt(); }
  const EndPoint& getBackLeftPt() const { return backLine.end1Pt(); }
  const EndPoint& getBackRightPt() const { return backLine.end2Pt(); }
  
  EndPoint &getFrontIntersect() { return frontIntersect; }
  EndPoint &getBackIntersect() { return backIntersect; }
  const EndPoint &getFrontIntersect() const { return frontIntersect; }
  const EndPoint &getBackIntersect() const { return backIntersect; }
  
  bool isFrontValid() const { return frontValid; }
  bool isBackValid() const { return backValid; }
  
  AngSignPi getOrientation() const { return orientation; }
  float getLength() const { return length; }
  float getWidth()  const { return width; }
  float getHeight() const { return height; }
  
  TargetData& operator=(const TargetData&);
  
  float perpendicularDistanceFrom(Point point);
  
  static Shape<TargetData> extractLineTarget(std::string frontColor = "yellow", std::string backColor = "pink", std::string rightColor = "blue", std::string occluderColor = "orange", const float height = 90.0f);
  static Shape<TargetData> extractLineTarget(Sketch<bool>& frontSketch, Sketch<bool>& backSketch, Sketch<bool>& rightSketch, Sketch<bool>& occluderSketch, const float height = 90.0f);
  static Shape<TargetData> extractLineTarget(Shape<LineData>& camFrontLine, Shape<LineData>& camBackLine, Shape<LineData>& camRightLine, rgb color, const float height = 90.0f);
  
private:
  //! Render into a sketch space and return reference. (Private.)
  Sketch<bool>* render() const;
  //@}

};

} // namespace

#endif

