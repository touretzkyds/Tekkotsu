//-*-c++-*-
#ifndef _POINT_H_
#define _POINT_H_

#include <iostream>
#include <vector>

#include "Shared/fmatSpatial.h"

#include "Shared/Measures.h"   // coordinate_t, AngPi
#include "ShapeTypes.h" // ReferenceFrameType_t

namespace DualCoding {

class LineData;
template<typename T> class Shape;

/*! We define Point as a separate lightweight class because it is used
 * as a component of all the xxxData classes, and we don't want to nest
 * these structures.
 */

class Point {

public:
  fmat::Column<3> coords;
  ReferenceFrameType_t refFrameType;

  //! Constructors
  //@{
  Point() : coords(), refFrameType(unspecified) {}
  Point(coordinate_t const &xp, coordinate_t const &yp, coordinate_t zp=0, ReferenceFrameType_t ref=unspecified)
	: coords(fmat::pack(xp, yp, zp)), refFrameType(ref) {}
  Point(const fmat::SubVector<3,const float>& c, ReferenceFrameType_t ref=unspecified)
	: coords(c), refFrameType(ref) {}
  //@}

  //! Copy constructor
  Point(const Point& otherPt) : coords(otherPt.coords), refFrameType(otherPt.refFrameType) {}

  //! Destructor
  virtual ~Point() { }

  fmat::Column<3>& getCoords() const { return const_cast<Point*>(this)->coords; }
  coordinate_t coordX() const { return coords[0]; }
  coordinate_t coordY() const { return coords[1]; }
  coordinate_t coordZ() const { return coords[2]; }
  ReferenceFrameType_t getRefFrameType() const { return refFrameType; }

  //! Set position.
  //@{
  void setCoords(const Point& otherPt) { coords = otherPt.coords; }
	void setCoords(const fmat::Column<3> otherCoords) { coords = otherCoords; }
  void setCoords(coordinate_t x, coordinate_t y, coordinate_t z=0) { coords[0]=x; coords[1]=y; coords[2]=z; }
  //@}

  //! Set reference frame type
  void setRefFrameType(const ReferenceFrameType_t ref) { refFrameType = ref; }

  //! Euclidean distance from another point to this one
  float distanceFrom(const Point& other) const;

  //! Euclidean distance from another point to this one, looking only at the x-y projection; z coordinates are ignored
  float xyDistanceFrom(const Point& other) const;

  //! Angle in xy-plane encoded by this vector
  AngSignPi atanYX() const;

  //! Length of the vector's projection in the x-y plane z coordinate is ignored
  float xyNorm() const;

  //! Length of the xyz vector
  float xyzNorm() const;

  //! Unit vector in the direction of this point
  Point unitVector() const;

  //! These functions need a ShapeSpace argument because left/right depends on reference frame type.
  //@{
  bool isLeftOf(const Point& other, float distance=0) const;
  bool isRightOf(const Point& other, float distance=0) const;
  bool isAbove(const Point& other, float distance=0) const;
  bool isBelow(const Point& other, float distance=0) const;
//@}

  //! These functions return true based on relative positions, assuming points lie in some x-y plane (z coordinate is ignored).
  //@{
  bool isBetween(const Point& other1, const Point& other2) const;
  bool isBetween(const Shape<LineData>& line1, const Shape<LineData>& line2) const;
  bool isBetween(const LineData& line1, const LineData& line2) const;
  bool isInside(const std::vector<LineData>& bound) const;
//@}

  float getHeightAbovePoint(const Point& groundPoint, const PlaneEquation& groundplane);
  

  void applyTransform(const fmat::Transform& T, const ReferenceFrameType_t newref=unspecified);
  
  //! projects this to ground plane according to camToBase matrix
  bool projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane);

  bool projectToGround(int xres, int yres, const PlaneEquation& groundplane);

  Point operator+(const Point& b) const;
  Point& operator+=(const Point& b);

  Point operator-(const Point& b) const;
  Point& operator-=(const Point& b);

  Point operator*(float b) const;
  Point& operator*=(float b);

  Point operator/(float b) const;
  Point& operator/=(float b);

  bool operator==(const Point& b) const;
  bool operator!=(const Point& b) const { return !operator==(b); }

  Point& operator=(const Point& b) {
    if (&b==this) return *this;
    setCoords(b);
    refFrameType = b.refFrameType;
    return *this;
  }

  void printData() const;

  friend class EndPoint;

	friend std::ostream& operator<< (std::ostream& out, const Point &p) {
		switch ( p.refFrameType ) {
			case unspecified:
				out << "u:";
				break;
			case camcentric:
				out << "c:";
				break;
			case egocentric:
				out << "e:";
				break;
			case allocentric:
				out << "a:";
				break;
			default:
				out <<"?:";
		}
		out << "[" << p.coordX() << ", " << p.coordY() 
		<< ", " << p.coordZ() << "]";
		return out;
	}
	
private:
  void makeRefFrameCompatible(const Point &other);

};

inline Point& leftMost(Point &pt1, Point &pt2) { return pt1.isLeftOf(pt2) ? pt1 : pt2; }
inline Point& rightMost(Point &pt1, Point &pt2) { return pt1.isLeftOf(pt2) ? pt2 : pt1; }
inline Point& topMost(Point &pt1, Point &pt2) { return pt1.isAbove(pt2) ? pt1 : pt2; }
inline Point& bottomMost(Point &pt1, Point &pt2) { return pt1.isAbove(pt2) ? pt2 : pt1; }
inline const Point& leftMost(const Point &pt1, const Point &pt2) { return pt1.isLeftOf(pt2) ? pt1 : pt2; }
inline const Point& rightMost(const Point &pt1, const Point &pt2) { return pt1.isLeftOf(pt2) ? pt2 : pt1; }
inline const Point& topMost(const Point &pt1, const Point &pt2) { return pt1.isAbove(pt2) ? pt1 : pt2; }
inline const Point& bottomMost(const Point &pt1, const Point &pt2) { return pt1.isAbove(pt2) ? pt2 : pt1; }

} // namespace

#endif
