//-*-c++-*-

#include <math.h>

#include "Shared/Config.h"
#include "Motion/Kinematics.h"

#include "Shared/Measures.h"
#include "Point.h"
#include "LineData.h"
#include "ShapeTypes.h"  // for ReferenceFrameType_t
#include "VRmixin.h"
#include "SketchSpace.h"

#ifdef PLATFORM_APERIOS
//! this is normally defined in <math.h>, but the OPEN-R cross-compiler isn't configured right
template<typename T> static inline int signbit(T x) { return x<0; }
#endif

using namespace std;

namespace DualCoding {

float Point::distanceFrom(const Point& otherPt) const {
  return (coords - otherPt.coords).norm();
}

float Point::xyDistanceFrom(const Point& other) const {
  float dx = coordX() - other.coordX();
  float dy = coordY() - other.coordY();
  return sqrt(dx*dx + dy*dy);
}

AngSignPi Point::atanYX() const {
  return atan2(coordY(), coordX());
}

float Point::xyNorm() const { return sqrt(coordX()*coordX() + coordY()*coordY()); }

float Point::xyzNorm() const { return sqrt(coordX()*coordX() + coordY()*coordY() + coordZ()*coordZ()); }

Point Point::unitVector() const {
  return (*this) / xyzNorm();
}


bool Point::isLeftOf(const Point& other, float distance) const {
  switch ( refFrameType ) {
  case camcentric:
    return coordX()+distance < other.coordX();
  case egocentric:  
    return coordY()-distance > other.coordY();
  case allocentric: {
	  // Calculate bearings to both points from current
	  // agent location and check sign of the difference
		AngSignPi thisBearing = ((*this)-VRmixin::theAgent->getCentroid()).atanYX();
		AngSignPi otherBearing = (other-VRmixin::theAgent->getCentroid()).atanYX();
		AngSignPi diff = thisBearing - otherBearing;
		return diff > 0;
	  //cout << "Allocentric Point::isLeftOf fudged for now" << endl;
	  //return coordY()-distance > other.coordY();
	}
  default:
	  cout<<"Unspecified reference frame for Point::isLeftOf"<<endl;
	  return false;
  }
}

bool Point::isRightOf(const Point& other, float distance) const {
  return other.isLeftOf(*this,distance); }

bool Point::isAbove(const Point& other,float distance) const {
  switch ( refFrameType ) {
  case camcentric:
    return coordY()+distance < other.coordY();
  case egocentric:  
    return coordX()-distance > other.coordX();
  case allocentric:
	  // cout << "Allocentric Point::isAbove fudged for now" << endl;
	  return coordX()-distance > other.coordX();
	  // Should really calculate bearings to both points from current
	  // agent location and check sign of the difference
  default:
	  cout<<"Unspecified reference frame for Point::isAbove"<<endl;
	  return false;
  }
}

bool Point::isBelow(const Point& other, float distance) const {
  return other.isAbove(*this,distance); }


//! Apply a transformation matrix to translate and/or rotate  the point.
void Point::applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref) {
	fmat::Column<3> tmp = Tmat*fmat::SubVector<3>(&coords[0]);
	coords[0] = tmp[0];
	coords[1] = tmp[1];
	coords[2] = tmp[2];
	if ( newref != unspecified )
		refFrameType = newref;
}

bool Point::projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane) {
  const float xres = VRmixin::camSkS.getWidth();
  const float yres = VRmixin::camSkS.getHeight();
  const float maxres = max(xres,yres);
  const float normX = float(2*coordX() - xres)/maxres;
  const float normY = float(2*coordY() - yres)/maxres;
  fmat::Column<3> camera_vector;
  config->vision.computeRay(normX, normY, camera_vector[0], camera_vector[1], camera_vector[2]);
	
	/*! Mathematical implementation:
	 *  
	 *  We'll convert the ray to the plane's reference frame, solve there.
	 *  We find a point on the ray (ro_b) and the direction of the ray (rv_b).
	 */
	fmat::Column<3> ro_b = camToBase.translation();
	fmat::Column<3> rv_b = camToBase.rotation() * camera_vector;
	
	/*! Find distance from the ray offset (ro_b) and the closest point on the plane. */
	float dist = groundplane.getDisplacement() - fmat::dotProduct(ro_b, groundplane.getDirection());
	/*! Find scaling factor by projecting ray vector (rv_b) onto plane normal. */
	float align = fmat::dotProduct(rv_b, groundplane.getDirection());
	
	/*! Intersection point will be rv_b*dist/align + ro_b, but need to watch out
	 *  for case where align==0 (rv_b and plane are parallel, no intersection) */
	refFrameType = egocentric;
	if ( std::abs(align) > numeric_limits<float>::epsilon() ) {
		coords = rv_b * (dist/align) + ro_b;
		/*
		std::cout << "Point.cc:  gdisp=" << groundplane.getDisplacement() << "  gdir=" << groundplane.getDirection() << endl;
		std::cout << "  dprod = " << fmat::dotProduct(ro_b, groundplane.getDirection()) << "    dist=" << dist << endl;
		std::cout << "  align=" << align << "   scale=" << dist/align << endl;
		std::cout << "  coord=" << coords << "  rv_b=" << rv_b << "  ro_b=" << ro_b << endl;
		*/
		return true;
	} else if ( align!=0 && dist!=0 && signbit(align)!=signbit(dist) ) {
		coords = -rv_b * 1.0e6f;
		return false;
	} else {
		coords = rv_b * 1.0e6f;
		return false;
	}
}

bool Point::projectToGround(int xres, int yres, const PlaneEquation& groundplane) {
#ifdef TGT_HAS_CAMERA
  // Normalize coordinate system to [-1,+1]
    const float normX = float(2*coordX() - xres)/xres;
    const float normY = float(2*coordY() - yres)/xres;
	fmat::Column<3> camera_vector;
	config->vision.computeRay(normX, normY, camera_vector[0],camera_vector[1],camera_vector[2]);
	fmat::Column<4> ground_point = kine->projectToPlane(CameraFrameOffset, camera_vector, BaseFrameOffset, groundplane, BaseFrameOffset);
	coords = fmat::SubVector<3>(ground_point);
	if(ground_point[3]!=1) {
		// only happens if parallel to ground (or almost so), just scale by a big value (instead of divide-by-zero)
		// this retains the direction for looking at an object
		coords*=1.0e6f;
		return false;
	}
	return true;
  //  std::cout << "Ground plane: " << NEWMAT::printmat(ground_plane) << 
  //    "  camera_point: " << NEWMAT::printmat(camera_point) << 
  //    "  ground_point: " << NEWMAT::printmat(ground_point) << std::endl;
#else
	return false;
#endif
}
  

// Calling point and groundPoint must both have been projectToGround'ed already
float Point::getHeightAbovePoint(const Point& groundPoint, const PlaneEquation& groundplane) {
#ifndef TGT_HAS_CAMERA
	return groundPoint.coordZ();
#else
	fmat::Transform baseToCam = kine->linkToBase(CameraFrameOffset);
	fmat::Column<3> camOffset = baseToCam.translation();
	//std::cout<<std::endl;
	//std::cout<<"cam offset = "<<NEWMAT::printmat(camOffset)<<std::endl;
	//std::cout<<"groundPlane = "<<NEWMAT::printmat(groundplane)<<std::endl;
	camOffset -= groundplane.getDirection()*groundplane.getDisplacement();
	
	//std::cout<<"top pt coords = "<<NEWMAT::printmat(coords)<<std::endl;
	float dx = camOffset[0] - coords[0];
	float dy = camOffset[1] - coords[1];
	float dz = camOffset[2] - coords[2];
	//std::cout<<"Cam to point = "<<dx<<" "<<dy<<" "<<dz<<std::endl;
	float dHorizCam = sqrt(dx*dx + dy*dy);
	
	Point dP = groundPoint - *this;
	float dHorizPoint = sqrt(dP.coordX()*dP.coordX() + dP.coordY()*dP.coordY());
	
	return dz*dHorizPoint/dHorizCam;
#endif
}


Point Point::operator+ (const Point& b) const { return Point(*this) += b; }

Point& Point::operator+= (const Point& b) {
  coords += b.coords;
  makeRefFrameCompatible(b);
  return *this;
}

Point Point::operator- (const Point& b) const { return Point(*this) -= b; }

Point& Point::operator-= (const Point& b) {
  coords -= b.coords;
  makeRefFrameCompatible(b);
  return *this;
}

void Point::makeRefFrameCompatible(const Point &other) {
  refFrameType = ( refFrameType == unspecified || refFrameType == other.refFrameType ) ?
    other.refFrameType : unspecified;
}

Point Point::operator* (float const b) const { return Point(coords*b,refFrameType); }

Point& Point::operator *= (float const b) {
  coords *= b;
  return *this;
}


Point Point::operator/ (float const b) const { return Point(coords/b,refFrameType); }

Point& Point::operator /= (float const b) {
  coords /= b;
  return *this;
}

bool Point::operator== (const Point& b) const {
  return (coordX() == b.coordX() 
					&& coordY() == b.coordY() 
					&& coordZ() == b.coordZ());
}

void Point::printData() const {
  cout<<"{"<<coordX()<<","<<coordY()<<","<<coordZ()<<"}";
}

} // namespace
