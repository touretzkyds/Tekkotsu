//-*-c++-*-
#ifndef LOADED_ShapeLandmarks_h
#define LOADED_ShapeLandmarks_h

#include <vector>
#include <string>

#include "Vision/colors.h"

#include "Shared/Measures.h"     // coordinate_t
#include "DualCoding/ShapeRoot.h"
#include "DualCoding/ShapeAprilTag.h"
#include "DualCoding/ShapeCylinder.h"
#include "DualCoding/ShapeMarker.h"

using namespace DualCoding;

//! Root class for the particle filter landmark classes.
class PfRoot {
 public:
  int type;
  int id;
  rgb color;
  bool mobile;
  coordinate_t x, y;
  const ShapeRoot* link;

  PfRoot(int _type, int _id, rgb _color, bool _mobile, coordinate_t _x, coordinate_t _y) :
    type(_type), id(_id), color(_color), mobile (_mobile), x(_x), y(_y),link(0) {}
    
  virtual ~PfRoot() {} //!< destructor, doesn't delete #link

  virtual void print(std::ostream &os) const = 0;

  void printRootInfo(std::ostream &os) const;

  static void loadLms(const std::vector<ShapeRoot> &lms, bool isWorld, std::vector<PfRoot*>& landmarks);

  static void deleteLms(std::vector<PfRoot*>& vec);

  static void findBounds(const std::vector<PfRoot*> &map, 
			 coordinate_t &xmin, coordinate_t &ymin,
			 coordinate_t &xmax, coordinate_t &ymax);
  
  static void printLms(const std::vector<PfRoot*> &lmvec);

private:
  PfRoot(const PfRoot&); //!< don't call this
  PfRoot& operator=(const PfRoot&); //!< don't call this
};

//! A line landmark; world lines will have two of these, with the endpoints switched
class PfLine : public PfRoot {
public:
  coordinate_t x2, y2;
  bool valid1, valid2;
  AngPi orientation;
  float length;

  PfLine(int _id, rgb _color, bool _mobile, coordinate_t _x, coordinate_t _y, 
	 coordinate_t _x2, coordinate_t _y2, bool _v1, bool _v2) :
    PfRoot(lineDataType, _id, _color, _mobile, _x, _y),
    x2(_x2), y2(_y2), valid1(_v1), valid2(_v2), orientation(0), length(0) {}

  virtual void print(std::ostream &os) const;
};

//! An ellipse landmark
class PfEllipse : public PfRoot {
public:
  PfEllipse(int _id, rgb _color, bool _mobile, coordinate_t _x, coordinate_t _y) :
    PfRoot(ellipseDataType, _id, _color, _mobile, _x, _y) {}

  virtual void print(std::ostream &os) const;
};

//! A point landmark
class PfPoint : public PfRoot {
public:
  PfPoint(int _id, rgb _color, bool _mobile, coordinate_t _x, coordinate_t _y) :
    PfRoot(pointDataType, _id, _color, _mobile, _x, _y) {}

  virtual void print(std::ostream &os) const;
};

//! A blob landmark
class PfBlob : public PfRoot {
public:
  PfBlob(int _id, rgb _color, bool _mobile, coordinate_t _x, coordinate_t _y) :
    PfRoot(blobDataType, _id, _color, _mobile, _x, _y) {}

  virtual void print(std::ostream &os) const;
};

//! A marker landmark
class PfMarker : public PfRoot {
public:
  Shape<MarkerData> data;

  PfMarker(int _id, rgb _color, bool _mobile, coordinate_t _x, coordinate_t _y, const Shape<MarkerData>& _data) :
    PfRoot(markerDataType, _id, _color, _mobile, _x, _y), data(_data) {}

  virtual void print(std::ostream &os) const;
};

//! An apriltag landmark
class PfAprilTag : public PfRoot {
public:
  Shape<AprilTagData> data;

  PfAprilTag(int _id, rgb _color, bool _mobile, coordinate_t _x, coordinate_t _y, const Shape<AprilTagData>& _data) :
    PfRoot(aprilTagDataType, _id, _color, _mobile, _x, _y), data(_data) {}

  virtual void print(std::ostream &os) const;
};

//! A cylinder landmark
class PfCylinder : public PfRoot {
public:
  PfCylinder(int _id, rgb _color, bool _mobile, coordinate_t _x, coordinate_t _y, const Shape<CylinderData>& cyl) :
    PfRoot(cylinderDataType, _id, _color, _mobile, _x, _y), radius(cyl->getRadius()), height(cyl->getHeight()) {}

  virtual void print(std::ostream &os) const;

	float radius, height;
};

//! A naught landmark
class PfNaught : public PfRoot {
public:
  PfNaught(int _id, rgb _color, bool _mobile, coordinate_t _x, coordinate_t _y) :
    PfRoot(naughtDataType, _id, _color, _mobile, _x, _y) {}

  virtual void print(std::ostream &os) const;
};

//! A cross landmark
class PfCross : public PfRoot {
public:
  PfCross(int _id, rgb _color, bool _mobile, coordinate_t _x, coordinate_t _y) :
    PfRoot(crossDataType, _id, _color, _mobile, _x, _y) {}

  virtual void print(std::ostream &os) const;
};


std::ostream& operator<<(std::ostream &os, const PfRoot &x);

#endif
