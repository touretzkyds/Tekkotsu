//-*-c++-*-
#ifndef _BLOBDATA_H_
#define _BLOBDATA_H_

#include <vector>
#include <set>
#include <map>
#include <iostream>
#include <string>

#include "Shared/fmatSpatial.h"

#include "BaseData.h"      // superclass
#include "Point.h"         // Point data member
#include "SketchTypes.h"   // uchar
#include "ShapeTypes.h"    // blobDataType
#include "ShapeFuns.h"

namespace CMVision {
	typedef unsigned char uchar;
	template<typename T> class run;
	struct region;
}

namespace DualCoding {

class ShapeRoot;
template<typename T> class Sketch;

//! Blob shapes, described by bounding boxes and an optional list of runs.

class BlobData : public BaseData {
 public:

  //! Assumed orientation of the blob in 3D space.
  enum BlobOrientation_t {
    groundplane,  //!< 2D shape lying flat on the ground
    pillar,       //!< 3D shape standing on the ground
    poster        //!< 3D shape hanging vertically in space
  };

  struct run {
  public:
    unsigned short int x, y, width;
    run() : x(0), y(0), width(0) {}
    run(unsigned short int _x, unsigned short int _y, unsigned short int _width) :
      x(_x), y(_y), width(_width) {}
  };

  // Data members

  bool topValid, bottomValid, leftValid, rightValid;

  BlobOrientation_t orientation; //!< Orientation of the blob
  coordinate_t assumedHeight; //!< Assumed height above ground of blob centroid (for poster) or top (for pillar)
  //! Bounding quadrilateral: may not be square when projected to ground space.
  Point topLeft, topRight, bottomLeft, bottomRight;
  float area; //!< Area of the blob; may not be integer when projected to ground space
  const std::vector<run> runvec; //!< Runs (for rendering in camera space)

 public:
  //! Constructor
  BlobData(ShapeSpace& _space,
	   const Point &_topLeft, const Point &_topRight,
	   const Point &_bottomLeft, const Point &_bottomRight,
	   const float _area=0,
	   const std::vector<run> &_runvec=std::vector<run>(), 
	   const BlobOrientation_t _orientation=groundplane,
	   const coordinate_t assumedHeight=0,
	   const rgb rgbvalue=rgb(),
	   bool _topValid=false, bool _bottomValid=false, bool _leftValid=false, bool _rightValid=false);

  static ShapeType_t getStaticType() { return blobDataType; }

  DATASTUFF_H(BlobData);

  friend class Shape<BlobData>;

  //! return the centroid of the shape in point format
  virtual Point getCentroid() const;
  
  //! Area of the blob
  float getArea() const { return area; }

  //! Print information about this shape.
  virtual void printParams() const;

  //! Transformations. (Virtual in BaseData.)
  virtual void applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref=unspecified);
  
  //! Project to ground

  //  virtual void projectToGround(int xres, int yres, const NEWMAT::ColumnVector& groundplane);
  virtual void projectToGround(const fmat::Transform& camToBase,
			       const PlaneEquation& gndplane);

  //! Update derived properties
  virtual void update_derived_properties();

  //! returns the bounding box of the blob
  BoundingBox2D getBoundingBox() const;

  //! Match blobs based on their parameters.  (Virtual in BaseData.)
  virtual bool isMatchFor(const ShapeRoot& other) const;

  virtual bool updateParams(const ShapeRoot& other, bool forceUpdate=false);

  virtual unsigned short getDimension() const { return (orientation==groundplane) ? 2 : 3; }


  //! Import blobs from Sketch<bool> as a vector of Shape<BlobData>
  static std::vector<Shape<BlobData> > 
  extractBlobs(const Sketch<bool> &sketch,
	       int minarea=25,
	       BlobOrientation_t orient=BlobData::groundplane, 
	       coordinate_t height=0,
	       int maxblobs=50); 

  //! Import blobs of all colors from Sketch<uchar> as a vector of Shape<BlobData>
  static std::vector<Shape<BlobData> >
  extractBlobs(const Sketch<uchar> &sketch,
	       int minarea=25,
	       BlobOrientation_t orient=BlobData::groundplane,
	       const coordinate_t height=0,
	       int maxblobs=50);

  //! Import blobs of specified colors from Sketch<uchar> as a vector of Shape<BlobData>
  static std::vector<Shape<BlobData> >
  extractBlobs(const Sketch<uchar> &sketch, 
	       const std::set<color_index>& colors,
	       const std::map<color_index,int>& minareas,
	       const std::map<color_index,BlobOrientation_t>& orients,
	       const std::map<color_index,coordinate_t>& heights,
	       int maxblobs);

  //! Utility function for making a new blob instance from CMVision's region data structures
  static BlobData* new_blob(ShapeSpace& space,
	   const CMVision::region &reg, 
	   const CMVision::run<CMVision::uchar> *rle_buff,
	   const BlobOrientation_t orient,
	   const coordinate_t height,
	   const rgb rgbvalue);

  //!@name Corner extraction for rectangular blobs
  //@{
  std::vector<Point> findCorners(unsigned int nExpected, std::vector<Point>& candidates, float &bestValue);
  std::vector<Point> findCornersDerivative();
  std::vector<Point> findCornersDiagonal();
  std::vector<Point> findCornersShapeFit(unsigned int ncorners, std::vector<Point>& candidates, float &bestValue);
  //@}

 // comparison predicates

  class AreaLessThan : public BinaryShapePred<BlobData> {
  public:
    bool operator() (const Shape<BlobData> &b1, const Shape<BlobData> &b2) const;
  };

private:
  //! Render into a sketch space and return reference. (Private.)
  virtual Sketch<bool>* render() const;

  BlobData& operator=(const BlobData&); //!< don't call

};

} // namespace

#endif // BLOBDATA_H_
