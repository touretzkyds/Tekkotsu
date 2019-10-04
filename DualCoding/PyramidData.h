//-*-c++-*-
#ifndef _PYRAMIDDATA_H_
#define _PYRAMIDDATA_H_

#include <vector>
#include <iostream>
#include <string>

#include "BaseData.h"    // superclass
#include "Point.h"       // Point data members
#include "ShapeTypes.h"  // brickDataType

#include "LineData.h"
#include "ShapeLine.h"
#include "ShapeBlob.h"

namespace DualCoding {

class ShapeRoot;
class SketchSpace;
template<typename T> class Sketch;

class PyramidData : public BaseData {  
 private:
  EndPoint FL;
  EndPoint FR;
  EndPoint BL;
  EndPoint BR;
  EndPoint Top;

  Point centroid;

 public:

  //! Constructor
  PyramidData(ShapeSpace& _space,
	    const EndPoint &FL, const EndPoint &FR, const EndPoint &BL, const EndPoint &BR, 
	    const EndPoint &Top);


  //! Copy constructor
  //PyramidData(PyramidData& otherPyramid);

  static ShapeType_t getStaticType() { return pyramidDataType; }
  DATASTUFF_H(PyramidData);
  
  //! Centroid. (Virtual in BaseData.)
  Point getCentroid() const { return centroid; } 
  
  EndPoint& getFL() {return FL;}
  const EndPoint& getFL() const {return FL;}
  EndPoint& getFR() {return FR;}
  const EndPoint& getFR() const {return FR;}
  EndPoint& getBL() {return BL;}
  const EndPoint& getBL() const {return BL;}
  EndPoint& getBR() {return BR;}
  const EndPoint& getBR() const {return BR;}
  EndPoint& getTop() {return Top;}
  const EndPoint& getTop() const {return Top;}
  
  //! Match pyramids based on their parameters.  (Virtual in BaseData.)
  virtual bool isMatchFor(const ShapeRoot& other) const;

  virtual void mergeWith(const ShapeRoot& other);

  virtual bool isAdmissible() const { return true; }

  virtual bool updateParams(const ShapeRoot& other, bool force=false);

  //! Print information about this shape. (Virtual in BaseData.)
  virtual void printParams() const;
  
  //! Transformations. (Virtual in BaseData.)
  void applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref=unspecified);
  
  //! Project to ground
  void projectToGround(const fmat::Transform & camToBase, const PlaneEquation& groundplane);
    
  virtual unsigned short getDimension() const { return 3; }

  //! Extraction.

  static Shape<PyramidData> extractPyramid(ShapeSpace& space, std::vector<Shape<BlobData> > &blobs);
    
  static std::vector<Point> findBoundingTriangle(ShapeSpace& space, Shape<BlobData> blob, 
					    Point centroid, Shape<LineData> parallel);
private:
  //! Render into a sketch space and return reference. (Private.)
  Sketch<bool>* render() const;


  //@}

  PyramidData& operator=(const PyramidData&); //!< don't call
};

} // namespace

#endif
