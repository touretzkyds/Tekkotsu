//-*-c++-*-
#ifndef _BRICKDATA_H_
#define _BRICKDATA_H_

#include <vector>
#include <iostream>

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

class BrickData : public BaseData {
protected:
  // T=Top, G=Ground, F=front, B=back, L=left, R=right
  EndPoint GFL;
  EndPoint GFR;
  EndPoint GBL;
  EndPoint GBR;
  EndPoint TFL;
  EndPoint TFR;
  EndPoint TBL;
  EndPoint TBR;

  Point centroid;
	fmat::Quaternion orientation;

public:

  //! Constructor, with points
  BrickData(ShapeSpace& _space,
						const EndPoint &GFL, const EndPoint &GBL, const EndPoint &GFR, const EndPoint &GBR, 
						const EndPoint &TFL, const EndPoint &TBL, const EndPoint &TFR, const EndPoint &TBR,
						const fmat::Quaternion &orient = fmat::Quaternion::IDENTITY);
  
  //! Constructor, with center, half-extents, and orientation
  BrickData(ShapeSpace& _space,
						const fmat::SubVector<3,const fmat::fmatReal>& _centroid,
						fmat::Column<3> extents,
						const fmat::SubMatrix<3,3,const fmat::fmatReal>& o);

  //! Copy constructor
  //BrickData(BrickData& otherBrick);

  static ShapeType_t getStaticType() { return brickDataType; }
  DATASTUFF_H(BrickData);
  
  //! Centroid. (Virtual in BaseData.)
  Point getCentroid() const { return centroid; }
	const fmat::Quaternion& getOrientation() const { return orientation; }
  
  EndPoint getGFL() const {return GFL;}
  EndPoint getGFR() const {return GFR;}
  EndPoint getGBL() const {return GBL;}
  EndPoint getGBR() const {return GBR;}
  EndPoint getTFL() const {return TFL;}
  EndPoint getTFR() const {return TFR;}
  EndPoint getTBL() const {return TBL;}
  EndPoint getTBR() const {return TBR;}
  
	//! Get bounding box in the plane
	BoundingBox2D getBoundingBox() const;

  //! Match bricks based on their parameters.  (Virtual in BaseData.)
  virtual bool isMatchFor(const ShapeRoot& other) const;

  virtual void mergeWith(const ShapeRoot& other);

  virtual bool isAdmissible() const { return true; }

  virtual bool updateParams(const ShapeRoot& other, bool force=false);

  //! Print information about this shape. (Virtual in BaseData.)
  virtual void printParams() const;
  
  //! Transformations. (Virtual in BaseData.)
  void applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref=unspecified);
  
  //! Project to ground
  virtual void projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane);

  virtual unsigned short getDimension() const { return 3; }

  //! Extraction.
  static std::vector<Shape<BrickData> > findBricks(ShapeSpace& ShS, std::vector<Shape<LineData> > lines);

  static std::vector<Shape<BrickData> > findBricksFromBlobs(ShapeSpace& ShS, 
							    std::vector<Shape<BlobData> > blobs1,
							    std::vector<Shape<BlobData> > blobs2,
							    std::vector<Shape<BlobData> > blobs3);

  static Shape<BrickData> extractBrick(ShapeSpace& space, std::vector<Shape<BlobData> > &blobs);
    
  static std::vector<Point> findOrthogonalBoundingBox(ShapeSpace& space, Shape<BlobData> blob, Point centroid, Shape<LineData> parallel);
private:
  //! Render into a sketch space and return reference. (Private.)
  Sketch<bool>* render() const;


  static int addBrickWithTwoSides(ShapeSpace& ShS,
				  std::vector<Point>& corners1, 
				  std::vector<Point>& corners2, 
				  std::vector<std::vector<Point> >& blobs3, 
				  std::vector<Shape<BrickData> >& result, 
				  float distanceThresh);

  //@}

  BrickData& operator=(const BrickData&); //!< don't call
};

} // namespace

#endif
