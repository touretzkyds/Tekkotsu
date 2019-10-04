#ifndef _SKELETONDATA_H_
#define _SKELETONDATA_H_

#include <vector>
#include <iostream>

#ifdef HAVE_OPENNI
#include <XnTypes.h>
#endif


#include "BaseData.h"    // superclass
#include "Point.h"       // Point data member
#include "Sketch.h"

namespace DualCoding {

#ifndef HAVE_OPENNI
  typedef unsigned int XnSkeletonJoint;
#endif

  class ShapeRoot;

  class Skeleton {
  public:
#ifdef HAVE_OPENNI
    static const unsigned int NumSkelJoints = XN_SKEL_RIGHT_FOOT;
#else
    static const unsigned int NumSkelJoints = 24;
#endif
    Point joints[NumSkelJoints+1];

    bool validJoint(unsigned int i) const {
      return (i <= NumSkelJoints) && 
	(joints[i].coordX() != 0.0 || joints[i].coordY() != 0.0 || joints[i].coordZ() != 0.0);
    }
  };

  class SkeletonData : public BaseData {
  public:

    //! Constructor
    SkeletonData(ShapeSpace& _space, const Skeleton& _skeleton);

    //! Constructor for manually building Skeleton instances in local or world maps
    SkeletonData(ShapeSpace& _space, const Point &_center, const Skeleton& _skeleton);

    //! Copy constructor
    SkeletonData(const SkeletonData &other);

    //! Destructor
    ~SkeletonData() {};

    static ShapeType_t getStaticType() { return skeletonDataType; }

    DATASTUFF_H(SkeletonData);
  
    const Skeleton& getSkeleton() const { return skeleton; }

    //! Centroid. (Virtual in BaseData.)
    Point getCentroid() const { return center; } 
    void setCentroid(const Point& other) { center.setCoords(other); }
  
    //! returns the bounding box of the Skeleton object
    virtual BoundingBox2D getBoundingBox() const;

    //! Bounding box points
    Point getTopLeft() const { return Point(); }  //**** FIX THIS HACK: 4 lines ****
    Point getTopRight() const { return Point(); }
    Point getBottomLeft() const { return Point(); }
    Point getBottomRight() const { return Point(); }

    //! Match Skeleton objects based on their parameters.  (Virtual in BaseData.)
    virtual bool isMatchFor(const ShapeRoot& other) const;

    virtual bool isAdmissible() const { return true; }

    virtual bool updateParams(const ShapeRoot& other, bool force=false);

    //! Print information about this shape. (Virtual in BaseData.)
    virtual void printParams() const;
  
    //! Transformations. (Virtual in BaseData.)
    void applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref=unspecified);
  
    //! Project to ground
    virtual void projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane);

    virtual unsigned short getDimension() const { return 3; }
  
    //! Properties functions.
    //@{
    //@}

  protected:
    //! The skeleton represented by this shape
    Skeleton skeleton;
    Point center;

  private:
    //! Render into a sketch space and return pointer. (Private.)
    virtual Sketch<bool>* render() const;
    SkeletonData& operator=(const SkeletonData&); //!< don't call
  };


} // namespace

#endif
