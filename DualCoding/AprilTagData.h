#ifndef _APRILTAGDATA_H_
#define _APRILTAGDATA_H_

#include <vector>
#include <iostream>

#include "BaseData.h"    // superclass
#include "Point.h"       // Point data member
#include "ShapeFuns.h"
#include "Vision/AprilTags/TagDetection.h"
namespace AprilTags {
	class TagFamily;
}

#include "Sketch.h"

namespace DualCoding {

  class ShapeRoot;
  class SketchSpace;
  template<typename T> class Sketch;

  class AprilTagData : public BaseData {
  public:

    //! Constructor
    AprilTagData(ShapeSpace& _space, const AprilTags::TagDetection& _tagDetection);

    //! Constructor for manually building AprilTag instances in local or world maps
    AprilTagData(ShapeSpace& _space, const AprilTags::TagDetection& _tagDetection, 
								 const Point &_center, const fmat::Quaternion &_q = fmat::Quaternion());

    //! Copy constructor
    AprilTagData(const AprilTagData &other);

    //! Destructor
    ~AprilTagData() {};

    static ShapeType_t getStaticType() { return aprilTagDataType; }

    DATASTUFF_H(AprilTagData);
  
    //! Returns the tag detection represented by this shape
    const AprilTags::TagDetection& getTagDetection() const { return tagDetection; }

    //! Centroid. (Virtual in BaseData.)
    Point getCentroid() const { return center; } 
    void setCentroid(const Point& other) { center.setCoords(other); }
  
    //! Returns the bounding box of the AprilTag object in the camera reference frame; not the same as getTopLeft() etc.
    virtual BoundingBox2D getBoundingBox() const;

    //! Bounding box points with respect to the tag itself, i.e., "top left" is always the same point on the tag, independent of tag orientation
    Point getTopLeft() const;
    Point getTopRight() const;
    Point getBottomLeft() const;
    Point getBottomRight() const;

    //! Match AprilTag objects based on their parameters.  (Virtual in BaseData.)
    virtual bool isMatchFor(const ShapeRoot& other) const;

    virtual bool isAdmissible() const { return true; }

    virtual bool updateParams(const ShapeRoot& other, bool force=false);

    //! Print information about this shape. (Virtual in BaseData.)
    virtual void printParams() const;
  
    //! Transformations. (Virtual in BaseData.)
    void applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref=unspecified);
  
    //! Project to ground
    virtual void projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane);

    virtual unsigned short getDimension() const { return 2; }
  
    //! Properties functions.
    //@{

		//! The ID number encoded in this AprilTag
    int getTagID() const { return tagDetection.id; }

    fmat::Quaternion getQuaternion() const; //! Only works for vertical tags at expected marker height

    int getHammingDistance() const { return tagDetection.hammingDistance; }
    //@}
    //! Extraction.
    static std::vector<Shape<AprilTagData> > extractAprilTags(const Sketch<uchar> &rawY, const AprilTags::TagFamily &tagFamily);

		//! Find a tag with the specified tag ID
    static Shape<AprilTagData> findTag(const std::vector<ShapeRoot> &shapevec, int id);
  
		//! For sorting tags by ID
		class TagIDLessThan : public BinaryShapePred<AprilTagData> {
		public:
			bool operator() (const Shape<AprilTagData> &tag1, const Shape<AprilTagData> &tag2) const;
		};

  protected:
		Point getBottomLeftImagePoint() const;  //!< Used for determining orientation of vertical AprilTags
		Point getBottomRightImagePoint() const;

    //! The tag detection represented by this shape
    AprilTags::TagDetection tagDetection;
    Point center;
    fmat::Quaternion q;
		Point bl, br;

  private:
    //! Render into a sketch space and return pointer. (Private.)
    virtual Sketch<bool>* render() const;
    AprilTagData& operator=(const AprilTagData&); //!< don't call
  };


} // namespace

#endif
