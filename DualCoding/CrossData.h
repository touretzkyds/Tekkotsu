#ifndef _CROSSDATA_H_
#define _CROSSDATA_H_

#include <vector>
#include <iostream>
#include "BaseData.h"    // superclass
#include "Point.h"       // Point data member
#include "ShapeFuns.h"
#include "Sketch.h"
#include "LineData.h"

namespace DualCoding {

  class ShapeRoot;
  class SketchSpace;
  template<typename T> class Sketch;

  class CrossData : public BaseData {

	private:
    Point center;
		float armWidth;
		float crossHeight;
		LineData line1;
		LineData line2;

    friend class ShapeSpace;
		friend class Shape<CrossData>;

  public:

    //! Constructor
		CrossData(ShapeSpace& _space, const LineData& line1, const LineData& line2, const fmat::Column<3>& extents);

    //! Destructor
    ~CrossData() {};

    static ShapeType_t getStaticType() { return crossDataType; }

    DATASTUFF_H(CrossData);

    //! get and set for basic parameters (center, armWidth, 
    Point getCentroid() const { return center; } 
    void setCentroid(const Point& other) { center.setCoords(other); }
 
    float getArmWidth() const { return armWidth; }
    void setArmWidth(float newWidth) { armWidth = newWidth; }

    float getcrossHeight() const { return crossHeight; }
    void setcrossHeight(float newHeight) { crossHeight = newHeight; }

		float getArmSemiLength() const { return std::max(line1.getLength(), line2.getLength()) / 2.0; }

    //! Line1. (Virtual in BaseData.)
    const LineData& getLine1() const { return line1; } 
    void setLine1(const LineData& newLine1) { line1 = newLine1; }

    //! Line2. (Virtual in BaseData.)
    const LineData& getLine2() const { return line2; } 
    void setLine2(const LineData& newLine2) { line2 = newLine2; }

    //! Returns the bounding box of the Cross object in the camera reference frame; not the same as getTopLeft() etc.
    virtual BoundingBox2D getBoundingBox() const;

    //! Bounding box points
    Point getTopLeft() const;
    Point getTopRight() const;
    Point getBottomLeft() const;
    Point getBottomRight() const;

    //! Match Cross objects based on their parameters.  (Virtual in BaseData.)
    virtual bool isMatchFor(const ShapeRoot& other) const;
		bool isMatchFor(const CrossData& other) const;

    //! Crosses are admissible if the angle between the lines is reasonable.
    bool isAdmissible() const;

		//! Updates cross given  a new cross
    virtual bool updateParams(const ShapeRoot& other, bool force=false);
		virtual bool updateParams(const CrossData &ground_cross, bool force);

    //! Print information about this shape. (Virtual in BaseData.)
    virtual void printParams() const;
  
    //! Transformations. (Virtual in BaseData.)
    void applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref=unspecified);

    //! Project to ground
    virtual void projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane);

    virtual unsigned short getDimension() const { return 2; }
    
    //TODO: write documentation for which arm is which and possibly rename
    enum ObjectFeature { none=0, arm1, arm2, arm3, arm4 };
    
    //! Compute a point on the destination target based on some feature of the target, e.g., the left side of a domino.
    std::vector<Point> computeGraspPoints() const;


    static std::vector<Shape<CrossData>> extractCrosses(ShapeSpace &ls, Sketch<bool>& sketch, const Sketch<bool>& occlusions, const fmat::Column<3>& dimensions);

  private:
    //! Render into a sketch space and return pointer. (Private.)
    virtual Sketch<bool>* render() const;
    CrossData& operator=(const CrossData&); //!< don't call
  };


} // namespace

#endif

