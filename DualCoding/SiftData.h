#ifndef _SIFTDATA_H_
#define _SIFTDATA_H_

#include <vector>
#include <string>

#include "BaseData.h"    // superclass
#include "Point.h"       // Point data member
#include "Vision/SIFT/API/SiftMatch.h"
#include "Vision/SIFT/API/SiftTekkotsu.h"

#include "Sketch.h"

namespace DualCoding {

  class ShapeRoot;
  class SketchSpace;
  template<typename T> class Sketch;

  class SiftData : public BaseData {
  private:
    Point center_pt;
    SiftMatch* match;
  
  public:

    //! Constructor
    SiftData(ShapeSpace& _space, SiftMatch* _match);

    //! Copy constructor
    SiftData(const SiftData &other);

    //! Destructor
    ~SiftData();

    static ShapeType_t getStaticType() { return siftDataType; }

    DATASTUFF_H(SiftData);
  
    //! Centroid. (Virtual in BaseData.)
    Point getCentroid() const { return center_pt; } 
    void setCentroidPt(const Point& other) { center_pt.setCoords(other); }
  
    //! returns the bounding box of the sift object
    virtual BoundingBox2D getBoundingBox() const;

    //! Bounding box points
    const Point& getTopLeft() const { return match->topLeft; }
    const Point& getTopRight() const { return match->topRight; }
    const Point& getBottomLeft() const { return match->bottomLeft; }
    const Point& getBottomRight() const { return match->bottomRight; }

    //! Match sift objects based on their parameters.  (Virtual in BaseData.)
    virtual bool isMatchFor(const ShapeRoot& other) const;

    virtual bool isAdmissible() const { return true; }

    virtual bool updateParams(const ShapeRoot& other, bool force=false);

    //! Print information about this shape. (Virtual in BaseData.)
    virtual void printParams() const;
  
    //! Transformations. (Virtual in BaseData.)
    void applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref=unspecified);
  
    //! Project to ground
    virtual void projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane);

    //! Center point access function.
    const Point& centerPt() const { return center_pt; }
  
    virtual unsigned short getDimension() const { return 2; }
  
    //! Properties functions.
    //@{
    int getObjectID() const { return match->objectID; }
    // **** Joehan: write accessors for the remaining fields of SiftMatch
    // const string& getObjectName const { return objectName; }
    //@}
    int getModelID() const { return match->modelID; } 
    const std::string& getObjectName() const { return match->objectName; }
    const std::string& getModelName() const { return match->modelName; }
    double getProbOfMatch() const { return match->probOfMatch; }
    double getError() const { return match->error; }
    double getScale() const { return match->scale; }
    double getOrientation() const { return match->orientation; }
    double getColumnTranslation() const { return match->columnTranslation; }
    double getRowTranslation() const { return match->rowTranslation; }
    const std::vector<keypointPair>& getInliers() const { return match->inliers; }
    const void siftMatchPrint() const { match->print("  "); }
    void getSiftMatchBoundingBox() const { match->computeBoundingBox(); }
    //! Extraction.
    static std::vector<Shape<SiftData> > extractSiftObjects(const Sketch<uchar> &sketch, const SiftTekkotsu &database);
    static std::vector<Shape<SiftData> > extractSiftObjects(const Sketch<uchar> &sketch, const SiftTekkotsu &database, const std::string &objectName);
  
    //! Display matched features as points in the object's shape space
    void displayMatchedFeatures();

  private:
    //! Render into a sketch space and return pointer. (Private.)
    virtual Sketch<bool>* render() const;
    SiftData& operator=(const SiftData&); //!< don't call
  };


} // namespace

#endif
