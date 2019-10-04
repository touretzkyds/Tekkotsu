//-*-c++-*-
#ifndef _BASEDATA_H_
#define _BASEDATA_H_

#include <vector>
#include <iostream>
#include <string>

#include "Shared/Measures.h"
#include "Shared/BoundingBox.h"
#include "Point.h"
#include "Shared/fmatSpatial.h"
#include "ShapeTypes.h"
#include "Vision/colors.h"

namespace DualCoding {

class Point;
class PointData;
class SketchSpace;
template<typename T> class Sketch;
class ShapeRoot;
class ShapeSpace;
class SketchDataRoot;
template<typename T> class Shape;

//! Base class that all shape data classes inherit from, e.g., @a LineData, @a BlobData, etc.
class BaseData {
public:
  friend class ShapeRoot;
  friend class ShapeSpace;
  
  
protected:
  ShapeSpace *space;
  std::string name;
  ShapeType_t type;	
  int id;
  int parentId;
  int lastMatchId;      //!< Id of the shape in the preceding space that gave rise to or was matched to this one.
  int refcount;
  bool viewable;
  rgb color_rgb;
  
  int confidence;       //!< Confidence that this shape exists and isn't noise.
  bool mobile;		//!< True if this shape can move in the world

  bool obstacle;        //!< True if shape is an obstacle
  bool landmark;	//!< True if shape should be used as a landmark
  
  Sketch<bool>* rendering_sketch;
  //DualCoding::ShapeSensorModel* sensorModel;

public:
  //! Constructor
  BaseData(ShapeSpace& _space, ShapeType_t typeval, int _parentId=0);

  //! Copy constructor
  BaseData(const BaseData& otherData);
  
  // BaseData(ShapeType_t typeval, int _parentID=0);

  //! Destructor.
  virtual ~BaseData(void);

  virtual BaseData* clone() const;
  ShapeRoot copy() const;

  ShapeSpace& getSpace() const { return *space; }
  void setSpace(ShapeSpace* _space) { space = _space; }
  
  ReferenceFrameType_t getRefFrameType() const;

  int getId() const { return id; }
  int getParentId() const { return parentId; }
  void setParentId(int _pid) { parentId = _pid; }

  bool isViewable() const { return viewable; }
  void setViewable(bool _viewable) { viewable = _viewable; }

  int getViewableId() const {
    if ( viewable )
      return id;
    else
      return parentId;
  }
  
  void inheritFrom(const BaseData &parent);
  void inheritFrom(const ShapeRoot &parent);
  void inheritFrom(const SketchDataRoot &parent);

  int getLastMatchId() const { return lastMatchId; }
  void setLastMatchId(int _lmid) { lastMatchId = _lmid; }

  const std::string& getName() const { return name; }
  void setName(const std::string& _name) { name = _name; }
  
  void V(std::string const &_name="");
  void N(std::string const &_name="");
  
	virtual void importAdjust() {} //!< Adjust shape components after the shape has been imported (e.g., used by polygons)

  virtual BoundingBox2D getBoundingBox() const { return BoundingBox2D(); }

  //! Confidence.
  //@{
  virtual int getConfidence() const { return confidence; } //!< returns confidence of Data. Reimpletemnted in PolygonData

  void increaseConfidence(int n=1, int maxConfidence=-1);
  void increaseConfidence(const BaseData& other, int maxConfidence=-1);
  void increaseConfidence(const ShapeRoot& other, int maxConfidence=-1);

  void decreaseConfidence();

  void setConfidence(const BaseData& other) { confidence = other.getConfidence(); }
  //@}
  
  //! Type.
  //@{
  //! Get the shape type.
  virtual ShapeType_t getType() const=0;
  const char* getTypeName() const;
  
  //! Test the shape type.
  bool isType(ShapeType_t this_type) const;
  
  //! Test if shape types are the same.
  bool isSameTypeAs(const ShapeRoot& other) const;
  //@}
  
  
  //! Color.
  //@{
  //! Get the color.
  rgb getColor() const { return color_rgb; }
  
  //! Set shape and rendering sketch color.
  //@{
  void setColor(const std::string &color_name);
  void setColor(const rgb &new_color);
  void setColor(const unsigned int color_index);
  //@}

  //! Test if shape colors are the same.
  bool isSameColorAs(const ShapeRoot& other) const;
  
  //@}
  
  //! Shapes match if their coordinates agree.  DOES NOT Assume type and color already checked.
  virtual bool isMatchFor(const ShapeRoot& other) const = 0;

  //! Combine two shapes by taking weighted average depending on confidence level.
  //  virtual void mergeWith(const ShapeRoot& other) = 0;

  //! Shapes are admissible to the local map if they're large enough not to be noise.
  virtual bool isAdmissible() const { return true; }

  //! Update shape parameters after matching to another shape.
  virtual bool updateParams(const ShapeRoot& other, bool forceUpdate=false) = 0;

  virtual unsigned short getDimension() const = 0;

  //! Mobility
  //@{
  bool getMobile() const;
  void setMobile(bool mobility);
  //@}

  virtual bool localizeByCamera() const {return false;}
	
  bool isObstacle() const { return obstacle; }
  void setObstacle(bool _obstacle=true) { obstacle = _obstacle; }
  
  bool isLandmark() const { return landmark; }
  void setLandmark(bool _landmark=true) { landmark = _landmark; }
  
  void deleteRendering();

  //! return the centroid of the shape in point format
  virtual Point getCentroid() const=0;

  virtual Shape<PointData> getCentroidPtShape() const;
  
  //! Set the position of a shape
  virtual void setPosition(const Point &pt);

  //! Prints information about this shape.
  virtual void printParams() const=0;
  
  //! Apply a transformation matrix to the shape.
  virtual void applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref=unspecified)=0;
  
  //! Project to ground plane using given matrix
  virtual void projectToGround(const fmat::Transform& camToBase,
			       const PlaneEquation& groundplane)=0;
	  
  //! Update properties of the shape derived from endpoints or other basic parameters.
  //  virtual void update_derived_properties() {}
  
  //! Rendering.
  //@{
  //! Returns a pointer to the rendering associated with the ShapeRoot object.
  //! If no such rendering exists, it is created.
  Sketch<bool>& getRendering();
  
private:
  //! Render into a sketch space.
  virtual Sketch<bool>* render() const=0;
	
  //@}

public:
	// Generic predicates

  //! returns if a point is inside the shape or not. Reimplemented by EllipseData, SphereData, PolygonData
  virtual bool isInside(const Point&) const { return false; }

	virtual bool isLeftOf(const BaseData& other);
	virtual bool isRightOf(const BaseData& other);
	virtual bool isAbove(const BaseData& other);
	virtual bool isBelow(const BaseData& other);


  //! Assignment operator. Assumes "&other =? this" check is done by the sub class calling this operator
  BaseData& operator=(const BaseData& other); 
};

#define DATASTUFF_H(T) \
  virtual ShapeType_t getType() const { return getStaticType(); } \
  virtual BaseData* clone() const; \
  Shape<T> copy() const;

#define DATASTUFF_CC(T) \
  BaseData* T::clone() const { return new T(*this); } \
  Shape<T> T::copy() const { return Shape<T>((T*)clone()); }

} // namespace

#endif
