//-*-c++-*-
#ifndef _SHAPESPACE_H_
#define _SHAPESPACE_H_

#include <vector>
#include <iostream>

#include "Vision/colors.h"
#include "Shared/fmatSpatial.h"

#include "Shared/Measures.h"
#include "ShapeTypes.h"

namespace DualCoding {

//****************
//
// ShapeSpace holds a collection of data objects, e.g., AgentData,
// LineData, EllipseData

class BaseData;
class ShapeRoot;
class Point;
class SketchSpace;
class LineData;
template<typename T> class Shape;

//! Holds a collection of diverse shapes such as LineData or EllipseData

class ShapeSpace {
private:
  friend class SketchSpace;
  friend class ShapeRoot;
  friend class VisualRoutinesBehavior;
  //  friend class WorldMapBuilder;
  
public:
  std::string name;  //!< Name of the ShapeSpace

private:
  SketchSpace* dualSpace;
  int id_counter;
  std::vector<ShapeRoot> shapeCache;
  ShapeRoot& addShape(BaseData* p);
  ReferenceFrameType_t refFrameType;
  
public:

  //! Constructor for ShapeSpace; requires dual SketchSpace. 
  ShapeSpace(SketchSpace* dualSkS, int init_id=70000, std::string const _name="", 
	     ReferenceFrameType_t _refFrameType=camcentric);
  
  ~ShapeSpace(void);
  
  SketchSpace& getDualSpace(void) const { return *dualSpace; }
  ReferenceFrameType_t getRefFrameType() const { return refFrameType; }
  
  void importShapes(std::vector<ShapeRoot>& foreign_shapes);
  BaseData* importShape(const ShapeRoot& foreign_shape);
  
  void deleteShape(ShapeRoot &b);
  void deleteShapes(std::vector<ShapeRoot>& shapes_vec); 
	template<typename T> void deleteShapes(std::vector<Shape<T> >& shapes_vec) {
		deleteShapes(*(std::vector<ShapeRoot>*)&shapes_vec); }

  template<typename T> void deleteShapes() { deleteShapeType(T::getStaticType()); }
  
  void clear();
  
  std::vector<ShapeRoot>& allShapes(void) { return shapeCache; }
  const std::vector<ShapeRoot>& allShapes(void) const { return shapeCache; }
  std::vector<ShapeRoot> allShapes(ShapeType_t type);
  std::vector<ShapeRoot> allShapes(rgb color);

  ShapeRoot getShapeFromId(int id);

  // Coerce a ShapeSpace into a vector of ShapeRoots it contains
  operator std::vector<ShapeRoot>&() { return shapeCache; }
  std::vector<ShapeRoot>::iterator begin() { return shapeCache.begin(); }
  std::vector<ShapeRoot>::iterator end() { return shapeCache.end(); }

  std::string getShapeListForGUI(void);  
  
  void printParams(void);
  void printSummary(void);
  
  //! Transformation and Location Utilities
  //@{
  void applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref=unspecified);
  
  Point getCentroid(void);
  
  Point getCentroidOfSubset(const std::vector<ShapeRoot>& subset);
  //@}

private:
  ShapeSpace(const ShapeSpace&);  //!< never call this
  ShapeSpace& operator=(const ShapeSpace&); //!< never call this
  void deleteShapeType(ShapeType_t type);
};

} // namespace

#endif
