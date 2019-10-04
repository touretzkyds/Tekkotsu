//-*-c++-*-

#ifndef INCLUDED_SketchDataRoot_h
#define INCLUDED_SketchDataRoot_h

#include <string>

#include "Shared/ProjectInterface.h"

#include "SketchTypes.h"
#include "Point.h"

namespace DualCoding {

class BaseData;
class ShapeRoot;
class ShapeSpace;
class SketchSpace;

//! Parent class for SketchData<T>
class SketchDataRoot {
private:
  //! The SketchSpace that owns the pool containing this SketchData object.
  SketchSpace *space;
  
  //! Name of this sketch.
  std::string name;

  //! Sketch-specific integer ID, for unique identification
  int id;

  //! Integer ID of the "parent" Sketch, 0 if no parent; used in GUI
  int parentId;

  //! Reference count for the sketch. When SketchPool detects this as 0, it may reuse it.
  int refcount;

  //! If true, don't garbage collect this sketch when refcount is zero, so it can still be retrieved by GET_SKETCH
  bool retained;

  //! True if the Sketch is currently viewable via the SketchGUI.
  bool viewable;

  //! Last time this sketch was included in a sketch list sent to the SketchGUI
  int refreshTag;

  //! True if we've tried to clear this sketch but the SketchGUI was looking at it
  bool clearPending;

  //! Color to use for displaying this sketch.  
  //! Only meaningful for Sketch<bool> but info is preserved for inheritance by children
  //! or in case we coerce bool to int or float and then back to bool.
  rgb color;

  //! Which color map to use; default is to use the robot's own color table.
  //! Other tables are used for displaying grayscale images, or scaled quantities like
  //! distance or area using a continuous color scale from red to blue.
  ColorMapType_t colormap;

  template<typename T> friend class SketchPool;
  template<typename T> friend class Sketch;
  template<typename T> friend class SketchData;
  friend class SketchRoot;

public:
  SketchDataRoot(SketchSpace *s) : 
	  space(s), name(), id(0), parentId(0), 
	  refcount(0), retained(false), viewable(false), refreshTag(0), clearPending(false),
	  color((ProjectInterface::getNumColors() != -1U) ? ProjectInterface::getColorRGB(1) : rgb(0,0,255)), // color 0 is invalid, so use color 1 as default, or blue if colors aren't loaded yet
	  colormap(segMap) {}

  virtual ~SketchDataRoot() {};

  //! Returns the SketchSpace that owns the pool containing this SketchData object.
  SketchSpace& getSpace() const { return *space; }

  //! Returns the ShapeSpace associated with the SketchSpace for this Sketch
  ShapeSpace& getDualSpace() const;

  //! Return the id of this sketch
  int getId() const { return id; }

  //! Return the id of this sketch's parent; value is zero for parentless sketches
  int getParentId() const { return parentId; }

  //! Set the parent id of a sketch; normally used only inside visops functions
  void setParentId(int const _id) { parentId = _id; }

  //! Return the id of the closest viewable ancestor; will be the sketch's own id if it's viewable
  int getViewableId() const { return (isViewable() ? getId() : getParentId()); }

  //! Returns true if the sketch is currently viewable by the SketchGUI
  bool isViewable() const { return viewable; }

  //! Sets whether the sketch is viewabile by the SketchGUI
  void setViewable(bool const v=true) { viewable=v; }

  //! Sets or clears the retain bit, which prevents a sketch from being reclaimed even if its reference count goes to zero; such a sketch may still be retrieved via GET_SKETCH
  void retain(bool const r=true) { retained=r; }

  //! Returns true if the sketch is currently retained
  bool isRetained() const { return retained; }

  //! Returns the reference count of the sketch; should always be positive in user code
  int getRefcount() const { return refcount; }

  //! Returns the current color assigned to the sketch; only used for rendering Sketch<bool>
  rgb getColor() const { return color; }

  //@{
  //! Set the color of the sketch
  void setColor(const rgb &_color) { color = _color; }
  void setColor(const std::string &colorname);
  void setColor(const color_index cindex);
  //@}

  //! Return the sketch's current colormap setting
  ColorMapType_t getColorMap() const { return colormap; }

  //! Change the colormap of a sketch
  void setColorMap(const ColorMapType_t _map) { colormap = _map; }

  //! Return the name of a sketch as a string
  const std::string& getName() const { return name; }

  //! Change the name of a sketch.  Note that since the name is actually stored in the SketchData object, all Sketch objects pointing to this sketch will be affected.
  void setName(const std::string &_name) { name = _name; }

  //! Return the type of data stored in this sketch.  Virtual function overridden by each SketchData<T> class.
  virtual SketchType_t getType() const=0;

  //! Make a sketch viewable and change its name; used by NEW_SKETCH macro
  void V(std::string const &_name="");

  //! Make a sketch non-viewable and change its name; used by NEW_SKETCH_N macro
  void N(std::string const &_name="");

  //@{
  //! Set parentId and inherit color and colormap properties from a parent sketch
  void inheritFrom(const SketchDataRoot &parent);
  void inheritFrom(const ShapeRoot &parent);
  void inheritFrom(const BaseData &parent);
  //@}

  //@{
  //! Width and height of sketches in this space.
  const unsigned int getWidth() const;
  const unsigned int getHeight() const;
  const unsigned int getNumPixels() const;
  //@}

  //! X coordinate encoded by sketch index
  int indexX(int index) { return index % getWidth(); }

  //! Y coordinate encoded by sketch index
  int indexY(int index) { return index / getWidth(); }

  //! Convert an index to a Point, using TmatInv
  Point indexPoint(const int index);

  //! Converts (x,y) into a sketch index
  int indexOf(int x, int y) { return y*getWidth() + x; }

  //! Save a sketch to a buffer; used in the SketchGUI interface
  virtual unsigned int saveBuffer(char buf[], unsigned int avail) const;

  virtual size_t savePixels(char buf[], size_t avail) const =0; //!< handle copying pixels to buffer
	
private:
  SketchDataRoot(const SketchDataRoot&); //!< never call this
  SketchDataRoot& operator=(const SketchDataRoot&); //!< never call this
};

} // namespace

#endif
