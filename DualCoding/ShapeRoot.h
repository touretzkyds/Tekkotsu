//-*-c++-*-
#ifndef _SHAPEROOT_H_
#define _SHAPEROOT_H_

#include <vector>

#include "ShapeSpace.h"
#include "ShapeTypes.h"
#include "BaseData.h"

namespace DualCoding {

//! Parent class for all Shape<xxxData> objects

/*! Shape<T> points to data objects of type T within a ShapeSpace, e.g.,
    @a Shape<LineData> points to a @a LineData object.

    Creating a Shape registers the data object in the ShapeSpace and sets the
    refcount to 1.  Copying a shape increments the refcount, and deleting
    a shape decrements it.  If refcount goes to zero we delete the
    data object.  If the user calls deleteShape on a Shape, we remove the
    shape from the ShapeSpace but don't actually delete it until the
    reference count goes to zero.
*/

class ShapeRoot {
protected:
  int        id;
  BaseData   *data;
  
  friend class ShapeSpace;
  friend std::ostream& operator<<(std::ostream &os, const ShapeRoot &r);
  friend std::istream& operator>>(std::istream &is, ShapeRoot &r) { r.id = 0; r.data = NULL; return is; }; // dummy to make DataEvent's load/save happy

public:
  //! Construct a dummy ShapeRoot, e.g., so we can assign into it, or return it
  //! as an indicator of an invalid or failure result.
  ShapeRoot(void): id(-1), data(NULL) {};  
  
  //! The usual constructor.
  ShapeRoot(BaseData *p);
  
  //! Copy constructor: shallow copy
  ShapeRoot(const ShapeRoot &other);

  virtual ~ShapeRoot(void);

  void deleteShape(void);

  void sanity_check(void) const;
  
  inline bool isValid() const { 
    return data != NULL && id > 0 && id == data->id;
  }

  void setInvalid() {
    if ( id > 0 )
      id = -id;
  }

  inline operator bool() const { return isValid(); }

  // Overload the -> operator, and other things...
  virtual BaseData* operator-> (void) { sanity_check(); return data; }
  virtual const BaseData* operator-> (void) const { sanity_check(); return data; }
  virtual BaseData& operator* (void) { sanity_check(); return *data; }
  virtual const BaseData& operator* (void) const { sanity_check(); return *data; }
  virtual BaseData& getData() { return *data; }
  virtual const BaseData& getData() const { return *data; }

  int getId() const { return id; }
  ShapeSpace& getSpace() const;

  //! Shape comparison.  Invalid shapes are considered equal.
  //@{
  virtual bool operator==(const ShapeRoot& other) const;

  virtual bool operator!=(const ShapeRoot& other) const {
    return ! operator==(other);
  }
  //@}

  ShapeRoot& operator=(const ShapeRoot&);

protected:
  ShapeRoot& addShape(BaseData *p) { return p->space->addShape(p); }

};

std::ostream& operator<<(std::ostream &os, const ShapeRoot &r);

// ****************


#define ShapeRootTypeConst(_arg,_type) (*reinterpret_cast<const Shape<_type>*>(&_arg))
#define ShapeRootType(_arg,_type) (*reinterpret_cast<Shape<_type>*>(&_arg))

#define SHAPESTUFF_H(T) \
  Shape<T>() : ShapeRoot() {} \
  Shape<T>(T* newdata) : ShapeRoot(addShape(newdata)) {}	\
  T* operator->() { sanity_check(); return static_cast<T*>(data); } \
  const T* operator->() const { sanity_check(); return static_cast<T*>(data); } \
  T& operator*() { sanity_check(); return *static_cast<T*>(data); } \
  const T& operator*() const { sanity_check(); return *static_cast<T*>(data); } \
  T& getData() { return *static_cast<T*>(data); } \
  const T& getData() const { return *static_cast<T*>(data); } \
	operator BaseData&() { return *data; }

	
#define SHAPESTUFF_CC(T) \


} // namespace

#endif
