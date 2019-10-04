//-*-c++-*-

#ifndef INCLUDED_Sketch_h
#define INCLUDED_Sketch_h

#include <valarray>
#include <string>

#include "SketchTypes.h"
#include "SketchRoot.h"

namespace DualCoding {

class SketchSpace;
class SketchIndices;
template<typename T> class SketchData;

//! Smart pointers for referencing @code SketchData<T> @endcode instances
/*! This is the structure that provides safe user-level access to sketches.
 *  It's a smart pointer that does reference counting, and
 *  overloads operator-> so it can do validity checking.
 *  If the validity check succeeds, operator-> dereferences to a
 *  SketchData<T> object.  */

template<typename T>
class Sketch : public SketchRoot {
public:
  int width, height;

  //! The SketchData object referenced by this Sketch.
  SketchData<T> *data;

  //! The image resource for the Sketch, owned by the SketchData object.
  std::valarray<T> *pixels;

  //! Constructor.  Allocates a new SketchData<T> to hold the data.
  Sketch(SketchSpace &_space, const std::string& _name = "(no name)");

  //! Constructor.  Inherits parent and color information from parent sketch.
  Sketch(const std::string& _name, const SketchRoot& parent);

  //! Dummy constructor, for use in vector construction.
  Sketch();

  /*! @brief Copy constructor, used in something like @code Sketch<bool> image = original; @endcode
   *  This is a shallow copy: it does not copy the underlying pixels. */
  Sketch(const Sketch &other);

  //! Shallow copy constructor used by NEW_SKETCH and NEW_SKETCH_N
  Sketch(const Sketch &other, const std::string &name, bool viewable);

  //! Destructor.  Cleans up and decrements SketchData reference count.
  virtual ~Sketch();

  //! Retrieve an existing sketch by name.
  Sketch(const std::string &name, SketchSpace &space);

  //! True if this Sketch actually points to a SketchData
  inline bool isValid() const { return data != NULL; }

  //! Print error message if Sketch fails isValid() test
  void checkValid() const;

  SketchData<T>* operator->() { checkValid(); return data; }
  const SketchData<T>* operator->() const { checkValid(); return data; }

  T& operator[] (size_t idx) { checkValid(); return (*pixels)[idx]; };
  const T& operator[] (size_t idx) const { checkValid(); return (*pixels)[idx]; };
  //! when passed indirection matrix (e.g. idx_left) returns resampled Sketch
  const Sketch<T> operator[] (const Sketch<uint>& indirection) const;

  T& operator() (size_t x, size_t y) { checkValid(); return (*pixels)[y*width + x]; };
  const T& operator() (size_t x, size_t y) const { checkValid(); return (*pixels)[y*width + x]; };

  Sketch& setIndices(const SketchIndices& indices, const T& value);

  //! Make this sketch point to another sketch's SketchData.
  void bind(const Sketch& other);

  //! Assignment operator: copies the pixels.
  Sketch& operator= (const Sketch& other);

  //! Sets all pixels in the Sketch to the specified value.
  Sketch& operator= (const T& value);

  Sketch<bool> operator!() const;
  
  Sketch<T>& operator+= (const Sketch<T>& other);
  Sketch<T>& operator-= (const Sketch<T>& other);
  Sketch<T>& operator*= (const Sketch<T>& other);
  Sketch<T>& operator/= (const Sketch<T>& other);

  Sketch<T>& operator+= (const T value);
  Sketch<T>& operator-= (const T value);
  Sketch<T>& operator*= (const T value);
  Sketch<T>& operator/= (const T value);

  Sketch& operator+= (const int value);
  Sketch& operator-= (const int value);
  Sketch& operator*= (const int value);
  Sketch& operator/= (const int value);

  void printVals() const;

  //! operator for implicitly or explicitly converting to Sketch<bool>
  operator Sketch<bool>() const;

  //! operator for implicity or explicitly converting to Sketch<uchar>
  operator Sketch<uchar>() const;

  //! operator for implicity or explicitly converting to Sketch<usint>
  operator Sketch<usint>() const;

  //! operator for implicity or explicitly converting to Sketch<uint>
  operator Sketch<uint>() const;

  //! operator for implicity or explicitly converting to Sketch<float>
  operator Sketch<float>() const;

};

// ****************************************************************


} // namespace

#include "SketchData.h"
#include "SketchIndices.h"
#include "SketchSpace.h"

namespace visops {
  template <class T> DualCoding::Sketch<T> copy(const DualCoding::Sketch<T>& other);
}

namespace DualCoding {

// First constructor: requires a SketchSpace
template <class T>
Sketch<T>::Sketch(SketchSpace &_space, const std::string &_name)
  : SketchRoot(), width(_space.getWidth()), height(_space.getHeight()),
    data(_space.get_pool(*this).getFreeElement()),
    pixels(&data->pixels)
{
  data->name = _name;
  data->id = getNewId();
  data->parentId = 0;
  data->viewable = false;
  data->colormap = segMap;
  ++data->refcount;
  (*pixels)[_space.dummy] = T();
  /*
  std::cout << "Creating new Sketch " << this << " '" << data->name << "'"
	    << " [" << data << "]"
	    << " id=" << getId() << ",parent=" << getParentId()
	    << ",refcount=" << data->refcount << std::endl;
  */
}

// Second constructor: requires a parent sketch
template <class T>
Sketch<T>::Sketch(const std::string& _name, const SketchRoot& _parent)
  : SketchRoot(), width(), height(), data(NULL), pixels(NULL) {
  SketchSpace& space = _parent.rootGetSpace();
  width = space.getWidth();
  height = space.getHeight();
  data = space.get_pool(*this).getFreeElement();
  data->name = _name;
  data->id = getNewId();
  data->inheritFrom(*_parent.rootGetData());
  data->viewable = false;
  ++data->refcount;
  pixels = &data->pixels;
  (*pixels)[space.dummy] = T();
  /*
  std::cout << "Creating new Sketch " << this << " '" << data->name << "'"
	    << " [" << data << "]"
	    << " id=" << getId() << ",parent=" << getParentId()
	    << ",refcount=" << data->refcount << std::endl;
  */
}

// Dummy constructor
template <typename T>
Sketch<T>::Sketch()
  : SketchRoot(), width(), height(), data(NULL), pixels(NULL) {}

// Retrieve existing sketch
template <typename T>
Sketch<T>::Sketch(const std::string &name, SketchSpace &space)
  : SketchRoot(), width(space.getWidth()), height(space.getHeight()), 
	 data(space.get_pool(*this).findSketchData(name)), 
	 pixels(data != NULL ? &data->pixels : NULL) {
  if ( data != NULL )
	 ++data->refcount;
}

#define NEW_SKETCH_N(name,T,value) DualCoding::Sketch<T> name(value,#name,false);
#define NEW_SKETCH(name,T,value) DualCoding::Sketch<T> name(value,#name,true);
#define GET_SKETCH(name,T,space) DualCoding::Sketch<T> name(#name,space);

template <class T>
Sketch<T>::Sketch(const Sketch<T> &other)
  : SketchRoot(),
    width(other.width), height(other.height),
    data(other.data), pixels(other.pixels) 
{
  if ( isValid() ) {
    ++data->refcount;
  /*
  std::cout << "Copying Sketch " << this << " '"
	    << "'  <--  Sketch '" << other.data->name << "'"
	    << " [" << data << "]"
	    << " id=" << getId() << ",parent=" << getParentId()
	    << ",refcount=" << data->refcount << std::endl;
  */
  }
}

// Shallow copy constructor: does not copy the underlying pixels.
template <class T>
Sketch<T>::Sketch(const Sketch<T> &other, const std::string &name, bool viewable)
  : SketchRoot(),
    width(other.width), height(other.height),
    data(other.data), pixels(other.pixels) 
{
  if ( isValid() ) {
    ++data->refcount;
    data->setName(name);
    if ( viewable )
      data->setViewable(viewable);
  }
}


// Destructor
template <class T> Sketch<T>::~Sketch() {
  if ( isValid() ) {
  /*
  std::cout << "Deleting Sketch " << this << " '" << getName() << "'"
	    << " [" << data << "]"
	    << " id=" << getId() << ",parent=" << getParentId()
	    << ",viewable=" << isViewable()
	    << ",refcount=" << data->refcount << std::endl;
  */
    --data->refcount;
    if ( data->refcount == 0 && ! data->viewable && data->refreshTag < data->space->getRefreshCounter() ) {
      data->clearPending = false;
    }
  }
}

template <class T>
void Sketch<T>::checkValid() const {
  if ( ! isValid() )
    std::cerr << "ERROR!  Attempt to dereference an invalid Sketch." << std::endl;
}

// Parallel indexed access via operator[]

template <class T>
const Sketch<T> Sketch<T>::operator[] (const Sketch<uint>& indirection) const
{
  checkValid();
  Sketch<T> result(*(data->space), "shift("+(*this)->getName()+","+indirection->getName()+")");
  result->setParentId((*this)->getParentId());
  result->setColorMap((*this)->getColorMap());
  for (size_t i = 0; i < pixels->size(); i++) {
    (*result.pixels)[i] = (*pixels)[indirection[i]];
  }
  return result;
}

template <class T>
Sketch<T>& Sketch<T>::setIndices(const SketchIndices& indices, const T& value) {
  checkValid();
  for (SketchIndices::CI it = indices.table.begin(); it != indices.table.end(); ++it)
    (*pixels)[*it] = value;
  return *this;
}

template <class T>
void Sketch<T>::bind(const Sketch<T>& other)
{
  if ( isValid() )
    --data->refcount;
  data = other.data;
  ++data->refcount;
  pixels = other.pixels;
  width = other.width;
  height = other.height;
}

template <class T>
Sketch<T>& Sketch<T>::operator= (const Sketch<T>& other) {
  if ( isValid() )
    if ( other.isValid() ) {
      *pixels = *other.pixels;  // deep assignment copies the pixels
      data->parentId = other.data->parentId;
    } else {
      if ( --data->refcount == 0 && data->refreshTag < data->space->getRefreshCounter() ) {
	data->setViewable(false);
	data->clearPending = false;
      }
      pixels = NULL;
      data = NULL;
    }
  else
    if ( other.isValid() )
      bind(::visops::copy(other));
  return *this;
}

template <class T>
Sketch<T>& Sketch<T>::operator= (const T& value) { 
  checkValid();
  *pixels = value; 
  return *this;
}


//================================================================

//! non-member math operators
//@{

#define DEF_MATHOPS_H(_T1, _T2, _Result) \
  DEF_MATHOP_H( +, _T1, _T2, _Result ) \
  DEF_MATHOP_H( -, _T1, _T2, _Result ) \
  DEF_MATHOP_H( *, _T1, _T2, _Result ) \
  DEF_MATHOP_H( /, _T1, _T2, _Result )

#define DEF_MATHOP_H(_Op, _T1, _T2, _Result) \
Sketch<_Result> operator _Op (const Sketch<_T1> &lhs, const Sketch<_T2> &rhs); \
Sketch<_Result> operator _Op (const Sketch<_T1> &lhs, const _T2 value);

// DEF_MATHOPS_H(bool, bool, bool) disallowed because valarray<bool> doesn't provide arithmetic
DEF_MATHOPS_H(bool, uchar, uchar)
DEF_MATHOPS_H(bool, uint, uint)
DEF_MATHOPS_H(bool, float, float)

DEF_MATHOPS_H(uchar, bool, uchar)
DEF_MATHOPS_H(uchar, uchar, uchar)
DEF_MATHOPS_H(uchar, uint, uint)
DEF_MATHOPS_H(uchar, float, float)

DEF_MATHOPS_H(usint, bool, usint)
DEF_MATHOPS_H(usint, uchar, usint)
DEF_MATHOPS_H(usint, usint, usint)
DEF_MATHOPS_H(usint, float, float)

DEF_MATHOPS_H(uint, bool, uint)
DEF_MATHOPS_H(uint, uchar, uint)
DEF_MATHOPS_H(uint, uint, uint)
DEF_MATHOPS_H(uint, float, float)

DEF_MATHOPS_H(float, bool, float)
DEF_MATHOPS_H(float, uchar, float)
DEF_MATHOPS_H(float, uint, float)
DEF_MATHOPS_H(float, float, float)

#undef DEF_MATHOPS_H
#undef DEF_MATHOP_H

#define DEF_MATHOPS_INT_H(_T1) \
  DEF_MATHOP_INT_H( +, _T1) \
  DEF_MATHOP_INT_H( -, _T1) \
  DEF_MATHOP_INT_H( *, _T1) \
  DEF_MATHOP_INT_H( /, _T1)

#define DEF_MATHOP_INT_H(_Op, _T1) \
Sketch<_T1> operator _Op (const Sketch<_T1>& lhs, const int value);

//DEF_MATHOPS_INT_H(bool, uchar) disallowed because valarray won't mix types
DEF_MATHOPS_INT_H(uchar)
DEF_MATHOPS_INT_H(usint)
DEF_MATHOPS_INT_H(uint)
DEF_MATHOPS_INT_H(float)

#undef DEF_MATHOPS_INT_H
#undef DEF_MATHOP_INT_H

#define DEF_MATHBOOL_INT_H(_Op) \
Sketch<uchar> operator _Op (const Sketch<bool>& lhs, const int value);

DEF_MATHBOOL_INT_H( + )
DEF_MATHBOOL_INT_H( - )
DEF_MATHBOOL_INT_H( * )
DEF_MATHBOOL_INT_H( / )

#undef DEF_MATHBOOL_INT_H

template <class T> Sketch<T>& Sketch<T>::operator+= (const Sketch<T>& other) { *pixels += *other.pixels; return *this; }
template <class T> Sketch<T>& Sketch<T>::operator-= (const Sketch<T>& other) { *pixels -= *other.pixels; return *this; }
template <class T> Sketch<T>& Sketch<T>::operator*= (const Sketch<T>& other) { *pixels *= *other.pixels; return *this; }
template <class T> Sketch<T>& Sketch<T>::operator/= (const Sketch<T>& other) { *pixels /= *other.pixels; return *this; }

template <class T> Sketch<T>& Sketch<T>::operator+= (const T value) {*pixels += (T)value; return *this; }
template <class T> Sketch<T>& Sketch<T>::operator-= (const T value) {*pixels -= (T)value; return *this; }
template <class T> Sketch<T>& Sketch<T>::operator*= (const T value) {*pixels *= (T)value; return *this; }
template <class T> Sketch<T>& Sketch<T>::operator/= (const T value) {*pixels /= (T)value; return *this; }

template <class T> Sketch<T>& Sketch<T>::operator+= (const int value) {*pixels += T(value); return *this; }
template <class T> Sketch<T>& Sketch<T>::operator-= (const int value) {*pixels -= T(value); return *this; }
template <class T> Sketch<T>& Sketch<T>::operator*= (const int value) {*pixels *= T(value); return *this; }
template <class T> Sketch<T>& Sketch<T>::operator/= (const int value) {*pixels /= T(value); return *this; }

  //@}


  //! non-member logical operators
  //@{
#define DEFINE_LOGICAL_OPERATOR(_Op)					             \
template <class T> 								     \
Sketch<bool> operator _Op (const Sketch<T>& lhs, const Sketch<T>& rhs) {             \
  Sketch<bool> result(lhs->getName() + #_Op + rhs->getName(), lhs);                  \
    *(result.pixels) = *(lhs.pixels) _Op *(rhs.pixels); 		             \
    return result;                                                                   \
}										     \
/* continued... */								     \
template <class T>								     \
Sketch<bool> operator _Op (const Sketch<T>& lhs, const T value) {		     \
  Sketch<bool> result(lhs->getName() + #_Op "scalar", lhs);                          \
    *(result.pixels) = *(lhs.pixels) _Op value; 				     \
    return result;								     \
}										     \
/* continued... */								     \
template <class T>								     \
Sketch<bool> operator _Op (const Sketch<T>& lhs, const int value) {		     \
  Sketch<bool> result(lhs->getName() + #_Op "scalar", lhs);                          \
    *(result.pixels) = *(lhs.pixels) _Op T(value); 				     \
    return result;								     \
}

  DEFINE_LOGICAL_OPERATOR( == )
  DEFINE_LOGICAL_OPERATOR( != )
  DEFINE_LOGICAL_OPERATOR( <  )
  DEFINE_LOGICAL_OPERATOR( >  )
  DEFINE_LOGICAL_OPERATOR( <= )
  DEFINE_LOGICAL_OPERATOR( >= )
  DEFINE_LOGICAL_OPERATOR( & )
  DEFINE_LOGICAL_OPERATOR( | )
  DEFINE_LOGICAL_OPERATOR( ^ )
#undef DEFINE_LOGICAL_OPERATOR
//@}

template <class T>
Sketch<bool> Sketch<T>::operator! () const {
  Sketch<bool> result("operator!",*this);
  *(result.pixels) = !(*pixels);
  return result;
}

//! Logical assignment operators.
//@{
Sketch<bool>& operator&= (Sketch<bool>& arg1, Sketch<bool> const& arg2);
Sketch<bool>& operator|= (Sketch<bool>& arg1, Sketch<bool> const& arg2);
Sketch<bool>& operator^= (Sketch<bool>& arg1, Sketch<bool> const& arg2);
//@}

template <class T>
void Sketch<T>::printVals() const {
  std::cout << ((*this)->getName() +":") << std::endl;
  for (size_t i = 0; i < pixels->size(); i++) {
    if ((i % width) == 0)
      std::cout << std::endl;
    std::cout << (*pixels)[i] << ' ';
  }
  std::cout << std::endl;
}

// Type coercion

template <class T>
Sketch<T>::operator Sketch<bool>() const {
  Sketch<bool> converted("bool(" + (*this)->getName() + ")", *this);
  copyPixels(converted, *this);
  return converted;
}

template <class T>
Sketch<T>::operator Sketch<uchar>() const {
  /*
  std::cout << "Converting " << this << " '" << getName() << "'"
	    << " id=" << getId() << ",parent=" << getParentId() << ",refcount=" << data->refcount
	    << " to Sketch<uchar>\n";
  */
  Sketch<uchar> converted("uchar("+(*this)->getName()+")", *this);
  copyPixels(converted, *this);
  return converted;
}

template <>
Sketch<bool>::operator Sketch<uchar>() const;

template <>
Sketch<uchar>::operator Sketch<bool>() const;

template <class T>
Sketch<T>::operator Sketch<usint>() const {
  Sketch<usint> converted("usint("+(*this)->getName()+")", *this);
  copyPixels(converted, *this);
  return converted;
}

template <class T>
Sketch<T>::operator Sketch<uint>() const {
  Sketch<uint> converted("uint("+(*this)->getName()+")", *this);
  copyPixels(converted, *this);
  return converted;
}

template <class T>
Sketch<T>::operator Sketch<float>() const {
  Sketch<float> converted("float("+(*this)->getName()+")", *this);
  copyPixels(converted, *this);
  return converted;
}

//! utility function used by type conversion operators
template<class A, class B>
void copyPixels(Sketch<A>& dest, const Sketch<B>& src)
{
  std::valarray<A> &destpix = *dest.pixels;
  const std::valarray<B> &srcpix = *src.pixels;
  size_t length = src->getSpace().getNumPixels();
  for (size_t i = 0; i < length; i++) {
    destpix[i] = (A)(srcpix[i]);
  }
}

} // namespace

/*! @file
 * @brief Templated class for an image-like Sketch
 * @author neilh (Creator)
 */

#endif
