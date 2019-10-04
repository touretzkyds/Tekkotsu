//-*-c++-*-
#ifndef INCLUDED_ShapeFuns_h_
#define INCLUDED_ShapeFuns_h_

#include <vector>
#include <algorithm>

#include "Shared/ProjectInterface.h"

#include "ShapeTypes.h"
#include "ShapeRoot.h"

namespace DualCoding {

// Note: the double parens in _name((_value)) are needed to keep the
// compiler from confusing a copy constructor call with a function
// description in certain cases, such as this one:
//
//    NEW_SHAPE(foo, LineData, Shape<LineData>(newline))

//! Create a new shape and make it visible in the SketchGUI
#define NEW_SHAPE(_name, _type, _value) \
  DualCoding::Shape<_type> _name((_value)); \
  if ( _name.isValid() ) _name->V(#_name);

//! Create a new shape but hide it from the SketchGUI
#define NEW_SHAPE_N(_name, _type, _value) \
  NEW_SHAPE(_name, _type, _value); \
  if ( _name.isValid() ) _name->N(#_name);

//! Retrieve a shape based on its name
#define GET_SHAPE(_name, _type, _shapevec) \
  DualCoding::Shape<_type> _name(find_shape<_type>(_shapevec,#_name));

//! Retrieve a shape based on its name, but bind to a new name
#define GET_RENAMED(_new_name, _old_name, _type, _shapevec) \
  DualCoding::Shape<_type> _new_name(find_shape<_type>(_shapevec,#_old_name));

//! Create a new vector of shapes of the specified type
#define NEW_SHAPEVEC(_name, _type, _value) \
  std::vector<DualCoding::Shape<_type> > _name((_value));

//! Create a new vector of shapes of mixed type
#define NEW_SHAPEROOTVEC(_name, _value) \
  std::vector<DualCoding::ShapeRoot> _name((_value));

//! Iterate over the shapes in a vector; _var is the iterator
#define SHAPEVEC_ITERATE(_shapesvec,_type,_var) \
  for ( std::vector<DualCoding::Shape<_type> >::iterator _var##_it = _shapesvec.begin(); \
        _var##_it != _shapesvec.end(); _var##_it++ ) { \
    DualCoding::Shape<_type> &_var = *_var##_it;

//! Nested iteration over the shapes in a vector; _var2 begins one past _var1
#define SHAPENEXT_ITERATE(_shapesvec,_type,_var1,_var2) \
  for ( std::vector<DualCoding::Shape<_type> >::iterator _var2##_it = ++std::vector<DualCoding::Shape<_type> >::iterator(_var1##_it); \
        _var2##_it != _shapesvec.end(); _var2##_it++ ) { \
    DualCoding::Shape<_type> &_var2 = *_var2##_it;

//! Iterate over a vector for shapes of mixed type
#define SHAPEROOTVEC_ITERATE(_shapesvec,_var) \
  for ( std::vector<DualCoding::ShapeRoot>::iterator _var##_it = _shapesvec.begin(); \
        _var##_it != _shapesvec.end(); _var##_it++ ) { \
    DualCoding::ShapeRoot &_var = *_var##_it;

#define END_ITERATE }

//! Obsolete: Iterate over the shapes in a vector; _var is the iterator
#define DO_SHAPEVEC(_shapesvec,_type,_var,_body) \
  for ( std::vector<DualCoding::Shape<_type> >::iterator _var##_it = _shapesvec.begin(); \
        _var##_it != _shapesvec.end(); _var##_it++ ) { \
    DualCoding::Shape<_type> &_var = *_var##_it; \
    _body } \

//! Obsolete: nested iteration over the shapes in a vector; _var2 begins one past _var1
#define DO_SHAPENEXT(_shapesvec,_type,_var1,_var2,_body) \
  for ( std::vector<DualCoding::Shape<_type> >::iterator _var2##_it = ++std::vector<DualCoding::Shape<_type> >::iterator(_var1##_it); \
        _var2##_it != _shapesvec.end(); _var2##_it++ ) { \
    DualCoding::Shape<_type> &_var2 = *_var2##_it; \
    _body }

//! Obsolete: Iterate over a vector for shapes of mixed type
#define DO_SHAPEROOTVEC(_shapesvec,_var,_body) \
  for ( std::vector<DualCoding::ShapeRoot>::iterator _var##_it = _shapesvec.begin(); \
        _var##_it != _shapesvec.end(); _var##_it++ ) { \
    DualCoding::ShapeRoot &_var = *_var##_it; \
    _body }

// ================================================================

//! Unary predicates over ShapeRoot objects
class UnaryShapeRootPred : public std::unary_function<const ShapeRoot, bool> {
public:
	virtual ~UnaryShapeRootPred() {} //!< virtual destructor to satisfy warning
};

//! Binary predicates over ShapeRoot objects
class BinaryShapeRootPred : public std::binary_function<const ShapeRoot, const ShapeRoot, bool> {
public:
	virtual ~BinaryShapeRootPred() {} //!< virtual destructor to satisfy warning
};

//! Unary predicates over Shape<T> objects
template <class T>
class UnaryShapePred : public std::unary_function<const Shape<T>, bool> {
public:
	virtual ~UnaryShapePred() {} //!< virtual destructor to satisfy warning
};

//! Binary predicates over Shape<T> objects
template <class T>
class BinaryShapePred : public std::binary_function<const Shape<T>, const Shape<T>, bool> {
public:
	virtual ~BinaryShapePred() {} //!< virtual destructor to satisfy warning
};

// ================ Short-Circuit AND and OR

//! Classes for implementing shortcircuit And and Or predicates.  Don't call directly; use AndPred and OrPred; use not1 for negation.
//@{
template<typename PredType1, typename PredType2>
class shortcircuit_and : public std::unary_function<typename PredType1::argument_type,bool> {
 public:
  PredType1 p1;
  PredType2 p2;
  shortcircuit_and(PredType1 _p1, PredType2 _p2) :
    std::unary_function<typename PredType1::argument_type,bool>(), p1(_p1), p2(_p2) {}
  bool operator() (const typename PredType1::argument_type &shape) const {
    if ( p1(shape) )
      return p2(shape);
    else return false;
  }
};

template<typename PredType1, typename PredType2>
class shortcircuit_or : public std::unary_function<typename PredType1::argument_type,bool> {
 public:
  PredType1 p1;
  PredType2 p2;
  shortcircuit_or(PredType1 _p1, PredType2 _p2) :
    std::unary_function<typename PredType1::argument_type,bool>(), p1(_p1), p2(_p2) {}
  bool operator() (const typename PredType1::argument_type &shape) const {
    if ( p1(shape) )
      return true;
    else return p2(shape);
  }
};

/*! Templated functions can pick up type information from their
  arguments, but templated class constructors cannot; they expect type
  info to be supplied explicitly as template arguments in <>.  So we
  define AndPred() and OrPred() templated functions to pick up the
  type info and pass it on to the shortcircuit_and and shortcircuit_or
  constructors as explicit template arguments.  The built-in function
  adaptors not1 and not2 already use this trick for negation. */

//@{

//! Conjunction of two predicates; p2 is called only if p1 returns true
template<typename PredType1, typename PredType2>
shortcircuit_and<PredType1,PredType2> AndPred(PredType1 p1, PredType2 p2) {
  return shortcircuit_and<PredType1,PredType2>(p1,p2);
}

//! Disjunction of two predicates; p2 is called only if p1 returns false
template<typename PredType1, typename PredType2>
shortcircuit_or<PredType1,PredType2>  OrPred(PredType1 p1, PredType2 p2) {
  return shortcircuit_or<PredType1,PredType2>(p1,p2);
}

//@}

// ================ Unary ShapeRoot predicates

class IsType : public UnaryShapeRootPred {
 public:
  ShapeType_t type;
  explicit IsType(ShapeType_t _type) : UnaryShapeRootPred(), type(_type) {}
   bool operator()(const ShapeRoot &shape) const {
    return shape->getType() == type; }
};

class IsColor : public UnaryShapeRootPred {
public:
  rgb color;
  explicit IsColor(int colorIndex) : UnaryShapeRootPred(), color(ProjectInterface::getColorRGB(colorIndex)) {}
  explicit IsColor(rgb _color) : UnaryShapeRootPred(), color(_color) {}
  explicit IsColor(std::string const &_colorname) : UnaryShapeRootPred(), color(ProjectInterface::getColorRGB(_colorname)) {}
  bool operator()(const ShapeRoot &shape) const {
    return shape->getColor() == color; }
};

class IsName : public UnaryShapeRootPred {
public:
  std::string name;
  explicit IsName(std::string _name) : UnaryShapeRootPred(), name(_name) {}
  bool operator()(const ShapeRoot &shape) const {
     //     return strcmp(shape->getName().c_str(),name.c_str()) == 0; }
     return shape->getName() == name; }
};

class IsNamePrefix : public UnaryShapeRootPred {
public:
  std::string name;
  size_t n;
  explicit IsNamePrefix(std::string _name) : UnaryShapeRootPred(), name(_name), n(_name.size()) {}
  bool operator()(const ShapeRoot &shape) const {
     return strncmp(shape->getName().c_str(), name.c_str(), n) == 0; }
};

class IsLastMatch : public UnaryShapeRootPred {
public:
  ShapeRoot wshape;
  explicit IsLastMatch(ShapeRoot _wshape) : UnaryShapeRootPred(), wshape(_wshape) {}
  bool operator()(const ShapeRoot &lshape) const {
    return lshape->getId() == wshape->getLastMatchId(); }
};

// ================ find_if, find_shape, subset, max_element, stable_sort, etc.

//! Find a ShapeRoot satisfying pred
template<typename PredType>
ShapeRoot find_if(const std::vector<ShapeRoot> &vec, PredType pred) {
  typename std::vector<ShapeRoot>::const_iterator result = find_if(vec.begin(),vec.end(),pred);
  if ( result != vec.end() )
    return *result;
  else
    return ShapeRoot();
}

//! Find a Shape<T> in a vector of ShapeRoot satisfying pred
template<class T, typename PredType>
Shape<T> find_if(const std::vector<ShapeRoot> &vec, PredType pred) {
  shortcircuit_and<IsType,PredType> tpred(AndPred(IsType(T::getStaticType()),pred));
  typename std::vector<ShapeRoot>::const_iterator result = find_if(vec.begin(),vec.end(),tpred);
  if ( result != vec.end() )
    return ShapeRootTypeConst(*result,T);
  else
    return Shape<T>();
}

//! Find a Shape<T> satisfying pred
template<class T, typename PredType>
Shape<T> find_if(const std::vector<Shape<T> > &vec, PredType pred) {
  typename std::vector<Shape<T> >::const_iterator result = find_if(vec.begin(),vec.end(),pred);
  if ( result != vec.end() )
    return *result;
  else
    return Shape<T>();
}

//! Find a Shape<T> in a vector of ShapeRoot
template<class T>
Shape<T> find_if(const std::vector<ShapeRoot> &vec) {
  typename std::vector<ShapeRoot>::const_iterator result = find_if(vec.begin(),vec.end(),IsType(T::getStaticType()));
  if ( result != vec.end() )
    return ShapeRootTypeConst(*result,T);
  else
    return Shape<T>();
}

//! Return the first ShapeRoot with the specified name, else return an invalid ShapeRoot
ShapeRoot find_shape(const std::vector<ShapeRoot> &vec, const std::string &name);

//! Return the first ShapeRoot with the specified name, else return an invalid ShapeRoot
template<class T>
Shape<T> find_shape(const std::vector<ShapeRoot> &vec, const std::string &name) {
  for ( std::vector<ShapeRoot>::const_iterator it = vec.begin();
				it != vec.end(); it++ )
    if ( (*it)->getType() == T::getStaticType() && (*it)->getName() == name )
      return ShapeRootTypeConst(*it,T);
  return Shape<T>();
}

//! Select the Shape<T> elements from a vector of ShapeRoots
template<class T>
std::vector<Shape<T> > select_type(std::vector<ShapeRoot> &vec) {
  std::vector<Shape<T> > result(vec.size());
  result.clear();
  IsType tpred(T::getStaticType());
  DO_SHAPEROOTVEC(vec, element, {
    if ( tpred(element) ) result.push_back(reinterpret_cast<const Shape<T>&>(element));
  });
  return result;
}

//! Find all elements in a vector of Shape<T> satisfying pred
template<class T, typename PredType>
std::vector<Shape<T> > subset(const std::vector<Shape<T> > &vec, PredType pred) {
  std::vector<Shape<T> > result;
  remove_copy_if(vec.begin(), vec.end(),
		 std::back_insert_iterator<std::vector<Shape<T> > >(result),
		 not1(pred));
  return result;
}

//! Find all elements in a vector of ShapeRoot satisfying pred
template<typename PredType>
std::vector<ShapeRoot> subset(const std::vector<ShapeRoot> &vec, PredType pred) {
  std::vector<ShapeRoot> result;
  remove_copy_if(vec.begin(), vec.end(),
		 std::back_insert_iterator<std::vector<ShapeRoot> >(result),
		 not1(pred));
  return result;
}

//! Find all elements of type T in a vector of ShapeRoot satisfying pred
template<class T, typename PredType>
std::vector<T> subset(const std::vector<ShapeRoot> &vec, PredType pred) {
  std::vector<Shape<T> > result;
  shortcircuit_and<IsType,PredType> tpred(AndPred(IsType(T::getStaticType()),pred));
  remove_copy_if(vec.begin(), vec.end(),
		 std::back_insert_iterator<std::vector<Shape<T> > >(result),
		 not1(tpred));
  return result;
}

template<class T, typename ComparisonType>
Shape<T> max_element(const std::vector<Shape<T> > &vec, ComparisonType comp) {
  typename std::vector<Shape<T> >::const_iterator result = max_element(vec.begin(),vec.end(),comp);
  if ( result != vec.end() )
    return *result;
  else
    return Shape<T>();
}

template<typename ComparisonType>
ShapeRoot max_element(const std::vector<ShapeRoot> &vec, ComparisonType comp) {
  typename std::vector<ShapeRoot>::const_iterator result = max_element(vec.begin(),vec.end(),comp);
  if ( result != vec.end() )
    return *result;
  else
    return ShapeRoot();
}

template<class T, typename ComparisonType>
Shape<T> min_element(const std::vector<Shape<T> > &vec, ComparisonType comp) {
  return max_element(vec, not2(comp));
}


template<typename ComparisonType>
ShapeRoot min_element(const std::vector<ShapeRoot> &vec, ComparisonType comp) {
  return max_element(vec, not2(comp));
}

template<class T, typename PredType>
std::vector<Shape<T> > stable_sort(const std::vector<Shape<T> > &vec, const PredType& pred) {
  std::vector<Shape<T> > result(vec);
  stable_sort(result.begin(), result.end(), pred);
  return result;
}

template<typename PredType>
std::vector<ShapeRoot> stable_sort(const std::vector<ShapeRoot> &vec, const PredType& pred) {
  std::vector<ShapeRoot> result(vec);
  stable_sort(result.begin(), result.end(), pred);
  return result;
}

// ================ IsLeftOf / IsRightOf / IsAbove / IsBelow
// ================ IsLeftOfThis / IsRightOfThis / IsAboveThis / IsBelowThis
//
// To enforce consistency, predicates are defined in pairs: IsRightOf(x,y)
// is defined as IsLeftOf(y,x).

#define DIRECTION_PAIR(dir, oppdir)                                             \
class Is##dir : public BinaryShapeRootPred {                                    \
 public:                                                                        \
  Is##dir(float distance=0) : dist(distance) {}                                 \
  bool operator() (const ShapeRoot&, const ShapeRoot&) const;                   \
 private:                                                                       \
  float dist;                                                                   \
};                                                                              \
                                                                                \
class Is##dir##This : public UnaryShapeRootPred {                               \
 public:                                                                        \
  Is##dir##This(const ShapeRoot &_s2, float distance=0) :                       \
     UnaryShapeRootPred(), s2(_s2), dist(distance) {}                           \
  bool operator() (const ShapeRoot &s1) const {                                 \
    return Is##dir(dist)(s1,s2); }                                             \
 private:                                                                       \
  ShapeRoot s2;                                                                 \
  float dist;                                                                   \
};                                                                              \
                                                                                \
class Is##oppdir : public BinaryShapeRootPred {                                 \
 public:                                                                        \
  Is##oppdir(float distance=0) : dist(distance) {}                              \
  bool operator() (const ShapeRoot &s1, const ShapeRoot &s2) const {            \
    return Is##dir(dist) (s2, s1); }                                            \
 private:                                                                       \
  float dist;                                                                   \
};                                                                              \
                                                                                \
class Is##oppdir##This : public UnaryShapeRootPred {                            \
 public:                                                                        \
  Is##oppdir##This(const ShapeRoot &_s2, float distance=0) :                    \
    UnaryShapeRootPred(), s2(_s2), dist(distance) {}                            \
  bool operator() (const ShapeRoot &s1) const {                                 \
    return Is##dir(dist) (s2,s1); }                                             \
 private:                                                                       \
  ShapeRoot s2;                                                                 \
  float dist;                                                                   \
};


DIRECTION_PAIR(LeftOf, RightOf)
DIRECTION_PAIR(Above, Below)

#undef DIRECTION_PAIR

} // namespace

#endif
