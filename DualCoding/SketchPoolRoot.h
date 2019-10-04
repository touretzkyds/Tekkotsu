//-*-c++-*-
#ifndef INCLUDED_SketchPoolRoot_h
#define INCLUDED_SketchPoolRoot_h

namespace DualCoding {

class SketchSpace;

//! Non-templated parent class of SketchPool<T>.
/*! SketchPoolRoot is the non-templated parent class of SketchPool<T>.
    It is needed in order to reference components of SketchSpace.  We
    can't \#include SketchSpace.h from SketchPool.h because of circular
	 dependencies, but we can safely include it from SketchPoolRoot.cc. */

class SketchPoolRoot {
protected:
  SketchSpace *space;
  std::string name;
  
  int getRefreshCounter() const;

 public:

  SketchPoolRoot(SketchSpace* _space, const std::string& _name) : space(_space), name(_name) {}

  const std::string& getName() const { return name; }
  const std::string& getSpaceName() const;
  
  virtual ~SketchPoolRoot()=0; //!< used as a base class, but never directly instantiated, so has a virtual abstract destructor

  SketchPoolRoot(const SketchPoolRoot&);  //!< never call

  SketchPoolRoot& operator=(const SketchPoolRoot&); //!< never call

};

} // namespace

#endif
