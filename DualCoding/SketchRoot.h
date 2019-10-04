//-*-c++-*-

#ifndef INCLUDED_SketchRoot_h
#define INCLUDED_SketchRoot_h

#include <iosfwd>

namespace DualCoding {

class SketchSpace;
class SketchDataRoot;

//! Parent class for all Sketch<T>
class SketchRoot {
 public:

  SketchRoot() {}
  virtual ~SketchRoot() {}

  SketchSpace& rootGetSpace() const;
  const SketchDataRoot* rootGetData() const;

  int getNewId() { return ++idCounter; }

  static void reset();

private:
  static int idCounter;

  friend class SketchRootReset;
  friend std::ostream& operator<<(std::ostream &os, const SketchRoot &r);
};

std::ostream& operator<<(std::ostream &os, const SketchRoot &r);

} // namespace

#endif
