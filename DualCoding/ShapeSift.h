//-*-c++-*-
#ifndef _SHAPESIFT_H_
#define _SHAPESIFT_H_

#include "ShapeRoot.h"
#include "SiftData.h"

namespace DualCoding {

//! Smart pointer to a @a SiftData object
template<>
class Shape<SiftData> : public ShapeRoot {
public:
  SHAPESTUFF_H(SiftData);   // defined in ShapeRoot.h

  Shape<SiftData>(ShapeSpace &s, SiftMatch* match)
    : ShapeRoot(addShape(new SiftData(s, match))) {};
	
};

} // namespace

#endif
