//-*-c++-*-
#ifndef _SHAPESKELETON_H_
#define _SHAPESKELETON_H_

#include "ShapeRoot.h"
#include "SkeletonData.h"

namespace DualCoding {

//! Smart pointer to a @a SkeletonData object
template<>
class Shape<SkeletonData> : public ShapeRoot {
public:
  SHAPESTUFF_H(SkeletonData);   // defined in ShapeRoot.h

  Shape<SkeletonData>(ShapeSpace &s, const Skeleton& skeleton)
    : ShapeRoot(addShape(new SkeletonData(s, skeleton))) {};
	
};

} // namespace

#endif
