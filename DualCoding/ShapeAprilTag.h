//-*-c++-*-
#ifndef _SHAPEAPRILTAG_H_
#define _SHAPEAPRILTAG_H_

#include "ShapeRoot.h"
#include "AprilTagData.h"

namespace DualCoding {

//! Smart pointer to a @a AprilTagData object
template<>
class Shape<AprilTagData> : public ShapeRoot {
public:
  SHAPESTUFF_H(AprilTagData);   // defined in ShapeRoot.h

  Shape<AprilTagData>(ShapeSpace &s, const AprilTags::TagDetection& tagDetection)
    : ShapeRoot(addShape(new AprilTagData(s, tagDetection))) {};
	
};

} // namespace

#endif
