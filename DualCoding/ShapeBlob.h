//-*-c++-*-
#ifndef _SHAPEBLOB_H_
#define _SHAPEBLOB_H_

#include "ShapeRoot.h"
#include "BlobData.h"

namespace DualCoding {

class ShapeSpace;

template<>
class Shape<BlobData> : public ShapeRoot {
public:
  SHAPESTUFF_H(BlobData);   // defined in ShapeRoot.h
};

} // namespace

#endif // _SHAPEBLOB_H_
