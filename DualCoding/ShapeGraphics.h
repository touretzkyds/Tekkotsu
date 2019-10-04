#ifndef _SHAPEGRAPHICS_H_
#define _SHAPEGRAPHICS_H_

#include "ShapeRoot.h"
#include "GraphicsData.h"

namespace DualCoding {

class ShapeSpace;

//! Smart pointer to a @a GraphicsData object
template<>
class Shape<GraphicsData> : public ShapeRoot {
public:
  SHAPESTUFF_H(GraphicsData);   // defined in ShapeRoot.h

  Shape<GraphicsData>(ShapeSpace &s)
    : ShapeRoot(addShape(new GraphicsData(s))) {};

};

} // namespace

#endif
