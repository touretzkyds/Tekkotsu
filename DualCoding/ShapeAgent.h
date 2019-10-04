//-*-c++-*-
#ifndef _SHAPEAGENT_H_
#define _SHAPEAGENT_H_

#include "Point.h"
#include "ShapeRoot.h"
#include "AgentData.h"

namespace DualCoding {

class ShapeSpace;

template<>
class Shape<AgentData> : public ShapeRoot {
public:
  SHAPESTUFF_H(AgentData);

  Shape<AgentData>(ShapeSpace &s, Point centerval=Point(0,0,0,allocentric)) :
    ShapeRoot(addShape(new AgentData(s,centerval))) { };
};

} // namespace

#endif
