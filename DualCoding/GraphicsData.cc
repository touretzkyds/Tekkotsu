using namespace std;

#include "Shared/ProjectInterface.h"

#include "GraphicsData.h"
#include "ShapeSpace.h"
#include "ShapeGraphics.h"
#include "Sketch.h"

namespace DualCoding {

GraphicsData::GraphicsData(ShapeSpace  &_space) :
  BaseData(_space, getStaticType()), elements() {
  mobile = false;
  obstacle = false;
}

GraphicsData::~GraphicsData() {
  for (size_t i = 0; i < elements.size(); i++)
    delete elements[i];
}

void GraphicsData::GraphicsElement::setColor(const std::string &colorname) {
  color = ProjectInterface::getColorRGB(colorname);
}

void GraphicsData::add(const GraphicsElement *elt) {
  elements.push_back(elt);
}

void GraphicsData::printParams() const {
  std::cout << "GraphicsData[id = " << getId()
	    << " elements=" << elements.size() << "]" << std::endl;
}

Sketch<bool>* GraphicsData::render() const {
  SketchSpace &SkS = space->getDualSpace();
  return new Sketch<bool>(SkS,"render("+getName()+")");
}

DATASTUFF_CC(GraphicsData);



} // namespace
