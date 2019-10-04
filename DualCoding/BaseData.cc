#include "BaseData.h"
#include "Sketch.h"   // this must precede references to Sketch
#include "ShapeRoot.h"
#include "ShapePoint.h"
#include "SketchDataRoot.h"
#include "ShapeSpace.h"
#include "Shared/Measures.h"

// include all shapes so that clone() can do type dispatch
#include "LineData.h"
#include "EllipseData.h"
#include "PointData.h"
#include "AgentData.h"
#include "SphereData.h"
#include "PolygonData.h"
#include "BlobData.h"
#include "BrickData.h"
#include "PyramidData.h"
#include "LocalizationParticleData.h"
#include "TargetData.h"
#include "MarkerData.h"
#include "CylinderData.h"
#include "SiftData.h"
#include "AprilTagData.h"
#include "GraphicsData.h"
#include "SkeletonData.h"

using namespace std;

namespace DualCoding {

BaseData::BaseData(ShapeSpace& _space, ShapeType_t _type, int _parentId) :
  space(&_space), name(data_name(_type)), type(_type), 
  id(0), parentId(_parentId), lastMatchId(0),
  refcount(0), viewable(true),
  color_rgb((ProjectInterface::getNumColors() != -1U) ? ProjectInterface::getColorRGB(1) : rgb(0,0,255)), // color 0 is invalid, so use color 1 as default, or blue if colors aren't loaded yet
  confidence(1),
  mobile(false), obstacle(true), landmark(false),
  rendering_sketch(NULL)
{};
    
BaseData::BaseData(const BaseData& other)
  : space(other.space), name(other.name), type(other.type),
    id(0), parentId(other.parentId), lastMatchId(other.lastMatchId),
    refcount(0), viewable(other.viewable),
    color_rgb(other.color_rgb),
    confidence(other.confidence),
    mobile(other.mobile), obstacle(other.obstacle), landmark(other.landmark),
    rendering_sketch(NULL)
{};


BaseData::~BaseData(void) { 
  if ( rendering_sketch != NULL )
    delete rendering_sketch;
}

Shape<PointData> BaseData::getCentroidPtShape() const {
  PointData *pt = new PointData(*space,getCentroid());
  pt->inheritFrom(*this);
  return Shape<PointData>(pt);
}

BaseData& BaseData::operator=(const BaseData& other) {
  // assumes &other =? this check is done by the sub class using BaseData::operator=
  //  if (&other == this)
  //    return *this;

  space = other.space ? &(*other.space) : NULL;
  name = other.name;
  type = other.type;	
  id = other.id;
  parentId = other.parentId;
  lastMatchId = other.lastMatchId;
  refcount = other.refcount;
  viewable = other.viewable;
  color_rgb = other.color_rgb;
  confidence = other.confidence;
  mobile = other.mobile;
  obstacle = other.obstacle;
  landmark = other.landmark;
  rendering_sketch = other.rendering_sketch ? &(*rendering_sketch) : NULL;
  return *this;
}

void BaseData::inheritFrom(const BaseData &parent) {   // used by leftPtShape, etc.
  setParentId(parent.getViewableId());
  setColor(parent.getColor());
}

void BaseData::inheritFrom(const ShapeRoot &parent) {
  setParentId(parent->getViewableId());
  setColor(parent->getColor());
}

void BaseData::inheritFrom(const SketchDataRoot &parent) {
  setParentId(parent.getViewableId());
  setColor(parent.getColor());
}

void BaseData::V(std::string const &_name) {
  setViewable(true);
  if ( !_name.empty() ) setName(_name);
}

void BaseData::N(std::string const &_name) {
  setViewable(false);
  if ( !_name.empty() ) setName(_name);
}

ReferenceFrameType_t BaseData::getRefFrameType() const {
  return space->getRefFrameType(); }


//!Type.
//{
//! Get shape type name.
const char* BaseData::getTypeName() const { return data_name(type); }

//! Test the shape type.
bool BaseData::isType(ShapeType_t this_type) const { return this_type == type; }

//! Test that two shapes are of same type.
bool BaseData::isSameTypeAs(const ShapeRoot& other) const {
	return((bool)(isType(other->type))); }


bool BaseData::isSameColorAs(const ShapeRoot& other) const {
  return getColor() == other->getColor(); }

void BaseData::setColor(const std::string &color_name) {
  setColor(ProjectInterface::getColorRGB(color_name));
}

void BaseData::setColor(const rgb &new_color) {
  color_rgb = new_color;
  if ( rendering_sketch != NULL )
    (*rendering_sketch)->setColor(new_color);
}

void BaseData::setColor(const unsigned int color_index) {
  setColor(ProjectInterface::getColorRGB(color_index));
}


bool BaseData::getMobile() const { return mobile; }

void BaseData::setMobile(bool _mobile) { mobile = _mobile; }

void BaseData::deleteRendering() {
  delete rendering_sketch;
  rendering_sketch = NULL;
}

Sketch<bool>& BaseData::getRendering() {
  if ( rendering_sketch != NULL )
    return *rendering_sketch;
  rendering_sketch = render();
  (*rendering_sketch)->setColor(getColor());
  (*rendering_sketch)->setParentId(id);
  (*rendering_sketch)->setName("render("+getName()+")");
  return *rendering_sketch;
}

BaseData* BaseData::clone() const {
  switch ( type ) {
  case unknownDataType: 
    std::cerr << "Error: tried to clone a BaseData with unknownDataType!" << std::endl;
    return NULL;
    break;
  case lineDataType: return new LineData(*(const LineData*)this);
  case ellipseDataType: return new EllipseData(*(const EllipseData*)this);
  case pointDataType: return new PointData(*(const PointData*)this);
  case agentDataType: return new AgentData(*(const AgentData*)this);
  case sphereDataType: return new SphereData(*(const SphereData*)this);
  case polygonDataType: return new PolygonData(*(const PolygonData*)this);
  case blobDataType: return new BlobData(*(const BlobData*)this);
  case brickDataType: return new BrickData(*(const BrickData*)this);
  case pyramidDataType: return new PyramidData(*(const PyramidData*)this);
  case localizationParticleDataType: return new LocalizationParticleData(*(const LocalizationParticleData*)this);
  case targetDataType: return new TargetData(*(const TargetData*)this);
  case markerDataType: return new MarkerData(*(const MarkerData*)this);
  case cylinderDataType: return new CylinderData(*(const CylinderData*)this);
  case siftDataType: return new SiftData(*(const SiftData*)this);
  case aprilTagDataType: return new AprilTagData(*(const AprilTagData*)this);
  case graphicsDataType: return new GraphicsData(*(const GraphicsData*)this);
  case skeletonDataType: return new SkeletonData(*(const SkeletonData*)this);
  default: return NULL;
  }
}

ShapeRoot BaseData::copy() const { return ShapeRoot(clone()); }

void BaseData::increaseConfidence(int n, int maxConfidence) {
  confidence += n;
  if ( maxConfidence > 0 )
    confidence = std::min(confidence, maxConfidence);
  // std::cout << id << " confidence now " << confidence << std::endl;
}
  
void BaseData::increaseConfidence(const BaseData& other, int maxConfidence) {
  increaseConfidence(other.getConfidence() > 0 ? other.getConfidence()+1 : 2, maxConfidence);
}
  
void BaseData::increaseConfidence(const ShapeRoot& other, int maxConfidence) { 
  increaseConfidence(other.getData(), maxConfidence);
}

void BaseData::decreaseConfidence() {
  confidence--;
  // std::cout << id << " decrease confidence" << std::endl;
}

void BaseData::setPosition(const Point &pt) {
  const Point diff = pt - getCentroid();
	applyTransform(fmat::Transform::offset(diff.getCoords()), getRefFrameType());
}

bool BaseData::isLeftOf(const BaseData& other) {
	if ( getRefFrameType() == camcentric )
		return ( getCentroid().coordX() < other.getCentroid().coordX() );
	else
		return ( getCentroid().coordY() > other.getCentroid().coordY() );
}

bool BaseData::isRightOf(const BaseData& other) {
	if ( getRefFrameType() == camcentric )
		return ( getCentroid().coordX() > other.getCentroid().coordX() );
	else
		return ( getCentroid().coordY() < other.getCentroid().coordY() );
}

bool BaseData::isAbove(const BaseData& other) {
	if ( getRefFrameType() == camcentric )
		return ( getCentroid().coordY() < other.getCentroid().coordY() );
	else
		return ( getCentroid().coordX() > other.getCentroid().coordX() );
}

bool BaseData::isBelow(const BaseData& other) {
	if ( getRefFrameType() == camcentric )
		return ( getCentroid().coordY() > other.getCentroid().coordY() );
	else
		return ( getCentroid().coordX() < other.getCentroid().coordX() );
}

} // namespace
