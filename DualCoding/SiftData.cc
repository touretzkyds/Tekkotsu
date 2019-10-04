#include "SiftData.h"
#include "ShapeFuns.h"
#include "ShapePoint.h"
#include "ShapeSift.h"
#include "Vision/SIFT/SIFTDatabase/keypoint.h"

namespace DualCoding {

SiftData:: SiftData(ShapeSpace& _space, SiftMatch* _match)
  : BaseData(_space, getStaticType()), center_pt(), match(_match) {
  if ( match )
    center_pt = (match->topLeft + match->topRight + match->bottomLeft + match->bottomRight) / 4;
}

SiftData::SiftData(const SiftData &other)
  : BaseData(other), center_pt(other.center_pt), 
    match(new SiftMatch(*other.match)) // must copy SiftMatch structure because destrutor will delete it
{}
  
SiftData::~SiftData() {
  delete match;
}

BoundingBox2D SiftData::getBoundingBox() const {
  return BoundingBox2D(center_pt.coords); // *** incomplete ***
}

bool SiftData::isMatchFor(const ShapeRoot& other) const {
  if ( ! isSameTypeAs(other) ) return false;
  const Shape<SiftData>& other_sift = ShapeRootTypeConst(other,SiftData);
  return (getObjectID() == other_sift->getObjectID());
}

bool SiftData::updateParams(const ShapeRoot& other, bool force) {
  // *** incomplete ***
  return true;
}

void SiftData::printParams() const {
  std::cout << "Type = " << getTypeName() << "  ID=" << getId() << "  ParentID=" << getParentId() << std::endl;
  match->print("  ");
}

void SiftData::applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref) {
  // *** incomplete definition ***
  std::cerr << "Don't know how to apply a transform to SiftData object!\n";
}

void SiftData::projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane) {
  // *** incomplete definition ***
  std::cerr << "Don't know how to project-to-ground a SiftData object!\n";
}

Sketch<bool>* SiftData::render() const {
  SketchSpace &SkS = space->getDualSpace();
  Sketch<bool> result(SkS, "render("+getName()+")");
  result = 0;   // *** incomplete rendering ***
  return new Sketch<bool>(result);
}

void SiftData::displayMatchedFeatures() {
  for ( std::vector<keypointPair>::const_iterator it = match->inliers.begin(); it != match->inliers.end(); it++ ) {
    NEW_SHAPE(feature, PointData, new PointData(*space, Point(it->getKey1()->modelX, it->getKey1()->modelY, 0, camcentric)));
    feature->setParentId(getId());
    feature->setColor(rgb(0,255,0));
  }
}
  
DATASTUFF_CC(SiftData);

} // namespace
