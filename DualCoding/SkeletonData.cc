#include "Shared/Config.h"

//#include "ShapeFuns.h"
#include "SkeletonData.h"
#include "ShapeSkeleton.h"
#include "VRmixin.h"

namespace DualCoding {

SkeletonData:: SkeletonData(ShapeSpace& _space, const Skeleton& _skeleton)
  : BaseData(_space, getStaticType()), skeleton(_skeleton),
    center(Point()) {}

SkeletonData::SkeletonData(ShapeSpace& _space, const Point &_center, const Skeleton& _skeleton)
  : BaseData(_space, getStaticType()), skeleton(_skeleton), center(_center) {}

SkeletonData::SkeletonData(const SkeletonData &other)
  : BaseData(other), skeleton(other.skeleton), center(other.center) {}
  
BoundingBox2D SkeletonData::getBoundingBox() const {
  return BoundingBox2D(center.coords); // *** incomplete ***
}

bool SkeletonData::isMatchFor(const ShapeRoot& other) const {
  if ( ! isSameTypeAs(other) ) return false;
  const Shape<SkeletonData>& other_Skeleton = ShapeRootTypeConst(other,SkeletonData);
  const float distance = getCentroid().distanceFrom(other_Skeleton->getCentroid());
  bool result = (distance < 200);
  return result;
}

bool SkeletonData::updateParams(const ShapeRoot& other, bool force) {
  // std::cout << "Merging Skeletons " << getId() << " and " << other->getId() << std::endl;
  center = (center + other->getCentroid()) / 2;
  return true;
}

void SkeletonData::printParams() const {
  std::cout << "Type = " << getTypeName() << "  ID=" << getId() << "  ParentID=" << getParentId() << std::endl;
}

void SkeletonData::applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref) {
  center.applyTransform(Tmat,newref);
}

void SkeletonData::projectToGround(const fmat::Transform& camToBase,
				   const PlaneEquation& groundplane) {
  // Transform into local coordinates using known distance from the
  // camera, which was computed earlier based on known height of the
  // marker.
  const float xres = VRmixin::camSkS.getWidth();
  const float yres = VRmixin::camSkS.getHeight();
  const float maxres = std::max(xres,yres);
  const float normX = float(2*center.coordX() - xres) / maxres;
  const float normY = float(2*center.coordY() - yres) / maxres;
  fmat::Column<3> camera_vector;
  config->vision.computeRay(normX, normY,
			    camera_vector[0],camera_vector[1],camera_vector[2]);
  //  cout << "center=" << center << "  camera_vector=" << camera_vector << "  norm=" << camera_vector.norm() << endl;

  // normalize and multiply by distance from camera to get actual point in space
  fmat::Column<3> camera_point = camera_vector * (center.coordZ() / camera_vector.norm());

  // transform to base (ground) frame
  center.coords = camera_point;
  center.applyTransform(camToBase, egocentric);
  // cout << " camera_point=" << camera_point << "   new center=" << center << "  camToBase=" << camToBase << endl;
}

Sketch<bool>* SkeletonData::render() const {
  SketchSpace &SkS = space->getDualSpace();
  Sketch<bool> result(SkS, "render("+getName()+")");
  result = 0;   // *** incomplete rendering ***
  return new Sketch<bool>(result);
}
  
DATASTUFF_CC(SkeletonData);

} // namespace
