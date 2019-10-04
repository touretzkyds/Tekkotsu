#include "ShapeFuns.h"
#include "AprilTagData.h"
#include "ShapeAprilTag.h"
#include "MarkerData.h"  // for calculateCameraDistance
#include "VRmixin.h"
#include "Crew/MapBuilderRequest.h"  // for defaultMarkerHeight
#include "Shared/Config.h"
#include "Vision/AprilTags/TagDetector.h"

namespace DualCoding {

AprilTagData:: AprilTagData(ShapeSpace& _space, const AprilTags::TagDetection& _tagDetection)
  : BaseData(_space, getStaticType()), tagDetection(_tagDetection),
    center(Point(_tagDetection.cxy.first, _tagDetection.cxy.second, 0, camcentric)),
		q(), bl(getBottomLeftImagePoint()), br(getBottomRightImagePoint()) {}

AprilTagData::AprilTagData(ShapeSpace& _space, const AprilTags::TagDetection& _tagDetection, const Point &_center, const fmat::Quaternion &_q)
  : BaseData(_space, getStaticType()), tagDetection(_tagDetection), center(_center), q(_q), bl(), br() {}

AprilTagData::AprilTagData(const AprilTagData &other)
	: BaseData(other), tagDetection(other.tagDetection), center(other.center), q(other.q),
		bl(other.bl), br(other.br) {}
  
BoundingBox2D AprilTagData::getBoundingBox() const {
  float xmin = min(min(tagDetection.p[0].first,tagDetection.p[1].first),
		   min(tagDetection.p[2].first,tagDetection.p[3].first));
  float xmax = max(max(tagDetection.p[0].first,tagDetection.p[1].first),
		   max(tagDetection.p[2].first,tagDetection.p[3].first));
  float ymin = min(min(tagDetection.p[0].second,tagDetection.p[1].second),
		   min(tagDetection.p[2].second,tagDetection.p[3].second));
  float ymax = max(max(tagDetection.p[0].second,tagDetection.p[1].second),
		   max(tagDetection.p[2].second,tagDetection.p[3].second));
  return BoundingBox2D(xmin,ymin,xmax,ymax);
}

Point AprilTagData::getTopLeft() const { return Point(tagDetection.p[3].first, tagDetection.p[3].second, 0, camcentric); }

Point AprilTagData::getTopRight() const { return Point(tagDetection.p[2].first, tagDetection.p[2].second, 0, camcentric); }

Point AprilTagData::getBottomLeft() const { return Point(tagDetection.p[0].first, tagDetection.p[0].second, 0, camcentric); }

Point AprilTagData::getBottomRight() const { return Point(tagDetection.p[1].first, tagDetection.p[1].second, 0, camcentric); }

bool AprilTagData::isMatchFor(const ShapeRoot& other) const {
  if ( ! isSameTypeAs(other) ) return false;
  const Shape<AprilTagData>& other_AprilTag = ShapeRootTypeConst(other,AprilTagData);
  if (getTagID() != other_AprilTag->getTagID()) return false;
  const float distance = getCentroid().distanceFrom(other_AprilTag->getCentroid());
	// disable orientation checking because we're not converting from camera to local/world
  const AngSignPi angdiff = 0; // *** getOrientation() - other_AprilTag->getOrientation();
  bool result = (distance < 200 && fabs(angdiff) < M_PI/6);
  return result;
}

bool AprilTagData::updateParams(const ShapeRoot& other, bool force) {
  // std::cout << "Merging AprilTags " << getId() << " and " << other->getId() << std::endl;
  center = (center + other->getCentroid()) / 2;
  return true;
}

void AprilTagData::printParams() const {
  std::cout << "Type = " << getTypeName() << "  ID=" << getId() << "  ParentID=" << getParentId() << std::endl;
}

void AprilTagData::applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref) {
  center.applyTransform(Tmat,newref);
  bl.applyTransform(Tmat,newref);
  br.applyTransform(Tmat,newref);
	float orient = float((br-bl).atanYX()) - M_PI/2;
	q = fmat::Quaternion::aboutZ(orient);
}

fmat::Quaternion AprilTagData::getQuaternion() const { return q; }

void AprilTagData::projectToGround(const fmat::Transform& camToBase,
																	 const PlaneEquation& groundplane) {
  // Transform into local coordinates using known distance from the
  // camera, which was computed by the MapBuilder based on known height
  // of the marker and placed in center.coordZ().
  const float xres = VRmixin::camSkS.getWidth();
  const float yres = VRmixin::camSkS.getHeight();
  const float maxres = std::max(xres,yres);
  const float normX = float(2*center.coordX() - xres) / maxres;
  const float normY = float(2*center.coordY() - yres) / maxres;
	const float distance = center.coordZ();
  fmat::Column<3> camera_vector;
  config->vision.computeRay(normX, normY,
			    camera_vector[0],camera_vector[1],camera_vector[2]);
  // normalize and multiply by distance from camera to get actual point in space
  fmat::Column<3> camera_point = camera_vector * (distance / camera_vector.norm());
  // transform to base (ground) frame
  center.coords = camera_point;
  center.applyTransform(camToBase, egocentric);
  // cout << " camera_point=" << camera_point << "   new center=" << center << "  camToBase=" << camToBase << endl;

  coordinate_t markerHeight = MapBuilderRequest::defaultMarkerHeight;
	MarkerData::calculateCameraDistance(bl, markerHeight);
	MarkerData::calculateCameraDistance(br, markerHeight);


	const float normX0 = (2*bl.coordX() - xres) / maxres;
	const float normY0 = (2*bl.coordY() - yres) / maxres;
  config->vision.computeRay(normX0, normY0,
			    camera_vector[0],camera_vector[1],camera_vector[2]);
	camera_point = camera_vector * (bl.coordZ() / camera_vector.norm());
	bl.coords = camera_point;
	bl.applyTransform(camToBase, egocentric);

	const float normX1 = (2*br.coordX() - xres) / maxres;
	const float normY1 = (2*br.coordY() - yres) / maxres;
  config->vision.computeRay(normX1, normY1,
			    camera_vector[0],camera_vector[1],camera_vector[2]);
	camera_point = camera_vector * (br.coordZ() / camera_vector.norm());
	br.coords = camera_point;
	br.applyTransform(camToBase, egocentric);

	AngTwoPi orient = AngTwoPi(float((br-bl).atanYX()) - M_PI/2);
	q = fmat::Quaternion::aboutZ(orient);
	//std::cout << "*** Tag " << getTagID() << " bl=" << bl << " br=" << br
	//					<< " orient = " << float(orient)*180/M_PI << std::endl;
	//fmat::Column<3> yprval = q.ypr();
	//std::cout << "   yaw=" << yprval[0] << " pitch=" << yprval[1] << " roll=" << yprval[2] << std::endl;
}

Sketch<bool>* AprilTagData::render() const {
  SketchSpace &SkS = space->getDualSpace();
  Sketch<bool> result(SkS, "render("+getName()+")");
  result = 0;   // *** incomplete rendering ***
  return new Sketch<bool>(result);
}

std::vector<Shape<AprilTagData> > AprilTagData::extractAprilTags(const Sketch<uchar> &rawY, const AprilTags::TagFamily &tagFamily) {
  AprilTags::TagDetector detector(tagFamily);
  std::vector<AprilTags::TagDetection> tags = detector.extractTags(rawY);
  std::vector<Shape<AprilTagData> > result;
  for ( std::vector<AprilTags::TagDetection>::const_iterator it = tags.begin();
				it != tags.end(); it++ ) {
    NEW_SHAPE(apriltag, AprilTagData, new AprilTagData(rawY->getSpace().getDualSpace(), *it));
    result.push_back(apriltag);
  }
  return result;
}

Shape<AprilTagData> AprilTagData::findTag(const std::vector<ShapeRoot> &shapevec, int tag) {
  for ( std::vector<ShapeRoot>::const_iterator it = shapevec.begin();
				it != shapevec.end(); it++ )
		if ( (*it)->isType(aprilTagDataType) ) {
      const Shape<AprilTagData> &t = ShapeRootTypeConst(*it,AprilTagData);
      if ( t->getTagID() == tag )
        return t;
    }
  return Shape<AprilTagData>();
}
  
bool AprilTagData::TagIDLessThan::operator() (const Shape<AprilTagData> &tag1, const Shape<AprilTagData> &tag2) const {
	return (tag1->getTagID() < tag2->getTagID());
}

Point AprilTagData::getBottomLeftImagePoint() const {
	float centerX = tagDetection.cxy.first;
	float centerY = tagDetection.cxy.second;
	for ( int i = 0; i < 4; i++ )
		if ( tagDetection.p[i].first < centerX )
			if ( tagDetection.p[i].second > centerY )
				return Point(tagDetection.p[i].first, tagDetection.p[i].second);
	return Point(); // should never get here
}

Point AprilTagData::getBottomRightImagePoint() const {
	float centerX = tagDetection.cxy.first;
	float centerY = tagDetection.cxy.second;
	for ( int i = 0; i < 4; i++ )
		if ( tagDetection.p[i].first > centerX )
			if ( tagDetection.p[i].second > centerY )
				return Point(tagDetection.p[i].first, tagDetection.p[i].second);
	return Point(); // should never get here
}

DATASTUFF_CC(AprilTagData);

} // namespace
