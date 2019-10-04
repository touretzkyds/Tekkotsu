//-*-c++-*-
#include <iostream>
#include <vector>
#include <map>

#include "Shared/Config.h"
#include "Motion/Kinematics.h"
#include "SketchSpace.h"
#include "Sketch.h"
#include "ShapeSpace.h"
#include "ShapeRoot.h"
#include "MarkerData.h"
#include "ShapeMarker.h"
#include "VRmixin.h"

#include "ShapeFuns.h"
#include "visops.h"
#include "BlobData.h"
#include "ShapeBlob.h"

// only match markers that are less than 5 cm away (this is just the default behavior)
#define MAX_MATCH_DISTANCE 50

using namespace std;

namespace DualCoding {

  // initialize dispatch map
  const MarkerType_t MarkerData::unknownMarkerType = "unknownMarkerType";

  MarkerData::MarkerData(ShapeSpace& _space, const Point& _center, rgb _rgbvalue) :
    BaseData(_space, getStaticType()),
    center(_center), typeOfMarker(unknownMarkerType)
  { setColor(_rgbvalue); }
      
  DATASTUFF_CC(MarkerData);

  //! return the centroid of the shape in point format
  Point MarkerData::getCentroid() const {
    return center;
  }
  
  void MarkerData::printParams() const {
    cout << "Type = " << getTypeName() << "  ID=" << getId() << "  ParentID=" << getParentId() << endl;
    printf("  color = %d %d %d\n",getColor().red,getColor().green,getColor().blue);
    cout << "  center =" << center.coords << endl
	 << endl;
  }

  Sketch<bool>* MarkerData::render() const {
    SketchSpace &SkS = space->getDualSpace();
    Sketch<bool>* result = new Sketch<bool>(SkS, "render("+getName()+")");
    *result = false;

    fmat::Column<3> ctr(center.getCoords());
    SkS.applyTmat(ctr);

    const float &cx = ctr[0];
    const float &cy = ctr[1];

    // pixel radius
    const float rad = 10;
    
    for (int dx = (int)floor(-rad); dx <= (int)ceil(rad); dx+=1) {
      const int yr = (int)sqrt(max((float)0, rad*rad - dx*dx));
      for (int dy = -yr; dy <= yr; dy+=1) {
	int px = int(cx + dx);
	int py = int(cy + dy);

	if ( px >= 0 && px < result->width &&
	     py >= 0 && py < result->height )
	  (*result)(px,py) = true;
      }
    }
    
    return result;
  }

  //! Transformations. (Virtual in BaseData.)
  void MarkerData::applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref) {
    center.applyTransform(Tmat,newref);
  }

   void MarkerData::projectToGround(const fmat::Transform& camToBase,
				   const PlaneEquation& groundplane) {   
    // transform into camera coordinates using known height of the marker
    const float normX = float(2*center.coordX() - VRmixin::camSkS.getWidth()) / VRmixin::camSkS.getWidth();
    const float normY = float(2*center.coordY() - VRmixin::camSkS.getHeight()) / VRmixin::camSkS.getWidth();
    fmat::Column<3> camera_point;
    config->vision.computeRay(normX, normY,
			      camera_point[0],camera_point[1],camera_point[2]);
    // cout << "normX=" << normX << " normY=" << normY << "  camera_point=" << camera_point << endl;
    //normalize and multiply by distance to get actual point
    float denom = fmat::SubVector<3>(camera_point).norm();
    camera_point[0] *= center.coordZ() / denom;
    camera_point[1] *= center.coordZ() / denom;
    camera_point[2] *= center.coordZ() / denom;
    // cout << "denom=" << denom << " camera_point=" << camera_point << endl;
    
    // transform to base (ground) frame
    center.coords = camera_point;
    center.applyTransform(camToBase, egocentric);
    // cout << "Transform yields: " << center.coords << endl;
  }
    
  void MarkerData::update_derived_properties() {}

  bool MarkerData::isMatchFor(const ShapeRoot& other) const {
    if (!isSameTypeAs(other))
      return false;
    else {
      const Shape<MarkerData>& other_marker = ShapeRootTypeConst(other,MarkerData);
    
      if (!isMatchingMarker(other_marker))
	return false;
    
      float dist = center.distanceFrom(other_marker->center);
      return dist < MAX_MATCH_DISTANCE;
    }
  }

  bool MarkerData::updateParams(const ShapeRoot& other, bool forceUpdate) {
    const Shape<MarkerData>& other_marker = ShapeRootTypeConst(other,MarkerData);
    int other_conf = other_marker->confidence;
    if (other_conf <= 0) {
      if (forceUpdate)
	other_conf = 1;
      else
	return false;
    }

    const int sumconf = confidence + other_conf;
    center = (center*confidence + other_marker->center*other_conf) / sumconf;
    confidence += other_conf;

    update_derived_properties();
    return true;
  }

  bool MarkerData::isMatchingMarker(const Shape<MarkerData>& other) const {
    // make sure they are the same marker type
    if (!(typeOfMarker == other->typeOfMarker))
      return false;
  
    // make sure colors are the same
    if (!(getColor() == other->getColor()))
      return false;

    return true;
  }

  string MarkerData::getMarkerDescription() const { return ""; }

  string MarkerData::getMarkerTypeName(MarkerType_t type) { return type; }

  vector<Shape<MarkerData> > MarkerData::extractMarkers(const Sketch<uchar> &sketch,
							MarkerType_t type,
							const MapBuilderRequest &req) {
    MarkerExtractFn_t extractor = getExtractorMap()[type];
    if (extractor == NULL) {
      cout << "Tried to extract markers for unknown marker type '" << type << "'." << endl;
      return vector<Shape<MarkerData> >();
    }
    else
      return (*extractor)(sketch,req);
  }

  vector<Shape<MarkerData> > MarkerData::extractMarkers(const Sketch<uchar> &sketch, const MapBuilderRequest &req) {
    vector<Shape<MarkerData> > markers;
    for (map<MarkerType_t, MarkerExtractFn_t>::const_iterator it = getExtractorMap().begin(); it != getExtractorMap().end(); it++)
      if (it->second != NULL) {
	vector<Shape<MarkerData> > single_type = (*(it->second))(sketch,req);
	markers.insert(markers.end(), single_type.begin(), single_type.end());
      }
  return markers;
  }

  void MarkerData::calculateCameraDistance(Point &p, const float assumedHeight) {
    // taken from blobdata poster code/some mapbuilder code
#ifdef TGT_HAS_CAMERA
    const fmat::Transform camToBase = kine->linkToBase(CameraFrameOffset);
#else
    const fmat::Transform camToBase = fmat::Transform::identity();
#endif
    // create an elevated plane by shifting the ground plane by assumedHeight
    PlaneEquation elevatedPlane = kine->calculateGroundPlane();
    // cout << "calculateCameraDistance: ground plane  = " << elevatedPlane << endl;
    float const new_displacement = elevatedPlane.getDisplacement() + assumedHeight*elevatedPlane.getZsign();
    // cout << "Elevated plane Zsign = " << elevatedPlane.getZsign() << "   new_displacement=" << new_displacement << endl;
    elevatedPlane.setDisplacement(new_displacement);
    // cout << "calculateCameraDistance:  assumedHeight=" << assumedHeight << "  elevatedPlane=" << elevatedPlane << endl;
  
    // Project the point onto the elevated plane
    Point p2 = p;
    p2.projectToGround(camToBase, elevatedPlane);
    // See if the point ended up behind the camera.  Must convert back
    // to the camera reference frame to do this; can't check sign in
    // the base frame because the robot's head could be facing
    // backwards.
    fmat::Column<3> p3 = camToBase.inverse() * p2.coords;
    if ( p3[2] < 0 ) {
      cout << "Warning: point " << p << " is above the horizon; cannot project to ground!" << endl;
      p2.coords = 1e15f;
    }

    // Translate from BaseFrame coordinates back to camera origin.  No need
    // for rotation because we're only going to measure distance from the origin
    fmat::Column<3> a = camToBase.translation() - p2.coords;

    // Set distance from camera as Z component of camera coordinates
    p.setCoords(p.coordX(), p.coordY(), a.norm());
    // cout << "  new p=" << p << endl;
  }

  MarkerType_t MarkerData::registerMarkerType(std::string markerTypeName, MarkerExtractFn_t extractor) {
    if (getExtractorMap()[markerTypeName] != NULL) {
      return "";
    }
    getExtractorMap()[markerTypeName] = extractor;
    return markerTypeName;
  }

  std::map<MarkerType_t, MarkerExtractFn_t>& MarkerData::getExtractorMap() {
    static map<MarkerType_t, MarkerExtractFn_t> extractorMap;
    return extractorMap;
  }

  const std::set<MarkerType_t> MarkerData::allMarkerTypes() {
    std::set<MarkerType_t> result;
    for (map<MarkerType_t, MarkerExtractFn_t>::const_iterator it = getExtractorMap().begin(); it != getExtractorMap().end(); it++)
      result.insert(it->first);
    return result;
  }

} // namespace
