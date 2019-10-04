//-*-c++-*-

#include "Shared/ProjectInterface.h"
#include "Vision/SegmentedColorGenerator.h"
#include "Vision/AprilTags/TagFamily.h"

#include "Crew/MapBuilderRequest.h"
#include "Crew/MapBuilder.h"

#include "DualCoding/LineData.h"
#include "DualCoding/ShapeBlob.h"
#include "DualCoding/ShapeCylinder.h"
#include "DualCoding/ShapeMarker.h"
#include "DualCoding/ShapeDomino.h"
#include "DualCoding/VRmixin.h"

using namespace std;

namespace DualCoding {

MapBuilderRequest::MapBuilderRequest(MapBuilderRequestType_t reqtype)
		: requestType(reqtype), objectColors(), secondColors(), occluderColors(), 
      extractorMinLineLength(std::min(VRmixin::camSkS.getHeight(),VRmixin::camSkS.getWidth()) *
                             LineData::MIN_FRACTIONAL_LENGTH),
      extractorGapTolerance(15),
			minLineLength(70), minRayLength(40), minLinesPerpDist(100),
      minEllipseSemiMajor(9),
		  minBlobAreas(), blobOrientations(), assumedBlobHeights(), 
		  assumedCylinderHeights(), markerTypes(), markerHeights(), floorColor(0),
		  siftDatabasePath(""), siftObjectNames(), aprilTagFamily(),
      dominoDimensions(fmat::pack(200.0, 89.0, 67.0)),
      naughtDimensions(fmat::pack(90.0, 90.0, 57.0)),
			crossDimensions(fmat::pack(100.0, 68.0, 41.0)),
		  motionSettleTime(1000), numSamples(1), sampleInterval(0), maxDist(1e10f), 
		  clearCamera(true), clearLocal(true), clearWorld(false), ignoreCamera(false),
		  pursueShapes(false), immediateRequest(false), manualHeadMotion(false), rawY(true), removePts(true), 
		  doScan(false), dTheta((float)M_PI/18),
		  searchArea(), worldTargets(),
		  userImageProcessing(NULL), userCamProcessing(NULL), userGroundProcessing(NULL), 
		  userLocalProcessing(NULL), userWorldProcessing(NULL),
		  exitTest(NULL),
		  groundPlaneAssumption(onLegs), customGroundPlane(), requestingBehavior(NULL),
		  setVerbosity(0), clearVerbosity(0),
		  baseTransform(),
		  gazePts(), baseToCamMats(),
		  requestID(0),
		  verbosity(0) // MapBuilder will set this during execution
  {}

MapBuilderRequest::MapBuilderRequest(const MapBuilderRequest &req)
  : requestType(req.requestType),
    objectColors(req.objectColors), secondColors(req.secondColors), occluderColors(req.occluderColors),
    extractorMinLineLength(req.extractorMinLineLength),
    extractorGapTolerance(req.extractorGapTolerance),    
    minLineLength(req.minLineLength), minRayLength(req.minRayLength),
    minLinesPerpDist(req.minLinesPerpDist),
    minEllipseSemiMajor(req.minEllipseSemiMajor),
    minBlobAreas(req.minBlobAreas), blobOrientations(req.blobOrientations),
    assumedBlobHeights(req.assumedBlobHeights), 
    assumedCylinderHeights(req.assumedCylinderHeights),
    markerTypes(req.markerTypes), markerHeights(req.markerHeights),
    floorColor(req.floorColor),
    siftDatabasePath(req.siftDatabasePath), siftObjectNames(req.siftObjectNames),
    aprilTagFamily(req.aprilTagFamily),
    dominoDimensions(req.dominoDimensions),
    naughtDimensions(req.naughtDimensions),
    crossDimensions(req.crossDimensions),
    motionSettleTime(req.motionSettleTime),
    numSamples(req.numSamples), sampleInterval(req.sampleInterval),
    maxDist(req.maxDist),
    clearCamera(req.clearCamera), clearLocal(req.clearLocal), clearWorld(req.clearWorld),
    ignoreCamera(req.ignoreCamera), pursueShapes(req.pursueShapes),
    immediateRequest(req.immediateRequest),
    manualHeadMotion(req.manualHeadMotion),
    rawY(req.rawY), removePts(req.removePts), 
    doScan(req.doScan), dTheta(req.dTheta),
    searchArea(req.searchArea), worldTargets(req.worldTargets),
    userImageProcessing(req.userImageProcessing), 
    userCamProcessing(req.userCamProcessing), 
    userGroundProcessing(req.userGroundProcessing), 
    userLocalProcessing(req.userLocalProcessing), 
    userWorldProcessing(req.userWorldProcessing),
    exitTest(req.exitTest),
    groundPlaneAssumption(req.groundPlaneAssumption),
    customGroundPlane(req.customGroundPlane),
    requestingBehavior(req.requestingBehavior),
    setVerbosity(req.setVerbosity), clearVerbosity(req.clearVerbosity),
    baseTransform(req.baseTransform),
    gazePts(req.gazePts), baseToCamMats(req.baseToCamMats),
    requestID(req.requestID), verbosity(req.verbosity)
{}

MapBuilderRequest& MapBuilderRequest::operator=(const MapBuilderRequest &req) {
    requestType = req.requestType;
    objectColors = req.objectColors;
		secondColors = req.secondColors;
    occluderColors = req.occluderColors;
    extractorMinLineLength = req.extractorMinLineLength;
    extractorGapTolerance = req.extractorGapTolerance;
		minLineLength = req.minLineLength;
		minRayLength = req.minRayLength;
    minLinesPerpDist = req.minLinesPerpDist;
		minEllipseSemiMajor = req.minEllipseSemiMajor;
    minBlobAreas = req.minBlobAreas;
    blobOrientations = req.blobOrientations;
    assumedBlobHeights = req.assumedBlobHeights;
    markerTypes = req.markerTypes;
    markerHeights = req.markerHeights;
    floorColor = req.floorColor;
    siftDatabasePath = req.siftDatabasePath;
    siftObjectNames = req.siftObjectNames;
    aprilTagFamily = req.aprilTagFamily;
		dominoDimensions = req.dominoDimensions;
    naughtDimensions = req.naughtDimensions;
		crossDimensions = req.crossDimensions;
    motionSettleTime = req.motionSettleTime;
    numSamples = req.numSamples;
    sampleInterval = req.sampleInterval;
    maxDist = req.maxDist;
    clearCamera = req.clearCamera;
    clearLocal = req.clearLocal;
    clearWorld = req.clearWorld;
		ignoreCamera = req.ignoreCamera;
    pursueShapes = req.pursueShapes;
    immediateRequest = req.immediateRequest;
    manualHeadMotion = req.manualHeadMotion;
    rawY = req.rawY;
    removePts = req.removePts;
    doScan = req.doScan;
    dTheta = req.dTheta;
    searchArea = req.searchArea;
    worldTargets = req.worldTargets;
    userImageProcessing = req.userImageProcessing;
    userCamProcessing = req.userCamProcessing;
    userGroundProcessing = req.userGroundProcessing;
    userLocalProcessing = req.userLocalProcessing;
    userWorldProcessing = req.userWorldProcessing;
    exitTest = req.exitTest;
    groundPlaneAssumption = req.groundPlaneAssumption;
    customGroundPlane = req.customGroundPlane;
    requestingBehavior = req.requestingBehavior;
    setVerbosity = req.setVerbosity;
    clearVerbosity = req.clearVerbosity;
    baseTransform = req.baseTransform;
    gazePts = req.gazePts;
    baseToCamMats = req.baseToCamMats;
    requestID = req.requestID;
    verbosity = req.verbosity;
    return *this;
  }

coordinate_t MapBuilderRequest::defaultMarkerHeight = 7.0f * 25.4f;  // 7 inches converted to millimeters

bool MapBuilderRequest::validateRequest() {
  return validateColors(objectColors) & validateColors(occluderColors) & validateColors(secondColors) &
		validateSift() & validateRequestType();
}

bool MapBuilderRequest::validateAdd(ShapeType_t type, std::string const &color_name) {
  if ( type >= numDataTypes ) {
    cerr << "*** Warning: MapBuilderRequest::add(...) called with invalid shape type " << type << endl;
    return false;
  }
  const color_index c = ProjectInterface::getColorIndex(color_name);
  if ( c == -1U) {
    cerr << "*** Warning: MapBuilderRequest::add(...) called with invalid color name '" << color_name << "'" << endl;
    return false;
  }
  return true;
}  

bool MapBuilderRequest:: validateColors(const map<ShapeType_t,set<color_index> > &shapes2colors) {
  color_index const numcolors = ProjectInterface::getNumColors();
  bool valid = true;
  for ( map<ShapeType_t,set<color_index> >::const_iterator shape_it = shapes2colors.begin();
	shape_it != shapes2colors.end(); shape_it++ ) {
    if ( shape_it->first == 0 || shape_it->first >= numDataTypes ) {
      cerr << "*** Warning: Mapbuilder request contains invalid shape type: " << shape_it->first << endl;
      valid = false;
    }
    for ( set<color_index>::const_iterator color_it = shape_it->second.begin();
	  color_it != shape_it->second.end(); color_it++ )
      if ( *color_it > numcolors ) {
	valid = false;
	cerr << "*** Warning: MapBuilder request involving " << data_name(shape_it->first)
	     << " contains invalid color index " << *color_it << endl;
      }
  }
  return valid;
}

bool MapBuilderRequest::validateSift() {
  return siftObjectNames.empty() || !siftDatabasePath.empty();
}

bool MapBuilderRequest:: validateRequestType() {
  if ( (pursueShapes || doScan || worldTargets.size()>0) &&
       getRequestType() == MapBuilderRequest::cameraMap ) {
    cout << "Warning: switching MapBuilderRequest type from cameraMap to localMap because request parameters require head movement." << endl;
    requestType = MapBuilderRequest::localMap;
  }
  if ( immediateRequest )
    if ( pursueShapes || doScan || searchArea.isValid() || worldTargets.size() > 0 || numSamples > 1 ) {
      cout << "Warning: MapBuilderRequest setting immedateRequest = false because of conflict with pursueShapes, doScan, searchArea, worldTargets, or numSamples" << endl;
      immediateRequest = false;
    }
  return true;
}

std::set<color_index> MapBuilderRequest::allColors() {
  std::set<color_index> result;
  for (color_index i = 1; i < ProjectInterface::defSegmentedColorGenerator->getNumColors(); i++ )
    if ( strcmp(ProjectInterface::getColorName(i), "black") != 0 )
      result.insert(i);
  return result;
}

void MapBuilderRequest::addObjectColor(ShapeType_t type, std::string const &color_name) {
  if ( validateAdd(type,color_name) )
    objectColors[type].insert(ProjectInterface::getColorIndex(color_name));
}

void MapBuilderRequest::addObjectColor(ShapeType_t type, rgb color) {
  objectColors[type].insert(ProjectInterface::getColorIndex(color));
}

void MapBuilderRequest::addSecondColor(ShapeType_t type, std::string const &color_name) {
  if ( validateAdd(type,color_name) )
    secondColors[type].insert(ProjectInterface::getColorIndex(color_name));
}

void MapBuilderRequest::addSecondColor(ShapeType_t type, rgb color) {
  secondColors[type].insert(ProjectInterface::getColorIndex(color));
}

void MapBuilderRequest::addAllObjectColors(ShapeType_t type) {
  objectColors[type] = allColors();
}

void MapBuilderRequest::addOccluderColor(ShapeType_t type, std::string const &color_name) {
  if ( validateAdd(type,color_name) )
    occluderColors[type].insert(ProjectInterface::getColorIndex(color_name));
}
 
void MapBuilderRequest::addMinBlobArea(std::string const &color_name, int area) {
  if ( validateAdd(blobDataType,color_name) )
    minBlobAreas[ProjectInterface::getColorIndex(color_name)] = area;
}

void MapBuilderRequest::addAllMinBlobAreas(int area) {
  for (color_index i = 1; i < ProjectInterface::defSegmentedColorGenerator->getNumColors(); i++ )
    minBlobAreas[i] = area;
}

void MapBuilderRequest::addBlobOrientation(std::string const &color_name, BlobData::BlobOrientation_t orient, coordinate_t assumedHeight){
  blobOrientations[ProjectInterface::getColorIndex(color_name)] = orient;
  assumedBlobHeights[ProjectInterface::getColorIndex(color_name)] =  assumedHeight;
}

void MapBuilderRequest::addBlobOrientation(rgb color, BlobData::BlobOrientation_t orient, coordinate_t assumedHeight){
  blobOrientations[ProjectInterface::getColorIndex(color)] = orient;
  assumedBlobHeights[ProjectInterface::getColorIndex(color)] =  assumedHeight;
}
 
void MapBuilderRequest::addCylinderHeight(std::string const &color_name, coordinate_t assumedHeight){
  assumedCylinderHeights[ProjectInterface::getColorIndex(color_name)] =  assumedHeight;
}

  void MapBuilderRequest::addCylinderHeight(rgb color, coordinate_t assumedHeight){
  assumedCylinderHeights[ProjectInterface::getColorIndex(color)] =  assumedHeight;
}

void MapBuilderRequest::addMarkerType(MarkerType_t type) {
  markerTypes.insert(type);
}

void MapBuilderRequest::addSiftObject(string const &name) {
  siftObjectNames.insert(name);
}

void MapBuilderRequest::addAttributes(const ShapeRoot &shape) {
  ShapeType_t type = shape->getType();
  rgb color = shape->getColor();
  color_index cindex = ProjectInterface::getColorIndex(color);
  addObjectColor(type,color);
  switch ( type ) {
  case blobDataType: {
    const Shape<BlobData> &blob = ShapeRootTypeConst(shape,BlobData);
    blobOrientations[cindex] = blob->orientation;
    assumedBlobHeights[cindex] = blob->assumedHeight;
    break;}
  case markerDataType:
    addMarkerType(ShapeRootTypeConst(shape,MarkerData)->getMarkerType());
    break;
  case cylinderDataType:
    assumedCylinderHeights[cindex] = ShapeRootTypeConst(shape,CylinderData)->getHeight();
    break;
  case aprilTagDataType:
    setAprilTagFamily();  // *** should really check the family of the tag itself
    break;
	case dominoDataType:
		addSecondColor(type, ShapeRootTypeConst(shape,DominoData)->getLineColor());
  default:
    break;
  }
}

void MapBuilderRequest::setAprilTagFamily() {
  int const notFound = 100000;
  int bits = notFound, minimumHammingDistance;
  for ( std::map<std::pair<int,int>,AprilTags::TagFamily*>::const_iterator it = AprilTags::TagFamily::tagFamilyRegistry.begin();
	it != AprilTags::TagFamily::tagFamilyRegistry.end(); it++ )
    if ( it->first.first < bits ) {
      bits = it->first.first;
      minimumHammingDistance = it->first.second;
    }
  if ( bits == notFound ) {
    cerr << "*** Warning: MapBuilderRequest::setAprilTagFamily called but there are no tag families available!" << endl;
    return;
  }
  aprilTagFamily = std::pair<int,int>(bits,minimumHammingDistance);
}

void MapBuilderRequest::setAprilTagFamily(int bits, int minimumHammingDistance) {
  aprilTagFamily = std::pair<int,int>(bits,minimumHammingDistance);
  if ( AprilTags::TagFamily::tagFamilyRegistry[aprilTagFamily] == NULL )
    cerr << "*** Error: MapBuilderRequest::setAprilTagFamily cannot find requested "
	 << bits << "h" << minimumHammingDistance << " tag family!" << endl;
}

void MapBuilderRequest::addAgentRecognition() {
	addObjectColor(agentDataType, "red");
}


} // namespace
