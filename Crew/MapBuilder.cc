//-*-c++-*-
#include "Events/EventRouter.h"
#include "Events/TextMsgEvent.h"
#include "Events/LookoutEvents.h"
#include "Motion/HeadPointerMC.h"
#include "Motion/Kinematics.h"
#include "Shared/mathutils.h"
#include "Shared/Measures.h"
#include "Shared/Config.h"
#include "Shared/MarkScope.h"

#include "DualCoding/ShapeRoot.h"
#include "DualCoding/ShapeLine.h"
#include "DualCoding/ShapeEllipse.h"
#include "DualCoding/ShapeBlob.h"
#include "DualCoding/ShapeCylinder.h"
#include "DualCoding/ShapePolygon.h"
#include "DualCoding/ShapeSphere.h"
#include "DualCoding/ShapeTarget.h"
#include "DualCoding/ShapeMarker.h"
#include "DualCoding/ShapeSift.h"
#include "DualCoding/ShapeAprilTag.h"
#include "DualCoding/ShapeDomino.h"
#include "DualCoding/ShapeNaught.h"
#include "DualCoding/ShapeCross.h"
#include "DualCoding/Sketch.h"    // for NEW_SKETCH
#include "DualCoding/visops.h"
#include "DualCoding/VRmixin.h"

#include "Crew/LookoutRequests.h"
#include "Crew/Lookout.h"
#include "Crew/MapBuilder.h"

#include "Vision/SIFT/API/SiftTekkotsu.h"
#include "Vision/AprilTags/TagDetector.h"
#include "Vision/AprilTags/TagDetection.h"

using namespace std;

namespace DualCoding {

inline float distSq(const fmat::Column<4>& vec) {
  return vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2];
}

MapBuilder::MapBuilder() : 
  BehaviorBase("MapBuilder"),
  camSkS(VRmixin::getCamSkS()), camShS(camSkS.getDualSpace()),
  groundShS(VRmixin::getGroundShS()),
  localSkS(VRmixin::getLocalSkS()), localShS(localSkS.getDualSpace()),
  worldSkS(VRmixin::getWorldSkS()), worldShS(worldSkS.getDualSpace()),
  xres(camSkS.getWidth()), yres(camSkS.getHeight()), 
  ground_plane(),
  theAgent(VRmixin::theAgent),
  localToWorldMatrix(fmat::Transform::identity()),
  worldToLocalMatrix(fmat::Transform::identity()), 
  badGazePoints(), 
  requests(), curReq(NULL), idCounter(0), maxDistSq(0), minLinesPerpDistSave(0), siftMatchers(),
  pointAtID(Lookout::invalid_LO_ID), scanID(Lookout::invalid_LO_ID),
  nextGazePoint() {}

void MapBuilder::preStart() {
  BehaviorBase::preStart();
  if ( verbosity & MBVstart )
    cout << "MapBuilder::start()\n";

  camSkS.clear(); camShS.clear();
  groundShS.clear();
  localSkS.clear(); localShS.clear();
  worldSkS.clear(); worldShS.clear();
  badGazePoints.clear();

  erouter->addListener(this,EventBase::textmsgEGID);  // listen for commands to move to next gaze point
  erouter->addListener(this,EventBase::lookoutEGID);  // listen for Lookout completion events
}

bool MapBuilder::retain = true;

MapBuilder::MapBuilderVerbosity_t MapBuilder::verbosity =
	(-1U & ~(MBVevents | MBVcomplete
#ifdef TGT_HAS_WHEELS
					 | MBVgroundPlane
#endif
					 | MBVgazePointQueue | MBVskipShape));

/*
  Since MapBuilder is constructed as static from VRmixin, the
  destructor doesn't get called until the robot shuts down.  We have
  to do everything assumed to be done in destructor in doStop, such as
  clearing the request queue and setting parameters to the initial
  values as set in the constructor.
*/
void MapBuilder::stop() {
  cout << "MapBuilder::stop()\n";
  while(!requests.empty()) {
    delete requests.front();
    requests.pop();
  }
  curReq = NULL;
  BehaviorBase::stop();
}

void MapBuilder::executeRequest(BehaviorBase *requestingBehavior, const MapBuilderRequest &req) {
  MapBuilderRequest newreq(req);
  newreq.requestingBehavior = requestingBehavior;
  executeRequest(newreq);
}

unsigned int MapBuilder::executeRequest(const MapBuilderRequest& req, unsigned int *req_id) {
	VRmixin::requireCrew("MapBuilder");
  MapBuilderRequest *newreq = new MapBuilderRequest(req);
  if ( !newreq->validateRequest() ) {
    cout << "*** Request rejected by MapBuilder" << endl;
    return 0;
  }
  const unsigned int this_request  = ++idCounter;
  newreq->requestID = this_request;
  if ( req_id != NULL ) *req_id = this_request;
  requests.push(newreq);
  executeRequest();
  return this_request;
}

void MapBuilder::executeRequest() {
  if ( curReq != NULL || requests.empty() ) return;
  curReq = requests.front();
  curReq->verbosity = (verbosity & ~curReq->clearVerbosity) | curReq->setVerbosity;
  if ( curReq->verbosity & MBVexecute )
    cout << "MapBuilder::executeRequest: execute " << curReq->requestID << endl;
  erouter->postEvent(EventBase::mapbuilderEGID, curReq->requestID, EventBase::activateETID,0);  
  minLinesPerpDistSave = LineData::minLinesPerpDist;
  LineData::minLinesPerpDist = curReq->minLinesPerpDist;

	calculateGroundPlane();
	maxDistSq = curReq->maxDist*curReq->maxDist;
	badGazePoints.clear();
 
	if ( curReq->clearCamera && ! curReq->ignoreCamera) {
		camShS.clear();
		camSkS.clear();
		curReq->gazePts.clear();
		curReq->baseToCamMats.clear();
	}
	if ( curReq->clearLocal ) {
		localShS.clear();
		localSkS.clear();
	}
	if ( curReq->clearWorld ) {
		worldShS.clear();
		worldSkS.clear();
	}

	if ( curReq->immediateRequest )
		grabCameraImageAndGo();
	else if ( ! curReq->searchArea.isValid() && curReq->worldTargets.size() == 0 )
		storeImage(false);
	else {
		setInitialGazePts();
		if ( curReq->doScan == true )
			return; // wait for Lookout to finish scanning
		else if ( curReq->worldTargets.size() > 0 )
			doNextSearch();
		else if ( determineNextGazePoint() )
			moveToNextGazePoint();
		else
			requestComplete();
	}
}
  
//================================================================

/*
	There are three kinds of events for MapBuilder::doEvent to handle:

	1. TextMsg event saying to move to the next gaze point; only needed
	when we've asserted manual control of gaze point sequencing for
	debugging.

	2. Lookout event announcing that a "point at" request is complete and
	an image has been stored.  We can now go ahead and process that image.

	3. Lookout event announcing that a "scan" request is complete and gaze
	points have been stored.  We can now start examining those gaze points.
*/

void MapBuilder::doEvent() {
  if ( curReq == NULL) return;
  if ( curReq->verbosity & MBVevents )
    cout << "MapBuilder got event " << event->getName() << endl;

  switch ( event->getGeneratorID() ) {
  case EventBase::textmsgEGID:
    if ( strcmp(dynamic_cast<const TextMsgEvent&>(*event).getText().c_str(),"MoveHead") == 0 )
      moveToNextGazePoint(true);
    break;

  case EventBase::lookoutEGID:
    if ( event->getSourceID() == pointAtID )
      processImage(dynamic_cast<const LookoutSketchEvent&>(*event));
    else if ( event->getSourceID() == scanID ) {
      const vector<Point>& pts = dynamic_cast<const LookoutScanEvent*>(event)->getTasks().front()->data;
      cout << " doScan found " << pts.size() << " interest points." << endl;
			for (vector<Point>::const_iterator it = pts.begin(); it != pts.end(); it++)
				curReq->gazePts.push_back(GazePoint(GazePoint::visible, *it));
    }
    else {
      cout << "MapBuilder: unexpected Lookout event " << event->getDescription()
					 << ",   current pointAtID=" << pointAtID << ", scanID=" << scanID << endl;
      return;
    }
    // we've dealt with the event (processed an image or did a scan); now see what we should do next
    if ( requestExitTest() )
      requestComplete();
    else if ( curReq->worldTargets.size() > 0 )
      doNextSearch();
    else if ( determineNextGazePoint() )
      moveToNextGazePoint();
    else
      requestComplete();
    break;

  default:
    cout << "MapBuilder received unexpected event: " << event->getDescription() << endl;
  }
}

void MapBuilder::processImage(const LookoutSketchEvent &e) {
  const fmat::Transform& camToBase = curReq->baseTransform * e.toBaseMatrix;
  const fmat::Transform baseToCam = camToBase.rigidInverse();
	if ( ! curReq->ignoreCamera ) {
		if ( curReq->clearCamera ) {
			camSkS.clear(false); // don't clear retained sketches
			camShS.clear();
		}
		if ( curReq->rawY ) {
			NEW_SKETCH(rawY, uchar, VRmixin::sketchFromRawY());
		}
		NEW_SKETCH(camFrame, uchar, e.getSketch());
		if (curReq->userImageProcessing != NULL) (*curReq->userImageProcessing)();
		getCameraShapes(camFrame);
		if (curReq->userCamProcessing != NULL) (*curReq->userCamProcessing)();
		if (curReq->getRequestType() > MapBuilderRequest::cameraMap) {
			projectToGround(camToBase);
			if (curReq->userGroundProcessing != NULL) (*curReq->userGroundProcessing)();
			filterGroundShapes(baseToCam);
		}
	}
  switch ( curReq->getRequestType() ) {
  case MapBuilderRequest::cameraMap:
  case MapBuilderRequest::groundMap:
    break;
  case MapBuilderRequest::localMap:
		if ( ! curReq->ignoreCamera )
			extendLocal(baseToCam);
    if (curReq->userLocalProcessing != NULL) (*curReq->userLocalProcessing)();
    break;
  case MapBuilderRequest::worldMap:
		if ( ! curReq->ignoreCamera )
			extendLocal(baseToCam);
    if (curReq->userLocalProcessing != NULL) (*curReq->userLocalProcessing)();
    extendWorld(baseToCam);
    if (curReq->userWorldProcessing != NULL) (*curReq->userWorldProcessing)();
  }
}

bool MapBuilder::determineNextGazePoint() {
	if ( (curReq->verbosity & MBVgazePointQueue) ) {
		std::cout << "GazePoint queue: ";
		if ( curReq->gazePts.empty() )
			std::cout << "empty";
		else
			for (std::vector<GazePoint>::const_iterator it = curReq->gazePts.begin();
					 it != curReq->gazePts.end(); it++ )
				std::cout << " " << it->point;
		std::cout << std::endl;
	}
  if (curReq->getRequestType() == MapBuilderRequest::worldMap) {
		// worldMap
    worldShS.applyTransform(worldToLocalMatrix,egocentric);
    bool b = determineNextGazePoint(worldShS.allShapes()) || determineNextGazePoint(curReq->gazePts);
    worldShS.applyTransform(localToWorldMatrix,allocentric); // transform back
    return b;
  }
  else // localMap
    return determineNextGazePoint(localShS.allShapes()) || determineNextGazePoint(curReq->gazePts);
}

  
bool MapBuilder::determineNextGazePoint(const vector<ShapeRoot>& shapes) {
	if ( ! curReq->pursueShapes )
		return false;
  HeadPointerMC headpointer_mc;
  for (vector<ShapeRoot>::const_iterator it = shapes.begin();
       it != shapes.end(); it++) {
    // look for invalid endpoints of lines / polygons
    if ((*it)->isType(lineDataType) || (*it)->isType(polygonDataType)) {
      const Shape<LineData>& ld = ShapeRootTypeConst(*it,LineData);
      const Shape<PolygonData>& pd = ShapeRootTypeConst(*it,PolygonData);
      bool isLine = (*it)->isType(lineDataType);
      EndPoint p[2] = { isLine ? ld->end1Pt(): pd->end1Pt(), isLine ? ld->end2Pt() : pd->end2Pt()};
      for (int i = 0; i < 2; i++) {
				if ( !p[i].isValid() && !isBadGazePoint(p[i]) ) {
					cout << "Next gazepoint at endpoint" << (i+1) << " of shape id " 
							 << (*it)->getId() << " at " << p[i] << endl;
					if ( !headpointer_mc.lookAtPoint(p[i].coordX(),p[i].coordY(),p[i].coordZ()) ) {
						if ( curReq->verbosity & MBVbadGazePoint )
							cout << "MapBuilder noting unreachable gaze point " << (Point)p[i] << endl;
						badGazePoints.push_back((Point)p[i]);
					}
					nextGazePoint = p[i];
					return true;
				}
      }
    }
    // look for blobs with incomplete bounding boxes
    if ( (*it)->isType(blobDataType) ) {
      const Shape<BlobData>& bd = ShapeRootTypeConst(*it,BlobData);
      if ( ! (bd->bottomValid && bd->topValid && bd->leftValid && bd->rightValid) ) {
				Point bcentroid = bd->getCentroid();
				if ( ! isBadGazePoint(bcentroid) &&
						 badGazePoints.end() == find(badGazePoints.begin(), badGazePoints.end(), bcentroid)) {
					// blob may never be completely visible, so give it one try but don't keep trying
					badGazePoints.push_back(bcentroid);
					cout << "Next gazepoint for blob at " << bcentroid << endl;
					nextGazePoint = bcentroid;
					return true;
				}
      }
    }
    // look for shapes w/ <2 confidence
		/*
    if ((!(*it)->isType(agentDataType)) &&
				(*it)->getLastMatchId() != 0 &&
				(*it)->getConfidence() <= 1 &&
				! isBadGazePoint((*it)->getCentroid()) &&
				badGazePoints.end() == find(badGazePoints.begin(), badGazePoints.end(), (*it)->getCentroid()))  {
      const Point pt = (*it)->getCentroid();
      cout << "Next gaze point is shape " << (*it)
					 << " (confidence level: " << (*it)->getConfidence() << ")" << endl;      
      cout << " at " << pt << endl;  
      if (! headpointer_mc.lookAtPoint(pt.coordX(),pt.coordY(),pt.coordZ()))
				badGazePoints.push_back(pt);
      nextGazePoint = pt;
      return true;
    }
		*/
    
    if ( curReq->verbosity & MBVskipShape )
      cout << "Skipping shape " << (*it)
					 << " (confidence level: " << (*it)->getConfidence() << ")" << endl;      
  }
  return false;
}


//! Find the next valid gazepoint in the queue, converting to egocentric if necessary.
bool MapBuilder::determineNextGazePoint(vector<GazePoint>& gazePts) {
	if ( ! gazePts.empty() && (curReq->verbosity & MBVgazePointQueue) )
		std::cout << "Gazepoint queue has " << gazePts.size() << " entries." << std::endl;
  for ( vector<GazePoint>::iterator it = gazePts.begin();
				it != gazePts.end();  it = gazePts.erase(it) ) {
    if (it->point.getRefFrameType() == DualCoding::allocentric) {
      it->point.applyTransform(worldToLocalMatrix,egocentric);
      it->point.setRefFrameType(egocentric);
    }
    if ( ! isBadGazePoint(it->point) ) {
      nextGazePoint = it->point;
			gazePts.erase(it);
			badGazePoints.push_back(nextGazePoint);
      return true;
    }
  }
  return false;
}

void MapBuilder::moveToNextGazePoint(const bool manualOverride) {
  if ( curReq == NULL ) {
    cout << "curReq == NULL in moveToNextGazePoint!" << endl;
    return;
  }
  if ( (curReq->verbosity & MBVnextGazePoint) || (curReq->manualHeadMotion && manualOverride==false) )
    cout << "moveToNextGazePoint " << nextGazePoint << endl;
  if ( curReq->manualHeadMotion && manualOverride==false ) {
    cout << "To proceed to this gaze point:  !msg MoveHead" << endl;
    return;
  }
  else
    storeImage(true);
}


void MapBuilder::doNextSearch() {
  LookoutSearchRequest *curLSR = curReq->worldTargets.front();
  curReq->worldTargets.pop();
  pointAtID = VRmixin::lookout->executeRequest(*curLSR);
}

bool MapBuilder::isBadGazePoint(const Point& pt) const {
  const coordinate_t x = pt.coordX();
  const coordinate_t y = pt.coordY();
	if ( x*x + y*y > maxDistSq )
		return true;
	float const badGazePointRadiusSq = 100;
	for ( vector<Point>::const_iterator it = badGazePoints.begin();
				it != badGazePoints.end(); it++ )
		if ( ((x-it->coordX())*(x-it->coordX()) + (y-it->coordY())*(y-it->coordY())) < badGazePointRadiusSq )
			return true;
  return false;
}

void MapBuilder::storeImage(bool useNextGazePoint) {
  LookoutPointRequest lreq;
  lreq.motionSettleTime = curReq->motionSettleTime;
  lreq.numSamples = curReq->numSamples;
  lreq.sampleInterval = curReq->sampleInterval;
  if ( useNextGazePoint )
    lreq.setTarget(nextGazePoint);
  else
    lreq.setHeadMotionType(LookoutRequestBase::noMotion);
  pointAtID = VRmixin::lookout->executeRequest(lreq);
}

void MapBuilder::grabCameraImageAndGo() {
  // This is a performance hack to avoid calling the Lookout or event
  // router, so the MapBuilder can produce results very quickly when
  // we need real-time performance, e.g., for particle filtering where
  // we take multiple snapshots.
  pointAtID = 0;
  Sketch<uchar> camFrame(VRmixin::sketchFromSeg());
#ifdef TGT_HAS_CAMERA
  const fmat::Transform camToBase = kine->linkToBase(CameraFrameOffset);
#else
  const fmat::Transform &camToBase = fmat::Transform::identity();
#endif
  LookoutSketchEvent dummy(true, camFrame, camToBase,
													 EventBase::lookoutEGID, pointAtID, EventBase::deactivateETID);
  processImage(dummy);
  requestComplete();
}

void MapBuilder::scanForGazePts() {
  LookoutScanRequest lreq;
  lreq.searchArea = curReq->searchArea;
  lreq.motionSettleTime = curReq->motionSettleTime;
  set<color_index> colors;  // colors of all the shape types we're looking for
  for (map<ShapeType_t,set<color_index> >::const_iterator it1 = curReq->objectColors.begin();
       it1 != curReq->objectColors.end(); it1++)
    for (set<color_index>::const_iterator it2 = it1->second.begin();
				 it2 != it1->second.end(); it2++)
      colors.insert(*it2);
  lreq.addTask(LookoutScanRequest::VisionRegionTask(colors,curReq->dTheta));
  scanID = VRmixin::lookout->executeRequest(lreq);
}

void MapBuilder::extendLocal(const fmat::Transform& baseToCam) {
  vector<ShapeRoot> all = localShS.allShapes();
  removeNoise(localShS, baseToCam);
  matchSrcToDst(groundShS,localShS,curReq->objectColors[polygonDataType]);
  // Took out the call to removeGazePts because AprilTags can get cut off if their 
  // centroid is visible but near the edge of the camera image; need to foveate them.
  // 
  // removeGazePts(curReq->gazePts, baseToCam);
  curReq->baseToCamMats.push_back(baseToCam);
}

void MapBuilder::extendWorld(const fmat::Transform& baseToCam) {
  worldShS.applyTransform(worldToLocalMatrix,egocentric);
  removeNoise(worldShS, baseToCam);
  matchSrcToDst(localShS,worldShS,curReq->objectColors[polygonDataType]);
  worldShS.applyTransform(localToWorldMatrix,allocentric);
  removeGazePts(curReq->gazePts,baseToCam);
  curReq->baseToCamMats.push_back(baseToCam);
}

bool MapBuilder::requestExitTest() {
  if ( curReq->exitTest == NULL )
    return false;
  else
    return (*curReq->exitTest)();
}

void MapBuilder::requestComplete() {
  LineData:: minLinesPerpDist = minLinesPerpDistSave;
  const size_t reqID = curReq->requestID;
  if ( curReq->verbosity & MBVcomplete )
    cout << "MapBuilderRequest " << reqID << " complete\n";
  BehaviorBase* reqbeh = curReq->requestingBehavior;
  delete curReq;
  curReq = NULL;
  requests.pop();
  if ( reqbeh )
    erouter->postEvent(EventBase::mapbuilderEGID, (size_t)reqbeh, EventBase::statusETID,0);
	else {
		erouter->postEvent(EventBase::mapbuilderEGID, reqID, EventBase::statusETID,0);
		erouter->postEvent(EventBase::mapbuilderEGID, reqID, EventBase::deactivateETID,0);
	}
  if ( requests.empty() )
    VRmixin::lookout->relax();
  else
    executeRequest(); // execute next request AFTER deactivate event has finished processing
}

void MapBuilder::setAgent(const Point &worldLocation, const AngTwoPi worldHeading, bool quiet) {
  if ( !quiet && ((curReq==NULL ? verbosity : curReq->verbosity) & MBVsetAgent) )
    cout << "Agent now at " << worldLocation << " hdg " << worldHeading 
				 << " (= " << float(worldHeading)*180/M_PI << " deg.)" << endl;
  theAgent->setCentroidPt(worldLocation);
  theAgent->setOrientation(worldHeading);
  const coordinate_t dx = worldLocation.coordX();
  const coordinate_t dy = worldLocation.coordY();
  const coordinate_t dz = worldLocation.coordZ();
  float const c = cos(worldHeading);
  float const s = sin(worldHeading);
  float localToWorld[] =
		{c, -s, 0, dx,
		 s,  c, 0, dy, 
		 0,  0, 1, dz};
  localToWorldMatrix = fmat::Matrix<4,3>(localToWorld).transpose();;
  worldToLocalMatrix = localToWorldMatrix.inverse();
}

void MapBuilder::moveAgent(coordinate_t const local_dx, coordinate_t const local_dy, coordinate_t const local_dz, AngSignPi dtheta) {
  Point const agentLoc = theAgent->getCentroid();
  AngTwoPi const heading = theAgent->getOrientation();
  float const c = cos(heading);
  float const s = sin(heading);
  float const dx = local_dx*c + local_dy*-s;
  float const dy = local_dx*s + local_dy*c;
  setAgent(agentLoc + Point(dx,dy,local_dz,allocentric), heading+dtheta);
}

void MapBuilder::importLocalToWorld() {
  worldShS.applyTransform(worldToLocalMatrix,egocentric);
  matchSrcToDst(localShS,worldShS);
  worldShS.applyTransform(localToWorldMatrix,allocentric);
}

ShapeRoot MapBuilder::importLocalShapeToWorld(const ShapeRoot &localShape) {
	ShapeRoot worldShape(worldShS.importShape(localShape));
	worldShape->applyTransform(localToWorldMatrix, allocentric);
	return worldShape;
}

ShapeRoot MapBuilder::importWorldToLocal(const ShapeRoot &worldShape) {
  ShapeRoot localShape(localShS.importShape(worldShape));
  localShape->applyTransform(worldToLocalMatrix,egocentric);
  return localShape;
}

bool MapBuilder::isPointVisible(const Point &pt, const fmat::Transform& baseToCam, float maxDistanceSq) {
	fmat::Column<3> camCoords = baseToCam*fmat::pack(pt.coordX(),pt.coordY(),pt.coordZ());
  //  if (camCoords(3) <=0 || distSq(camCoords) >= maxDistanceSq) return false;
  if ( fmat::SubVector<2>(camCoords).sumSq() >= maxDistanceSq )
    return false;
  float normX,normY; // normalized coordinates in cam frame
  config->vision.computePixel(camCoords[0],camCoords[1],camCoords[2],normX,normY);
  return (fabs(normX) < 0.9f && fabs(normY) < config->vision.aspectRatio*.9f); //normX and normY range from -1 to 1. Giving 10% offset here
}

bool MapBuilder::isLineVisible(const LineData& ln, const fmat::Transform& baseToCam) {
  float normX1,normX2,normY1,normY2;
  fmat::Column<3> camCoords;
  Point pt = ln.end1Pt();
  camCoords = baseToCam*fmat::pack(pt.coordX(),pt.coordY(),pt.coordZ());
  config->vision.computePixel(camCoords[0],camCoords[1],camCoords[2],normX1,normY1);
	pt = ln.end2Pt();
  camCoords = baseToCam*fmat::pack(pt.coordX(),pt.coordY(),pt.coordZ());
  config->vision.computePixel(camCoords[0],camCoords[1],camCoords[2],normX2,normY2);
	const float xRange = 0.9f;
	const float yRange = config->vision.aspectRatio*.9f; // y ranges ± aspect ratio, then 10% margin
  const bool end1Pt_visible = fabs(normX1) < xRange && fabs(normY1) < yRange;
  const bool end2Pt_visible = fabs(normX2) < xRange && fabs(normY2) < yRange;
  if (end1Pt_visible && end2Pt_visible)
    return true;
  LineData lnCam(VRmixin::groundShS, Point(normX1,normY1), Point(normX2,normY2));
  // define bounding box of camera frame in terms of normalized coordinates with 10% offset
  LineData camBounds[] = {LineData(VRmixin::groundShS, Point( xRange, yRange), Point( xRange,-yRange)),
													LineData(VRmixin::groundShS, Point( xRange,-yRange), Point(-xRange,-yRange)),
													LineData(VRmixin::groundShS, Point(-xRange,-yRange), Point(-xRange, yRange)),
													LineData(VRmixin::groundShS, Point(-xRange, yRange), Point( xRange, yRange))};
  unsigned int ptCount = 0;
  Point p[2];
  // find if a portion of the line shows up in cam
  if (end1Pt_visible) p[ptCount++].setCoords(normX1,normY1,0); // end1Pt in frame
  else if (end2Pt_visible) p[ptCount++].setCoords(normX2,normY2,0); // end2Pt in frame
  for (int i = 0; i < 4; i++)
    if (camBounds[i].intersectsLine(lnCam)) {
      p[ptCount++].setCoords(lnCam.intersectionWithLine(camBounds[i]));
      // Let's say portion of line seen in cam should be longer than .1 normalized
      if (ptCount > 1)
				return p[0].distanceFrom(p[1]) > 0.1; 
    }
  return false;
}

bool MapBuilder::isShapeVisible(const ShapeRoot &ground_shape, const fmat::Transform& baseToCam,
																float maxDistanceSq) {
  if (ground_shape->isType(lineDataType))
    return isLineVisible(ShapeRootTypeConst(ground_shape,LineData).getData(), baseToCam);
  else if (ground_shape->isType(polygonDataType)) {
    const Shape<PolygonData>& polygon = ShapeRootTypeConst(ground_shape,PolygonData);
    for (vector<LineData>::const_iterator edge_it = polygon->getEdges().begin();
				 edge_it != polygon->getEdges().end(); edge_it++)
      if (isLineVisible(*edge_it,baseToCam))
				return true;
    return false;
  }
  else 
    return isPointVisible(ground_shape->getCentroid(), baseToCam, maxDistanceSq);
}


// filter "bad" ground shapes before importing to dst shape space.
// 1. ignore shapes too far from dog or projected to the other side of cam plane
// 2. chop off line at max distance if it is extending beyond the distance and leave the endpoint invalid
void MapBuilder::filterGroundShapes(const fmat::Transform& baseToCam) {
  //  cout << "MapBuilder::filterGroundShapes()" << endl;
  vector<ShapeRoot> ground_shapes = groundShS.allShapes();

  for (vector<ShapeRoot>::iterator ground_it = ground_shapes.begin();
       ground_it != ground_shapes.end(); ground_it++ ) {
    const coordinate_t cenX = (*ground_it)->getCentroid().coordX();
    const coordinate_t cenY = (*ground_it)->getCentroid().coordY();
    if (cenX*cenX + cenY*cenY > maxDistSq) { // too far
      if ( curReq->verbosity & MBVnotAdmissible )
				cout << "ground shape " << (*ground_it)->getId() << " (lastMatch " 
						 << (*ground_it)->getLastMatchId() << ") too far, delete\n";
      ground_it->deleteShape();
    }
    fmat::Column<3> coords = Kinematics::pack(cenX,cenY,(*ground_it)->getCentroid().coordZ());
    coords = baseToCam*coords;
    if (coords[2] < 0) { // negative z-coordinate in camera frame indicates projection failed
      if ( curReq->verbosity & MBVprojectionFailed )
				cout << "Projection failed for ground shape " << (*ground_it)->getId()
						 << ": " << coords
						 << " (lastMatch " << (*ground_it)->getLastMatchId() << "): deleting\n";
      ground_it->deleteShape();
    }
    // if a line is extending to maxDistance, chop it off at maxdistance and mark the endpoint invalid
    else if ((*ground_it)->isType(lineDataType)) {
      Shape<LineData>& line = ShapeRootType(*ground_it,LineData);
      const coordinate_t e1x = line->end1Pt().coordX();
      const coordinate_t e1y = line->end1Pt().coordY();
      const coordinate_t e2x = line->end2Pt().coordX();
      const coordinate_t e2y = line->end2Pt().coordY();
      if (e1x*e1x + e1y*e1y > maxDistSq && e2x*e2x + e2y*e2y > maxDistSq)
				ground_it->deleteShape();
      else if (e1x*e1x + e1y*e1y > maxDistSq || e2x*e2x + e2y*e2y > maxDistSq) {
				//	cout << (*ground_it)->getId() << "(lastMatch " << (*ground_it)->getLastMatchId() 
				//	     << ")  extends beyond maximum distance we want. Chop off the line" << endl;
				vector<float> line_abc = line->lineEquation_abc();
				Point pt;
				const EndPoint* far_ept = (e1x*e1x + e1y*e1y > maxDistSq) ? &line->end1Pt() : &line->end2Pt(); 
      	if (line_abc[1] == 0.0) {
					const coordinate_t y_abs = sqrt(maxDistSq - line_abc[2]*line_abc[2]);
					if (fabs(far_ept->coordY()-y_abs) < fabs(far_ept->coordY()+y_abs))
						pt.setCoords(e1x, y_abs, far_ept->coordZ());
					else
						pt.setCoords(e1x, -y_abs, far_ept->coordZ());
				}
				else {
					const float a = - line_abc[0]/line_abc[1];
					const float b = line_abc[2]/line_abc[1];
					const coordinate_t x1 = (std::sqrt((a*a+1)*maxDistSq-b*b)-a*b)/(a*a+1);
					const coordinate_t x2 = (-std::sqrt((a*a+1)*maxDistSq-b*b)-a*b)/(a*a+1);
					if (std::abs(far_ept->coordX()-x1) < std::abs(far_ept->coordX()-x2))
						pt.setCoords(x1, a*x1+b, far_ept->coordZ());
					else
						pt.setCoords(x2, a*x2+b, far_ept->coordZ());
				}
				EndPoint temp_endPt(pt);
				temp_endPt.setValid(false);
				//	cout << " (" << far_ept->coordX() << "," << far_ept->coordY() << ") => ("
				//	     << pt.coordX() << "," << pt.coordY() << ")" << endl;
				if (e1x*e1x + e1y*e1y > maxDistSq)
					line->setEndPts(temp_endPt, line->end2Pt());
				else
					line->setEndPts(line->end1Pt(), temp_endPt);
				badGazePoints.push_back(pt);
      }
    }
  }
}

void MapBuilder::calculateGroundPlane() {
  switch(curReq->groundPlaneAssumption) {
  case MapBuilderRequest::onLegs:
    ground_plane = kine->calculateGroundPlane(); 
    if ( curReq->verbosity & MBVgroundPlane ) 
      cout << "Calculated ground plane: " << ground_plane << endl;
    break;
  case MapBuilderRequest::onStand:
#if defined(TGT_ERS210) || defined(TGT_ERS220) || defined(TGT_ERS2xx)
    ground_plane = PlaneEquation(0,0,-1,200);
#else
    ground_plane = PlaneEquation(0,0,-1,170);
#endif
    // cout << "Hard-coded ground plane: " << NEWMAT::printmat(ground_plane) << endl;    
    break;
  case MapBuilderRequest::onWheel:
#ifdef TGT_REGIS1
    std::cout << "Target Regis 1 Mapping";
		ground_plane = fmat::pack<float>(0,0,(-1/85.0),1);
#endif
		break;    
  case MapBuilderRequest::custom:
    ground_plane = curReq->customGroundPlane;
  }
}

void MapBuilder::projectToGround(const fmat::Transform& camToBase) {
  VRmixin::projectToGround(camToBase, ground_plane);
}

ShapeRoot MapBuilder::projectToLocal(ShapeRoot &shape) {
#ifdef TGT_HAS_CAMERA
	fmat::Transform camToBase = kine->linkToBase(CameraFrameOffset);
#else
	fmat::Transform camToBase = fmat::Transform::identity();
#endif
	ground_plane = kine->calculateGroundPlane();
	
	groundShS.importShape(shape);
	ShapeRoot &newShape = groundShS.allShapes().back();
	newShape->projectToGround(camToBase, ground_plane);
	localShS.importShape(newShape);
	return localShS.allShapes().back();
}
	
void MapBuilder::matchSrcToDst(ShapeSpace &srcShS, ShapeSpace &dstShS,
															 set<color_index> polCols, bool mergeSrc, bool mergeDst) {
  vector<ShapeRoot> src_shapes = srcShS.allShapes();
  vector<ShapeRoot> dst_shapes = dstShS.allShapes();
  vector<LineData> polygon_edges;
  
  // clean up src_shapes before messing with dst space
  std::set<int> markedForDeletion;
  vector<ShapeRoot> cleaned_src;
  for (vector<ShapeRoot>::iterator src_it = src_shapes.begin();
       src_it != src_shapes.end(); src_it++ ) {
		// mark inadmissible shapes for deletion
    if (!(*src_it)->isAdmissible()) {
      if (curReq && curReq->verbosity & MBVnotAdmissible )
				cout << "shape " << (*src_it)->getId() << " (lastMatch " 
						 << (*src_it)->getLastMatchId() << ") is not admissible." << endl;
      // (*src_it)->printParams();
      markedForDeletion.insert((*src_it)->getId());
    }
    else if ((*src_it)->isType(polygonDataType) && (*src_it)->getParentId() != 0) {
      const vector<LineData>& edges = ShapeRootTypeConst(*src_it, PolygonData)->getEdges();
			// for old polygons not built by the user (parentId not 0), save
			// their edges and mark them for deletion because we're going to
			// rebuild them
      polygon_edges.insert(polygon_edges.end(), edges.begin(), edges.end());
      markedForDeletion.insert((*src_it)->getId());
    }
    else if ((*src_it)->isType(lineDataType)) {
			// mark lines for deletion (but save their edges) if we will be
			// incorporating them into polygons
      const color_index colorIndex = ProjectInterface::getColorIndex((*src_it)->getColor());
      if ( polCols.end() != find(polCols.begin(), polCols.end(), colorIndex)) {
				polygon_edges.push_back(ShapeRootTypeConst(*src_it, LineData).getData());
				markedForDeletion.insert((*src_it)->getId());
      }
    }
  }
  // now copy the shapes we didn't mark for deletion
  for ( vector<ShapeRoot>::iterator it = src_shapes.begin();
				it != src_shapes.end(); it++ )
    if ( markedForDeletion.find((*it)->getId()) == markedForDeletion.end() )
      cleaned_src.push_back(*it);
  src_shapes = cleaned_src;

  // merge shapes inside srcShS (useful for lines that should have been merged)
  if (mergeSrc) {
    markedForDeletion.clear();
    cleaned_src.clear();
    for ( vector<ShapeRoot>::iterator it = src_shapes.begin();
					it != src_shapes.end(); it++ )
      for ( vector<ShapeRoot>::iterator it2 = it+1;
						it2 != src_shapes.end(); it2++)
				if ((*it2)->isMatchFor(*it) && (*it)->updateParams(*it2)) {
					if (curReq && curReq->verbosity & MBVshapesMerge )
						cout << "merging shape " << (*it)->getId() << " (from " << (*it)->getLastMatchId()
								 << ") and shape " << (*it2)->getId() << " (from " << (*it2)->getLastMatchId() << ")" << endl;
					markedForDeletion.insert((*it2)->getId());
				}
    // now copy the shapes we didn't mark for deletion
    for ( vector<ShapeRoot>::iterator it = src_shapes.begin();
					it != src_shapes.end(); it++ )
      if ( markedForDeletion.find((*it)->getId()) == markedForDeletion.end() )
				cleaned_src.push_back(*it);
    src_shapes = cleaned_src;
  }

  // update dst shapes from src shapes, and note existing polygons
  markedForDeletion.clear();
  cleaned_src.clear();
  std::set<int> markedForDeletionDst;
  vector<Shape<PolygonData> > existingPolygons;
  for (vector<ShapeRoot>::iterator dst_it = dst_shapes.begin();
       dst_it != dst_shapes.end(); dst_it++ ) {
    // cout << "update dest shape " << *dst_it << endl;
    if ( (*dst_it)->isType(polygonDataType) && (*dst_it)->getParentId() != 0 ) {
      existingPolygons.push_back(ShapeRootType(*dst_it,PolygonData));
      continue;
    }
    if ( (*dst_it)->isType(localizationParticleDataType) )
      continue;
    for (vector<ShapeRoot>::iterator src_it = src_shapes.begin();
				 src_it != src_shapes.end(); src_it++)
      if ((*dst_it)->isMatchFor(*src_it) && (*dst_it)->updateParams((*src_it))) {
				(*dst_it)->increaseConfidence(2);  // add 2 because we'll subtract 1 at the end
				(*dst_it)->setLastMatchId((*src_it)->getId());
				if (curReq && curReq->verbosity & MBVshapeMatch )
					cout << "Matched src shape " << *src_it << " (lastMatch " << (*src_it)->getLastMatchId()
							 << ") to dst shape " << (*dst_it)->getId() << endl;
				markedForDeletion.insert((*src_it)->getId());
      } else if((*dst_it)->getType() == agentDataType && (*src_it)->getType() == agentDataType &&
                (*dst_it)->getName() == (*src_it)->getName()) {
        //any agents that were just seen and don't match an existing agent's position
        //but have the same name should take precedence over the existing agent
        //i.e. the agent moved and we are seeing it in a new position
        //cout << "marking " << (*dst_it)->getId() << " for deletion" << endl;
        markedForDeletionDst.insert((*dst_it)->getId());
        break;
      }
  }

  // now copy the shapes we didn't mark for deletion
  for ( vector<ShapeRoot>::iterator it = src_shapes.begin();
				it != src_shapes.end(); it++ )
    if ( markedForDeletion.find((*it)->getId()) == markedForDeletion.end() ) {
      cleaned_src.push_back(*it);
      if((*it)->getType() == agentDataType) {
        const Shape<AgentData> &a = ShapeRootTypeConst(*it, AgentData);
        cout << a->getName() << ", " << a->getId() << ": " << a->getCentroid() << ", " << a->getOrientation() << endl;
      }
    }
  src_shapes = cleaned_src;
  for(vector<ShapeRoot>::iterator it = dst_shapes.begin();
            it != dst_shapes.end(); ++it) {
    if(markedForDeletionDst.find((*it)->getId()) != markedForDeletionDst.end()) {
      dstShS.deleteShape(*it);
    }
  }
  
  // form polygons from lines and import unmatched src shapes into dstShS
  vector<Shape<PolygonData> > deletedPolygons;
  vector<ShapeRoot> newPolygons = PolygonData::rebuildPolygons(polygon_edges,existingPolygons,deletedPolygons);
  for (vector<Shape<PolygonData> >::iterator delete_it = deletedPolygons.begin();
       delete_it != deletedPolygons.end(); delete_it++) {
    delete_it->deleteShape();
	}
  dstShS.importShapes(newPolygons);
  dstShS.importShapes(src_shapes);
  if (curReq && curReq->verbosity & MBVimportShapes ) {
		std::map<ShapeType_t,int> shapeCounts;
		if ( ! newPolygons.empty() )
			shapeCounts[polygonDataType] += newPolygons.size();
		for (std::vector<ShapeRoot>::iterator it = src_shapes.begin();
				 it != src_shapes.end(); it++)
			++shapeCounts[(*it)->getType()];
		cout << dstShS.name << "ShS imported";
		int comma = 0;
		if ( src_shapes.empty() && newPolygons.empty() )
			cout << " nothing";
		else
			for (std::map<ShapeType_t,int>::const_iterator it = shapeCounts.begin();
					 it != shapeCounts.end(); it++)
				cout << (comma++ ? "," : "") << " " << it->second << " " << data_name(it->first);
		cout << endl;
  }
  // match shapes inside dstShS
  // ***NOTE: this code is wrong because deleting a shape should force a restart of the iteration
  if (mergeDst) {
    dst_shapes = dstShS.allShapes();
    for ( vector<ShapeRoot>::iterator it = dst_shapes.begin();
					it != dst_shapes.end(); it++ ) {
      if ( (*it)->isType(localizationParticleDataType) || (*it)->isType(agentDataType) )
				continue;
			for ( vector<ShapeRoot>::iterator it2 = it+1;
						it2 < dst_shapes.end(); it2++)
				if ((*it2)->isMatchFor(*it) && (*it)->updateParams(*it2,true)) {
					cout << "Matched src shape " << *it << " (lastMatch " 
							 << (*it)->getLastMatchId()<< ") is a match for " 
							 << *it2 << " (lastMatch " << (*it2)->getLastMatchId()
               << "), delete " << (*it2)->getId() << endl;;
          (*it)->setLastMatchId((*it2)->getLastMatchId());
					it2->deleteShape();
				}
		}
  }
}

// decrease confidence of those shapes that should have been seen in the last snap,
// remove shapes from ShapeSpace if confidence becomes < 0
void MapBuilder::removeNoise(ShapeSpace& ShS, const fmat::Transform& baseToCam) {
	//  cout << "MapBuilder::removeNoise()\n";
	vector<ShapeRoot> shapes = ShS.allShapes();
	for (vector<ShapeRoot>::iterator it = shapes.begin();
			 it != shapes.end(); it++ ) {
		// If we were not looking for this shape in the last snap, then
		// it's not fair to decrease this shape's confidence.
		if (curReq->objectColors[(*it)->getType()].find(ProjectInterface::getColorIndex((*it)->getColor())) ==
				curReq->objectColors[(*it)->getType()].end())
			continue; 
		// For polygons, verify that each edge that should be visible is seen.
		// If not, then delete the edge and rebuild the polygon.
		if ( (*it)->isType(polygonDataType) && (*it)->getParentId() != 0 ) {
			Shape<PolygonData>& polygon = ShapeRootType(*it,PolygonData);
			vector<LineData>& edges = polygon->getEdgesRW();
			unsigned int edge_index = 0;
			for (vector<LineData>::iterator edge_it = edges.begin();
					 edge_it != edges.end(); edge_it++, edge_index++) {
				if (isLineVisible(*edge_it, baseToCam)) {
					// if ( curReq->verbosity & MBVshouldSee )
					//   cout << "edge " << edge_index << " of polygon " << (*it)->getId() << "(confidence: " 
					//	      << edge_it->getConfidence() << ") should be visible in this frame" << endl;
					edge_it->decreaseConfidence();
				}
			}
			vector<ShapeRoot> brokenPolygons = polygon->updateState();
			ShS.importShapes(brokenPolygons);
			if (!polygon->isAdmissible()) {
				if ( curReq->verbosity & MBVdeleteShape )
					cout << "delete polygon " << (*it)->getId() << " from map" << endl;
				it->deleteShape();
			}
		}
		else if ((!(*it)->isType(agentDataType)) && isShapeVisible(*it, baseToCam, maxDistSq)) {
			if ((*it)->getConfidence() < 0 ) {
				if ( curReq->verbosity & MBVshouldSee )
					cout << "shape " << (*it)->getId() << "(confidence: " << (*it)->getConfidence() 
							 << ") should be visible in this frame; delete it" << endl;
				it->deleteShape();
			}
      else  // confidence good enough; decrement here because we'll increment later
        (*it)->decreaseConfidence(); // decrease confidence by 1 for every visible shape
		}
	}
}

//================ Gaze points ================

void MapBuilder::setInitialGazePts() {
	const ShapeRoot &sArea = curReq->searchArea;
	if ( ! sArea.isValid() )
		return;
	else if ( curReq->doScan == true )
		scanForGazePts();
	else
		switch ( sArea->getType() ) {

		case pointDataType:
			curReq->gazePts.push_back(GazePoint(GazePoint::centered,sArea->getCentroid()));
			break;

		case lineDataType: {
			static const float meshSize=50;
			const Shape<LineData>& line = ShapeRootTypeConst(sArea,LineData);
			if ( curReq->doScan == true )
				scanForGazePts();
			else {
				int numIntervals = (int) (line->getLength()/meshSize);
				const EndPoint& ep1 = line->end1Pt();
				const EndPoint& ep2 = line->end2Pt();
				curReq->gazePts.push_back(GazePoint(GazePoint::centered,ep1));
				for (int i = 1; i < numIntervals; i++)
					curReq->gazePts.push_back(GazePoint(GazePoint::centered,(ep1*i + ep2*(numIntervals-i))/numIntervals));
				curReq->gazePts.push_back(GazePoint(GazePoint::centered,ep2));
			}
		}
			break;

		case polygonDataType:
			if ( curReq->doScan == true )
				scanForGazePts();
			else {
				const Shape<PolygonData> &poly = ShapeRootTypeConst(sArea,PolygonData);
				const vector<Point> &verts = poly->getVertices();
				for (std::vector<Point>::const_iterator it = verts.begin(); it != verts.end(); it++)
					curReq->gazePts.push_back(GazePoint(GazePoint::centered,*it));
			}
			break;

		default:
			cerr << "ERROR MapBuilder::setInitialGazePts: Supported searchArea shapes are Point, Line, and Polygon\n";
			requestComplete();
			break;
		}
}

void MapBuilder::removeGazePts(vector<GazePoint> &gazePts, const fmat::Transform& baseToCam) {
	if (curReq->removePts) {
		int num_points_seen = 0;
		for ( vector<GazePoint>::iterator it = gazePts.begin();
					it != gazePts.end(); it++ ) {
			if ( it->point == nextGazePoint || 
					 (it->type == GazePoint::visible && isPointVisible(it->point,baseToCam,maxDistSq)) ) {
				if ( it->point != nextGazePoint )
					cout << "Removing already-visible gaze point " << it->point << endl;
				num_points_seen++;
				gazePts.erase(it--);
			}
		}
		// cout << num_points_seen << " pre-defined gaze points seen in last image, "
		//      << gazePts.size() << " left\n";
	}
}


//================================================================

//Prints shapespace in the format to be used for particle filter on simulator
void MapBuilder::printShS(ShapeSpace &ShS) const {
  cout << "MapBuilder::printShS()" << endl;
  unsigned int line_count = 0;
  vector<ShapeRoot> shapes = ShS.allShapes();
  for (vector<ShapeRoot>::const_iterator it = shapes.begin();
       it != shapes.end(); it++) {
    if ((*it)->isType(lineDataType)) {
      const Shape<LineData>& ld = ShapeRootTypeConst(*it,LineData);
      cout << (*it)->getId() << " " << lineDataType << " " 
					 << ProjectInterface::getColorIndex((*it)->getColor()) 
					 << " " << ld->end1Pt().coordX()  << " " << ld->end1Pt().coordY()
					 << " " << ++line_count << " " << ld->getLength() << " " << ld->end1Pt().isValid() << endl; 
      cout << (*it)->getId() << " " << lineDataType << " " 
					 << ProjectInterface::getColorIndex((*it)->getColor()) 
					 << " " << ld->end2Pt().coordX()  << " " << ld->end2Pt().coordY()
					 << " " << line_count << " " << ld->getLength() << " " << ld->end2Pt().isValid() << endl;
    }
    else {
      cout << (*it)->getId() << " " << (*it)->getType() << " " 
					 << ProjectInterface::getColorIndex((*it)->getColor()) 
					 << " " << (*it)->getCentroid().coordX()  << " " << (*it)->getCentroid().coordY() << endl;
    }
  }
}


//================ Shape extraction ================

void MapBuilder::getCameraShapes(const Sketch<uchar>& camFrame) { 
  float extractorMinLineLengthSave = LineData::extractorMinLineLength;
  float extractorGapToleranceSave = LineData::extractorGapTolerance;
  LineData::extractorMinLineLength = curReq->extractorMinLineLength;
  LineData::extractorGapTolerance = curReq->extractorGapTolerance;

  getCamLines(camFrame, curReq->objectColors[lineDataType], curReq->occluderColors[lineDataType]);
  getCamEllipses(camFrame, curReq->objectColors[ellipseDataType], curReq->occluderColors[ellipseDataType]);
  getCamPolygons(camFrame, curReq->objectColors[polygonDataType], curReq->occluderColors[polygonDataType]);
  getCamSpheres(camFrame, curReq->objectColors[sphereDataType], curReq->occluderColors[sphereDataType]);
  getCamCylinders(camFrame, curReq->objectColors[cylinderDataType], curReq->assumedCylinderHeights, curReq->minBlobAreas);
  getCamWalls(camFrame, curReq->floorColor);
  if ( curReq->numSamples == 1 && !curReq->searchArea.isValid() && 
       !curReq->objectColors[blobDataType].empty() && curReq->userImageProcessing == NULL )  // use CMVision's blob extraction from current camera frame
    getCamBlobs(curReq->objectColors[blobDataType]);
  else
    getCamBlobs(camFrame, curReq->objectColors[blobDataType], curReq->minBlobAreas, curReq->blobOrientations, curReq->assumedBlobHeights);
  getCamTargets(camFrame, curReq->objectColors[targetDataType], curReq->occluderColors[targetDataType]);
  getCamMarkers(camFrame, curReq->objectColors[markerDataType], curReq->occluderColors[markerDataType],
								curReq->markerTypes);
	getCamDominoes(camFrame, curReq->objectColors[dominoDataType], curReq->secondColors[dominoDataType]);

	getCamNaughts(camFrame, curReq->objectColors[naughtDataType], curReq->naughtDimensions);
	getCamCrosses(camFrame, curReq->objectColors[crossDataType], curReq->crossDimensions);

  // ***** HACK: SHOULD BE GETTING THIS FROM LOOKOUT *****
  if ( !curReq->siftDatabasePath.empty() || curReq->aprilTagFamily.first != 0) {
		NEW_SKETCH_N(rawY, uchar, VRmixin::sketchFromRawY());
		getCamSiftObjects(rawY, curReq->siftDatabasePath, curReq->siftObjectNames);
		getCamAprilTags(rawY);
	}
	if ( ! curReq->objectColors[agentDataType].empty() ) {
		NEW_SKETCH_N(camFrameYUV, yuv, VRmixin::sketchFromYUV());
		getCamAgents(camFrame, camFrameYUV, curReq->objectColors[agentDataType]);
	}

  // Cleanup before exit
  LineData::extractorMinLineLength = extractorMinLineLengthSave;
  LineData::extractorGapTolerance = extractorGapToleranceSave;
}

vector<Shape<LineData> > MapBuilder::getCamLines(const Sketch<uchar> &camFrame, const set<color_index>& objectColors, 
																								 const set<color_index>& occluderColors) const {
  vector<Shape<LineData> > linesReturned;
  if ( objectColors.empty() ) 
    return linesReturned;
  if ( curReq->verbosity & MBVshapeSearch )
    cout << "*** Find the lines ***" << endl;
  NEW_SKETCH_N(occluders,bool,visops::zeros(camFrame));
  for ( set<color_index>::const_iterator it = occluderColors.begin();
				it != occluderColors.end(); it++ )
    occluders |= visops::minArea(visops::colormask(camFrame,*it));

  for (set<color_index>::const_iterator it = objectColors.begin();
       it != objectColors.end(); it++) {
    NEW_SKETCH_N(colormask,bool,visops::colormask(camFrame,*it));
    NEW_SKETCH_N(cleancolor,bool,visops::minArea(colormask));
    NEW_SKETCH_N(fatmask,bool,visops::fillin(cleancolor,1,2,8));
    NEW_SKETCH_N(skel,bool,visops::skel(fatmask));
    vector<Shape<LineData>> line_shapes(LineData::extractLines(skel,cleancolor|occluders));
    linesReturned.insert(linesReturned.end(), line_shapes.begin(), line_shapes.end());
    if ( curReq->verbosity & MBVshapesFound )
      cout << "Found " << line_shapes.size() << " " 
					 << ProjectInterface::getColorName(*it) << " lines." << endl;
  }
  return linesReturned;
}

vector<Shape<EllipseData> > 
MapBuilder::getCamEllipses(const Sketch<uchar> &camFrame,
													 const set<color_index>& objectColors, const set<color_index>& ) const {
  vector<Shape<EllipseData> > ellipses;
  if (objectColors.empty())
    return ellipses;
  if ( curReq->verbosity & MBVshapeSearch )
    cout << "*** Find the ellipses ***" << endl;
  for (set<color_index>::const_iterator it = objectColors.begin();
       it !=objectColors.end(); it++) {
    NEW_SKETCH_N(colormask,bool,visops::colormask(camFrame,*it));
    vector<Shape<EllipseData> > ellipse_shapes = EllipseData::extractEllipses(colormask);
    ellipses.insert(ellipses.end(), ellipse_shapes.begin(),ellipse_shapes.end());
    if ( curReq->verbosity & MBVshapesFound )
      cout << "Found " << ellipse_shapes.size() << " "
					 << ProjectInterface::getColorName(*it) << " ellipses." << endl;
  }
  return ellipses;
}

void MapBuilder::getCamPolygons(const Sketch<uchar> &camFrame,
																const set<color_index>& objectColors,
																const set<color_index>& occluderColors) const {
  if ( objectColors.empty() ) 
    return;
  if ( curReq->verbosity & MBVshapeSearch )
    cout << "*** Find the polygons ***" << endl;
  NEW_SKETCH_N(occluders,bool,visops::zeros(camFrame));
  for ( set<color_index>::const_iterator it = occluderColors.begin();
				it !=occluderColors.end(); it++ )
    occluders |= visops::colormask(camFrame,*it);
  
  for (set<color_index>::const_iterator it = objectColors.begin();
       it !=objectColors.end(); it++) {
    NEW_SKETCH_N(colormask,bool,visops::colormask(camFrame,*it));
    NEW_SKETCH_N(fatmask,bool,visops::fillin(colormask,1,2,8));
    NEW_SKETCH_N(skel,bool,visops::skel(fatmask));
    NEW_SKETCH_N(fatskel,bool,visops::fillin(skel,1,2,8));
    
    vector<Shape<LineData> > pLines = LineData::extractLines(fatskel,fatmask|occluders);
		vector<LineData> polygonLines;
		for ( vector<Shape<LineData> >::iterator ln_it = pLines.begin();
					ln_it != pLines.end(); ln_it++ ) {
			polygonLines.push_back(ln_it->getData());
			// cout << "pLine id = " << (*ln_it)->getId() << endl;
			ln_it->deleteShape();
		}
		vector<ShapeRoot> polygons = PolygonData::formPolygons(polygonLines);
    if ( curReq->verbosity & MBVshapesFound )
      cout << "Found " << polygons.size() << " polygons." << endl;
  }
}

void MapBuilder::getCamSpheres(const Sketch<uchar> &camFrame,
															 const set<color_index>& objectColors, const set<color_index>& ) const {
  vector<Shape<SphereData> > spheres;
  if ( objectColors.empty() )
    return;
  if ( curReq->verbosity & MBVshapeSearch )
    cout << "*** Find the spheres ***" << endl;
  for (set<color_index>::const_iterator it = objectColors.begin();
       it !=objectColors.end(); it++) {
    NEW_SKETCH_N(colormask,bool,visops::colormask(camFrame,*it));
    vector<Shape<SphereData> > sphere_shapes = SphereData::extractSpheres(colormask);
    spheres.insert(spheres.end(), spheres.begin(), spheres.end());
    if ( curReq->verbosity & MBVshapesFound )
      cout << "Found " << sphere_shapes.size() << " spheres." << endl;
  }
}

void MapBuilder::getCamCylinders(const Sketch<uchar>& camFrame,
																 const set<color_index>& colors,
																 const map<color_index,coordinate_t>& assumedHeights,
																 const map<color_index,int>& minCylinderAreas) {
  if ( colors.empty() )
    return;
  if ( curReq->verbosity & MBVshapeSearch )
    cout << "*** Find the cylinders ***" << endl;
  int const maxcylinders = 50;
	vector<GazePoint> addGazePts;
  vector<Shape<CylinderData> > result(CylinderData::extractCylinders(camFrame, colors, assumedHeights, 
																																		 minCylinderAreas, maxcylinders, addGazePts));
  if ( curReq->verbosity & MBVshapesFound )
    if ( !result.empty() )
      cout << "Found " << result.size() << " cylinders." << endl;
	if ( ! addGazePts.empty() ) {
		const fmat::Transform camToBase = kine->linkToBase(CameraFrameOffset);
		for ( std::vector<GazePoint>::iterator it = addGazePts.begin();
					it != addGazePts.end(); it++ ) {
			it->point.projectToGround(camToBase,ground_plane);
			// curReq->gazePts.insert(curReq->gazePts.begin(),*it);
			std::cout << "Blob not valid; would have added gazepoint at " << it->point << std::endl;
		}
	}
}
  
vector<Shape<LineData> > 
MapBuilder::getCamWalls(const Sketch<uchar> &camFrame, unsigned int floorColor) const {
  if (floorColor == 0)
    return vector<Shape<LineData> >();
  if ( curReq->verbosity & MBVshapeSearch )
    cout << "*** Find the walls ***" << endl;
  const int camFrame_offset = 8;
  
  NEW_SKETCH_N(colormask,bool,visops::colormask(camFrame,floorColor));
  NEW_SKETCH_N(fillinmask ,bool,visops::fillin(colormask, 1, 6, 8)); //remove pixels w/ connectivity<6 (noise)
  NEW_SKETCH_N(fillinmask2 ,bool,visops::fillin(fillinmask, 2, 3, 8)); //remove pixels w/ connectivity<3 and fill in the others
  NEW_SKETCH_N(edgemask ,bool,visops::fillin(fillinmask2, 1, 5, 7)); //remove pixels w/ connectivity=8 (non-edge pixels)
  NEW_SKETCH_N(edgemask2 ,bool,visops::non_bounds(edgemask, camFrame_offset)); //remove pixels close to cam_bound

  NEW_SKETCH_N(occluders_floor, bool, (camFrame != uchar(0)) & (camFrame != uchar(floorColor)));
  NEW_SKETCH_N(occ_mask ,bool,visops::fillin(occluders_floor, 1, 8, 8)); //remove pixels w/ connectivity<7 (noises)
  usint const clear_dist = 15;
  Sketch<bool> not_too_close = (visops::mdist(occ_mask) >= clear_dist); 
  edgemask2 &= not_too_close; //remove pixels around occuluders
  
  NEW_SKETCH_N(fatmask ,bool,visops::fillin(edgemask2,2,2,8)); //make the remaining pixels fat
  NEW_SKETCH_N(skel,bool,visops::skel(fatmask));
  NEW_SKETCH_N(fatskel,bool,visops::fillin(skel,1,2,8));
  
  vector<Shape<LineData> > wall_bounds = PolygonData::extractPolygonEdges(fatskel,fatmask|occluders_floor);

  // larger offset from the cam frame should be applied to these lines
  // since all pixels near cam frame bounds are removed before extracting these lines.
  for (vector<Shape<LineData> >::iterator it = wall_bounds.begin();
       it != wall_bounds.end(); it++) {
    if (((*it)->end1Pt().coordX() < camFrame_offset*2.0 || (*it)->end1Pt().coordX() > xres - camFrame_offset*2.0
				 || (*it)->end1Pt().coordY() < camFrame_offset*2.0 || (*it)->end1Pt().coordY() > yres - camFrame_offset*2.0)
				&& (*it)->end1Pt().isValid())
      (*it)->end1Pt().setValid(false);
    if (((*it)->end2Pt().coordX() < camFrame_offset*2.0 || (*it)->end2Pt().coordX() > xres - camFrame_offset*2.0
				 || (*it)->end2Pt().coordY() < camFrame_offset*2.0 || (*it)->end2Pt().coordY() > yres - camFrame_offset*2.0)
				&& (*it)->end2Pt().isValid())
      (*it)->end2Pt().setValid(false);
  }
  
  if ( curReq->verbosity & MBVshapesFound )
    cout << "Found " << wall_bounds.size() << " wall boundary lines" << endl;
  return wall_bounds;
}

void MapBuilder::getCamBlobs(const Sketch<uchar>& camFrame,
														 const set<color_index>& colors,
														 const map<color_index,int>& minBlobAreas,
														 const map<color_index, BlobData::BlobOrientation_t>& blobOrientations,
														 const map<color_index,coordinate_t>& assumedBlobHeights) {
  if ( colors.empty() )
    return;
  int const maxblobs = 50;
  vector<Shape<BlobData> > result(BlobData::extractBlobs(camFrame, colors, minBlobAreas, blobOrientations, assumedBlobHeights, maxblobs));
  if ( curReq->verbosity & MBVshapesFound )
    if ( !result.empty() )
      cout << "Found " << result.size() << " blobs." << endl;
}

// The code below grabs blobs directly from the region generator stream, instead of
// calling BlobgData::extractBlobs to do region extraction on camFrame 
void MapBuilder::getCamBlobs(const set<color_index>& colors, int defMinBlobArea) {
  // don't check objectColors here because this may also be called by extractors for various marker types
  if ( curReq->verbosity & MBVshapeSearch )
    cout << "*** Find the blobs ***" << endl;
  for (set<color_index>::const_iterator it = colors.begin();
       it != colors.end(); it++) {
    int const minarea = (curReq->minBlobAreas.find(*it)==curReq->minBlobAreas.end()) ? 
      defMinBlobArea : curReq->minBlobAreas[*it];
    BlobData::BlobOrientation_t const orient = (curReq->blobOrientations.find(*it)==curReq->blobOrientations.end()) ? 
      BlobData::groundplane : curReq->blobOrientations[*it];
    coordinate_t const height = (curReq->assumedBlobHeights.find(*it)==curReq->assumedBlobHeights.end()) ? 
      0 : curReq->assumedBlobHeights[*it];
    vector<Shape<BlobData> > blob_shapes(VRmixin::getBlobsFromRegionGenerator(*it,minarea,orient,height));
    if ( curReq->verbosity & MBVshapesFound )
      if ( !blob_shapes.empty() )
				cout << "Found " << blob_shapes.size() << " "
						 << ProjectInterface::getColorName(*it) << " region generator blobs." << endl;
  }
}

void MapBuilder::getCamTargets(const Sketch<uchar> &camFrame, const set<color_index>& objectColors, const set<color_index>& occluderColors) const {
  vector<Shape<TargetData> > targets;
  if (objectColors.empty())
    return;
  if ( curReq->verbosity & MBVshapeSearch )
    cout << "*** Find the targets ***" << endl;
  
  NEW_SKETCH_N(occluders,bool,visops::zeros(camFrame));
  for (set<color_index>::const_iterator it = occluderColors.begin();
       it != occluderColors.end(); it++)
    occluders |= visops::minArea(visops::colormask(camFrame,*it));
  
  // assumes multiples of 3 for objectColors (stays on the last color otherwise)
  for (set<color_index>::const_iterator it = objectColors.begin();
       it != objectColors.end(); it++) {
    NEW_SKETCH_N(front_colormask, bool, visops::colormask(camFrame,*it));
    it++;
    if (it == objectColors.end()) {
      it--;
    }
    NEW_SKETCH_N(back_colormask, bool, visops::colormask(camFrame,*it));
    it++;
    if (it == objectColors.end()) {
      it--;
    }
    NEW_SKETCH_N(right_colormask, bool, visops::colormask(camFrame,*it));
    Shape<TargetData> target = TargetData::extractLineTarget(front_colormask, back_colormask, right_colormask, occluders);
    if (target.isValid()) {
      targets.insert(targets.end(), target);
    }
    if ( curReq->verbosity & MBVshapesFound )
      cout << "Found " << (target.isValid() ? 1 : 0) << " targets." << endl;
  }
}

vector<Shape<MarkerData> > 
MapBuilder::getCamMarkers(const Sketch<uchar> &camFrame, const set<color_index>& objectColors,
													const set<color_index>&, const set<MarkerType_t>& markerTypes) const
{
  vector<Shape<MarkerData> > markers;
  if (objectColors.empty() || markerTypes.empty())
    return markers;

  if ( curReq->verbosity & MBVshapeSearch )
    cout << "*** Find the markers ***" << endl;

  /* Skip this and let the individual marker extraction algorithms do masking if they need it
		 NEW_SKETCH_N(colormask,bool,visops::zeros(camFrame));
		 for (set<color_index>::const_iterator it = objectColors.begin();
		 it != objectColors.end(); it++) {
		 colormask |= visops::colormask(camFrame,*it);
		 }
		 NEW_SKETCH_N(masked,uchar,visops::mask(camFrame,colormask));
  */

  for (set<MarkerType_t>::const_iterator it = markerTypes.begin();
       it != markerTypes.end(); it++) {
    vector<Shape<MarkerData> > single_type = MarkerData::extractMarkers(camFrame, *it, *curReq);
    markers.insert(markers.end(), single_type.begin(), single_type.end());
  }

  if ( curReq->verbosity & MBVshapesFound )
    cout << "Found " << markers.size() << " markers." << endl;
				    
  return markers;
}

void MapBuilder::getCamSiftObjects(const Sketch<uchar> &rawY, const std::string &siftDatabasePath, 
																	 const std::set<std::string> &siftObjectNames) {
  if ( siftDatabasePath.empty() ) return;
  SiftTekkotsu *matcher = siftMatchers[siftDatabasePath];
  if ( matcher == NULL ) {
    matcher = new SiftTekkotsu;
    matcher->loadFile(siftDatabasePath);
    matcher->setParameter("probOfMatch", 0.8);
    siftMatchers[siftDatabasePath] = matcher;
  }
  ImageBuffer buff = matcher->sketchToBuffer(rawY);
  vector<SiftMatch*> results;
  if ( siftObjectNames.empty() )
    matcher->findAllObjectsInImage(buff, results);
  else
    for ( std::set<std::string>::const_iterator it = siftObjectNames.begin();
					it != siftObjectNames.end(); it++ ) {
      int id = matcher->getObjectID(*it);
      matcher->findObjectInImage(id, buff, results);
    };
  if ( curReq->verbosity & MBVshapesFound )
    cout << "Found " << results.size() << " sift objects." << endl;
  for ( vector<SiftMatch*>::const_iterator it = results.begin();
				it != results.end(); it++ ) {
    NEW_SHAPE(siftobj, SiftData, new SiftData(VRmixin::camShS, *it));
  }
}
    
void MapBuilder::newSiftMatcher(const std::string &siftDatabasePath) {
  if ( siftDatabasePath.empty() ) return;
  SiftTekkotsu *matcher = siftMatchers[siftDatabasePath];
  if ( matcher == NULL ) {
    matcher = new SiftTekkotsu;
    // need to check if file exists matcher->loadFile(siftDatabasePath);
    matcher->setParameter("probOfMatch", 0.8);
    siftMatchers[siftDatabasePath] = matcher;
  }
}

void MapBuilder::getCamAprilTags(const Sketch<uchar> &rawY) {
  std::map<std::pair<int,int>,AprilTags::TagFamily*>::const_iterator registryEntry =
    AprilTags::TagFamily::tagFamilyRegistry.find(curReq->aprilTagFamily);
  if ( registryEntry == AprilTags::TagFamily::tagFamilyRegistry.end() )
    return;
  AprilTags::TagFamily *tagFamily = registryEntry->second;
  std::vector<Shape<AprilTagData> > results = AprilTagData::extractAprilTags(rawY, *tagFamily);
  // calculate distance from the camera based on assumed marker height
  coordinate_t markerHeight = MapBuilderRequest::defaultMarkerHeight;
  for ( std::vector<Shape<AprilTagData> >::iterator it = results.begin();
				it != results.end(); it++ ) {
    Point c = (*it)->getCentroid();
    MarkerData::calculateCameraDistance(c, markerHeight);
    (*it)->setCentroid(c);
  }
}

void MapBuilder::getCamDominoes(const Sketch<uchar> &camFrame,
																const std::set<color_index>& objectColors,
																const std::set<color_index>& secondColors) {
  if ( objectColors.empty() ) 
    return;
  if ( curReq->verbosity & MBVshapeSearch )
    cout << "*** Find the dominoes ***" << endl;

  float saveMinLineLength = curReq->minLineLength;
	curReq->minLineLength = 35; // hard-coded based on Mauricio Contreras' domino demo
  vector<Shape<LineData> > camLines;
  // If secondColors is empty, assume the lines are the same colors as the dots.
	const set<color_index> &lineColors = (! secondColors.empty()) ? secondColors : objectColors;
  for (set<color_index>::const_iterator it = lineColors.begin();
       it != lineColors.end(); it++) {
    NEW_SKETCH_N(colormask,bool,visops::colormask(camFrame,*it));
    NEW_SKETCH_N(fatmask,bool,visops::fillin(colormask,1,2,8));
    NEW_SKETCH_N(skel,bool,visops::skel(fatmask));
    NEW_SKETCH_N(fatskel,bool,visops::fillin(skel,1,2,8));
		fatskel->retain(); fatmask->retain();
    vector<Shape<LineData> > newLines = LineData::extractLines(fatskel,fatmask);
		camLines.insert(camLines.end(),newLines.begin(),newLines.end());
	}
	curReq->minLineLength = saveMinLineLength;

	float saveMinEllipseSemiMajor = curReq->minEllipseSemiMajor;
	curReq->minEllipseSemiMajor = 3; // hard-coded from Maurcio Contreras' domino demo
  vector<Shape<EllipseData> > camEllipses;
  for (set<color_index>::const_iterator it = objectColors.begin();
       it != objectColors.end(); it++) {
    NEW_SKETCH_N(dotcolormask,bool,visops::colormask(camFrame,*it));
    vector<Shape<EllipseData> > ellipse_shapes = EllipseData::extractEllipses(dotcolormask);
    camEllipses.insert(camEllipses.end(), ellipse_shapes.begin(),ellipse_shapes.end());
	}
	curReq->minEllipseSemiMajor = saveMinEllipseSemiMajor;

	if ( camLines.empty() || camEllipses.empty() ) {
		if ( curReq->verbosity & MBVshapesFound )
			cout << "Found " << camLines.size() << " lines and "
					 << camEllipses.size() << " ellipses in camera space; no dominoes." << endl;
		SHAPEVEC_ITERATE(camLines, LineData, line) {
			line.deleteShape();
		} END_ITERATE;
		SHAPEVEC_ITERATE(camEllipses, EllipseData, ellipse) {
			ellipse.deleteShape();
		} END_ITERATE;
		return;
	}

# ifdef TGT_HAS_CAMERA
	  fmat::Transform camToBase = kine->linkToBase(CameraFrameOffset);
# else
	  fmat::Transform camToBase = fmat::Transform::identity();
# endif
	PlaneEquation groundPlane(fmat::pack(0,0,1), curReq->dominoDimensions[2]); // domino top face

	std::vector<Shape<LineData> > localLines;
	float minlen = curReq->dominoDimensions[1] * 0.65; // because actual line is narrower than brick
	float maxlen = curReq->dominoDimensions[1] * 1.1;
	SHAPEVEC_ITERATE(camLines, LineData, line) {
	  line->projectToGround(camToBase, groundPlane);
		float len = line->getLength();
		if ( len >= minlen && len <= maxlen )
			localLines.push_back(line);
	} END_ITERATE;

	std::vector<Shape<EllipseData> > localEllipses;
	SHAPEVEC_ITERATE(camEllipses, EllipseData, ellipse) {
		ellipse->projectToGround(camToBase, groundPlane);
		if ( ellipse->getSemimajor() < maxlen/6 ) // if not a giant (spurious) ellipse
			localEllipses.push_back(ellipse);
	} END_ITERATE;

	int dominoesFound = 0;
	if ( localLines.size() > 0 && localEllipses.size() > 0 ) {
		fmat::Column<3> halfdims = fmat::pack(curReq->dominoDimensions[0]/2,
																					curReq->dominoDimensions[1]/2,
																					curReq->dominoDimensions[2]/2);
		SHAPEVEC_ITERATE(localLines, LineData, line) {
			NEW_SHAPE(domino, DominoData,
								new DominoData(localShS, 0, 0,
															 fmat::pack(line->getCentroid().coordX(),
																					line->getCentroid().coordY(),
																					halfdims[2]),
															 halfdims, // extents
															 fmat::rotationZ(AngPi(line->getOrientation() + (AngPi)(M_PI/2)))));
			domino->setColor(localEllipses[0]->getColor());
			domino->setLineColor(line->getColor());
			domino->increaseConfidence(5); // *** hack
			LineData line1(localShS, domino->getTFL(), domino->getTFR());
			LineData line2(localShS, domino->getTBL(), domino->getTBR());
			std::vector<Point> points1, points2;
			points1.push_back(line1.getCentroid());
			points1.push_back(domino->getTFL());
			points1.push_back(domino->getTBL());
			points1.push_back(line2.getCentroid());
			points2.push_back(line1.getCentroid());
			points2.push_back(domino->getTFR());
			points2.push_back(domino->getTBR());
			points2.push_back(line2.getCentroid());
			PolygonData poly1(localShS, points1, true);
			PolygonData poly2(localShS, points2, true);
			int count1 = 0, count2 = 0;
			SHAPEVEC_ITERATE(localEllipses, EllipseData, ellipse) {
				if ( poly1.isInside(ellipse->getCentroid()) ) count1++;
				else if ( poly2.isInside(ellipse->getCentroid()) ) count2++;
			} END_ITERATE; // ellipses
			if ( count1 > 0 && count2 > 0 ) {
				domino->setValues(min(count1,count2), max(count1,count2));
				if ( domino->getLowValue() != count1 )
					domino->flipLeftRight();
				++dominoesFound;
			}
			else
				domino.deleteShape();
		} END_ITERATE; // lines
	}
	if ( curReq->verbosity & MBVshapesFound ) {
		cout << "Built " << dominoesFound << " local dominoes from " << localLines.size()
				 << " local lines and " << localEllipses.size() << " local ellipses";
		if ( camLines.size() != localLines.size() || camEllipses.size() != localEllipses.size() )
			cout << " (" << camLines.size() << " camera lines, " << camEllipses.size() << " camera ellipses)";
		cout << endl;
	}

	// Cleanup: delete the lines and ellipses so they don't get imported
  SHAPEVEC_ITERATE(camLines, LineData, line) {
		line.deleteShape();
	} END_ITERATE;
  SHAPEVEC_ITERATE(camEllipses, EllipseData, ellipse) {
    ellipse.deleteShape();
	} END_ITERATE;
}

void MapBuilder::getCamNaughts(const Sketch<uchar> &camFrame, 
															 const set<color_index>& objectColors, 
															 const fmat::Column<3>& dimensions) const {
  if (objectColors.empty())
    return;
  if ( curReq->verbosity & MBVshapeSearch )
    cout << "*** Find the naughts ***" << endl;
  for (set<color_index>::const_iterator it = objectColors.begin();
       it !=objectColors.end(); it++) {
    NEW_SKETCH_N(colormask,bool,visops::colormask(camFrame,*it));
    vector<Shape<NaughtData> > naughtShapes = 
      NaughtData::extractNaughts(colormask, dimensions);
    if ( curReq->verbosity & MBVshapesFound )
      cout << "Found " << naughtShapes.size() << " "
					 << ProjectInterface::getColorName(*it) << " naughts." << endl;
  }
}

void MapBuilder::getCamCrosses(const Sketch<uchar> &camFrame, 
                               const set<color_index>& objectColors,
                               const fmat::Column<3>& dimensions) const {
  if ( objectColors.empty() ) 
    return;
  if ( curReq->verbosity & MBVshapeSearch )
    cout << "*** Find the crosses ***" << endl;
  for (set<color_index>::const_iterator it = objectColors.begin();
       it !=objectColors.end(); it++) {
    NEW_SKETCH_N(colormask,bool,visops::colormask(camFrame,*it));
    NEW_SKETCH_N(cleancolor,bool,visops::minArea(colormask));
    NEW_SKETCH_N(fatmask,bool,visops::fillin(cleancolor,1,2,8));
    NEW_SKETCH_N(skel,bool,visops::skel(fatmask));
    std::vector<Shape<CrossData>> crossShapes = 
      CrossData::extractCrosses(camShS, skel, fatmask, dimensions);
    if ( curReq->verbosity & MBVshapesFound )
      cout << "Found " << crossShapes.size() << " "
					 << ProjectInterface::getColorName(*it) << " crosses." << endl;
  }
}

void MapBuilder::getCamAgents(const Sketch<uchar> &camFrame, const Sketch<yuv> &camFrameYUV,
															const std::set<color_index>& objectColors) const {
	if ( objectColors.empty() ) return;
	if ( curReq->verbosity & MBVshapeSearch )
		cout << "*** Find the agents ***" << endl;
	AgentData::extractAgents(camFrame, camFrameYUV, objectColors);
}

void MapBuilder::saveSiftDatabase(const std::string &siftDatabasePath) {
  if ( siftDatabasePath.empty() ) return;
  SiftTekkotsu *matcher = siftMatchers[siftDatabasePath];
  if ( matcher == NULL )
    cout << "Error in saveSiftDatabase: no database named '" << siftDatabasePath << "' has been loaded." << endl;
  else {
    matcher->saveToFile(siftDatabasePath, false);
    cout << "Wrote SIFT database " << siftDatabasePath << endl;
  }
}

void MapBuilder::trainSiftObject(const std::string &siftDatabasePath,
																 const std::string &objectName, const std::string &modelName) {
  NEW_SKETCH_N(rawY, uchar, VRmixin::sketchFromRawY());
  trainSiftObject(siftDatabasePath, rawY, objectName, modelName);
}

void MapBuilder::trainSiftObject(const std::string &siftDatabasePath, const Sketch<uchar> &sketch,
																 const std::string &objectName, const std::string &modelName) {
  if ( siftDatabasePath.empty() ) return;
  SiftTekkotsu *matcher = siftMatchers[siftDatabasePath];
  if ( matcher == NULL ) {
    cout << "Error in trainSiftObject:: no database named '" << siftDatabasePath << "' has been loaded." << endl;
    return;
  }
  ImageBuffer buffer = SiftTekkotsu::sketchToBuffer(sketch);
  int id = matcher->getObjectID(objectName);
  if ( id == -1 ) {
    int new_id = matcher->train_addNewObject(buffer);
    matcher->setObjectName(new_id, objectName);
  } else {
    matcher->train_addToObject(id, buffer);
  }
}

} // namespace

