#include "Motion/WalkMC.h"
#ifdef TGT_HAS_WALK

#include <vector>

#include "Crew/Pilot.h"
#include "DualCoding/VRmixin.h"
#include "Events/PilotEvent.h"
# include "Planners/Navigation/ShapeSpacePlannerXYTheta.h"
#include "Shared/mathutils.h" // for isnan fix
#include "Vision/VisualOdometry/VisualOdometry.h"

namespace DualCoding {

void Pilot::doStart() {
  if (verbosity & PVstart)
    cout << "Pilot starting up: walk_id= " << walk_id <<"  waypointwalk_id= " << waypointwalk_id << endl;
  setAgent(Point(0,0,0,allocentric),0,false,false);
  VRmixin::particleFilter->displayParticles(50);
  addNode(new RunMotionModel)->start();
  addNode(new CollisionChecker)->start();
  addNode(new RunOpticalFlow)->start();
}

void Pilot::doStop() {
  pilotAbort();
  if (verbosity & PVstart)
    cout << "Pilot is shutting down." << endl;
  motman->removeMotion(walk_id);
  motman->removeMotion(waypointwalk_id);
  waypointwalk_id = MotionManager::invalid_MC_ID;
  walk_id = MotionManager::invalid_MC_ID;
  delete defaultLandmarkExtractor;
  defaultLandmarkExtractor = NULL;

  // The following teardown() call is necessary because setup
  // hard-wires the MC_ID for all the walk nodes.  If we leave the
  // waypoint walk motion command active when Tekkotsu shuts down,
  // it crashes and generates a nasty backtrace.  But if we remove
  // the motion command, there's no easy way to put it back because
  // setup won't be called again unless we do this teardown() call.
  teardown();
}

void Pilot::doEvent() {
  if (event->getGeneratorID() == EventBase::timerEGID && event->getSourceID() == 9999)
    executeRequest();
  else
    cout << "Pilot got unexpected event: " << event->getDescription() << endl;
}

void Pilot::unwindForNextRequest() {
  erouter->addTimer(this,9999,1,false); // allow time to unwind event stack before executing next request
}

void Pilot::setAgent(const Point &loc, AngTwoPi heading, bool quiet, bool updateWaypoint) {
  if (updateWaypoint) {
    cout << "Updating waypoint walk to " << loc << " hdg=" << heading << endl;
    MMAccessor<WaypointWalkMC> wp_acc(waypointwalk_id);
    wp_acc->setCurPos(loc.coordX(), loc.coordY(), heading);
  }
  VRmixin::mapBuilder->setAgent(loc, heading, quiet);
  // Synching the estimate (below) is a no-op if setAgent was
  // invoked by Pilot::RunMotionModel, but not if the user called
  // setAgent directly.
  VRmixin::particleFilter->setPosition(loc.coordX(), loc.coordY(), heading);
  VRmixin::particleFilter->synchEstimateToAgent();
}

unsigned int Pilot::executeRequest(BehaviorBase* requestingBehavior, const PilotRequest& req) {
	VRmixin::requireCrew("Pilot");
  requests.push(new PilotRequest(req));
  const unsigned int reqID = ++idCounter;
  requests.back()->requestID = reqID;
  requests.back()->requestingBehavior = requestingBehavior;
  if (curReq == NULL)
    unwindForNextRequest();
  else if (verbosity & PVqueued)
    cout << "Pilot request " << requests.back()->requestID << " queued." << endl;
  return reqID;
}

void Pilot::executeRequest() {
  if (curReq == NULL && !requests.empty()) {
    curReq = requests.front();
    if (verbosity & PVexecute)
      cout << "Pilot executing request " << curReq->requestID
					 << ": " << PilotTypes::RequestTypeNames[curReq->requestType] << endl;
    VRmixin::particleFilter->synchEstimateToAgent();
    if ( ! curReq->walkParameters.empty() ) {
      MMAccessor<WaypointWalkMC>(VRmixin::pilot->getWaypointwalk_id())->WalkMC::loadFile(curReq->walkParameters.c_str());
      MMAccessor<WalkMC>(VRmixin::pilot->getWalk_id())->loadFile(curReq->walkParameters.c_str());
    }
    dispatch_->start();
  }
}

void Pilot::requestComplete(PilotTypes::ErrorType_t errorType) {
  if (curReq == NULL) {
    cout << "Pilot::requestComplete had NULL curReq !!!!!!!!!!!!!!!!" << endl;
    return;
  }
  const BehaviorBase* requester = curReq->requestingBehavior;
  PilotEvent e(EventBase::pilotEGID,
	       (requester==NULL) ? curReq->requestID : (size_t)requester,
	       EventBase::statusETID);
  e.requestType = curReq->requestType;
  e.errorType = errorType;
  if (verbosity & PVcomplete)
    cout << "Pilot request " << curReq->requestID << " complete: "
	 << PilotTypes::ErrorTypeNames[errorType] <<  endl;
  requestComplete(e);
}

void Pilot::requestComplete(const PilotEvent &e) {
  //if ( curReq->trackRequest != NULL )
  //  VRmixin::lookout.stopTrack();
  dispatch_->finish();
  // now clean up everything before posting the event
  delete curReq->landmarkExtractor;
  delete curReq->searchObjectExtractor;
  delete curReq;
  curReq = NULL;
  requests.pop();
  // post the event and allow time to unwind before starting on next event in the queue
  erouter->postEvent(e);
  VRmixin::autoRefreshSketchWorld();
  unwindForNextRequest();
}

void Pilot::setWorldBounds(float minX, float width, float minY, float height) {
  VRmixin::particleFilter->setWorldBounds(minX, width, minY, height);
}

void Pilot::setWorldBounds(const Shape<PolygonData> &bounds) {
  VRmixin::particleFilter->setWorldBounds(bounds);
}

void Pilot::randomizeParticles(float widthIncr, float heightIncr) {
  computeParticleBounds(widthIncr, heightIncr);
	VRmixin::particleFilter->resetFilter();
}

void Pilot::computeParticleBounds(float widthIncr, float heightIncr) {
	GET_SHAPE(worldBounds, PolygonData, VRmixin::worldShS);
	if ( worldBounds.isValid() )
		setWorldBounds(worldBounds);
	else {
		float minX=1e20, maxX=-minX, minY=1e20, maxY=-minY;
		SHAPEROOTVEC_ITERATE(VRmixin::worldShS, s) {
			BoundingBox2D b(s->getBoundingBox());
			minX = min(minX, b.min[0]);
			minY = min(minY, b.min[1]);
			maxX = max(maxX, b.max[0]);
			maxY = max(maxY, b.max[1]);
		} END_ITERATE;
		setWorldBounds(minX-widthIncr/2, maxX-minX+widthIncr,
									 minY-heightIncr/2, maxY-minY+heightIncr);
	}
}

void Pilot::setDefaultLandmarks(const std::vector<ShapeRoot> &landmarks) {
  defaultLandmarks = landmarks;
}

void Pilot::setDefaultLandmarkExtractor(const MapBuilderRequest &mapreq) {
  defaultLandmarkExtractor = new MapBuilderRequest(mapreq);
  defaultLandmarkExtractor->clearLocal = false;
}

void Pilot::pilotPop() {
  if ( curReq != NULL ) {
    if ( verbosity & PVpop )
      cout << "Pilot popping current request." << endl;
    dispatch_->stop();
    VRmixin::pilot->requestComplete(PilotTypes::abort);
  }
}

void Pilot::pilotAbort() {
  if ( verbosity & PVabort ) {
		if ( curReq == NULL )
			cout << "Pilot aborting but there is no current request." << endl;
		else
			cout << "Pilot aborting." << endl;
	}
  dispatch_->stop();
  VRmixin::isWalkingFlag = false;
	if ( curReq != NULL )
		VRmixin::pilot->requestComplete(abort);
  while (!requests.empty()) {
    delete requests.front();
    requests.pop();
  }
}

void Pilot::setupLandmarkExtractor() {
  if ( curReq->landmarkExitTest == NULL )
    curReq->landmarkExitTest = &Pilot::defaultLandmarkExitTest;
  if ( curReq->landmarkExtractor != NULL )  // user supplied his own, so go with that
    return;
  else if ( defaultLandmarkExtractor != NULL ) {
    curReq->landmarkExtractor = new MapBuilderRequest(*defaultLandmarkExtractor);
    return;
  }
  vector<ShapeRoot> landmarkTemp;
  if ( ! curReq->landmarks.empty() )
    landmarkTemp = curReq->landmarks;
  else if ( ! defaultLandmarks.empty() )
    landmarkTemp = defaultLandmarks;
  else {
    const vector<ShapeRoot> &wshapes = VRmixin::worldShS.allShapes();
    for ( vector<ShapeRoot>::const_iterator it = wshapes.begin();
	  it != wshapes.end(); it++ )
      if ( (*it)->isLandmark() )
				landmarkTemp.push_back(*it);
  }
  if ( landmarkTemp.empty() ) {
    // the user specified no landmarks, so look in worldShS for possibilities

    NEW_SHAPEVEC(markers, MarkerData, select_type<MarkerData>(VRmixin::worldShS));
    landmarkTemp.insert(landmarkTemp.begin(), markers.begin(), markers.end());
    NEW_SHAPEVEC(apriltags, AprilTagData, select_type<AprilTagData>(VRmixin::worldShS));
    landmarkTemp.insert(landmarkTemp.begin(), apriltags.begin(), apriltags.end());
  }
  if ( !landmarkTemp.empty() ) {
    curReq->landmarks = landmarkTemp;  // save what we found so the navigation planner can use them
    curReq->landmarkExtractor = new MapBuilderRequest(MapBuilderRequest::localMap);
    curReq->landmarkExtractor->clearVerbosity = -1U;
    curReq->landmarkExtractor->setVerbosity = MapBuilder::MBVimportShapes;
    for ( vector<ShapeRoot>::const_iterator it = landmarkTemp.begin();
	  it != landmarkTemp.end(); it++ )
      curReq->landmarkExtractor->addAttributes(*it);
  }
}

bool Pilot::defaultLandmarkExitTest() {
	// Test to see if we have enough landmarks to localize.  We should
	// be more selective than this, but for now, count the localShS shapes and
	// subtract any point shapes, and any AprilTags whose tag ID doesn't match a
	// declared landmark.  In the future we could extend this by matching color
	// for ellipses/lines/cylinders, checking tag family, maybe more.
	std::vector<ShapeRoot> &locs = VRmixin::localShS.allShapes();
	int n = locs.size();
	SHAPEROOTVEC_ITERATE(locs, s) {
		if ( s->isType(pointDataType) )
			--n;
		else if ( s->isType(aprilTagDataType) ) {
			n--;
			SHAPEROOTVEC_ITERATE(VRmixin::pilot->curReq->landmarks, lm) {
				if ( lm->isType(aprilTagDataType) &&
						 ShapeRootTypeConst(lm,AprilTagData)->getTagID() ==
						 ShapeRootTypeConst(s,AprilTagData)->getTagID() ) {
					n++;
					break;
				} END_ITERATE;
			}
		}
	} END_ITERATE;
	// std::cout << "defaultLandmarkExitTest: locs=" << locs.size()
	//			<< "  lms=" << VRmixin::pilot->curReq->landmarks.size() << " n=" << n << std::endl;
  return ( n >= 2 );
}

std::vector<ShapeRoot>
  Pilot::calculateVisibleLandmarks(const DualCoding::Point &currentLocation,
				   AngTwoPi currentOrientation, AngTwoPi maxTurn,
				   const std::vector<DualCoding::ShapeRoot> &possibleLandmarks) {
  std::vector<ShapeRoot> result;
  for(unsigned int i = 0; i<possibleLandmarks.size(); i++)
    if ( isLandmarkViewable(possibleLandmarks[i], currentLocation, currentOrientation, maxTurn) )
      result.push_back(possibleLandmarks[i]);
  return result;
}

bool Pilot::isLandmarkViewable(const DualCoding::ShapeRoot &selectedLandmark,
			       const DualCoding::Point &currentLocation,
			       AngTwoPi currentOrientation, AngTwoPi maxTurn) {
  AngSignPi bearing = (selectedLandmark->getCentroid() - currentLocation).atanYX();

  // is the camera within our possible field of view (in front of us)?
  if ( fabs(bearing) > (CameraHorizFOV/2.0+maxTurn) )
    return false;

  // If there are no walls, assume all landmarks are visible,
  // but this isn't really true for poster-type markers where
  // we need to know the viewing angle.  Will fix this someday.
  Shape<PolygonData> boundary;   // either worldBounds or wall
  GET_SHAPE(worldBounds, PolygonData, VRmixin::worldShS);
  if ( worldBounds.isValid() )
    boundary = worldBounds;
  else {
    GET_SHAPE(wall, PolygonData, VRmixin::worldShS);
    boundary = wall;
  }
  if ( ! boundary.isValid() )
    return true;
  // do we intersect with the world bounds when looking at marker?
  LineData lineOfSight(VRmixin::worldShS, currentLocation, selectedLandmark->getCentroid());
  if ( boundary->intersectsLine(lineOfSight) )
    return false;

  // this landmark is viewable
  return true;
}

void Pilot::CollisionDispatch::doStart() {
  switch ( VRmixin::pilot->curReq->collisionAction ) {
  case collisionStop:
  case collisionReplan:
    erouter->addListener(this, EventBase::pilotEGID, (size_t)VRmixin::pilot, EventBase::statusETID);
    break;
  default:
    break;
  }
}

void Pilot::CollisionDispatch::doEvent() {
  // If we're listening for Pilot collisions then we want to stop or
  // replan, so rebroadcast with this as the source node to trigger
  PilotEvent e(*event);
  e.setSourceID((size_t)this);
  erouter->postEvent(e);
}

void Pilot::TerminateDueToCollision::doStart() {
  PilotEvent e(*event);  // copy and convert from an internal Pilot event to a public one
  e.setSourceID((size_t)VRmixin::pilot->curReq->requestingBehavior);
  VRmixin::isWalkingFlag = false;
  VRmixin::pilot->requestComplete(e);
}

//================ Background processes ================

void Pilot::RunMotionModel::doStart() {
#ifdef TGT_IS_CREATE
  unsigned int odometryInterval = 250;  // slow updates because of Create odometry bug (angle updatng)
#else
  unsigned int odometryInterval = 32;   // fast updates for better accuracy
#endif
  erouter->addTimer(this, 1, odometryInterval, true);
}

void Pilot::RunMotionModel::doEvent() {
  if (event->getGeneratorID() == EventBase::timerEGID) {
    VRmixin::particleFilter->updateMotion();
    LocalizationParticle estimate = VRmixin::particleFilter->getEstimate();
    VRmixin::mapBuilder->setAgent(Point(estimate.x, estimate.y, 0, allocentric), estimate.theta, true);
  }
}

void Pilot::RunOpticalFlow::doStart() {
  if ( VRmixin::imageOdometry != NULL )
    erouter->addTimer(this, 1, VRmixin::imageOdometry->suggestedFrameRate(), true);
}

void Pilot::RunOpticalFlow::doEvent() {
  if (event->getGeneratorID() == EventBase::timerEGID)
    VRmixin::imageOdometry->update();
}

void Pilot::CollisionChecker::doStart() {
  enableDetection();
}

void Pilot::CollisionChecker::enableDetection() {
#if defined(TGT_IS_CREATE) || defined(TGT_IS_CREATE2)
  // std::cout << "Enabled collision detection." << std::endl;
  erouter->addListener(this, EventBase::buttonEGID, BumpLeftButOffset, EventBase::activateETID);
  erouter->addListener(this, EventBase::buttonEGID, BumpRightButOffset, EventBase::activateETID);
  erouter->addListener(this, EventBase::buttonEGID, OvercurrentLeftWheelOffset, EventBase::activateETID);
  erouter->addListener(this, EventBase::buttonEGID, OvercurrentRightWheelOffset, EventBase::activateETID);
  erouter->addTimer(this, overcurrentResetTimer, 1500, true); // reset overcurrentCounter frequently
#else
  std::cout << "Warning: Pilot has no collision check method defined for this robot type" << std::endl;
#endif
}

void Pilot::CollisionChecker::disableDetection(unsigned int howlong=-1U) {
#if defined(TGT_IS_CREATE) || defined(TGT_IS_CREATE2)
  // std::cout << "Disabling collision detection for " << howlong << " ms." << std::endl;
  erouter->removeListener(this, EventBase::buttonEGID);
#endif
  erouter->addTimer(this, 1, howlong, false);
}

void Pilot::CollisionChecker::doEvent() {
  if ( event->getGeneratorID() == EventBase::timerEGID ) {
    if ( event->getSourceID() == collisionEnableTimer )
      enableDetection();
#if defined(TGT_IS_CREATE) || defined(TGT_IS_CREATE2)
    else if ( event->getSourceID() == overcurrentResetTimer )
      overcurrentCounter = 0;
    return;
#endif
  }
  // If we get here, event must have been a button press (bump senssor or overcurrent)
  if ( VRmixin::pilot->curReq == NULL || 
       VRmixin::pilot->curReq->collisionAction == collisionIgnore )
    return;
#if defined(TGT_IS_CREATE) || defined(TGT_IS_CREATE2)
  if ( event->getSourceID() == BumpLeftButOffset ||
       event->getSourceID() == BumpRightButOffset )
    reportCollision();
  else // overcurrent -- could be transient.  Count 'em and see.
    if ( ++overcurrentCounter >= 12 ) {
      overcurrentCounter = 0;
      reportCollision();
    }
#endif
}

void Pilot::CollisionChecker::reportCollision() {
  if ( VRmixin::pilot->verbosity & PVcollision )
    cout << "Pilot: collision detected!" << endl;
  disableDetection(1000); // ignore any further collisions for the next second
  PilotEvent e(EventBase::pilotEGID, (size_t)VRmixin::pilot, EventBase::statusETID);
  e.errorType = collisionDetected;
  // Note: in the future we might want to add info about where on the
  // robot's body the collision occurred.
  erouter->postEvent(e);
}

void Pilot::PlanPath::clearGraphics() {
	GET_SHAPE(plannedPath, GraphicsData, VRmixin::worldShS);
	plannedPath.deleteShape();
	GET_SHAPE(plannerTrees, GraphicsData, VRmixin::worldShS);
	plannerTrees.deleteShape();
}

void Pilot::PlanPath::doPlan(NavigationPlan &plan, Point initPoint, AngTwoPi initHead, PilotRequest &req) {
  plan.clear();
	clearGraphics();
  Point targetPoint = req.targetShape->getCentroid();
  if ( targetPoint.getRefFrameType() == egocentric ) {
    targetPoint.applyTransform(VRmixin::mapBuilder->localToWorldMatrix);
    targetPoint.setRefFrameType(allocentric);
  }
	AngTwoPi targetHeading = req.targetHeading;
	fmat::Column<3> baseOffset = req.baseOffset;

  const bool planForHeading = ! std::isnan((float)targetHeading);
  std::cout << "Planning path from " << initPoint << " to point: " << targetPoint;
  if ( baseOffset != fmat::ZERO3 )
    cout << " target offset " << baseOffset;
  if ( planForHeading )
    cout << " heading " << float(targetHeading)/M_PI*180 << " deg.";
  cout << std::endl;

  /* Decide if we should walk backward: do this if the distance is
	 less than the max allowed, and the heading change if we don't would
	 be > 100 degrees, and the target heading doesn't require a
	 reversal.  Walking backward requires planning a forward path from
	 the target to the initial position, then reversing both the path
	 points and the direction of travel.

	 We can't use the planner's baseOffset feature if we're walking backwards,
	 so we must apply the baseOffset here instead.
	 
	 *** NOTE: walking backward without a target heading isn't supported yet.

	*/
	Point baseTemp = Point(fmat::rotationZ(targetHeading)*baseOffset, allocentric);
  Point straightVec = (targetPoint - baseTemp) - initPoint;
  float straightDist = straightVec.xyNorm();
  AngTwoPi straightAng = float(straightVec.atanYX());
  AngSignPi straightAngDiff = float(straightAng - initHead);
  if ( req.allowBackwardMotion && planForHeading &&
       straightDist <= req.maxBackwardDistance && 
			 abs(straightAngDiff) > M_PI/1.8 &&
			 abs(AngSignPi(targetHeading - initHead)) < M_PI/1.8 ) {
    plan.walkBackward = true;
    std:: cout << "Walking backward..." << std::endl;
  }
	
  Point gatePoint = targetPoint;
  if ( plan.walkBackward ) {
		gatePoint = targetPoint - baseTemp;
		baseOffset = fmat::ZERO3;
    swap(initPoint, gatePoint);
		swap(initHead, targetHeading);
	}

  // set up and call the XYTheta path planner
  GET_SHAPE(worldBounds, PolygonData, VRmixin::worldShS);
  ShapeSpacePlannerXYTheta planner(VRmixin::worldShS, worldBounds, req.obstacleInflation);
  std::vector<NodeValue_t> *pathResult = new std::vector<NodeValue_t>;
  std::vector<NodeType_t> *treeStartResult = req.displayTree ? new std::vector<NodeType_t> : NULL;
  std::vector<NodeType_t> *treeEndResult = req.displayTree ? new std::vector<NodeType_t> : NULL;
  GenericRRTBase::PlannerResult2D result =
    planner.planPath(initPoint, baseOffset, req.gateLength,
										 gatePoint, initHead, targetHeading, req.maxRRTIterations,
										 pathResult, treeStartResult, treeEndResult);
  if ( plan.walkBackward ) {
    if ( result.code == GenericRRTBase::SUCCESS ) {
			swap(initPoint, gatePoint); // restore original values
			swap(initHead, targetHeading);
			// Reverse the path.  This is a little tricky because the first
			// element must be the starting position (not a movement step)
			// and turns must still PRECEDE translations.
			NodeValue_t &final = (*pathResult)[pathResult->size()-1];
			NodeValue_t &preFinal = (*pathResult)[pathResult->size()-2];
			std::vector<NodeValue_t> *revPath = new std::vector<NodeValue_t>;
			revPath->reserve(pathResult->size()+1);
			revPath->push_back(NodeValue_t(final.x, final.y, final.theta, 0));
			revPath->push_back(NodeValue_t(preFinal.x, preFinal.y, final.theta, 0));
			for ( int i = pathResult->size()-3; i >= 0; i-- )
				revPath->push_back(NodeValue_t((*pathResult)[i].x, (*pathResult)[i].y,
																			 (*pathResult)[i+1].theta, -(*pathResult)[i+2].turn));
			revPath->push_back(NodeValue_t((*pathResult)[0].x, (*pathResult)[0].y,
																		 (*pathResult)[0].theta, -(*pathResult)[1].turn));
			delete pathResult;
			pathResult = revPath;
		} else if ( result.code == GenericRRTBase::START_COLLIDES )
      result.code = GenericRRTBase::END_COLLIDES;
    else if ( result.code == GenericRRTBase::END_COLLIDES )
      result.code = GenericRRTBase::START_COLLIDES;
  } else // not walking backward
		if ( result.code == GenericRRTBase::SUCCESS && planForHeading ) {
			// add final segment from gate point to target point
			AngSignPi headingChange = AngSignPi(targetHeading - pathResult->back().theta);
			AngSignTwoPi turn;
			if ( RRTNodeXYTheta::safeTurn(pathResult->back(), headingChange, turn, planner.getCC()) ) {
				fmat::Column<3> offsetTarget = targetPoint.getCoords() - fmat::rotationZ(targetHeading) * baseOffset;
				NodeValue_t final(offsetTarget[0],offsetTarget[1],targetHeading, turn);
				pathResult->push_back(final);
			}
			else
				result.code = GenericRRTBase::END_COLLIDES;
  }

  if ( req.displayTree ) {
    NEW_SHAPE(plannerTrees, GraphicsData, new GraphicsData(VRmixin::worldShS));
    planner.plotTree(*treeStartResult, plannerTrees, rgb(0,0,0));
    planner.plotTree(*treeEndResult, plannerTrees, rgb(0,192,0));
    delete treeStartResult;
    delete treeEndResult;
  }
  if ( req.displayObstacles || req.autoDisplayObstacles ) {
    if ( result.code == GenericRRTBase::END_COLLIDES ) {
      VRmixin::robotObstaclesPt = targetPoint;
      VRmixin::robotObstaclesOri = planForHeading ? targetHeading : AngTwoPi(0);
      planner.addObstaclesToShapeSpace(VRmixin::worldShS);
    } else if ( req.displayObstacles || result.code != GenericRRTBase::SUCCESS ) {
      // for START_COLLIDES or MAX_ITER, or if user explicitly asked for obstacle display
      VRmixin::robotObstaclesPt = VRmixin::theAgent->getCentroid();
      VRmixin::robotObstaclesOri = VRmixin::theAgent->getOrientation();
      planner.addObstaclesToShapeSpace(VRmixin::worldShS);
    }
  }

  if ( result.code != GenericRRTBase::SUCCESS ) {
    switch ( result.code ) {
    case GenericRRTBase::START_COLLIDES:
      std::cout << "Navigation path planning failed: start state " << result.movingObstacle->toString()
		<< " is in collision with " << result.collidingObstacle->toString() << ".\n";
      break;
    case GenericRRTBase::END_COLLIDES:
      std::cout << "Navigation path planning failed: end state " << result.movingObstacle->toString()
		<< " is in collision with " << result.collidingObstacle->toString() << ".\n";
      break;
    case GenericRRTBase::MAX_ITER:
      std::cout << "Navigation path planning failed: max iterations " << req.maxRRTIterations << " exceeded.\n";
      break;
    default: break; // dummy to suppress compiler warning
    }
    postStateSignal<GenericRRTBase::PlanPathResultCode>(result.code);
    return;
  }

  // Let user edit the path if desired, then save in the nav plan
  if ( req.pathEditor )
    (*req.pathEditor)(pathResult, req);
  plan.path = *pathResult;
  delete pathResult;

  // Print the (possibly edited) path
  if ( verbosity & PVshowPath )
		showPath(plan.path);
  // Display the path in worlShS if requested
  if ( req.displayPath ) {
    NEW_SHAPE(plannedPath, GraphicsData, new GraphicsData(VRmixin::worldShS));
    ShapeSpacePlannerXYTheta::plotPath(plan.path, plannedPath, rgb(0,0,255));
  }
  
  // Now execute the plan, unless requested not to
  if ( req.executePath )
    postStateCompletion();
  else
    postStateFailure();
}

void Pilot::PlanPath::showPath (const vector<RRTNodeXYTheta::NodeValue_t> &path) {
	if ( path.empty() )
		std::cout << "[Empty navigation path]" << std::endl;
	else
		for ( size_t i = 0; i < path.size(); i++ ) {
      Point pt(path[i].x, path[i].y, 0, allocentric);
      cout << "path[" << i << "] = ";
			if ( i == 0 )
				std::cout << "start at ";
			else
				std::cout << "turn " << float(path[i].turn)*180/M_PI << " deg., then ";
			std::cout << pt << " hdg " << float(AngTwoPi(path[i].theta)) * 180/M_PI << " deg." << std::endl;
    }
}

void Pilot::ConstructNavPlan::doAnalysis(NavigationPlan &plan, Point initPoint, 
					 AngTwoPi initHead, PilotRequest &req) {
  Point currentPosition = initPoint;
  plan.steps.clear();

  for (unsigned int i = 1; i<(plan.path.size()); i++) {

    // Calculate vector between this and next point on path
    Point delta = Point(plan.path[i].x - plan.path[i-1].x,
			plan.path[i].y - plan.path[i-1].y,
			0, allocentric);
    const float distanceBetween = delta.xyNorm();
    const AngSignTwoPi turn = plan.path[i].turn;
    //std::cout << "### delta=" << delta << "  orientation=" << orientation
    //	      << "  headingChange=" << headingChange;
    
    // If this step involves a non-negligible heading change, turn  first
    //if( fabs(turn) > M_PI/18 )
    //  plan.addNavigationStep(turnStep, plan.path[i], req.landmarks);
    // If we have a pan/tilt and we made a large turn, relocalize
#ifdef TGT_HAS_HEAD
    if ( fabs(turn) > M_PI / 6 )
      if ( req.landmarks.size() > 0)
	plan.addNavigationStep(localizeStep, plan.path[i], req.landmarks);
#endif

    // Calculate the number of times to localize during travel, but
    // don't try to localize if we have no landmarks.
    float maxDistanceBetween = 500;   //!< farthest we're willing to travel before localizing again
    //if ( req.landmarks.empty() ) {
    //  maxDistanceBetween = 1e10f;
    //}
    int numberOfLocalizations = (int)floor(distanceBetween / maxDistanceBetween);
    Point deltaStep = delta.unitVector() * maxDistanceBetween;
    Point currentPoint = Point(plan.path[i-1].x, plan.path[i-1].y, 0, allocentric);
    for ( int j=0; j < numberOfLocalizations; j++ ) {
      currentPoint += deltaStep;
      plan.addNavigationStep(travelStep, currentPoint, plan.path[i].theta, 0, req.landmarks);
      if ( req.landmarks.size() > 0)
	plan.addNavigationStep(localizeStep, currentPoint, plan.path[i].theta, 0, req.landmarks);
    }
    plan.addNavigationStep(travelStep, plan.path[i], req.landmarks);
  }
  AngTwoPi finalHeading = std::isnan((float)req.targetHeading) ? plan.path.back().theta : req.targetHeading;
  plan.addNavigationStep(headingStep, Point(), finalHeading, 0, req.landmarks);
  plan.currentStep = plan.steps.begin();

  if ( req.planEditor )  // Let user edit the plan if desired
    (*req.planEditor)(plan, req);
  postStateCompletion();
}

void Pilot::ExecutePlan::ExecuteStep::doExecute(NavigationPlan &plan, 
						DisplacementInstruction &nextDisplacementInstruction,
						const bool pushType) {

  //std::cout << "--------ExecutePlan::ExecuteStep::doExecute" << std::endl;
  DualCoding::Point position = VRmixin::theAgent->getCentroid();

  if (plan.steps.size() == 0) {
    std::cout << "Warning: empty path in Pilot::ExecutePlan::doStart" << std::endl;
    postStateCompletion();
    return;
  }

  nextDisplacementInstruction.walkBackward = plan.walkBackward;
  switch (plan.currentStep->type) {

  case localizeStep:
    if (!pushType)
        postStateSignal<NavigationStepType_t>(localizeStep);
    else
#ifdef TGT_HAS_HEAD
        postStateSignal<NavigationStepType_t>(localizeStep);
#else
        postStateSuccess();
#endif
    break;

  case travelStep: {
    position = VRmixin::theAgent->getCentroid();
    fmat::Column<3> pos = position.getCoords();
    fmat::Column<3> next = plan.currentStep->waypoint.getCoords();			
    fmat::Column<2> disp = fmat::SubVector<2>(next) - fmat::SubVector<2>(pos);
    AngSignPi course = atan2(disp[1], disp[0]);
    if ( verbosity & PVnavStep )
      cout << ">>> Travel: from " << pos
	   << " hdg " << float(AngSignPi(VRmixin::theAgent->getOrientation()))*180/M_PI 
	   << " deg., course " << float(course)*180/M_PI <<" to " << next;
    if (position.xyDistanceFrom(plan.currentStep->waypoint) < allowableDistanceError) {
      if ( verbosity & PVnavStep )
				cout << ".   Close enough!" << endl;
      postStateSuccess();
    } else {
      AngSignPi angle = course - VRmixin::theAgent->getOrientation();
      if ( nextDisplacementInstruction.walkBackward ) {
				angle += M_PI;
				cout << "nextDisplacementInstruction.walkBackward is true" << endl;
			}
      if (fabs(angle) < allowableAngularError) {
        nextDisplacementInstruction.nextPoint = disp;
        if ( verbosity & PVnavStep )
					cout << ", disp=" << disp << endl;
        postStateSignal<NavigationStepType_t>(travelStep);
      }
      else {
        if ( verbosity & PVnavStep )
					cout << "... deferred" << endl << ">>> Turn before travel: "
							 << float(angle)*180/M_PI << " deg" << endl;
        nextDisplacementInstruction.angleToTurn = angle;
        postStateSignal<NavigationStepType_t>(preTravelStep);
      }
    }
    break;
  }

  case turnStep: {
    position = VRmixin::theAgent->getCentroid();
    fmat::Column<3> pos = position.getCoords();
    fmat::Column<3> next = plan.currentStep->waypoint.getCoords();
    fmat::Column<2> disp = fmat::SubVector<2>(next) - fmat::SubVector<2>(pos);
    AngSignPi directTurn = AngSignPi(atan2(disp[1], disp[0]) - VRmixin::theAgent->getOrientation());
    if ( fabs(AngSignPi(directTurn - plan.currentStep->turn)) > M_PI/1.8 )
	 directTurn += M_PI;  // intent was to walk backwards
    AngSignTwoPi actualTurn = float(directTurn);

    // The amount of the turn is calculated dynamically based on our
    // actual position.  Its sign may change relative to the planned
    // turn due to position error.  But to assure obstacle avoidance,
    // if the planned turn amount is more than 90 degrees, the sign is
    // unlikely to have changed, and the planned turning direction
    // (left or right) will be honored even if we have to turn more
    // than 180 degrees.
    if ( fabs(plan.currentStep->turn) > M_PI/2 ) {
      if ( directTurn > 0 && plan.currentStep->turn < 0 )
	actualTurn -= 2*M_PI;
      else if ( directTurn < 0 && plan.currentStep->turn > 0 )
	actualTurn += 2*M_PI;
    }

    if (fabs(actualTurn) < allowableAngularError) {
      if ( verbosity & PVnavStep )
	cout << ">>> Turn: " << float(actualTurn)*180/M_PI << " deg is smaller than minimum "
	     << float(allowableAngularError)*180/M_PI << " deg (skipped)" << endl;
      postStateSuccess();
    } else {
      if ( verbosity & PVnavStep )
	cout << ">>> Turn: " << float(actualTurn)*180/M_PI << " deg" << endl;
      nextDisplacementInstruction.angleToTurn = actualTurn;
      postStateSignal<NavigationStepType_t>(turnStep);
    }
    break;
    }

  case headingStep: {
    AngSignPi angle = float(plan.currentStep->orientation - VRmixin::theAgent->getOrientation());
    if (fabs(angle) < allowableAngularError) {
      if ( verbosity & PVnavStep )
	cout << ">>> Turn: " << float(angle)*180/M_PI << " deg is smaller than minimum "
	     << float(allowableAngularError)*180/M_PI << " deg (skipped)" << endl;
      postStateSuccess();
    } else {
      if ( verbosity & PVnavStep )
	cout << ">>> Turn: " << float(angle)*180/M_PI 
	     << " deg to heading " << float(plan.currentStep->orientation)*(180/M_PI) << endl;
      nextDisplacementInstruction.angleToTurn = angle;
      postStateSignal<NavigationStepType_t>(headingStep);
    }
    break;
  }

  default:
    cout << "Incorrect step type " << plan.currentStep->type << "in ExecuteStep" << endl;
  }
}

void Pilot::ExecutePlan::AdjustHeading::doAdjust(NavigationPlan &plan) {
	cout << "AdjustHeading: navplan = " << plan.toString() << endl;
	DualCoding::Point position = VRmixin::theAgent->getCentroid();
	fmat::Column<3> pos = position.getCoords();

	fmat::Column<3> cur, next;
	cur = plan.currentStep->waypoint.getCoords();
	if (adjustmentType == collisionAdjust) {
		next = plan.currentStep->waypoint.getCoords();
		cout << "*************** ADJUST TURN COLLISION ***********************************" << endl;
	} else {
		plan.currentStep++;
		next = plan.currentStep->waypoint.getCoords();
		plan.currentStep--;
	}

	// *** Should adjust for walking backward

	fmat::Column<2> disp = fmat::SubVector<2>(next) - fmat::SubVector<2>(pos);
	AngTwoPi newHeading = atan2(disp[1], disp[0]);
	AngSignPi turnAngle = float(newHeading - VRmixin::theAgent->getOrientation());
	//std::cout << plan << std::endl;
	std::cout << "AdjustHeading: pos = " << pos << "  curWP = " << cur << "  nextWP = " << next << std::endl;
	std::cout << "AdjustHeading: disp=" << disp << "  new heading " << newHeading
						<< " (" << float(newHeading)*180/M_PI << " deg.); turn by "
						<< float(turnAngle)*180/M_PI << " deg." << std::endl;
	if ( plan.currentStep != plan.steps.begin() && abs(turnAngle) > 45.0*(M_PI/180.0) ) {
		std::cout << "AdjustHeading: overshot target; won't try to correct heading now." << std::endl;
		postStateCompletion();
		return;
	}
	if ( (disp.norm() < 50.0 && abs(turnAngle) > 5.0*(M_PI/180.)) ||
       abs(turnAngle) < 5.0*(M_PI/180.) ) {
		std::cout << "AdjustHeading: punting insignificant turn." << std::endl;
		postStateCompletion();
		return;
	}
	PilotRequest &curReq = *VRmixin::pilot->curReq;
#if defined(TGT_IS_CREATE) || defined(TGT_IS_CREATE2)
	float va = (curReq.turnSpeed > 0) ? curReq.turnSpeed : 0.25f;
#else
	// these values are for Chiara
	float va = (curReq.turnSpeed > 0) ? curReq.turnSpeed : 0.1f;
#endif
	getMC()->setTargetDisplacement(0,0,turnAngle,0,0,va);
	motman->setPriority(VRmixin::pilot->getWalk_id(),MotionManager::kStdPriority);
}



//================ LocalizationUtility ================

const float Pilot::LocalizationUtility::Localize::minConfidentWeight = -30;

#ifdef TGT_HAS_HEAD
void Pilot::LocalizationUtility::ChooseGazePoints::setupGazePoints(std::deque<Point> &gazePoints) {
  std::vector<ShapeRoot> visibleLandmarks =
    calculateVisibleLandmarks(VRmixin::theAgent->getCentroid(), VRmixin::theAgent->getOrientation(), 
			      M_PI*(2.0/3.0), // assume head can turn 120 degrees left or right
			      VRmixin::pilot->curReq->landmarks);
  gazePoints = std::deque<Point>();
  for ( unsigned int i = 0; i < visibleLandmarks.size(); i++ ) {
    // convert to egocentric coordinates
    Point allovec = visibleLandmarks[i]->getCentroid() - VRmixin::theAgent->getCentroid();
    Point egovec = Point(fmat::rotationZ(- VRmixin::theAgent->getOrientation()) * allovec.getCoords());
    egovec.setRefFrameType(egocentric);
    gazePoints.push_back(egovec);
  }
  std::cout << "Pilot wants to localize: now at " << VRmixin::theAgent->getCentroid()
	    << " hdg. " << VRmixin::theAgent->getOrientation() << " "
	    << float(VRmixin::theAgent->getOrientation())*180/M_PI << " deg. with " 
	    << gazePoints.size() << " gaze points." << std::endl;
}

#else  // robot has no head, so plan on turning the body
void Pilot::LocalizationUtility::ChooseGazePoints::setupGazeHeadings
                (const AngTwoPi initialHeading, std::deque<AngTwoPi> &gazeHeadings, const AngTwoPi maxTurn) {
  const AngTwoPi robotHeading = VRmixin::theAgent->getOrientation();
  std::vector<ShapeRoot> visibleLandmarks =
    calculateVisibleLandmarks(VRmixin::theAgent->getCentroid(), robotHeading, 
			      maxTurn, // assume body can turn the full +/- 180 degrees
			      VRmixin::pilot->curReq->landmarks);
  gazeHeadings = std::deque<AngTwoPi>();
  for ( unsigned int i = 0; i < visibleLandmarks.size(); i++ ) {
    Point allovec = visibleLandmarks[i]->getCentroid() - VRmixin::theAgent->getCentroid();
    gazeHeadings.push_back(AngTwoPi(allovec.atanYX()));
  }
  // Landmark headings are allocentric, but to minimize turn distance
  // we want to sort them by their bearing relative to the robot.
  std::sort(gazeHeadings.begin(), gazeHeadings.end(), BearingLessThan(robotHeading));
  // Reverse the list if the closest landmark is to the right instead of the left
  if ( gazeHeadings.size() > 1 &&
       abs(AngSignPi(gazeHeadings.front()-robotHeading)) > abs(AngSignPi(gazeHeadings.back()-robotHeading)) )
    reverse(gazeHeadings.begin(), gazeHeadings.end());
}

void Pilot::LocalizationUtility::PrepareForNextGazePoint::prepareForNext(std::deque<AngTwoPi> &gazeHeadings, const AngTwoPi maxTurn) {
  if ( gazeHeadings.empty() ) {
    postStateFailure();
    return;
  }
  const AngTwoPi curHeading = VRmixin::theAgent->getOrientation();
  const AngTwoPi newHeading = gazeHeadings.front();
  gazeHeadings.pop_front();
  const AngSignPi turnAmount = AngSignPi(newHeading - curHeading);
  // std::cout << "*** curHeading=" << curHeading << "  newHeading=" << newHeading
	// 					<< "  turnAmount=" << turnAmount << "  maxturn=" << maxTurn << std::endl;
  if (fabs(turnAmount) > maxTurn) {
    postStateSuccess();
    return;
  }
  MMAccessor<WalkMC> walk_acc(getMC());
  float va = VRmixin::pilot->curReq->turnSpeed;
  if ( va == 0 )
    va = 1.0f;   // good turn speed for CREATE odometry
  walk_acc->setTargetDisplacement(0, 0, turnAmount, 0, 0, va);
  motman->setPriority(VRmixin::pilot->getWalk_id(),MotionManager::kStdPriority);
}

#endif // TGT_HAS_HEAD

  void Pilot::LocalizationUtility::Localize::doStart() {
    VRmixin::particleFilter->updateSensors(VRmixin::particleFilter->getSensorModel(),false,true);
    const ShapeBasedParticleFilter::particle_type &estimate = VRmixin::particleFilter->getEstimate();
    if ( estimate.weight < minConfidentWeight ) {
      std::cout << "  Particle filter bestWeight=" << estimate.weight
		<< " < minConfidentWeight=" << minConfidentWeight << ": resample." << std::endl;
      VRmixin::particleFilter->updateSensors(VRmixin::particleFilter->getSensorModel(),false,true);
    }
    if ( estimate.weight < minConfidentWeight ) {
      std::cout << "  Particle filter bestWeight=" << estimate.weight
		<< " < minConfidentWeight=" << minConfidentWeight << ": resample once more." << std::endl;
      VRmixin::particleFilter->updateSensors(VRmixin::particleFilter->getSensorModel(),false,true);
    }
    VRmixin::mapBuilder->setAgent(Point(estimate.x,estimate.y,0,allocentric), estimate.theta, true);
    AngTwoPi heading = VRmixin::theAgent->getOrientation();
    cout << "Pilot localized: agent now at " << VRmixin::theAgent->getCentroid() << " hdg " << heading
				 << " (= " << float(heading)*180/M_PI << " deg.)" 
				 << "  std. dev. " << sqrt(((ShapeBasedParticleFilter*)VRmixin::particleFilter)->getVariance().x) << " mm, "
				 << float(((ShapeBasedParticleFilter*)VRmixin::particleFilter)->getVariance().theta) * 180 / M_PI << " deg."
				 << "  wtvar " << ((ShapeBasedParticleFilter*)VRmixin::particleFilter)->getVariance().y
				 << "  bestWeight=" << estimate.weight
				 << endl;
#if defined(TGT_IS_CREATE) || defined(TGT_IS_CREATE2)
		// cout << "GPS = [ " << state->sensors[GPSXOffset] << " , " << state->sensors[GPSYOffset] << " ]" << endl;
#endif
    if ( VRmixin::pilot->curReq->displayIndividualParticles > 0 )
      VRmixin::particleFilter->displayIndividualParticles(VRmixin::pilot->curReq->displayIndividualParticles);
    else if ( VRmixin::pilot->curReq->displayParticles > 0 )
      VRmixin::particleFilter->displayParticles(VRmixin::pilot->curReq->displayParticles);
    else {
      GET_SHAPE(particles, GraphicsData, VRmixin::worldShS);
      if ( particles.isValid() ) {
        int numParticles = particles->elements.size();
        VRmixin::particleFilter->displayParticles(numParticles > 0 ? numParticles : 50);
      }
    }
    VRmixin::autoRefreshSketchWorld();
  }

//================ WalkMachine ================

void Pilot::WalkMachine::WalkMachine_Walker::doStart() {
  PilotRequest &preq = *VRmixin::pilot->curReq;
  MMAccessor<WalkMC> walk_acc(getMC());
#ifdef TGT_HAS_WHEELS
  float vx = (preq.forwardSpeed > 0) ? preq.forwardSpeed : 50.f;
  float vy = 0;
  float va = (preq.turnSpeed > 0) ? preq.turnSpeed : 0.25f;
#else
  // these values are for Chiara
  float vx = (preq.forwardSpeed > 0) ? preq.forwardSpeed : 15.f;
  float vy = preq.strafeSpeed;
  float va = (preq.turnSpeed > 0) ? preq.turnSpeed : 0.1f;
#endif
	std::cout << "Pilot walking:";
	if ( preq.dx != 0 )
		std::cout << " dx = " << preq.dx;
	if ( preq.dy != 0 )
		std::cout << " dy = " << preq.dy;
	if ( preq.da != 0 )
		std::cout << " da = " << preq.da << " (" << preq.da*180/M_PI << " deg.)";
	std::cout << endl;
  walk_acc->setTargetDisplacement(preq.dx, preq.dy, preq.da, vx, vy, va);
  motman->setPriority(VRmixin::pilot->getWalk_id(),MotionManager::kStdPriority);
}


//================ WaypointWalkmachine ================

void Pilot::WaypointWalkMachine::WaypointWalkMachine_WaypointWalker::doStart() {
  PilotRequest &req = *VRmixin::pilot->curReq;
  MMAccessor<WaypointWalkMC> wp_acc(getMC());
  if (req.clearWaypoints)
    wp_acc->clearWaypointList();
  if ( req.waypointList.size() > 0 )
    wp_acc->appendWaypoints(req.waypointList);
  else
    std::cout << "Warning: Pilot request to waypointWalk with empty waypoint list." << std::endl;
  motman->setPriority(VRmixin::VRmixin::pilot->getWaypointwalk_id(),MotionManager::kStdPriority);
}

//================ SetVelocityMachine ================

void Pilot::SetVelocityMachine::SetVelocityMachine_Walker::doStart() {
  PilotRequest &preq = *VRmixin::pilot->curReq;
  MotionManager::MC_ID wid = VRmixin::pilot->getWalk_id();
  MMAccessor<WalkMC>(wid)->setTargetVelocity(preq.forwardSpeed, preq.strafeSpeed, preq.turnSpeed);
  motman->setPriority(wid,MotionManager::kStdPriority);
}

void Pilot:: changeVelocity(float vx, float vy, float va) {
	if ( VRmixin::pilot->curReq == NULL ) {
		std::cout << "Warning: call to Pilot::changeVelocity() when no Pilot request is active: ignored." << std::endl;
		return;
	}
  PilotRequest &preq = *VRmixin::pilot->curReq;
	switch ( preq.requestType ) {
	case PilotTypes::walk:
	case PilotTypes::setVelocity:
		{
			MotionManager::MC_ID wid = VRmixin::pilot->getWalk_id();
			if ( wid == MotionManager::invalid_MC_ID )
				cout << "Error: Pilot::changeVelocity() tried to use an invalid MC_ID" << endl;
			else
				MMAccessor<WalkMC>(wid)->setTargetVelocity(vx, vy, va);
			break;
		}
	default:
		{
			std::cout << "Warning: Pilot::changeVelocity() called, but the active Pilot request is not 'walk' or 'setVelocity': ignored."
								<< std::endl;
		}
	}
}

void Pilot::cancelVelocity() {
  if ( VRmixin::pilot->curReq != NULL && VRmixin::pilot->curReq->requestType == PilotTypes::setVelocity ) {
    MotionManager::MC_ID wid = VRmixin::pilot->getWalk_id();
    erouter->postEvent(EventBase(EventBase::motmanEGID, wid, EventBase::statusETID, 0));  // force WalkNode to complete
  }
  else
    std::cout << "Warning: call to Pilot::cancelVelocity() when not executing a setVelocity request: ignored." << std::endl;
}

//================ GoToShapeMachine ================

const float Pilot::ExecutePlan::allowableDistanceError = 100.f;
const float Pilot::ExecutePlan::allowableAngularError = 0.2f;

void Pilot::GoToShapeMachine::CheckParameters::doStart() {

  PilotRequest &req = *VRmixin::pilot->curReq;
  if ( ! req.targetShape.isValid() ) {
    cout << "Pilot request goToShape fails: no valid targetShape specified" << endl;
    postStateFailure();
    return;
  }
  if ( req.targetShape->isObstacle() ) {
    cout << "Warning: Pilot marking targetShape '" << req.targetShape->getName()
	 << "' (" << req.targetShape->getId() << ") as non-obstacle to permit path planning." << endl;
    req.targetShape->setObstacle(false);
  }
  // arguments look good; okay to proceed
  postStateSuccess();
}

//================PushObjectMachine================
void Pilot::PushObjectMachine::CheckParameters::doStart() {

  PilotRequest &req = *VRmixin::pilot->curReq;
  if ( ! req.objectShape.isValid() ) {
    cout << "Pilot request pushObject fails: no valid objectShape specified" << endl;
    postStateFailure();
    return;
  if ( ! req.targetShape.isValid() ) {
    cout << "Pilot request pushObject fails: no valid targetShape specified" << endl;
    postStateFailure();
    return;
  }
  if ( req.targetShape->isObstacle() ) {
    cout << "Warning: Pilot marking targetShape '" << req.targetShape->getName()
	 << "' (" << req.targetShape->getId() << ") as non-obstacle to permit path planning." << endl;
    req.targetShape->setObstacle(false);
  }
  }
  // arguments look good; okay to proceed
  postStateSuccess();
}


//=== Pilot static variables 
  unsigned int Pilot::verbosity = -1U & ~Pilot::PVnavStep;
  
//=== PushObjectMachine static variables 
  int Pilot::PushObjectMachine::backupDist = 600;
  int Pilot::PushObjectMachine::obstDist = 150;
  int Pilot::PushObjectMachine::robotDiam = 150;
  int Pilot::PushObjectMachine::objSize = 50;
  float Pilot::PushObjectMachine::prePushTurnSpeed = 0.25f;
  float Pilot::PushObjectMachine::prePushForwardSpeed = 100.f;
  float Pilot::PushObjectMachine::pushTurnSpeed = 0.25f;
  float Pilot::PushObjectMachine::pushForwardSpeed = 100.f;
} // namespace

#endif
