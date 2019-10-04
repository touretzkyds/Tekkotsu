#include <limits>

#include "Crew/PilotRequest.h"
#include "Motion/WaypointList.h"

namespace DualCoding {

PilotRequest::PilotRequest(PilotTypes::RequestType_t _type) :
  requestType(_type), 
  dx(0), dy(0), da(0), forwardSpeed(0), strafeSpeed(0), turnSpeed(0), walkParameters(""),
  collisionAction(PilotTypes::collisionStop),
  waypointList(), clearWaypoints(false),
  landmarkExtractor(NULL), landmarkExitTest(NULL),
  searchObjectExtractor(NULL),  searchExitTest(NULL), searchRotationAngle(0),
  targetShape(), baseOffset(), targetHeading(std::numeric_limits<float>::quiet_NaN()), gateLength(0),
  objectShape(), objectExtractor(NULL), objectMatcher(NULL),
  acquireExtractor(NULL), acquireTest(NULL),
  allowBackwardMotion(true), maxBackwardDistance(500.f),
  pathEditor(NULL), planEditor(NULL),
  obstacleInflation(3*25.4f),
  avoidCliffs(false), cliffThreshold(200), avoidObstacles(false), obstacleThreshold(100),
  trackRequest(NULL), 
  displayParticles(50), displayIndividualParticles(0), displayPath(true), displayTree(false), displayObstacles(false), autoDisplayObstacles(true),
  maxRRTIterations(4000), executePath(true), landmarks(),
  /*
  safeDistanceAroundTarget(300), subtendAngle(0.2f), approachAngle(0.1f),
  positionRelativeToTarget(200, 0, 0), angleToPushTarget((direction_t)M_PI),
  buildTargetParamsFn(NULL), buildTargetMapBuilderRequestFn(NULL),
  */
  requestingBehavior(NULL), requestID(PilotTypes::invalid_Pilot_ID) {}

PilotRequest::PilotRequest(const PilotRequest &req) :
  requestType(req.requestType),
  dx(req.dx), dy(req.dy), da(req.da), 
  forwardSpeed(req.forwardSpeed), strafeSpeed(req.strafeSpeed), turnSpeed(req.turnSpeed),
  walkParameters(req.walkParameters),
  collisionAction(req.collisionAction),
  waypointList(req.waypointList), clearWaypoints(req.clearWaypoints),
  landmarkExtractor(req.landmarkExtractor), landmarkExitTest(req.landmarkExitTest),
  searchObjectExtractor(req.searchObjectExtractor), searchExitTest(req.searchExitTest),
  searchRotationAngle(req.searchRotationAngle),
  targetShape(req.targetShape), baseOffset(req.baseOffset), targetHeading(req.targetHeading),
  gateLength(req.gateLength),
  objectShape(req.objectShape), objectExtractor(req.objectExtractor), objectMatcher(req.objectMatcher),
  acquireExtractor(req.acquireExtractor), acquireTest(req.acquireTest),
  allowBackwardMotion(req.allowBackwardMotion), maxBackwardDistance(req.maxBackwardDistance),
  pathEditor(req.pathEditor), planEditor(req.planEditor),
  obstacleInflation(req.obstacleInflation),
  avoidCliffs(req.avoidCliffs), cliffThreshold(req.cliffThreshold),
  avoidObstacles(req.avoidObstacles), obstacleThreshold(req.obstacleThreshold),
  trackRequest(req.trackRequest), 
  displayParticles(req.displayParticles), displayIndividualParticles(req.displayIndividualParticles),
  displayPath(req.displayPath), displayTree(req.displayTree), displayObstacles(req.displayObstacles), autoDisplayObstacles(req.autoDisplayObstacles),
  maxRRTIterations(req.maxRRTIterations), executePath(req.executePath),landmarks(req.landmarks),
  /*
  safeDistanceAroundTarget(req.safeDistanceAroundTarget), subtendAngle(req.subtendAngle), approachAngle(req.approachAngle),
  positionRelativeToTarget(req.positionRelativeToTarget), angleToPushTarget(req.angleToPushTarget),
  buildTargetParamsFn(req.buildTargetParamsFn), buildTargetMapBuilderRequestFn(req.buildTargetMapBuilderRequestFn),
  */
  requestingBehavior(req.requestingBehavior), requestID(req.requestID) {}

PilotRequest& PilotRequest::operator=(const PilotRequest &other) {
  requestType = other.requestType;
  dx = other.dx;
  dy = other.dy;
  da = other.da;
  forwardSpeed = other.forwardSpeed;
  strafeSpeed = other.strafeSpeed;
  turnSpeed = other.turnSpeed;
  walkParameters = other.walkParameters;
  collisionAction = other.collisionAction;
  waypointList = other.waypointList;
  clearWaypoints = other.clearWaypoints;
  landmarkExtractor = other.landmarkExtractor;
  landmarkExitTest = other.landmarkExitTest;
  searchObjectExtractor = other.searchObjectExtractor;
  searchExitTest = other.searchExitTest;
  searchRotationAngle = other.searchRotationAngle;
  targetShape = other.targetShape;
  baseOffset = other.baseOffset;
  targetHeading = other.targetHeading;
  gateLength = other.gateLength;
  objectShape = other.objectShape;
  objectExtractor = other.objectExtractor;
  objectMatcher = other.objectMatcher;
  acquireExtractor = other.acquireExtractor;
  acquireTest = other.acquireTest;
  allowBackwardMotion = other.allowBackwardMotion;
  maxBackwardDistance = other.maxBackwardDistance;
  pathEditor = other.pathEditor;
  planEditor = other.planEditor;
  obstacleInflation = other.obstacleInflation;
  avoidCliffs = other.avoidCliffs;
  cliffThreshold = other.cliffThreshold;
  avoidObstacles = other.avoidObstacles;
  obstacleThreshold = other.obstacleThreshold;
  trackRequest = other.trackRequest;
  displayParticles = other.displayParticles;
  displayIndividualParticles = other.displayIndividualParticles;
  displayPath = other.displayPath;
  displayTree = other.displayTree;
  displayObstacles = other.displayObstacles;
  autoDisplayObstacles = other.autoDisplayObstacles;
  maxRRTIterations = other.maxRRTIterations;
  executePath = other.executePath;
  landmarks = other.landmarks;
  return *this;
}

} // namespace
