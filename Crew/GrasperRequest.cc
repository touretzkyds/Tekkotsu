#include "Shared/RobotInfo.h"
#if (defined(TGT_HAS_ARMS) || !defined(STRICT_TGT_MODEL)) && !defined(TGT_IS_AIBO)

#if defined(TGT_HANDEYE) or defined(TGT_CHIARA) or defined(TGT_CHIARA2)
#  include "Crew/CBracketGrasperPredicate.h"
#endif
#include "Crew/GrasperRequest.h"

GrasperRequest::GrasperRequest(GrasperRequestType_t _type) :
  requestType(_type),
  graspStrategy(sideGrasp),
  restType(GrasperRequest::stationary),
  armRestState(),
  setVerbosity(0),
  clearVerbosity(0),
  effectorOffset(),
  rrtMaxIterations(2000),
  rrtInflation(10),
  rrtInterpolationStep(),
  armTimeFactor(1.0f),
  openGripperOnRest(false),
  object(), 
  objectFeature(), objectGraspPoints(), 
	approachOrientation(std::numeric_limits<float>::quiet_NaN()),
	verifyStrategy(verifyNone), verifyGraspFunction(NULL),
  targetLocation(),
	targetOrientation(std::numeric_limits<float>::quiet_NaN()),
  allowBodyMotion(true),
	gripPressure(
#ifdef TGT_IS_CALLIOPE
    -280
#elif defined(TGT_IS_CALLIOPE3)
    -280
#else
    0
#endif
  ),
  gripperAngleRangesX(),
  gripperAngleRangesY(),
  gripperAngleRangesZ(),
  maxNumberOfAngles(100),
  angleResolution((float)M_PI/12),
  sweepObjects(), 
  envObstacles(), 
  predicate(NULL),
  populateEventPathWith(noPath),
  pilotreq(PilotTypes::goToShape),
  mapreq(NULL),
  displayPath(false),
  displayTree(false),
  approachPose(),
  transportPose(),
  withdrawPose(),
  approachPath(),
  deliverPath(),
  releasePath(),
  requestingBehavior(),
  verbosity(0),
  requestID(0)
{	
#ifdef TGT_HANDEYE
  allowBodyMotion = false;
#endif
#if defined(TGT_HANDEYE) or defined(TGT_CHIARA) or defined(TGT_CHIARA2)
  predicate = new CBracketGrasperPredicate<NumArmJoints>();
  effectorOffset = RobotInfo::GripperFrameOffset;
#endif
#if defined(TGT_CHIARA) or defined(TGT_CHIARA2)
  restType = GrasperRequest::settleBodyAndArm;
#endif
#if defined(TGT_HAS_GRIPPER)
  effectorOffset = RobotInfo::GripperFrameOffset;
#endif
  for (unsigned int i = 0; i < numPlannerJoints; i++) {
    armRestState[i] = 0.0f;
    rrtInterpolationStep[i] = M_PI / 180;
  }
#ifdef TGT_IS_CALLIOPE5
  std::pair<float,float> noSlop(0,0);
  gripperAngleRangesX.push_back(noSlop);
  gripperAngleRangesY.push_back(noSlop);
  gripperAngleRangesZ.push_back(noSlop);
#endif
}

GrasperRequest::GrasperRequest(const GrasperRequest &req) :
  requestType(req.requestType),
  graspStrategy(req.graspStrategy),
  restType(req.restType),
  armRestState(req.armRestState),
  setVerbosity(req.setVerbosity), clearVerbosity(req.clearVerbosity),
  effectorOffset(req.effectorOffset),
  rrtMaxIterations(req.rrtMaxIterations),
  rrtInflation(req.rrtInflation),
  rrtInterpolationStep(req.rrtInterpolationStep),
  armTimeFactor(req.armTimeFactor),
  openGripperOnRest(req.openGripperOnRest),
  object(req.object), 
  objectFeature(req.objectFeature), objectGraspPoints(req.objectGraspPoints), 
	approachOrientation(req.approachOrientation),
	verifyStrategy(req.verifyStrategy), verifyGraspFunction(req.verifyGraspFunction),
  targetLocation(req.targetLocation),
	targetOrientation(req.targetOrientation),
  allowBodyMotion(req.allowBodyMotion),
	gripPressure(req.gripPressure),
  gripperAngleRangesX(req.gripperAngleRangesX),
  gripperAngleRangesY(req.gripperAngleRangesY),
  gripperAngleRangesZ(req.gripperAngleRangesZ),
  maxNumberOfAngles(req.maxNumberOfAngles),
  angleResolution(req.angleResolution),
  sweepObjects(req.sweepObjects), 
  envObstacles(req.envObstacles), 
  predicate(req.predicate),
  populateEventPathWith(req.populateEventPathWith),
  pilotreq(req.pilotreq),
  mapreq(req.mapreq),
  displayPath(req.displayPath),
  displayTree(req.displayTree),
  approachPose(req.approachPose),
  transportPose(req.transportPose),
  withdrawPose(req.withdrawPose),
  approachPath(req.approachPath),
  deliverPath(req.deliverPath),
  releasePath(req.releasePath),
  requestingBehavior(req.requestingBehavior),
  verbosity(req.verbosity),
  requestID(req.requestID)
{}

GrasperRequest::GrasperErrorType_t GrasperRequest::validateRequest() {
  // If grasper action requires an object, make sure that one was supplied
  switch ( requestType ) {
  case checkGraspable:
  case checkMovable:
  case computeMove:
  case grasp:
  case computeReach:
  case reach:
  case touch:
  case turn:
  case moveTo:
    if ( ! object.isValid() ) {
      cout << "GrasperRequest of this type requires a valid 'object' field. " << endl;
      return invalidRequest;
    }
    break;
  default:
    break;
  }

	// If verifyStrategy is specified, check validity
	switch ( verifyStrategy ) {
	case verifyUser:
		if ( verifyGraspFunction == NULL ) {
			cout << "GrasperRequest with verifyStrategy of 'verifyUser' requires a verifyGraspFunction" << endl;
			return invalidRequest;
		}
	// Could also check other values to make sure the request type includes grasping and
	// the object type is compatible.
	default:
		break;
	}

  // If grasper action requires a taget location, make sure that one was supplied
  switch ( requestType ) {
  case checkMovable:
  case computeMove:
  case computeReach:
  case moveTo:
    if ( ! targetLocation.isValid() ) {
      cout << "GrasperRequest of this type requires a valid 'targetLocation' field. " << endl;
      return invalidRequest;
    }
    break;
  default:
    break;
  }

  if (armTimeFactor < 0) {
    cout << "Grasper request has armTimeFactor < 0: changing to 1." << endl;
    armTimeFactor = 1;
  }

  return noError;
}

#endif
