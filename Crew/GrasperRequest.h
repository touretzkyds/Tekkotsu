//-*-c++-*-
#ifndef INCLUDED_GrasperRequest_h_
#define INCLUDED_GrasperRequest_h_

#include "Behaviors/StateNode.h"
#include "DualCoding/ShapeAgent.h"

#include "PilotRequest.h"
#include "MapBuilderRequest.h"

#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3) || defined(TGT_IS_CALLIOPE5) || defined(TGT_IS_MANTIS)
#  include "Planners/Manipulation/ShapeSpacePlanner3DR.h"
#else
#  include "Planners/Manipulation/ShapeSpacePlanner2DR.h"
#endif

using namespace std;
using namespace DualCoding;

//! Request to the @a Grasper to manipulate something

class GrasperRequest {
public:
#if defined(TGT_IS_CALLIOPE5)
	typedef ShapeSpacePlanner3DR<NumArmJoints-2> Planner;
	typedef PlannerObstacle3D PlannerObstacleG;
	static const unsigned int numPlannerJoints = NumArmJoints-2;
#elif defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
	typedef ShapeSpacePlanner3DR<NumArmJoints-1> Planner;
	typedef PlannerObstacle3D PlannerObstacleG;
	static const unsigned int numPlannerJoints = NumArmJoints-1;
#elif defined(TGT_IS_MANTIS)
	typedef ShapeSpacePlanner3DR<JointsPerFrLeg> Planner;
	typedef PlannerObstacle3D PlannerObstacleG;
	static const unsigned int numPlannerJoints = JointsPerFrLeg;
#elif defined(TGT_HAS_FINGERS)
	typedef ShapeSpacePlanner2DR<NumArmJoints-2> Planner;
	typedef PlannerObstacle2D PlannerObstacleG;
	static const unsigned int numPlannerJoints = NumArmJoints-2;
#elif defined(TGT_HAS_GRIPPER)
	typedef ShapeSpacePlanner2DR<NumArmJoints-1> Planner;
	typedef PlannerObstacle2D PlannerObstacleG;
	static const unsigned int numPlannerJoints = NumArmJoints-1;
#else
	typedef ShapeSpacePlanner2DR<NumArmJoints> Planner;
	typedef PlannerObstacle2D PlannerObstacleG;
	static const unsigned int numPlannerJoints = NumArmJoints;
#endif

	typedef Planner::NodeType_t NodeType_t;
	typedef NodeType_t::NodeValue_t NodeValue_t;
	
private:
  friend class Grasper;
  
  typedef unsigned int GrasperVerbosity_t;
  
public:
  
  //! What we're asking the @a Grasper to do
  enum GrasperRequestType_t {
    checkGraspable,	//!< Check whether @a object is graspable; don't actually move
    checkMovable,	//!< Check whether @a object can be moved to @a targetLocation; don't actually move
    checkRestable,	//!< Check whether @a the arm can be moved to the resting position; don't actually move
    computeReach,	//!< Compute and return a path for the arm to reach to a point
    reach,		//!< Move the arm to @a targetLocation, then rest
    grasp,		//!< Approach and grasp @a object, then rest the arm
    touch,		//!< Grasp the @a object but don't rest the arm afterwards
    release,		//!< Release and disengage from whatever the arm is holding
    moveTo,		//!< Move @a object to @a targetLocation
    rest,		//!< Move the arm (and possibly the body) to a resting position
    turn,               //!< Turn the robot towards the object
    //todo
    sweep,		//!< Sweep arm along an arc (to clear a region of the workspace)
    computeMove,	//!< Compute and return a path for moving @a object to @a targetLocation
    computeRest		//!< Compute and return a path for moving the arm to the resting position
  };
  
  //! What error is the Grasper going to return
  enum GrasperErrorType_t {
    noError = 0,
    someError,		//!< Generic error will match anything but noError in a GrasperTrans
    invalidRequest, 
    noGraspState,	//!< IK cannot get the gripper around the object
    noGraspPath,	//!< No unobstructed path to the grasp configuration
    lostObject,		//!< Lost sight of the object
    noDeliverState,
    noDeliverPath,
    noRestPath,
    badGrasp,
    badMove,
    pickUpUnreachable,
    dropOffUnreachable
  };
  
  //! The grasp strategy to use
  enum GraspStrategy_t {
    unconstrainedGrasp,   //!< Any orientation
    sideGrasp,            //!< Side approach: fingers parallel to the ground
    overheadGrasp         //!< Overhead approach: fingers perpendicular to the ground
  };

  //! What path are we talking about, e.g., to return in the @a GrasperEvent
  enum GrasperPathType_t {
    noPath,	//!< Don't return any path
    doApproach,	//!< Return the unconstrained path to @a object
    doDeliver, 	//!< Return the constrained path to move @a object to @a targetLocation
    doRelease	//!< Return the path to disengage from the object and move the arm to the rest position
  };
  
  //! The kind of rest state to reach. stationary means none, settleArm moves the arm to the arm configuration in @a armRestState, settleBodyAndArm does settleArm and resets the ground plane
  enum GrasperRestType_t {
    stationary,
    settleArm,
    settleBodyAndArm
  };
  
	//! How to verify a grasp
	enum GrasperVerifyStrategy_t {
		verifyNone, //!< Don't try to verify the grasp,
		verifyAprilTag, //!< Check for an AprilTag in the gripper
		verifyDomino, //!< Check for a domino in the gripper
		verifyCross, //!< Check for a cross in the gripper
		verifyLoad, //!< Verify by gripper load signal
		verifyUser //!< User will supply a verify function
	};

  //! Constructor
  GrasperRequest(GrasperRequestType_t _type);
  
  //! Copy constructor
  GrasperRequest(const GrasperRequest &req);
  
  //! Validate the GrasperRequest and return an error code if there is a problem
  GrasperErrorType_t validateRequest();
  
  //! The type of this Grasper request
  GrasperRequestType_t requestType;
  
  //! Type of grasp strategy to use
  GraspStrategy_t graspStrategy;

  //! The rest type for this request
  GrasperRestType_t restType;
  
  //! The rest arm configuration 
  NodeValue_t armRestState;
  
  GrasperVerbosity_t setVerbosity;   //!< Verbosity elements to force on
  GrasperVerbosity_t clearVerbosity;   //!< Verbosity elements to force off
  
  //RRT PARAMETERS

  //! Offset of the end effector in the Kinematic chain to use (typically GripperFrameOffset)
  unsigned int effectorOffset;

  //! The maximum number of iterations the RRT should undergo
  unsigned int rrtMaxIterations;

  //! The amount to inflate obstacles for safe planning (mm)
  float rrtInflation;

  //! The amount to move each joint while extending
  NodeValue_t rrtInterpolationStep;
  
  //! Factor to change how fast arm movements will be executed
  float armTimeFactor;
  
  //! Whether to open the gripper upon resting
  bool openGripperOnRest;
  
  //! The object to manipulate
  ShapeRoot object;
  
	//! Object-dependent enumerated type, e.g., "high end" or "low end" for a domino
	unsigned int objectFeature;

	//! Points on the object where a grasp could be made (relative to its centroid); currently unused
	std::vector<Point> objectGraspPoints;

	//! Orientation with which to approach the object
	AngTwoPi approachOrientation;

	//! Grasp verify strategy
	GrasperVerifyStrategy_t verifyStrategy;

	//! User-supplied function to verify object has been grasped
	bool (*verifyGraspFunction)(ShapeRoot&);
  
  //! Where to place the object
  ShapeRoot targetLocation;
  
  //! Orientation at which to place the object
  AngTwoPi targetOrientation;

  //! If true, the Grasper can issue Pilot requests to move the body
  bool allowBodyMotion;

	//! Desired grip pressure (units are robot-dependent)
	float gripPressure;

  //! Vector of ranges of egocentric angles in which the gripper is allowed to end up
  std::vector<std::pair<float, float> > gripperAngleRangesX;
  
  //! Vector of ranges of egocentric angles in which the gripper is allowed to end up
  std::vector<std::pair<float, float> > gripperAngleRangesY;
  
  //! Vector of ranges of egocentric angles in which the gripper is allowed to end up
  std::vector<std::pair<float, float> > gripperAngleRangesZ;
  
  //! Maximum number of angles to allow the gripper to have in angle ranges
  unsigned int maxNumberOfAngles;
  
  //! Resolution of angles to use within gripperAngleRanges
  float angleResolution;
  
  //! vector of objects to be swept
  std::vector<ShapeRoot> sweepObjects;
  
  //! List of obstacles in the environment
  std::vector<ShapeRoot> envObstacles;
  
  //! The desired predicate to use during planning for @a moveConstrainedPath
  AdmissibilityPredicate<NodeType_t> *predicate;
  
  //! What kind of path to return with the GrasperEvent
  GrasperPathType_t populateEventPathWith;
  
  //! Template to use for Pilot requests for body motion
  PilotRequest pilotreq;

  //! MapBuilder request to reacquire the object after motion
  MapBuilderRequest *mapreq;

  //! If true, the planned path is displayed in the shape space
  bool displayPath;
  
  //! If true, the RRT search tree is displayed in the shape space
  bool displayTree;
  
private:
  //! Robot's desired location and heading for reaching to an object
  Shape<AgentData> approachPose;

  //! Robot's desired location and heading for transporting the object
  Shape<AgentData> transportPose;

  //! Robot's desired location and heading for withdrawing
  Shape<AgentData> withdrawPose;

  //! The "approach" arm path, which is not predicate-constrained
  std::vector<NodeValue_t> approachPath;

  //! The predicate-constrained arm path for moving an object
  std::vector<NodeValue_t> deliverPath;

  //! The "release" arm path, which is not predicate-constrained
  std::vector<NodeValue_t> releasePath;
  
  //! The Behavior making this request; used for posting grasper events
  BehaviorBase* requestingBehavior;
  
  //! Verbosity value computed by Grasper::executeRequest
  GrasperVerbosity_t verbosity;
  
protected:
  unsigned int requestID;
  GrasperRequest& operator=(const GrasperRequest&); //!< don't call this
};

typedef GrasperRequest::GrasperErrorType_t GraspError;

#endif
