//-*-c++-*-
#ifndef INCLUDED_PilotRequest_h_
#define INCLUDED_PilotRequest_h_

#include "DualCoding/ShapeRoot.h"
#include "Behaviors/BehaviorBase.h"
#include "Motion/WaypointList.h"

#include "PilotTypes.h"

namespace DualCoding {

class MapBuilderRequest;
class LookoutTrackRequest;

//! Request to the @a Pilot for motion or navigation.
class PilotRequest {
  friend class Pilot;

 public:

  //! Constructor
  PilotRequest(PilotTypes::RequestType_t _type = PilotTypes::noRequest);

  PilotRequest(const PilotRequest &req);

  PilotTypes::RequestType_t getRequestType() const { return requestType; }

  PilotTypes::RequestType_t requestType; //!< Type of pilot request
  float dx; //!< Forward distance in mm (negative means go backward)
  float dy; //!< Sideways distance in mm (positive to the left)
  float da; //!< Rotation angle in radians (positive is counterclockwise)
  float forwardSpeed; //!< Translation speed in mm/sec for @a dx or @a dy
  float strafeSpeed; //!< Sideways translational speed used for setVelocity
  float turnSpeed; //!< Rotational speed in radians/sec for @a da
  std::string walkParameters; // !< Name of walk parameter file to load
  PilotTypes::CollisionAction_t collisionAction;  //!< What to do about collisions

  WaypointList waypointList; //!< Waypoint list for waypointWalk
  bool clearWaypoints;  //!< If true, the waypointList will be cleared before appending new waypoints

  MapBuilderRequest *landmarkExtractor; //!< pointer to MapBuilderRequest used to find landmarks; will be deleted by the Pilot
  bool (*landmarkExitTest)(); //!< Should return true if there are enough landmarks to localize

  MapBuilderRequest *searchObjectExtractor; //!< MapBuilderRequest to be used for visual search
  bool (*searchExitTest)(); //!< If true, terminate search and post a completion event
  AngSignPi searchRotationAngle; //!< Angle to rotate body to continue a visual search

  ShapeRoot targetShape; //!< Shape to walk to
  fmat::Column<3> baseOffset; //!< Point in the base reference frame to bring to the target location
  AngTwoPi targetHeading; //!< Heading on which we want to arrive at the target
  float gateLength; //!< Distance between the gate point and the target point when targetHeading or baseOffset used

  ShapeRoot objectShape; //!< Object we want to push to the target
  MapBuilderRequest *objectExtractor; //!< Pointer to MapbuilderRequest for finding the object to be pushed; will be deleted by the Pilot
  ShapeRoot (*objectMatcher)(const ShapeRoot&); //!< Finds the localShS object matching objectShape in worldShS
  MapBuilderRequest *acquireExtractor; //!< Pointer to MapBuilderRequest to check if we have acquired the object
  bool (*acquireTest)(const ShapeRoot&); //!< Returns true if vision confirms that we have successfully acquired the object (e.g., for pushing)

  bool allowBackwardMotion; //!< True if the robot should avoid large turns by walking backwards if distance is short
  float maxBackwardDistance; //!< Maximum allowable distance to walk backward instead of turning around

  void (*pathEditor)(std::vector<PilotTypes::NodeValue_t> *pathResult, PilotRequest &req);
  void (*planEditor)(PilotTypes::NavigationPlan &plan, PilotRequest &req);

  float obstacleInflation; //!< Inflation in mm of obstacle bounding shapes for path planning
  bool avoidCliffs; //!< If true, use IR to avoid walking off a cliff
  int cliffThreshold; //!< Maximum tolerable distance to the ground (millimeters)
  bool avoidObstacles; //!< If true, use rangefinder sensors to dynamically avoid obstacles (not yet implemented)
  int obstacleThreshold; //!< Minimum tolerable rangefinder distance to an obstacle (millimeters)
  LookoutTrackRequest *trackRequest; //!< Lookout request for tracking objects while walking

  float displayParticles; //!< How many particles to display (number or percentage) as a single Graphics shape
  float displayIndividualParticles; //!< How many particles to display (number or percentage) as LocalizationParticle shapes
  bool displayPath; //!< If true, the planned path is displayed in the shape space
  bool displayTree; //!< If true, the RRT search tree is displayed in the world shape space
  bool displayObstacles; //!< If true, the obstacle boundaries used by the collision checker are displayed in the world shape space
  bool autoDisplayObstacles; //!< If true, the obstacle boundaries used by the collision checker are displayed in the world shape space automatically if path planning fails
  unsigned int maxRRTIterations; //!< Maximum number of iterations for path planner RRT search
  bool executePath;  //!< If true, the Pilot will execute the path it has planned; if false, it plans but does not execute
  std::vector<DualCoding::ShapeRoot> landmarks; //!< Vector of specific landmarks to use for localization, overriding the default

  /* Target-related items from Somchaya Liemhetcharat's work; disabled for now
  float safeDistanceAroundTarget; //!< The distance to stay away from the target while circling
  AngSignPi subtendAngle; //!< The angle in which to subtend the target while circling
  AngSignPi approachAngle; //!< The angle in which to approach the desired position around the target
  Point positionRelativeToTarget; //!< The desired position around the target, relative to the target
  AngSignPi angleToPushTarget; //!< The angle in which to push the target
  void (*buildTargetParamsFn)(bool *buildFrontLeft, bool *buildFrontRight, bool *buildBackLeft, bool *buildBackRight, bool *lookAtCentroid, int *maxRetries); //!< function to return the parameters to build the target
  MapBuilderRequest* (*buildTargetMapBuilderRequestFn)(Point point); //!< function to return a dynamically-constructed MapBuilderRequest, given a point to look at, which BuildTarget will use to build the target
  */

  BehaviorBase* requestingBehavior;  //!< Used to construct a PilotEvent to notify the requesting node of the results of this Pilot operation
  
  PilotRequest& operator=(const PilotRequest &req);

  bool operator==(const PilotRequest &other) const { return this == &other; } // needed for SignalTrans<PilotRequest>

private:
  unsigned int requestID;
};

} // namespace

#endif
