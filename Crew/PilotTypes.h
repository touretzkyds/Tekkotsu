#ifndef DEFINED_PilotTypes_h_
#define DEFINED_PilotTypes_h_

#include <iostream>

#include "DualCoding/Point.h"
#include "DualCoding/ShapeRoot.h"
#include "Planners/Navigation/ShapeSpacePlannerXYTheta.h"

namespace DualCoding{
  namespace PilotTypes {

    typedef ShapeSpacePlannerXYTheta::NodeType_t NodeType_t;
    typedef NodeType_t::NodeValue_t NodeValue_t;

  //! What we're asking the Pilot to do
  enum RequestType_t {
    localize, //!< Localize using the available landmarks
    walk, //!< Walk the distance specified by @a dx / @a dy / @a da
    waypointWalk, //!< Execute the trajectory in @a waypointlist
    setVelocity, //!< Set velocity for continuous walking
    goToShape, //!< Plan a path to @a targetShape
    pushObject, //!< Push @a objectSjape to @a targetShape
    visualSearch, //!< Turn until search predicate returns true
    setOdometry, //!< Turn head and set the visual odometry settings
    noRequest, //!< Dummy case
    numRequestTypes
  };

  extern const char* RequestTypeNames[numRequestTypes];

  //! What error the Pilot is going to return
  enum ErrorType_t {
    noError = 0,        //!< Request completed without error
    someError,          //!< Generic error will match anything but noError in a PiotTrans
    invalidRequest,     //!< Ill-formed PilotRequest, such as mssing parameters
    abort,              //!< Request was aborted
    cantLocalize,       //!< Localization failed
    startCollides,      //!< Start state would be in collision
    endCollides,        //!< End state would be in collision
    noPath,             //!< Path planning failed
    noSpace,		//!< Not enough space to maneuver
    collisionDetected,  //!< The robot hit something
    searchFailed,       //!< Visual search gave up
    numErrorTypes
  };

  extern const char* ErrorTypeNames[numErrorTypes];

  //! Options for handling collisions
  enum CollisionAction_t {
    collisionIgnore = 0,     //!< Don't check for collisions
    collisionReport,         //!< Report the collision in a PilotEvent, but continue
    collisionStop,           //!< Stop the robot and terminate the request
    collisionReplan          //!< Try to recover and proceed
  };

  enum NavigationStepType_t {
    localizeStep,     //!< localize during path following
    travelStep,       //!< travel forward during path following
    turnStep,         //!< turn to face next point on path
    preTravelStep,    //!< adjust heading before travel forward
    headingStep,      //!< turn to specific heading (at end of plan)
    turnObjStep,      //!< turn while pushing an object
    acquireObjStep    //!< move to acquire contact with object
  };

  struct NavigationStep {
    NavigationStep(NavigationStepType_t _type, Point _waypoint, AngTwoPi _orientation, AngSignTwoPi _turn): 
      type(_type), waypoint(_waypoint), orientation(_orientation), turn(_turn), visibleLandmarks() {};
    NavigationStepType_t type;
    Point waypoint;
    AngTwoPi orientation;
    AngSignTwoPi turn;
    std::vector<ShapeRoot> visibleLandmarks;
    std::string toString() const;
  };

	std::ostream& operator<<(std::ostream& os, const NavigationStep &step);

  //! Stores and indexes into a sequence of actions to complete a navigation task
  struct NavigationPlan {
    NavigationPlan() : path(), walkBackward(false), steps(), currentStep() {};
    std::vector<NodeValue_t> path;   //!< Path returned by path planner, from which the navigation plan is generated
    bool walkBackward; //!< If true, use heading opposite the direction of travel
    std::vector<NavigationStep> steps;     //!< The plan is a sequence of navigation steps
    std::vector<NavigationStep>::const_iterator currentStep;   //!< Index into the plan
    std::string toString() const;
    void addNavigationStep(NavigationStepType_t type, const DualCoding::Point &waypoint,
                           AngTwoPi orientation, AngSignTwoPi turn, const std::vector<ShapeRoot> &landmarks);
    void addNavigationStep(NavigationStepType_t type, const NodeValue_t &waypoint,
                           const std::vector<ShapeRoot> &landmarks);
    void clear() {
      path.clear();
      walkBackward = false;
      steps.clear();
      currentStep = steps.begin();
    }
  };

	std::ostream& operator<<(std::ostream& os, const NavigationPlan &plan);

  static const unsigned int invalid_Pilot_ID = -1U;

} } // end of namespaces

#endif
