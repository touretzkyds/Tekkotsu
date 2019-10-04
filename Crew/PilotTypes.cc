#include "Motion/WalkMC.h"
#ifdef TGT_HAS_WALK

#include <ostream>

#include "Crew/Pilot.h"

namespace DualCoding {

  namespace PilotTypes {

const char* RequestTypeNames[] = {
  "localize",
  "walk",
  "waypointWalk",
  "setVelocity",
  "goToShape",
  "pushObject",
  "visualSearch",
	"setOdometry",
  "noRequest"
};

const char* ErrorTypeNames[] = {
  "noError",
  "someError",
  "invalidRequest",
  "abort",
  "cantLocalize",
  "startCollides",
  "endCollides",
  "noPath",
  "noSpace",
  "collisionDetected",
  "searchFailed"
};

//================ Navigation plans and steps ================

std::string NavigationPlan::toString() const {
  std::ostringstream os;
  os << "Navigation plan (" << steps.size() << " steps):" << std::endl;
	std::vector<NavigationStep>::const_iterator stepper = steps.begin();
  for ( size_t i=0; i<steps.size(); i++ ) 
    os << ( (stepper++ == currentStep ) ? "-> " : "   " )
			 << steps[i].toString() << std::endl;
  return os.str();
}

void NavigationPlan::addNavigationStep(NavigationStepType_t type, 
				       const DualCoding::Point &waypoint,
				       AngTwoPi orientation,
				       AngSignTwoPi turn,
				       const std::vector<ShapeRoot> &landmarks) {
  NavigationStep step(type, waypoint, orientation, turn);
  if ( type == localizeStep )
    step.visibleLandmarks = Pilot::calculateVisibleLandmarks(waypoint, orientation, M_PI/2, landmarks);
  if ( type != localizeStep || !step.visibleLandmarks.empty() )
    steps.push_back(step);
}

void NavigationPlan::addNavigationStep(NavigationStepType_t type, const NodeValue_t &waypoint,
				       const std::vector<ShapeRoot> &landmarks) {
  addNavigationStep(type,
		    Point(waypoint.x, waypoint.y, 0, allocentric),
		    waypoint.theta,
		    waypoint.turn,
		    landmarks);
}
		    

std::string NavigationStep::toString() const {
  std::ostringstream os;
  os << "NavigationStep[";
  switch (type) {
  case localizeStep: os << "localizeStep"; break;
  case turnStep: os << "turnStep"; break;
  case travelStep: os << "travelStep"; break;
  case headingStep: os << "headingStep"; break;
  case turnObjStep: os << "turnObjStep"; break;
  case acquireObjStep: os << "acquireObjStep"; break;
  default:
    os << "unknown step type";
  }
  if ( type == localizeStep ) {
    os << ", visible=(";
    for (size_t i = 0; i < visibleLandmarks.size(); i++)
      os << ( (i>0) ? "," : "")  << visibleLandmarks[i]->getName();
    os << ")";
  } else if ( type == headingStep || type == turnStep )
    os << ", hdg " << float(orientation)*(180/M_PI) << " deg."
       << ", turn " << float(turn)*(180/M_PI) << " deg.";
  else
    os << ", waypoint=" << waypoint;
  os << "]";
  return os.str();
}

std::ostream& operator<<(std::ostream& os, const NavigationStep &step) { return os << step.toString(); }
std::ostream& operator<<(std::ostream& os, const NavigationPlan &plan) { return os << plan.toString(); }

} } // namespaces

#endif
