#include "Shared/RobotInfo.h"
#if defined(TGT_HAS_LEGS) || defined(TGT_HAS_WHEELS)

#include "Behaviors/BehaviorBase.h"

#include "Events/EventRouter.h"
#include "Events/VisionObjectEvent.h"
#include "Shared/WorldState.h"
#include "Motion/WalkMC.h"

#include "Motion/MotionPtr.h"
#include <cmath>

//! A simple behavior to chase after any objects seen by the vision system
/*! Similar to ChaseBallBehavior, but this one doesn't try to move the head, so
 *  it's a little more... simple.  However, it does make sure to take into account
 *  which direction the head is pointing when it sees the object. */
class SimpleChaseBallBehavior : public BehaviorBase {
public:
	//!constructor
	SimpleChaseBallBehavior()
		: BehaviorBase("SimpleChaseBallBehavior"), walker()
	{}

	//! adds a headpointer and a walker, and a listens for vision events
	virtual void doStart() {
		BehaviorBase::doStart();
		addMotion(walker);
		erouter->addListener(this,EventBase::visObjEGID);
	}
	
	//! sets the head to point at the object and sets the body to move where the head points
	virtual void doEvent() {
		if(event->getGeneratorID()==EventBase::visObjEGID && event->getTypeID()==EventBase::statusETID) {
			// See if the camera is on a pan joint, look up joint names in Shared/*Info.h or using the 'File Access/Posture Editor' control
			const unsigned int panIdx = capabilities.findOutputOffset("NECK:pan");
			float panAngle = (panIdx!=-1U) ? state->outputs[panIdx] : 0; // get its angle, or assume 0 if not found
			
			//x and y are the direction to walk; positive x is forward and positive y is left
			//so these calculations walk the direction the head is pointing (at 120 mm/sec)
			float x=120.0f*std::cos(panAngle);
			float y=120.0f*std::sin(panAngle);

			//z is the amount to turn in radians; conveniently enough, we can use the
			//x parameter from the vision event as the speed to turn -- you could
			//scale this up or down to make it be more or less sensitive
			float z=-static_cast<const VisionObjectEvent*>(event)->getCenterX();

			//now pass these values to the walk motion command:
			walker->setTargetVelocity(x,y,z);
		}
	}
			
protected:
	MotionPtr<WalkMC> walker; // !< a WalkMC object
};

// double registration, run on its own or in combination with StareAtBallBehavior
REGISTER_BEHAVIOR_MENU(SimpleChaseBallBehavior,DEFAULT_TK_MENU);
REGISTER_BEHAVIOR_MENU_OPT(SimpleChaseBallBehavior,"Background Behaviors",BEH_NONEXCLUSIVE);

#endif

/*! @file
 * @brief Describes SimpleChaseBallBehavior, which runs around after whatever the dog sees
 * @author ejt (Creator)
 */
