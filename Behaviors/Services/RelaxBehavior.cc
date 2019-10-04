#include <iostream>
#include "Behaviors/BehaviorBase.h"
#include "Motion/PIDMC.h"
#include "IPC/SharedObject.h"
#include "Motion/MotionManager.h"
#include "Motion/MotionPtr.h"

//! A behavior that sets all the pids to zero for the tail and legs servos.
/*! This should hopefully make the robot quieter and consume less power. */
class RelaxBehavior : public BehaviorBase {
public:
	//! contstructor
	RelaxBehavior() : BehaviorBase("RelaxBehavior"), 
		pid(SharedObject<PIDMC>(0)) // initialize PIDMC using constructor with "power level" of 0 for all joints
	{}
	
	virtual void doStart() {
		std::cout << "Relaxing servos." << std::endl;
		
		// You could also change the priority level so that anytime
		// a joint is not in use it goes limp (try kBackgroundPriority)
		addMotion(pid,PERSISTENT,MotionManager::kHighPriority);

		// if you want to keep some joints from being turned off, e.g.:
		//pid->setJointPowerLevel(HeadOffset+TiltOffset,1);
		// (or you could change the contructor too...)
		// http://www.tekkotsu.org/dox/classPIDMC.html
	}

	virtual void doStop() {
		std::cout << "Relax terminated." << std::endl;
		removeMotion(pid);
		// this "one-shot" version of doing things will restore the PIDs on our way out
		// note the explicit call to motman with a naked SharedObject instead of using
		// BehaviorBase::addMotion or MotionPtr Ñ otherwise this would be immediately
		// stopped again as the behavior exits and not have time to do anything
		motman->addPrunableMotion(SharedObject<PIDMC>(1));
	}

	static std::string getClassDescription() { return "Sets PID parameters for all applicable joints to 0, allowing the joints to move freely, reducing noise and power consumption"; }

	virtual std::string getDescription() const { return getClassDescription(); }

protected:
	MotionPtr<PIDMC> pid; //!< the pid motion command
};

REGISTER_BEHAVIOR_MENU_OPT(RelaxBehavior,"Background Behaviors",BEH_NONEXCLUSIVE);

/*! @file
 * @brief Defines RelaxBehavior, which sets all the PIDs to zero for the tail and legs servos.
 * @author Erik Berglund <tekkotsu.dev.spam.345@blog.no> (Creator)
 * @author ejt (Modifications)
 */
