#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_HEAD

#include "StareAtBallBehavior.h"
#include "Events/EventRouter.h"
#include "Events/VisionObjectEvent.h"
#include "Shared/WorldState.h"
#include "Motion/HeadPointerMC.h"
#include "Motion/MMAccessor.h"
#include "Shared/ProjectInterface.h"
#include "Shared/ERS7Info.h"
#include "IPC/SharedObject.h"

// double registration, run on its own or in combination with SimpleChaseBallBehavior
REGISTER_BEHAVIOR_MENU(StareAtBallBehavior,DEFAULT_TK_MENU);
REGISTER_BEHAVIOR_MENU_OPT(StareAtBallBehavior,"Background Behaviors",BEH_NONEXCLUSIVE);

//! Converts degrees to radians
inline double DtoR(double deg) { return (deg/180.0*M_PI); }

void StareAtBallBehavior::doStart() {
	BehaviorBase::doStart();
	headpointer_id = motman->addPersistentMotion(SharedObject<HeadPointerMC>());
	erouter->addListener(this,EventBase::visObjEGID,ProjectInterface::visPinkBallSID);
}

void StareAtBallBehavior::doStop() {
	erouter->removeListener(this);
	motman->removeMotion(headpointer_id);
	BehaviorBase::doStop();
}

//this could be cleaned up event-wise (only use a timer when out of view)
void StareAtBallBehavior::doEvent() {
	float horiz=0,vert=0;
	if(event->getGeneratorID()==EventBase::visObjEGID && event->getTypeID()==EventBase::statusETID) {
		const VisionObjectEvent& objev=static_cast<const VisionObjectEvent&>(*event);
		horiz=objev.getCenterX();
		vert=objev.getCenterY();
	}

	// for portability, look to see if the host hardware has a head pan & tilt joints
	const unsigned int panIdx = capabilities.findOutputOffset(ERS7Info::outputNames[ERS7Info::HeadOffset+ERS7Info::PanOffset]);
	const unsigned int tiltIdx = capabilities.findOutputOffset(ERS7Info::outputNames[ERS7Info::HeadOffset+ERS7Info::TiltOffset]);
	if(panIdx==-1U || tiltIdx==-1U)
		return; // guess not...
	
	//cout << horiz << ' ' << vert << endl;

	// Very simple visual servoing control -- move the head a small distance in the direction of the target
	// This is "proportional" control, because we move the head proportionally further when the error (horiz and vert) is larger
	// so it homes in on the ball (here p=12, dist to move is err*FOV/2)
	// http://en.wikipedia.org/wiki/Proportional_control
	float tilt=state->outputs[tiltIdx]-vert*CameraVertFOV/6;
	float pan=state->outputs[panIdx]-horiz*CameraHorizFOV/6;
	
	// now request access to the headpointer we added in doStart and set the joint angles
	MMAccessor<HeadPointerMC> headpointer(headpointer_id);
#ifdef TGT_IS_AIBO
	if(RobotName == ERS7Info::TargetName) {
		//on an ers-7, we want to set the nod joint to look up (maximum value), since tilt can only look down
		headpointer->setJoints(tilt,pan,outputRanges[HeadOffset+NodOffset][MaxRange]);
	} else {
		//on other models (we'll just assume ers-2xx), center the roll joint
		headpointer->setJoints(tilt,pan,0);
	}
#else
	/* really should do a kinematic solution with lookInDirection, but that assumes
	 * user has done a .kin file for this robot.  Let's just keep it simple and try to
	 * set the joints directly */
	if(NumHeadJoints>2)
		tilt/=2; // we're going to replicate the tilt parameter in the next call, so divide by 2
	headpointer->setJoints(tilt,pan,tilt);
#endif
}

#endif // check for TGT_HAS_HEAD

/*! @file
 * @brief Implements StareAtBallBehavior, which points the head at the ball
 * @author tss (Creator)
 */
