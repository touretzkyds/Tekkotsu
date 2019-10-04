#include "Shared/RobotInfo.h"
#if defined(TGT_HAS_HEAD) && (defined(TGT_HAS_LEGS) || defined(TGT_HAS_WHEELS))

#include "ChaseBallBehavior.h"
#include "Events/EventRouter.h"
#include "Events/VisionObjectEvent.h"
#include "Shared/WorldState.h"
#include "Shared/ProjectInterface.h"
#include "Shared/mathutils.h"

REGISTER_BEHAVIOR_MENU(ChaseBallBehavior,DEFAULT_TK_MENU);

void ChaseBallBehavior::doStart() {
	addMotion(head);
	addMotion(walk);
	erouter->addListener(this,EventBase::visObjEGID,ProjectInterface::visPinkBallSID);
}

//this could be cleaned up event-wise (only use a timer when out of view)
void ChaseBallBehavior::doEvent() {
	static float horiz=0, vert=0;
	
	if(event->getGeneratorID()==EventBase::visObjEGID && event->getTypeID()==EventBase::statusETID) {
		horiz=static_cast<const VisionObjectEvent*>(event)->getCenterX();
		vert=static_cast<const VisionObjectEvent*>(event)->getCenterY();
		std::cout << get_time() << ' ' << horiz << ' ' << vert << std::endl;
	}
		
	// For portability, look to see if the host hardware has a head pan & tilt joints,
	// Check the Shared/*Info.h files to look up the canonical "name" of the pan and tilt joints.
	// Note if you were coding for a specific robot, could just do "tiltIdx = HeadOffset + TiltOffset"
	// directly and not bother with this, or add a '&& defined(TGT_HAS_HEAD)' to the #if above to limit compilation
	const unsigned int panIdx = capabilities.findOutputOffset("NECK:pan");
	const unsigned int tiltIdx = capabilities.findOutputOffset("NECK:tilt");
	if(panIdx==-1U || tiltIdx==-1U)
		return; // guess we're headless, leave now...
	
	// these are millimeters per second
	const float FAST = 160, SLOW = 100;
	
	// We use the "Walk" motion command, but if it's wheel based, the WalkMC will still handle it anyway
	if(state->outputs[panIdx]<-.05 || state->outputs[panIdx]>.05)
		walk->setTargetVelocity(SLOW,0,state->outputs[panIdx]);
	else
		walk->setTargetVelocity(FAST,0,0); // target straight ahead, full speed!
	
	// Very simple visual servoing control -- move the head a small distance in the direction of the target
	// This is "proportional" control, because we move the head proportionally further when the error (horiz and vert) is larger
	// so it homes in on the ball (here p=12, dist to move is err*FOV/2)
	// http://en.wikipedia.org/wiki/Proportional_control
	float tilt=state->outputs[tiltIdx]-vert*CameraVertFOV/6;
	float pan=state->outputs[panIdx]-horiz*CameraHorizFOV/6;
	
	// We'll limit tilt and pan to their range of motion, although this isn't actually necessary, just demonstration:
	tilt = mathutils::limitRange(tilt, outputRanges[tiltIdx][MinRange], outputRanges[tiltIdx][MaxRange]);
	pan = mathutils::limitRange(pan, outputRanges[panIdx][MinRange], outputRanges[panIdx][MaxRange]);
	
	head->setJoints(tilt,pan,0);
}

#endif

/*! @file
 * @brief Implements ChaseBallBehavior, which runs around after whatever the dog sees
 * @author ejt (Creator)
 */
