#include "Shared/RobotInfo.h"
#if defined(TGT_HAS_BUTTONS) && defined(TGT_HAS_HEAD) && ( defined(TGT_HAS_LEGS) || defined(TGT_HAS_WHEELS) )

#include "FollowHeadBehavior.h"
#include "Events/EventRouter.h"
#include "Shared/debuget.h"
#include "Shared/WorldState.h"
#include "Motion/MMAccessor.h"
#include "Motion/HeadPointerMC.h"
#include "Motion/WalkMC.h"
#include "Motion/PIDMC.h"

REGISTER_BEHAVIOR_MENU(FollowHeadBehavior,DEFAULT_TK_MENU);

using namespace std;

FollowHeadBehavior::FollowHeadBehavior() :
	BehaviorBase("FollowHeadBehavior"),
	head_release(EventBase::buttonEGID,capabilities.findButtonOffset("ChinBut"),EventBase::activateETID,0),
	head_lock(EventBase::buttonEGID,capabilities.findButtonOffset("ChinBut"),EventBase::deactivateETID,0),
	clock(EventBase::timerEGID,0,EventBase::statusETID,250),
	walker_id(MotionManager::invalid_MC_ID)
{
	// if the requested button(s), aren't available, just use the 'first' one...
	if(head_release.getSourceID()==-1U)
		head_release.setSourceID(0);
	if(head_lock.getSourceID()==-1U)
		head_lock.setSourceID(0);
}

void FollowHeadBehavior::doStart() {
	//set up the shared motions
	walker_id=motman->addPersistentMotion(SharedObject<WalkMC>());
	//register for events and timers
	erouter->addListener(this,head_release);
	erouter->addListener(this,head_lock);
	//prime the head pointer so we start walking *right now*
	processEvent(clock);
	erouter->addTimer(this,clock); // this will keep us on track
}

void FollowHeadBehavior::doStop() {
	//remove timers and listeners
	erouter->removeListener(this);
	//remove motion commands, set them to invalid
	motman->removeMotion(walker_id);
	walker_id=MotionManager::invalid_MC_ID;
}

float FollowHeadBehavior::getRelativePosition(const char* outputName) {
	unsigned int offset = capabilities.findOutputOffset(outputName);
	if(offset==-1U)
		return 0; // not available on this model... oh well
	// outputRanges is defined in the RobotInfo header for the current target model
	float rangeOfMotion = (outputRanges[offset][MaxRange]-outputRanges[offset][MinRange]);
	// state is a global containing current sensor readings, from WorldState.h
	float positionInRange = (state->outputs[offset]-outputRanges[offset][MinRange]);
	// now we know our position as a percentage of the range of motion:
	float relativePosition = positionInRange/rangeOfMotion;
	// convert from range 0..1 to -1..1 
	return relativePosition*2-1;
}

void FollowHeadBehavior::doEvent() {
	if(clock == *event) {
		//x,y,a are percentage of maximum speed in that direction
		float x=-getRelativePosition("NECK:tilt");
		float y=-getRelativePosition("NECK:roll");
		float a=-getRelativePosition("NECK:pan");
		MMAccessor<WalkMC> walk(walker_id);
		walk->setTargetVelocity(x*walk->getMaxXVel(),y*walk->getMaxYVel(),a*walk->getMaxAVel());

	} else if(head_release == *event) {
		cout << "release" << endl;
		motman->addPrunableMotion(SharedObject<PIDMC>(HeadOffset,HeadOffset+NumHeadJoints,0));
		erouter->addTimer(this,clock);

	} else if(head_lock == *event) {
		cout << "lock" << endl;
		motman->addPrunableMotion(SharedObject<PIDMC>(HeadOffset,HeadOffset+NumHeadJoints,1));
		for(unsigned int i=HeadOffset; i<HeadOffset+NumHeadJoints; i++) {
			motman->setOutput(NULL,i,state->outputs[i]); //doing this prevents the head from jerking back when you released it to where it was before you pressed the button
			std::cout << state->outputs[i]/M_PI*180 << ' ';
		}
		std::cout << endl;
		erouter->removeTimer(this,clock.getSourceID());

	} else {
		ASSERT(false,"unprocessed event " << event->getName() << endl);
	}
}

#endif

/*! @file
 * @brief Implements FollowHeadBehavior, walks where the head is pointing
 * @author ejt (Creator)
 */

