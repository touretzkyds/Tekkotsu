//-*-c++-*-
#ifndef INCLUDED_FollowHeadBehavior_h_
#define INCLUDED_FollowHeadBehavior_h_

#include "Behaviors/BehaviorBase.h"
#include "Motion/MotionManager.h"
#include "IPC/SharedObject.h"
#include "Events/EventBase.h"

//! Will walk where the head is pointing
/*! Press the chin button to loosen the head to point it, release the
 *  button to lock it again
 *
 *  Tilt of head determines x axis (forward/backward)<br>
 *  Roll of head determines y axis (sideways strafing)<br>
 *  Pan of head determines z axis (rotational)
 *
 *  The zero point of joint position is zero motion.  Since the tilt
 *  is asymmetric (can tilt down farther than it can tilt up), the
 *  full range of the down tilt isn't used - if you tilt down farther
 *  than you could tilt it back, it'll just clip the speed.  Besides,
 *  if the head is all the way down, it screws up the walk because
 *  the center of balance is changed.
 */
class FollowHeadBehavior : public BehaviorBase {
 public:
	//! just sets up the variables
	FollowHeadBehavior();

	//! Register for events and creates and adds two motion commands - a walker and a head pointer
	virtual void doStart();

	//! Removes its two motion commands
	virtual void doStop();

	//! Handles event processing
	/*! After every clock pulse, sets walk in direction of head */
	virtual void doEvent();

	static std::string getClassDescription() { return "Walks whereever you point the head - press the chin button to loosen the head, release to lock it"; }
	virtual std::string getDescription() const { return getClassDescription(); }

 protected:
	//! returns the current position of the specified output, normalized to range -1..1
	static float getRelativePosition(const char* outputName);
	
	EventBase head_release; //!< event mask for releasing head (chin button down)
	EventBase head_lock;    //!< event mask for locking head (chin button up)
	EventBase clock;        //!< event mask for updating walk direction (every 150 ms)
	MotionManager::MC_ID walker_id;      //!< MC_ID for walker
};

/*! @file
 * @brief Describes FollowHeadBehavior, walks where the head is pointing
 * @author ejt (Creator)
 */

#endif
