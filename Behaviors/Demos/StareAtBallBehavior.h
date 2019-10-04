//-*-c++-*-
#ifndef INCLUDED_StareAtBallBehavior_h_
#define INCLUDED_StareAtBallBehavior_h_

#include "Behaviors/BehaviorBase.h"
#include "Motion/MotionManager.h"

//! A simple behavior to chase after any objects seen by the vision system
class StareAtBallBehavior : public BehaviorBase {
public:
	//!constructor
	StareAtBallBehavior()
		: BehaviorBase("StareAtBallBehavior"), headpointer_id(MotionManager::invalid_MC_ID)
	{}
	//!destructor
	virtual ~StareAtBallBehavior() {}

	//! adds a headpointer and a listens for vision events
	virtual void doStart();

	//! removes motion commands and stops listening
	virtual void doStop();

	//! sets the head to point at the object and sets the body to move where the head points
	virtual void doEvent();
			
	static std::string getClassDescription() { return "Tracks any pink objects seen by the vision system"; }
	virtual std::string getDescription() const { return getClassDescription(); }

protected:
	MotionManager::MC_ID headpointer_id; //!< a HeadPointerMC object
};

/*! @file
 * @brief Describes StareAtBallBehavior, which runs around after whatever the dog sees
 * @author tss (Creator)
 */

#endif
