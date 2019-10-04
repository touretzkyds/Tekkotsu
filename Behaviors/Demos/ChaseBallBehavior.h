//-*-c++-*-
#ifndef INCLUDED_ChaseBallBehavior_h_
#define INCLUDED_ChaseBallBehavior_h_

#include "Behaviors/BehaviorBase.h"
#include "Motion/MotionPtr.h"
#include "Motion/HeadPointerMC.h"
#include "Motion/WalkMC.h"

//! A simple behavior to chase after any objects seen by the vision system
class ChaseBallBehavior : public BehaviorBase {
public:
	//!constructor
	ChaseBallBehavior() : BehaviorBase(), head(), walk() {}

	//! adds a headpointer and a walker, and a listens for vision events
	virtual void doStart();

	//! sets the head to point at the object and sets the body to move where the head points
	virtual void doEvent();
			
	static std::string getClassDescription() { return "Follows ball with head and walks whereever the head is pointing"; }
	virtual std::string getDescription() const { return getClassDescription(); }

protected:
	MotionPtr<HeadPointerMC> head; //!< a HeadPointerMC object
	MotionPtr<WalkMC> walk;      //!< a WalkMC object
};

/*! @file
 * @brief Describes ChaseBallBehavior, which runs around after whatever the dog sees
 * @author ejt (Creator)
 */

#endif
