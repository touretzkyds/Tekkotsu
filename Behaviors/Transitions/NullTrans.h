//-*-c++-*-
#ifndef INCLUDED_NullTrans_h_
#define INCLUDED_NullTrans_h_

#include "Behaviors/Transition.h"
#include "Events/EventRouter.h"
#include "Shared/MarkScope.h"

//! a transition that occurs (via a 1 msec Timer event) as soon as the source node finishes starting up
class NullTrans : public Transition {
public:
	//! constructor
	NullTrans(StateNode* destination) : Transition(destination) {}
	
	//! constructor
	NullTrans(const std::string& name, StateNode* destination) : 
	Transition(name,destination) {}
	
	//!starts 1 msec timer, so transition will occur very soon
	virtual void preStart() {
		Transition::preStart();
		erouter->addTimer(this,9999,0,false);
	}
	
	//!when timer event is received, fire() the transition
	virtual void doEvent() { fire(); }
	
};

/*! @file
 * @brief Defines NullTrans, which causes a transition as soon as the source node finishes starting up
 * @author dst (Creator)
 */

#endif
