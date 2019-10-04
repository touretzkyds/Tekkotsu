//-*-c++-*-
#ifndef INCLUDED_VisualTargetTrans_h_
#define INCLUDED_VisualTargetTrans_h_

#include "Events/EventRouter.h"
#include "Shared/debuget.h"
#include "Shared/WorldState.h"

//! causes a transition when a visual object has been seen for at least 6 camera frames
class VisualTargetTrans : public Transition {
public:
	//!constructor
	VisualTargetTrans(StateNode* destination, unsigned int source_id)
		: Transition(destination), sid(source_id), count(0) {}

	//!constructor
	VisualTargetTrans(const std::string& name, StateNode* destination, unsigned int source_id)
		: Transition(name,destination), sid(source_id), count(0) {}

	//!starts listening for the object specified by the source id in the constructor
	virtual void postStart() {
		Transition::postStart();
		count=0;
		erouter->addListener(this,EventBase::visObjEGID,sid);
	}

	//!if the object is "close", calls fire()
	virtual void doEvent() {
		//serr->printf("VisualTargetTrans::doEvent() - enter %d\n",get_time());

		if(event->getTypeID()==EventBase::deactivateETID)
			count=0;
		else
			count++;
		if(count>5)
			fire(*event);

		//serr->printf("VisualTargetTrans::doEvent() - leave %d\n",get_time());
	}

protected:
	//!Source ID of object to track
	unsigned int sid;
	//! number of frames for which we've seen the object
	unsigned int count;
};

/*! @file
 * @brief Defines VisualTargetTrans, which causes a transition when a visual object has been seen for at least 6 camera frames
 * @author ejt (Creator)
 */

#endif
