//-*-c++-*-
#ifndef INCLUDED_VisualTargetCloseTrans_h_
#define INCLUDED_VisualTargetCloseTrans_h_

#include "Events/EventRouter.h"
#include "Events/VisionObjectEvent.h"
#include "Shared/debuget.h"
#include "Shared/WorldState.h"
#include "Shared/ERS210Info.h"
#include "Shared/ERS220Info.h"
#include "Shared/ERS7Info.h"
#include "Shared/MarkScope.h"

//! causes a transition when a visual object is "close"
class VisualTargetCloseTrans : public Transition {
public:
	//!constructor
	VisualTargetCloseTrans(StateNode* destination, unsigned int source_id, float threshold=225)
		: Transition(destination), sid(source_id), distanceThreshold(threshold) {}

	//!constructor
	VisualTargetCloseTrans(const std::string& name, StateNode* destination, unsigned int source_id, float threshold=225)
		: Transition(name,destination), sid(source_id), distanceThreshold(threshold) {}

	//!starts listening for the object specified by the source id in the constructor
	virtual void postStart() {
		Transition::postStart();
		erouter->addListener(this,EventBase::visObjEGID,sid);
	}

	//!if the object is "close", calls fire()
	virtual void doEvent() {
		const VisionObjectEvent* ve=dynamic_cast<const VisionObjectEvent*>(event);
		ASSERTRET(ve!=NULL,"Casting error");
		float x=ve->getCenterX();
		float y=ve->getCenterY();
		unsigned int IRDistOffset=::IRDistOffset;
		//The ERS-7 adds more IR distance sensors, so we have to
		//break it down so we can specify which one
		if(RobotName == ERS7Info::TargetName)
			IRDistOffset=(distanceThreshold<350) ? ERS7Info::NearIRDistOffset : ERS7Info::FarIRDistOffset;
		if(x*x+y*y<0.02f && IRDistOffset!=-1U && state->sensors[IRDistOffset]<distanceThreshold)
			fire(*event);
	}

protected:
	//!Source ID of object to track
	unsigned int sid;
	
	//!Distance at which to trigger transition, in millimeters
	float distanceThreshold;
};

/*! @file
 * @brief Defines VisualTargetCloseTrans, which causes a transition when a visual object is "close"
 * @author ejt (Creator)
 */

#endif
