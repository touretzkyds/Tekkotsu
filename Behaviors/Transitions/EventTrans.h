//-*-c++-*-
#ifndef INCLUDED_EventTrans_h_
#define INCLUDED_EventTrans_h_

#include "Behaviors/Transition.h"
#include "Events/EventRouter.h"

//! causes a transition when the specified event is received

class EventTrans : public Transition {
private:
	int argcount; //!< the number of arguments which were supplied to the constructor (granularity of filtering)
	EventBase::EventGeneratorID_t egid; //!< the requested generator
	size_t esid; //!< the requested source
	unsigned int *esidAddr;
	EventBase::EventTypeID_t etid; //!< the requested type
	
public:
	//!@name Constructors
	//! follows general pattern of EventRouter::addListener()
	EventTrans(StateNode* destination, EventBase::EventGeneratorID_t gid)
		: Transition(destination), argcount(1), egid(gid), esid(0), esidAddr(NULL), etid(EventBase::statusETID)
	{}
	
	EventTrans(StateNode* destination, EventBase::EventGeneratorID_t gid, size_t sid)
		: Transition(destination), argcount(2), egid(gid), esid(sid), esidAddr(NULL), etid(EventBase::statusETID)
	{}
	
	EventTrans(StateNode* destination, EventBase::EventGeneratorID_t gid, unsigned int *sidAddr)
		: Transition(destination), argcount(2), egid(gid), esid(0), esidAddr(sidAddr), etid(EventBase::statusETID)
	{}
	
	EventTrans(StateNode* destination, EventBase::EventGeneratorID_t gid, size_t sid, EventBase::EventTypeID_t tid)
		: Transition(destination), argcount(3), egid(gid), esid(sid), esidAddr(NULL), etid(tid)
	{}
	
	EventTrans(StateNode* destination, EventBase::EventGeneratorID_t gid, unsigned int* sidAddr, EventBase::EventTypeID_t tid)
		: Transition(destination), argcount(3), egid(gid), esid(0), esidAddr(sidAddr), etid(tid)
	{}
	
	EventTrans(const std::string& name, StateNode* destination, EventBase::EventGeneratorID_t gid)
		: Transition(name,destination), argcount(1), egid(gid), esid(0),  esidAddr(NULL), etid(EventBase::statusETID)
	{}
	
	EventTrans(const std::string& name, StateNode* destination, EventBase::EventGeneratorID_t gid, size_t sid)
		: Transition(name,destination), argcount(2), egid(gid), esid(sid),  esidAddr(NULL), etid(EventBase::statusETID)
	{}
	
	EventTrans(const std::string& name, StateNode* destination, EventBase::EventGeneratorID_t gid, unsigned int *sidAddr)
		: Transition(name,destination), argcount(2), egid(gid), esid(0),  esidAddr(sidAddr), etid(EventBase::statusETID)
	{}
	
	EventTrans(const std::string& name, StateNode* destination, EventBase::EventGeneratorID_t gid, size_t sid, EventBase::EventTypeID_t tid)
		: Transition(name,destination), argcount(3), egid(gid), esid(sid),  esidAddr(NULL), etid(tid)
	{}
	
	EventTrans(const std::string& name, StateNode* destination, EventBase::EventGeneratorID_t gid, unsigned int *sidAddr, EventBase::EventTypeID_t tid)
		: Transition(name,destination), argcount(3), egid(gid), esid(0),  esidAddr(sidAddr), etid(tid)
	{}
	//@}
	
	//! starts listening
	virtual void preStart() {
		Transition::preStart();
		if ( esidAddr != 0 )
			erouter->addListener(this,egid);  // must check sid dynamically as events come in
		else {
			switch (argcount) {
				case 1: erouter->addListener(this,egid); break;
				case 2: erouter->addListener(this,egid,esid); break;
				case 3: erouter->addListener(this,egid,esid,etid); break;
			}
		}
	}
	
	//! fire the transition if an event is seen
	virtual void doEvent() {
		if ( esidAddr == 0 )
			fire(*event);
		else if ( event->getSourceID() == *esidAddr )
			if ( argcount == 2 || event->getTypeID() == etid )
				fire(*event);
	}
	
private:
	EventTrans(const EventTrans&); //!< do not call
	EventTrans& operator=(const EventTrans&); //!< do not call
};

/*! @file
 * @brief Defines EventTrans, which causes a transition if an event of the specified type occurs
 * @author dst (Creator)
 */

#endif
