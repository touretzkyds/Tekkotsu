//-*-c++-*-
#ifndef INCLUDED_TimerEvent_h_
#define INCLUDED_TimerEvent_h_

#include "Events/EventBase.h"

class EventListener;

//! Adds a target field to EventBase so listeners can resolve source ID conflict between different behaviors
/*! See EventRouter's class documentation for discussion of how to request
 *  and use timers. */
class TimerEvent : public EventBase {
public:
	//! empty constructor, initializes #target to NULL
	TimerEvent() : EventBase(), target(NULL) {}
	//! the full specification constructor, pass original requester @a tgt, generator @a gid (probably should always be EventBase::timerEGID), source @a sid, type @a tid (typically EventBase::statusETID), and duration @a dur
	TimerEvent(EventListener * tgt, EventGeneratorID_t gid, size_t sid, EventTypeID_t tid, unsigned int dur=0) : EventBase(gid,sid,tid,dur), target(tgt) {}
	//! copy constructor, does a shallow copy (copies pointer value, doesn't try to clone #target!)
	TimerEvent(const TimerEvent& te) : EventBase(te), target(te.target) {}
	//! assignment operator, does a shallow copy (copies pointer value, doesn't try to clone #target!)
	TimerEvent& operator=(const TimerEvent& te) { target=te.target; EventBase::operator=(te); return *this; }

	virtual EventBase* clone() const { return new TimerEvent(*this); }
	
	virtual unsigned int getClassTypeID() const { return autoRegisterTimerEvent; }
	
	EventListener * getTarget() const { return target; } //!< returns #target
	void setTarget(EventListener* tgt) { target=tgt; } //!< assigns @a tgt to #target

	std::string getDescription(bool showTypeSpecific=true, unsigned int verbosity=0) const;
	
	virtual unsigned int getBinSize() const;
	virtual unsigned int loadBinaryBuffer(const char buf[], unsigned int len);
	virtual unsigned int saveBinaryBuffer(char buf[], unsigned int len) const;
	virtual void loadXML(xmlNode* node);
	virtual void saveXML(xmlNode * node) const;
		
protected:
	EventListener * target; //!< indicates the listener for which the timer was created

	//! causes class type id to automatically be regsitered with EventBase's FamilyFactory (getTypeRegistry())
	static const EventBase::classTypeID_t autoRegisterTimerEvent;
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
