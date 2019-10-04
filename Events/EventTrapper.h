#ifndef INCLUDED_EventTrapper_h
#define INCLUDED_EventTrapper_h

#include "EventBase.h"

//! An interface to allow a standard method of trapping events
/*! Trappers get first dibs on events and can prevent the event from being sent any further\n
 *  This is handy in situations where an event is more than a notification, and must be "handled" - 
 *  the trapper which handles it returns true, otherwise it is passed to the next one\n
 *  A trapper can filter any and all events, EXCEPT timers.  This *could* be changed, if a good
 *  reason is presented. */
class EventTrapper {
 public:
	//! destructor
	virtual ~EventTrapper() {}

	//! for receiving events - you must override this to inherit
	/*! @see EventRouter
	 *  @param event the event being received
	 *  @return @c true if the event was trapped (shouldn't be sent to listeners), @c false otherwise*/
	virtual bool trapEvent(const EventBase& event)=0;
};

/*! @file
 * @brief Defines EventTrapper class, an interface for anything that wants to trap events
 * @author ejt (Creator)
 */

#endif
