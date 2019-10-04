#ifndef INCLUDED_EventListener_h
#define INCLUDED_EventListener_h

class EventBase;

//! An interface to allow a standard method of passing events
class EventListener {
 public:
	//! destructor
	virtual ~EventListener() {}

	//! for receiving events - you must override this to inherit
	/*! @see EventRouter
	 *  @param event the event being received */
	virtual void processEvent(const EventBase& event)=0;
};

/*! @file
 * @brief Defines EventListener class, an interface for anything that wants to receive events
 * @author ejt (Creator)
 */

#endif
