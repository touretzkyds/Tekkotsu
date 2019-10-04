//-*-c++-*-
#ifndef INCLUDED_CompareTrans_h_
#define INCLUDED_CompareTrans_h_

#include "Behaviors/Transition.h"
#include "Events/EventRouter.h"
#include "Shared/MarkScope.h"

//! causes a transition if a value (through a pointer) goes above a given value
/*! You will need to specify an event mask which will be listened for.  This event
 *  will then be listened for - each time it is received, CompareTrans will check
 *  the values for possible activation.
 *
 *  For example, if you want to transition when the IR sensor goes below, say 200,
 *  pass &state->sensors[IRDistOffset], CompareTrans::LT, 200, and
 *  EventBase(EventBase::sensorEGID,SensorSrcID::UpdatedSID,EventBase::statusETID)
 *  as the polling event.  Or a timer event to just check at a certain interval.
 *
 *  If you pass a class as the templated type, only requires that < operator is
 *  defined for comparing inequality, == for equality, and a copy constructor (CompareTrans
 *  holds a protected copy of the value)
 *  
 *  Passing NULL as the value to monitor will cause a transition on the first event received
 */
template<class T>
class CompareTrans : public Transition {
public:
	//! use these values to sepecify what kind of comparison should be made to test for activation
	enum Test_t {
		LT, //!< less than
		GT, //!< greater than
		LTE, //!< less than or equal
		GTE, //!< greater than or equal
		EQ, //!< equal
		NE //!< not equal
	};
	
	//! constructor, only checks @a monitor when it is first activated (no polling)
  CompareTrans(StateNode* destination, const T* monitor, Test_t test, const T& value)
		: Transition(destination), mon(monitor), tst(test), val(value), isPolling(false), poller()
	{ }
	
	//! constructor, only checks @a monitor when it is first activated (no polling)
  CompareTrans(const std::string& name, StateNode* destination, const T* monitor, Test_t test, const T& value)
		: Transition(name,destination), mon(monitor), tst(test), val(value), isPolling(false), poller()
	{ }
	
	//! constructor, see CompareTrans class notes for information
  CompareTrans(StateNode* destination, const T* monitor, Test_t test, const T& value, const EventBase& poll)
		: Transition(destination), mon(monitor), tst(test), val(value), isPolling(true), poller(poll)
	{ }
	
	//! constructor, see CompareTrans class notes for information
  CompareTrans(const std::string& name, StateNode* destination, const T* monitor, Test_t test, const T& value, const EventBase& poll)
		: Transition(name,destination), mon(monitor), tst(test), val(value), isPolling(true), poller(poll)
	{ }
	
	//!starts listening
	virtual void postStart() {
		Transition::postStart();
		if(isPolling)
			erouter->addListener(this,poller);
		else
			doEvent();
	}

	//!don't care about the event, just a pulse to check the values
	virtual void doEvent() {
		switch(tst) {
		case LT:
			if(*mon<val) fire();
			break;
		case GT:
			if(val<*mon) fire();
			break;
		case LTE:
			if(!(val<*mon)) fire();
			break;
		case GTE:
			if(!(*mon<val)) fire();
			break;
		case EQ:
			if(*mon==val) fire();
			break;
		case NE:
			if(!(*mon==val)) fire();
			break;
		}
	}

protected:
	const T* mon; //!< address of value to monitor
	Test_t tst; //!< test to make
	T val; //!< value to compare against
	bool isPolling; //!< set to true if #poller should be used (otherwise only checked on activation)
	EventBase poller; //!< event to listen to, when it comes, compare the values

private:
	CompareTrans(const CompareTrans& node); //!< don't call this
	CompareTrans operator=(const CompareTrans& node); //!< don't call this
};

/*! @file
 * @brief Defines CompareTrans, which causes a transition if a value (through a pointer) goes above a given value
 * @author ejt (Creator)
 */

#endif
