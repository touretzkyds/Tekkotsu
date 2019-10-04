//-*-c++-*-
#ifndef INCLUDED_TextMsgTrans_h_
#define INCLUDED_TextMsgTrans_h_

#include "Behaviors/Transition.h"
#include "Events/TextMsgEvent.h"
#include "Events/EventRouter.h"
#include "Shared/MarkScope.h"

//! Fires when a matching string is received
class TextMsgTrans : public Transition {

	// **************************** //
	// ******* CONSTRUCTORS ******* //
	// **************************** //
public:
	//! default constructor, use type name as instance name
	TextMsgTrans(StateNode* destination, const std::string& message)
	  : Transition(destination), msg(message), haveMsg(true), savedEvent()
	{}

	//! constructor with explicit instance name
	TextMsgTrans(const std::string& name, StateNode* destination, const std::string& message)
	  : Transition(name,destination), msg(message), haveMsg(true), savedEvent()
	{}
	
	//! constructor with explicit instance name but no message; will match anything
	TextMsgTrans(const std::string& name, StateNode* destination)
	  : Transition(name,destination), msg(), haveMsg(false), savedEvent()
	{}
	
	
	// **************************** //
	// ********* METHODS ********** //
	// **************************** //
public:
	virtual void preStart() {
		Transition::preStart();
		erouter->addListener(this, EventBase::textmsgEGID );
	}

	virtual void doEvent() {
    switch ( event->getGeneratorID() ) {

    case EventBase::textmsgEGID:
	    // we're looking to match a specific message
	    if ( haveMsg ) {
	      if ( const TextMsgEvent *txtev = dynamic_cast<const TextMsgEvent*>(event) ) {
          if ( txtev->getText() == msg )
            fire(*event);
          else
            return;
	      }
	    }
	    // default match case (haveMsg is false): set timer so we
	    // can unwind stack and then match this message if no
	    // other transition has fired
	    else {
	      savedEvent = *event;
	      erouter->addTimer(this, 9999, 1, false);
	    }
      break;

	  // timer has expired, so we'll match this message
    case EventBase::timerEGID:
      fire(savedEvent);
      break;

    default:
      std::cout << "TextMsgTrans received unexpected event type: " << event->getDescription() << std::endl;
    }
  }

	static std::string getClassDescription() { return "Fires when a matching string is received"; }
	virtual std::string getDescription() const { return getClassDescription(); }


	// **************************** //
	// ********* MEMBERS ********** //
	// **************************** //
protected:
	std::string msg; //!< the trigger to match messages against
	bool haveMsg; //!< true if msg value was supplied in constructor
	TextMsgEvent savedEvent; //!< copy of triggering event, used for reporting default match


	// **************************** //
	// ********** OTHER *********** //
	// **************************** //
private:
	// Providing declarations for these functions will avoid a compiler warning if
	// you have any class members which are pointers.  However, as it is, an error
	// will result if you inadvertantly cause a call to either (which is probably
	// a good thing, unless you really intended to copy/assign a behavior, in
	// which case simply provide implementations for the functions)
	TextMsgTrans(const TextMsgTrans&); //!< don't call (copy constructor)
	TextMsgTrans& operator=(const TextMsgTrans&); //!< don't call (assignment operator)
};

/*! @file
 * @brief Defines TextMsgTrans, which fires when a matching string is received
 * @author ejt (Creator)
 */

#endif
