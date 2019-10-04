//-*-c++-*-
#ifndef INCLUDED_SignalTrans_h_
#define INCLUDED_SignalTrans_h_

#include "Behaviors/Transition.h"
#include "Events/DataEvent.h"
#include "Events/EventRouter.h"

//! causes a transition if a DataEvent<T> from stateSignalEGID occurs, and checks for a specific value if one is specified
/*! This allows a state node to signal a transition to another state
 *  in a clean symbolic way.  Only the transition itself needs to know
 *  the address of the destination node.  The value passed in the
 *  DataEvent will be supplied to the destination node's doStart
 *  method.  A SignalTrans with no value supplied acts like a
 *  default case and will fire if no other SignalTrans fires first.
 */

template<class T>
class SignalTrans : public Transition {
public:
  //! Constructor
  SignalTrans(StateNode *destination) :
    Transition(destination), val(), valueSupplied(false), savedEvent()
  { }

  //! Constructor
  SignalTrans(StateNode *destination, const T &value) :
    Transition(destination), val(value), valueSupplied(true), savedEvent()
  { }

  //! Constructor
  SignalTrans(const std::string &name, StateNode *destination) :
    Transition(name,destination), val(), valueSupplied(false), savedEvent()
  { }

  //! Constructor
  SignalTrans(const std::string &name, StateNode *destination, const T &value) :
    Transition(name,destination), val(value), valueSupplied(true), savedEvent()
  { }

  virtual void postStart() {
    Transition::postStart();
    for ( std::vector<StateNode*>::const_iterator it = srcs.begin(); it != srcs.end(); it++ )
      erouter->addListener(this,EventBase::stateSignalEGID,(size_t)*it);
  }

  virtual void doEvent() {
    switch ( event->getGeneratorID() ) {

    case EventBase::stateSignalEGID: {
      const DataEvent<T> *d_event = dynamic_cast<const DataEvent<T>*>(event);
      if ( d_event != NULL ) {
        savedEvent = *d_event;
        if ( ! valueSupplied )
          // wildcard case: wait 2 msec to see if some other SignalTrans matches the value
          erouter->addTimer(this, 9999, 1, false);
        // not wildcard: if supplied value matches the event, queue for firing
        else if ( d_event->getData() == val )
          erouter->addTimer(this, 9999, 0, false);
      }
      break;
    }

    case EventBase::timerEGID:
      fire(savedEvent);
      break;

    default:
      std::cout << "SignalTrans received unexpected event type: " << event->getDescription() << std::endl;
    }
  }

  //! Copy constructor required in case we're storing a pointer
  SignalTrans<T>(const SignalTrans<T> &src) : Transition(src), val(T(src.val)), valueSupplied(src.valueSupplied) {}

  //! Assignment operator required in case we're storing a pointer
  SignalTrans<T>& operator=(const SignalTrans<T>& src) {
    val = src.val;
    valueSupplied = src.valueSupplied;
    return *this;
  }

protected:
  T val; //!< value to compare against
  bool valueSupplied;  //!< true if a value was supplied in the constructor
  DataEvent<T> savedEvent; //!< Copy of current event to be used after timer expires
};

#endif
