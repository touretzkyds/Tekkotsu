//-*-c++-*-
#ifndef INCLUDED_GrasperTrans_h_
#define INCLUDED_GrasperTrans_h_

#include "Behaviors/Transition.h"
#include "Events/GrasperEvent.h"
#include "Events/EventRouter.h"

//! Causes a transition if a GrasperEvent from grasperEGID occurs, and checks for a specific error value if one is supplied.
/*! This allows a GrasperNode to signal a transition to another state
 *  in a clean symbolic way.  Only the transition itself needs to
 *  know the address of the destination node.  The GrasperEvent
 *  will be supplied to the destination node's doStart() method.
 */

class GrasperTrans : public Transition {
public:
  //! Constructor
  GrasperTrans(StateNode *destination) :
    Transition(destination), errorCode(), valueSupplied(false)
  { }

  //! Constructor
  GrasperTrans(StateNode *destination, GrasperRequest::GrasperErrorType_t value) :
    Transition(destination), errorCode(value), valueSupplied(true)
  { }

  //! Constructor
  GrasperTrans(const std::string &name, StateNode *destination) :
    Transition(name,destination), errorCode(), valueSupplied(false)
  { }

  //! Constructor
  GrasperTrans(const std::string &name, StateNode *destination, GrasperRequest::GrasperErrorType_t value) :
    Transition(name,destination), errorCode(value), valueSupplied(true)
  { }

  virtual void postStart() {
    Transition::postStart();
    for ( std::vector<StateNode*>::const_iterator it = srcs.begin(); it != srcs.end(); it++ )
      erouter->addListener(this,EventBase::grasperEGID,(size_t)*it);
  }

  virtual void doEvent() {
    const GrasperEvent *graspEvent = dynamic_cast<const GrasperEvent*>(event);
    if ( graspEvent != NULL )
      // Fire if we have a GrasperEvent and (1) no errorCode was
      // specified for the transition, or (2) the event matches the
      // specified code, or (3) the specified code is someError and
      // the event has any code other than noError.
      if ( !valueSupplied || 
	   graspEvent->getErrorType() == errorCode ||
	   (errorCode == GrasperRequest::someError && graspEvent->getErrorType() != GrasperRequest::noError) )
	fire(*event);
  }

  //! Copy constructor required in case we're storing a pointer
  GrasperTrans(const GrasperTrans &src) : Transition(src), errorCode(src.errorCode), valueSupplied(src.valueSupplied) {}
  
  //! Assignment operator required in case we're storing a pointer
  GrasperTrans& operator=(const GrasperTrans& src) {
    errorCode = src.errorCode;
    valueSupplied = src.valueSupplied;
    return *this;
  }

protected:
  GrasperRequest::GrasperErrorType_t errorCode; //!< value to compare against
  bool valueSupplied;  //!< true if a value was supplied in the constructor
};

#endif
