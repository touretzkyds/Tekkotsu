//-*-c++-*-
#ifndef INCLUDED_PilotTrans_h_
#define INCLUDED_PilotTrans_h_

#include "Behaviors/Transition.h"
#include "Events/PilotEvent.h"
#include "Events/EventRouter.h"
#include "Crew/PilotTypes.h"

//! Causes a transition if a PilotEvent from pilotEGID occurs, and checks for a specific error value if one is supplied.
/*! This allows a PilotNode to signal a transition to another state
 *  in a clean symbolic way.  Only the transition itself needs to
 *  know the address of the destination node.  The PilotEvent
 *  will be supplied to the destination node's doStart() method.
 */

class PilotTrans : public Transition {
public:
  //! Constructor
  PilotTrans(StateNode *destination) :
    Transition(destination), errorType(), valueSupplied(false), savedEvent() {}

  //! Constructor
  PilotTrans(StateNode *destination, DualCoding::PilotTypes::ErrorType_t value) :
    Transition(destination), errorType(value), valueSupplied(true), savedEvent() {}

  //! Constructor
  PilotTrans(const std::string &name, StateNode *destination) :
    Transition(name,destination), errorType(), valueSupplied(false), savedEvent() {}

  //! Constructor
  PilotTrans(const std::string &name, StateNode *destination, DualCoding::PilotTypes::ErrorType_t value) :
    Transition(name,destination), errorType(value), valueSupplied(true), savedEvent() {}

  //! Copy constructor
  PilotTrans(const PilotTrans &src);
  
  virtual void postStart();
  virtual void doEvent();

protected:
  DualCoding::PilotTypes::ErrorType_t errorType; //!< value to compare against
  bool valueSupplied;  //!< true if a value was supplied in the constructor
  PilotEvent savedEvent;

private:
  PilotTrans& operator=(const PilotTrans& src);  //!< don't call this

};

#endif
