#include "PilotTrans.h"
#include "DualCoding/VRmixin.h"
#include "Shared/MarkScope.h"

void PilotTrans::postStart() {
  Transition::postStart();
  for ( std::vector<StateNode*>::const_iterator it = srcs.begin(); it != srcs.end(); it++ )
    erouter->addListener(this,EventBase::pilotEGID,(size_t)*it);
}

void PilotTrans::doEvent() {
  switch ( event->getGeneratorID() ) {
  case EventBase::pilotEGID: {
    const PilotEvent *pilotEvent = dynamic_cast<const PilotEvent*>(event);
    if ( pilotEvent != NULL ) {
      // If no error value was supplied with the transition, this
      // transition is for the default case: start a timer and fire
      // if no other transition fires first.
      if ( ! valueSupplied ) {
	savedEvent = *event;
	erouter->addTimer(this, 9999, 1, false);
      }
      else
	// Fire if the supplied type matches the type in the event,
	// or the supplied type is someError and the event's error
	// type is anything but noError.
	if ( pilotEvent->getErrorType() == errorType ||
	     (errorType == someError && pilotEvent->getErrorType() != noError) )
	  fire(*event);
    }
    break;
  }
  case EventBase::timerEGID:
    fire(savedEvent);
    break;
  default:
    std::cout << "PilotTrans received unexpected event type: " << event->getDescription() << std::endl;
  }
}

PilotTrans::PilotTrans(const PilotTrans &src) : 
  Transition(src), errorType(src.errorType), valueSupplied(src.valueSupplied), savedEvent(src.savedEvent) {}
