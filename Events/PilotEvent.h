//-*-c++-*-
#ifndef INCLUDED_PilotEvent_h_
#define INCLUDED_PilotEvent_h_

#include <iostream>

#include "EventBase.h"

#include "Crew/PilotRequest.h"
#include "Crew/PilotTypes.h"

using namespace DualCoding::PilotTypes;

//! Event for reporting the results of a Pilot operation
class PilotEvent : public EventBase {
public:
	
  RequestType_t requestType;
  ErrorType_t errorType;
	
  virtual RequestType_t getRequestType() const { return requestType; }
  virtual ErrorType_t getErrorType() const { return errorType; }
	
  //! Constructor
  PilotEvent() : EventBase(), requestType(), errorType(noError) {}

  //! Constructor
  PilotEvent(EventGeneratorID_t gid, size_t sid, EventTypeID_t tid, unsigned int dur=0)
    : EventBase(gid,sid,tid,dur), requestType(), errorType(noError) {}
	
  virtual EventBase* clone() const { return new PilotEvent(*this); }
	
  PilotEvent(const PilotEvent &other) :
    EventBase(other), requestType(other.requestType), errorType(other.errorType) {}

};

#endif
