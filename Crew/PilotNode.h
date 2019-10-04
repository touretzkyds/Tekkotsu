 //-*-c++-*-
#ifndef INCLUDED_PilotNode_h_
#define INCLUDED_PilotNode_h_

#include "Crew/PilotRequest.h"
#include "Behaviors/StateNode.h"

using namespace DualCoding;

//! Creates a PilotRequest @a pilotreq that the user can modify in their doStart() function, then executes the request when doStart() returns
class PilotNode : public StateNode {
public:
  //! Constructor
  PilotNode(PilotTypes::RequestType_t requestType = PilotTypes::noRequest) :
    StateNode(), pilotreq(requestType), cancelFlag(false) {}

  //! Constructor
  PilotNode(std::string const &name, PilotTypes::RequestType_t requestType = PilotTypes::noRequest) :
    StateNode(name), pilotreq(requestType), cancelFlag(false) {}

  virtual void stop();

  //! If called inside doStart, prevents request from being passed to Pilot, and posts a failure event
  virtual void cancelThisRequest();

  PilotRequest pilotreq;  // must be public so state machine node initializers can access it

protected:
  virtual void preStart();
  virtual void postStart();
  virtual void doEvent();
	
  bool cancelFlag;  //!< If set true in doStart, request will not be submitted to the Pilot
};

#endif
