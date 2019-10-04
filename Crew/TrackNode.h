 //-*-c++-*-
#ifndef INCLUDED_TrackNode_h_
#define INCLUDED_TrackNode_h_

#include "Behaviors/StateNode.h"

using namespace DualCoding;

//! Creates a LookoutTrackRequest @a trackreq that the user can modify in their doStart() function, then has the Lookout execute the request when doStart() returns
class TrackNode : public StateNode {
public:
  //! Constructor
  TrackNode() :
    StateNode(), trackreq(), cancelFlag(false) {}

  //! Constructor
  TrackNode(std::string const &name) :
    StateNode(name), trackreq(), cancelFlag(false) {}

  virtual void stop();

  //! If called inside doStart, prevents request from being passed to Track, and posts a failure event
  virtual void cancelThisRequest();

  LookoutTrackRequest trackreq;  // must be public so state machine node initializers can access it

protected:
  virtual void preStart();
  virtual void postStart();
  virtual void doEvent();
	
  bool cancelFlag;  //!< If set true in doStart, request will not be submitted to the Track
};

#endif
