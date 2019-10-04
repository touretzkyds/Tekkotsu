//-*-c++-*-
#ifndef INCLUDED_MapBuilderNode_h_
#define INCLUDED_MapBuilderNode_h_

#include "Behaviors/StateNode.h"
#include "Crew/MapBuilder.h"
#include "Crew/MapBuilderRequest.h"

using namespace DualCoding;

//! Creates a MapBuilderRequest @a mapreq that the user can modify in their doStart() function, then executes the request when doStart() returns
class MapBuilderNode : public StateNode {
public:
  //! Constructor
  MapBuilderNode(MapBuilderRequest::MapBuilderRequestType_t requestType=MapBuilderRequest::cameraMap) :
    StateNode(), mapreq(requestType), cancelFlag(false) {}

  //! Constructor
  MapBuilderNode(std::string const name,
		 MapBuilderRequest::MapBuilderRequestType_t requestType=MapBuilderRequest::cameraMap) :
    StateNode(name), mapreq(requestType), cancelFlag(false) {}

  virtual void preStart() {
    cancelFlag = false;
    StateNode::preStart();
  }
	
  virtual void postStart() {
    StateNode::postStart();
    if ( ! cancelFlag ) {
      erouter->addListener(this, EventBase::mapbuilderEGID, (size_t)this, EventBase::statusETID);
      VRmixin::mapBuilder->executeRequest(this,mapreq);
    }
  }

  virtual void doEvent() {
    postStateCompletion();
  }

  //! If called inside doStart, prevents request from being passed to MapBuilder, and posts a failure event
  virtual void cancelThisRequest() {
    cancelFlag = true;
    postStateFailure();
  }

protected:
  MapBuilderRequest mapreq;
  bool cancelFlag;  //!< If set true in doStart, request will not be submitted to the MapBuilder

};

#endif
