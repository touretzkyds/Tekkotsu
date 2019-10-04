#ifndef INCLUDED_GrasperNode_h_
#define INCLUDED_GrasperNode_h_

#include "Crew/GrasperRequest.h"
#include "Behaviors/StateNode.h"

extern MotionManagerMsg::MC_ID motionNodesWalkMC;

//! Creates a GrasperRequest @a graspreq that the user can modify in their doStart() function, then executes the request when doStart() returns
class GrasperNode : public StateNode {
public:
  //! Constructor
  GrasperNode(GrasperRequest::GrasperRequestType_t requestType)
    : StateNode(), graspreq(requestType), cancelFlag(false) {}
	
  //! Constructor
  GrasperNode(std::string const name, GrasperRequest::GrasperRequestType_t requestType)
    : StateNode(name), graspreq(requestType), cancelFlag(false) {}
	
  virtual void cancelThisRequest();
	
protected:
  virtual void preStart();
  virtual void postStart();
  virtual void doEvent();
	
  GrasperRequest graspreq;
  bool cancelFlag;
	
};

#endif
