#include "Shared/RobotInfo.h"
#if (defined(TGT_HAS_ARMS) || !defined(STRICT_TGT_MODEL)) && !defined(TGT_IS_AIBO)

#include "GrasperNode.h"
#include "Crew/Grasper.h"
#include "Events/GrasperEvent.h"
#include "Shared/MarkScope.h"

void GrasperNode::preStart() {
  StateNode::preStart();
  cancelFlag = false;
}

void GrasperNode::postStart() {
  StateNode::postStart();
#ifdef TGT_HAS_ARMS
  if ( !cancelFlag ) {
    VRmixin::grasper->executeRequest(graspreq,this);
    erouter->addListener(this,EventBase::grasperEGID,(size_t)this);
  }
#endif
}

void GrasperNode::doEvent() {
  // Normally a =GRASP=> event will trigger on any Grasper event, but
  // if the user is testing for explicit error codes using =GRASP(t)=> and
  // none of the tests succeeds, we want to provide for a default case by
  // posting a failure event that can be picked up by an =F=> transition.
  switch ( event->getGeneratorID() ) {
  case EventBase::grasperEGID: {
    const GrasperEvent *graspEvent = dynamic_cast<const GrasperEvent*>(event);
    if ( graspEvent && graspEvent->getErrorType() != GrasperRequest::noError )
      erouter->addTimer(this,1,1,false);   // delay so any =GRASP(t)=> transitions can fire first
    else
      postStateCompletion();
    break;
  }
  case EventBase::timerEGID:  // delay has expired and nothing else fired; post failure
    postStateFailure();
    break;
  default:
    break;
  }
}

void GrasperNode::cancelThisRequest() {
  cancelFlag = true;
  postStateFailure();
}

#endif
