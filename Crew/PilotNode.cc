#include "Motion/WalkMC.h"
#ifdef TGT_HAS_WALK
#include "Crew/Pilot.h"
#endif

#include "Events/PilotEvent.h"

#include "PilotNode.h"

void PilotNode::preStart() {
  StateNode::preStart();
  cancelFlag = false;
  const PilotRequest *p = tryExtractSignal<PilotRequest>(event);
  if ( p )
    pilotreq = *p;
}

void PilotNode::postStart() {
  StateNode::postStart();
  if ( !cancelFlag ) {
    erouter->addListener(this, EventBase::pilotEGID, (size_t)this, EventBase::statusETID);
#ifdef TGT_HAS_WALK
    VRmixin::pilot->executeRequest(this,pilotreq);
#endif
  }
}

void PilotNode::stop() {
#ifdef TGT_HAS_WALK
  VRmixin::pilot->pilotPop();  // in case the request was still executing when we exited
#endif
  StateNode::stop();
}

void PilotNode::doEvent() {
  postStateCompletion();
}

void PilotNode::cancelThisRequest() {
  cancelFlag = true;
  postStateFailure();
}
