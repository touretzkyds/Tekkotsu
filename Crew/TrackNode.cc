#ifdef TGT_HAS_HEAD
#include "Crew/Lookout.h"
#endif

#include "Events/LookoutEvents.h"

#include "TrackNode.h"

void TrackNode::preStart() {
  StateNode::preStart();
  cancelFlag = false;
  const LookoutTrackRequest *p = tryExtractSignal<LookoutTrackRequest>(event);
  if ( p )
    trackreq = *p;
}

void TrackNode::postStart() {
  StateNode::postStart();
  if ( !cancelFlag ) {
    erouter->addListener(this, EventBase::lookoutEGID, (size_t)this, EventBase::statusETID);
#ifdef TGT_HAS_HEAD
    VRmixin::lookout->executeRequest(this,trackreq);
#endif
  }
}

void TrackNode::stop() {
#ifdef TGT_HAS_HEAD
  VRmixin::lookout->stopTrack();  // in case the request was still executing when we exited
#endif
  StateNode::stop();
}

void TrackNode::doEvent() {
  postStateCompletion();
}

void TrackNode::cancelThisRequest() {
  cancelFlag = true;
  postStateFailure();
}
