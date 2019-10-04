#include "CompletionTrans.h"

void CompletionTrans::postStart() {
  Transition::postStart();
  unsigned int const numsrcs = getSources().size();
  completions = new bool[numsrcs];
  for (unsigned int i = 0; i < numsrcs; i++) {
    completions[i] = false;
    erouter->addListener(this, EventBase::stateMachineEGID, 
                         reinterpret_cast<size_t>(getSources()[i]), EventBase::statusETID);
  };
}

void CompletionTrans::stop() {
  erouter->removeListener(this);
  delete completions;
  completions = NULL;
  Transition::stop();
}

void CompletionTrans::doEvent() {
  switch ( event->getGeneratorID() ) {

  case EventBase::stateMachineEGID: {
    int numcomplete = 0;
    for ( unsigned int i=0; i<getSources().size(); i++ ) {
      if ( event->getSourceID() == reinterpret_cast<size_t>(getSources()[i]) )
        completions[i] = true;
      if ( completions[i] )
        ++numcomplete;
    }
    int const threshold = ( (minsrcs > 0) ? minsrcs : (int)getSources().size() );
    if ( numcomplete >= threshold )
      savedEvent = *event;
      erouter->addTimer(this, 9999, 0, false);
    break;
  }

  case EventBase::timerEGID:
    fire(savedEvent);

  default: ;
  }
}

