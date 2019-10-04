//-*-c++-*-
#ifndef INCLUDED_LostTargetTrans_h_
#define INCLUDED_LostTargetTrans_h_

#include "Behaviors/Transitions/TimeOutTrans.h"

//! Causes a transition if the target has not been seen @e minframe times within @e delay milliseconds.
class LostTargetTrans : public TimeOutTrans {
 public:

  //! constructor, specify delay in milliseconds
  LostTargetTrans(StateNode* destination, unsigned int source_id,
		  unsigned int delay, int minframes=5) :
    TimeOutTrans("LostTargetTrans","LostTargetTrans",destination,delay),
    sid(source_id), minf(minframes), counter(0) {}

  //! constructor, specify delay in milliseconds
  LostTargetTrans(const std::string &name, StateNode* destination, unsigned int source_id,
		  unsigned int delay, int minframes=5) :
    TimeOutTrans("LostTargetTrans",name,destination,delay),
    sid(source_id), minf(minframes), counter(0) {}

  //!starts timer
  virtual void preStart() {
    TimeOutTrans::preStart();
    erouter->addListener(this,EventBase::visObjEGID,sid);
  }

  virtual void doEvent() {
    if (event->getGeneratorID()==EventBase::visObjEGID && event->getSourceID()==sid) {
      ++counter;
      if (counter > minf) resetTimer();
    }
    else
      TimeOutTrans::doEvent();
  }

  //! resets timer; does not deactivate it
  virtual void resetTimer() {
    TimeOutTrans::resetTimer();
    counter = 0;
  }

  //! set minimum number of frames that target must be seen before resetting the timer
  virtual void set_minframes(int minframes) { minf = minframes; }

protected:
	//! constructor, only to be called by subclasses (which need to specify their own @a classname)
  LostTargetTrans(const std::string &classname, const std::string &instancename, 
		  StateNode* destination, unsigned int source_id,
		  unsigned int delay, int minframes=5) :
    TimeOutTrans(classname,instancename,destination,delay),
    sid(source_id), minf(minframes), counter(0) {}


 private:
  unsigned int sid; //!< the source id from the detector of the object which is being monitored
  int minf;   //!< number of frames that target must be seen before resetting the timer
  int counter; //!< number of frames target has been seen so far
};

#endif

/*! @file
 * @brief Defines LostTargetTrans, which causes a transition if the target has not been seen @e minframe times within @e delay milliseconds.
 * @author ejt (Creator)
 */
