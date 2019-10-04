//-*-c++-*-
#ifndef INCLUDED_TimerExecThread_h_
#define INCLUDED_TimerExecThread_h_

#include "IPC/PollThread.h"
#include "Shared/Resource.h"

//! executes EventRouter::processTimers() as necessary (allows timers to work without any other vision or sensor processing)
class TimerExecThread : public PollThread {
public:
	explicit TimerExecThread(Resource& bl, bool autoStart=true) : PollThread(), behaviorLock(bl) { if(autoStart) reset(); }
	virtual void reset(); //!< starts and stops thread as needed, or interrupts thread to reset sleep time if already running
	
protected:
	virtual long calcSleepTime(); //!< returns the time in milliseconds to sleep until the next timer; resets PollThread::startTime
	virtual bool launched();
	virtual bool poll();
	virtual void interrupted();

	Resource& behaviorLock; //!< a lock on behaviors which should be obtained before processing timer events
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
