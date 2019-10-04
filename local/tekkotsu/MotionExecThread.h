//-*-c++-*-
#ifndef INCLUDED_MotionExecThread_h_
#define INCLUDED_MotionExecThread_h_

#include "IPC/PollThread.h"
#include "Shared/RobotInfo.h"
#include "IPC/MessageQueue.h"
#include "Shared/get_time.h"
#include "Motion/PostureEngine.h"
#include "SharedGlobals.h"
#include <list>
#include <fcntl.h>

#include "local/MotionHook.h"

class RCRegion;
class Resource;

//! description of MotionExecThread
class MotionExecThread : public PollThread {
public:
	//! constructor, enables trackPollTime, but not auto-start (call reset() when you're ready to start it)
	/*! @arg bl a process lock to ensure mutual exclusion between MotionExecThread::poll() and other threads in the process */
	MotionExecThread(Resource& bl)
		: PollThread(0L, FrameTime*NumFrames/globals->timeScale/1000, true), motionLock(bl),
		motionBuffers(), motionBufferPos(), lastPoll(-1U)
	{
		motionBuffers.push_front(new float[NumFrames][NumOutputs]);
		for(unsigned int f=0; f<NumFrames; ++f)
			for(unsigned int o=0; o<NumOutputs; ++o)
				motionBuffers.front()[f][o]=0;
		motionBufferPos=motionBuffers.begin();
	}
	virtual ~MotionExecThread() {
		if(isStarted()) {
			stop();
			join();
		}
		while(motionBuffers.size()>0) {
			delete [] motionBuffers.front();
			motionBuffers.pop_front();
		}
	}
	
	virtual void reset(); //!< starts and stops thread as needed, or interrupts thread to reset sleep time if already running
	
	//virtual void start();
	virtual bool poll();

	//! returns time (in milliseconds) of next motion frame -- multiples of FrameTime*NumFrames
	static unsigned int getNextMotion() {
		unsigned int pd=FrameTime*NumFrames;
		return (get_time()/pd+1)*pd;
	}

	//! writes output values, delayed by SharedGlobals::MotionSimConfig::feedbackDelay, as "sensor" values for outputs without real feedback into SharedGlobals::sensorState
	void applyPostureFeedback();
	
protected:
	virtual bool launched();
	//! resets PollThread::delay and PollThread::period to appropriate values for current SharedGlobals::timeScale value
	virtual void interrupted();
	
	Resource& motionLock; //!< a lock on the motions which should be obtained before updating

	std::list<float(*)[NumOutputs]> motionBuffers;
	std::list<float(*)[NumOutputs]>::iterator motionBufferPos;
	
	unsigned int lastPoll;
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
