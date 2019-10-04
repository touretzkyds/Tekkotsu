//-*-c++-*-
#ifndef INCLUDED_FailsafeThread_h_
#define INCLUDED_FailsafeThread_h_

#include <unistd.h> // for usleep

#include "Shared/TimeET.h"

//! Enforces a timeout on another thread
/*! The target thread needs to either complete execution or set the progressFlag to 'true'
 *  within the specified timeout period.  If the progressFlag is set, it will be cleared at the
 *  end of the timeout, thus requiring the target to re-set within the next timeout period. */
class FailsafeThread : public Thread {
public:
	//! constructor, specify target thread, timeout period, and optionally whether to start now
	FailsafeThread(Thread& th, const TimeET& delayTime, bool autostart=false)
	: Thread(), restartFlag(false), progressFlag(false), delay(useconds_t(delayTime.Value()*1000000)), engageFunc(&Thread::stop), target(th), engaged(false) { if(autostart) start(); }
	
	//! if set to true, the failsafe thread will restart the target if it times out instead of just stopping it
	volatile bool restartFlag;
	
	//! should be set by target thread if it's still making progress and wants another #delay
	volatile bool progressFlag;
	
	//! microseconds to wait between checks on #progressFlag
	/*! Changing this value won't change the @e current timeout period.  You would need to stop and
	 *  restart the thread for a change to immediately take effect. */
	volatile useconds_t delay;
	
	//! the function to call on the target thread, defaults to Thread::stop, but Thread::interrupt may be useful
	Thread& (Thread::*engageFunc)();
		
	//! returns true if the FailsafeThread is waiting for the target to stop running
	/*! This is useful for the target thread to check whether it is being stopped from a timeout
	 *  (in which case isEngaged() will return true), or if it has been stopped for some other reason. */
	bool isEngaged() const { return engaged; }
	
protected:
	virtual unsigned int runloop() {
		engaged=false;
		// sleep as long as progress is being made
		usleep(delay);
		testCancel();
		while(progressFlag) {
			progressFlag=false;
			usleep(delay);
			testCancel();
		}
		// no more progress -- is the thread still running?
		if(!target.isStarted())
			return -1U; // no, go away
		// yes, stop it
		//std::cout << "failsafe engaged" << std::endl;
		engaged=true;
		(target.*engageFunc)();
		// we killed it... restart it?
		if(!restartFlag)
			return -1U; // no, go away
		// yes, wait for stop to go through, then start it again
		target.join();
		if(!restartFlag) // check again just in case restart was changed while waiting for join()
			return -1U;
		testCancel();
		target.start();
		engaged=false;
		return 0;
	}
	
	//virtual void cancelled() { engaged=false; }
	
	//! the thread being monitored (or at least the one that will be stopped if progressFlag isn't set)
	Thread& target;
	
	//! set to true when FailsafeThread is in the process of stopping (and possibly restarting) the target thread
	bool engaged;
};


/*! @file
 * @brief Describes FailsafeThread, which enforces a timeout on another thread
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
