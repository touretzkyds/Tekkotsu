//-*-c++-*-
#ifndef INCLUDED_PollThread_h_
#define INCLUDED_PollThread_h_

#ifdef PLATFORM_APERIOS
#  warning PollThread class is not Aperios compatable
#else

#include "Thread.h"
#include "Shared/TimeET.h"

//! description of PollThread
class PollThread : public Thread {
public:
	//! constructor
	PollThread() : Thread(), delay(0L), period(0L), startTime(0L), trackPollTime(false), initialPoll(false) {}
	//! constructor
	explicit PollThread(const TimeET& initial, const TimeET& freq, bool countPollTime)
		: Thread(), delay(initial), period(freq), startTime(0L), trackPollTime(countPollTime), initialPoll(true)
	{}
	//! destructor
	~PollThread() {
		if(isStarted()) {
			stop();
			join();
		}
	}
	
	virtual void start();
	
	virtual bool getTrackPollTime() { return trackPollTime; } //!< returns #trackPollTime
	virtual void setTrackPollTime(bool countPollTime) { trackPollTime=countPollTime; } //!< sets #trackPollTime
	
protected:
	//! this is the function which will be called at the specified frequency, override it with your own functionality
	/*! @return true if run() should continue, false to stop the thread.
	 *  The default implementation calls runloop() and sets #period to its return value if that value is not -1U.
	 *  If the value is -1U, it returns false.  Feel free to override this directly, instead of runloop(),
	 *  particularly if you don't intend to change period dynamically. */
	virtual bool poll();
	
	//! called if a signal is sent while sleeping, should reset #delay to indicate remaining sleep time, relative to startTime
	/*! On return, #delay should be set such that #delay-startTime.Age() is remaining sleep time.  In other words,
	 *  simply set #delay to the #period to maintain previously requested timing.
	 *  
	 *  This default implementation will set #delay to the remaining time needed to maintain current period setting.
	 *  Feel free to override and reset #period (or other member variables) if you need to change timing dynamically.
	 *  If the period is shortened such that poll() should have already occurred based on time of previous
	 *  call and the new period (plus any delay value), then poll() will be called immediately upon return. */
	virtual void interrupted();
	
	virtual void * run();
	//virtual void cancelled();
	
	//! called if a SIGALRM is sent (generally, by a call to interrupt())
	static void handleInterrupt(int signal);
	
	TimeET delay; //!< amount of time to delay between call to start() and first call to poll(), or if interrupt occurs after first poll(), amount of time to re-sleep
	TimeET period; //!< amount of time between calls to poll() -- if zero or negative, no delay will be made between calls (other than a call to testCancel())
	TimeET startTime; //!< the time at which start() was called or the current period began
	bool trackPollTime; //!< if true, the time spent in poll() is subtracted from the next sleep time so frequency is fixed; if false, #period is added onto whatever time poll() takes
	bool initialPoll; //!< set to true after start until after first call to poll has completed
};

#endif //Aperios check

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
