//-*-c++-*-
#ifndef INCLUDED_Thread_h_
#define INCLUDED_Thread_h_

#ifdef PLATFORM_APERIOS
#  warning Thread class is not Aperios compatable
#else

#ifdef __APPLE__
/* Mac OS X doesn't handle pthread_cancel in (most) system calls, (read, listen, etc.) so we need to send a signal to wake it up. */
#  define USE_SIGNAL_TO_CANCEL_THREAD
/* Further, OS X doesn't currently (10.6 and prior) correctly unwind the stack, so destructors
  * aren't called — ideally, catch(...) blocks should also be triggered.  So we don't use pthread_cancel()
  * at all!  Instead, stop() will interrupt() and then expect the thread to call testCancel to generate our
  * own exception. */
#  undef USE_PTHREAD_CANCEL
#  undef USE_TESTCANCEL_IN_INTERRUPT
#else
/* On other platforms, we'll assume pthread_cancel does an exception-style cancellation like linux does */
#  define USE_PTHREAD_CANCEL
#  define USE_TESTCANCEL_IN_INTERRUPT
#endif


#include "Shared/Resource.h"
#include <stddef.h>

struct timespec;

namespace stacktrace {
	struct StackFrame;
}

//! Provides a nice wrapping of pthreads library
/*! If you need to provide cleanup functions on stop(), cancelled(), etc., you
 *  should override the destructor to stop and join so that you can be assured
 *  that your cleanup will be called if the thread is auto-destructed by going out of scope */
class Thread {
public:
	class Condition;
	
	//! an inter-thread lock -- doesn't work across processes, only threads within a process.  (see MutexLock for inter-process locks)
	class Lock : public Resource {
	public:
		Lock(); //!< constructor
		//explicit Lock(const Lock& l); //!< copy constructor -- shallow copy, share a lock, is handy for locking over a scope!!! (lock is automatically obtained on copy -- to avoid autolock, pass false to the two-argument constructor: Lock(const Lock& l, bool autolock) )
		//Lock(const Lock& l, bool autolock); //!< copy constructor -- shallow copy, share a lock, is handy for locking over a scope!!!
		//Lock& operator=(const Lock& l); //!< assignment -- dereference (and release) any previous lock, take on the new storage (shallow copy!)
		~Lock(); //!< destructor -- dereference and release (if any references remain)
		void lock() { Thread::pushNoCancel(); useResource(emptyData); } //!< block until lock is obtained
		bool trylock(); //!< see if lock is available
		void unlock() { releaseResource(emptyData); Thread::popNoCancel(); } //!< release lock, if held
		unsigned int getInstanceLockLevel() const { return locklevel; } //!< returns the lock level of the local instance of Lock (as opposed to the lock storage structure, which might be shared with other Lock instances)
		unsigned int getLockLevel() const; //!< returns the lock level of the lock storage itself, the sum of all instance's lock levels
	protected:
		friend class MarkScope;
		friend class Condition;
		virtual void useResource(Resource::Data&);
		virtual void releaseResource(Resource::Data&);
		
		class LockStorage; //!< this internal class will hold the system-dependent lock information
		static LockStorage* glock; //!< The global lock to protect Locks sharing #mylock's
		LockStorage* mylock; //!< This lock's implementation
		static void setup(); //!< creates a new #glock if it is currently NULL (should be called by the Lock() constructor)
		unsigned int locklevel; //!< the current lock level from this Lock, may differ from #mylock's lock level if several Locks are sharing a storage!
	private:
		Lock(const Lock& l); //!< don't call
		Lock& operator=(const Lock& l); //!< don't call
	};
	
	//! Provides an inter-thread signaling and synchronization mechanism
	/*! When waiting (either wait() or timedwait()), the lock argument must initially
	 *  be passed in locked state.  During the wait, the lock is released, and then reaquired
	 *  before returning, regardless of return status (i.e. timeout).  This seems to be enforced at a 
	 *  priority that precludes thread cancellation (tested on Linux and Mac OS X), so
	 *  although a Thread::stop() call will cancel the conditional wait, it won't cancel
	 *  the subsequent lock acquisition (even with additional Thread::stop() calls!)
	 *
	 *  On USE_SIGNAL_TO_CANCEL_THREAD platforms (i.e. Mac OS X), wait() and
	 *  timedwait() imply a call to Thread::testCurrentCancel in order to handle
	 *  cancellation, so you can assume if they return 'naturally' that the condition
	 *  was detected, not cancellation. */
	class Condition {
	public:
		Condition(); //!< constructor
		~Condition(); //!< destructor
		
		void broadcast() const; //!< wake up all threads waiting on the condition
		void signal() const; //!< wake up a single thread waiting on the condition (which thread is unspecified)
		bool timedwait(Lock& l, const timespec* abstime, bool noWarn=false) const; //!< wait for at most @a abstime for the condition before giving up (return true if condition found), @a l must be locked, will be released and re-aquired (regardless of signal vs. timeout,  see class overview docs)
		void wait(Lock& l, bool noWarn=false) const; //!< wait for condition, @a l must be locked, will be released and re-aquired (see class overview docs)
	protected:
		static void displayRecursiveLockWarning(const char * fn, unsigned int locklevel);
		class ConditionStorage; //!< internal class to hold system-dependent information
		ConditionStorage* mycond; //!< the condition's implementation storage
	private:
		Condition(const Condition& l); //!< don't call
		Condition& operator=(const Condition& l); //!< don't call
	};
	
	struct NoCancelScope {
		NoCancelScope() { Thread::pushNoCancel(); }
		~NoCancelScope() { Thread::popNoCancel(); }
	};
	
	static void* CANCELLED; //!< return value from join() when the thread was cancelled instead of returning a value

	Thread(); //!< constructor, does not start thread by itself (although subclasses may)
	
	//! destructor, will stop and join the thread, but you should override it to do the same if you provide any cleanup functions
	/*! Note that this destructor will send a stop() signal... if you want your subclass 
	 *  to let the thread run to its "natural" completion on destruction, you can 
	 *  either pushNoCancel() within the thread, or override the destructor to join() without stop() */
	virtual ~Thread()=0; 
	
	//! requests that the thread be started, if not already running (you need to create separate instances if you want to run multiple copies)
	virtual void start();
	
	//! sends a signal (SIGALRM) to the thread which will interrupt any sleep/read/etc. calls (and trigger interrupted() to be called within the thread)
	/*! This may be called to request a cancellation on systems which don't directly support
	 *  pthread_cancel they way we would like.  See stop(). */
	virtual Thread& interrupt();
	
	//! requests that the thread be stopped gracefully, if running.
	/*! A cancel flag is sent, and the thread will be stopped at next cancel point, defined
	 *  by whenever testCancel(), or a set of other system functions, are called.
	 *  See your system's pthread_testcancel() manual page for a list of cancel points.
	 *
	 *  This function may imply a call to interrupt() on systems which have extremely limited
	 *  system cancel points or don't handle thread cancellation as a C++ exception (currently
	 *  this consists of Mac OS X and possibly other BSD-based systems).
	 *  
	 *  This means that you should be able to rely on using try/catch(...) to handle both
	 *  exceptions as well as thread cancellation, but interruptable system calls
	 *  should test errno for EINTR and call testCancel() when it is encountered to ensure
	 *  portability.
	 *
	 *  Returns *this, for convenience of chaining a call to join()
	 *
	 *  @see pushNoCancel(), popNoCancel() */
	virtual Thread& stop();
	
	//! sends a SIGUSR1 to the thread, breaking its execution, but still allowing handle_exit (and thus cancelled()) to be called.
	/*! Beware if your thread uses mutual exclusion locks, this can cause the thread to terminate while still holding locks
	 *  Returns *this, for convenience of chaining a call to join() */
	virtual Thread& kill();
	
	//! detaches thread and sends SIGSTOP, which immediately halts the thread without any chance for cleanup
	/*! Beware if your thread uses mutual exclusion locks, this @b will cause the thread to terminate while still holding locks.
	 *  Returns *this, for convenience of chaining a call to join() */
	virtual Thread& murder();
	
	//! sends a signal to the thread
	virtual void sendSignal(int sig);
	
	//! blocks calling thread until this Thread has terminated, either via run() returning of its own accord, or a stop() cancelling the thread
	/*! return value is the response from run(), or #CANCELLED if stop() was called.
	  *  See getReturnValue() as a possible way to get results from a cancelled thread. */
	virtual void * join() const;
	
	//! returns #returnValue, for use with threads which may have intermediate results, or partial results following a cancellation
	virtual void * getReturnValue() { return returnValue; }
	
	//! indicates whether start() has been called (but may be some delay before isRunning() is true...)
	virtual bool isStarted() const { return started; }
	
	//! indicates whether the thread is currently alive and running, implies isStarted()
	virtual bool isRunning() const { return running; }
	
	//! returns the Thread object for the current thread (or NULL for the main thread)
	static Thread* getCurrent() ;
	
	//! should be called before any threads are created to allow some global thread-specific data to be set up
	static void initMainThread();
	//! should be called if you no longer expect to have any threads in use
	static void releaseMainThread();

	//! should be called whenever a critical section has been entered (i.e. mutex obtained) -- prevents cancel from occurring until popNoCancel() is called
	static void pushNoCancel();
	//! should be called whenever a critical section is left (i.e. mutex released) -- if it was the last one, tests cancellability as well if @a doTestCancel (generally should, except in destructors, may already be unwinding from exception, would cause terminate)
	static void popNoCancel(bool doTestCancel=true);
	
	//! Should be called before system calls which are not cancellation points, but support interruption by signal.
	/*! On OS X, @e everything is done this way, so no need to call this function.  Only needed for calls on systems
	 *  where we actually use pthread_cancel (Linux), but at system calls which are not cancellation points.
	 *  At the moment, semop() is the only known case. */
	static void requestInterruptOnCancel();
	//! Should be called after system calls which are not cancellation points, but support interruption by signal.
	/*! On OS X, @e everything is done this way, so no need to call this function.  Only needed for calls on systems
	 *  where we actually use pthread_cancel (Linux), but at system calls which are not cancellation points.
	 *  At the moment, semop() is the only known case. */
	static void unrequestInterruptOnCancel();
	
	//! Acquiring this lock allows you to atomically test whether the thread is already running and (re)start it or stop/join it
	Lock& getStartLock() { return startLock; }
	
	//! returns #group
	void* getGroup() const { return group; }
	//! assigns #group, which will then be inherited by any threads instantiated by this one (the constructor call queries the current thread, no the start() or launch())
	void setGroup(void* g) { group=g; }
	
	//! checks to see if stop() has been called for the current thread, and if so, will exit (passing through handle_exit() first)
	static void testCurrentCancel();
	
protected:
	//! called by launch() when thread is first entered, return false to cancel launch (set #returnValue as well if you care)
	virtual bool launched() { return true; }
	//! called by launch() once the thread has been set up; when this returns, the thread ends, see runloop()
	/*! Default implementation repeatedly calls runloop(), usleep(), and testCancel().
	 *  If you override, you should also be sure to call testCancel occasionally in order to support stop()
	 *  If function returns a value, that value overrides #returnValue.  If cancel occurs, #returnValue is used. */
	virtual void * run();
	//! override this as a convenient way to define your thread -- return the number of *micro*seconds to sleep before the next call; return -1U to indicate end of processing
	virtual unsigned int runloop() { return -1U; }
	//! called when handle_exit() is triggered, either by the thread being cancelled, or when run() has returned voluntarily
	virtual void cancelled() {}
	//! called as last instruction of handle_exit(), following cancelled() and all other cleanup.  A self-deleting thread should do so here.
	virtual void dereference() {}
	
	//! checks to see if stop() has been called for the currently executing thread, and if so, will exit (passing through handle_exit() first)
	static void testCancel() { testCurrentCancel(); }
	//! thread entry point -- calls launched() on the thread (as indicated by @a msg), and then run()
	static void * launch(void * msg);
	//! indicates kill() has been called (or SIGUSR1 was sent from some other source) while launch() was still running
	static void handle_launch_signal(int sig);
	//! indicates kill() has been called (or SIGUSR1 was sent from some other source)
	static void handle_signal(int sig);
	//! indicates the thread is exiting, either voluntary (run() returned), stop(), or kill() -- calls cancelled() for the thread as indicated by @a th
	static void handle_exit(void * th);

	//! called by handleInterrupt() in target thread following call to interrupt(), assuming thread has not been cancelled (which can intercept the interrupt)
	virtual void interrupted() {}
	
	//! called by SIGALRM signal handler installed by interrupt() just before it posts the corresponding SIGALRM
	/*! tests for thread cancel condition before calling on to interrupted() */
	static void handleInterrupt(int signal);
	
	//! emit a warning that the last thread exited while the self-pointer thread-specific key still exists (need to call releaseMainThread() or handle_exit())
	static void warnSelfUndestructed(void* msg);

	//! stores the actual pthread data fields
	struct Threadstorage_t * pt;
	//! set to true once start() has been called, set back to false by handle_exit(), or by murder() itself
	bool started;
	//! set to true once launch() has been called, set back to false by handle_exit(), or by murder() itself
	bool running;
	//! set to true once handle_exit() or murder() has been called, set back to false by start() (and initially by constructor)
	bool exited;
	//! indicates the value to be returned by the thread entry point (and thus passed back to join()) -- set this in runloop() or launched(), overridden by run()'s return value
	void * returnValue;
	//! depth of the pushNoCancel() stack
	unsigned int noCancelDepth;
#ifndef USE_SIGNAL_TO_CANCEL_THREAD
	//! depth of the requestInterruptOnCancel() stack
	unsigned int reqIntrDepth;
#endif
	//! cancel status at root of no-cancel stack (may be no-cancel through and through)
	int cancelOrig;
	//! set to true if using signal-based thread cancellation instead of pthread_cancel
	bool cancelRequested;
	//! set to true if the cancellation has been triggered and stack unwind is in progress (don't throw anything!)
	bool cancelInProgress;

	//! indicates a common group of threads, inherited from the thread which created this one, default NULL if created from main thread
	void* group;
	
	//! stores a stack trace of the call to start(), for error reporting and debugging
	stacktrace::StackFrame * startTrace;
	//! prevents concurrent starts
	mutable Lock startLock;
	//! prevents concurrent stops
	mutable Lock stopLock;
	
private:
	//! this is thrown by testCancel() on systems where pthread_testcancel() doesn't do what we want
	/*! This isn't thrown on other platforms, use a catch(...) to get this for portability */
	struct cancellation_exception {};
	
	Thread(const Thread& r); //!< don't call, not a well defined operation
	Thread& operator=(const Thread& r); //!< don't call, not a well defined operation
};

#endif //Aperios check

#endif

/*! @file
* @brief Describes the Thread class and its AutoThread templated subclass
* @author ejt (Creator)
*/

