#ifndef PLATFORM_APERIOS
#include "Thread.h"
#include "Shared/ReferenceCounter.h"
#include "ProcessID.h"
#include "Shared/StackTrace.h"
#include "Shared/MarkScope.h"

#include <pthread.h>
#include <string.h>
#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <cassert>
#include <cstdio>
#include <errno.h>
#include <stdexcept>

using namespace std;

#define THREADCANCEL_SANITY_CHECKS

/*! @cond INTERNAL */
//! provides the system-dependent implementation of a thread
struct Threadstorage_t {
	//! constructor
	Threadstorage_t() : threadInfo(), hasThread(false) {}
	
	//! the main POSIX reference to the thread
	pthread_t threadInfo;
	//! Set to true when threadInfo is given to pthread_create (can't rely on 0/NULL being an invalid thread), so it can pthread_detach before overwrite
	/*! We'll assume joinability, leave the thread joinable (i.e. not detached) until the main Thread class either starts a new thread, is deleted, or is joined. */
	bool hasThread;
	
	//! storage which will be set up as a thread-specific memory value, so threads can tell themselves apart
	static pthread_key_t selfKey;
private:
	Threadstorage_t(const Threadstorage_t& r); //!< don't call
	Threadstorage_t& operator=(const Threadstorage_t& r); //!< don't call
};
static const pthread_key_t INVALID_THREADKEY=(pthread_key_t)-1;
pthread_key_t Threadstorage_t::selfKey=INVALID_THREADKEY;
/*! @endcond */

void* Thread::CANCELLED = PTHREAD_CANCELED;

Thread::Thread()
	: pt(new Threadstorage_t), started(false), running(false), exited(false), returnValue(NULL),
	noCancelDepth(0), 
#ifndef USE_SIGNAL_TO_CANCEL_THREAD
	reqIntrDepth(0),
#endif
	cancelOrig(PTHREAD_CANCEL_ENABLE), cancelRequested(false), cancelInProgress(false),
	group(NULL), startTrace(NULL), startLock(), stopLock()
{
	Thread* cur=getCurrent();
	if(cur!=NULL)
		group=cur->getGroup();
}

Thread::~Thread() {
	startLock.lock();
	if(started && this!=getCurrent()) {
		stop();
		join();
	}
	/*if(pt==NULL) {
		std::cerr << "Thread storage already deleted!?!?!" << std::endl;
		*(int*)NULL=0xDEADDEAD;
	}*/
	assert(pt!=NULL);
	if(pt->hasThread) {
		if(int err=pthread_detach(pt->threadInfo)) {
			cerr << "~Thread(), thread_detach: " << strerror(err) << endl;
			stacktrace::displayStackTrace(startTrace);
		}
		pt->hasThread=false;
	}
	delete pt;
	pt=NULL;
	if(startTrace!=NULL)
		stacktrace::freeStackTrace(startTrace);
}

void Thread::start() {
	MarkScope sl(startLock);
	if(started) {
		std::cerr << "Thread::start() -- thread is already started!\n"
		"   Make another instance if you want to run another copy of this thread\n"
		"   ** Original start:" << std::endl;
		stacktrace::displayStackTrace(startTrace);
		std::cerr << "   ** Duplicate start:" << std::endl;
		stacktrace::displayCurrentStackTrace();
		return;
	}
	if(startTrace!=NULL)
		stacktrace::freeStackTrace(startTrace);
	startTrace = stacktrace::recordStackTrace();
	exited=cancelRequested=false;
	started=true;
	pthread_attr_t   threadAttributes;
	if(int err = pthread_attr_init(&threadAttributes))
		cerr << "Thread start(), could not init stack attributes: " << strerror(err) << endl;
	const size_t REQ_STACK = 2*1024*1024; // OS X default is 512KB, let's up that to 2MB (portably)
	size_t stackSize=0;
	if(int err = pthread_attr_getstacksize(&threadAttributes, &stackSize)) 
		cerr << "Thread start(), get stack size: " << strerror(err) << endl;
	if(stackSize < REQ_STACK) {
		if(int err = pthread_attr_setstacksize(&threadAttributes, REQ_STACK)) 
			cerr << "Thread start(), set stack size: " << strerror(err) << endl;
	}
	if(pt->hasThread) {
		if(int err=pthread_detach(pt->threadInfo))
			cerr << "Thread start(), thread_detach of previous thread: " << strerror(err) << endl;
		pt->hasThread=false;
	}
	if(int err=pthread_create(&pt->threadInfo, &threadAttributes, launch, this))
		cerr << "Thread start(), pthread_create: " << strerror(err) << endl;
	else
		pt->hasThread=true;
}

void * Thread::run() {
	for(;;) {
		unsigned int sleeptime=runloop();
		if(sleeptime==-1U)
			return returnValue;
		if(sleeptime>0)
			usleep(sleeptime);
		testCancel();
	}
	// this return is just to satisfy warnings with silly compiler
	return returnValue; //never happens -- cancel or max sleep time would exit
}

Thread& Thread::interrupt() {
	if(!isRunning()) //can't interrupt before thread has been launched!
		return *this;
	struct sigaction sa;
	sa.sa_handler = handleInterrupt;
	if(sigemptyset(&sa.sa_mask)!=0)
		perror("Thread::interrupt(): clearing signal set via sigemptyset()");
	sa.sa_flags = 0;
	if(sigaction(SIGALRM,&sa,NULL)!=0)
		perror("Thread::interrupt(): installing interrupt handler via sigaction()");
	sendSignal(SIGALRM);
	return *this;
}

Thread& Thread::stop() {
	MarkScope l(stopLock);
	if(exited)
		return *this;
	if(!started && !running) {
		std::cerr << "Thread::stop() -- thread has not been started!" << std::endl;
		dereference();  // added by DST on 3/2014 to stop infinite loop
		// stacktrace::displayCurrentStackTrace();
		return *this;
	}
#ifdef USE_PTHREAD_CANCEL
	if(started && !running)
		usleep(50000);
	if(started && !running)
		std::cerr << "Thread::stop(): Waiting for thread launch to complete (stillborn thread?)" << std::endl;
	while(started && !running)
		usleep(100000);
	if(!running)
		return *this;
	if(int err=pthread_cancel(pt->threadInfo))
		cerr << "Thread cancel(), pthread_cancel("<<pt->threadInfo<<"): " << strerror(err) << endl;
#else
	// not using pthread_cancel, don't have to wait for launch to complete -- just set flag now
	cancelRequested=true;
#endif
#ifdef USE_SIGNAL_TO_CANCEL_THREAD
#  ifndef USE_PTHREAD_CANCEL
	// not using pthread_cancel, if launch hasn't completed, don't wait around just to signal it.  Launch will test cancellation after calling launched().
	if(!running)
		return *this;
#  endif
	if(noCancelDepth>0)
		return *this;
	interrupt(); // break thread out of any long sleep commands
#else
	if(reqIntrDepth>0)
		interrupt();
#endif
	return *this;
}

Thread& Thread::kill() {
	sendSignal(SIGUSR1);
	return *this;
}

Thread& Thread::murder() {
	if(pt->hasThread) {
		if(int err=pthread_detach(pt->threadInfo))
			cerr << "Thread kill(), thread_detach: " << strerror(err) << endl;
		pt->hasThread=false;
	}
	sendSignal(SIGSTOP);
	started=running=false;
	exited=true;
	return *this;
}

void Thread::sendSignal(int sig) {
	if(started && !running)
		usleep(50000);
	if(started && !running)
		std::cerr << "Thread::stop(): Waiting for thread launch to complete (stillborn thread?)" << std::endl;
	while(started && !running)
		usleep(100000);
	if(!isRunning())
		return;
	if(int err=pthread_kill(pt->threadInfo,sig))
		if(err!=ESRCH) // thread exit during send?
			cerr << "Thread sendSignal(), pthread_kill("<<sig<<"): " << strerror(err) << endl;
}

void * Thread::join() const {
	MarkScope l(startLock);
	if(!started || !pt->hasThread) // already gone?
		return cancelRequested ? CANCELLED : returnValue;
	void * ans=NULL;
	pthread_t cur = pt->threadInfo;
	pt->hasThread = false; // one way or another, don't try to detach or join again...
	if(int err=pthread_join(cur, &ans)) {
		cerr << "thread join() returned " << err << " " << strerror(err) << endl;
		if((err==EINVAL || err==ESRCH) && (!started || !pthread_equal(cur, pt->threadInfo))) // already gone?
			return cancelRequested ? CANCELLED : returnValue;
		cerr << "Thread join() " << err << " (" << EINVAL << ',' << ESRCH << "), pthread_join: " << strerror(err) << endl;
		stacktrace::displayCurrentStackTrace();
	}
	return ans;
}

Thread* Thread::getCurrent() {
	if(Threadstorage_t::selfKey==INVALID_THREADKEY) {
		static bool gaveError=false;
		if(!gaveError) {
			cerr << "ERROR: In Thread::getCurrent(), selfKey uninitialized; Thread::initMainThread was not called." << endl;
			cerr << "       (This error will only be displayed once)" << endl;
			gaveError=true;
		}
		return NULL;
	}
	return static_cast< Thread* >(pthread_getspecific(Threadstorage_t::selfKey));
}

void Thread::initMainThread() {
	if(int err=pthread_key_create(&Threadstorage_t::selfKey,warnSelfUndestructed))
		cerr << "WARNING: In Thread::initMainThread(), pthread_key_create(selfKey) returned " << strerror(err) << endl;
	if(int err=pthread_setspecific(Threadstorage_t::selfKey,NULL))
		cerr << "WARNING: In Thread::initMainThread(), pthread_setspecific(selfKey) returned " << strerror(err) << endl;
}

void Thread::releaseMainThread() {
	//handle_exit(NULL);
	if(Threadstorage_t::selfKey==INVALID_THREADKEY)
		return;
	if(int err=pthread_key_delete(Threadstorage_t::selfKey))
		cerr << "WARNING: In Thread::releaseMainThread, pthread_key_delete(selfKey) returned " << strerror(err) << endl;
}

void Thread::testCurrentCancel() {
	Thread * cur = getCurrent();
	if(cur==NULL) { // already gave warning in getCurrent
#ifdef USE_PTHREAD_CANCEL
		pthread_testcancel();
#endif
		return;
	}
#ifdef DEBUG
	if(cur->noCancelDepth!=0) {
		cerr << "WARNING: Thread::testCancel called with noCancelDepth=="<<cur->noCancelDepth<<" (process="<<ProcessID::getID()<<", thread="<<pthread_self()<<")"<<endl;
		cerr << "The thread was started at:" << endl;
		stacktrace::displayStackTrace(cur->startTrace);
		cerr << "The testCancel call is from:" << endl;
		stacktrace::displayCurrentStackTrace();
	}
#endif
	if(cur->noCancelDepth!=0 || cur->cancelInProgress)
		return;
#ifdef USE_PTHREAD_CANCEL
	cur->cancelInProgress=true; // if next line engages cancellation, don't throw again during this throw!
	pthread_testcancel();
	cur->cancelInProgress=false; // phew, guess we're still running, reset flag
#else
	if(cur->cancelRequested) {
		cur->cancelInProgress=true; // don't throw again during this throw!
		throw cancellation_exception();
	}
#endif
}

void * Thread::launch(void * msg) {
	//cout << "Spawn thread " << pthread_self() << " from process " << ProcessID::getID() << endl;
	Thread* cur=static_cast<Thread*>(msg);
	if(cur==NULL) {
		cerr << "ERROR: Thread::launch with NULL msg" << endl;
		return NULL;
	}

	if(int err=pthread_setspecific(Threadstorage_t::selfKey,msg))
		cerr << "WARNING: In Thread::launch(), pthread_setspecific(selfKey) returned " << strerror(err) << endl;
	
	cur->cancelInProgress=false;
	
	//disable cancel while calling launch()
	if(int err=pthread_setcancelstate(PTHREAD_CANCEL_DISABLE,NULL))
		cerr << "Thread launch(), pthread_setcanceltype: " << strerror(err) << endl;
	++(cur->noCancelDepth);
	if(signal(SIGUSR1,Thread::handle_launch_signal)==SIG_ERR)
		perror("Thread launch(), signal(SIGUSR1,handle_launch_signal)");
	cur->running=true;
	if(!cur->launched()) {
		//subclass's launch cancelled launch
		--(cur->noCancelDepth);
		handle_exit(NULL);
		return cur->returnValue;
	}
	--(cur->noCancelDepth);
	
	// handle_exit calls dereference(), which may delete this; store the return value since 'cur' can't be trusted
	// also, if this is not overwritten, indicates cancellation occurred.
	cur->returnValue=CANCELLED; 
	
	//These pthread functions actually define a scope between them (ugh)
	//I've added braces of my own to make this explicitly clear
	pthread_cleanup_push(Thread::handle_exit,msg); {
		
		if(signal(SIGUSR1,Thread::handle_signal)==SIG_ERR)
			perror("Thread launch(), signal(SIGUSR1,handle_signal)");
		
		try {
			if(cur->noCancelDepth==0) {
				//reset cancelability before run
				if(int err=pthread_setcancelstate(cur->cancelOrig,NULL))
					cerr << "Thread launch(), pthread_setcancelstate: " << strerror(err) << endl;
				if(int err=pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED,NULL))
					cerr << "Thread launch(), pthread_setcanceltype: " << strerror(err) << endl;
				cur->testCancel();
			}
			cur->returnValue=cur->run();
		} catch(const Thread::cancellation_exception&) {
			// if on a system not using pthread_cancel, we throw this exception from testCancel to trigger stack unwinding so we can exit...
			// note returnedValue remains 'CANCELLED' since we excepted instead of returning
		}
		/* catch(const std::exception& e) {
			std::cout << "WTF is this exception? " << e.what() << std::endl;
			cancelDetected=true;
		} catch(...) {
			std::cout << "WTF is this unknown exception?" << std::endl;
		}*/
		
    usleep(50); // **** HACK (dst) 7/28/2016: usleep here seems to prevent crashes in the pthread mechanism
	} pthread_cleanup_pop(true);
	return cur->returnValue;
}

void Thread::handle_launch_signal(int /*sig*/) {
	handle_exit(NULL);
	pthread_exit(NULL);
}

void Thread::handle_signal(int /*sig*/) {
	pthread_exit(NULL);
}

void Thread::handle_exit(void * th) {
	//cout << "Cull thread " << pthread_self() << endl;
	Thread* cur=getCurrent();
	if(cur==NULL) {
		cerr << "ERROR: handle_exit called for a NULL thread" << endl;
		if(th!=NULL) {
			static_cast<Thread*>(th)->cancelled();
			static_cast<Thread*>(th)->started=static_cast<Thread*>(th)->running=false;
		}
		return;
	}
	
	{ //scope limiting for stop lock, release lock before dereference in case it deletes this
		MarkScope l(cur->stopLock);
		if(th!=NULL && th!=cur)
			cerr << "WARNING: handle_exit argument does not match selfKey" << endl;
		if(cur->noCancelDepth!=0) {
			cerr << "WARNING: thread " << pthread_self() << " of ProcessID_t " << ProcessID::getID() << " exited while noCancelDepth>0 (was " << cur->noCancelDepth << ")" << endl;
			cerr << "         This may indicate a mutex was left locked." << endl;
			cur->noCancelDepth=0; // reset in case run again
		}
		if(int err=pthread_setspecific(Threadstorage_t::selfKey,NULL))
			cerr << "WARNING: In Thread::handle_exit(), pthread_setspecific(selfKey) returned " << err << endl;
		cur->cancelled();
		cur->started=cur->running=false;
		cur->exited=true;
	}
	cur->dereference();
}

void Thread::pushNoCancel() {
	Thread * cur=getCurrent();
	if(cur==NULL) {
		//cerr << "ERROR: Thread::pushNoCancel was given NULL thread by getCurrent, thread=" << pthread_self() << endl;
		//not so bad, indicates already canceled -- don't test cancel again, don't want to cancel-recurse
		if(int err=pthread_setcancelstate(PTHREAD_CANCEL_DISABLE,NULL))
			cerr << "ERROR: Thread pushNoCancel(), pthread_setcanceltype: " << strerror(err) << endl;
	} else {
		++(cur->noCancelDepth);
		int previous=-1;
		if(int err=pthread_setcancelstate(PTHREAD_CANCEL_DISABLE,&previous))
			cerr << "ERROR: Thread pushNoCancel(), pthread_setcanceltype: " << strerror(err) << endl;
#ifdef THREADCANCEL_SANITY_CHECKS
		if(cur->noCancelDepth==1 && previous!=cur->cancelOrig)
			cerr << "WARNING: In Thread::pushNoCancel, cancel state was wrong (was " << previous << ", expected " << cur->cancelOrig << ")" << endl;
		else if(cur->noCancelDepth!=1 && previous!=PTHREAD_CANCEL_DISABLE)
			cerr << "WARNING: In Thread::pushNoCancel, cancel state was somehow re-enabled" << endl;
#endif
	}
}
void Thread::popNoCancel(bool doTestCancel/*=true*/) {
	Thread * cur=getCurrent();
	if(cur==NULL) {
		//cerr << "ERROR: Thread::popNoCancel was given NULL thread by getCurrent, thread=" << pthread_self() << endl;
		//not so bad, indicates already canceled -- don't test cancel again, don't want to cancel-recurse
		return; //no point in continuing
	} else if(cur->noCancelDepth==0) {
		cerr << "ERROR: Thread::popNoCancel underflow" << endl;
	} else
		--(cur->noCancelDepth);
	int previous=-1;
	if(cur->noCancelDepth==0) {
		if(int err=pthread_setcancelstate(cur->cancelOrig,&previous))
			cerr << "ERROR: Thread popNoCancel(), pthread_setcanceltype: " << strerror(err) << endl;
		if(cur->cancelOrig==PTHREAD_CANCEL_ENABLE && doTestCancel)
			cur->testCancel(); // I thought setcancelstate(ENABLE) implied this, but apparently not
	}
#ifdef THREADCANCEL_SANITY_CHECKS
	else { //still disabled, double check it
		if(int err=pthread_setcancelstate(PTHREAD_CANCEL_DISABLE,&previous))
			cerr << "ERROR: Thread popNoCancel(), pthread_setcanceltype: " << strerror(err) << endl;
	}
	if(previous!=PTHREAD_CANCEL_DISABLE)
		cerr << "WARNING: In Thread::popNoCancel, cancel state was somehow re-enabled" << endl;
#endif
}

void Thread::requestInterruptOnCancel() {
#ifndef USE_SIGNAL_TO_CANCEL_THREAD
	Thread * cur=getCurrent();
	if(cur==NULL)
		return; //no point in continuing
	pushNoCancel();
	++(cur->reqIntrDepth);
#ifdef DEBUG
	if(cur->reqIntrDepth!=1)
		std::cerr << "WARNING: recursive Thread::requestInterruptOnCancel() " << cur->reqIntrDepth << std::endl;
#endif
#endif
}
	
void Thread::unrequestInterruptOnCancel() {
#if !defined(USE_SIGNAL_TO_CANCEL_THREAD) || !defined(USE_TESTCANCEL_IN_INTERRUPT)
	Thread * cur=getCurrent();
	if(cur==NULL)
		return; //no point in continuing
#endif
#ifndef USE_SIGNAL_TO_CANCEL_THREAD
	if(cur->reqIntrDepth==0) {
		cerr << "ERROR: Thread::unrequestInterruptOnCancel underflow" << endl;
	} else {
		--(cur->reqIntrDepth);
	}
	popNoCancel();
#endif
#ifndef USE_TESTCANCEL_IN_INTERRUPT
	if(cur->noCancelDepth==0)
		cur->testCancel();
#endif
}

void Thread::handleInterrupt(int /*signal*/) {
	//if(signal(SIGALRM,SIG_DFL)==SIG_ERR)
	//	perror("Thread::handleInterrupt(): could not re-enable signal");
	Thread * cur=Thread::getCurrent();
	if(cur==NULL) {
		// implies signal handler was delivered when in the process of shutting down
		// just give up on it silently
		//std::cerr << "Thread::handleInterrupt called from non-Thread" << endl;
		return;
	}
#ifdef USE_TESTCANCEL_IN_INTERRUPT
	if(cur->noCancelDepth==0)
		cur->testCancel();
#endif
	cur->interrupted();
}

void Thread::warnSelfUndestructed(void* msg) {
	cerr << "ERROR: Thread local data (selfKey) not deleted by Thread::handle_exit" << endl;
	Thread* cur = getCurrent();
	if(cur!=NULL)
		cerr << "       Weird, key wasn't cleared... (" << cur << ") " << cur->noCancelDepth << " locks on stack? " << endl;;
	if(msg==NULL) {
		cerr << "       Message is null, warnCancelDepthUndestructed shouldn't have been called." << endl;
	} else {
		if(cur!=NULL && cur!=msg)
			cerr << "       and current thread does not match msg (" << cur << " vs " << msg << ")" << endl;
		cur = static_cast<Thread*>(msg);
	}
	if(cur!=NULL) {
		//try to recover
		if(cur->noCancelDepth==0) {
			cerr << "       But at least the depth is 0" << endl;
		} else {
			cerr << "       The depth indicates there may be " << cur->noCancelDepth << " locks left in place" << endl;
		}
		cur->cancelled();
		cur->started=cur->running=false;
		pthread_setspecific(Threadstorage_t::selfKey,NULL);
	}
}


/*! @cond INTERNAL */
//! This handles the actual lock implementation, which allows Lock to provide an abstract interface
class Thread::Lock::LockStorage : public ReferenceCounter {
	friend class Thread::Condition;
public:
	//! constructor
	LockStorage() : ReferenceCounter(), locklevel(0), mutex(), attr(), threadkey() {
		addReference();
		pthread_mutexattr_init(&attr);
		pthread_mutexattr_settype(&attr,PTHREAD_MUTEX_RECURSIVE);
		pthread_mutex_init(&mutex,&attr);
	}
	//! destructor, releases any pending locks (with warning
	~LockStorage() {
		pthread_mutexattr_destroy(&attr);
		pthread_mutex_destroy(&mutex);
		if(locklevel>1) //having one left is ok, perhaps even good (keeping the lock as it is destroyed)
			cerr << "WARNING: lockstorage destructed with " << locklevel << " locks still in effect" << endl;
		while(locklevel>0) {
			locklevel--;
			Thread::popNoCancel(false); // no testCancel because we could be in unwind from previous exception, cancellation here is non-portable (causes terminate on OS X)
		}
	}
	//! copy constructor (functional!) -- both locks will wind up referencing the same system resource, so this is more of an alias than a clone
	LockStorage(const LockStorage& ls) : ReferenceCounter(ls), locklevel(ls.locklevel), mutex(ls.mutex), attr(ls.attr), threadkey(ls.threadkey) {}
	//! assignment (functional!) -- both locks will wind up referencing the same system resource, so this is more of an alias than a clone
	LockStorage& operator=(const LockStorage& ls) { ReferenceCounter::operator=(ls); locklevel=ls.locklevel; mutex=ls.mutex; attr=ls.attr; threadkey=ls.threadkey; return *this; }
	
	//! trigger and wait for a mutual exclusion lock, recursively
	void lock() {
		if(int err=pthread_mutex_lock(&mutex)) {
			cerr << "ERROR: Thread::Lock::lock() failed: " << strerror(err) << endl;
		} else {
			locklevel++;
		}
	}
	//! attempt to get a lock, but return false if it is not immediately available
	bool trylock() {
		if(!pthread_mutex_trylock(&mutex)) {
			locklevel++;
			return true;
		} else {
			return false;
		}
	}
	//! release a lock (recursively, won't actually release the lock resource until all calls to lock() have been balanced)
	void unlock() {
		if(locklevel==0) {
			cerr << "ERROR: Thread::Lock::unlock() underflow" << endl;
			stacktrace::displayCurrentStackTrace();
		}
		locklevel--;
		if(int err=pthread_mutex_unlock(&mutex))
			cerr << "ERROR: Thread::Lock::unlock() failed: " << strerror(err) << endl;
	}
	//! returns the depth of the lock recursion (#locklevel)
	unsigned int getLockLevel() { return locklevel; }
	
protected:
	unsigned int locklevel; //!< depth of lock recursion (i.e. number of calls to lock() minus calls to unlock())
	pthread_mutex_t mutex; //!< system lock resource
	pthread_mutexattr_t attr; //!< system lock resource attributes (used to specify #mutex is recursive in the system as well)
	pthread_key_t threadkey; //!< not making use of the thread specific nature of these, but we are making use of the call to a destructor (emergencyUnlock) on cancel
};

Thread::Lock::LockStorage* Thread::Lock::glock=NULL;
/*! @endcond */

Thread::Lock::Lock() : mylock(new LockStorage), locklevel(0) {
	if(glock==NULL)
		setup();
}
/*Thread::Lock::Lock(const Lock& l)
	: mylock(l.mylock), locklevel(0)
{
	glock->lock();
	mylock->addReference();
	glock->unlock();
	lock();
}
Thread::Lock::Lock(const Lock& l, bool autolock)
	: mylock(l.mylock), locklevel(0)
{
	glock->lock();
	mylock->addReference();
	glock->unlock();
	if(autolock)
		lock();
}
Thread::Lock& Thread::Lock::operator=(const Lock& l) {
	glock->lock();
	lock();
	if(locklevel>2)
		cerr << "WARNING: Thread::Lock overwritten with "<<locklevel<<" locks still in effect" << endl;
	if(!mylock->removeReference())
		while(locklevel>0)
			unlock();
	mylock=l.mylock;
	locklevel=0;
	glock->unlock();
	return *this;
}*/
Thread::Lock::~Lock() {
	glock->lock();
	if(locklevel>1)
		cerr << "WARNING: Thread::Lock destructed with "<<locklevel<<" locks still in effect" << endl;
	if(!mylock->removeReference()) {
		std::cerr << "Lock was deleted with external reference?" << std::endl;
		stacktrace::displayCurrentStackTrace();
		while(locklevel>0)
			unlock();
	}
	mylock=NULL;
	glock->unlock();
}

void Thread::Lock::useResource(Resource::Data&) {
	mylock->lock();
	locklevel++;
}
bool Thread::Lock::trylock() {
	Thread::pushNoCancel();
	if(mylock->trylock()) {
		locklevel++;
		return true;
	} else {
		Thread::popNoCancel();
		return false;
	}
}
void Thread::Lock::releaseResource(Resource::Data&) {
	locklevel--;
	mylock->unlock();
}
unsigned int Thread::Lock::getLockLevel() const {
	return mylock->getLockLevel();
}
void Thread::Lock::setup() {
	if(glock==NULL)
		glock=new LockStorage;
}

/*! @cond INTERNAL */
//! Implement system-dependent portion of a thread condition, a signaling mechanism.
/*! This is a very basic wrapper -- just adds a constructor and destructor to the POSIX pthread_cond_t. */
class Thread::Condition::ConditionStorage {
public:
	//! constructor
	ConditionStorage() : cond() {
		if(int err=pthread_cond_init(&cond,NULL)) {
			cerr << "ERROR: Thread::Condition::ConditionStorage() failed: " << strerror(err) << endl;
		}
	}
	//! destructor
	~ConditionStorage() {
		if(int err=pthread_cond_destroy(&cond)) {
			cerr << "ERROR: Thread::Condition::~ConditionStorage() failed: " << strerror(err) << endl;
		}
	}
	//! system resource storage
	pthread_cond_t cond;
};
/*! @endcond */

Thread::Condition::Condition() : mycond(new ConditionStorage) {}
Thread::Condition::~Condition() { delete mycond; mycond=NULL; }

void Thread::Condition::broadcast() const {
	if(int err=pthread_cond_broadcast(&mycond->cond)) {
		cerr << "ERROR: Thread::Condition::broadcast() failed: " << strerror(err) << endl;
	}
}
void Thread::Condition::signal() const {
	if(int err=pthread_cond_signal(&mycond->cond)) {
		cerr << "ERROR: Thread::Condition::signal() failed: " << strerror(err) << endl;
	}
}
bool Thread::Condition::timedwait(Lock& l, const timespec* abstime, bool noWarn/*=false*/) const {
#ifdef USE_SIGNAL_TO_CANCEL_THREAD
	Thread::testCurrentCancel();
#endif
	unsigned int locklevel=l.mylock->locklevel;
	if(locklevel==1) {
		// no-op for common case to shortcut other tests
	} else if(locklevel>1) {
		if(!noWarn)
			displayRecursiveLockWarning("timedwait",locklevel);
		while(l.mylock->locklevel>1)
			l.mylock->unlock();
	} else { // 0
		throw std::logic_error("Thread::Condition::timedwait() called without holding lock");
	}
	if(int err=pthread_cond_timedwait(&mycond->cond,&l.mylock->mutex,abstime)) {
		if(err!=ETIMEDOUT)
			cerr << "ERROR: Thread::Condition::timedwait() failed: " << strerror(err) << endl;
		while(l.mylock->locklevel<locklevel)
			l.mylock->lock();
		return false;
	}
	while(l.getLockLevel()<locklevel)
		l.mylock->lock();
#ifdef USE_SIGNAL_TO_CANCEL_THREAD
	Thread::testCurrentCancel();
#endif
	return true;
}
void Thread::Condition::wait(Lock& l, bool noWarn/*=false*/) const {
#ifdef USE_SIGNAL_TO_CANCEL_THREAD
	Thread::testCurrentCancel();
#endif
	unsigned int locklevel=l.mylock->locklevel;
	if(locklevel==1) {
		// no-op for common case to shortcut other tests
	} else if(locklevel>1) {
		if(!noWarn)
			displayRecursiveLockWarning("wait",locklevel);
		while(l.mylock->locklevel>1)
			l.mylock->unlock();
	} else { // 0
		throw std::logic_error("Thread::Condition::wait() called without holding lock");
	}
	if(int err=pthread_cond_wait(&mycond->cond,&l.mylock->mutex)) {
		cerr << "ERROR: Thread::Condition::wait() failed: " << strerror(err) << endl;
	}
	while(l.getLockLevel()<locklevel)
		l.mylock->lock();
#ifdef USE_SIGNAL_TO_CANCEL_THREAD
	Thread::testCurrentCancel();
#endif
}

void Thread::Condition::displayRecursiveLockWarning(const char * fn, unsigned int locklevel) {
	std::cerr << "WARNING: Thread::Condition::"<<fn<<"() called holding a recursive lock. (depth " << locklevel << ")\n"
	"  You should verify outer lock scopes are safe to temporarily free during wait,\n"
	"  then pass 'true' for the noWarn argument to timedwait() to disable this message." << std::endl;
	stacktrace::displayCurrentStackTrace();
}


#endif // PLATFORM check

/*! @file
* @brief Describes the Thread class and its AutoThread templated subclass
* @author ejt (Creator)
*/
