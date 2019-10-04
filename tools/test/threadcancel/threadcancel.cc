#define TK_ENABLE_THREADING
#include "local/minisim.h"

#include "IPC/CallbackThread.h"
#include "IPC/FailsafeThread.h"
#include "Shared/MarkScope.h"
#include <iostream>
#include <unistd.h>
#include <list>
#include <algorithm>
#include <set>

using namespace std;
using namespace minisim;

class DestructorTest {
public:
	DestructorTest() : done(false) {}
	~DestructorTest() { cout << (done ? "Foo cancel failed" : "Foo was cancelled") << endl; }
	bool done;
};

Thread::Lock l;
const unsigned int SLEEPTIME=3;

class TestThread1 : public Thread {
protected:
	void * run() {
		MarkScope autolock(l);
		cout << "test1 started" << endl;
		DestructorTest dt;
		if(sleep(SLEEPTIME)!=0)
			testCancel();
		dt.done=true;
		return NULL;
	}
};

class TestThread2 : public Thread {
protected:
	void * run() {
		l.lock();
		cout << "test2 started" << endl;
		DestructorTest dt;
		if(sleep(SLEEPTIME)!=0)
			testCancel();
		cout << "should still get here" << endl;
		dt.done=true;
		l.unlock();
		testCancel();
		cout << "shouldn't get here!" << endl;
		return NULL;
	}
};

class TestThread3 : public Thread {
protected:
	void * run() {
		MarkScope autolock(l);
		cout << "test3 started" << endl;
		DestructorTest dt;
		char buf[10];
		cin.exceptions(std::ios_base::badbit);
		try {
			cin.read(buf,10);
			if(!cin || cin.gcount()!=10)
				testCancel();
		} catch(...) {
			cout << "Exception thrown!" << endl;
			throw;
		}
		dt.done=true;
		return NULL;
	}
};

#include "Wireless/netstream.h"
class ServerThread : public Thread {
protected:
	void * run() {
		ionetstream server;
		server.listen(1234U);
		testCancel();
		cout << "got client" << endl;
		std::string x;
		server >> x;
		testCancel();
		if(server)
			cout << x << endl;
		sleep(SLEEPTIME);
		testCancel();
		cout << "server wasn't cancelled!" << endl;
		return NULL;
	}
};

class TestThread4 : public Thread {
protected:
	void * run() {
		MarkScope autolock(l);
		cout << "test4 started" << endl;
		DestructorTest dt;
		char buf[10];
		ionetstream client;
		client.exceptions(std::ios_base::badbit);
		while(!client.is_open()) {
			client.open("localhost",1234U);
			testCancel();
			usleep(5000);
		}
		//client << "Message sent" << endl;
		try {
			client.read(buf,10);
			if(!client || client.gcount()!=10)
				testCancel();
		} catch(...) {
			cout << "Exception thrown!" << endl;
			throw;
		}
		dt.done=true;
		return NULL;
	}
};

class MultiThreads {
protected:
	// each of these functions will run in a separate thread
	void fn1() { while(true) increment(count1); }
	void fn2() { while(true) increment(count2); }
	void fn3() { while(true) increment(count3); }

	void increment(unsigned int& c) {
		{
			MarkScope hold_lock_for_scope(lock);
			++lockedCount; // only one thread can add at a time, should be safe
		} // lock is released here (or if exception occurs, or if thread is cancelled...)
		++c;
		++sharedCount; // could have race condition
		Thread::testCurrentCancel();
	}

	unsigned int count1,count2,count3; // counters for the individual threads
	unsigned int sharedCount; // incremented outside of a lock, may be inaccurate
	unsigned int lockedCount; // incremented after mutual exclusion lock is obtained, should be safe
	Thread::Lock lock; // represents a mutual exclusion lock, best used with MarkScope
	CallbackThread thread1,thread2,thread3;
	
public:
	MultiThreads()
		: count1(0), count2(0), count3(0), sharedCount(0), lockedCount(0), lock(),
		thread1(std::mem_fun(&MultiThreads::fn1),this),
		thread2(std::mem_fun(&MultiThreads::fn2),this),
		thread3(std::mem_fun(&MultiThreads::fn3),this)
	{
		thread1.start();
		thread2.start();
		thread3.start();
	}
	~MultiThreads() {
		thread1.stop().join();
		thread2.stop().join();
		thread3.stop().join();
		unsigned int sumCount = (count1+count2+count3);
		std::cout << "@VAR " << count1 << " + " << count2 << " + " << count3 << " = " << sumCount << std::endl;
		std::cout << "@VAR " << "sharedCount is " << sharedCount << ", lockedCount is " << lockedCount << std::endl;
		if(lockedCount != sumCount) {
			std::cout << "ERROR locked count does not match sum of individual counts!  Bad lock?" << std::endl;
		}
	}
};

class HelloThread : public Thread {
protected:
	virtual void * run() {
		std::cout << "Hello World!" << std::endl;
		return NULL;
	}
};

#include "IPC/SemaphoreManager.h"
SemaphoreManager semgr;
class SemaphoreThread : public Thread {
public:
	SemaphoreThread() : Thread(), semid(semgr.getSemaphore()) {
		semgr.setValue(semid,1);
	}
	SemaphoreManager::semid_t semid;
protected:
	void * run() {
		std::cout << "Blocking on semaphore" << std::endl;
		semgr.testZero(semid,true);
		// This can be bad on linux... cancellation is detected in the flush, leaves stream in bad state
		// thus next output either throws ios_base::failure (if exceptions(ios_base::badbit) is set)
		// or all output is dropped until ostream::clear() is called...
		// UPDATE: Now fixed by having SemaphoreManager's semop calls request an interrupt and check cancel afterward
		std::cout << "Checking cancel?" << std::endl;
		testCancel();
		std::cout << "Semaphore zeroed" << std::endl;
		return NULL;
	}
	void interrupted() {
		std::cout << "SemaphoreThread interrupted!" << std::endl;
		Thread::interrupted();
	}
};

int test1(void* th) { std::cout << "Callback test 1 @VAR(" << th << ")" << std::endl; return 0; }
int test2() { std::cout << "Callback test 2" << std::endl; return 0; }

#include "IPC/MutexLock.h"
class MutexStressTest : public Thread {
public:
	MutexStressTest(MutexLock<ProcessID::NumProcesses>& lock)
		: Thread(), mutex(lock), x(0), c(0), desc(false), stats(), fs(*this,5L,false) {}
	void start() {
		x=0;
		if(!fs.isEngaged()) {
			stats=Stats();
			c=0;
			fs.restartFlag=true;
			fs.start();
		}
		Thread::start();
	}
	MutexStressTest& stop() {
		if(!fs.isEngaged()) {
			fs.restartFlag=false;
			fs.stop().join();
		}
		Thread::stop();
		return *this;
	}
	void shutdown() {
		stop().join();
	}
	void report() {
		std::cout << "Ascending cancels: " << stats.ascCancels << "   @VAR Descending cancels: " << stats.descCancels << "   Calls: " << c << std::endl;
	}
protected:
	MutexLock<ProcessID::NumProcesses>& mutex;
	int x,c;
	bool desc;
	struct Stats {
		Stats() : descCancels(0), ascCancels(0) {}
		size_t descCancels;
		size_t ascCancels;
	} stats;
	FailsafeThread fs;
	int depth(int d) {
		MarkScope l(mutex);
		x=mutex.get_lock_level();
		//std::cout << "depth " << x << " of " << d << " by " << pthread_self() << std::endl;
		int ans = d>0 ? depth(d-1)+1 : 0;
		desc=true;
		return ans;
	}
	void* run() {
		int d=0;
		while(true) {
			try {
				desc=false;
				x=0;
				c+=depth(d=(rand()%20)+1);
				//testCancel();
			} catch(...) {
				if(x-1>c)
					std::cerr << "ERROR: depth of " << x << " exceeds target " << d << std::endl;
				//std::cout << "cancel " << pthread_self() << " at " << x << " of " << d << (desc ? " descending" : " ascending") << std::endl;
				throw;
			}
		}
		return NULL;
	}
	void cancelled() {
		if(c==0)
			std::cerr << "Last count was 0?" << std::endl;
		if(desc)
			++stats.descCancels;
		else
			++stats.ascCancels;
	}
};

TimeET ck;

class ConditionThread : public Thread {
public:
	Lock l;
	Condition c;
protected:
	virtual void* run() {
		MarkScope lock(l);
		std::cout << int(ck.Age().Value()) << " child waiting" << std::endl;
		try {
			c.wait(l);
		} catch(...) {
			std::cout << int(ck.Age().Value()) << " child exited wait (cancellation)" << std::endl;
			sleep(2);
			std::cout << int(ck.Age().Value()) << " child exiting, releasing lock" << std::endl;
			throw;
		}
		std::cout << int(ck.Age().Value()) << " child exited wait (returned)" << std::endl;
		sleep(2);
		std::cout << int(ck.Age().Value()) << " child testing cancellation" << std::endl;
		testCancel();
		std::cout << int(ck.Age().Value()) << " child got condition!" << std::endl;
		return NULL;
	}	
};

const char * CALLBACK = "callback";
const char * STRESS = "stress";
const char * SEMAPHORE = "semaphore";
const char * MULTI = "multi";
const char * THREAD1 = "thread1";
const char * THREAD2 = "thread2";
const char * THREAD3 = "thread3";
const char * SERVER = "server";
const char * CONDITION = "condition";

const char * TestNames[] = {
	CALLBACK,
	STRESS,
	SEMAPHORE,
	MULTI,
	THREAD1,
	THREAD2,
	THREAD3,
	SERVER,
	CONDITION,
	NULL
};

void usage(const std::string& x) {
	std::cerr << "USAGE: " << x << " [tests]" << std::endl;
	std::cerr << "Where [tests] are a list of either --test_name or --no-test_name:" << std::endl;
	for(const char** n=TestNames; *n!=NULL; ++n) {
		std::cerr << "  -[-no]-" << *n << std::endl;
	}
}

int main(int argc, char** argv) {
	initialize();
	std::cin.exceptions(ios_base::badbit);
	std::cout.exceptions(ios_base::badbit);
	
	std::set<std::string> tests;
	std::set<std::string> alltests;
	for(const char** n=TestNames; *n!=NULL; ++n) {
		alltests.insert(*n);
	}
	
	if(argc<=1) {
		tests = alltests;
	} else {
		std::string first=argv[1];
		if(first=="-h" || first=="--help") {
			usage(argv[0]);
			return 2;
		} else if(first.substr(0,5)=="--no-") {
			// initialize to all tests, then remove as directed
			tests = alltests;
		}
		for(int i=1; i<argc; ++i) {
			std::string name=argv[i];
			if(name.substr(0,5)=="--no-") {
				name = name.substr(5);
				if(!alltests.count(name)) {
					std::cerr << "Unknown test " << name << std::endl;
					usage(argv[0]);
					return 1;
				}
				tests.erase(name);
			} else if(name.substr(0,2)=="--") {
				name = name.substr(2);
				if(!alltests.count(name)) {
					std::cerr << "Unknown test " << name << std::endl;
					usage(argv[0]);
					return 1;
				}
				tests.insert(name);
			} else {
				usage(argv[0]);
				return 1;
			}
		}
	}
	
	
	if(tests.count(CALLBACK)) {
		HelloThread hello;
		hello.start();
		hello.join();
		CallbackThread doTest1(&test1,&hello);
		doTest1.start();
		doTest1.join();
		CallbackThread doTest2(&test2);
		doTest2.start();
		doTest2.join();
		std::cout << std::endl;
	}
	
	if(tests.count(STRESS)) {
		MutexLock<ProcessID::NumProcesses> mutex;
		std::list<MutexStressTest*> ths;
		for(unsigned int i=0; i<4; ++i) {
			ths.push_back(new MutexStressTest(mutex));
			std::cout << "Mutex stress test, " << ths.size() << " thread" << (ths.size()>1?"s":"") << std::endl;
			TimeET t;
			std::for_each(ths.begin(),ths.end(),std::mem_fun(&MutexStressTest::start));
			sleep(2);
			std::for_each(ths.begin(),ths.end(),std::mem_fun(&MutexStressTest::shutdown));
			std::for_each(ths.begin(),ths.end(),std::mem_fun(&MutexStressTest::report));
		}
		while(!ths.empty()) {
			delete ths.back();
			ths.pop_back();
		}
		ths.clear();
		std::cout << std::endl;
	}
	
	if(tests.count(SEMAPHORE)) {
		SemaphoreThread th;
		th.start();
		CallbackThread joiner(&Thread::join,th);
		joiner.start();
		sleep(2);
		//FailsafeThread failsafe(joiner,2.0,true);
		std::cout << "cancelling semop" << std::endl;
		th.stop();
		void* joinval=joiner.join();
		std::cout << /*"failsafe engage " << failsafe.isEngaged() << ' ' <<*/ (joinval==Thread::CANCELLED) << std::endl;
		if(th.isStarted()) {
			cout << "Zeroing semaphore" << endl;
			semgr.setValue(th.semid,0);
			if(th.join()==Thread::CANCELLED)
				cout << "cancel blocked" << std::endl;
			else
				cout << "cancel unsuccessful" << std::endl;
		} else if(joiner.getReturnValue()==Thread::CANCELLED)
			cout << "cancel successful" << std::endl;
		else
			cout << "cancel successful, but didn't set return code? " << joiner.getReturnValue() << " vs " << Thread::CANCELLED << std::endl;
		std::cout << std::endl;
	}

	if(tests.count(MULTI)) {
		{
			std::cout << "Testing thread increment race condition" << std::endl;
			MultiThreads th;
			sleep(5);
		}
		std::cout << '\n';
	}
	
	if(tests.count(THREAD1)) {
		{
			TestThread1 th;
			th.start();
			sleep(1);
			cout << "cancelling (should be immediate)" << endl;
		}
		cout << "testing lock" << endl;
		l.lock();
		cout << "works" << endl;
		l.unlock();
	}
	
	if(tests.count(THREAD2)) {
		{
			TestThread2 th;
			th.start();
			sleep(1);
			cout << "cancelling (should be ignored until end of sleep, " << (SLEEPTIME-1) << " seconds)" << endl;
		}
		cout << "testing lock" << endl;
		l.lock();
		cout << "works" << endl;
		l.unlock();
	}
	
	if(tests.count(THREAD3)) {
		{
			TestThread3 th;
			th.start();
			sleep(1);
			cout << "cancelling (should be immediate)" << endl;
		}
		cout << "testing lock" << endl;
		l.lock();
		cout << "works" << endl;
		l.unlock();
	}
	
	if(tests.count(SERVER)) {
		{
			ServerThread st;
			st.start();
			TestThread4 th;
			th.start();
			sleep(1);
			cout << "cancelling (should be immediate)" << endl;
		}
		cout << "testing lock" << endl;
		l.lock();
		cout << "works" << endl;
		l.unlock();
	}
	
	if(tests.count(CONDITION)) {
		cout << "\nTesting cancellation from conditional:" << endl;
		ck.Set();
		TimeET round(0.5);
		ck-=round;
		ConditionThread th;
		th.start();
		sleep(1);
		{
			MarkScope lock(th.l);
			//std::cout << ck.Age() << " Sending signal" << std::endl;
			//th.c.broadcast();
			std::cout << int(ck.Age().Value()) << " Parent cancelling child..." << std::endl;
			th.stop();
			sleep(1);
			std::cout << int(ck.Age().Value()) << " Parent cancelling child again..." << std::endl;
			th.stop();
			sleep(1);
			std::cout << int(ck.Age().Value()) << " Parent releasing lock" << std::endl;
		}
		sleep(1);
		std::cout << int(ck.Age().Value()) << " Parent reaquire lock" << std::endl;
		MarkScope lock(th.l);
		std::cout << int(ck.Age().Value()) << " Parent got lock" << std::endl;
	}
	
	destruct();
	return 0;
}

