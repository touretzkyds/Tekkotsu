//-*-c++-*-
#ifndef __MUTEX_LOCK_ET__
#define __MUTEX_LOCK_ET__

#include "Shared/Resource.h"
#include "ProcessID.h"
#if !defined(PLATFORM_APERIOS)
#  include "IPC/RCRegion.h"
#endif
#include <iostream>
#include <exception>
#include <typeinfo>

// If you want to use the same software-only lock on both
// PLATFORM_LOCAL and Aperios, then uncomment this next line:
//#define MUTEX_LOCK_ET_USE_SOFTWARE_ONLY

// However, that's probably only of use if you want to debug a problem with the lock itself

class SemaphoreManager;

//! The main purpose of this base class is actually to allow setting of usleep_granularity across all locks
/*! It would be nice if we just put functions in here so we could
 *  reference locks without regard to the number of doors, but
 *  then all processes which use the lock would have to have been
 *  created via fork to handle virtual calls properly, and I don't
 *  want to put that overhead on the otherwise lightweight SoundPlay
 *  process under Aperios. */
class MutexLockBase : public Resource {
public:
	virtual ~MutexLockBase() {} //!< basic destructor
	
	static const unsigned int NO_OWNER=-1U; //!< marks as unlocked
	static unsigned int usleep_granularity; //!< the estimated cost in microseconds of usleep call itself -- value passed to usleep will be 10 times this (only used by software lock implementation on non-Aperios)

	//This section is only needed for the non-software-only locks, using the SemaphoreManager
#if !defined(PLATFORM_APERIOS) && !defined(MUTEX_LOCK_ET_USE_SOFTWARE_ONLY)
#  if !defined(TEKKOTSU_SHM_STYLE) || TEKKOTSU_SHM_STYLE==NO_SHM
	static void aboutToFork() {}
#  else
	//! exception if a lock is created but there aren't any more semaphores available
	class no_more_semaphores : public std::exception {
	public:
		//! constructor
		no_more_semaphores() throw() : std::exception() {}
		//! returns a constant message string
		virtual const char* what() const throw() { return "SemaphoreManager::getSemaphore() returned invalid()"; }
	};
	
	//! sets the SemaphoreManager which will hand out semaphores for any and all locks
	/*! see #preallocated for an explanation of why this function does what it does */
	static void setSemaphoreManager(SemaphoreManager* mgr);
	
	//! returns #semgr;
	static SemaphoreManager* getSemaphoreManager() {
		return semgr;
	}
	//! this should be called if a fork() is about to occur (need to pass notification on to #preallocated)
	static void aboutToFork();
	
protected:
	//! the global semaphore manager object for all locks, may point to preallocated during process initialization or destruction
	static SemaphoreManager* semgr;
	
	//! if a semaphore needs to be reserved, and #semgr is NULL, use #preallocated's current value and increment it
	/*! Here's the conundrum: each shared region needs a lock, each
	 *  lock needs an ID from the semaphore manager, and the semaphore
	 *  manager needs to be in a shared region to coordinate handing out
	 *  IDs.  So this is resolved by having the locks check #semgr to see
	 *  if it is initialized yet, and use this if it is not.
	 *  Then, when the SemaphoreManager is assigned, we will copy over
	 *  preallocated IDs from here
	 *
	 *  For reference, only MutexLock needs to worry about this because
	 *  it's the only thing that's going to need an ID before the
	 *  manager is created.*/
	static SemaphoreManager preallocated;
#  endif
#endif
};



#if !defined(PLATFORM_APERIOS) && !defined(MUTEX_LOCK_ET_USE_SOFTWARE_ONLY)
#  if !defined(TEKKOTSU_SHM_STYLE) || TEKKOTSU_SHM_STYLE==NO_SHM
#    include "Thread.h"

//! Implements a mutual exclusion lock using pthread mutex
/*! Use this to prevent more than one thread from accessing a data structure
*  at the same time (which often leads to unpredictable and unexpected results)
*
*  The template parameter is not used (only needed if compiling with IPC enabled)
*
*  Locks in this class can be recursive or non-recursive, depending
*  whether you call releaseAll() or unlock().  If you lock 5 times, then
*  you need to call unlock() 5 times as well before it will be
*  unlocked.  However, if you lock 5 times, just one call to releaseAll()
*  will undo all 5 levels of locking.
*
*  Just remember, unlock() releases one level.  But releaseAll() completely unlocks.
*
*  Note that there is no check that the thread doing the unlocking is the one
*  that actually has the lock.  Be careful about this.
*/
template<unsigned int num_doors>
class MutexLock : public MutexLockBase {
public:
	//! constructor, gets a new semaphore from the semaphore manager
	MutexLock() : owner_index(NO_OWNER), thdLock() {}
	
	//! destructor, releases semaphore back to semaphore manager
	~MutexLock() {
		if(owner_index!=NO_OWNER) {
			owner_index=NO_OWNER;
			while(thdLock.getLockLevel()>0)
				thdLock.unlock();
		}
	}
	
	//! blocks until lock is achieved.  This is done efficiently using a SysV style semaphore
	/*! You should pass some process-specific ID number as the input - just
	 *  make sure no other process will be using the same value. */
	void lock(int id) {
		thdLock.lock();
		if(owner_index!=static_cast<unsigned>(id))
			owner_index=id;
	}
	
	//! attempts to get a lock, returns true if it succeeds
	/*! You should pass some process-specific ID number as the input - just
	 *  make sure no other process will be using the same value.*/
	bool try_lock(int id) {
		if(!thdLock.trylock())
			return false;
		owner_index=id;
		return true;
	}
	
	//! releases one recursive lock-level from whoever has the current lock
	inline void unlock() {
		if(thdLock.getLockLevel()<=0)
			std::cerr << "Warning: MutexLock::unlock caused underflow" << std::endl;
		if(thdLock.getLockLevel()<=1)
			owner_index=NO_OWNER;
		thdLock.unlock();
	}
	
	//! completely unlocks, regardless of how many times a recursive lock has been obtained
	void releaseAll() {
		owner_index=NO_OWNER;
		while(thdLock.getLockLevel()>0)
			thdLock.unlock();
	}
	
	//! returns the lockcount
	unsigned int get_lock_level() const { return thdLock.getLockLevel(); }
	
	//! returns the current owner's id
	inline int owner() const { return owner_index; }
	
protected:
	friend class MarkScope;
	virtual void useResource(Resource::Data&) { lock(ProcessID::getID()); }
	virtual void releaseResource(Resource::Data&) { unlock(); }
	
	unsigned int owner_index; //!< holds the tekkotsu process id of the current lock owner
	Thread::Lock thdLock; //!< all the actual implementation is handed off to the thread lock
};

#  else /* IPC Lock using Semaphores*/
#    include "SemaphoreManager.h"
#    include <unistd.h>
#    include <pthread.h>

//! Implements a mutual exclusion lock using semaphores (SYSV style through SemaphoreManager)
/*! Use this to prevent more than one process from accessing a data structure
 *  at the same time (which often leads to unpredictable and unexpected results)
 *
 *  The template parameter specifies the maximum number of different processes
 *  which need to be protected.  This needs to be allocated ahead of time, as
 *  there doesn't seem to be a way to dynamically scale as needed without
 *  risking possible errors if two processes are both trying to set up at the
 *  same time.  Also, by using a template parameter, all data structures are
 *  contained within the class's memory allocation, so no pointers are involved.
 *
 *  Locks in this class can be recursive or non-recursive, depending
 *  whether you call releaseAll() or unlock().  If you lock 5 times, then
 *  you need to call unlock() 5 times as well before it will be
 *  unlocked.  However, if you lock 5 times, just one call to releaseAll()
 *  will undo all 5 levels of locking.
 *
 *  Just remember, unlock() releases one level.  But releaseAll() completely unlocks.
 *
 *  Note that there is no check that the process doing the unlocking is the one
 *  that actually has the lock.  Be careful about this.
 *
 *  @warning Doing mutual exclusion in software is tricky business, be careful about any
 *  modifications you make!
 */
template<unsigned int num_doors>
class MutexLock : public MutexLockBase {
public:
	//! constructor, gets a new semaphore from the semaphore manager
	MutexLock() : MutexLockBase(), 
		sem(semgr->getSemaphore()), owner_index(NO_OWNER), owner_thread()
	{
		if(sem==semgr->invalid())
			throw no_more_semaphores();
		semgr->setValue(sem,0);
	}
	
	//! constructor, use this if you already have a semaphore id you want to use from semaphore manager
	MutexLock(SemaphoreManager::semid_t semid) : MutexLockBase(), 
		sem(semid), owner_index(NO_OWNER), owner_thread()
	{
		if(sem==semgr->invalid())
			throw no_more_semaphores();
		semgr->setValue(sem,0);
	}
	
	//! destructor, releases semaphore back to semaphore manager
	~MutexLock() {
		if(owner_index!=NO_OWNER) {
			owner_index=NO_OWNER;
			owner_thread=pthread_self();
			if(semgr!=NULL && !semgr->hadFault()) {
				unsigned int depth=semgr->getValue(sem);
				semgr->setValue(sem,0);
				while(depth-->0)
					Thread::popNoCancel(); 
			}
		}
		if(semgr!=NULL && !semgr->hadFault())
			semgr->releaseSemaphore(sem);
		else
			std::cerr << "Warning: MutexLock leaked semaphore " << sem << " because SemaphoreManager is NULL" << std::endl;
	}
	
	//! blocks until lock is achieved.  This is done efficiently using a SysV style semaphore
	/*! You should pass some process-specific ID number as the input - just
	 *  make sure no other process will be using the same value. */
	void lock(int id) {
		Thread::pushNoCancel();
		doLock(id);
	}
	
	//! attempts to get a lock, returns true if it succeeds
	/*! You should pass some process-specific ID number as the input - just
	 *  make sure no other process will be using the same value.*/
	bool try_lock(int id) {
		Thread::pushNoCancel();
		if(semgr==NULL || semgr->hadFault()) {
			std::cerr << "Warning: MutexLock assuming try_lock success of " << sem << " because SemaphoreManager is NULL" << std::endl;
			owner_index=id;
			return true;
		}
		// blind grab
		semgr->raise(sem,1);
		// see if we have it
		if(owner()==id && isOwnerThread()) {
			//we already had the lock, blind grab added one to its lock level, good to go
			return true;
		} else {
			if(semgr->getValue(sem)==1) {
				// got it with the blind grab, set our info
				owner_index=id;
				owner_thread=pthread_self();
				return true;
			} else {
				// someone else owns it, relinquish the blind grab
				if(!semgr->lower(sem,1,false))
					std::cerr << "Warning: MutexLock::trylock failure caused strange underflow" << std::endl;
				Thread::popNoCancel();
				return false;
			}
		}
	}
	
	//! releases one recursive lock-level from whoever has the current lock
	inline void unlock() {
		releaseResource(emptyData);
		Thread::popNoCancel(); 
	}
	
	//! Completely unlocks, regardless of how many times a recursive lock has been obtained.
	/*! Use with extreme caution. */
	void releaseAll() {
		owner_index=NO_OWNER;
		owner_thread=pthread_self();
		if(semgr==NULL || semgr->hadFault()) {
			std::cerr << "Warning: MutexLock assuming releaseAll of " << sem << " because SemaphoreManager is NULL" << std::endl;
			return;
		}
		unsigned int depth=semgr->getValue(sem);
		semgr->setValue(sem,0);
		while(depth-->0)
			Thread::popNoCancel(); 
	}
	
	//! returns the lockcount
	unsigned int get_lock_level() const {
		if(semgr==NULL || semgr->hadFault())
			return (owner_index==NO_OWNER) ? 0 : 1;
		else
			return semgr->getValue(sem);
	}
	
	//! returns the current owner's id
	inline int owner() const { return owner_index; }
	
protected:
	//! returns true if the current thread is the one which owns the lock
	bool isOwnerThread() {
		pthread_t cur=pthread_self();
		return pthread_equal(cur,owner_thread);
	}
	friend class MarkScope;
	//! does the actual lock acquisition
	void doLock(int id) {
		if(owner_index!=static_cast<unsigned>(id) || !isOwnerThread()) {
			//have to wait and then claim lock
			if(semgr!=NULL && !semgr->hadFault()) {
				semgr->testZero_add(sem,1);
#ifdef DEBUG
				if(owner_index!=NO_OWNER || !pthread_equal(pthread_t(),owner_thread))
					std::cerr << "Owner is not clear: " << owner_index << ' ' << owner_thread << std::endl;
#endif
			} else
				std::cerr << "Warning: MutexLock assuming lock of " << sem << " because SemaphoreManager is NULL" << std::endl;
			owner_index=id;
			owner_thread=pthread_self();
		} else {
			//we already have lock, add one to its lock level
			if(semgr!=NULL && !semgr->hadFault())
				semgr->raise(sem,1);
			else
				std::cerr << "Warning: MutexLock assuming lock of " << sem << " because SemaphoreManager is NULL" << std::endl;
		}
	}
	virtual void useResource(Resource::Data&) {
		doLock(ProcessID::getID());
	}
	virtual void releaseResource(Resource::Data&) {
		if(semgr==NULL || semgr->hadFault()) {
			std::cerr << "Warning: MutexLock assuming unlock of " << sem << " from " << owner_index << " because SemaphoreManager is NULL" << std::endl;
			owner_index=NO_OWNER;
			owner_thread=pthread_t();
			return;
		}
		if(owner_index==NO_OWNER || !isOwnerThread()) {
			std::cerr << "Warning: MutexLock::unlock called by thread that didn't own the lock " << owner_index << ' ' << owner_thread << ' ' << pthread_self() << ' ' << semgr->getValue(sem) << std::endl;
			return;
		}
		int depth = semgr->getValue(sem);
		if(depth==1) {
			owner_index=NO_OWNER;
			owner_thread=pthread_t();
		} else if(depth<=0) {
			std::cerr << "Warning: MutexLock::unlock caused underflow" << std::endl;
			owner_index=NO_OWNER;
			owner_thread=pthread_t();
			return;
		}
		if(!semgr->lower(sem,1,false))
			std::cerr << "Warning: MutexLock::unlock caused strange underflow" << std::endl;
	}
	
	SemaphoreManager::semid_t sem; //!< the SysV semaphore number
	unsigned int owner_index; //!< holds the tekkotsu process id of the current lock owner
	pthread_t owner_thread; //!< holds a thread id for the owner thread
	
private:
	MutexLock(const MutexLock& ml); //!< copy constructor, do not call
	MutexLock& operator=(const MutexLock& ml); //!< assignment, do not call
};




#  endif /* uni-process or (potentially) multi-process lock? */
#else //SOFTWARE ONLY mutual exclusion, used on Aperios, or if MUTEX_LOCK_ET_USE_SOFTWARE_ONLY is defined


//#define MUTEX_LOCK_ET_USE_SPINCOUNT

// if DEBUG_MUTEX_LOCK is defined, we'll display information about each lock
// access while the left front paw button is pressed
//#define DEBUG_MUTEX_LOCK

#ifdef DEBUG_MUTEX_LOCK
#  include "Shared/WorldState.h"
#endif

//! A software only mutual exclusion lock. (does not depend on processor or OS support)
/*! Use this to prevent more than one process from accessing a data structure
 *  at the same time (which often leads to unpredictable and unexpected results)
 *
 *  The template parameter specifies the maximum number of different processes
 *  which need to be protected.  This needs to be allocated ahead of time, as
 *  there doesn't seem to be a way to dynamically scale as needed without
 *  risking possible errors if two processes are both trying to set up at the
 *  same time.  Also, by using a template parameter, all data structures are
 *  contained within the class's memory allocation, so no pointers are involved.
 *
 *  Locks in this class can be recursive or non-recursive, depending
 *  whether you call releaseAll() or unlock().  If you lock 5 times, then
 *  you need to call unlock() 5 times as well before it will be
 *  unlocked.  However, if you lock 5 times, just one call to releaseAll()
 *  will undo all 5 levels of locking.
 *
 *  Just remember, unlock() releases one level.  But releaseAll() completely unlocks.
 *
 *  Note that there is no check that the process doing the unlocking is the one
 *  that actually has the lock.  Be careful about this.
 *
 *  @warning Doing mutual exclusion in software is tricky business, be careful about any
 *  modifications you make!
 *
 * Implements a first-come-first-served Mutex as laid out on page 11 of: \n
 * "A First Come First Served Mutal Exclusion Algorithm with Small Communication Variables" \n
 * Edward A. Lycklama, Vassos Hadzilacos - Aug. 1991
*/
template<unsigned int num_doors>
class MutexLock : public MutexLockBase {
 public:
	//! constructor, just calls the init() function.
	MutexLock() : doors_used(0), owner_index(NO_OWNER), lockcount(0) { init();	}

#ifndef PLATFORM_APERIOS
	//! destructor, re-enables thread cancelability if lock was held (non-aperios only)
	~MutexLock() {
		if(owner_index!=NO_OWNER) {
			owner_index=NO_OWNER;
			thdLock[sem].unlock();
		}
	}
#endif

	//! blocks (by busy looping on do_try_lock()) until a lock is achieved
	/*! You should pass some process-specific ID number as the input - just
	 *  make sure no other process will be using the same value.
	 *  @todo - I'd like to not use a loop here */
	void lock(int id);

	//! attempts to get a lock, returns true if it succeeds
	/*! You should pass some process-specific ID number as the input - just
	 *  make sure no other process will be using the same value.*/
	bool try_lock(int id);

	//! releases one recursive lock-level from whoever has the current lock
	inline void unlock();

	//! completely unlocks, regardless of how many times a recursive lock has been obtained
	void releaseAll() { lockcount=1; unlock(); }
	
	//! returns the lockcount
	unsigned int get_lock_level() const { return lockcount;	}

	//! returns the current owner's id
	inline int owner() const { return owner_index==NO_OWNER ? NO_OWNER : doors[owner_index].id; }

	//! allows you to reset one of the possible owners, so another process can take its place.  This is not tested
	void forget(int id);

#ifdef MUTEX_LOCK_ET_USE_SPINCOUNT
	inline unsigned int getSpincount() { return spincount; } //!< returns the number of times the spin() function has been called
	inline unsigned int resetSpincount() { spincount=0; } //!< resets the counter of the number of times the spin() function has been called
#endif
	
 protected:
	friend class MarkScope;
	virtual void useResource(Resource::Data&) {
		lock(ProcessID::getID());
	}
	virtual void releaseResource(Resource::Data&) {
		unlock();
	}
	
	//! Does the work of trying to get a lock
	/*! Pass @c true for @a block if you want it to use FCFS blocking
	 *  instead of just returning right away if another process has the lock */
	bool do_try_lock(unsigned int index, bool block);

	//! returns the internal index mapping to the id number supplied by the process
	unsigned int lookup(int id); //may create a new entry

#ifdef MUTEX_LOCK_ET_USE_SPINCOUNT
	volatile unsigned int spincount; //!< handy to track how much time we're wasting
	void init() { spincount=0; }//memset((void*)doors,0,sizeof(doors)); } //!< just resets spincount
	inline void spin() {
		spincount++;
#ifndef PLATFORM_APERIOS
		usleep(usleep_granularity*10); //this is a carefully chosen value intended to solve all the world's problems (not)
#endif
	} //!< if you find a way to sleep for a few microseconds instead of busy waiting, put it here
#else
	void init() { } //!< Doesn't do anything if you have the MUTEX_LOCK_ET_USE_SPINCOUNT undef'ed.  Used to do a memset, but that was causing problems....
	//memset((void*)doors,0,sizeof(doors)); } 
	inline void spin() {
#ifndef PLATFORM_APERIOS
		usleep(usleep_granularity*10); //this is a carefully chosen value intended to solve all the world's problems (not)
#endif
	} //!< If you find a way to sleep for a few microseconds instead of busy waiting, put it here
#endif
		
	//! Holds per process shared info, one of these per process
	struct door_t {
		door_t() : id(NO_OWNER), FCFS_in_use(false), BL_ready(false), BL_in_use(false), turn('\0'), next_turn_bit('\0') {} //!< constructor
		//door_t(int i) : id(i), FCFS_in_use(false), BL_ready(false), BL_in_use(false), next_turn_bit('\0') {}
		int id; //!< process ID this doorway is assigned to
		volatile bool FCFS_in_use; //!< In FCFS doorway, corresponds to 'c_i'
		volatile bool BL_ready; //!< Signals past FCFS doorway, ready for BL doorway, corresponds to 'v_i'
		volatile bool BL_in_use; //!< Burns-Lamport doorway, corresponds to 'x_i'
		volatile unsigned char turn; //!< clock pulse, initial value doesn't matter
		unsigned char next_turn_bit; //!< selects which bit of turn will be flipped next
	};

	door_t doors[num_doors]; //!< holds all the doors
	unsigned int doors_used; //!< counts the number of doors used
	unsigned int owner_index; //!< holds the door index of the current lock owner
	unsigned int lockcount; //!< the depth of the lock, 0 when unlocked
};


template<unsigned int num_doors>
void
MutexLock<num_doors>::lock(int id) {
#ifndef PLATFORM_APERIOS
	thdLock[sem].lock();
#endif
	if(owner()!=id) {
		if(!do_try_lock(lookup(id),true)) {
			//spin(); //note the block argument above -- should never spin if that is actually working
			std::cout << "Warning: lock() failed to achieve lock" << std::endl;
		}
	} else {
#ifdef DEBUG_MUTEX_LOCK
		if(state==NULL || state->buttons[LFrPawOffset])
			std::cerr << id << " re-locked " << this << " level " << lockcount+1 << std::endl;
#endif
	}
	lockcount++;
}


template<unsigned int num_doors>
bool
MutexLock<num_doors>::try_lock(int id) {
#ifndef PLATFORM_APERIOS
	if(!thdLock[sem].trylock())
		return false;
#endif
	if(owner()==id) {
#ifdef DEBUG_MUTEX_LOCK
		if(state==NULL || state->buttons[LFrPawOffset])
			std::cerr << id << " re-locked " << this << " level " << lockcount+1 << std::endl;
#endif
		lockcount++;
		return true;
	} else {
		if(do_try_lock(lookup(id),false)) {
			lockcount++;
			return true;
		} else {
#ifndef PLATFORM_APERIOS
			thdLock[sem].unlock())
#endif
			return false;
		}
	}
}


template<unsigned int num_doors>
void
MutexLock<num_doors>::unlock() {
	if(lockcount==0) {
		std::cerr << "Warning: MutexLock::unlock caused underflow" << std::endl;
		return;
	}
#ifdef DEBUG_MUTEX_LOCK
	if(state==NULL || state->buttons[LFrPawOffset])
		std::cerr << doors[owner_index].id << " unlock " << this << " level "<< lockcount << std::endl;
#endif
	if(--lockcount==0) {
		if(owner_index!=NO_OWNER) {
			unsigned int tmp = owner_index;
			owner_index=NO_OWNER;
			doors[tmp].BL_in_use=false;
			doors[tmp].BL_ready=false;
			// *** Lock has been released *** //
#ifndef PLATFORM_APERIOS
			if(owner_index==id) {
				thdLock[sem].unlock();
			}
#endif
		}
	}
}


//! If you define this to do something more interesting, can use it to see what's going on in the locking process
//#define mutexdebugout(i,c) { std::cout << ((char)(i==0?c:((i==1?'M':'a')+(c-'A')))) << std::flush; }


template<unsigned int num_doors>
bool
MutexLock<num_doors>::do_try_lock(unsigned int i, bool block) {
	if(i==NO_OWNER) {
		std::cerr << "WARNING: new process attempted to lock beyond num_doors ("<<num_doors<<")" << std::endl;
		return false;
	}
#ifdef DEBUG_MUTEX_LOCK
	if(state==NULL || state->buttons[LFrPawOffset])
		std::cerr << doors[i].id << " attempting lock " << this << " held by " << owner_index << " at " << get_time() << std::endl;
#endif	
	unsigned char S[num_doors]; // a local copy of everyone's doors
	// *** Entering FCFS doorway *** //
	doors[i].FCFS_in_use=true;
	for(unsigned int j=0; j<num_doors; j++)
		S[j]=doors[j].turn;
	doors[i].next_turn_bit=1-doors[i].next_turn_bit;
	doors[i].turn^=(1<<doors[i].next_turn_bit);
	doors[i].BL_ready=true;
	doors[i].FCFS_in_use=false;
	// *** Leaving FCFS doorway *** //
	for(unsigned int j=0; j<num_doors; j++) {
		while(doors[j].FCFS_in_use || (doors[j].BL_ready && S[j]==doors[j].turn))
			if(block)
				spin();
			else {
				doors[i].BL_ready=false;
#ifdef DEBUG_MUTEX_LOCK
				if(state==NULL || state->buttons[LFrPawOffset])
					std::cerr << doors[i].id << " giving up on lock " << this << " held by " << owner_index << " at " << get_time() << std::endl;
#endif	
				return false;
			}
	}
	// *** Entering Burns-Lamport *** //
	do {
		doors[i].BL_in_use=true;
		for(unsigned int t=0; t<i; t++)
			if(doors[t].BL_in_use) {
				doors[i].BL_in_use=false;
				if(!block) {
					doors[i].BL_ready=false;
#ifdef DEBUG_MUTEX_LOCK
					if(state==NULL || state->buttons[LFrPawOffset])
						std::cerr << doors[i].id << " giving up on lock " << this << " held by " << owner_index << " at " << get_time() << std::endl;
#endif	
					return false;
				}
				while(doors[t].BL_in_use)
					spin();
				break;
			}
	} while(!doors[i].BL_in_use);
	for(unsigned int t=i+1; t<num_doors; t++)
		while(doors[t].BL_in_use)
			spin();
	// *** Leaving Burns-Lamport ***//
	// *** Lock has been given *** //
	owner_index=i;
#ifdef DEBUG_MUTEX_LOCK
	if(state==NULL || state->buttons[LFrPawOffset])
		std::cerr << doors[i].id << " received lock " << this << " at " << get_time() << std::endl;
#endif	
	return true;
}


template<unsigned int num_doors>
unsigned int
MutexLock<num_doors>::lookup(int id) {
	// TODO - this could break if two new processes are adding themselves at the same time
	//        or an id is being forgotten at the same time
	//I'm expecting a very small number of processes to be involved
	//probably not worth overhead of doing something fancy like a sorted array
	unsigned int i;
	for(i=0; i<doors_used; i++)
		if(doors[i].id==id)
			return i;
	if(i==num_doors)
		return NO_OWNER;
	doors[i].id=id;
	doors_used++;
	return i;
}


template<unsigned int num_doors>
void
MutexLock<num_doors>::forget(int id) { //not tested thoroughly (or at all?)
	unsigned int i = lookup(id);
	do_try_lock(i,true);
	doors[i].id=doors[--doors_used].id;
	doors[doors_used].id=NO_OWNER;
	releaseAll();
}

#endif //MUTEX_LOCK_ET_USE_SOFTWARE_ONLY

/*! @file 
 * @brief Defines MutexLock, a software only mutual exclusion lock.
 * @author ejt (Creator), Edward A. Lycklama, Vassos Hadzilacos (paper from which this was based)
 */

#endif
