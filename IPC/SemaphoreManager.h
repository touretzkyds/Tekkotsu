//-*-c++-*-
#ifndef INCLUDED_SemaphoreManager_h_
#define INCLUDED_SemaphoreManager_h_

#ifdef PLATFORM_APERIOS
#  warning SemaphoreManager is not Aperios compatable, this is not going to compile
#else

#include "ListMemBuf.h"
#include "Shared/attributes.h"

#ifndef SYSTEM_MAX_SEM
//! Ideally, this would be SEMMSL, but that is hard to portably determine at compile time
/*! If you can't increase your system's SEMMSL (and possibly SEMMNS), you
 *  may want to consider decreasing this. */
#define SYSTEM_MAX_SEM 250
#endif

//! initializes, manages, and releases a set of System V style semaphores
/*! Should be initialized pre-fork into a shared region */
class SemaphoreManager {
protected:
	typedef ListMemBuf<bool,SYSTEM_MAX_SEM> sems_t; //!< shorthand for the type of #sems
	
public:
	//! wouldn't want to claim the entire system's worth, even if we could
	static const unsigned int MAX_SEM=SYSTEM_MAX_SEM;

	//! shorthand for the semaphore indexing type
	typedef sems_t::index_t semid_t;
	//! specify options for how to handle EINTR (signal occurred) error condition during a semaphore operation, see setInterruptPolicy()
	enum IntrPolicy_t {
		INTR_CANCEL_VERBOSE, //!< give up, return failure, display error message on console
		INTR_CANCEL, //!< give up, return failure
		INTR_RETRY_VERBOSE, //!< just repeat semaphore operation, display error message on console
		INTR_RETRY, //!< just repeat semaphore operation
		INTR_THROW_VERBOSE, //!< throw a std::runtime_error exception, display error message on console
		INTR_THROW, //!< throw a std::runtime_error exception
		INTR_EXIT, //!< kill process, with error message
	};
	
	//! Creates a SemaphoreManager with room for sems_t::MAX_ENTRIES entries
	/*! 2 of these entries are reserved for internal use, so user code
	 *  will actually only have access to sems_t::MAX_ENTRIES-2 entries.
	 *  Use available() to determine this value at runtime. */
	SemaphoreManager();
	
	//! Creates a SemaphoreManager with room for @a numRequest entries
	/*! adds 2 for SemaphoreManager's own reference count and mutex lock,
	 *  so you'll actually have @a numRequest semaphores available */
	explicit SemaphoreManager(unsigned int numRequest);
	
	SemaphoreManager(const SemaphoreManager& mm); //!< copy supported
	SemaphoreManager& operator=(const SemaphoreManager& mm); //!< assignment supported
	~SemaphoreManager(); //!< destructor
	
	//! call this on semaphores in shared memory regions if a fork is about to occur so reference counts can be updated for the other process
	void aboutToFork();
	//! call this if semaphores need to be released during an emergency shutdown (otherwise semaphore sets can leak past process shutdown -- bad system design IMHO!)
	void faultShutdown();
	//! returns true if #semid is invalid, indicating further semaphore operations are bogus
	bool hadFault() const { return semid==-1; }

	//! returns a new semaphore id, or invalid() if none are available
	semid_t getSemaphore() ATTR_must_check;
	//! marks a semaphore as available for reassignment
	void releaseSemaphore(semid_t id); 

	//! return the semaphore's interrupt policy (doesn't check @a id validity)
	IntrPolicy_t getInterruptPolicy(semid_t id) const { return intrPolicy[id]; }
	//! set the semaphore's interrupt policy (doesn't check @a id validity)
	void setInterruptPolicy(semid_t id, IntrPolicy_t p) { intrPolicy[id]=p; }
	
	//! Lowers a semaphore's value by @a x, optionally blocking if the value would go negative until it is raised enough to succeed.
	/*! Returns true if the semaphore was successfully lowered.
	 *  If a thread cancellation occurs, the operation WILL NOT be undone before the call unwinds, as hitting zero can unblock threads which can't be recalled. */
	bool lower(semid_t id, unsigned int x, bool block=true) const;
	//! raises a semaphore's value by @a x
	void raise(semid_t id, unsigned int x) const;

	int getValue(semid_t id) const; //!< returns the semaphore's value
	void setValue(semid_t id, int x) const; //!< sets the semaphore's value
	int getNumZeroBlockers(semid_t id) const; //!< return the number of threads/processes which are blocking for the indicated semaphore to hit zero
	
	//! tests if the semaphore's value is zero, optionally blocking until it is
	/*! returns true if the semaphore's value is zero.
	 *  @see testZero_add(), add_testZero() */
	bool testZero(semid_t id, bool block=true) const;
	//! tests if the semaphore's value is zero, optionally blocking until it is, and then adding @a x
	/*! If @a x is negative, then @a addblock will cause it to block
	 *  again until someone else raises the semaphore.  Otherwise, the
	 *  add should be performed as an atomic unit with the test.  If @a
	 *  x is non-negative, then the add should never block.
	 *  If a thread cancellation occurs, the operation WILL be undone before the call unwinds, as this usage is exclusive to other threads.
	 *  @see add_testZero(), testZero() */
	bool testZero_add(semid_t id, int x, bool testblock=true, bool addblock=true) const;
	//! adds @a x to the semaphore's value, optionally blocking in the case that @a x is negative and larger than the semaphore's value.  After adding, test whether the semaphore is zero, optionally blocking again until it is.
	/*! If no blocking is required (e.g. @a x is positive) then the add
	 *  and test should be an atomic pair.
	 *  If a thread cancellation occurs, the operation WILL NOT be undone before the call unwinds, as adding to the count can unblock threads which can't be recalled.
	 *  @see testZero_add(), testZero() */
	bool add_testZero(semid_t id, int x, bool addblock=true, bool testblock=true) const;
	//! adds @a x to the semaphore's value, optionally blocking in the case that @a x1 is negative and larger than the semaphore's value.  After adding, test whether the semaphore is zero, optionally blocking again until it is, and then adding @a x2
	/*! If no blocking is required (e.g. @a x is positive) then the add
	 *  and test should be an atomic pair.
	 *  If a thread cancellation occurs, the operation WILL NOT be undone before the call unwinds, as adding to the count can unblock threads which can't be recalled.
	 *  @see testZero_add(), testZero() */
	bool add_testZero_add(semid_t id, int x1, int x2, bool add1block=true, bool testblock=true, bool add2block=true) const;
	
	//! returns the number of semaphores currently available in the set
	unsigned int available() const { return sems_t::MAX_ENTRIES-sems.size(); }
	//! returns the number of semaphores which have been used
	/*! the SemaphoreManager requires 2 semaphores from the set, one for
	 *  reference counting and one for mutual exclusion during function
	 *  calls */
	unsigned int used() const { return sems.size()-(sems_t::MAX_ENTRIES-nsem); }
	//! returns the "invalid" id value, for testing against getSemaphore
	semid_t invalid() const { return sems.end(); }

protected:
	void init(); //!< basic initialization called by the constructor -- don't call twice

	sems_t sems; //!< tracks which semaphores have been handed out; the bool value isn't actually used
	unsigned int nsem; //!< number of real semaphores in the set obtained from the system
	int semid; //!< the system's identifier for the set
	semid_t mysem; //!< a lock semaphore for management operations on the set (handing out or taking back semaphores from clients)
	semid_t refc; //!< a reference count of this semaphore set -- when it goes back to 0, the set is released from the system
	IntrPolicy_t intrPolicy[sems_t::MAX_ENTRIES]; //!< interrupt policy for each semaphore
};

/*! @file
 * @brief Defines SemaphoreManager, which initializes, manages, and releases a set of System V style semaphores
 * @author ejt (Creator)
 */

#endif //Aperios check

#endif //INCLUDED

