#ifndef PLATFORM_APERIOS

#include "SemaphoreManager.h"
#include "Shared/debuget.h"
#include "Thread.h"
#include <cstdlib>
#include <cerrno>
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <iostream>
#include <sys/types.h>
#include <sys/sem.h>
#include <cstring>
#include <unistd.h>

#if defined(__GNU_LIBRARY__) && !defined(_SEM_SEMUN_UNDEFINED) || defined(__FreeBSD__) || defined(__NetBSD__) || defined(__MACH__)
/* union semun is defined by including <sys/sem.h> */
#else
/*! @cond INTERNAL */
/* according to X/OPEN we have to define it ourselves */
union semun {
	int val;                  /* value for SETVAL */
	struct semid_ds *buf;     /* buffer for IPC_STAT, IPC_SET */
	unsigned short *array;    /* array for GETALL, SETALL */
	/* Linux specific part: */
	struct seminfo *__buf;    /* buffer for IPC_INFO */
};
/*! @endcond */
#endif

using namespace std;

SemaphoreManager::SemaphoreManager()
: sems(), nsem(sems_t::MAX_ENTRIES), semid(-1), mysem(sems.end()), refc(sems.end())
{init();}

SemaphoreManager::SemaphoreManager(unsigned int numRequest)
: sems(), nsem(numRequest+2), semid(-1), mysem(sems.end()), refc(sems.end())
{init();}

void SemaphoreManager::init() {
	if(nsem>sems_t::MAX_ENTRIES) {
		cout << "SemaphoreManager created with request for " << nsem << " semaphores, but sems_t::MAX_ENTRIES is " << sems_t::MAX_ENTRIES << endl;
		nsem=sems_t::MAX_ENTRIES;
	}
	unsigned int req=nsem;

	//the seminfo structure is kernel-private and I can't find a portable way to access
	//SEMMSL without it.
	/*semun params; 
	seminfo info;
	params.__buf=info;
	if(semctl(semid,-1,IPC_INFO,params)<0) {
		perror("WARNING: SemaphoreManager query (semctl)");
		//we'll just forge ahead with the default value...
		//exit(EXIT_FAILURE);
	} else {
		if(nsem>info.semmsl)
			nsem=info.semmsl;
	}*/
	
	//So instead we'll do a binary search for the size:
	unsigned int lowbound=0; //inclusive
	unsigned int highbound=nsem; //inclusive
	//note that first pass asks for highbound - if it succeeds there's no search
	while(lowbound!=highbound) {
		semid=semget(IPC_PRIVATE,nsem,IPC_CREAT | IPC_EXCL | 0666);
		if(semid<0) {
			if(errno!=EINVAL && errno!=ENOSPC) {
				perror("ERROR: SemaphoreManager upper limit detection (semget)");
				exit(EXIT_FAILURE);
			}
			//too big
			highbound=nsem-1;
		} else {
			//succeeded -- too low?
			if(semctl(semid,-1,IPC_RMID)<0) {
				perror("ERROR: SemaphoreManager destruction (semctl)");
				exit(EXIT_FAILURE);
			}
			lowbound=nsem;
		}
		nsem=(lowbound+highbound+1)/2;
	}
	//get the semaphore set
	semid=semget(IPC_PRIVATE,nsem,IPC_CREAT | IPC_EXCL | 0666);
	if(semid<0) {
		perror("ERROR: SemaphoreManager construction (semget)");
				exit(EXIT_FAILURE);
	}
	if(nsem!=req)
		cerr << "WARNING: System can only allocate " << nsem << " semaphores per set for id=" << semid << " (SEMMSL or SEMMNS max reached). " << req << " were requested." << endl;
		
	//initialize to 0 (unlocked)
	unsigned short int semvals[sems_t::MAX_ENTRIES];
	for(unsigned int i=0; i<nsem; i++)
		semvals[i]=0;
	semun params; 
	params.array=semvals;
	if(semctl(semid,-1,SETALL,params)<0) {
		perror("ERROR: SemaphoreManager construction (semctl)");
		exit(EXIT_FAILURE);
	}
	
	//burn any extra ids we couldn't actually get from the system
	if(nsem!=sems_t::MAX_ENTRIES) {
		//first use up all the IDs
		while(sems.new_back()!=sems.end()) {}
		//now free the first nsem
		for(unsigned int i=0; i<nsem; i++)
			sems.pop_front();
	}
		
	//take one for ourselves to lock handing out semaphores
	mysem=sems.new_front();
	if(mysem==sems.end()) {
		cerr << "ERROR: could not allocate SemaphoreManager internal lock" << endl;
		exit(EXIT_FAILURE);
	}
	//only one semaphore can be in the process of creation or release at any given time, we have the lock
	setValue(mysem,1);
	//take another for ourselves to use as a reference count on the semaphore set
	refc=sems.new_front();
	if(refc==sems.end()) {
		cerr << "ERROR: could not allocate SemaphoreManager reference counter" << endl;
		exit(EXIT_FAILURE);
	}
	//reference count starts at 0 -- underflow is signalled by negative count
	setValue(refc,0);
	//cerr << "Semaphore set " << semid << " created" << endl;
}

SemaphoreManager::SemaphoreManager(const SemaphoreManager& mm)
: sems(), nsem(mm.nsem), semid(mm.semid), mysem(mm.mysem), refc(mm.refc)
{
	ASSERT(mm.semid!=-1,"Copy of SemaphoreManager with invalid semid!");
	lower(mysem,1); //get a lock on the new set
	sems=mm.sems; //we didn't copy sems earlier because we need a lock for this
	raise(refc,1); //add 1 to reference counter for our new set
	raise(mysem,1); //release lock on new set
	//cerr << "Semaphore set " << semid << " copied" << endl;
}

SemaphoreManager& SemaphoreManager::operator=(const SemaphoreManager& mm) {
	if(&mm==this)
		return *this;
	//ASSERT(semid!=-1,"Assignment to SemaphoreManager with invalid semid!");
	//ASSERT(mm.semid!=-1,"Assignment of SemaphoreManager with invalid semid!");
	if(semid==mm.semid) {
		//both reference the same set, just update some fields
		if(mm.semid!=-1)
			mm.lower(mm.mysem,1); //get a lock on the new set
		mysem=mm.mysem;
		sems=mm.sems;
		nsem=mm.nsem;
		if(mm.semid!=-1)
			mm.raise(mm.mysem,1); //release lock on new set
	} else {
		//we're replacing one set with the other, need to dereference our current set
		//cerr << "Semaphore set " << semid << " dereferenced" << endl;
		if(semid!=-1) {
			lower(mysem,1); //lock current set
			if(!lower(refc,1,false)) { //remove 1 from the reference counter for our current set
				//ran out of references to the old set, delete it
				//cerr << "Semaphore set " << semid << " deleted" << endl;
				sems.erase(refc);
				sems.erase(mysem);
				for(semid_t it=sems.begin(); it!=sems.end(); it=sems.next(it))
					if(it<nsem)
						cerr << "Warning: semaphore id " << it << " from set " << semid << " was still active when the set was dereferenced" << endl;
				if(semctl(semid,-1,IPC_RMID)<0) {
					perror("ERROR: SemaphoreManager deletion from operator= (semctl)");
					exit(EXIT_FAILURE);
				}
				semid=-1;
			} else
				raise(mysem,1); // it's still referenced, unlock for others
		}
		if(mm.semid!=-1)
			mm.lower(mm.mysem,1); //get a lock on the new set
		mysem=mm.mysem;
		sems=mm.sems;
		nsem=mm.nsem;
		semid=mm.semid;
		if(mm.semid!=-1) {
			raise(refc=mm.refc,1); //add 1 to reference counter for our new set
			mm.raise(mm.mysem,1); //release lock on new set
		}
		//cerr << "Semaphore set " << semid << " assigned" << endl;
	}
	return *this;
}

SemaphoreManager::~SemaphoreManager() {
	if(semid==-1)
		return;
	//cerr << "Semaphore set " << semid << " dereferenced" << endl;
	lower(mysem,1); //lock current set
	if(!lower(refc,1,false)) { //remove 1 from the reference counter for our current set
		//ran out of references to the old set, delete it
		//cerr << "Semaphore set " << semid << " deleted" << endl;
		/* // on the final shutdown, the process-local copies can't tell if semaphores were freed remotely
		sems.erase(refc);
		sems.erase(mysem);
		for(semid_t it=sems.begin(); it!=sems.end(); it=sems.next(it))
			cerr << "Warning: semaphore id " << it << " from set " << semid << " was still active when the set was dereferenced" << endl;
		*/
		if(semctl(semid,-1,IPC_RMID)<0) {
			perror("ERROR: SemaphoreManager deletion from destructor (semctl)");
			exit(EXIT_FAILURE);
		}
		semid=-1;
	} else
		raise(mysem,1);
}

void SemaphoreManager::aboutToFork() {
	raise(refc,1);
}

void SemaphoreManager::faultShutdown() {
	if(semid==-1)
		return; // already released set
	if(semctl(semid,-1,IPC_RMID)<0)
		perror("WARNING: SemaphoreManager faultShutdown (semctl)");
	semid=-1;
}

SemaphoreManager::semid_t SemaphoreManager::getSemaphore() {
	lower(mysem,1);
	semid_t id=sems.new_front();
	raise(mysem,1);
	if(id!=sems.end())
		setValue(id,0);
	intrPolicy[id]=INTR_RETRY;
	return id;
}
void SemaphoreManager::releaseSemaphore(semid_t id) {
	lower(mysem,1);
	sems.erase(id);
	raise(mysem,1);
}

bool SemaphoreManager::lower(semid_t id, unsigned int x, bool block/*=true*/) const {
	sembuf sb={id,(short)-x,short(block?0:IPC_NOWAIT)};
	while(true) {
		Thread::requestInterruptOnCancel();
		int res = semop(semid,&sb,1);
		int theErr = errno;
		Thread::unrequestInterruptOnCancel();
		if(res==0)
			break;
		if(theErr==EAGAIN)
			return false;
		if(theErr==EINTR) {
			switch(intrPolicy[id]) {
				case INTR_CANCEL_VERBOSE:
					perror("ERROR: SemaphoreManager unable to lower semaphore (semop)");
					cerr << "       semop was interrupted by signal, cancelling lower()";
				case INTR_CANCEL:
					return false;
				case INTR_RETRY_VERBOSE:
					perror("ERROR: SemaphoreManager unable to lower semaphore (semop)");
					cerr << "       semop was interrupted by signal.  Trying again...";
					break;
				case INTR_RETRY:
					break; //while loop will retry
				case INTR_THROW_VERBOSE:
					perror("ERROR: SemaphoreManager unable to lower semaphore (semop)");
					cerr << "       semop was interrupted by signal.  Throwing exception...";
				case INTR_THROW:
					throw std::runtime_error("EINTR returned by lower semop");
				case INTR_EXIT:
					perror("ERROR: SemaphoreManager unable to lower semaphore (semop)");
					cerr << "       semop was interrupted by signal.  Exiting...";
					exit(EXIT_FAILURE);
			}
		} else {
			if(theErr==EIDRM)
				usleep(500000); // probably in the process of shutdown, wait half a second before complaining
			cerr << "ERROR: SemaphoreManager unable to lower semaphore (semop): " << strerror(theErr) << endl;
			cerr << "       ";
			if(theErr==EIDRM) {
				cerr << "Semaphore set has been removed.  " << endl;
			}
			if(theErr==EINVAL) {
				cerr << "Semaphore set was deleted.  " << endl;
			}
			//prevent recuring problems
			cerr << "Goodbye" << endl;
			exit(EXIT_FAILURE);
		}
	}
	return true;
}
void SemaphoreManager::raise(semid_t id, unsigned int x) const {
	sembuf sb={id,(short)x,0};
	if(semop(semid,&sb,1)<0) {
		perror("ERROR: SemaphoreManager unable to raise semaphore (semop)");
	}
}
int SemaphoreManager::getValue(semid_t id) const {
	int ans=semctl(semid,id,GETVAL);
	if(ans<0)
		perror("ERROR: SemaphoreManager getValue (semctl)");
	return ans;
}
void SemaphoreManager::setValue(semid_t id, int x) const {
	semun params; 
	params.val=x;
	if(semctl(semid,id,SETVAL,params)<0) {
		perror("ERROR: SemaphoreManager::setValue (semctl)");
		exit(EXIT_FAILURE);
	}
}
int SemaphoreManager::getNumZeroBlockers(semid_t id) const {
	int ans=semctl(semid,id,GETZCNT);
	if(ans<0)
		perror("ERROR: SemaphoreManager getNumZeroBlockers (semctl)");
	return ans;
}
bool SemaphoreManager::testZero(semid_t id, bool block/*=true*/) const {
	sembuf sb={id,0,short(block?0:IPC_NOWAIT)};
	while(true) {
		Thread::requestInterruptOnCancel();
		int res = semop(semid,&sb,1);
		int theErr = errno;
		Thread::unrequestInterruptOnCancel();
		if(res==0)
			break;
		if(theErr==EAGAIN)
			return false;
		if(theErr!=EINTR) { // && theErr!=ERESTART?
			if(theErr==EIDRM)
				usleep(500000); // probably in the process of shutdown, wait half a second before complaining
			cerr << "ERROR: SemaphoreManager unable to testZero() (semop): " << strerror(theErr) << '\n';
			cerr << "       ";
			if(theErr==EIDRM) {
				cerr << "Semaphore set has been removed.  " << endl;
			} else if(theErr==EINVAL) {
				cerr << "Semaphore set was deleted.  " << endl;
			} else {
				cerr << "Error code was " << theErr << endl;
			}
			cerr << "Goodbye" << endl;
			exit(EXIT_FAILURE);
		} else {
			switch(intrPolicy[id]) {
				case INTR_CANCEL_VERBOSE:
					perror("ERROR: SemaphoreManager unable to testZero (semop)");
					cerr << "       semop was interrupted by signal, cancelling testZero()";
				case INTR_CANCEL:
					return false;
				case INTR_RETRY_VERBOSE:
					perror("ERROR: SemaphoreManager unable to testZero (semop)");
					cerr << "       semop was interrupted by signal.  Trying again...";
					break;
				case INTR_RETRY:
					break; //while loop will retry
				case INTR_THROW_VERBOSE:
					perror("ERROR: SemaphoreManager unable to testZero (semop)");
					cerr << "       semop was interrupted by signal.  Throwing exception...";
				case INTR_THROW:
					throw std::runtime_error("EINTR returned by testZero semop");
				case INTR_EXIT:
					perror("ERROR: SemaphoreManager unable to testZero (semop)");
					cerr << "       semop was interrupted by signal.  Exiting...";
					exit(EXIT_FAILURE);
			}
		}
	}
	return true;
}
bool SemaphoreManager::testZero_add(semid_t id, int x, bool testblock/*=true*/, bool addblock/*=true*/) const {
	sembuf sb[2]={
		{id,0,short(testblock?0:IPC_NOWAIT)},
		{id,(short)x,short(addblock?0:IPC_NOWAIT)}
	};
	while(true) {
		Thread::requestInterruptOnCancel();
		int res = semop(semid,sb,2);
		int theErr = errno;
		try {
			Thread::unrequestInterruptOnCancel();
		} catch(...) { // thread cancelled
			if(res==0) { // if we weren't interrupted, then semop was successful, undo it
#ifdef DEBUG
				ASSERT(lower(id,x,false),"Could not undo semop after thread cancel in testZero_add (would block?)");
#else
				lower(id,x,true); // block should not occur
#endif
			}
			throw;
		}
		if(res==0)
			break;
		if(theErr==EAGAIN)
			return false;
		if(theErr!=EINTR) { // && theErr!=ERESTART?
			if(theErr==EIDRM)
				usleep(500000); // probably in the process of shutdown, wait half a second before complaining
			cerr << "ERROR: SemaphoreManager unable to testZero_add() (semop): " << strerror(theErr) << '\n';
			cerr << "       ";
			if(theErr==EIDRM) {
				cerr << "Semaphore set has been removed.  " << endl;
			} else if(theErr==EINVAL) {
				cerr << "Semaphore set was deleted.  " << endl;
			} else {
				cerr << "Error code was " << theErr << endl;
			}
			cerr << "Goodbye" << endl;
			exit(EXIT_FAILURE);
		} else {
			switch(intrPolicy[id]) {
				case INTR_CANCEL_VERBOSE:
					perror("ERROR: SemaphoreManager unable to testZero_add (semop)");
					cerr << "       semop was interrupted by signal, cancelling testZero_add()";
				case INTR_CANCEL:
					return false;
				case INTR_RETRY_VERBOSE:
					perror("ERROR: SemaphoreManager unable to testZero_add (semop)");
					cerr << "       semop was interrupted by signal.  Trying again...";
					break;
				case INTR_RETRY:
					break; //while loop will retry
				case INTR_THROW_VERBOSE:
					perror("ERROR: SemaphoreManager unable to testZero_add (semop)");
					cerr << "       semop was interrupted by signal.  Throwing exception...";
				case INTR_THROW:
					throw std::runtime_error("EINTR returned by testZero_add semop");
				case INTR_EXIT:
					perror("ERROR: SemaphoreManager unable to testZero_add (semop)");
					cerr << "       semop was interrupted by signal.  Exiting...";
					exit(EXIT_FAILURE);
			}
		}
	}
	return true;
}
bool SemaphoreManager::add_testZero(semid_t id, int x, bool addblock/*=true*/, bool testblock/*=true*/) const {
	sembuf sb[2]={
		{id,(short)x,short(addblock?0:IPC_NOWAIT)},
		{id,0,short(testblock?0:IPC_NOWAIT)}
	};
	while(true) {
		Thread::requestInterruptOnCancel();
		int res = semop(semid,sb,2);
		int theErr = errno;
		Thread::unrequestInterruptOnCancel();
		if(res==0)
			break;
		if(theErr==EAGAIN)
			return false;
		if(theErr!=EINTR) { // && theErr!=ERESTART?
			if(theErr==EIDRM)
				usleep(500000); // probably in the process of shutdown, wait half a second before complaining
			cerr << "ERROR: SemaphoreManager unable to add_testZero() (semop): " << strerror(theErr) << '\n';
			cerr << "       ";
			if(theErr==EIDRM) {
				cerr << "Semaphore set has been removed.  " << endl;
			} else if(theErr==EINVAL) {
				cerr << "Semaphore set was deleted.  " << endl;
			} else {
				cerr << "Error code was " << theErr << endl;
			}
			cerr << "Goodbye" << endl;
			exit(EXIT_FAILURE);
		} else {
			switch(intrPolicy[id]) {
				case INTR_CANCEL_VERBOSE:
					perror("ERROR: SemaphoreManager unable to add_testZero (semop)");
					cerr << "       semop was interrupted by signal, cancelling add_testZero()";
				case INTR_CANCEL:
					return false;
				case INTR_RETRY_VERBOSE:
					perror("ERROR: SemaphoreManager unable to add_testZero (semop)");
					cerr << "       semop was interrupted by signal.  Trying again...";
					break;
				case INTR_RETRY:
					break; //while loop will retry
				case INTR_THROW_VERBOSE:
					perror("ERROR: SemaphoreManager unable to add_testZero (semop)");
					cerr << "       semop was interrupted by signal.  Throwing exception...";
				case INTR_THROW:
					throw std::runtime_error("EINTR returned by add_testZero semop");
				case INTR_EXIT:
					perror("ERROR: SemaphoreManager unable to add_testZero (semop)");
					cerr << "       semop was interrupted by signal.  Exiting...";
					exit(EXIT_FAILURE);
			}
		}
	}
	return true;
}

bool SemaphoreManager::add_testZero_add(semid_t id, int x1, int x2, bool add1block/*=true*/, bool testblock/*=true*/, bool add2block/*=true*/) const {
	sembuf sb[3]={
		{id,(short)x1,short(add1block?0:IPC_NOWAIT)},
		{id,0,short(testblock?0:IPC_NOWAIT)},
		{id,(short)x2,short(add2block?0:IPC_NOWAIT)}
	};
	while(true) {
		Thread::requestInterruptOnCancel();
		int res = semop(semid,sb,3);
		int theErr = errno;
		Thread::unrequestInterruptOnCancel();
		if(res==0)
			break;
		if(theErr==EAGAIN)
			return false;
		if(theErr!=EINTR) { // && theErr!=ERESTART?
			if(theErr==EIDRM)
				usleep(500000); // probably in the process of shutdown, wait half a second before complaining
			cerr << "ERROR: SemaphoreManager unable to add_testZero() (semop): " << strerror(theErr) << '\n';
			cerr << "       ";
			if(theErr==EIDRM) {
				cerr << "Semaphore set has been removed.  " << endl;
			} else if(theErr==EINVAL) {
				cerr << "Semaphore set was deleted.  " << endl;
			} else {
				cerr << "Error code was " << theErr << endl;
			}
			cerr << "Goodbye" << endl;
			exit(EXIT_FAILURE);
		} else {
			switch(intrPolicy[id]) {
				case INTR_CANCEL_VERBOSE:
					perror("ERROR: SemaphoreManager unable to add_testZero (semop)");
					cerr << "       semop was interrupted by signal, cancelling add_testZero()";
				case INTR_CANCEL:
					return false;
				case INTR_RETRY_VERBOSE:
					perror("ERROR: SemaphoreManager unable to add_testZero (semop)");
					cerr << "       semop was interrupted by signal.  Trying again...";
					break;
				case INTR_RETRY:
					break; //while loop will retry
				case INTR_THROW_VERBOSE:
					perror("ERROR: SemaphoreManager unable to add_testZero (semop)");
					cerr << "       semop was interrupted by signal.  Throwing exception...";
				case INTR_THROW:
					throw std::runtime_error("EINTR returned by add_testZero semop");
				case INTR_EXIT:
					perror("ERROR: SemaphoreManager unable to add_testZero (semop)");
					cerr << "       semop was interrupted by signal.  Exiting...";
					exit(EXIT_FAILURE);
			}
		}
	}
	return true;
}

/*! @file
 * @brief Implements SemaphoreManager, which initializes, manages, and releases a set of System V style semaphores
 * @author ejt (Creator)
 */

#endif //Aperios check
