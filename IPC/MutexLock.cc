#include "MutexLock.h"

unsigned int MutexLockBase::usleep_granularity=5000;

#if !defined(PLATFORM_APERIOS) && !defined(MUTEX_LOCK_ET_USE_SOFTWARE_ONLY)
#  if !defined(TEKKOTSU_SHM_STYLE) || TEKKOTSU_SHM_STYLE==NO_SHM
#  else
void MutexLockBase::setSemaphoreManager(SemaphoreManager* mgr) {
	if(mgr==NULL) {
		preallocated=*semgr;
		semgr=&preallocated;
	} else {
		*mgr=*semgr;
		semgr=mgr;
	}
}
void MutexLockBase::aboutToFork() {
	preallocated.aboutToFork();
}
SemaphoreManager MutexLockBase::preallocated;
SemaphoreManager* MutexLockBase::semgr=&preallocated;
#  endif
#endif

/*! @file 
* @brief Defines MutexLock, a software only mutual exclusion lock.
* @author ejt (Creator), Edward A. Lycklama, Vassos Hadzilacos (paper from which this was based)
*/
