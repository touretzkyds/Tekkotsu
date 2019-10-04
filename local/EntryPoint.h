//-*-c++-*-
#ifndef INCLUDED_EntryPoint_h_
#define INCLUDED_EntryPoint_h_

#include "IPC/Thread.h"
#include "Shared/Resource.h"

//! manages a thread lock to serialize behavior computation and mark whether ::state is being read or written
typedef Thread::Lock EntryPoint;
/*
class EntryPoint : public Resource {
public:
	//! constructor, need to specify the WorldStatePool (presumably it's in a shared memory region...)
	explicit EntryPoint() : Resource(), lock() {}
	
	//! an EmptyData implies a WorldStateRead should be passed on to the pool, requesting a write requires a WorldStateWrite to be passed
	virtual void useResource(Data& d) {
		static_cast<Resource&>(lock).useResource(emptyData); //important to get lock first to make sure using shared defaultRead is safe in multi-threaded env.
	}
	//! an EmptyData implies a WorldStateRead should be passed on to the pool, requesting a write requires a WorldStateWrite to be passed
	virtual void releaseResource(Data& d) {
		static_cast<Resource&>(lock).releaseResource(emptyData); //important to release lock last to make sure using shared defaultRead is safe in multi-threaded env.
	}
	
	//! this can be useful when planning to write, get the threadlock to do some initial setup before grabbing an entry from the pool
	Thread::Lock& getLock() { return lock; }
	
protected:
	Thread::Lock lock; //!< only one behavior runs at a time
};
*/

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
