//-*-c++-*-
#ifndef INCLUDED_MotionPtr_h_
#define INCLUDED_MotionPtr_h_

#include "Shared/Resource.h"
#include "Events/EventRouter.h"
#include "MMAccessor.h"
#include "IPC/SharedObject.h"
#include "Shared/debuget.h"
#include <stack>

//! A shared_ptr for MotionCommands; provides allocation and access to a MotionCommand, ensuring mutually exclusive access and scope-based deallocation
/*! The MotionPtr class performs several jobs:
 *  - Maintain a reference to the enveloping shared memory region, via SharedObject
 *  - Access to object within a shared memory region, enforcing mutual exclusion via MMAccessor
 *
 *  The first point means that even if the motion is removed from the motion manager or dereferenced by an
 *  external source, your MotionPtr can still access the motion without error.  Further, since the motion's MC_ID
 *  is stored in the motion, user code can test whether a motion is currently active or not.  A corollary is
 *  that when this class goes out of scope, the memory region will also be dereferenced, preventing leaks.
 *  To enforce this more rigidly, if the MotionPtr detects that the memory region it is dereferencing is
 *  only referenced by the MotionManager, it will remove the motion from the MotionManager as well.
 *
 *  The second job addresses the concern that the Motion process may be querying the motion
 *  for output values while you're in the middle of updating the motion command's parameters.
 *  By overloading operator-> to return an MMAccessor, each access to the motion is protected
 *  by a mutual exclusion lock for the scope of access.  Note that if you want to maintain a lock
 *  over several lines (for either performance or because the motion is left in a bad state between
 *  the calls), then you can use MarkScope to maintain the lock:
 *  @code
 *  MotionPtr<Foo> foo;
 *  foo->bar(); // a lock is obtained for the duration of this call
 *  foo->tweak(); // a second lock is obtained for this call
 *  @endcode
 *
 *  Alternatively:
 *  @code
 *  MotionPtr<Foo> foo;
 *  MarkScope lockfoo(foo); // a lock is obtained until 'lockfoo' goes out of scope
 *  foo->bar(); // the lock is maintained across these calls...
 *  foo->tweak();
 *  @endcode
 *
 *  The motion in MotionPtr is always "valid", and will be allocated using the templated motion's default
 *  constructor at the first access.  If you wish to call a different constructor for the motion, you can
 *  pass a SharedObject created with the custom constructor calls to either MotionPtr's constructor or
 *  using operator=:
 *  @code
 *  MotionPtr<Foo> foo; // technically, no Foo has been allocated yet, but that's transparent
 *  foo->bar(); // first access, now the allocation actually occurs using default constructor
 *  foo = SharedObject<Foo>(1,2,4); // replacing the motion, this one constructed by Foo(1,2,4)
 *  
 *  MotionPtr<Foo> foo2(SharedObject<Foo>(8,16,32)); // use motion initialized by Foo(8,16,32)
 *  @endcode
 *  
 *  You can share a motion between multiple MotionPtr instances, thanks to reference counting performed
 *  on the underlying memory region.  When a SharedObjects, or another MotionPtrs, is assigned to a MotionPtr,
 *  a shallow copy is performed internally, so they all reference a common motion command:
 *  @code
 *  // ...continuing previous sample
 *  foo = foo2; // now foo and foo2 both refer to the Foo(8,16,32) instance
 *  @endcode
 *
 *  One advanced feature is that motions can have their memory region reset between activations
 *  by calling #retain(false).  If motions are not retained, this means that if the motion
 *  is removed from the motion manager, the MotionPtr will release its reference to the memory region.
 *  If the motion is accessed again thereafter, a new one will be created.  Note that it is only a reference
 *  counting operation: if external references to the unretained motion remain, the original motion will
 *  survive, it just won't be accessible by this MotionPtr anymore unless reassigned.
 *
 *  Another advanced technique is if you @e really want to "leak" a motion, such as a self-pruning motion
 *  to restore some state as the behavior exits, or if another behavior holds only a MC_ID for the motion
 *  (thus there is no explicit reference to the underlying memory region), then you can extract the motion by:
 *  @code
 *  MotionPtr<Foo> foo; // motion you want to extract
 *  SharedObject<Foo> tmp = foo; // make a new reference to the region
 *  foo.clear() // drop foo's reference now
 *  // SharedObject will still dereference foo's memory region as it leaves this scope,
 *  // but assuming the motion is actually active, the motion manager holds a reference, so the
 *  // motion will survive (otherwise the memory will still be released as usual)
 *  @endcode
 *
 *  However, ideally MotionPtrs should be used for extended shared access to a motion (instead of sharing just the MC_ID).
 *  And if you are adding a "dangling" motion during exit, you should just call motman->addPrunableMotion
 *  directly with a SharedObject instead of using this class.
 */
template<class T>
class MotionPtr : public Resource, public EventListener {
public:
	//! Constructor, defaults to an empty SharedObject instance and will retain the allocation once it is created
	explicit MotionPtr() : Resource(), EventListener(), mcObj(SharedObjectBase::NoInit()), checkouts(), retained(true) {}
	//! Constructor, adopts a reference to @a obj, and will retain the reference between additions to the motion manager
	MotionPtr(const SharedObject<T> obj) : Resource(), EventListener(), mcObj(obj), checkouts(), retained(true) {}
	//! Destructor, if this holds the last non-motion manager reference to the motion, removes it from the motion manager
	~MotionPtr() {
		if(erouter!=NULL) // allows use of statics in libtekkotsu without crashing on destruction
			erouter->removeListener(this,EventBase::motmanEGID);
		ASSERTRET(ProcessID::getID()!=ProcessID::MotionProcess,"How is ~MotionPtr running in motion process?");
		if(active()) {
			MotionManager::MC_ID mcid = mcObj->getID();
			int rc = mcObj.getRegion()->NumberOfReference();
			if(motman->hasReference(ProcessID::MotionProcess,mcid))
				--rc;
			if(rc==2 && motman->hasReference(ProcessID::MainProcess,mcid)) // 2 = 1 for this + 1 for the process's MotionManager
				motman->removeMotion(mcObj->getID()); // last non-MotionManager reference, kill the MC
		}
	}
	
	//! Automatically exposes the underlying shared object so you can pass it to SharedObject-based functions (e.g. BehaviorBase::addMotion() or MotionManager::addPersistentMotion())
	operator const SharedObject<T>&() const {
		if(!allocated())
			const_cast<MotionPtr&>(*this) = SharedObject<T>();
		return mcObj;
	}
	//! Automatically create MMAccessor instance for mutual-exclusion access
	/*! This lets you assign the MotionPtr to a MMAccessor, so that the lock persists under the new scope and name.
	 *  You could also use MarkScope with the MotionPtr. */
	operator MMAccessor<T>() const {
		if(!allocated())
			const_cast<MotionPtr&>(*this) = SharedObject<T>();
		return *mcObj;
	}
	//! Forward smart-pointer style access to MMAccessor; this is the secret to one-line locking
	/*! Keep in mind you can use MarkScope with the MotionPtr to retain the lock over the course of multiple accesses */
	MMAccessor<T> operator->() const { return MMAccessor<T>(*this); }
	//! Forward smart-pointer style access to MMAccessor, just for completeness vs. operator->, which is what you'll want to use
	/*! This is actually a little syntactically wrong, because operator*() should return a reference vs. operator->'s pointer, 
	 *  but then we would lose the locking guarantee. */
	MMAccessor<T> operator*() const { return MMAccessor<T>(*this); }
	
	//! Reassigns the motion reference; henceforth shall share the region with @a obj (shallow copy)
	/*! Internal reference counting by the shared memory region will keep everything happy :) */
	MotionPtr& operator=(const SharedObject<T>& obj) {
		if(active()) {
			if(!retained)
				erouter->removeListener(this,EventBase::motmanEGID,mcObj->getID(),EventBase::deactivateETID);
			if(mcObj.getRegion()->NumberOfReference()==3) // 3 = 1 for this + 1 for Main's MotionManager + 1 for Motion's Motion Manager
				motman->removeMotion(mcObj->getID()); // last non-MotionManager reference, kill the MC
		}
		mcObj = obj;
		if(active() && !retained)
			erouter->addListener(const_cast<MotionPtr*>(this),EventBase::motmanEGID,mcObj->getID(),EventBase::deactivateETID);
		return *this;
	}
	//! Reassigns the motion reference; henceforth shall share the region with @a wr (shallow copy)
	/*! Internal reference counting by the shared memory region will keep everything happy :) */
	MotionPtr& operator=(const MotionPtr& wr) const { operator=(wr.mcObj); }
	
	//! Returns the motion command ID associated with the motion, or MotionManager::invalid_MC_ID if the motion is not allocated or otherwise not registered with the motion manager
	virtual MotionManager::MC_ID getID() const {
		if(!allocated())
			return MotionManager::invalid_MC_ID;
		return mcObj->getID();
	}
	
	//! Removes the reference to the current motion memory region.
	/*! A new one will be created with the default constructor if a future access is attempted. */
	void clear() { mcObj = SharedObjectBase::NoInit(); }
	
	//! Returns true if the motion is non-NULL
	bool allocated() const { return (mcObj.getRegion()!=NULL); }
	
	//! Returns true if the motion is registered with the motion manager (i.e. it is both allocated and has a valid MC_ID)
	bool active() const { return (allocated() && mcObj->getID()!=MotionManager::invalid_MC_ID); }
	
	//! Returns #retained
	bool retain() const { return retained; }
	
	//! Sets #retained
	void retain(bool r) {
		if(retained==r)
			return;
		retained=r;
		if(!retained) {
			if(active())
				erouter->addListener(this,EventBase::motmanEGID,mcObj->getID(),EventBase::deactivateETID);
			else
				clear();
		}
	}
	
protected:
	virtual void useResource(Data&) {
		if(!allocated())
			*this = SharedObject<T>();
		if(mcObj->getID()==MotionManager::invalid_MC_ID)
			return;
		checkouts.push(mcObj->getID());
		motman->checkoutMotion(checkouts.top());
	}
	virtual void releaseResource(Data&) {
		ASSERTRET(checkouts.size()>0,"MotionPtr::releaseResource underflow");
		motman->checkinMotion(checkouts.top());
		checkouts.pop();
	}
	
	//! We only listen for motion deactivate events if not #retained, so if we get a call here we should clear the reference
	virtual void processEvent(const EventBase& event) {
		if(!allocated()) {
			std::cerr << "MotionPtr received event " << event.getDescription() << " without any motion" << std::endl;
			return;
		}
		if(event.getGeneratorID()!=EventBase::motmanEGID || event.getSourceID()!=mcObj->getID() || event.getTypeID()!=EventBase::deactivateETID) {
			std::cerr << "MotionPtr received event " << event.getDescription() << " does not correspond to deactivation of MCID " << mcObj->getID() << std::endl;
			return;
		}
		if(!retained) {
			std::cerr << "MotionPtr received motion deactivation event for " << mcObj->getID() << " even though it is retained" << std::endl;
			return;
		}
		clear();
	}
	
	//! Maintains the reference to the motion's shared memory region
	SharedObject<T> mcObj;
	
	//! Retains a history of useResource() calls, to be released by matching releaseResource() calls.
	/*! This solves the problem of removing a motion while it is locked... the removal clears the ID value in #mcObj, thus
	 *  we need to store it here so we can checkin (unlock) the motion later.  Using a stack lets us handle recursively
	 *  re-adding the motion and receiving a new ID while still locked under the original ID. */
	std::stack<MotionManager::MC_ID> checkouts;
	
	//! If false, clears the motion instance each time it is removed from the motion manager, and then recreate it if a future access occurs.
	/*! If true, the reference on #mcObj will be retained between additions to the motion manager, which is the default behavior. */
	bool retained; 
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
