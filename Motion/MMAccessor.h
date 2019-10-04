#ifndef INCLUDED_MMAccessor_h_
#define INCLUDED_MMAccessor_h_

#include "MotionManager.h"

//! This class allows convenient ways of accessing a motion command
/*! Since MotionCommands must be checked out of the motion manager and then checked back
 *  in when they are done, this is a common source of errors, leading to deadlock.  This class
 *  will check the motion out when it's created, and check it back in when it goes out of scope\n
 *  It supports recursive checkin/checkouts. \n
 *  Uses global ::motman
 *
 *  So, instead of doing things like this:
 *  @code
 *  YourMotionCommand* ymc = dynamic_cast<YourMotionCommand*>(motman->checkoutMotion(your_mc_id));
 *  //do 'stuff' with ymc, e.g.: ymc->rollOver();
 *  motman->checkinMotion(your_mc_id);
 *  @endcode
 *  ...which can be error prone in many regards - if 'stuff' returns without checking in, or you
 *  forget to check in, or you get lazy and leave it checked out longer than you should, which can
 *  cause general performance issues (or worse, deadlock)
 *  Using MMAccessor makes it much easier to solve these problems, and is easier to code:
 *  @code
 *  MMAccessor<YourMotionCommand> mma(your_mc_id);
 *  //do 'stuff' with mma, e.g.: mma->rollOver();
 *  @endcode
 *  We can call a return at any point and the motion command will automatically be checked in, and
 *  since C++ guarrantees that the destructor of mma will be called, we don't have to worry about
 *  forgetting about it.  We can limit the scope by placing {}'s around the segment in question:
 *  @code
 *  //pre-stuff
 *  {
 *    MMAccessor<YourMotionCommand> mma(your_mc_id);
 *    //do 'stuff' with mma, e.g.: mma->rollOver();
 *  }
 *  //post-stuff - has no knowledge of mma, out of its scope
 *  @endcode
 *  And, for those astute enough to notice that the theoretical @a rollOver() function is called on
 *  MMAccessor when it's actually a member of YourMotionCommand, this is because MMAccessor behaves as a
 *  'smart pointer', which overloads operator->() so it is fairly transparent to use.
 *
 *  In some cases, you may wish to access a prunable motion, but may be unsure of whether the
 *  motion is still alive.  If it has been pruned, the MC_ID is no longer valid, and will not provide
 *  access to the motion.  Worse, however, is that enough new motions have been created that the
 *  ID has been recycled and now refers to another, different, motion.
 *
 *  The solution to this requires two steps.  First, you must retain the SharedObject you used to
 *  initially create the motion.  This is required because if the MotionManager prunes the motion,
 *  it will dereference the memory region, and if there are no other references to the region, it
 *  will be deallocated, destroying the data.  Second, you pass this SharedObject to the MMAccessor
 *  constructor as shown:
 *  @code
 *  SharedObject<YourMC> yourmc;
 *  // ... stuff ... if yourmc was added to MotionManager, it may or may not still be active
 *  MMAccessor<YourMC> your_acc(*yourmc); // doesn't matter!
 *  // your_acc now provides no-op access if not in MotionManager, checks it out if it is
 *  @endcode
 *  This guarantees safe access regardless to the current status of the motion.  (Note that you can
 *  also just listen for the (EventBase::motmanEGID, MC_ID, EventBase::deactivateETID) event
 *  to be notified when a motion is pruned... however, unless you still have a reference to
 *  the SharedObject, you won't be able to access/reuse the motion after it was pruned)
 *
 *  MMAccessor is a small class, you may consider passing it around instead of a MotionManager::MC_ID
 *  if appropriate.  (Would be appropriate to avoid multiple checkin/outs in a row from different
 *  functions, but not as appropriate for storage and reuse of the same MMAccessor.)
 */
template<class MC_t>
class MMAccessor {
 public:

	//! constructor, checks out by default
	/*! @param id the motion command to check out
	 *  @param ckout if true (default) will checkout upon creation.  otherwise it just gets current address (so you can peek at member fields, which should be safe) */
	MMAccessor(MotionManager::MC_ID id,bool ckout=true) : mc_id(id), checkOutCnt(0), mcptr(NULL) {
		if(ckout)
			checkout();
		else
			mcptr=static_cast<MC_t*>(motman->peekMotion(id));
	}

	//! constructor, allows objects to provide uniform access to MotionCommands, regardless of whether they are currently in the MotionManager
	/*! if ckout is true (default parameter), will attempt to check out the motion if the motion reports it has a valid ID */
	MMAccessor(MC_t & ptr, bool ckout=true) : mc_id(ptr.getID()), checkOutCnt(0), mcptr(&ptr) {
		if(ckout && mc_id!=MotionManager::invalid_MC_ID)
			checkout();
	}

	//! copy constructor - will reference the same mc_id - checkin/checkouts are independent between this and @a a; however, if @a a is checked out, @c this will check itself out as well
	/*! If the original was checked out, this will checkout as well (so #checkOutCnt will be 1) */
	MMAccessor(const MMAccessor& a) : mc_id(a.mc_id), checkOutCnt(0), mcptr(a.mcptr) {
		if(a.checkOutCnt>0)
			checkout();
	}

	//! destructor, checks in if needed
	~MMAccessor() {
		while(checkOutCnt>0)
			checkin();
	}

	//! allows assignment of MMAccessor's, similar to the copy constructor - the two MMAccessor's will control the same MotionCommand
	/*! If the original was checked out, this will checkout as well (so #checkOutCnt will be 1) */
	MMAccessor<MC_t> operator=(const MMAccessor<MC_t>& a) {
		mc_id=a.mc_id;
		mcptr=a.mcptr;
		checkOutCnt=0;
		if(a.checkOutCnt>0)
			checkout();
		return *this;
	}
	
	//! So you can check out if not done by default (or you checked in already)
	/*! @param throwOnNULL if true, indicates an exception should be thrown if the checked out motion is NULL (indicates #mc_id does not reference an active MC) */
	inline MC_t* checkout(bool throwOnNULL=true) {
		mcptr=static_cast<MC_t*>(motman->checkoutMotion(mc_id));
		if(throwOnNULL && mcptr==NULL)
			throw std::runtime_error("MMAccessor attempted to checkout an invalid MC_ID");
		checkOutCnt++;
		return mcptr;
	}

	//! Returns the motion command's address so you can call functions
	inline MC_t* mc() const { return mcptr; }

	//! Checks in the motion
	/*! Don't forget, you can also just limit the scope using extra { }'s */
	inline void checkin() {
		if(checkOutCnt>0) {
			if(mc_id!=MotionManager::invalid_MC_ID)
				motman->checkinMotion(mc_id);
			checkOutCnt--;
		}
		if(checkOutCnt==0)
			mcptr=NULL; // fail fast if we use it after last checkin
	}

	//! Checks in the motion, passing through the value it is passed.
	/*! @return the same value it's passed
	 *
	 *  Useful in situations like this:
	 *  @code
	 *  MMAccessor<myMC> mine(myMC_id);
	 *  if(mine.mc()->foo())
	 *    //do something with motman here
	 *  @endcode
	 *  But we want to check in @a mine ASAP - if we don't reference it
	 *  anywhere in the if statement, we're leaving the MC locked longer
	 *  than we need to.  How about instead doing this:
	 *  @code
	 *  bool cond;
	 *  {MMAccessor<myMC> mine(myMC_id); cond=mine.mc()->foo();}
	 *  if(cond)
	 *    //do something with motman here
	 *  @endcode
	 *  But that uses an extra variable... ewwww... so use this function
	 *  as a pass through to checkin the MC:
	 *  @code
	 *  MMAccessor<myMC> mine(myMC_id);
	 *  if(mine.checkin(mine.mc()->foo()))
	 *    //do something with motman here
	 *  @endcode */
	template<class Ret_t> Ret_t checkin(Ret_t ret) {
		checkin();
		return ret;
	}

	MC_t* operator->() { return mc(); } //!< smart pointer to the underlying MotionCommand
	const MC_t* operator->() const { return mc(); } //!< smart pointer to the underlying MotionCommand
	MC_t& operator*() { return *mc(); } //!< smart pointer to the underlying MotionCommand
	const MC_t& operator*() const { return *mc(); } //!< smart pointer to the underlying MotionCommand
	MC_t& operator[](int i) { return mc()[i]; } //!< smart pointer to the underlying MotionCommand
	const MC_t& operator[](int i) const { return mc()[i]; } //!< smart pointer to the underlying MotionCommand

 protected:
	MotionManager::MC_ID mc_id; //!< the MC_ID that this Accessor was constructed with
	unsigned int checkOutCnt;   //!< counter so we know how many times checkout was called
	MC_t* mcptr;                //!< a pointer to the motion command, should always be valid even when not checked out so you can access member fields (which is reasonably safe)
};

/*! @file
 * @brief Defines MMAccessor, allows convenient ways to check MotionCommands in and out of the MotionManager
 * @author ejt (Creator)
 */

#endif
