//-*-c++-*-
#ifndef INCLUDED_MotionCommand_h
#define INCLUDED_MotionCommand_h

#include "Shared/RobotInfo.h"
#include "MotionManagerMsg.h"
#include "Shared/WorldState.h"
#include "OutputCmd.h"
#include <stddef.h>

class EventTranslator;
class EventBase;

//! The abstract base class for motions, provides common interface.  All motions should inherit from this
/*! For instructions on how to create:
 * - <b>an instantiation</b> of an existing MotionCommand, see MotionManager
 * - <b>a new subclass</b> of MotionCommand, read on.  Also see the step-by-step tutorials:
 *   - <a href="../FirstMotionCommand.html">Tekkotsu's "First MotionCommand" Tutorial</a>
 *   - <a href="http://www.cs.cmu.edu/~dst/Tekkotsu/Tutorial/motion.shtml">David Touretzky's Motion Commands chaper</a>
 *   - <a href="http://www.cs.cmu.edu/afs/cs/academic/class/15494-s06/www/lectures/motion_commands.pdf">CMU's Cognitive Robotics slides</a>
 * 
 * To create a new type of motion, you'll want to subclass this.  You
 * don't need to do anything fancy, but just be sure to override the 3
 * abstract functions, updateOutputs(), isAlive(), and isDirty().
 *
 * There is a quick-start boilerplate included in the distribution: <a href="http://cvs.tekkotsu.org/cgi/viewcvs.cgi/Tekkotsu/project/templates/motioncommand.h?rev=HEAD&content-type=text/vnd.viewcvs-markup"><i>project</i><tt>/templates/motioncommand.h</tt></a>:
 *
 * When an output is set to a value, that output is held at that value
 * until it is set to a new value, even if the MotionCommand that set
 * it is pruned or stops using the output.  Outputs never "reset" to 0
 * or some other relatively arbitrary base value if all the
 * MotionCommands are removed.
 *
 * However, PID values will reset to the default values if pruned or
 * not set since these values <i>do</i> have a base value which you
 * will want to use 99% of the time.
 *
 * Be aware that there is a delay between when you set a joint to a
 * value and that actually is taken into account by the system - it's
 * on the order of FrameTime*NumFrames (currently 8*4 = 32 ms, so
 * worse case 2*8*4 = 64 ms) This is because the commands are double
 * buffered.  PIDs, on the other hand, seem to take effect more
 * quickly.  This un-synchronization can sometimes cause a bit of
 * jerkiness (mainly on startup, where there's a large difference
 * between desired and target values.)
 *
 * Here is the cycle of calls made by MotionManager to your command:
 * -# shouldPrune() (by default, this will return !isAlive() iff #autoprune==true)
 * -# updateOutputs() (assuming the MC wasn't pruned after the previous step)
 * 
 * So, if you want to hold a joint at a value, each time your
 * updateOutputs function is called, you should tell the
 * MotionManager to keep the joint there (using one of
 * MotionManager::setOutput()'s).  If you do not set a joint after a
 * call to updateOutputs, the MotionManager will assume you are no
 * longer using that joint and a lower priority MotionCommand may
 * inherit it.
 *
 * MotionCommands which generate events should use the inherited
 * MotionCommand::postEvent() instead of trying to access a global ::erouter - the
 * inherited version will properly handle sending the events
 * regardless of the current process context, but trying to access a
 * non-shared global like ::erouter could cause problems otherwise.
 *
 * @warning <b>Be careful what you call in MotionManager</b> \n
 * Some functions are marked MotionCommand-safe - this is another
 * issue due to our "fake" fork.  In short, when a function is called
 * on a MotionCommand, it uses the context of whatever process created
 * it, not the process that actually made the function call.  Thus,
 * when Motion calls updateOutputs(), calls that the MotionCommand
 * makes are indistinguishable from concurrent calls from Main.  This
 * can cause deadlock if a function is called which locks the
 * MotionManager.  To get around this, we need to pass the 'this'
 * parameter on functions that require a lock, namely
 * MotionManager::setOutputs().  This allows the MotionManager to
 * figure out that it's the same MotionCommand it previously called
 * updateOutputs() on, and thus avoid trying to lock itself again.
 *
 * @warning <b>Don't store pointers in motion commands!</b> \n
 * Since motion commands are in shared memory, and these shared memory
 * regions can have different base pointers in each process, pointers
 * will only be valid in the process from which they were assigned.
 * In other processes, that address may point to something else,
 * especially if it was pointing outside of the shared memory
 * regions.\n
 * There are convoluted ways of getting around this.  If needed,
 * MotionManager could be modified to hand out shared memory regions
 * upon request.  Let's try to avoid this for now.  Keep
 * MotionCommands simple, without dynamic memory.  Do more complicated
 * stuff with behaviors, which only have to worry about running in Main.
 */
class MotionCommand : public MotionManagerMsg {
	//!@nosubgrouping
public:

	//        *****************
	//! @name *** ABSTRACT: *** (must be defined by subclasses)
	//        *****************

	//! is called once per update cycle, can do any processing you need to change your priorities or set output commands on the MotionManager
	/*! @return zero if no changes were made, non-zero otherwise
	 *  @see RobotInfo::NumFrames @see RobotInfo::FrameTime */
	virtual int updateOutputs()=0;

	//! not used by MotionManager at the moment, but could be used to reduce recomputation, and you may find it useful
	/*! @return zero if none of the commands have changed since last
	 *  getJointCmd(), else non-zero */
	virtual int isDirty()=0;

	//! used to prune "dead" motions from the MotionManager
	/*! note that a motion could be "paused" or inactive and therefore
	 * not dirty, but still alive, biding its time to "strike" ;)
	 * @return zero if the motion is still processing, non-zero otherwise */
	virtual int isAlive()=0;

	//@}

	
	//        ******************
	//! @name *** INHERITED: ***
	//        ******************

	//! Constructor: Defaults to kStdPriority and autoprune==true
	MotionCommand() : MotionManagerMsg(), queue(NULL), autoprune(true), started(false)
#ifdef PLATFORM_APERIOS
		, state()
#endif
	{ }
	//! Destructor
	virtual ~MotionCommand() {}
	
	//! called after this is added to MotionManager; don't override this, use doStart instead
	virtual void start() { started=true; doStart(); }

	//! called after this is removed from MotionManager; don't override this, use doStop instead
	virtual void stop() { doStop(); started=false; }

	//! returns true if the MotionCommand is currently running (although it may be overridden by a higher priority MotionCommand)
	virtual bool isActive() const { return started; }

	/*! @return current setting of autopruning - used to remove motion from groups when !isAlive() */
	virtual bool getAutoPrune() { return autoprune; }

	/*! @param ap bool representing requested autopruning setting */
	virtual void setAutoPrune(bool ap) { autoprune=ap; }

	//! whether this motion should be removed from its motion group automatically ( MotionCommand::autoprune && !isAlive())
	/*! @return (MotionCommand::autoprune && !isAlive())*/
	virtual bool shouldPrune() { return (autoprune && !isAlive()); }
	
	//! only called from MMCombo during process setup, allows MotionCommands to send events
	void setTranslator(EventTranslator * q) { queue=q; }
	
#ifdef PLATFORM_APERIOS
	//! called by MotionManager each time a process checks out the motion, makes sure #state is set for the calling process
	void setWorldState(WorldState* s) { state.cur=s; }
#endif

protected:
	//! Override this if you want to run some startup code after being added to the MotionManager.
	virtual void doStart() {}
	//! Override this if you want to clean up after removal from MotionManager
	/*! The old MotionManagerMsg::mc_id will still be available, but this value is no longer valid within the MotionManager.
	 *  Only use it if you need to clean up associated values stored elsewhere. */
	virtual void doStop() {}
	
	//! this utility function will probably be of use to a lot of MotionCommand's
	/*! Does a weighted average of a and b, favoring b by x percent (so x==0 results in a, x==1 results in b)
	 * @param a first value
	 * @param b second value
	 * @param x weight of second value as opposed to first
	 * @return @f$ a*(1.0-x)+b*x @f$
	 * @todo - replace with a more fancy spline based thing? */
	static inline double interpolate(double a, double b, double x) {
		return a*(1.0-x)+b*x;
	}
	//! this utility function will probably be of use to a lot of MotionCommand's
	/*! Does a weighted average of a and b, favoring b by x percent (so x==0 results in a, x==1 results in b)
	 * @param a first value
	 * @param b second value
	 * @param x weight of second value as opposed to first
	 * @return @f$ a*(1.0-x)+b*x @f$
	 * @todo - replace with a more fancy spline based thing? */
	static inline float interpolate(float a, float b, float x) {
		return a*(1-x)+b*x;
	}
	//! this utility function will probably be of use to a lot of MotionCommand's, see interpolate(double a,double b,double r)
	/*! interpolates both value and weights of JointCmd's
	 *  @param a first joint cmd
	 *  @param b second joint cmd
	 *  @param x weight to favor b's value and weight
	 *  @param r joint cmd to store the result */
	static inline void interpolate(const OutputCmd& a, const OutputCmd& b, float x, OutputCmd& r) {
		r.set(interpolate(a.value,b.value,x),interpolate(a.weight,b.weight,x));
	}
	//@}

	//! calls EventTranslator::trapEvent() directly (avoids needing erouter, which is a non-shared global, causes problems with context, grr, silly OS)
	void postEvent(const EventBase& event);

	EventTranslator * queue; //!< send events using this, instead of posting to the erouter

	int autoprune; //!< default true, autoprune setting, if this is true and isAlive() returns false, MotionManager will attempt to remove the MC automatically
	bool started; //!< true if the MotionCommand is currently running (although it may be overridden by a higher priority MotionCommand)
#ifdef PLATFORM_APERIOS
	//! purposely shadows global ::state as a member of MotionCommand, so subclasses will be guaranteed to use the current process's instance instead of that of the process they were created in
	struct StateRedirect {
		WorldState * operator->() const { return cur==NULL ? WorldState::getCurrent() : cur; }
		WorldState * cur;
	} state; 
#endif
	
private:
	MotionCommand(const MotionCommand&); //!< don't call
	MotionCommand& operator=(const MotionCommand&); //!< don't call
};

/*! @file
 * @brief Defines the MotionCommand class, used for creating motions of arbitrary complexity
 * @author ejt (Creator)
 */

#endif

