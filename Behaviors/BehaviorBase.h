//-*-c++-*-
#ifndef INCLUDED_BehaviorBase_h_
#define INCLUDED_BehaviorBase_h_

#include "Events/EventListener.h"
#include "Shared/ReferenceCounter.h"
#include "Motion/MotionManagerMsg.h"
#include "Behaviors/Controls/ControlBase.h"
#include <string>
#include <set>
#include <map>

class SharedObjectBase;
class BehaviorSwitchControlBase;
class RCRegion;

//! Argument for REGISTER_BEHAVIOR_OPT and REGISTER_BEHAVIOR_MENU_OPT, indicates default options (does not start automatically)
#define BEH_DEFAULTS ControlBase::DEFAULTS

//! Argument for REGISTER_BEHAVIOR_OPT and REGISTER_BEHAVIOR_MENU_OPT, indicates the behavior should be activated at launch instead of waiting for the user to do it manually
#define BEH_START ControlBase::BEH_START

//! Argument for REGISTER_BEHAVIOR_OPT and REGISTER_BEHAVIOR_MENU_OPT, indicates the behavior's memory allocation will *not* be freed between activations, so state will be maintained
#define BEH_RETAIN ControlBase::BEH_RETAIN

//! Argument for REGISTER_BEHAVIOR_OPT and REGISTER_BEHAVIOR_MENU_OPT, indicates the behavior can be run alongside other behaviors
#define BEH_NONEXCLUSIVE ControlBase::BEH_NONEXCLUSIVE


//! The basis from which all other Behaviors should inherit
/*! 
 *  For complex behaviors, it may be helpful to break aspects of the behaviors into independent 'states', and
 *  use a state machine formalism to control them.  See StateNode and Transition for more information.
 *
 *  Quick-start boilerplate is included in the distribution: <a href="http://cvs.tekkotsu.org/cgi/viewcvs.cgi/Tekkotsu/project/templates/behavior.h?rev=HEAD&content-type=text/vnd.viewcvs-markup"><i>project</i><tt>/templates/behavior.h</tt></a>:
 * 
 *  Tutorials:
 *  - <a href="../FirstBehavior.html">Tekkotsu's "First Behavior" Tutorial</a>
 *  - <a href="http://www.cs.cmu.edu/~dst/Tekkotsu/Tutorial/behaviors.shtml">David Touretzky's "Behaviors" Chapter</a>
 *  - <a href="http://www.cs.cmu.edu/afs/cs/academic/class/15494-s06/www/lectures/behaviors.pdf">CMU's Cognitive Robotics course slides</a>
 *  
 *  start() and stop() are control functions to trigger the appropriate state change in the behavior.
 *  preStart(), doStart(), and postStart() are hooks to notify subclasses in order to enact this change.
 *  Thus, subclasses should override one (or more) of the latter 3 functions, and users should call
 *  the main start() / stop() methods to (de)activate the behavior, not the hooks.
 *
 *  Also, if you instantiate a behavior on the stack instead of the heap (this is very rarely done), remember to call 
 *  setAutoDelete(false) (provided from the ReferenceCounter base class) — don't want it to try to free memory
 *  on the stack when the behavior is stopped!  (The stack limits the allocation of the behavior
 *  to the current scope, which overrides the reference counting.)
 *
 *  Don't forget to include a call to the #REGISTER_BEHAVIOR macro so that your behavior will be inserted into the menu system!
 */
class BehaviorBase : public ReferenceCounter, public EventListener {
public:
	//! destructor - if is active when deleted, will display a warning (don't delete directly - use removeReference())
	virtual ~BehaviorBase();
	
	//! Calling this signals that the behavior should start running.  It will increment the reference counter, add to the registry, and call the subclass hooks (preStart() / doStart() postStart() )
	/*! Generally you shouldn't override this -- override some combination of preStart(), doStart(), and postStart() instead. */
	virtual void start();

	//! Calling this signals the behavior to stop running.  In turn, this calls doStop(), then removes the behavior from the registry and subtracts from the reference counter Ñ thus may delete the object if no other references remain
	/*! You shouldn't override this — override doStop instead.\n
	 <b>Warning:</b> if you <i>do</i> override, call this at the <i>end</i> of your stop(), not beginning (as it might @c delete @c this !) */
	virtual void stop();
	
	//! Assigns @a curEvent to #event and calls to doEvent() for user processing
	/*! This is basically a hack for noobs who can't grok function arguments at the expense
	 *  of creating a more complicated overall design for everyone else...\n
	 *  On the up side, this also allows optional event references for StateNodes receiving
	 *  a transition, so they can inspect the triggering event.  Also if we someday want to
	 *  make BehaviorBase smarter and do some built-in event processing, it would go here. */
	virtual void processEvent(const EventBase& curEvent);
	
	//! Identifies the behavior in menus and such
	virtual std::string getName() const { return (instanceName.size()==0) ? getClassName() : instanceName; }
	
	//! Allows dynamic renaming of behaviors
	virtual void setName(const std::string& name) { instanceName=name; }

	//! Gives a short description of what this particular instantiation does (in case a more specific description is needed on an individual basis)
	/*! By default simply returns getName(), because any calls from a
	 *  BehaviorBase function to getClassDescription() are going to call
	 *  BehaviorBase::getClassDescription(), not
	 *  ~YourSubClass~::getClassDescription(), because static functions
	 *  can't be virtual in C++ (doh!)
	 *
	 *  This means that getDescription called on a pointer to a
	 *  BehaviorBase of unknown subtype would always return an empty
	 *  string, which is pretty useless.  So instead we return the name
	 *  in this situation.  If you want getDescription to return
	 *  getClassDescription, you'll have to override it in your subclass
	 *  to do so. */
	virtual std::string getDescription() const {
		std::string d=getClassDescription();
		return (d.size()==0)?getName():d;
	}

	//! Returns the name of the class of this behavior (aka its type) using typeid and gcc's demangle 
	virtual std::string getClassName() const;
	
	//! Gives a short description of what this class of behaviors does... you should override this (but don't have to)
	/*! If you do override this, also consider overriding getDescription() to return it */
	static std::string getClassDescription() { return ""; }

	//! Returns true if the behavior is currently running
	virtual bool isActive() const { return started; }
	
	//! This read-only set allows us list all the currently instantiated behaviors
	/*! Not all of these behaviors are necessarily active, this is everything that has been allocated and not yet deallocated */
	static const std::set<BehaviorBase*>& getRegistry() { return getRegistryInstance(); }
	
	//! Registers a behavior class to be added to the specified Controller menu, use '/' to specify sub-menus, see REGISTER_BEHAVIOR macro.
	/*! This only works when called as an initializer to a static variable.  Once the menus have been initialized, later calls to this function won't update the menus. */
	template<class T> static BehaviorSwitchControlBase* registerControllerEntry(const std::string& name, const std::string& menu, int flags=BEH_DEFAULTS);
	
	//! A simple utility call for #REGISTER_BEHAVIOR and friends to remove trailing "Behavior" suffix from display names and fix CamelCase. Disabled because it interferes with the Storyboard and other components; should be re-implemented inside ControllerGUI.
	static std::string humanifyClassName(const std::string& name);

protected:
	//! Typically would use MotionManager::MC_ID, but re-specified here for convenience and to avoid dependence on MotionManager.h
	typedef MotionManagerMsg::MC_ID MC_ID;
	//! Typically would use MotionManager::invalid_MC_ID, but re-specified here for convenience and to avoid dependence on MotionManager.h
	static const MC_ID invalid_MC_ID=MotionManagerMsg::invalid_MC_ID;
	
	//! Called by start() before the doStart(), allows superclasses to do some initialization startup preceeding subclass customization
	/*! For robustness to future change, subclasses should be sure to call the superclass implementation. */
	virtual void preStart() {}
	
	//! Delegate function for subclasses to be notified when the behavior starts up.
	/*! Should be overridden by subclasses to subscribe to events, install motion commands, etc.
	 *
	 *  doStart() is basically a hook to allow subclasses to jump in and do some customization
	 *  of behavior parameters while the behavior is starting.  If you are writing a behavior
	 *  class and do not expect further derivation, just override doStart() yourself.  However,
	 *  if you do expect further derivation of your class, consider using preStart() or postStart()
	 *  instead, and leave doStart() for the 'leaf' classes. */
	virtual void doStart() {}
	
	//! Called by start() after the doStart(), allows superclasses to complete initialization
	/*! For robustness to future change, subclasses should be sure to call the superclass implementation. */
	virtual void postStart() {}
	
	//! Delegate function for subclasses to be notified when the behavior starts up.
	/*! May be overridden to cleanup when the behavior is shutting down.  However
	 *  events will automatically be unsubscribed, and by using addMotion(), motions
	 *  will automatically be removed by stop(), so you may not need any cleanup.*/
	virtual void doStop() {}
	
	//! Delegate function for event processing, the event itself is pointed to (only for the duration of the doEvent() call!) by #event
	/*! Default implementation watches for 'private' text message events (those forwarded by a BehaviorSwitchControl from ControllerGUI input)
	 *  and will publically rebroadcast them.  The idea is that your own processEvent gets first dibs, but if the behavior doesn't handle
	 *  the text message, it will be handed off for others. */
	virtual void doEvent();
	
	//! Specifies parameters for addMotion()
	enum Prunability_t {
		PERSISTENT, //!< The motion will not be removed until forced to do so by behavior ending or a call to removeMotion()
		PRUNABLE //!< The motion can remove itself once it is "complete", which varies motion to motion.  Some may never "end", making this equivalent to #PERSISTENT in those cases.
	};
	
	//! Registers the MotionCommand (contained within a shared memory region) with the MotionManager, and allows the behavior to track active motions so they can be automatically removed on stop()
	/*! If you manually call motman->addPersistentMotion() or motman->addPrunableMotion(), you avoid
	 *  this safety net, such that for better or worse the motion could be "leaked" and outlive the behavior.
	 *  In rare cases, this may be desired if the motion is self-pruning or shared with another behavior.
	 *
	 *  Note that you can pass either SharedObject<MC> instances, or MotionPtr<MC> instances, which
	 *  automatically expose their internal SharedObject when passed as such. */
	virtual MC_ID addMotion(const SharedObjectBase& mc, Prunability_t prune=PERSISTENT);
	
	//! Registers the MotionCommand (contained within a shared memory region) with the MotionManager, and allows the behavior to track active motions so they can be automatically removed on stop()
	/*! If you manually call motman->addPersistentMotion() or motman->addPrunableMotion(), you avoid
	 *  this safety net, such that for better or worse the motion could be "leaked" and outlive the behavior.
	 *  In rare cases, this may be desired if the motion is self-pruning or shared with another behavior.
	 *
	 *  Note that you can pass either SharedObject<MC> instances, or MotionPtr<MC> instances, which
	 *  automatically expose their internal SharedObject when passed as such. */
	virtual MC_ID addMotion(const SharedObjectBase& mc, Prunability_t prune, float priority);
	
	//! Removes the motion from the MotionManager.
	/*! As long as there exists a SharedObject reference to the underlying memory region (such as within
	 *  a MotionPtr instance), the memory is not actually deallocated, and you can resubmit the same motion
	 *  later if desired. */
	virtual void removeMotion(MC_ID mcid);
	
	//! forwarding function so you can pass SharedObject<T> or MotionPtr<T> directly; extracts the MC_ID and passes to the other removeMotion
	template<template<typename T> class M, class T> void removeMotion(const M<T>& mc) { removeMotion(mc->getID()); }
	
	//! An instance is created for each motion activated through addMotion(), listening in case the motion is removed by some other means, thus invalidating the MC_ID.
	/*! This specifically addresses the use of prunable motions, which could disappear any time.
	 *  We don't want BehaviorBase::autoMotions filling up with dead MC_IDs, which would cause
	 *  trouble when the behavior ends and it tries to re-remove the motions.  (Particularly causing
	 *  trouble if the MC_ID has since been reassigned to someone else!) */
	class MonitorMotion : public EventListener {
	public:
		//! Constructor, no-op; expects a call to monitor()
		MonitorMotion() : EventListener(), owner(NULL), mcid(invalid_MC_ID), mcregion(NULL) {}
		//! Copy constructor (shallow copy), begins monitoring the same MC_ID, on the assumption the original will be going away
		MonitorMotion(const MonitorMotion& m) : EventListener(), owner(NULL), mcid(invalid_MC_ID), mcregion(NULL) { monitor(*m.owner,m.mcid,m.mcregion); }
		//! Assignment (shallow copy), begins monitoring the same MC_ID, on the assumption the original will be going away
		MonitorMotion& operator=(const MonitorMotion& m) { monitor(*m.owner,m.mcid,m.mcregion); return *this; }
		//! Destructor, if #mcid is still valid (not invalid_MC_ID), then it removes it from owner->autoMotions
		~MonitorMotion();
		//! Begins listening for a (motmanEGID, mcid, deactivate) event, storing the #owner so it can be updated when the motion is removed
		void monitor(BehaviorBase& parent, MC_ID mc_id, RCRegion* region);
	protected:
		//! Assumes the event indicates the motion has been removed, so sets mcid to invalid_MC_ID and removes it from owner->autoMotions
		/*! Note that as a result of removing the BehaviorBase::autoMotions entry, this class has also been destructed, so nothing else should happen afterward. */
		virtual void processEvent(const EventBase& event);
		
		BehaviorBase * owner; //!< the behavior which created the motion in #mcid
		MC_ID mcid; //!< the motion which is being monitored for removal
		RCRegion* mcregion; //!< keep a reference to the motion region
	};
	
	//! A list of active motions registered with this behavior to be removed when the behavior stops
	/*! If a motion is externally removed, e.g. prunes itself, the MonitorMotion instance kicks in to remove the dead MC_ID from this list */
	std::map<MC_ID,MonitorMotion> autoMotions;
	
	//! static function to provide well-defined initialization order
	static std::set<BehaviorBase*>& getRegistryInstance();

	//! constructor, will use getClassName() as the instance name
	BehaviorBase();
	//! constructor, @a name is used as both instance name and class name
	explicit BehaviorBase(const std::string& name);
	//! copy constructor; assumes subclass handles copying approriately - i.e. if @a b is active, the copy will be as well, even though doStart was never called..
	BehaviorBase(const BehaviorBase& b);
	//! assignment operator; assumes subclass handles assignment appropriately - i.e. if @a b is active, the copy will be as well, even though doStart was never called..
	BehaviorBase& operator=(const BehaviorBase& b);

	bool started; //!< true when the behavior is active
	std::string instanceName; //!< holds the name of this instance of behavior, if empty then getName() forwards to getClassName()
	const EventBase* event; //!< the event, received by processEvent, stored for duration of call to doEvent() (also doStart() in the case of a StateNode receiving a transition), NULL at all other times

private:
	virtual void DoStart(struct __USE_doStart_NOT_DoStart__&) {} //!< temporary to produce warnings to help people update
	virtual void DoStop(struct __USE_doStart_NOT_DoStart__&) {} //!< temporary to produce warnings to help people update
};

// this is a bit of a hack to resolve mutual references...
#ifndef INCLUDED_BehaviorSwitchControl_h_
#include "Controls/BehaviorSwitchControl.h"

template<class T> BehaviorSwitchControlBase* BehaviorBase::registerControllerEntry(const std::string& name, const std::string& menu, int flags/*=BEH_DEFAULTS*/) {
	ControlBase::ControlRegistry_t& reg = ControlBase::getControllerEntries();
	bool isnew = reg.find(menu)==reg.end() || reg[menu].find(name)==reg[menu].end();
	if(!isnew) {
		std::cerr << "Warning: attempted to register duplicate behavior class " << name << " in menu " << menu << std::endl;
		return NULL; // avoid memory leak on previous entry
	}
	BehaviorSwitchControlBase* entry = new BehaviorSwitchControl<T>(name,flags & BEH_RETAIN);
	//reg[menu][name] = std::make_pair(entry,flags);
	ControlBase::registerControllerEntry(entry,menu,flags);
	return entry;
}
#endif

//! Menu name to place user behaviors by default when using #REGISTER_BEHAVIOR
#define DEFAULT_MENU "User Behaviors"
//! Menu name for framework behaviors, to override #DEFAULT_MENU
#define DEFAULT_TK_MENU "Framework Demos"


//! Should be called in the global namespace of a translation unit (i.e. a '.cc' file) in order to register a behavior in the controller menu system.
/*! Specify the menu name (use '/' to demark submenus) and allows you to pass optional flags (see #BEH_START, #BEH_RETAIN, #BEH_NONEXCLUSIVE). */
#define REGISTER_BEHAVIOR_MENU_OPT(behaviorClass, menu, flags) \
const BehaviorSwitchControlBase* AUTO_REGISTER_NAME(behaviorClass,__LINE__) \
 = BehaviorBase::registerControllerEntry<behaviorClass>(#behaviorClass,menu,flags)

//! Should be called in the global namespace of a translation unit (i.e. a '.cc' file) in order to register a behavior in the controller menu system.
/*! This version assumes the DEFAULT_MENU but allows you to pass optional flags (see #BEH_START, #BEH_RETAIN, #BEH_NONEXCLUSIVE). */
#define REGISTER_BEHAVIOR_OPT(behaviorClass, flags) REGISTER_BEHAVIOR_MENU_OPT(behaviorClass, DEFAULT_MENU, flags)

//! Should be called in the global namespace of a translation unit (i.e. a '.cc' file) in order to register a behavior in the controller menu system.
/*! This version allows you to specify a menu (use '/' to demark submenus), but uses the default style (exclusive execution, no auto-start). */
#define REGISTER_BEHAVIOR_MENU(behaviorClass, menu) REGISTER_BEHAVIOR_MENU_OPT(behaviorClass, menu, BEH_DEFAULTS)

//! Should be called in the global namespace of a translation unit (i.e. a '.cc' file) in order to register a behavior in the controller menu system.  See also #REGISTER_BEHAVIOR_OPT, #REGISTER_BEHAVIOR_MENU, and #REGISTER_BEHAVIOR_MENU_OPT.
/*! This version assumes the #DEFAULT menu and will add the behavior to a behavior group so only one runs at a time. */
#define REGISTER_BEHAVIOR(behaviorClass) REGISTER_BEHAVIOR_MENU_OPT(behaviorClass, DEFAULT_MENU, BEH_DEFAULTS)



/*! @file
 * @brief Describes BehaviorBase from which all Behaviors should inherit
 * @author ejt (Creator)
 */

#endif
