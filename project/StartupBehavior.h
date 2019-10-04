//-*-c++-*-
#ifndef INCLUDED_StartupBehavior_h_
#define INCLUDED_StartupBehavior_h_

#include <vector>

#include "Behaviors/BehaviorBase.h"
#include "Behaviors/Controls/BehaviorSwitchControl.h"
#include "Motion/MotionManager.h"
#include "Motion/MotionPtr.h"
#include "Motion/PIDMC.h"

class Controller; // defined in Controller.h, don't actually need header file yet
class ControlBase; // defined in ControlBase.h, don't actually need header file yet

//! This is the default init behavior - it launches daemons, builds the Controller menu structure, does some nice things to fade the joints in to full power (reduces twitch)

/*! This is done by assigning an instance of StartupBehavior to the
 *  global ProjectInterface::startupBehavior variable.  Note that
 *  there's nothing special about the type of this class, it's just
 *  another BehaviorBase - ProjectInterface::startupBehavior could be
 *  any behavior.
 *
 *  This behavior is similar in idea to the init process in
 *  unix/linux.
 *  
 *  If you want some other behavior to handle the root initialization,
 *  simply remove the assignment of ::theStartup to startupBehavior at
 *  the beginning of StartupBehavior.cc.  Then you can substitute your
 *  own behavior instead.  Probably a good way to distribute project
 *  code when you're all done with development and want to ship a
 *  single-purpose memory stick (assuming your users won't want to get
 *  access to things like the Controller anymore... up to you.)
 *
 *  To make your behavior available to run via the Controller, include
 *  a call to the REGISTER_BEHAVIOR macro in a .cc file:
 *  @code
 *  REGISTER_BEHAVIOR( YourBehavior );
 *  @endcode
 *
 *  If you want to make your behavior to launch at startup,
 *  you don't need to replace the startupBehavior.
 *  Just use the REGISTER_BEHAVIOR_OPT macro with BEH_START, e.g.:
 *  @code
 *  REGISTER_BEHAVIOR_OPT( YourBehavior, BEH_START );
 *  @endcode
 *  Be aware that only one-top level behavior runs at a time unless
 *  you specify BEH_NONEXCLUSIVE option as well.  So if you have multiple
 *  BEH_START's, each will be started, and then stopped as the
 *  next starts, with the order undefined.
 */
class StartupBehavior : public BehaviorBase {
public:
	//! Constructor
	StartupBehavior();
	//! Destructor
	virtual ~StartupBehavior();

	static std::string getClassDescription() { return "The initial behavior, when run, sets up everything else"; }
	virtual std::string getDescription() const { return getClassDescription(); }
	
protected:
	virtual void doStart();
	virtual void doStop();
	virtual void doEvent();
	
	//! Initializes the Controller menu structure - calls each of the other Setup functions in turn
	/*! Also adds Shutdown and Help items at the end */
	virtual ControlBase* SetupMenus();
	virtual void processRegisteredMenus();

	virtual ControlBase* SetupVision();              //!< sets up the Vision menu

	virtual void initVision(); //!< Sets up the vision pipelines (held in StartupBehavior_SetupVision.cc)

	virtual void startSubMenu(const std::string& name, const std::string& description=""); //!< future calls to addItem() will be placed under the most recently added item
	virtual void addItem(ControlBase * control); //!< inserts a control at the end of the current menu
	virtual ControlBase* endSubMenu();  //!< closes out a submenu so that future calls to addItem() will be added to the enclosing menu, returns the menu being closed out
	virtual ControlBase* findMenu(ControlBase* cur, const std::string& name) const; //!< finds a '/' delimited submenu, creating it if necessary.

	std::vector<BehaviorBase*> spawned; //!< Holds pointers to all the behaviors spawned from doStart, so they can automatically be stopped on doStop (should only happen on shutdown, but ensures cleanliness)
	std::vector<ControlBase*> setup;     //!< only used during setup - easier than passing it around all the Setup*() functions
	MotionPtr<PIDMC> pids; //!< used to fade in the PIDs up to full strength (from initial zero) This is so the joints don't jerk on startup.
	BehaviorSwitchControlBase::BehaviorGroup behGroup; //!< a behavior group for mutually exclusive execution, used for behaviors which don't set #BEH_NONEXCLUSIVE during registration
};

#endif
