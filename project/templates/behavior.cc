
// This is an empty Behavior template file.
// Replace YOURNAMEHERE, CLASSNAME, and DESCRIPTION as appropriate, and go to town!

#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"

using namespace std;

//! DESCRIPTION
class CLASSNAME : public BehaviorBase {
protected:
	// <class members>
	// e.g. MotionPtr<LedMC> leds;
	
public:
	//! Constructor, typically shouldn't take any parameters, just initialize members
	CLASSNAME() : BehaviorBase() {}
	
	//! The description will show up as a tooltip in the ControllerGUI
	static std::string getClassDescription() { return "DESCRIPTION"; }
	virtual std::string getDescription() const { return getClassDescription(); }

protected:
	virtual void doStart() {
		// <your startup code here>
		// e.g. erouter->addListener(this, ... );
		// e.g. addMotion(leds);
		// (Event IDs are found in Events/EventBase.h)
	}

	virtual void doStop() {
		// <your shutdown code here>  (if any)
		// Motions added via BehaviorBase::addMotion will be removed automatically.
		// The BehaviorBase will also unsubscribe itself from all events.
	}

	virtual void doEvent() {
		// <your event processing code here>
		// e.g. leds->set(AllLEDMask, 1);
		// you can delete this function if you don't use any events...
		// (in which case, you may want to call stop() at the end of doStart()
	}

private:
	// Providing declarations for these functions will avoid a compiler warning if
	// you have any class members which are pointers.  However, as it is, an error
	// will result if you inadvertantly call to either of these (which is probably
	// a good thing, unless you really intended to copy/assign a behavior, in
	// which case simply provide implementations for the functions)
	CLASSNAME(const CLASSNAME&); //!< don't call (copy constructor)
	CLASSNAME& operator=(const CLASSNAME&); //!< don't call (assignment operator)
};

// This will register with the ControllerGUI menu system so you can start/stop
REGISTER_BEHAVIOR( CLASSNAME );
// to auto-start or allow parallel execution use:
// REGISTER_BEHAVIOR_OPT( CLASSNAME, BEH_START | BEH_NONEXCLUSIVE );

/*! @file
 * @brief Defines CLASSNAME, which DESCRIPTION
 * @author YOURNAMEHERE (Creator)
 */
