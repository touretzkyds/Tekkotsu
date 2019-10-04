#include "Behaviors/BehaviorBase.h"
#include "Wireless/Wireless.h"

//! Demonstrates (lack of) blocking using serr to (not) pinpoint a crash
class CrashTestBehavior : public BehaviorBase {
public:
	CrashTestBehavior() : BehaviorBase("CrashTestBehavior")	{}
	
	virtual void doStart() {
		//call superclass first for housekeeping:
		BehaviorBase::doStart();

		serr->printf("I will now crash immediately following line 33\n");
		//now do your code:
		for(unsigned int i=0; i<100; i++) {
			serr->printf("Hello serr!  This is %d\n",i);
			if(i==33)
				*(int*)0xDEADDEAD=0x600DB4E;
		}
		//Hate to break it to you, but we're never going to get here...
	}
	
	static std::string getClassDescription() {
		// This string will be shown by the HelpControl or by the tooltips of the Controller GUI
		return "A little demo of blocking output before a crash after output #33 (yes, this crashes the AIBO)";
	}
	
	virtual std::string getDescription() const { return getClassDescription(); }
};

REGISTER_BEHAVIOR_MENU_OPT(CrashTestBehavior,"Background Behaviors/Debugging Tests",BEH_NONEXCLUSIVE);

/*! @file
 * @brief Defines CrashTestBehavior, demonstrates (lack of) blocking using serr to (not) pinpoint a crash
 * @author ejt (Creator)
 */
