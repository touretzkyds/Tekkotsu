#include "Behaviors/BehaviorBase.h"
#include "Wireless/Wireless.h"
#include "Behaviors/Controller.h"

//! Demonstrates serr, sout, and cout
class HelloWorldBehavior : public BehaviorBase {
public:
	//Note that we pass the name of our class as an argument to the
	//BehaviorBase constructor.  This is used for the default name for
	//instances of this class, and may allow more readable debugging
	//information
	HelloWorldBehavior() : BehaviorBase("HelloWorldBehavior")	{}
	
	virtual void doStart() {
		//call superclass first for housekeeping:
		BehaviorBase::doStart();

		//now do your code:
		const unsigned int LINES=3;
		for(unsigned int i=1; i<=LINES; i++)
			serr->printf("Hello serr!  This is %d of %d\n", i, LINES);
		for(unsigned int i=1; i<=LINES; i++)
			sout->printf("Hello sout!  This is %d of %d\n", i, LINES);
		for(unsigned int i=1; i<=LINES; i++)
			std::cout << "Hello cout!  This is " << i << " of " << LINES << std::endl;
		for(unsigned int i=1; i<=LINES; i++)
			printf("Hello printf!  This is %d of %d\n", i, LINES);

		// This will popup a message in the ControllerGUI, if one is connected
		std::vector<std::string> errmsg;
		errmsg.push_back("Hello World!  This is an informational message via the Controller\n"
		                 "More messages have been sent on the console (cout, sout, etc.)");
		Controller::loadGUI("org.tekkotsu.mon.ControllerErr","",0,errmsg);
		
		// just stop right away since this Behavior has no 'active' state.
		stop(); // Note that you could also override the doStop function to do cleanup...
	}
	
	//! static function allows GUI to present tooltip before any allocation
	/*! Not required to implement, but nice! */
	static std::string getClassDescription() {
		return "A little demo of text output, sends some lines of text "
		"to serr, sout, cout, and printf.";
	}
	
	//! virtual function provides description of instance of unknown type
	/*! Usually just a matter of forwarding call to getClassDescription().
	 *  Again, not required to do, but nice! :) */
	virtual std::string getDescription() const {
		return getClassDescription();
	}
	
};

REGISTER_BEHAVIOR_MENU(HelloWorldBehavior,DEFAULT_TK_MENU);

/*! @file
 * @brief Defines HelloWorldBehavior, a little demo of text output (serr, sout, and cout)
 * @author ejt (Creator)
 */
