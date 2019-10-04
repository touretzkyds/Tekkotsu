#ifndef INCLUDED_BatteryCheckControl_h_
#define INCLUDED_BatteryCheckControl_h_

#include "ControlBase.h"
#include "Events/EventListener.h"

//! When activated, this will print a battery report to stdout and light up LEDs to specify power level
/*! The LEDs use the LedEngine::displayPercent() function, with minor/major style.  This means
 *  the left column (viewing the dog head on) will show the overall power level, and the
 *  right column will show the level within the tick lit up in the left column.  The more geeky
 *  among you may prefer to think of this as a two digit base 5 display.
 *
 *  This gives you pretty precise visual feedback as to remaining power (perhaps more than
 *  you really need, but it's as much a demo as a useful tool)
 *
 *  This is implemented as a Control instead of a Behavior on the assumption you
 *  wouldn't want to leave this running while you were doing other things (ie not
 *  in e-stop). But it definitely blurs the line between the two.
 */
class BatteryCheckControl : public ControlBase, public EventListener {
public:
	
	//!Constructor
	BatteryCheckControl() : ControlBase("Battery Check","Reports % power remaining, and gives details on console") {}
	
	//!Destructor
	virtual ~BatteryCheckControl() {}
	
	//!Prints a report to stdio and lights up the face to show battery level
	/*! keeps running until deactivated - will listen for power events and continue to update display */
	virtual ControlBase * activate(MC_ID display, Socket * gui);
	
	//! stops listening for power events and sets display to invalid
	virtual void pause();
	
	//! calls report()
	virtual void refresh();
	
	//! stops listening for power events and sets display to invalid
	virtual void deactivate();
	
	//! calls refresh() to redisplay with new information if it's not a vibration event
	virtual void processEvent(const EventBase& event);
	
	virtual ControlBase * doSelect() { return this; }
	
	//! redisplay text to sout and refresh LED values
	void report();	
};

/*! @file
 * @brief Describes BatteryCheckControl, which will spew a power report to stdout upon activation
 * @author ejt (Creator)
 */

#endif
