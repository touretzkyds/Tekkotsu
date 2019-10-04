#ifndef INCLUDED_MoCapLogger_h_
#define INCLUDED_MoCapLogger_h_

#include "Behaviors/Controls/ControlBase.h"
#include "Behaviors/Controls/NullControl.h"
#include "Behaviors/Controls/ToggleControl.h"
#include "Behaviors/Controls/StringInputControl.h"
#include "Events/MoCapEvent.h"
#include "Events/TextMsgEvent.h"
#include "Events/EventCallback.h"
#include "Shared/TimeET.h"
#include <fstream>
#include <memory>

//! Provides display and logging of mocap data
/*! Send a text message event "mocap" to report next mocap event on the console.
 *  From the Tekkotsu HAL, that would be the command 'msg mocap'.
 *  
 *  Logged data is tab delimited with fields: time frame posX posY posZ [ori0 ori1 ori2]
 *  
 *  Availability of particular frames or orientation data depends on the data source.
 *  Orientation data can be either yaw-pitch-roll (in degrees), or the axis component
 *  of the quaternion, depending on the state of #rotAxis.  Default is yaw-pitch-roll. */
class MoCapLogger : public ControlBase {
protected:
	TimeET lastRefresh; //!< limits rate of GUI updates
	NullControl* x; //!< displays current x position
	NullControl* y; //!< displays current y position
	NullControl* z; //!< displays current z position
	NullControl* r0; //!< displays the first rotation component (heading or x)
	NullControl* r1; //!< displays the second rotation component (pitch or y)
	NullControl* r2; //!< displays the third rotation component (roll or z)
	ToggleControl * consoleLog; //!< lets user control whether to dump to console
	StringInputControl * fileLog; //!< lets user control whether to dump to file
	ToggleControl * rotAxis; //!< lets user control whether they want yaw-pitch-roll or the rotation axis
	
	std::ofstream file; //!< the output stream if dumping to file
	
	//! convenience function for updating UI elements
	static void rename(ControlBase* c, std::ostream& sstream);
	
	
	// Note how auto_ptr deletes during destruction, thus EventCallback automatically unsubscribes: no cleanup necessary
	// We store pointers so we can test for current status, but could use erouter->isListening() instead...
	
	std::unique_ptr<EventCallbackAs<MoCapEvent> > mocapGUI; //!< triggers callback to gotMoCapEvent()
	void gotMoCapGUI(const MoCapEvent& mce); //!< refreshes the ControllerGUI display
	
	std::unique_ptr<EventCallbackAs<MoCapEvent> > mocapConsole; //!< triggers callback to gotMoCapConsole()
	void gotMoCapConsole(const MoCapEvent& mce); //!< forwards to dump(std::cout, mce)... if not for #rotAxis, could make dump() static and just call it directly
	
	std::unique_ptr<EventCallbackAs<MoCapEvent> > mocapFile; //!< triggers callback to gotMoCapFile()
	void gotMoCapFile(const MoCapEvent& mce); //!< forwards to dump(#file, mce)... if not for #rotAxis, could make dump() static and just call it directly
	
	
	// These two are 'always on', just stored directly as members
	
	EventCallbackAs<TextMsgEvent> txtmsgSingle; //!< triggers callback to gotTxtMsgSingle()
	void gotTxtMsgSingle(const TextMsgEvent& txt); //! activates mocapSingle to dump next mocap event, and a timer to report error
	
	EventCallback mocapSingle; //!< triggers callback to gotMoCapSingle()
	void gotMoCapSingle(const EventBase& event); //!< if receives MoCapEvent, dumps to std::cout, otherwise assumes timer and complains about lack of MoCapEvents
	
public:
	//! Constructor
	MoCapLogger() : ControlBase("MoCap Logger","Displays and log motion capture event data (e.g. from Mirage)"), 
		lastRefresh(), x(), y(), z(), r0(), r1(), r2(), consoleLog(), fileLog(), rotAxis(), file(), mocapGUI(), mocapConsole(), mocapFile(),
		txtmsgSingle(&MoCapLogger::gotTxtMsgSingle,*this),
		mocapSingle(&MoCapLogger::gotMoCapSingle,*this)
	{
		pushSlot(x = new NullControl("X: waiting"));
		pushSlot(y = new NullControl("Y: waiting"));
		pushSlot(z = new NullControl("Z: waiting"));
		pushSlot(r0 = new NullControl("Heading: waiting"));
		pushSlot(r1 = new NullControl("Pitch: waiting"));
		pushSlot(r2 = new NullControl("Roll: waiting"));
		pushSlot(consoleLog = new ToggleControl("Log To Console","Will write MoCapEvent data to the console"));
		pushSlot(fileLog = new StringInputControl("Log To File","Enter filename to dump mocap data into"));
		pushSlot(rotAxis = new ToggleControl("Rotation Axis","If selected, will display/log the quaternion axis, otherwise heading/pitch/roll"));
	}
	
	//! Serialize data from @a mce into @a os
	void dump(std::ostream& os, const MoCapEvent& mce);
	
	//! called when a child has deactivated and this control should refresh its display, or some other event (such as the user pressing the refresh button) has happened to cause a refresh to be needed
	virtual void refresh();
	
	//! called when this control is being popped from the control stack
	virtual void deactivate();
	
	//! when the user has trigger an "open selection"
	/*! default is to return the hilighted control */
	virtual ControlBase * doSelect();
	
	//! Add listener for mocap log requests.
	/*! Can't do this from constructor, due to auto-registration that will run at global initialization, possibly before erouter is available */
	virtual void registered() {
		erouter->addListener(&txtmsgSingle,EventBase::textmsgEGID);
	}
	
private:
	MoCapLogger(const MoCapLogger& o); //!< Do not call
	MoCapLogger& operator=(const MoCapLogger& o); //!< Do not call
};

/*! @file
 * @brief Describes MoCapLogger, which provides display and logging of mocap data
 * @author ejt (Creator)
 */

#endif
