//-*-c++-*-
#ifndef INCLUDED_Controller_h
#define INCLUDED_Controller_h

#include "Controls/ControlBase.h"
#include "Behaviors/BehaviorBase.h"
#include "Events/EventBase.h"
#include "Events/EventRouter.h"
#include "Motion/MotionManager.h"
#include "Wireless/Wireless.h"
#include "Wireless/Socket.h"
#include <stack>

//! Handles the menu/command system... when it detects the EmergencyStopMC is activated, it'll kick into high priority.
/*! Keeps track of a command stack.  A Control can designate another sub-control, which will receive events until it finishes\n
 *  Events will then be sent to the parent again.
 *
 *  The GUI uses the same commands as the user (makes it much easier to have only one parser).
 *  The commands are:
 *  - '<tt>!refresh</tt>' - redisplays the current control (handy on first connecting, or when other output has scrolled it off the screen)
 *	- '<tt>!reset</tt>' - return to the root control
 *	- '<tt>!next</tt>' - calls ControlBase::doNextItem() of the current control
 *	- '<tt>!prev</tt>' - calls ControlBase::doPrevItem() of the current control
 *	- '<tt>!select</tt> [<i>item</i>]' - calls ControlBase::doSelect() of the current control, unless <i>item</i> is specified, in which case it is searched for, starting at the root.
 *	- '<tt>!cancel</tt>' - calls current ControlBase::doCancel(), indicates control should cease activity and return to parent (e.g. "Back" button)
 *	- '<tt>!dump_stack</tt>' - requests a dump of the current stack of submenus (useful if the GUI (re)connects and thus current robot state is unknown)
 *  - '<tt>!post </tt><i>generator source type</i> [<i>duration</i>]' - posts an event of your choosing; <i>Generator</i> should be an entry in EventBase::EventGeneratorNames (e.g. timerEGID) or numeric value; <i>source</i> should be a numeric value (unless generator is buttonEGID, in which case it could be an entry from #buttonNames); <i>type</i> can be a numeric value, EventBase::EventTypeNames (e.g. <tt>activate</tt>), or EventBase::EventTypeAbbr (e.g. <tt>A</tt>); <i>duration</i>, if specified, gives the value for EventBase::duration
 *	- '<tt>!msg </tt><i>text</i>' - sends <i>text</i> out as a TextMsgEvent; also note that any text entered on the console port while a GUI is also connected will also be sent as a TextMsgEvent, without needing the !input.
 *  - '<tt>!root </tt><i>text</i>' - calls ControlBase::takeInput(<i>text</i>) on the root control
 *  - '<tt>!hello</tt>' - responds with '<tt>hello\\n</tt><i>count</i>\\n' where <i>count</i> is the number of times '<tt>!hello</tt>' has been sent.  Good for detecting first connection after boot vs. a reconnect.
 *	- '<tt>!hilight</tt> [<i>n1</i> [<i>n2</i> [...]]]' - hilights zero, one, or more items in the menu
 *	- '<tt>!input </tt><i>text</i>' - calls ControlBase::takeInput(text) on the currently hilighted control(s)
 *  - '<tt>!set </tt><i>section</i><tt>.</tt><i>key</i><tt>=</tt><i>value</i>' - will be sent to Config::setValue(<i>section</i>,<i>key</i>,<i>value</i>)
 *  - '<tt>!refreshsketchworld</tt>' - refresh world sketch
 *  - '<tt>!refreshsketchlocal</tt>' - refresh local sketch
 *  - '<tt>!refreshsketchcamera</tt>' - refresh camera sketch
 *	- any text not beginning with '<tt>!</tt>' - sent to ControlBase::takeInput() of the current control
 *
 *  In return, to send the menus to the GUI, the following messages are sent: (newlines are required where shown)
 *  - '<tt>push</tt>' - signals a submenu has been activated
 *  - '<tt>pop</tt>' - signals a submenu has been closed
 *  - '<tt>refresh</tt>\n
 *    <i>text:title</i>\n
 *    <i>int:numitems</i>\n
 *    <i>bool:hasSubmenus</i><sub>1</sub>\n
 *    <i>bool:hilighted</i><sub>1</sub>\n
 *    <i>text:item-title</i><sub>1</sub>\n
 *    <i>text:item-description</i><sub>1</sub>\n
 *    ...\n
 *    <i>bool:hasSubmenus<sub>numitems</sub></i>\n
 *    <i>bool:hilighted<sub>numitems</sub></i>\n
 *    <i>text:item-title<sub>numitems</sub></i>\n
 *    <i>text:item-description<sub>numitems</sub></i>' - refreshes the current menu\n
 *  - '<tt>status</tt>\n
 *    <i>text</i>' - sets the status bar to <i>text</i> (until the next refresh)
 *  - '<tt>load</tt>\n
 *    <i>text:classname</i>\n
 *    <i>text:instancename</i>\n
 *    <i>int:port</i>\n
 *    [<i>arg1</i> [<i>arg2</i> [...]]]' - tells the GUI to load the java class named <i>classname</i>, and have it connect to <i>port</i>, passing it the argument list.
 *    <i>classname</i> should contain a constructor of the form <tt>Classname(String </tt><i>host</i>, <tt>int </tt><i>port</i>, <tt>String </tt><i>args</i><tt>[])</tt>
 *    the argument list is parsed as if it were on the console - unescaped or unquoted spaces will separate args into elements in the array
 *  - '<tt>close</tt>\n
 *    <i>text:instancename</i>' - calls <tt>close()</tt> on an object previously created by a <tt>load</tt> message.
 *    The Java object is expected to contain a function <tt>void close()</tt>.
 *  - '<tt>stack_dump</tt>\n
 *    <i>int:depth</i>\n
 *    <i>text:item-title</i><sub>1</sub>\n
 *    ...\n
 *    <i>text:item-title</i><sub>depth</sub>' - a listing of the current stack, first item is root, last item is current control
 *  - '<tt>goodbye</tt>' - Indicates the connection is about to be closed purposefully, to differentiate from an accidental cut off.
 *  
 *  bool types are expected to be numerical values, 0 for false,
 *  non-zero for true.
 *
 *  <tt>load</tt> and <tt>close</tt> are intended to allow pop-up
 *  windows for custom displays.
 *
 *  The upstream is the responsibility of the individual Controls, but
 *  the protocol is listed here to keep it together.  When a control's
 *  state changes, it's that control's responsiblity to refresh the UI
 *  (LEDs, console, and GUI as appropriate).  Thus, future extensions
 *  to the upstream protocol are between the control which will use it
 *  and the GUI.  Future extensions to the downstream protocol would
 *  involve changing Controller and the GUI.
 *
 *  The Controller may connect to serr in the future to pop-up an alert
 *  anytime output to serr occurs.
 *
 *  Note that all state is maintained on the robot - even if the GUI
 *  is connected, you can still use the buttons to interact with the
 *  controller, and the GUI will update to reflect the changes.  In
 *  HCI (Human Computer Interaction) parlance, this is the MVC,
 *  Model-View-Controller architecture, almost by necessity. (HCI
 *  happens to be my double major when I was an undergrad ;)
 *    
 *  Also, the Controller is responsible for sending out TextMsgEvents
 *  from user input it receives - either a !msg command from the
 *  console or GUI, or <i>any text at all</i> which is received on the
 *  console if there is already a GUI connected.
 *
 *  These TextMsgEvents are always status events, and the duration
 *  field is always 0.
 */
class Controller : public BehaviorBase, public EventTrapper {
public:
	Controller() : BehaviorBase("Controller"), EventTrapper(), display(MotionManager::invalid_MC_ID), estop_id(MotionManager::invalid_MC_ID), root(NULL), cmdstack(), last_time(0), cur_time(0), nextEv_val(0), nextEv_dur(0), prevEv_val(0), prevEv_dur(0), alreadyGotBoth(false), isControlling(false), usesButtons(false), gui_comm(NULL)  {init();}	//!< Constructor
	Controller(ControlBase* r) : BehaviorBase("Controller"), EventTrapper(), display(MotionManager::invalid_MC_ID), estop_id(MotionManager::invalid_MC_ID), root(r), cmdstack(), last_time(0), cur_time(0), nextEv_val(0), nextEv_dur(0), prevEv_val(0), prevEv_dur(0), alreadyGotBoth(false), isControlling(false), usesButtons(false), gui_comm(NULL) { init(); } //!< Constructor, sets a default root control
	virtual ~Controller() {
		delete root;
		theOneController=NULL;
	} //!< Destructor

	//@{
	//! event masks used by processEvent()
	static EventBase nextItem; 
	static EventBase prevItem;
	static EventBase nextItemFast;
	static EventBase prevItemFast;
	static EventBase selectItem;
	static EventBase cancel; //@}

	virtual void doStart(); //!< register for events and resets the cmdstack
	virtual void doStop(); //!< stop listening for events and resets the cmdstack
	virtual void doEvent(); //!< just for e-stop activation/deactivation
	virtual bool trapEvent(const EventBase& e); //!< passes an event to the top control
	
	void reset();   //!< will take the command stack back down to the root
	void refresh(); //!< refreshes the display, for times like sub-control dying, the previous control needs to reset it's display

	void refreshSketchWorld(); //!< refresh world sketches
	void refreshSketchLocal(); //!< refresh local sketches
	void refreshSketchCamera(); //!< refresh camera sketches

	void push(ControlBase* c); //!< puts a new control on top
	void pop();                //!< kills the top control, goes to previous
	ControlBase* top() { return cmdstack.top(); } //!< returns the current control

	Controller& setRoot(ControlBase* r); //!< sets the root level control

	Controller& setEStopID(MotionManager::MC_ID estopid); //!< Sets the emergency stop MC to monitor for pausing
	
	static std::string getClassDescription() { return "Provides interface for activating/deactivating controls (and through them, behaviors)"; }
	virtual std::string getDescription() const { return getClassDescription(); }


	static void loadGUI(const std::string& type, const std::string& name, unsigned int port) {loadGUI(type,name,port,std::vector<std::string>());} //!< attempts to open a Java object on the desktop
	static void loadGUI(const std::string& type, const std::string& name, unsigned int port, const std::vector<std::string>& args); //!< attempts to open a Java object on the desktop
	static void closeGUI(const std::string& name); //!< calls close() on a Java object loaded with loadGUI() (on the desktop)

	static int gui_comm_callback(char *buf, int bytes); //!< called by wireless when there's new data from the GUI
	static int console_callback(char *buf, int bytes);  //!< called by wireless when someone has entered new data on the tekkotsu console (NOT cin)

protected:
	//! calls initButtons with the appropriate button offsets for the host robot model
	void init();

	//! assigns appropriate values to the static event bases
	void initButtons(unsigned fastTime, unsigned downTime, unsigned nextB, unsigned prevB, unsigned nextFastB, unsigned prevFastB, unsigned selectB, unsigned cancelB);

	//! called with each line that's entered on the tekkotsu console or from the GUI
	void takeLine(const std::string& s);
	
	//! sends stack of currently active controls
	void dumpStack();

	//! called with slots (options), a name to lookup; will select the named control
	bool select(ControlBase* item, const std::string& name);
	
	//! sets a config value - some values may require additional processing (done here) to have the new values take effect
	int setConfig(const std::string& str);

	//! maintains top Control
	/*! @param next one of: @li NULL: pop() ::cmdstack @li ::cmdstack.top(): nothing @li other address: ::push(@a next)
	 *  @return true, all the time, for convenience from trapEvent() */
	bool setNext(ControlBase* next);

	//! called when the estop switches on
	/*!  causes the top control to activate, registers for button events */
	void activate();

	//! called when the estop switches off\n
	/*!  causes the top control to deactivate, stops listening for buttons */
	void deactivate();
	
	//! @brief returns true if a valid control is available on the stack
	/*!  if the stack is empty, will push root if it's non-null */
	bool chkCmdStack();

	//! invalid_MC_ID if not active, otherwise id of high priority LEDs
	MotionManager::MC_ID display;

	//! the EmergencyStopMC MC_ID that this Controller is monitoring
	MotionManager::MC_ID estop_id;

	//! the base control, if cmdstack underflows, it will be reset to this
	ControlBase * root;

	/*! @brief the stack of the current control hierarchy\n
	 *  should never contain NULL entries */
	std::stack< ControlBase* > cmdstack;

	//! returns true when the current time and last time are in different periods
	static bool calcPulse(unsigned int t, unsigned int last, unsigned int period) {
		if(period<t-last)
			return true;
		bool nextclock=(t/period)&1;
		bool lastclock=(last/period)&1;
		return (lastclock!=nextclock);
	}


	unsigned int last_time; //!< the time of the last event
	unsigned int cur_time; //!< the time of the current event (do*() can check this instead of calling get_time() )
	float nextEv_val; //!< the magnitude of the last next event (::nextItem)
	unsigned int nextEv_dur; //!< the duration of the last next event (::nextItem)
	float prevEv_val; //!< the magnitude of the last prev event (::prevItem)
	unsigned int prevEv_dur; //!< the duration of the last prev event (::prevItem)
	bool alreadyGotBoth; //!< if doReadStdIn() was already called, but the buttons are both still down
	bool isControlling; //!< true if the Controller is currently active (in the activate()/deactivate() sense, not doStart()/doStop() sense - use isActive() for that...)
	bool usesButtons; //!< true if ControllerGUI knows how to use the buttons for menu navigation, will intercept button presses

	Socket * gui_comm; //!< the socket to listen on for the gui
public:
	static Controller * theOneController; //!< currently can't pull connection socket off of server socket, so only one Controller
	
private:
	Controller(const Controller&); //!< shouldn't be called...
	Controller& operator=(const Controller&); //!< shouldn't be called...
};

/*! @file
 * @brief Describes Controller class, a behavior that should be started whenever the emergency stop goes on to provide menus for robot control
 * @author ejt (Creator)
 */

#endif
