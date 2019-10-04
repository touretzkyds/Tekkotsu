//-*-c++-*-
#ifndef INCLUDED_CLASSNAME_h
#define INCLUDED_CLASSNAME_h

/* Controls provide an interface for user interaction.  Generally this is done
 * in the format of a menu listing, where the user can select one or more items
 * at the same time, and may (or may not) be able to send textual input to these
 * selected items.  Much more information is available in the ControlBase
 * documentation.
 *
 * Early control implementations would completely override the ControlBase
 * implementations and directly encode the ControlBase protocols for interaction
 * with the user's terminal and/or GUI.  This may be more efficient, but it is
 * easier to write and possibly to maintain code that builds/maintains a
 * set of submenus and blindly relies on the ControlBase implementation to
 * manage communication.
 *
 * Using multiple inheritance, a control can also be considered an EventListener
 * or even a full-fledged behavior.  This is completely expected and can be very
 * useful for providing real time feedback or editing capabilties to a user.
 *
 * You are highly encouraged to familiarize yourself with the controls available
 * in Behaviors/Controls in order to compose your interface.
 * 
 * Some methods return a ControlBase*.  These values are interpreted
 * by the Controller as follows:
 *   - 'this' if the control should stay active (default for most)
 *   - 'NULL' to return to parent control
 *   - other address to activate a child control */

// Replace YOURNAMEHERE, CLASSNAME, and DESCRIPTION as appropriate, and go to town!


#include "Behaviors/Controls/ControlBase.h"

//! DESCRIPTION
class CLASSNAME : public ControlBase {

	// **************************** //
	// ******* CONSTRUCTORS ******* //
	// **************************** //
	// Not all of these necessarily make sense to implement... feel free
	// to remove those which don't -- none are required.

public:
	//! default constructor
	CLASSNAME()
		: ControlBase("CLASSNAME","DESCRIPTION")
	{init();}
	//! constructor which allows a custom name
	CLASSNAME(const std::string& n)
		: ControlBase(n,"DESCRIPTION")
	{init();}
	//! constructor which allows a custom name and tooltip
	CLASSNAME(const std::string& n, const std::string& d)
		: ControlBase(n,d)
	{init();}

	//! destructor
	virtual ~CLASSNAME() {}

protected:
	//! initialization
	virtual void init() {
		// If your Control is relatively static, you'll probably want to do a series
		// of pushSlot(...)'s here in order to build up your interface.  See
		// controls available in Behaviors/Controls.
		// Otherwise, see the refresh() function.
	}


	// **************************** //
	// ********* METHODS ********** //
	// **************************** //
	// You *aren't* expected to override all of these functions.
	// Feel free to remove those which you don't use -- none are
	// required, and the ControlBase implementation is fully functional.

public:
	//! Called when the control is activated (or the control system is reactivating)
	/*! Takes the id number of a LedMC which the control should use,
	 *  maintained by Controller.  Controls share the display which is
	 *  passed, and may use the socket @a gui to communicate with the
	 *  GUI controller, although you shouldn't assume it is connected. */
	virtual ControlBase * activate(MotionManager::MC_ID disp_id, Socket * gui) {
		return ControlBase::activate(disp_id,gui);
	}

	//! called when a control is being overriden by a child, or the control system is deactivating (e-stop being turned off)
	virtual void pause() {
		ControlBase::pause();
	}

	//! called when a child has deactivated and this control should refresh its display, or some other event (such as the user pressing the refresh button) has happened to cause a refresh to be needed
	virtual void refresh() {
		// If you are displaying dynamic information, you may wish to call clearSlots()
		// and then rebuild the menus with the current information.
		// Alternatively, if you know which items need to be modified, you can simply
		// change their name before calling the ControlBase::refresh below.
		ControlBase::refresh();
	}
	
	//! called when this control is being popped from the control stack
	virtual void deactivate() {
		ControlBase::deactivate();
	}

	//! when the user has trigger an "open selection"
	/*! default is to return the hilighted control */
	virtual ControlBase * doSelect() {
		return ControlBase::doSelect();
	}

	//! when the user wants to increment the control
	/*! default is to hilight the first non-null slot after the last hilight, and return @c this */
	virtual ControlBase * doNextItem() {
		return ControlBase::doNextItem();
	}

	//! when the user wants to decrement the control
	/*! default is to hilight the last non-null slot before the first hilight, and return @c this */
	virtual ControlBase * doPrevItem() {
		return ControlBase::doPrevItem();
	}

	//! when the user wants to cancel
	/*! you should almost always return NULL now unless you need to override the cancel in order to confirm something (e.g. "Save changes?") */
	virtual ControlBase * doCancel() {
		return ControlBase::doCancel();
	}

	//! prompt the user for text input on the current input device (cin, tekkotsu console (sout), or GUI)
	virtual ControlBase * doReadStdIn(const std::string& prompt=std::string()) {
		return ControlBase::doReadStdIn(prompt);
	}

	//! called when the user has supplied a text string
	/*! May or may not have been prompted by doReadStdIn()!  May not
	 *  even be active yet - the user can direct the same input to a set
	 *  of hilighted menus */
	virtual ControlBase * takeInput(const std::string& msg) {
		return ControlBase::takeInput(msg);
	}

	//! may be called before takeInput to verify this Control can make sense of msg
	virtual bool validInput(const std::string& msg) {
		return ControlBase::validInput(msg);
	}


	// **************************** //
	// ********* MEMBERS ********** //
	// **************************** //
protected:
	// <class members go here>


	// **************************** //
	// ********** OTHER *********** //
	// **************************** //
private:
	CLASSNAME(const CLASSNAME&); //!< you can override, but don't call this...
	CLASSNAME& operator=(const CLASSNAME&); //!< you can override, but don't call this...
};

/*! @file
 * @brief Defines CLASSNAME, which DESCRIPTION
 * @author YOURNAMEHERE (Creator)
 */
#endif
