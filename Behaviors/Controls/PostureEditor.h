//-*-c++-*-
#ifndef INCLUDED_PostureEditor_h_
#define INCLUDED_PostureEditor_h_

#include "ControlBase.h"
#include "Motion/PostureEngine.h"
#include "Events/EventListener.h"

//! allows logging of events to the console or a file
class PostureEditor : public ControlBase, public EventListener {
public:
	//! constructor
	explicit PostureEditor();

	//! destructor
	virtual ~PostureEditor();

	virtual ControlBase * activate(MC_ID disp_id, Socket * gui);
	virtual void refresh(); //!< if we're back from a child slot, it's either load or save, so we need to handle it
	virtual void pause();
	virtual void deactivate();

	//! listens for the EStop to be turned off before moving
	virtual void processEvent(const EventBase& e);
	
protected:
	PostureEngine pose; //!< the current target posture
	MC_ID reachID; //!< id of motion sequence used to slow "snapping" to positions

	class FileInputControl* loadPose; //!< the control for loading postures
	class NullControl* disabledLoadPose; //!< a message to display instead of loadPose when EStop is on
	class StringInputControl* savePose; //!< the control for saving postures
	bool pauseCalled; //!< true if refresh hasn't been called since pause
	
	static const unsigned int moveTime=1500; //!< number of milliseconds to take to load a posture - individual joint changes will be done in half the time
	
	bool isEStopped(); //!< called to check status of estop
	void updatePose(unsigned int delay); //!< called anytime pose is modified; uses #reachID to move to #pose if estop is off

private:
	PostureEditor(const PostureEditor& ); //!< don't call
	PostureEditor& operator=(const PostureEditor& ); //!< don't call
};

/*! @file
 * @brief Describes PostureEditor, which allows numeric control of joints and LEDs
 * @author ejt (Creator)
 */

#endif
