//-*-c++-*-
#ifndef INCLUDED_WaypointWalkControl_h_
#define INCLUDED_WaypointWalkControl_h_

#include "ControlBase.h"
#include "IPC/SharedObject.h"
#include "Motion/WaypointEngine.h"

class NullControl;
class ToggleControl;
class FileInputControl;
class StringInputControl;

//! Allows interactive control and execution of a set of waypoints 
class WaypointWalkControl : public ControlBase {
public:
	//!constructor
	WaypointWalkControl();

	//! add #walk_id to ::motman
	virtual ControlBase * activate(MC_ID disp_id, Socket * gui);

	//!will be called after each waypoint is reached so we can update the menu
	virtual void refresh();

	//! remove the #walk_id from ::motman
	virtual void deactivate(); 

	//!handles selection of menu items
	virtual ControlBase* doSelect();
	
	// sends all events received to stdout and/or logfile
	//virtual void processEvent(const EventBase& event);

	//! handles editing of individual waypoints
	class WaypointEditControl : public ControlBase {
	public:
		//!constructor
		WaypointEditControl(const std::string& n, const std::string& d, MC_ID walkid, WaypointEngine::WaypointListIter_t waypointid);
		//!handles selection of menu items
		virtual ControlBase* doSelect();
	protected:
		MC_ID walk_id; //!< id number of the WaypointWalk
		WaypointEngine::WaypointListIter_t waypoint_id; //!< id of the waypoint this is editing
		NullControl * up;  //!< command to move up in list
		NullControl * down;//!< command to move down in list
		NullControl * del; //!< command to delete from list
		NullControl * set; //!< command to start targeting this location
	private:
		WaypointEditControl(const WaypointEditControl&); //!< don't call
		WaypointEditControl operator=(const WaypointEditControl&); //!< don't call
	};

protected:
	bool isRunning; //!< true if #walk_id is currently running
	NullControl * startstopCtl; //!< start and stop waypoint running
	ToggleControl * loopCtl; //!< repeat waypoints
	NullControl * addEgoWPCtl; //!< start and stop waypoint running
	NullControl * addOffWPCtl; //!< start and stop waypoint running
	NullControl * addAbsWPCtl; //!< start and stop waypoint running
	FileInputControl * loadCtl; //!< allows loading of a path
	StringInputControl * saveCtl; //!< save a path to a file
	StringInputControl * localizationCtl; //!< enter localization updates manually

	unsigned int listOffset; //!< the index of the first waypoint in the menu

	MC_ID walk_id; //!< id number of the walk we're using, so we can check it out before modifying it

private:
	WaypointWalkControl(const WaypointWalkControl&); //!< don't call
	WaypointWalkControl operator=(const WaypointWalkControl&); //!< don't call
};

/*! @file
 * @brief Describes WaypointWalkControl, which allows interactive control and execution of a set of waypoints 
 * @author ejt (Creator)
 */

#endif
