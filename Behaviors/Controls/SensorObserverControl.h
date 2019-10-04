//-*-c++-*-
#ifndef INCLUDED_SensorObserverControl_h_
#define INCLUDED_SensorObserverControl_h_

#include "ControlBase.h"
#include "Events/EventListener.h"
#include "StringInputControl.h"
#include "ToggleControl.h"
#include <fstream>

//! allows logging of sensor information to the console or file
class SensorObserverControl : public ControlBase, public EventListener {
public:
	//!constructor
	SensorObserverControl();

	//!opens a custom (embedded) menu to toggle individual sensors
	virtual ControlBase* doSelect();
	
	virtual void refresh();

	//!sends all events received to stdout and/or logfile
	virtual void processEvent(const EventBase& event);

protected:
	//!checks to see if logfilePath differs from the StringInputControl's value and switches it if it is
	void checkLogFile();

	//!update the real-time sub-control view (#rtCtl)
	void updateRT();

	//! The real time view for SensorObserverControl is split into a separate class for more straightfoward handling of refreshes
	class RTViewControl : public ControlBase, public EventListener {
	public:
		//!constructor, pass pointer to SensorObserverControl which contains it
		RTViewControl(SensorObserverControl* p) : ControlBase("Real-time View"), EventListener(), parent(p), period(500) {}
		virtual void processEvent(const EventBase& /*event*/) { refresh(); } //!< causes control to refresh whenever an event (i.e. timer) is received
		virtual void refresh();
		virtual void pause();
		virtual void deactivate();
		virtual void setPeriod(unsigned int x); //!< sets #period
	protected:
		SensorObserverControl* parent; //!< a back pointer to SensorObserverControl which contains it so updateRT can be triggered
		unsigned int period; //!< the time to wait between automatic refreshes
	private:
		RTViewControl(const RTViewControl& ); //!< don't call
		RTViewControl& operator=(const RTViewControl& ); //!< don't call
	};
	friend class RTViewControl;
	
	//!address of the logfile, if any (empty string is no logfile)
	std::string logfilePath;

	//!if a filename is given, events are logged to here
	std::ofstream logfile;

	ControlBase * helpCtl; //!< control containing help info
	ControlBase * sensorCtl; //!< control of sensor selectors
	ControlBase * buttonCtl; //!< control of buttons selectors
	ControlBase * outputCtl; //!< control of outputs selectors
	ControlBase * dutyCtl; //!< control of duty selectors
	ToggleControl * consoleCtl; //!< turn logging to the console on and off
	StringInputControl * fileCtl; //!< turn logging to a file on and off
	RTViewControl * rtCtl; //!< a submenu containing real-time view of current values
	StringInputControl * rtFreqCtl; //!< the frequency at which rtCtl should be updated

	unsigned int numListeners; //!< count of active console or file control so we know if we're actually logging

private:
	SensorObserverControl(const SensorObserverControl& ); //!< don't call
	SensorObserverControl& operator=(const SensorObserverControl& ); //!< don't call

};

/*! @file
 * @brief Describes SensorObserverControl which allows logging of sensor information to the console or file
 * @author ejt (Creator)
 */

#endif
