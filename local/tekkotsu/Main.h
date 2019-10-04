//-*-c++-*-
#ifndef INCLUDED_Main_h_
#define INCLUDED_Main_h_

#include "Process.h"
#include "sim.h"
#include "IPC/SharedObject.h"
#include "SharedGlobals.h"
#include "Motion/MotionManager.h"
#include "Sound/SoundManager.h"
#include "local/EntryPoint.h"
#include "Vision/BufferedImageGenerator.h"
#include "Shared/Profiler.h"
#include "Events/EventListener.h"
#include "Shared/WorldState.h"

class Main : public Process, public virtual EventListener {
public:
	//! constructor
	Main();
	//! destructor
	~Main();

	virtual void doStart();
	virtual void run();
	virtual void doStop();
	
	//! listens for a sensor update event, then calls SharedGlobals::signalHaveSensors()
	virtual void processEvent(const EventBase& event);

	static const char* getClassName() { return "Main"; }
	static ProcessID::ProcessID_t getID() { return ProcessID::MainProcess; }
	
	static const char * getEventsID() { return "MainEvents"; }
	
protected:
	SharedObject<sim::SoundPlayQueue_t> sounds;
	SharedObject<sim::MotionCommandQueue_t> motions;
	SharedObject<sim::EventQueue_t> events;
	SharedObject<sim::CameraQueue_t> cameraFrames;
	SharedObject<sim::SensorQueue_t> sensorFrames;
	SharedObject<sim::TimerWakeup_t> timerWakeup;
	SharedObject<sim::StatusRequest_t> statusRequest;
	SharedObject<MotionManager> motionmanager;
	SharedObject<SoundManager> soundmanager;
	SharedObject<motionProfiler_t> motionProf;
	SharedObject<soundProfiler_t> soundProf;
	
	class MessageReceiver * visrecv;
	class MessageReceiver * sensrecv;
	class MessageReceiver * evtrecv;
	class MessageReceiver * timerrecv;
	class MessageReceiver * statusrecv;
	class TimerExecThread * timerExec;
	bool visionRead;
	WirelessThread wireless_thread;
	WorldState worldStateCache; //!< used for temporary storage when parsing sensor frames, allocation here avoids rather costly constructor call on every sensor frame
	EntryPoint behaviorLock;

	static bool gotCamera(RCRegion* msg);
	static bool gotSensors(RCRegion* msg);
	static bool gotEvent(RCRegion* msg);
	static bool gotTimer(RCRegion* msg);
	void gotThreadedEvent(EventBase* evt);
	
	RCRegion * curimgregion;
	BufferedImageGenerator::ImageSource img; //!< root data source for vision stream, references data in #curimgregion
	unsigned int lastVisionSN; //!< serial number of last camera frame, just so we can give warning message when we drop (if verbose is set)
	
private:
	Main(const Main&); //!< don't call (copy constructor)
	Main& operator=(const Main&); //!< don't call (assignment operator)
};

/*! @file
 * @brief Defines Main, which DESCRIPTION
 * @author ejt (Creator)
 */

#endif
