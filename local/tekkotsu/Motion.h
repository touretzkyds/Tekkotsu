//-*-c++-*-
#ifndef INCLUDED_Motion_h_
#define INCLUDED_Motion_h_

#include "Process.h"
#include "sim.h"
#include "IPC/ProcessID.h"
#include "IPC/SharedObject.h"
#include "SharedGlobals.h"
#include "Motion/MotionManager.h"
#include "Sound/SoundManager.h"
#include "Shared/Profiler.h"
#include "local/EntryPoint.h"
#include <list>

class Motion : public Process {
public:
	//! constructor
	Motion();
	//! destructor
	~Motion();

	virtual void doStart();
	virtual void run();
	virtual void doStop();

	static const char * getClassName() { return "Motion"; }
	static ProcessID::ProcessID_t getID() { return ProcessID::MotionProcess; }
	
	static const char * getMotionCommandID() { return "MotionCommands"; }
	static const char * getMotionOutputID() { return "MotionOutput"; }
	static const char * getMotionOutputPIDsID() { return "MotionOutputPIDs"; }
	static const char * getMotionManagerID() { return "MotionManager"; }
	static const char * getMotionProfilerID() { return "MotionProfiler"; }

protected:
	SharedObject<sim::SoundPlayQueue_t> sounds;
	SharedObject<sim::MotionCommandQueue_t> motions;
	SharedObject<sim::MotionOutput_t> motionout;
	SharedObject<sim::MotionOutputPIDs_t> motionoutpids;
	SharedObject<sim::EventQueue_t> events;
	SharedObject<sim::StatusRequest_t> statusRequest;
	SharedObject<MotionManager> motionmanager;
	SharedObject<SoundManager> soundmanager;
	SharedObject<sim::MotionWakeup_t> motionWakeup;
	SharedObject<motionProfiler_t> motionProf;
	
	IPCEventTranslator * etrans;
	class MessageReceiver * statusrecv;
	class MessageReceiver * wakeuprecv;
	class MotionExecThread * motionExec;
	class IPCMotionHook * motionfwd;
	WirelessThread wireless_thread;

	EntryPoint motionLock;
	
	static bool gotWakeup(RCRegion* msg);
	
private:
	Motion(const Motion&); //!< don't call (copy constructor)
	Motion& operator=(const Motion&); //!< don't call (assignment operator)
};

/*! @file
 * @brief Defines Motion, which DESCRIPTION
 * @author ejt (Creator)
 */

#endif
