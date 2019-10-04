//-*-c++-*-
#ifndef INCLUDED_CLASSNAME_h_
#define INCLUDED_CLASSNAME_h_

#include "IPC/MutexLock.h"
#include "IPC/SemaphoreManager.h"
#include "IPC/ProcessID.h"
#include "Shared/plist.h"
#include "Shared/TimeET.h"
#include "Shared/RobotInfo.h"
#include "local/DataSource.h"

//! A class to hold various simulator parameters which need to be accessed from multiple processes
class SharedGlobals {
public:
	//! constructor
	SharedGlobals()
		: waitForSensors(false), simulatorTime(0), timeScale(1), motion(), sensors(), vision(), lock(), sensorState(), 
		nextTimer(-1U), nextMotion(-1U), nextSensorUpdate(-1U), bootTime(), timeOffset(0), lastTimeScale(0), autoPauseTime(-1U),
			semgr(2), running(semgr.getSemaphore()), sensorValid(semgr.getSemaphore())
	{
		sensorState.motionOverride = &motion.override;
		for(unsigned int i=0; i<NUM_RUNLEVELS; i++)
			level_count[i]=0;
		semgr.raise(running,1);
		semgr.raise(sensorValid,1);
	}
	//! destructor
	~SharedGlobals() {
		semgr.releaseSemaphore(running);
		semgr.releaseSemaphore(sensorValid);
	}
	
	//       ****************
	//!@name Startup Control
	//       ****************
	
	//! Controls whether to wait for initial sensor readings before triggering the startup behavior or starting the motion polling thread.
	/*! This can avoid jumping to the 0-point on simulator launch.  Changes after initial launch are ignored. */
	plist::Primitive<bool> waitForSensors;
	
	//! Called by Main when the first sensorEGID event is generated.
	/*! When the waitForSensors setting is enabled, the startupBehavior is not activated until this occurs */
	void signalHaveSensors() {
		semgr.lower(sensorValid,1,false);
	}
	//! test to see if the initial sensor event has been generated
	bool haveSensors() const {
		return semgr.testZero(sensorValid,false);
	}
	//! blocks until the initial sensor event has been generated
	void waitSensors() {
		semgr.testZero(sensorValid,true);
	}
	
	//@}
	
	
	//       ****************
	//!@name Shutdown Control
	//       ****************

	//! call this to cause "system shutdown" -- clean halt of the simulator (not actually the host system)
	void signalShutdown() {
		semgr.setValue(running,0);
		if(waitForSensors && !haveSensors())
			signalHaveSensors(); // break out of deadlock in Main and Motion if we were waiting for the first sensor
	}
	//! test to see if the shutdown flag has been set (non-blocking)
	bool isShutdown() const {
		return semgr.testZero(running,false);
	}
	//! blocks until shutdown flag has been set
	bool waitShutdown() {
		return semgr.testZero(running,true);
	}
	
	//! access to #semgr, returns SemaphoreManager::hadFault()
	bool hadFault() const { return semgr.hadFault(); }
	
	//! access to #semgr's SemaphoreManager::faultShutdown() -- call this *after* a fault has occured from the signal handler; doesn't signal the fault itself
	void faultShutdown() { semgr.faultShutdown(); }
	
	//@}


	//       ************
	//!@name Time Control
	//       ************

	//! returns the current simulator time, in milliseconds since startup
	/*! the simulator should set project_get_time::get_time_callback to call this,
	 *  so calls to ::get_time() will be forwarded here.  That wall all processes
	 *  will share the same time */
	unsigned int get_time();
	
	//! returns the current simulator #timeScale (speed factor), as a ratio of real time (e.g. '2' means simulation is running two times wall clock)
	/*! the simulator should set project_get_time::get_timeScale_callback to call this,
	 *  so calls to ::getTimeScale() will be forwarded here. */
	float getTimeScale() const;
	
	//! the current time within the simulation, only applicable when timeScale is negative (non-realtime)
	unsigned int simulatorTime;
	
	//! Controls the speed at which time from get_time() will move
	/*! You can use this to pretend your hardware is faster or slower
	 *  than it actually is.  For instance, a value of .5 means time
	 *  will move at half speed (pretending your hardware is twice as
	 *  fast)  This can be useful for "slow motion" analysis, or you
	 *  can speed up time to simulate a more processor-restrictive platform.
	 *
	 *  Negative values indicate full-speed processing -- time will be
	 *  incremented only as quickly as it can be without dropping any
	 *  video or sensor frames. (may be faster or slower than realtime)
	 *  in this case, #simulatorTime is used by calls to get_time()
	 *
	 *  A value of zero halts time. */
	plist::Primitive<double> timeScale;

	//! sets #autoPauseTime
	void setAutoPauseTime(unsigned int t) { autoPauseTime=t; }
	//! returns #autoPauseTime
	unsigned int getAutoPauseTime() const { return autoPauseTime; }
	
	//@}


	//       **********************
	//!@name Runlevel Communication
	//       **********************

	//! defines the runlevels that each process passes through; runlevels should monotonically increase (can't go backwards)
	enum runlevel_t {
		CREATED=0,    //!< corresponding element of #level_count is incremented prior to each fork -- not strictly a runlevel per se
		CONSTRUCTING, //!< currently initializing
		STARTING,     //!< setting up shared memory regions with other processes
		RUNNING,      //!< full activity, stay here until the #running semaphore is set to 0
		STOPPING,     //!< dereferencing shared regions, waiting for threads to finish
		DESTRUCTING,  //!< destructors are in progress
		DESTRUCTED,   //!< destruction has completed, corresponding element of #level_count is incremented immediately prior to process completion
	};
	static const unsigned int NUM_RUNLEVELS=DESTRUCTED+1; //!< symbolic access to the total number of runlevel stages

	//! string versions of runlevel_t for runtime user-feedback
	static const char * const runlevel_names[NUM_RUNLEVELS+1];

	//! a count of the number of processes which have passed through each runlevel
	unsigned int level_count[NUM_RUNLEVELS];

	//@}


	//       **********************
	//!@name Configuration Parameters
	//       **********************

	//! holds configuration parameters for the motion process
	class MotionSimConfig : public virtual plist::Dictionary {
	public:
		//! constructor
		MotionSimConfig() : plist::Dictionary(), verbose(1), feedbackDelay(0),
			zeroPIDFeedback(false), /*speedLimit(0),*/ feedbackRangeLimits(true), override(false), startPose(), frameNumber(-1U)
		{
			setLoadSavePolicy(FIXED,SYNC);
			addEntry("Verbose",verbose,"Report whenever motion commands are being processed or joints are updated\n0 - nothing, 1 - errors, 2 - warnings (e.g. dropped frames), 3 - notification every frame");
			addEntry("FeedbackDelay",feedbackDelay,"Delay (in milliseconds) to apply to motion output before feeding back to sensor values (simulates (very roughly) inertia and system response time); 0 indicates instantaneous/perfect joint control, negative values indicate no feedback (only sensor data sets joint positions)");
			addEntry("ZeroPIDFeedback",zeroPIDFeedback,"When set to false, if PIDs are set to zero, then sensor values are used to set joint positions; otherwise joint position sensors would only be used if FeedbackDelay is negative");
			//addEntry("EnforceSpeedLimit",speedLimit,"The simulated motion of joints is limited to this factor of model's recommended speed limits.  0 (or negative) disables speed limit altogether.");
			addEntry("FeedbackRangeLimits",feedbackRangeLimits,"If true, feedback will be limited to the RobotInfo::mechnicalLimits values");
			addEntry("OverrideSensors",override,"Allows motion feedback to override position values from sensor data loaded from disk.\nIf false, feedback is only provided when no other sensor data is being provided");
			addEntry("StartPose",startPose,"Name of a posture file to load as the initial output values before MotionHooks or Behaviors are activated (if empty, everything will be 0)");
		}
		plist::Primitive<int> verbose; //!< Report whenever motion commands are being processed or joints are updated; 0 - nothing, 1 - errors, 2 - warnings (e.g. dropped frames), 3 - notification every frame
		plist::Primitive<int> feedbackDelay; //!< Delay (in milliseconds) to apply to motion output before feeding back to sensor values (simulates (very roughly) inertia and system response time); 0 indicates instantaneous/perfect joint control, negative values indicate no feedback (only sensor data sets joint positions)
		plist::Primitive<bool> zeroPIDFeedback; //!< When set to false, if PIDs are set to zero, then sensor values are used to set joint positions; otherwise joint position sensors would only be used if FeedbackDelay is negative
		//plist::Primitive<float> speedLimit; //!< The simulated motion of joints is limited to this factor of model's recommended speed limits.  0 (or negative) disables speed limit altogether.
		plist::Primitive<bool> feedbackRangeLimits; //!< If true, feedback will be limited to the RobotInfo::mechnicalLimits values
		plist::Primitive<bool> override; //!< Allows motion feedback to override position values from sensor data loaded from disk; if false, feedback is only provided when no other sensor data is being provided
		plist::Primitive<std::string> startPose; //!< Name of a posture file to load as the initial output values before MotionHooks or Behaviors are activated (if empty, everything will be 0)
		unsigned int frameNumber; //!< a monotonically increasing count of the number of sensor frames which have been "completed".  Needed to allow coordination between sensor loading from disk and feedback from motion.  Count is increased by the simulator process, which will send a heartbeat message over Simulator::sensorQueue when it does so.
	} motion;
	
	class StreamSimConfig : public virtual plist::Dictionary {
	public:
		StreamSimConfig() : plist::Dictionary(),
			framerate(1000.f/(FrameTime*NumFrames)), verbose(0), heartbeat(false), sources()
		{
			addEntry("Framerate",framerate,"The rate at which data should be loaded.  This is a hint to the hardware devices, which generally use their 'native' framerate, but may use this to limit data flow.");
			addEntry("Verbose",verbose,"Controls how much feedback to give on the console regarding progress\n  0 - none\n  1 - report when frames are dropped\n  2 - also report when a frame is sent\n  3 - also report when heartbeat is sent/dropped\n  4 - also report when each frame is received and processed");
			addEntry("Heartbeat",heartbeat,"If enabled, an empty \"heartbeat\" message is sent at the appropriate framerate, even if no data is being processed (i.e. frozen, no data loaded, or out of frames); this will cause an update event within the simulator, repeating processing on the previous data.");
			addEntry("Sources",sources,"Indicates which data sources should be activated at launch");
			setLoadSavePolicy(FIXED,SYNC);
		}
		
		//! frames per second to send -- this is only a suggestion to hardware devices, which generally use their 'native' framerate, but may use this to limit data flow
		plist::Primitive<float> framerate;
		
		//! Controls how much feedback to give on the console regarding progress
		/*! 0 - none\n
		 *  1 - report when frames are dropped\n
		 *  2 - also report when a frame is sent\n
		 *  3 - also report when heartbeat is sent/dropped\n
		 *  4 - also report when each frame is received and processed */
		 plist::Primitive<int> verbose;
		
		//! if enabled, an empty "heartbeat" message is sent at the appropriate framerate, even if no data is being processed (i.e. no data loaded or out of frames); this will cause an update event within the simulator, repeating processing on the previous data.
		plist::Primitive<bool> heartbeat;

		//! list of names of DataSources which are to be activated at launch
		plist::ArrayOf<plist::Primitive<std::string> > sources;
	};
	StreamSimConfig sensors;
	StreamSimConfig vision;
	
	//@}
	
	//! allows mutually exclusive access to the fields of SharedObject
	MutexLock<ProcessID::NumProcesses> lock;

	//! holds the host system's process ID for each simulator process
	pid_t pids[ProcessID::NumProcesses];

	//! maximum storage size of strings in #processNames
	static const unsigned int MAX_PROCESS_NAME_LEN=32;

	//! each process should set a string version of its name for user feedback
	char processNames[ProcessID::NumProcesses][MAX_PROCESS_NAME_LEN];
	
	bool setNextTimer(unsigned int t) { if(nextTimer==t) return false; nextTimer=t; return true; } //!< sets #nextTimer, returns true if the new value differs from previous value
	unsigned int getNextTimer() { return nextTimer; } //!< gets #nextTimer
	
	void setNextMotion(unsigned int t) { nextMotion=t; } //!< sets #nextMotion
	unsigned int getNextMotion() { return nextMotion; } //!< gets #nextMotion
	
	void setNextSensorUpdate(unsigned int t) { nextSensorUpdate=t; } //!< sets #nextSensorUpdate
	unsigned int getNextSensorUpdate() { return nextSensorUpdate; } //!< gets #nextSensorUpdate
	
	void resetBootTime() { timeOffset=bootTime.Age().Value()*timeScale*1000; simulatorTime=0; }
	
	SensorState sensorState;
	
protected:
	//! this returns time since boot (#bootTime), scaled by @a scale, relative to #timeOffset
	unsigned int get_real_time(double scale) const {
		return static_cast<unsigned int>(bootTime.Age().Value()*scale*1000-timeOffset);
	}

	//! set by setNextTimer, called with the current value of EventRouter::getNextTimer() after each user code section, indicates time of next timer event
	unsigned int nextTimer;
	
	//! updated by Motion process after each motion update
	unsigned int nextMotion;
	
	//! updated by Main process after each sensor update
	unsigned int nextSensorUpdate;
	
	//! real time since simulator startup (or, at least, since SharedGlobals was constructed... close enough)
	TimeET bootTime; 

	//! the scaled value of #bootTime at which isRealTime was last activated, allows you to start and stop realtime fluidly
	double timeOffset; 
	
	//! updated by each call to get_time(), if timeScale differs, allows timeOffset to be updated fluidly
	double lastTimeScale;
	
	//! if simulatorTime is about to move past this value, timeScale is set to 0 instead, and simulatorTime is set to this
	unsigned int autoPauseTime;
	
	SemaphoreManager semgr; //!< a semaphore set, only used for #running and #sensorValid
	SemaphoreManager::semid_t running; //!< the semaphore within #semgr to communicate shutdown status between processes -- when the semaphore is set to 0, shutdown is requested
	SemaphoreManager::semid_t sensorValid; //!< the semaphore within #semgr to notify processes when the first sensor frame is available
};

const unsigned int MAX_SUBJECTS=50; //!< maximum number of message queues the simulator can maintain
const unsigned int MAX_SUBJECT_NAME=50; //!< maximum storage capacity of subject names

// just a forward definition of RegionRegistry
template<unsigned int MAX_SUBJECTS, unsigned int MAX_SUBJECT_NAME> class RegionRegistry;
//! the type to use for the inter-process communication registry
typedef RegionRegistry<MAX_SUBJECTS,MAX_SUBJECT_NAME> ipc_setup_t;

extern ipc_setup_t * ipc_setup; //!< a global pointer to the inter-process message queue registry (a RegionRegistry)
extern SharedGlobals * globals; //!< a global pointer to the SharedGlobals instance
extern float getTimeScale(); //!< a prototype for accessing current time scale without referencing ::globals directly

/*! @file
 * @brief A class to hold various simulator parameters which need to be accessed from multiple processes
 * @author ejt (Creator)
 */

#endif
