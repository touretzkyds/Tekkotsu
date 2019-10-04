//-*-c++-*-
#ifndef INCLUDED_Simulator_h_
#define INCLUDED_Simulator_h_

#include "Process.h"
#include "sim.h"
#include "IPC/SharedObject.h"
#include "IPC/MessageQueueStatusThread.h"
#include "Sound/SoundManager.h"
#include "SharedGlobals.h"
#include "Shared/plist.h"
#include "Shared/RobotInfo.h"
#include "Shared/debuget.h"
#include "local/MotionHook.h"
#include "IPC/CallbackThread.h"
#include "IPC/PollThread.h"
#include <set>

class MessageReceiver;
class DeviceDriver;

class Simulator : public Process,  public plist::PrimitiveListener, public plist::CollectionListener, public MessageQueueStatusThread::StatusListener {
public:
	//! constructor
	Simulator();
	
	~Simulator();

	virtual void doStart();
	virtual void doStop();
	virtual void run();
	
	static const char * getClassName() { return "Simulator"; }
	static ProcessID::ProcessID_t getID() { return ProcessID::SimulatorProcess; }
	
	static const char * getCameraQueueID() { return "CameraData"; }
	static const char * getSensorQueueID() { return "SensorData"; }
	static const char * getTimerWakeupID() { return "TimerWakeup"; }
	static const char * getMotionWakeupID() { return "MotionWakeup"; }
	static const char * getStatusRequestID() { return "StatusRequest"; }
	static const char * getCommandQueueID() { return "CommandQueue"; }
	
	virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	virtual void plistCollectionEntryAdded(plist::Collection& col, plist::ObjectBase& primitive);
	virtual void plistCollectionEntryRemoved(plist::Collection& col, plist::ObjectBase& primitive);
	virtual void plistCollectionEntriesChanged(plist::Collection& col);
		
	virtual void messagesRead(MessageQueueBase& mq, unsigned int n);
	
	static void sendCommand(const std::string& cmd);
	
	//! Registers the MotionHook to have its motionCheck() called following each motion update.
	/*! Responsibility for memory (de)allocation is NOT assumed by registration. */
	static void registerMotionHook(MotionHook& h) {
		MarkScope l(theSim ? dynamic_cast<Resource&>(theSim->simLock) : ::emptyResource);
		theSim->motionHooks.insert(&h);
	}
	
	//! Removes the MotionHook from the list, no longer receives any notifications
	/*! Responsibility for memory (de)allocation is NOT assumed by registration, so this doesn't affect @a h directly. */
	static void deregisterMotionHook(MotionHook& h) {
		MarkScope l(theSim ? dynamic_cast<Resource&>(theSim->simLock) : ::emptyResource);
		theSim->motionHooks.erase(&h);
	}
	
	static void clearMotionHooks() {
		MarkScope l(theSim ? dynamic_cast<Resource&>(theSim->simLock) : ::emptyResource);
		theSim->motionHooks.clear();
	}
	
	static void setMotionStarting();
	static void setMotionStopping();
	static void updateMotion(const float outputs[][NumOutputs]);
	static void updatePIDs(const std::vector<MotionHook::PIDUpdate>& pids);
	static void setMotionLeavingRealtime(bool isFullSpeed);
	static void setMotionEnteringRealtime();
	
	class MotionMonitorThread : public Thread {
	public:
		MotionMonitorThread() : Thread(), curHook(NULL), curFuncName(NULL), motionCheckTime() { start(); }
		MotionHook * curHook;
		const char * curFuncName;
		TimeET motionCheckTime;
	protected:
		static const TimeET timeout;
		virtual unsigned int runloop();
	private:
		MotionMonitorThread(const MotionMonitorThread&);  //!< dummy
		MotionMonitorThread& operator=(const MotionMonitorThread&);  //!< dummy
	};
	static MotionMonitorThread * motionHookMonitor;
	
protected:
	static void replaceEntry(const std::string& name, plist::Dictionary& d, const std::string& comment);
	
	//! subscribed by Simulator to message queue's send; ensures that SharedGlobals::MotionSimConfig::frameNumber stays in sync with the message serial number
	class FrameCounter : public MessageQueueBase::MessageFilter {
	public:
		virtual bool filterSendRequest(RCRegion* /*rcr*/) {
			globals->motion.frameNumber++;
			//ASSERT(globals->motion.frameNumber==LoadFileThread::recoverSN(rcr),"globals->motion.frameNumber does not match frame number sent! (" << globals->motion.frameNumber << " vs " << LoadDataThread::recoverSN(rcr));
			return true;
		}
	} frameCounter;
	
	class CommandThread : public Thread {
	public:
		CommandThread() : Thread(), initThreadsLock(), initThreads() {}
		~CommandThread();
		virtual void * run();
		virtual void runInitThread(Thread& th);
		virtual void abortInitThreads();
	protected:
		Thread::Lock initThreadsLock;
		std::set<Thread*> initThreads;
	private:
		CommandThread(const CommandThread&); //!< do not call
		CommandThread& operator=(const CommandThread&); //!< do not call
	} cmdThread;
	
	//! blocks until the thread is either aborted by the user via cmdThread (i.e. shutdown) or completes successfully
	template<typename T> void abortable(const T& fn) {
		CallbackThread th(fn,false);
		cmdThread.runInitThread(th);
	}
	
	//! blocks until the thread is either aborted by the user via cmdThread (i.e. shutdown) or completes successfully
	template<typename T,typename C> void abortable(const T& fn, C& userdata) {
		CallbackThread th(fn,userdata,false);
		cmdThread.runInitThread(th);
	}
	
	//! blocks until the thread is either aborted by the user via cmdThread (i.e. shutdown) or completes successfully
	template<typename T,typename C> void abortable(const T& fn, const C& userdata) {
		CallbackThread th(fn,userdata,false);
		cmdThread.runInitThread(th);
	}
	
	class SyncDataThread : virtual public plist::PrimitiveListener, public PollThread {
	public:
		typedef bool (Simulator::*callback_t)(bool);
		SyncDataThread(Simulator* p, callback_t func) : plist::PrimitiveListener(), PollThread(0.f, 1.0/globals->sensors.framerate, true), process(p), callback(func) {}
		void sync() { (process->*callback)(true); }
	protected:
		virtual bool launched() {
			interrupted();
			globals->sensors.framerate.addPrimitiveListener(this);
			globals->timeScale.addPrimitiveListener(this);
			return PollThread::launched();
		}
		virtual void cancelled() {
			globals->sensors.framerate.removePrimitiveListener(this);
			globals->timeScale.removePrimitiveListener(this);
		}
		virtual bool poll() { (process->*callback)(false); return true; }
		virtual void interrupted() { delay = period = (1.0/globals->sensors.framerate) / globals->timeScale; }
		virtual void plistValueChanged(const plist::PrimitiveBase&) { interrupt(); }
		Simulator* process;
		callback_t callback;
	private:
		SyncDataThread(const SyncDataThread&);
		SyncDataThread& operator=(const SyncDataThread&);
	};
	SyncDataThread sensorThread;
	bool sendSensorSent;

	bool sendSensor(bool syncCall);
	unsigned int nextVisionTime();
	unsigned int nextSensorTime();
	//! this is registered as a callback with DataSource::resourceSync so we send notifications immediately following updates, cutting latency
	static void syncSensors() { theSim->sensorThread.sync(); }
	
	enum step_t {
		STEP_NONE,
		STEP_CAMERA,
		STEP_SENSOR,
		STEP_TIMER,
		STEP_MOTION
	};
	
	void incrementTime();
	void sendTimerWakeup();
	unsigned int getNextFrame();
	void resetSpeedMode();
	
	void processRunlevel(SharedGlobals::runlevel_t curRunLevel);
	bool processCommand(const std::string& line, bool addToHistory); //!< process an individual command
	static bool gotCommand(RCRegion* msg); //!< for commands coming from other processes via #commandQueue and #commandrecv
	static bool gotMotion(RCRegion* msg); //!< when running in multi-process mode, receives output value updates from motion process
	static bool gotMotionPIDs(RCRegion* msg); //!< when running in multi-process mode, receives output PID updates from motion process
	
	typedef void (DeviceDriver::*getDataSources_t)(std::map<std::string,DataSource*>&);
	void updateDataSources(std::set<std::string>& active, std::set<DataSource*>& activeSrcs, const plist::ArrayOf<plist::Primitive<std::string> >& requested, getDataSources_t getDataSources);
	void lookupDataSource(const std::string& name, getDataSources_t getDataSources, std::set<DataSource*>& dataSrcs, std::string& errStr);
	std::string lookupDataSourceName(const DataSource* src) const;
	
	void cmdQuit(const std::vector<std::string>& args);
	void cmdLoad(const std::vector<std::string>& args);
	void cmdSave(const std::vector<std::string>& args);
	void cmdRunlevel(const std::vector<std::string>& args);
	bool cmdPrint(const std::vector<std::string>& args);
	bool cmdSet(const std::vector<std::string>& args);
	void cmdRun(const std::vector<std::string>& args, bool isRelative);
	void cmdRun(const std::vector<std::string>& args);
	void cmdPause(const std::vector<std::string>& args);
	void cmdHelp(const std::vector<std::string>& args);
	void cmdStep(const std::vector<std::string>& args);
	void cmdStatus(const std::vector<std::string>& args);
	void cmdAdvance(const std::vector<std::string>& args);
	void cmdFreeze(bool v, const std::vector<std::string>& args);
	void cmdReset(const std::vector<std::string>& args);
	void cmdNew(const std::vector<std::string>& args);
	void cmdDelete(const std::vector<std::string>& args);
	void cmdPost(const std::vector<std::string>& args);
	void cmdMsg(const std::vector<std::string>& args);
	
	SharedObject<sim::CameraQueue_t> cameraQueue;
	SharedObject<sim::SensorQueue_t> sensorQueue;
	SharedObject<sim::TimerWakeup_t> timerWakeup;
	SharedObject<sim::MotionWakeup_t> motionWakeup;
	SharedObject<sim::StatusRequest_t> statusRequest;
	SharedObject<SoundManager> soundmanager;
	SharedObject<sim::SoundPlayQueue_t> sounds;
	SharedObject<sim::EventQueue_t> events;
	SharedObject<sim::MotionOutput_t> motionout;
	SharedObject<sim::MotionOutputPIDs_t> motionoutpids;
	typedef MessageQueue<10> CommandQueue_t;
	SharedObject<CommandQueue_t> commandQueue;
	
	MessageQueueStatusThread cameraStatus;
	MessageQueueStatusThread sensorStatus;
	MessageQueueStatusThread timerStatus;
	MessageQueueStatusThread motionStatus;
	MessageQueueStatusThread eventsStatus;

	IPCEventTranslator * etrans;
	MessageReceiver * commandrecv;
	MessageReceiver * motionrecv;
	MessageReceiver * motionpidsrecv;
	
	static Simulator* theSim;
	std::set<unsigned int> frameTimes;
	double runSpeed;
	double lastTimeScale;
	step_t step;
	unsigned int waitingSteps;
	SharedGlobals::runlevel_t curLevel;
	
	static std::set<MotionHook*> motionHooks;
	std::set<std::string> activeSensors;
	std::set<DataSource*> activeSensorSrcs;
	std::set<std::string> activeCameras;
	std::set<DataSource*> activeCameraSrcs;
		
	TimeET fullspeedWallStart; //!< "real" wall-clock time that full-speed mode was entered
	unsigned int fullspeedSimStart; //!< simulator time at which full-speed mode was entered
	TimeET lastFrameWallStart; //!< "real" wall-clock time that processing started on last frame (only valid in full-speed mode)
	float avgWallTime; //!< running average of frame processing time
	float avgSimTime; //!< running average of frame increments
	static const float avgSpeedupGamma; //!< gamma parameter for calculating running average in #avgWallTime and #avgSimTime
	
	Thread::Lock simLock;
	
private:
	Simulator(const Simulator&); //!< don't call (copy constructor)
	Simulator& operator=(const Simulator&); //!< don't call (assignment operator)
};

/*! @file
 * @brief Defines Simulator, which DESCRIPTION
 * @author ejt (Creator)
 */

#endif
