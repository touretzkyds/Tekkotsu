//-*-c++-*-
#ifndef INCLUDED_SoundPlay_h_
#define INCLUDED_SoundPlay_h_

#include "Process.h"
#include "sim.h"
#include "IPC/ProcessID.h"
#include "IPC/SharedObject.h"
#include "SharedGlobals.h"
#include "Sound/SoundManager.h"
#include "Shared/Profiler.h"
#include "SoundPlayThread.h"

class SoundPlay : public Process {
public:
	//! constructor
	SoundPlay();
	//! destructor
	~SoundPlay();

	virtual void doStart();
	virtual void doStop();

	static const char * getClassName() { return "SoundPlay"; }
	static ProcessID::ProcessID_t getID() { return ProcessID::SoundProcess; }
	
	static const char * getSoundPlayID() { return "Sounds"; }
	static const char * getSoundManagerID() { return "SoundManager"; }
	static const char * getSoundProfilerID() { return "SoundProfiler"; }

protected:
	SharedObject<sim::SoundPlayQueue_t> requests;
	SharedObject<sim::EventQueue_t> events;
	SharedObject<sim::StatusRequest_t> statusRequest;
	SharedObject<SoundManager> soundmanager;
	SharedObject<soundProfiler_t> soundProf;
	
	static SoundPlayThread * player;
	
	IPCEventTranslator * etrans;
	class MessageReceiver * sndrecv;
	class MessageReceiver * statusrecv;
	static bool gotSnd(RCRegion* msg);
	
private:
	SoundPlay(const SoundPlay&); //!< don't call (copy constructor)
	SoundPlay& operator=(const SoundPlay&); //!< don't call (assignment operator)
};

/*! @file
 * @brief Defines SoundPlay, which DESCRIPTION
 * @author ejt (Creator)
 */

#endif
