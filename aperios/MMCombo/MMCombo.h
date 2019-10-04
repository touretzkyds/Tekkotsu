//-*-c++-*-
#ifndef INCLUDED_MMCombo_h_
#define INCLUDED_MMCombo_h_

#include "Shared/debuget.h"
#include "Shared/MarkScope.h"
#include "Shared/RobotInfo.h"
#include "Shared/plistBase.h"
#include "Wireless/Wireless.h"
#include "IPC/ProcessID.h"
#include "aperios/EntryPoint.h"

#include <OPENR/OObject.h>
#include <OPENR/OSubject.h>
#include <OPENR/OObserver.h>
#include "aperios/MMCombo/def.h"

class EventTranslator;
class WorldState;

//! Contains code for both MainObj and MotoObj processes
/*! Why go to all this trouble?  Virtual functions and polymorphism!  Instead
 *  of writing my own object typing and serialization system, I would rather
 *  just use C++'s.  But function lookups of the run time type information (RTTI)
 *  will break unless the object that created the object and the object that's
 *  actually calling the function agree on what object A's information is.
 *
 *  The easiest way to guarantee this is to compile them as one object, and
 *  then replace the strings in the source binary with strings for each of
 *  the final objects so they'll each have their own identity, but share
 *  the same code.
 *
 *  This is as close as I can get to a "fork", which is what i really want.
 */
class MMCombo : public OObject {
public:
	//! constructor
	MMCombo();
	virtual ~MMCombo() {} //!< destructor
	
	OSubject*    subject[numOfSubject];   //!< holds information for each of our subjects (data we provide)
	OObserver*   observer[numOfObserver]; //!< holds information for each of the sources we're observing
	
	virtual OStatus DoInit   (const OSystemEvent&);      //!< first call (after constructor), set up memory
	virtual OStatus DoStart  (const OSystemEvent&);      //!< second call, ask for messages
	virtual OStatus DoStop   (const OSystemEvent&);      //!< next to last call, stop sending and receiving messages
	virtual OStatus DoDestroy(const OSystemEvent&);      //!< last call (before destructor), clean up memory here
	
	void ReadyRegisterWorldState(const OReadyEvent&);    //!< main only, send out the state global
	void GotWorldState(const ONotifyEvent& event);       //!< motion only, called when state global is received
	void ReadyRegisterMotionManager(const OReadyEvent&); //!< motion only, send out motman global
	void GotMotionManager(const ONotifyEvent& event);    //!< main only, called when motman global is received
	void GotInterProcessEvent(const ONotifyEvent& event);//!< main only, called when another process sends an event
	void ReadyRegisterProfiler(const OReadyEvent&); //!< motion only, send out motion's profiler
	void GotMotionProfiler(const ONotifyEvent& event);    //!< main only, called when Motion's profiler is received
	void GotSoundProfiler(const ONotifyEvent& event);    //!< main only, called when SoundPlay's profiler is received
	
	void ReadySendJoints(const OReadyEvent& event);      //!< motion only (until main does ears again, then both) calls SendJoints, if DoStart has already been called
	void GotSensorFrame(const ONotifyEvent& event);      //!< main only, called when new sensor information is available
	void GotImage(const ONotifyEvent& event);            //!< main only, called when a new image is available
	void GotAudio(const ONotifyEvent& event);            //!< main only, called when a new audio buffer is available
	void GotPowerEvent(void * msg);                      //!< main only, called when a power event occurs (can be just status events)
	
	void GotMotionMsg(const ONotifyEvent& event);        //!< both, called when a new MotionManagerMsg has been received
	
	void GotSoundManager(const ONotifyEvent& event);     //!< both, called when the sndman global is received

	void ReadyRegisterProcessMap(const OReadyEvent&);    //!< main only, send out the process map
	void GotProcessMap(const ONotifyEvent& event);       //!< motion only, called when process map is received	
	
	void ListenCont (void* msg) { MarkScope ep(entryPt); wireless->ListenCont(msg); }  //!< main only, called when //ALTODO
	void BindCont   (void* msg) { MarkScope ep(entryPt); wireless->BindCont(msg); }    //!< main only, called when //ALTODO
	void ConnectCont(void* msg) { MarkScope ep(entryPt); wireless->ConnectCont(msg); } //!< main only, called when //ALTODO
	void SendCont   (void* msg) { MarkScope ep(entryPt); wireless->SendCont(msg); }    //!< main only, called when //ALTODO
	void ReceiveCont(void* msg) { MarkScope ep(entryPt); wireless->ReceiveCont(msg); } //!< main only, called when //ALTODO
	void CloseCont  (void* msg) { MarkScope ep(entryPt); wireless->CloseCont(msg); }   //!< main only, called when //ALTODO

protected:
	void OpenPrimitives();                               //!< both, called from SetupOutputs() (mostly for motion, but main does ears), uses #open to tell which to open
	void SetupOutputs(const bool to_open[NumOutputs]);   //!< both, called from DoInit() (mostly for motion, but main does ears)
	RCRegion* InitRegion(unsigned int size);             //!< both, called to set up a shared memory region of a given size
	void initRNG();                                      //!< called when we're ready to initialize the RNG, either from sensor values or from static value depending on config settings

	RCRegion * motmanMemRgn;     //!< Motion creates, Main receives
	RCRegion * motionProfilerMemRgn; //!< Motion creates, Main receives
	RCRegion * soundProfilerMemRgn; //!< Sound creates, Main receives
	RCRegion * worldStateMemRgn; //!< Main creates, Motion receives
	RCRegion * soundManagerMemRgn; //!< SoundPlay creates, Main & Motion receives
	RCRegion * processMapMemRgn; //!< Main creates, Motion receives

	OPrimitiveID primIDs[NumOutputs];    //!< both, Main ears only, Motion the rest
	static const unsigned int NUM_COMMAND_VECTOR=2; //!< both, for double buffering
	RCRegion*    region[NUM_COMMAND_VECTOR]; //!< both, the actual buffers

	float  ledActivation[NumLEDs]; //!< Motion, used for partial LED activation

	unsigned int runLevel;           //!< Main, incremented until all sections are ready
	static const unsigned int readyLevel=7; //!< Main, runLevel at which StartBehavior is created. (1st power event, 1st sensor event, motman init, motion profiler, sound profiler, sndman init, MainObj::DoStart())
	void addRunLevel();              //!< Main, checks runLevel and creates StartBehavior when ready

	bool open[NumOutputs];    //!< both, holds information regarding which outputs are open in ("controlled by") this process
	unsigned int num_open;  //!< both, count of how many are open

	EventTranslator * etrans; //!< both, allows events to be sent between processes (from other processes besides these two too)
	
	EntryPoint entryPt; //!< should be used with MarkScope to wrap each entry point from the system, handles marking root stack frame

	bool isStopped; //!< true if we've received a DoStart and no DoStop - we need this because sometimes an extra message seems to slip in after we've been told to stop, in which case we should ignore it

	//! intercepts changes made to the camera's gain setting, and calls the system API to make it so
	class GainSettingListener : public plist::PrimitiveListener {
	public:
		virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	} gainListener;
	
	//! intercepts changes made to the camera's shutter speed, and calls the system API to make it so
	class ShutterSettingListener : public plist::PrimitiveListener {
	public:
		virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	} shutterListener;
	
	//! intercepts changes made to the camera's white balance setting, and calls the system API to make it so
	class WhiteBalanceSettingListener : public plist::PrimitiveListener {
	public:
		virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	} wbListener;
	
	//! Motion only, maintains the activation level of the LEDs, returns whether it should be 'fired'
	inline OLEDValue calcLEDValue(unsigned int i,float x) {
		if(x<=0.0) {
			ledActivation[i]*=.9; //decay activation... resets to keeps LEDs in sync, looks a little better
			return oledOFF;
		} else if(x>=1.0) {
			return oledON;
		} else {
			x*=x; // squared to "gamma correct" - we can see a single pulse better than a single flicker - after image and all that
			ledActivation[i]+=x;
			if(ledActivation[i]>=1.0) {
				ledActivation[i]-=1.0;
				return oledON;
			} else {
				return oledOFF;
			}						
		}
	}

	//! returns @a f clipped to be between 0 and 1
	inline static float clipRange01(float f) {
		if(f>1)
			return 1;
		if(f<0)
			return 0;
		return f;
	}

private:
	MMCombo(const MMCombo&); //!< should never be called...
	MMCombo& operator=(const MMCombo&); //!< should never be called...
};

/*! @file
 * @brief Describes MMCombo, the OObject which "forks" (sort of) into Main and Motion processes
 * @author ejt (Creator)
 */

#endif





