//-*-c++-*-
#ifndef INCLUDED_LedMC_h
#define INCLUDED_LedMC_h

#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_LEDS

#include "Events/EventBase.h"
#include "Events/EventRouter.h"
#include "MotionCommand.h"
#include "LedEngine.h"
#include "OutputCmd.h"
#include "MotionManager.h"

//! This is just a simple wrapper - you probably want to be looking at LedEngine
/*! This is handy if all you want to do is control the LED's, but since other
 *  MotionCommands will probably also want to make use of the LEDs, they can
 *  just use the engine component to do all the work. */
class LedMC : public MotionCommand, public LedEngine {
public:
	//! constructor
	LedMC() : MotionCommand(), LedEngine(), notified(true)
	{
#ifdef TGT_HAS_LEDS
		setWeights(AllLEDMask,1);
#endif
	}
	//! destructor
	virtual ~LedMC() {}
	
	virtual void doStart() {
		MotionCommand::doStart();
		dirty=true;
	}
	
	void setDirty() { dirty = true; }

	//! updates the cmds from LedEngine::updateLEDs()
	virtual int updateOutputs() {
		notified = !updateLEDFrames(cmds) || dirty;
#ifdef TGT_HAS_LEDS
		for(unsigned int i=0; i<NumLEDs; i++)
			if(cmds[i][0].weight!=0)
				motman->setOutput(this,i+LEDOffset,cmds[i]);
#endif
		if (nextFlashEnd < (unsigned int)-1)  // do we have a flash in progress?
			dirty=true;
		else if (notified == false) {  // flash has ended (nextFlashEnd == -1), but notice not yet sent
			postEvent(EventBase(EventBase::motmanEGID,getID(),EventBase::statusETID));
			notified=true;
		};
		return NumLEDs;
	}
	
	virtual int isDirty() { return LedEngine::isDirty(); }
	
	virtual int isAlive() { return LedEngine::isDirty() || nextFlashEnd < (unsigned int)-1; }
	
#ifdef TGT_HAS_LEDS
	//! Sets the JointCmd::weight of the LEDs specified by @a leds to @a weight
	void setWeights(LEDBitMask_t leds, float weight) {
		for(unsigned int i=0; i<NumLEDs; i++)
			if((leds>>i)&1)
				for(unsigned int f=0; f<NumFrames; f++)
					cmds[i][f].weight=weight;
	}
#endif
	
protected:
	OutputCmd cmds[NumLEDs][NumFrames]; //!< needed to store weight values of LEDs (useful to mark LEDs as unused)
	bool notified; //!< set to true when we've posted a status event for completion of a flash/cflash
};

/*!@file
 * @brief Defines LedMC, which provides a basic MotionCommand wrapper to LedEngine
 * @author ejt (Creator)
 */

#endif // TGT_HAS_LEDS

#endif

// Another way of doing things (this style for everyone else, look at EmergencyStopMC for instance
// But for the main LedMC, this gets out of sync when i change the engine, so i just use inheritance
/* 
 class LedMC : public MotionCommand, public LedEngine {
 public:
 LedMC() : MotionCommand() {MCInit(); setPriority(kLowPriority); setWeight(~0,1); }
 virtual int updateJointCmds() { return engine.updateLEDs(cmds); }
 virtual inline const JointCmd& getJointCmd(unsigned int i) { return (i>=LEDOffset && i<LEDOffset+NumLEDs)?cmds[i-LEDOffset]:unusedJoint; }
 virtual int isDirty() { return engine.isDirty(); }
 virtual int isAlive() { return true; }
 
 void invert(LEDBitMask_t leds) { engine.invert(leds); }
 void cset(LEDBitMask_t leds, float value) { engine.cset(leds,value); }
 void set(LEDBitMask_t leds, float value) { engine.set(leds,value); }
 void cflash(LEDBitMask_t leds, unsigned int ms=500) { engine.cflash(leds,ms); }
 void flash(LEDBitMask_t leds, unsigned int ms=500) { engine.flash(leds,ms); }
 void ccycle(LEDBitMask_t leds, unsigned int period, float amp, int offset=0) { engine.ccycle(leds,period,amp,offset); }
 void cycle(LEDBitMask_t leds, unsigned int period, float amp, int offset=0) { engine.cycle(leds,period,amp,offset); }
 void clear() { engine.clear(); }
 void setWeight(LEDBitMask_t leds, float weight) {
 for(unsigned int i=0; i<NumLEDs; i++)
 if((leds>>i)&1)
 cmds[i].weight=weight;
 }
 
 float getSetting(LEDOffset_t led_id) { return engine.getSetting(led_id); }
 float getValue(LEDOffset_t led_id) { return engine.getValue(led_id); }
 
 protected:
 static unsigned int crID;
 virtual void setClassRegistrationID(unsigned int id) { crID=id; }
 virtual unsigned int getClassRegistrationID() const { return crID; }
 
 LedEngine engine;
 
 JointCmd cmds[NumLEDs];
 };
 */
