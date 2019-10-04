//-*-c++-*-
#ifndef INCLUDED_RunSequenceControl_h_
#define INCLUDED_RunSequenceControl_h_

#include "FileBrowserControl.h"
#include "Motion/MotionSequenceMC.h"
#include "Motion/EmergencyStopMC.h"
#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_LEDS
#  include "Shared/ERS210Info.h"
#  include "Motion/LedMC.h"
#endif
#include "Events/EventRouter.h"
#include "Sound/SoundManager.h"
#include "Shared/TimeET.h"
#include "Shared/Config.h"
#include "Shared/ProjectInterface.h"
#include <string>

//! Upon activation, loads a position from a file name read from cin (stored in ms/data/motion...)
/*! The template parameter is passed to MotionSequenceMC's template
 *  parameter in order to specify the number of keyframes to reserve -
 *  larger values use more memory, but will allow you to load more
 *  complicated sequences.
 *
 *  The motion sequence doesn't actually start playing until the
 *  emergency stop is deactivated.  This avoids either cutting off the
 *  beginning of the sequence while still in estop, or having to
 *  override the estop, which may be unexpected.
 */
template<unsigned int SequenceSize>
class RunSequenceControl : public FileBrowserControl, public EventListener {
public:
	//! Constructor, sets filter to *.mot
	RunSequenceControl(const std::string& n)
		: FileBrowserControl(n,"Runs a motion sequence from a user-specified file",config->motion.root),
			ledid(invalid_MC_ID), waitingFile()
	{
		setFilter("*.mot");
	}

	//! destructor
	virtual ~RunSequenceControl() {
		erouter->removeListener(this);
		motman->removeMotion(ledid);
		ledid=invalid_MC_ID;
	}

	//! only called when e-stop has been turned off and we're waiting to load a file
	virtual void processEvent(const EventBase& /*event*/) {
		erouter->removeListener(this);
		runFile();
		motman->removeMotion(ledid);
		ledid=invalid_MC_ID;
	}
	
protected:
	//! loads the motion sequence and runs it
	void runFile() {
		//TimeET timer;
		SharedObject< MotionSequenceMC<SequenceSize> > s(waitingFile);
		//cout << "Load Time: " << timer.Age() << endl;
		motman->addPrunableMotion(s);
		waitingFile="";
	}

	//!does the actual loading of the MotionSequence
	virtual ControlBase* selectedFile(const std::string& f) {
		waitingFile=f;
		if(!ProjectInterface::estop->getStopped()) {
			runFile();
		} else {
			//we have to wait for the estop to be turned off
			sndman->playFile("donkey.wav");
#ifdef TGT_HAS_LEDS
			SharedObject<LedMC> led;
			led->cset(FaceLEDMask,0);
			unsigned int botl = capabilities.findOutputOffset(ERS210Info::outputNames[ERS210Info::BotLLEDOffset]);
			unsigned int botr = capabilities.findOutputOffset(ERS210Info::outputNames[ERS210Info::BotRLEDOffset]);
			if(botl==-1U || botr==-1U) {
				botl=LEDOffset;
				botr=NumLEDs>1 ? botl+1 : botl;
			}
			led->cycle(1<<(botl-LEDOffset),1000,3,0,0);
			led->cycle(1<<(botr-LEDOffset),1000,3,0,500);
			ledid=motman->addPersistentMotion(led);
#endif
			erouter->addListener(this,EventBase::estopEGID,ProjectInterface::estop.getID(),EventBase::deactivateETID);
		}
		return this;
	}

	MC_ID ledid; //!< MC_ID of the led we use to signal there's a MotionSequence lined up
	std::string waitingFile; //!< filename of the motion sequence waiting to load
};

/*! @file
 * @brief Defines RunSequenceControl, which when activated, loads and runs a motion sequence from a file name read from cin (stored in ms/data/motion)
 * @author ejt (Creator)
 */

#endif
