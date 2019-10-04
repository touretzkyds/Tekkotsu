#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_LEDS

#include "Shared/ERS210Info.h"
#include "Shared/ERS220Info.h"
#include "Shared/ERS7Info.h"

#include "FlashIPAddrBehavior.h"
#include "Events/EventRouter.h"
#include "Motion/MMAccessor.h"
#include "Motion/LedEngine.h"
#include "Shared/WorldState.h"
#include "Shared/Config.h"
#include "Sound/SoundManager.h"
#include "Wireless/Wireless.h"

REGISTER_BEHAVIOR_MENU_OPT(FlashIPAddrBehavior,"Background Behaviors/System Daemons",BEH_NONEXCLUSIVE|BEH_START);

void FlashIPAddrBehavior::doStart() {
	BehaviorBase::doStart(); // do this first
	if(config->behaviors.flash_on_start) {
		setupSequence();
		loadSounds();
		ms_id = motman->addPrunableMotion(ms,MotionManager::kEmergencyPriority+1);
		erouter->addListener(this,EventBase::motmanEGID,ms_id,EventBase::deactivateETID);
	}
#ifdef TGT_HAS_BUTTONS
	erouter->addListener(this,EventBase::buttonEGID,button1);
	erouter->addListener(this,EventBase::buttonEGID,button2);
#endif
}

void FlashIPAddrBehavior::doStop() {
	erouter->removeListener(this);
	motman->removeMotion(ms_id);
	ms_id=MotionManager::invalid_MC_ID;
	releaseSounds();
	BehaviorBase::doStop(); // do this last
}

void FlashIPAddrBehavior::doEvent() {
	if(event->getGeneratorID()==EventBase::timerEGID) {

		if(event->getSourceID()==ACTIVATE_TIMER) {
			//buttons have been held down long enough, time to run display
			if(ms_id!=MotionManager::invalid_MC_ID) {
				//there's already one running, have to check it out to clear it
				MMAccessor<MSMC_t> ms_acc(ms_id);
				setupSequence();
			} else
				setupSequence();
			loadSounds();
			ms_id = motman->addPrunableMotion(ms);
			erouter->addListener(this,EventBase::motmanEGID,ms_id,EventBase::deactivateETID);
				
		} else { //its time to play a digit sound file
			//the source id was set to correspond to an element of the sounds vector
			if(event->getSourceID()>=sounds.size())
				serr->printf("ERROR: %s received invalid timer event %s\n",getName().c_str(),event->getName().c_str());
			else {
				sndman->playFile(sounds[event->getSourceID()]);
				if(event->getSourceID()==sounds.size()-1)
					releaseSounds();
			}
				
		}

#ifdef TGT_HAS_BUTTONS
	} else if(event->getGeneratorID()==EventBase::buttonEGID) {
		//if it's an activate, start a timer to expire in a few seconds
		//if it's a deactivate, cancel that timer
		if(event->getTypeID()==EventBase::activateETID) {
			if(state->buttons[button1] && state->buttons[button2])
				erouter->addTimer(this,ACTIVATE_TIMER,2000,false);
		} else if(event->getTypeID()==EventBase::deactivateETID)
			erouter->removeTimer(this,ACTIVATE_TIMER);
#endif

	} else if(event->getGeneratorID()==EventBase::motmanEGID) {
		// display has completed, mark it as such
		if(event->getSourceID()!=ms_id)
			serr->printf("WARNING: %s received event %s, doesn't match ms_id (%d)\n",getName().c_str(),event->getName().c_str(),ms_id);
		ms_id=MotionManager::invalid_MC_ID;
		erouter->removeListener(this,EventBase::motmanEGID);
			
	}
}

void FlashIPAddrBehavior::loadSounds() {
	for(unsigned int i=0; i<sounds.size(); i++)
		sndman->loadFile(sounds[i]);
}

void FlashIPAddrBehavior::releaseSounds() {
	for(unsigned int i=0; i<sounds.size(); i++)
		sndman->releaseFile(sounds[i]);
	sounds.clear();
}

void FlashIPAddrBehavior::setupSequence() {
	const unsigned int DISP_TIME=600;
	const unsigned int GROUP_TIME=500;
	const unsigned int DOT_TIME=400;
	const unsigned int FADE_TIME=1;
	const unsigned int BLANK_TIME=100-FADE_TIME*2;
	erouter->removeTimer(this);
	ms->clear();
	releaseSounds();
	unsigned int a=wireless->getIPAddress();
	unsigned int n=config->behaviors.flash_bytes;
	if(n>4)
		n=4;
	LedEngine disp;
	for(unsigned int i=n-1; i!=-1U; i--) {
		unsigned int b=(a>>(i*8))&0xFF;
		unsigned int digits=1;
		if(b>=10)
			digits++;
		if(b>=100)
			digits++;
		//cout << "byte " << i << " is " << b << " -- " << digits << " digits" << endl;
		//cout << "Setting LEDs: ";
		for(unsigned int d=0; d<digits; d++) {
			unsigned int digit=b;
			for(unsigned int j=d;j<digits-1;j++)
				digit/=10;
			digit-=(digit/10)*10;
			disp.displayNumber(digit,LedEngine::onedigit);
			std::string soundfile="numbers/";
			soundfile+=(digit+'0');
			soundfile+=".wav";
			erouter->addTimer(this,sounds.size(),ms->getTime()+delay,false);
			sounds.push_back(soundfile);
			for(unsigned int j=0; j<NumLEDs; j++)
				if(FaceLEDMask&(1<<j)) {
					//if(disp.getValue(static_cast<LEDOffset_t>(LEDOffset+j)))
					//cout << j << ' ';
					ms->setOutputCmd(LEDOffset+j,disp.getValue(static_cast<LEDOffset_t>(LEDOffset+j)));
				}
			ms->advanceTime(DISP_TIME);
			for(unsigned int j=0; j<NumLEDs; j++)
				if(FaceLEDMask&(1<<j))
					ms->setOutputCmd(LEDOffset+j,disp.getValue(static_cast<LEDOffset_t>(LEDOffset+j)));
			ms->advanceTime(FADE_TIME);
			for(unsigned int j=0; j<NumLEDs; j++)
				if(FaceLEDMask&(1<<j))
					ms->setOutputCmd(LEDOffset+j,0);
			ms->advanceTime(BLANK_TIME);
			if(d==digits-1)
				ms->advanceTime(GROUP_TIME);
			for(unsigned int j=0; j<NumLEDs; j++)
				if(FaceLEDMask&(1<<j))
					ms->setOutputCmd(LEDOffset+j,0);
			ms->advanceTime(FADE_TIME);
		}
		//cout << endl;
		if(i!=0) {
			LEDBitMask_t dot=LedEngine::defaultCountNumMasks[10]; //default in case we don't recognize model
#ifdef TGT_HAS_LED_PANEL
			if(RobotName == ERS210Info::TargetName) {
				dot=LedEngine::defaultMimicNumMasks[10];
			} else if(RobotName == ERS220Info::TargetName) {
				dot=LedEngine::ERS220numMasks[10];
			} else if(RobotName == ERS7Info::TargetName) {
				dot=LedEngine::ERS7numMasks[10];
			}
#endif
			erouter->addTimer(this,sounds.size(),ms->getTime()+delay,false);
			sounds.push_back("numbers/dot.wav");
			for(unsigned int j=0; j<NumLEDs; j++)
				if(FaceLEDMask&(1<<j))
					ms->setOutputCmd(LEDOffset+j,(dot>>j)&1);
			ms->advanceTime(DOT_TIME);
			for(unsigned int j=0; j<NumLEDs; j++)
				if(FaceLEDMask&(1<<j))
					ms->setOutputCmd(LEDOffset+j,(dot>>j)&1);
			ms->advanceTime(FADE_TIME);
			for(unsigned int j=0; j<NumLEDs; j++)
				if(FaceLEDMask&(1<<j))
					ms->setOutputCmd(LEDOffset+j,0);
			ms->advanceTime(BLANK_TIME);
			for(unsigned int j=0; j<NumLEDs; j++)
				if(FaceLEDMask&(1<<j))
					ms->setOutputCmd(LEDOffset+j,0);
			ms->advanceTime(FADE_TIME);
		}
	}
	ms->play();
}

#endif

/*! @file
 * @brief Implements FlashIPAddrBehavior, which displays IP address by flashing a series of numbers on the LED face panel
 * @author ejt (Creator)
 */
