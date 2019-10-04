#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_BUTTONS

#include "Behaviors/BehaviorBase.h"
#include "Sound/SoundManager.h"
#include "Events/EventRouter.h"
#include "Shared/WorldState.h"

//! allows you to experiment with playing sounds different ways.
/*! A different sound will be played for each of the buttons, except the head buttons.
 *  When the chin button is held down, any sounds (from this behavior) will be queued
 *  up and then played successively once the chin button is released.
 *
 *  Notice that this doesn't preload all needed sounds:\n
 *  - @c barkmed.wav is listed in ms/config/tekkotsu.xml as a preloaded system sound
 *  - @c growl.wav will be loaded before being played automatically - notice the
 *    hiccup this can cause.
 */
class SoundTestBehavior : public BehaviorBase {
public:
	//! Constructor
	SoundTestBehavior()
		: BehaviorBase("SoundTestBehavior"), curplay(SoundManager::invalid_Play_ID), endtime(0),
			newChain(EventBase::buttonEGID,0,EventBase::activateETID),
			resume(EventBase::buttonEGID,0,EventBase::deactivateETID),
			snd1(EventBase::buttonEGID,1,EventBase::activateETID),
			snd2(EventBase::buttonEGID,2,EventBase::activateETID),
			snd3(EventBase::buttonEGID,3,EventBase::activateETID),
			snd4(EventBase::buttonEGID,4,EventBase::activateETID),
			snd5(EventBase::buttonEGID,5,EventBase::activateETID)
	{
#ifdef TGT_IS_AIBO
		newChain.setSourceID(ChinButOffset);
		resume.setSourceID(ChinButOffset);
		snd1.setSourceID(LFrPawOffset);
		snd2.setSourceID(RFrPawOffset);
		snd3.setSourceID(LBkPawOffset);
		snd4.setSourceID(RBkPawOffset);
		if(capabilities.getButtonOffset("BackBut")!=-1U)
			snd5.setSourceID(capabilities.getButtonOffset("BackBut"));
#endif
		// otherwise unknown model, stick with buttons 0-5, whatever those are
	}
	
	
	//! Load some sounds, listen for button events
	virtual void doStart() {
		BehaviorBase::doStart();
		erouter->addListener(this,EventBase::buttonEGID);
		sndman->loadFile("yap.wav");
		sndman->loadFile("howl.wav");
		sndman->loadFile("whimper.wav");
	}

	//! Release sounds we loaded in doStart()
	virtual void doStop() {
		BehaviorBase::doStop();
		erouter->removeListener(this);
		sndman->releaseFile("howl.wav");
		sndman->releaseFile("yap.wav");
		sndman->releaseFile("whimper.wav");
	}

	//! Play the sound corresponding to the button
	virtual void doEvent() {
		if(*event==snd1)
			play("howl.wav");
		else if(*event==snd2)
			play("yap.wav");
		else if(*event==snd3)
			play("whimper.wav");
		else if(*event==snd4)
			play("growl.wav");
		else if(*event==snd5)
			play("barkmed.wav");
		else if(*event==newChain) {
			curplay=SoundManager::invalid_Play_ID;
			endtime=0;
		} else if(pauseWhileChin && *event==resume)
			sndman->resumePlay(curplay);
	}

	//! returns name to system
	static std::string getClassDescription() { return "Plays different sounds when buttons are pressed.  Holding the chin button queues the sounds."; }
	virtual std::string getDescription() const { return getClassDescription(); }

protected:
	//! called when a button is pressed - checks if it should enqueue or just play
	void play(const char* name) {
		if(!state->buttons[newChain.getSourceID()]) {

			// Just play the sound
			// This is probably all you need how to do unless you want to get fancy
			sndman->playFile(name);

		} else {

			// Enqueue the sound - mainly useful if you have a set of sounds and want to play a song with them
			if(curplay==SoundManager::invalid_Play_ID || (!pauseWhileChin && get_time()>=endtime) ) {
				//start a new chain, either this is the first or we already finished playing the chain
				curplay=sndman->playFile(name);
				if(pauseWhileChin)
					sndman->pausePlay(curplay);
			} else //add to existing chain
				sndman->chainFile(curplay,name);
			endtime=sndman->getRemainTime(curplay)+get_time()-SoundBufferTime;
			//-SoundBufferTime to guarrantee ID validity, see SoundManager::getRemainTime() documentation

		}
	}
	static const bool pauseWhileChin=true; //!< if this is true, won't start playing chain until you release the chin button
	SoundManager::Play_ID curplay; //!< current chain (may not be valid if chin button not down or time is past #endtime)
	unsigned int endtime; //!< the expected end of play time for the current chain
	
	//!@name Event Templates
	//!Used to match against the different buttons that have sounds mapped to them
	EventBase newChain,resume,snd1,snd2,snd3,snd4,snd5;
	//@}
};

REGISTER_BEHAVIOR_MENU(SoundTestBehavior,DEFAULT_TK_MENU);

#endif  // check for TGT_HAS_BUTTONS

/*! @file
 * @brief Defines the SoundTestBehavior demo, which allows you to experiment with playing sounds different ways.
 * @author ejt (Creator)
 */
