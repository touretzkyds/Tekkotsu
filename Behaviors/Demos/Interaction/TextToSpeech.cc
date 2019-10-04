#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"
#include "Events/TextMsgEvent.h"
#include "Sound/SoundManager.h"
#include "Events/EventCallback.h"
#include "Shared/debuget.h"

//! A behavior which speaks any text message events, tests ability to chain sounds and pause/resume/stop speech.
/*! If the input string includes a '?', the 'ping.wav' sound will be played afterward (tests sound chaining).
 *  If the input string includes a '#', it will be paused after 0.5s and resumed 3s after that. */
class TextToSpeech : public BehaviorBase {
public:
	static const unsigned int HASH_DELAY = 500; //!< time until speech is paused for '#'
	static const unsigned int HASH_PAUSE = 3000; //!< duration of pause in speaking for '#'

	//! constructor
	TextToSpeech() : BehaviorBase("TextToSpeech"), ids(),
		callGotMessage(&TextToSpeech::gotMessage,*this),
		callGotPause(&TextToSpeech::gotPause,*this),
		callGotResume(&TextToSpeech::gotResume,*this),
		callGotSoundCompleted(&TextToSpeech::gotSoundCompleted,*this)
	{}
	
	virtual void doStart() {
		std::cout << " * Use the msg command to send a TextMsgEvent and the robot will speak it." << std::endl;
		std::cout << " * If your string includes a '?', the 'ping.wav' sound will be played afterward (chaining)" << std::endl;
		std::cout << " * If your string includes a '#', it will be paused after 0.5s and resumed 3s after that." << std::endl;
		gotMessage(TextMsgEvent("I will speak text-message events"));
		erouter->addListener(&callGotMessage, EventBase::textmsgEGID);
	}
	
	//! called when the user send a text message event
	void gotMessage(const TextMsgEvent& textev) {
		// remove hash character if present
		std::string msg = textev.getText();
		if(msg.find('#')!=std::string::npos)
			msg.erase(msg.find('#'),1);

		// speak the text message
		SoundManager::Play_ID playid = sndman->speak(msg);
		ids.insert(playid); // so we can track and stop on doStop()
		
		// so we can print a statement when it's done (verify events)
		// and also clear playid so we don't re-stop the sound on doStop
		erouter->addListener(&callGotSoundCompleted, EventBase::audioEGID, playid, EventBase::deactivateETID);
		
		// if it has a '?', chain a sound afterward
		if(textev.getText().find('?') != std::string::npos)
			sndman->chainFile(playid,"ping.wav");
		
		if(textev.getText().find('#') != std::string::npos)
			erouter->addTimer(&callGotPause,playid,HASH_DELAY,false);
	}
	
	//! this indicates we wanted to pause the speech (for one second)
	void gotPause(const EventBase& ev) {
		sndman->pausePlay(ev.getSourceID());
		erouter->addTimer(&callGotResume,ev.getSourceID(),HASH_PAUSE,false);
		std::cout << "Paused at " << get_time() << std::endl;
	}
	
	//! the pause has expired, continue the speech
	void gotResume(const EventBase& ev) {
		std::cout << "Resuming at " << get_time() << std::endl;
		sndman->resumePlay(ev.getSourceID());
	}
	
	//! the speech has completed
	void gotSoundCompleted(const EventBase& ev) {
		erouter->removeListener(&callGotSoundCompleted,ev); // stop listening in case sndman recycles playid's
		std::cout << ev.getName() << " has completed" << std::endl;
		ids.erase(ev.getSourceID());
	}
	
	void gotSoundStopped(const EventBase& ev) {
		std::cout << "Interrupted: " << ev.getName() << std::endl;
	}

	virtual void doStop() {
		// stopping sounds on behavior exit is NOT required, but here for testing and demonstration...
		
		// reuse existing listener, but have it dump a different message now that we're interrupting
		callGotSoundCompleted.redirect(&TextToSpeech::gotSoundStopped,*this);
		
		// if we kept gotSoundCompleted active (could also call erouter->removeListener(callGotSoundCompleted) first)
		//std::set<SoundManager::Play_ID> toStop = ids; // because gotSoundCompleted will receive events and modify ids
		
		for(std::set<SoundManager::Play_ID>::const_iterator it=ids.begin(); it!=ids.end(); ++it)
			sndman->stopPlay(*it);
	}
	
	
protected:
	std::set<SoundManager::Play_ID> ids; //!< the play id, just so we can test pause/resume/stop
	EventCallbackAs<TextMsgEvent> callGotMessage; //!< forwards text message events to gotMessage()
	EventCallback callGotPause; //!< forwards timer events for pausing to getPause()
	EventCallback callGotResume; //!< forwards timer events for resuming to gotResume()
	EventCallback callGotSoundCompleted; //!< forwards sound events to gotSoundCompleted
};

REGISTER_BEHAVIOR_MENU(TextToSpeech,DEFAULT_TK_MENU"/Interaction Demos");
