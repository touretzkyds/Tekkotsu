//-*-c++-*-
#ifndef INCLUDED_SoundNode_h_
#define INCLUDED_SoundNode_h_

#include "Behaviors/StateNode.h"
#include "Events/EventRouter.h"
#include "Sound/SoundManager.h"

//! A simple StateNode that plays a sound upon startup and posts a status event on completion
/*! Doesn't automatically preload the sound buffer - if you want the sound file
 *  to be preloaded, you'll have to make the
 *  SoundManager::loadFile() / SoundManager::releaseFile() calls yourself.
 *  
 *  By default, sound playback will continue even after this node has been deactivated.
 *  If this is not the behavior you desire, set the #autostop flag (through setAutoStop())
 */
class SoundNode : public StateNode {
public:
	//! constructor, specify a sound file to play
	SoundNode(const std::string& soundfilename="") : 
	StateNode(), filename(soundfilename), curplay_id(SoundManager::invalid_Play_ID), autostop(false) {}
	
	//! constructor, specify instance name and sound file to play
	SoundNode(const std::string& nodename, const std::string& soundfilename) : 
	StateNode(nodename), filename(soundfilename), curplay_id(SoundManager::invalid_Play_ID), autostop(false) {}
	
	//! activate the node, starts playing the sound
	virtual void preStart() {
		StateNode::preStart();
		if(event!=NULL) {
			const DataEvent<std::string>* ev = dynamic_cast<const DataEvent<std::string>*>(event);
			if ( ev != NULL )
				filename = ev->getData();
		}
	}
	
	virtual void postStart() {
		StateNode::postStart();
		startPlaying();
	}
	
	//! deactivate the node, doesn't stop the sound playback unless the #autostop flag has been set
	virtual void stop() {
		if(autostop)
			stopPlay();
		StateNode::stop();
	}
	
	//! receive audioEGID status event and post stateMachineEGID status event
	virtual void doEvent() {
		curplay_id = SoundManager::invalid_Play_ID;
		postStateCompletion();
	}
	
	//! interrupts playing of the current sound
	void stopPlay() {
		sndman->stopPlay(curplay_id);
		curplay_id = SoundManager::invalid_Play_ID;
	}
	
	//! returns the name of the sound file associated with this node
	std::string getFileName() { return filename; }
	
	//! sets the name of the sound file associated with this node
	void setFileName(std::string &soundfilename) { filename = soundfilename; }
	
	//! returns the current status of the #autostop flag
	bool getAutoStop() { return autostop; }
	
	//! sets the #autostop flag
	void setAutoStop(bool astop) { autostop=astop; }
	
protected:
	virtual void startPlaying() {
		if(filename.size()>0) {
			curplay_id = sndman->playFile(filename);
			erouter->addListener(this,EventBase::audioEGID,curplay_id,EventBase::deactivateETID);
		}
	}
	
	std::string filename; //!< filename of sound to play, accessed through setFileName() and getFileName()
	SoundManager::Play_ID curplay_id; //!< holds the playback identification so it can be halted any time
	bool autostop; //!< if set to true by setAutoStop(), when this node is deactivated, playback will be halted.  Otherwise, playback will continue even after the node is deactivated
	
};

/*! @file
 * @brief Defines SoundNode, a simple StateNode that plays a sound and throws a status event upon completion
 * @author dst (Creator)
 */

#endif
