//-*-c++-*-
#ifndef INCLUDED_PlaySoundControl_h_
#define INCLUDED_PlaySoundControl_h_

#include "FileBrowserControl.h"
#include "Sound/SoundManager.h"

//! Upon activation, loads plays a selected sound stored in ms/data/sound)
class PlaySoundControl : public FileBrowserControl {
 public:
	//! Constructor
	PlaySoundControl(const std::string& n)
		: FileBrowserControl(n,"Plays a sound from a user specified sound file",config->sound.root)
	{
		setFilter("*.wav");
	}
	//! Destructor
	virtual ~PlaySoundControl() {}

protected:
	//!does the actual loading of the MotionSequence
	virtual ControlBase* selectedFile(const std::string& f) {
	  if(sndman) {
	    sndman->stopPlay();
	    sndman->playFile(f.c_str());
	  }
	  return this;
	}
};

/*! @file
 * @brief Defines PlaySoundControl, which when activated, plays a sound file
 * @author ejt (Creator)
 */

#endif
