//-*-c++-*-
#ifndef INCLUDED_CameraBehavior_h_
#define INCLUDED_CameraBehavior_h_

#include "Behaviors/BehaviorBase.h"
#include "Motion/MotionManager.h"
#include "Events/EventBase.h"

//! Will take images and write to log file
/*! Press the head button to take a picture, back button to write to memory
 *  stick.  The leds will flash when finished writing.
 *
 *  The reason for this is to provide sample code for accessing vision
 *  data, and also simply because we should have a way to save
 *  pictures to memstick instead of relying solely on having wireless
 *  to transmit them over.
 *
 *  Image format is chosen by current config settings for the
 *  Config::vision_config::RawCamConfig::compression and
 *  Config::vision_config::RawCamConfig::channel.  However, the double
 *  resolution layer is always saved instead of whatever the current
 *  config skip value indicates.
 */
class CameraBehavior : public BehaviorBase {
 public:
	//! constructor, just sets up the variables
	CameraBehavior()
		: BehaviorBase("CameraBehavior"), camera_click(EventBase::buttonEGID,0,EventBase::deactivateETID,150), index(0), ledID(MotionManager::invalid_MC_ID)
	{}

	//! Register for events
	virtual void doStart();
	
	//! Removes its two motion commands
	virtual void doStop();
	
	//! Handles event processing - determines which generator to save from and writes to current file
	virtual void doEvent();

	static std::string getClassDescription() { return "Push head button to save a picture"; }
	virtual std::string getDescription() const { return getClassDescription(); }

 protected:
	//! opens the next file to be saved to (with @a ext extension on the file name)
	FILE * openNextFile(const std::string& ext);

	//! returns the path and name of the next file to be saved to (with @a ext extension on the file name)
	std::string getNextName(const std::string& ext);

	//! scans the /ms/data directory for image files and assigns the next unused index to #index
	void initIndex();

	EventBase camera_click; //!< event mask for taking a picture (head button)
	unsigned int index; //!< the index to use for the next image saved
	
	MotionManager::MC_ID ledID; //!< the id of the LedMC used to signal completion
};

/*! @file
 * @brief Describes CameraBehavior, for taking pictures
 * @author alokl (Creator)
 * @author ejt (rewrite for new vision system)
 */

#endif
