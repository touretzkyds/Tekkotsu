#ifndef INCLUDED_SaveWalkControl_h
#define INCLUDED_SaveWalkControl_h

#include "StringInputControl.h"
#include "Motion/MotionManager.h"
#include "Motion/WalkMC.h"
#include <string>

//! When activated, saves walk parameters to a file specified from cin
class SaveWalkControl : public StringInputControl {
 public:
	//! constructor, pass the MC_ID of the walk you want to save
	SaveWalkControl(const std::string& n, MC_ID w) : StringInputControl(n,"Saves Walk parameters to filename read from user","Please enter name for posture file (in data/motion)..."), walk_id(w), thewalk(NULL) {}
	//! constructor, pass a pointer to the walk you want to save
	SaveWalkControl(const std::string& n, WalkMC * awalk) : StringInputControl(n,"Saves Walk parameters to filename read from user","Please enter name for posture file (in data/motion)..."), walk_id(invalid_MC_ID), thewalk(awalk) {}
	//! destructor
	virtual ~SaveWalkControl() {}

	virtual ControlBase * takeInput(const std::string& msg) {
		if(msg.size()>0) {
			std::string filename=config->motion.makePath(msg);
			if(filename.find(".")==std::string::npos)
				filename+=".prm";
			MC_ID id = (thewalk==NULL ? walk_id : thewalk->getID() );
			WalkMC* walk=thewalk;
			if(id!=invalid_MC_ID)
				walk = (WalkMC*)motman->checkoutMotion(id);
			if(walk==NULL)
				serr->printf("Invalid walk for saving\n");
			else {
				walk->saveFile(filename.c_str());
				if(id!=invalid_MC_ID)
					motman->checkinMotion(id);
			}
		}
		return StringInputControl::takeInput(msg);
	}

 protected:
	MC_ID walk_id; //!< MC_ID of walk to save from
	WalkMC * thewalk; //!< walk to save from

private:
	SaveWalkControl(const SaveWalkControl&); //!< don't call
	SaveWalkControl operator=(const SaveWalkControl&); //!< don't call
};

/*! @file
 * @brief Defines SaveWalkControl, which when activated, saves walk parameters to a file specified from cin
 * @author ejt (Creator)
 */
#endif
