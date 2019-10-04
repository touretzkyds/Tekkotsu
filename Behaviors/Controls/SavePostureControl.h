//-*-c++-*-
#ifndef INCLUDED_SavePostureControl_h_
#define INCLUDED_SavePostureControl_h_

#include "StringInputControl.h"
#include "Motion/PostureEngine.h"

//! Upon activation, saves the current position to a file name read from user (default is /ms/data/motion...)
class SavePostureControl : public StringInputControl {
 public:
	//! Constructor
	SavePostureControl() : StringInputControl("Save Posture","Saves current posture to filename read from user","Please enter name for posture file (in data/motion)...") {}
	//! Constructor
	SavePostureControl(const std::string& n) : StringInputControl(n,"Saves current posture to filename read from user","Please enter name for posture file (in data/motion)...") {}

	virtual ControlBase * takeInput(const std::string& msg) {
		if(msg.size()>0) {
			std::string filename;
			if(filename.find(".")==std::string::npos)
				filename+=".pos";
			filename=config->motion.makePath(msg);
			PostureEngine post;
			post.takeSnapshot();
			post.setWeights(1);
			post.saveFile(filename.c_str());
		}
		return StringInputControl::takeInput(msg);
	}
};

/*! @file
 * @brief Defines SavePostureControl, which when activated, saves the current position to a file name read from user (default is /ms/data/motion...)
 * @author ejt (Creator)
 */

#endif
