//-*-c++-*-
#ifndef INCLUDED_LoadWalkControl_h
#define INCLUDED_LoadWalkControl_h

#include "FileBrowserControl.h"
#include "Motion/WalkMC.h"
#include "Shared/Config.h"
#include <string>

//! When activated, loads a set of walk parameters from a file specified by user
class LoadWalkControl : public FileBrowserControl {
public:
	//! constructor, pass the MC_ID of the WalkMC which you want to save
	LoadWalkControl(const std::string& n, MC_ID w)
		: FileBrowserControl(n,"Loads a set of walk parameters from a file specified by user",config->motion.root), walk_id(w), thewalk(NULL)
	{
		setFilter("*.prm");
	}

	//! constructor, pass a pointer to the WalkMC which you want to save
	LoadWalkControl(const std::string& n, WalkMC * awalk)
		: FileBrowserControl(n,"Loads a set of walk parameters from a file specified by user",config->portPath(config->motion.root)), walk_id(invalid_MC_ID), thewalk(awalk)
	{
		setFilter("*.prm");
	}

	//! destructor
	virtual ~LoadWalkControl() {}

protected:
	//!does the actual loading of the MotionSequence
	virtual ControlBase* selectedFile(const std::string& f) {
		MC_ID id = thewalk==NULL?walk_id:thewalk->getID();
		WalkMC* walk=thewalk;
		if(id!=invalid_MC_ID)
			walk = (WalkMC*)motman->checkoutMotion(id);
		if(walk==NULL)
			serr->printf("Invalid walk for loading\n");
		else {
			walk->loadFile(f.c_str());
			if(id!=invalid_MC_ID)
				motman->checkinMotion(id);
		}
		return NULL;
	}

	MC_ID walk_id; //!< the MC_ID of the walk to load into
	WalkMC * thewalk; //!< the walk to load into (if NULL, check walk_id)

private:
	LoadWalkControl(const LoadWalkControl&); //!< don't call
	LoadWalkControl operator=(const LoadWalkControl&); //!< don't call
};

/*! @file
 * @brief Defines LoadWalkControl, which when activated, loads a set of walk parameters from a file read from cin.
 * @author ejt (Creator)
 */

#endif
