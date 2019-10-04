//-*-c++-*-
#ifndef INCLUDED_SimConfig_h_
#define INCLUDED_SimConfig_h_

#include "Shared/plist.h"
#include "SharedGlobals.h"
#include "IPC/RCRegion.h"

//! Provides the root dictionary of the simulator configuration, items from SharedGlobals and LoadFileThreads are added as entries in this dictionary
class SimConfig : public plist::Dictionary {
public:
	SimConfig() : plist::Dictionary(false),
		cmdPrompt("hal> "),
		initSimTime(0),
		tgtRunlevel(SharedGlobals::RUNNING, SharedGlobals::runlevel_names),
		multiprocess(false),
		lastfile()
	{
		sim::config.setUnusedWarning(false);
		addEntry("InitialTime",initSimTime,"The value to initialize the simulator's clock (in milliseconds)");
		addEntry("InitialRunlevel",tgtRunlevel,"Specifies how far startup should proceed before pausing for user interaction.\nThis value only affects startup, and setting this value from the simulator command prompt will have no effect.  (Use the 'runlevel' command instead.)");
		addEntry("Multiprocess",multiprocess,"The processing/threading model to use - true to use real process forks a la Aibo/Aperios, or false to just more threads like a sane person would do");
	}
	
	std::string cmdPrompt; //!< Not persistently stored -- [re]set by main(...) on each run
	plist::Primitive<unsigned int> initSimTime; //!< The "boot" time to start the simulator clock at (default 0)
	plist::NamedEnumeration<SharedGlobals::runlevel_t> tgtRunlevel; //!< The runlevel the simulator should move to (i.e. stop before 'running' to debug startup code)
	plist::Primitive<bool> multiprocess; //!< The processing/threading model to use -- true to use real process forks a la Aibo/Aperios, or false to just more threads like a sane person would do
	
	void setLastFile(const std::string& str) const {
		lastfile=str;
	}
	const std::string& getLastFile() const {
		return lastfile;
	}
	virtual unsigned int loadFile(const char* filename) {
		lastfile=filename;
		return plist::Dictionary::loadFile(filename);
	}
	virtual unsigned int saveFile(const char* filename) const {
		lastfile=filename;
		return plist::Dictionary::saveFile(filename);
	}
	
protected:
	mutable std::string lastfile;
};

/*! @file
 * @brief Provides the root dictionary of the simulator configuration, items from SharedGlobals and LoadFileThreads are added as entries in this dictionary
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
