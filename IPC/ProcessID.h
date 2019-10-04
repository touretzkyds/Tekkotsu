//-*-c++-*-
#ifndef INCLUDED_ProcessID_h_
#define INCLUDED_ProcessID_h_

#ifdef PLATFORM_APERIOS
#  include "Shared/StackTrace.h"
#endif

//! holds information to identify currently running process
/*!
 * Although the ProcessID_t enum specifies the maximum number of processes
 * in its NumProcesses value, this doesn't have to correspond to the actual
 * number of active processes.  If you want an element of dynamicism to launch
 * and/or close processes at runtime, simply increase the NumProcesses to a 
 * higher value so that space can be allocated for those processes in shared 
 * memory regions for things like attachment pointers or other per-process
 * data.
 *
 * Not all processes need to have a *named* id, they just need a unique id.
 */
namespace ProcessID {
	//! Holds ID number for each process
	enum ProcessID_t {
		MainProcess,   //!< MainObj process
		MotionProcess, //!< MotoObj process
		SoundProcess,  //!< SoundPlay process
#ifndef PLATFORM_APERIOS
		SimulatorProcess, //!< for interaction with OS and managing global environment
#endif
		NumProcesses   //!< maximum number of 'friendly' processes -- see class docs
	};
	
	ProcessID_t getID();  //!< returns process's ID number, subject to overriding via setIDHooks()
	void setID(ProcessID_t id); //!< sets the ID during init (be careful you know what you're doing if you call this), subject to overriding via setIDHooks()
	
#ifndef PLATFORM_APERIOS
	//! allows you to override the behavior of getID and setID -- pass NULL to use default behavior
	void setIDHooks(ProcessID_t (*customGetID)(), void (*customSetID)(ProcessID_t id)); 

	//! returns process's ID number from static global, the default behavior if setIDHooks was passed NULL
	ProcessID_t getUnhookedID();
	
	//! sets the ID into static global (be careful you know what you're doing if you call this), this is the default behavior if setIDHooks was passed NULL
	void setUnhookedID(ProcessID_t id);
#endif
	
	//! returns a string version of the name of the process
	inline const char* getIDStr(ProcessID_t pid) {
		switch(pid) {
			case MainProcess: return "Main";
			case MotionProcess: return "Motion";
			case SoundProcess: return "Sound";
#ifndef PLATFORM_APERIOS
			case SimulatorProcess: return "Simulator";
#endif
			default: return "Invalid Process";
		}
	}
	
	//! returns a string version of the name of the current process
	inline const char* getIDStr() { return getIDStr(getID()); }
	
#ifdef PLATFORM_APERIOS
	//! sets location of shared memory map from IDs to current entry point, this is required to be set up before any entry()'s
	/*! @param idmap array of StackFrame structures, one per #NumProcesses
	 *  the idea is to have idmap stored in a shared memory region, so functions can tell which one they belong to */
	void setMap(stacktrace::StackFrame idmap[]);
	
	//! returns the stack frame which should be set to the entry point of the current function
	/*! this is only valid if it is called before any shared object processing is done */
	stacktrace::StackFrame* getMapFrame();
#endif
}

/*! @file
 * @brief Defines ProcessID - simple little global for checking which process is currently running, kind of. (see ProcessID::getID() )
 * @author ejt (Creator)
 */

#endif
