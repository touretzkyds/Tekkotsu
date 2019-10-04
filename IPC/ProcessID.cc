#include "ProcessID.h"
#include <cstddef>

#ifdef DEBUG_STACKTRACE
#  include "Shared/WorldState.h"
#endif

#ifdef PLATFORM_APERIOS
#  include <iostream>
using namespace stacktrace;
#endif

using namespace std;

namespace ProcessID {
	ProcessID_t ID=NumProcesses; //!< holds ID number
	
#ifndef PLATFORM_APERIOS
	ProcessID_t getUnhookedID() { return ID; }
	void setUnhookedID(ProcessID_t id) { ID=id; }
	
	ProcessID_t (*getIDHook)()=&getUnhookedID; //!< hook allows overriding the ID system, set with ProcessID::setIDHooks
	void (*setIDHook)(ProcessID_t id)=&setUnhookedID; //!< hook allows overriding the ID system, set with ProcessID::setIDHooks
	
	// on "normal" platforms we can trust the id specified by the process to stay consistent
	ProcessID_t getID() { return (*getIDHook)(); }
	
	void setID(ProcessID_t id) { (*setIDHook)(id); }

	void setIDHooks(ProcessID_t (*customGetID)(), void (*customSetID)(ProcessID_t id)) {
		getIDHook = (customGetID==NULL) ? &getUnhookedID : customGetID;
		setIDHook = (customSetID==NULL) ? &setUnhookedID : customSetID;
	}

#else
	// but on the Aibo, we have to unroll the stack to see which thread it is

	//! array of StackFrame structures, one per #NumProcesses, set by setMap()
	StackFrame* frames=NULL;
	
	//! 
	void setMap(stacktrace::StackFrame idmap[]) {
		frames=idmap;
	}
	
	stacktrace::StackFrame* getMapFrame() {
		if(frames==NULL) //setMap hasn't been called yet
			return NULL;
		if(ID==NumProcesses) // ID hasn't been set
			return NULL;
		return &frames[ID];
	}


	ProcessID_t getID() {
		if(frames==NULL) { //setMap hasn't been called yet
			//cerr << "getID() called before setMap() id==" << ID << endl;
			//displayCurrentStackTrace();
			return ID;
		}
		StackFrame f;
#ifdef DEBUG_STACKTRACE
		f.debug=(state!=NULL)?(state->buttons[LFrPawOffset]>.1):1;
		if(f.debug)
			fprintf(stderr,"getID() for %d: ",ID);
#endif
		getCurrentStackFrame(&f);
		while(unrollStackFrame(&f,&f)) {}
		for(unsigned int i=0; i<NumProcesses; i++) {
			if(frames[i].gp==f.gp) {
#ifdef DEBUG_STACKTRACE
				if(i!=(unsigned int)ID || f.debug)
					cout << "getID() from " << ID << " is " << i << endl;
#endif
				return static_cast<ProcessID_t>(i);
			}
		}
		cerr << "ERROR: Unknown entry point (sp=" << f.sp << ",ra=" << (void*)f.ra << ",gp=" << (void*)f.gp << "), implied process " << ID << endl;
		displayCurrentStackTrace();
		cerr << "Map:" << endl;
		for(unsigned int i=0; i<NumProcesses; i++)
			cerr << "  " << i << " (sp=" << frames[i].sp << ",ra=" << (void*)frames[i].ra << ",gp=" << (void*)frames[i].gp << ")" << endl;
		return ID;
	}
	
	void setID(ProcessID_t id) { ID=id; }

#endif
}



/*! @file
 * @brief Declares the static ProcessID::ID, that's all
 * @author ejt (Creator)
 */

