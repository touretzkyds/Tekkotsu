//-*-c++-*-
#ifndef INCLUDED_Process_h_
#define INCLUDED_Process_h_

#include "IPC/ProcessID.h"
#include <string>

class RCRegion;

//! Represents a common interface for each process being run
class Process {
public:
	Process(ProcessID::ProcessID_t pid, const std::string& pname);
	virtual ~Process();
	virtual void start() { doStart(); }
	virtual void stop() { doStop(); }
	virtual void run();

	static const char* getName() { return ProcessID::getIDStr(); }
	static Process * getCurrent() { return procs[ProcessID::getID()]; }
	
	virtual void statusReport(std::ostream& os);
	static bool statusReport(RCRegion* msg);

protected:
	virtual void doStart() {}
	virtual void doStop() {}
	static Process* procs[ProcessID::NumProcesses];
	
private:
	Process(const Process&);            //!< don't call
	Process& operator=(const Process&); //!< don't call
};

/*! @file
 * @brief 
 * @author ejt (Creator)
 */

#endif
