#include "Process.h"
#include "SharedGlobals.h"
#include "Sound/SoundManager.h"
#include "IPC/SharedObject.h"
#include "Shared/MarkScope.h"
#include "Shared/debuget.h"
#include <unistd.h>

using namespace std;

Process* Process::procs[ProcessID::NumProcesses];

Process::Process(ProcessID::ProcessID_t pid, const std::string& pname) {
	ProcessID::setID(pid);
	procs[pid]=this;
	globals->pids[ProcessID::getID()]=getpid();
	strncpy(globals->processNames[ProcessID::getID()],pname.c_str(),SharedGlobals::MAX_PROCESS_NAME_LEN);
}

Process::~Process() {
	procs[ProcessID::getID()]=NULL;
}

void Process::run() {
	globals->waitShutdown();
}

void Process::statusReport(std::ostream& os) {
	os << getName() << " (" << ProcessID::getID() << ") Attached Regions -----------" << endl;
	RCRegion::attachedRegions_t::const_iterator it=RCRegion::attachedBegin(true);
	for(; it!=RCRegion::attachedEnd(); RCRegion::attachedAdvance(it)) {
		//subtract one from the reference counts to discount iterator's own reference (using thread-safe iterator access, uses a reference)
		unsigned int lref=(it->second->NumberOfLocalReference()-1);
		unsigned int ref=(it->second->NumberOfReference()-1);
		os << '\t' << setw(16) << left << it->first << setw(8) << right << it->second->Size() << " bytes" << setw(8) << lref<<'/'<<ref << " references" << endl;
	}
	os << '\t' << setw(16) << left << "Next RCRegion ID: " << setw(8) << right << RCRegion::getNextKey() << endl;
	os << '\t' << setw(16) << left << "Next ShdObj ID: " << setw(8) << right << SharedObjectBase::getNextKey() << endl;
	if(sndman!=NULL)
		os << '\t' << setw(16) << left << "Next SndRgn ID: " << setw(8) << right << sndman->getNextKey() << endl;
}

bool Process::statusReport(RCRegion* msg) {
	Process* cur=Process::getCurrent();
	ASSERTRETVAL(cur!=NULL,"statusReport, but NULL process!",true);
	if(strncasecmp(msg->Base(),getName(),msg->Size())==0 || strncasecmp(msg->Base(),"all",msg->Size())==0) {
		MarkScope l(globals->lock); //prevent jumbling reports
		cur->statusReport(cout);
		cout << endl;
	}
	return true;
}


/*! @file
 * @brief 
 * @author ejt (Creator)
 */
