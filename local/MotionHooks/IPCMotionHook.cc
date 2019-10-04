#include "IPCMotionHook.h"
#include "IPC/MessageQueue.h"
#include <sstream>

using namespace std;

unsigned int IPCMotionHook::count=0;

IPCMotionHook::~IPCMotionHook() {
	for(msgbuf_t::iterator it=regions.begin();it!=regions.end(); ++it)
		(*it)->RemoveReference();
}

void IPCMotionHook::motionCheck(const float outputs[][NumOutputs]) {
	RCRegion * r = getRegion();
	memcpy(r->Base(),outputs,r->Size());
	mq.sendMessage(r);
	regions.push_back(r);
}

void IPCMotionHook::updatePIDs(const std::vector<PIDUpdate>& pids) {
	stringstream ss;
	ss << "MotionPIDUpdate." << count++;
	//cout << "Created " << ss.str() << endl;
	RCRegion * r = new RCRegion(ss.str(),pids.size()*sizeof(PIDUpdate));
	memcpy(r->Base(),&pids[0],r->Size());
	pidmq.sendMessage(r,true);
}

RCRegion* IPCMotionHook::getRegion() {
	for(msgbuf_t::iterator it=regions.begin();it!=regions.end(); ++it) {
		if((*it)->NumberOfReference()==1) {
			RCRegion * ans=*it;
			regions.erase(it);
			return ans;
		}
	}
	// no unused regions found, make a new one
	stringstream ss;
	ss << "MotionUpdate." << count++;
	//cout << "Created " << ss.str() << endl;
	return new RCRegion(ss.str(),sizeof(float)*NumOutputs*NumFrames);
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
