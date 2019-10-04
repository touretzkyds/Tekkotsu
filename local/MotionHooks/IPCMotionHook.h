//-*-c++-*-
#ifndef INCLUDED_IPCMotionHook_h_
#define INCLUDED_IPCMotionHook_h_

#include "local/MotionHook.h"
#include <list>

class MessageQueueBase;
class RCRegion;

//! description of IPCMotionHook
class IPCMotionHook : public MotionHook {
public:
	IPCMotionHook(MessageQueueBase& q, MessageQueueBase& pidq) : MotionHook(), mq(q), pidmq(pidq), regions() {}
	virtual ~IPCMotionHook();
	
	virtual bool isConnected() { return true; }
	
	virtual void motionCheck(const float outputs[][NumOutputs]);
	virtual void updatePIDs(const std::vector<PIDUpdate>& pids);
	
protected:
	MessageQueueBase& mq;
	MessageQueueBase& pidmq;

	static unsigned int count; //!< count of created regions
	RCRegion* getRegion();
	typedef std::list<RCRegion* > msgbuf_t; //!< type of collection of shared data regions
	msgbuf_t regions; //!< for efficiency, reuse old buffers -- oldest at front, most recently used at back
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
