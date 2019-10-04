#include "MotionExecThread.h"
#include "Simulator.h"
#include "SharedGlobals.h"
#include "Shared/ProjectInterface.h"
#include "Shared/WorldState.h"
#include "Shared/MarkScope.h"
#include "Motion/MotionManager.h"
#include "IPC/MessageQueue.h"
#include "Shared/debuget.h"
#include "Shared/Profiler.h"
#include "Shared/mathutils.h"

using namespace std; 

void MotionExecThread::reset() {
	MarkScope sl(startLock);
	globals->setNextMotion(getNextMotion());
	if(globals->timeScale<=0 && isStarted())
		stop();
	else if(globals->timeScale>0 && !isStarted())
		start();
	else if(isStarted()) {
		interrupt();
	}
}

/*void MotionExecThread::start() {
	lastPoll=-1U;
	launching=initialPoll=true;
	Thread::start(); //skipping PollThread::start() because we don't want to reset startTime
}*/

bool MotionExecThread::poll() {
	MarkScope l(motionLock);
	PROFSECTION("ReadySendJoints()",*motionProfiler);
	//this is OK: (can happen if the thread gets behind and the last one finished late, still want to do the current one, which appears to come early)
	//ASSERTRETVAL(get_time()>=globals->getNextMotion()-1,"MotionExecThread::poll() early (time="<<get_time()<< " vs. nextMotion=" <<globals->getNextMotion()<<")",true);
	if(get_time()<globals->getNextMotion())
		return true;
	
	{
		MarkScope sensorLock(globals->sensorState);
		state->read(globals->sensorState,false);
	}

	NoCancelScope nc;
	
	if(globals->motion.verbose>=3)
		cout << "Motion processing at " << get_time() << endl;
	const unsigned int bufTime=NumFrames*FrameTime;
	unsigned int tgtSize=(globals->motion.feedbackDelay>0 ? (globals->motion.feedbackDelay)/bufTime+1 : 1);
	if(motionBuffers.size()<tgtSize) {
		motionBuffers.insert(motionBufferPos,new float[NumFrames][NumOutputs]);
		motionBufferPos--;
	} else {
		while(motionBuffers.size()>tgtSize) {
			std::list<float(*)[NumOutputs]>::iterator tmp=motionBufferPos++;
			delete [] *tmp;
			motionBuffers.erase(tmp);
			if(motionBufferPos==motionBuffers.end())
				motionBufferPos=motionBuffers.begin();
		}
	}
	if(lastPoll!=-1U && motionBuffers.size()>1) {
		std::list<float(*)[NumOutputs]>::iterator prev=motionBufferPos;
		if(prev==motionBuffers.begin())
			prev=motionBuffers.end();
		prev--;
		unsigned int cnt=motionBuffers.size()-1;
		while(get_time()>=lastPoll+bufTime*2) {
			lastPoll+=bufTime;
			if(globals->motion.verbose>=2)
				cout << "Dropped motion frame in poll() (late call)" << endl;
			if(cnt==0)
				continue; //we've overwritten all of the buffer, no need wrap through it again
			memcpy(*motionBufferPos,*prev,sizeof(float[NumFrames][NumOutputs]));
			prev=motionBufferPos;
			if(++motionBufferPos==motionBuffers.end())
				motionBufferPos=motionBuffers.begin();
			cnt--;
		}
	}
	try {
		motman->getOutputs(*motionBufferPos);
	} catch(const std::exception& ex) {
		if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MotionManager processing",&ex))
			throw;
	} catch(...) {
		if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MotionManager processing",NULL))
			throw;
	}
	Simulator::updateMotion(*motionBufferPos);
	if(++motionBufferPos==motionBuffers.end())
		motionBufferPos=motionBuffers.begin();
	{
		std::vector<std::pair<unsigned int, float[3]> > pids;
		try {
			motman->updatePIDs(pids);
		} catch(const std::exception& ex) {
			if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MotionManager processing",&ex))
				throw;
		} catch(...) {
			if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MotionManager processing",NULL))
				throw;
		}
		if(pids.size()>0) {
			// update MotionHooks / Drivers
			std::vector<MotionHook::PIDUpdate> updates;
			updates.reserve(pids.size());
			std::copy(pids.begin(),pids.end(),std::back_inserter(updates));
			Simulator::updatePIDs(updates);
			
			// copy over feedback values
			MarkScope sslock(globals->sensorState);
			for(std::vector<MotionHook::PIDUpdate>::const_iterator it=updates.begin(); it!=updates.end(); ++it)
				memcpy(globals->sensorState.pids[it->idx], it->pids, sizeof(float)*3);
		}
	}
	globals->setNextMotion(getNextMotion());
	lastPoll=get_time();
	applyPostureFeedback();
	return true;
}

bool MotionExecThread::launched() {
	if(globals->timeScale<=0)
		return false;
	startTime.Set();
	globals->setNextMotion(getNextMotion());
	//reset startTime to last motion time
	startTime-=(get_time()-(getNextMotion()-FrameTime*NumFrames))/globals->timeScale/1000;
	interrupted();
	//delay=(getNextMotion()>get_time()) ? (getNextMotion()-get_time())/globals->timeScale/1000 : 0;
	return PollThread::launched();
}

/*void MotionExecThread::cancelled() {
	bool isFullSpeed=(globals->timeScale<0);
}*/

void MotionExecThread::interrupted() {
	period=FrameTime*NumFrames/globals->timeScale/1000;
	delay=(globals->getNextMotion()-get_time())/globals->timeScale/1000+startTime.Age();
	//cout << "interrupt " << get_time() << ' ' << globals->getNextMotion() << ' ' << startTime.Age() << ' ' << delay << ' ' << period << ' ' << isStarted() << ' ' << globals->timeScale << endl;
}

void MotionExecThread::applyPostureFeedback() {
	if(globals->motion.feedbackDelay<0)
		return ;
	if(!globals->motion.override && !globals->sensorState.hasUnprovidedOutput())
		return;
	
	const unsigned int bufTime=NumFrames*FrameTime;
	MarkScope l(motionLock);
	if(lastPoll!=-1U && motionBuffers.size()>1) {
		std::list<float(*)[NumOutputs]>::iterator prev=motionBufferPos;
		if(prev==motionBuffers.begin())
			prev=motionBuffers.end();
		prev--;
		unsigned int cnt=motionBuffers.size()-1;
		while(get_time()>=lastPoll+bufTime*2) {
			lastPoll+=bufTime;
			cout << "Dropped motion frame in applyPostureFeedback()" << endl;
			if(cnt==0)
				continue; //we've overwritten all of the buffer, no need wrap through it again
			memcpy(*motionBufferPos,*prev,sizeof(float[NumFrames][NumOutputs]));
			prev=motionBufferPos;
			if(++motionBufferPos==motionBuffers.end())
				motionBufferPos=motionBuffers.begin();
			cnt--;
		}
	}
	unsigned int bufDelay=globals->motion.feedbackDelay/bufTime;
	unsigned int frameDelay=(globals->motion.feedbackDelay-bufDelay*bufTime)/FrameTime;
	int bufAdvance=motionBuffers.size()-1-bufDelay;
	std::list<float(*)[NumOutputs]>::iterator tgt=motionBufferPos;
	if(bufAdvance<0) {
		//cout << "negative bufAdvance " << bufAdvance << ' ' << globals->motion.feedbackDelay << ' ' << motionBuffers.size() << endl;
		frameDelay=0;
	} else if(bufAdvance>0) {
		//cout << "positive bufAdvance " << bufAdvance << ' ' << globals->motion.feedbackDelay << ' ' << motionBuffers.size() << endl;
		while(bufAdvance-- > 0) {
			if(++tgt==motionBuffers.end())
				tgt=motionBuffers.begin();
		}
	}
	float * outputs = (*tgt)[NumFrames-1-frameDelay];
	MarkScope sslock(globals->sensorState);
	for(unsigned int i=0; i<NumOutputs; ++i) {
		if(globals->motion.override || globals->sensorState.providedOutputs[i]==0) {
			float v = outputs[i];
			if(globals->motion.feedbackRangeLimits)
				v = mathutils::limitRange<float>(v, outputRanges[i][MinRange], outputRanges[i][MaxRange]);
			globals->sensorState.outputs[i] = v;
		}
	}
}


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
