#include "PostureMC.h"
#include "Shared/Config.h"
#include "Shared/WorldState.h"
#include "Shared/ERS7Info.h"
#include "Shared/ERS210Info.h"
#include "Wireless/Wireless.h"
#include "Shared/get_time.h"
#include "Events/EventBase.h"

PostureMC& PostureMC::setDirty(bool d/*=true*/) {
	dirty=d;
	targetReached=false;
	for(unsigned int i=0; i<NumOutputs; i++)
		curPositions[i]=motman->getOutputCmd(i).value; //not state->outputs[i]; - see function documentation
	return *this;
} 

void PostureMC::defaultMaxSpeed(float x/*=1*/) {
	for(unsigned int i=0; i<NumOutputs; i++)
		maxSpeeds[i]=MaxOutputSpeed[i]*FrameTime*x/1000.f; // maxSpeeds is radians per frame
	//respect the config values for the neck joints (which are stored as rad/sec)
#ifdef TGT_IS_AIBO
	maxSpeeds[HeadOffset+TiltOffset]=config->motion.max_head_tilt_speed*FrameTime*x/1000.f; 
	maxSpeeds[HeadOffset+PanOffset]=config->motion.max_head_pan_speed*FrameTime*x/1000.f;
	maxSpeeds[HeadOffset+RollOffset]=config->motion.max_head_roll_speed*FrameTime*x/1000.f;
#else
	const char* n = ERS7Info::outputNames[ERS7Info::HeadOffset+ERS7Info::TiltOffset];
	unsigned int i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		maxSpeeds[i]=config->motion.max_head_tilt_speed*FrameTime*x/1000.f; 
	n = ERS7Info::outputNames[ERS7Info::HeadOffset+ERS7Info::PanOffset];
	i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		maxSpeeds[i]=config->motion.max_head_pan_speed*FrameTime*x/1000.f;
	n = ERS7Info::outputNames[ERS7Info::HeadOffset+ERS7Info::NodOffset];
	i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		maxSpeeds[i]=config->motion.max_head_roll_speed*FrameTime*x/1000.f;
	n = ERS210Info::outputNames[ERS210Info::HeadOffset+ERS210Info::RollOffset];
	i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		maxSpeeds[i]=config->motion.max_head_roll_speed*FrameTime*x/1000.f;
#endif
}

int PostureMC::updateOutputs() {
	int tmp=isDirty();
	if(tmp || hold) {
		dirty=false;
		for(unsigned int i=0; i<NumOutputs; i++) {
			if(maxSpeeds[i]<=0) {
				curPositions[i]=cmds[i].value;
				motman->setOutput(this,i,cmds[i]);
			} else { // we may be trying to exceeded maxSpeed
				unsigned int f=0;
				while(cmds[i].value>curPositions[i]+maxSpeeds[i] && f<NumFrames) {
					curPositions[i]+=maxSpeeds[i];
					motman->setOutput(this,i,OutputCmd(curPositions[i],cmds[i].weight),f);
					f++;
				}
				while(cmds[i].value<curPositions[i]-maxSpeeds[i] && f<NumFrames) {
					curPositions[i]-=maxSpeeds[i];
					motman->setOutput(this,i,OutputCmd(curPositions[i],cmds[i].weight),f);
					f++;
				}
				if(f<NumFrames) { //we reached target value, fill in rest of frames
					curPositions[i]=cmds[i].value;
					for(;f<NumFrames;f++)
						motman->setOutput(this,i,cmds[i],f);
				} else // we didn't reach target value, still dirty
					dirty=true;
			}
		}
		if(!dirty && !targetReached) {
			postEvent(EventBase(EventBase::motmanEGID,getID(),EventBase::statusETID));
			targetReached=true;
			targetTimestamp=get_time();
		}
	}
	return tmp;
}

int PostureMC::isAlive() {
	if(dirty || !targetReached)
		return true;
	if(targetReached && (!hold || get_time()-targetTimestamp>timeout)) { //prevents a conflicted PostureMC's from fighting forever
		if(get_time()-targetTimestamp>timeout && getAutoPrune())
			serr->printf("WARNING: posture timed out - possible joint conflict or out-of-range target\n");
		return false;
	}
	float max=0;
	for(unsigned int i=0; i<NumOutputs; i++)
		if(cmds[i].weight>0) {
			float dif=cmds[i].value-state->outputs[i];
			if(dif>max)
				max=dif;
		}
	return (max>tolerance);
}

void PostureMC::init() {
	defaultMaxSpeed();
	setDirty();
}

/*! @file
 * @brief Implements PostureMC, a MotionCommand shell for PostureEngine
 * @author ejt (Creator)
 */
