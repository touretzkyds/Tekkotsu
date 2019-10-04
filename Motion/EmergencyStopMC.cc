#include "EmergencyStopMC.h"
#include "Shared/WorldState.h"
#include "Shared/get_time.h"
#include "Motion/MotionManager.h"
#include "Sound/SoundManager.h"
#include "Shared/Config.h"
#include "Events/EventRouter.h"
#include "Shared/RobotInfo.h"
#include "Shared/ERS210Info.h"
#include "Shared/ERS220Info.h"
#include "Wireless/Wireless.h"

EmergencyStopMC::EmergencyStopMC()
	: PostureMC(), paused(false), stilldown(false), active(true), period(2000),
		timeoflastbtn(0), timeofthisbtn(0), timeoflastfreeze(0), timeoflastrelease(0), duration(600),
		pidcutoff(0.2f)
#ifdef TGT_HAS_LEDS
		, ledengine()
#endif
{
	for(unsigned int i=0; i<NumPIDJoints; i++)
		piddutyavgs[i]=0;
#if defined(TGT_ERS2xx) || defined(TGT_ERS210) || defined(TGT_ERS220)
	// because of the dual-boot ability, these models have to test dynamically
	// (TGT_ERS210 and TGT_ERS220 could make the assumption, but don't want to repeat the code...)
	if(RobotName == ERS210Info::TargetName) {
		int red=capabilities.getOutputOffset(ERS210Info::outputNames[ERS210Info::TlRedLEDOffset]) - LEDOffset;
		int blue=capabilities.getOutputOffset(ERS210Info::outputNames[ERS210Info::TlBluLEDOffset]) - LEDOffset;
		ledengine.cycle((1<<red),period,1,0,period/2);
		ledengine.cycle((1<<blue),period,1);
	} else if(RobotName == ERS220Info::TargetName) {
		int tl=capabilities.getOutputOffset(ERS220Info::outputNames[ERS220Info::TailLeftLEDOffset]) - LEDOffset;
		int tc=capabilities.getOutputOffset(ERS220Info::outputNames[ERS220Info::TailCenterLEDOffset]) - LEDOffset;
		int tr=capabilities.getOutputOffset(ERS220Info::outputNames[ERS220Info::TailRightLEDOffset]) - LEDOffset;
		int bl1=capabilities.getOutputOffset(ERS220Info::outputNames[ERS220Info::BackLeft1LEDOffset]) - LEDOffset;
		int bl2=capabilities.getOutputOffset(ERS220Info::outputNames[ERS220Info::BackLeft2LEDOffset]) - LEDOffset;
		int bl3=capabilities.getOutputOffset(ERS220Info::outputNames[ERS220Info::BackLeft3LEDOffset]) - LEDOffset;
		int br1=capabilities.getOutputOffset(ERS220Info::outputNames[ERS220Info::BackRight1LEDOffset]) - LEDOffset;
		int br2=capabilities.getOutputOffset(ERS220Info::outputNames[ERS220Info::BackRight2LEDOffset]) - LEDOffset;
		int br3=capabilities.getOutputOffset(ERS220Info::outputNames[ERS220Info::BackRight3LEDOffset]) - LEDOffset;
		ledengine.cycle((1<<tc), period, 2.0f, -.5f, (int)(period * 0/5.5));
		ledengine.cycle((1<<tl)|(1<<tr),   period, 2.0f, -.5f, (int)(period * 1/5.5));
		ledengine.cycle((1<<bl3)|(1<<br1), period, 2.0f, -.5f, (int)(period * 2/5.5));
		ledengine.cycle((1<<bl2)|(1<<br2), period, 2.0f, -.5f, (int)(period * 3/5.5));
		ledengine.cycle((1<<bl1)|(1<<br3), period, 2.0f, -.5f, (int)(period * 4/5.5));
	}
#elif defined(TGT_ERS7)
	ledengine.cycle(MdBackColorLEDMask,2*period/3,.15f,.15f/2-.5f,0);
#elif defined(TGT_IS_QWERK)
	ledengine.cycle(1<<8,period,1,0,period/2);
	ledengine.cycle(1<<9,period,1);
#elif defined(TGT_IS_CREATE) || defined(TGT_IS_CREATE2)
	int red = RobotInfo::PowerRedLEDOffset - LEDOffset;
	int green = RobotInfo::PowerGreenLEDOffset - LEDOffset;
	ledengine.cycle(1<<red,period/2,1);
	ledengine.cycle(1<<green,period/2,1,0,period/4);
#elif defined(TGT_CHIARA)
	ledengine.cycle(ChiaraInfo::RedLEDOffset, 2*period/3, 5.0f);
#elif defined(TGT_CHIARA2)
	ledengine.cycle(Chiara2Info::AllLEDMask, 2*period/3, 5.0f);
#elif defined(TGT_HANDEYE) || defined(TGT_HANDEYEZ)
#elif defined(TGT_HAS_LEDS)
#  warning EmergencyStop using generic LED special effects (last two LEDs will alternate)
	ledengine.cycle(1<<(NumLEDs-1),period,1,0,period/2);
#  if TGT_HAS_LEDS>1
	ledengine.cycle(1<<(NumLEDs-2),period,1);
#  endif
#endif
	defaultMaxSpeed(.15f);
	takeSnapshot();
}


int EmergencyStopMC::updateOutputs() {
#if defined(TGT_IS_CREATE) || defined(TGT_IS_KOBUKI) || defined(TGT_IS_CREATE2)
	if(trigger()) {
		setStopped(true);
	}
#else
	if(trigger()) {
		if(!stilldown) {
			stilldown=true;
			timeoflastbtn=timeofthisbtn;
			timeofthisbtn=get_time();
			//			cout << "Press " << timeofthisbtn << ' ' << timeoflastbtn << endl;
		}
	//			cout << "Down" << endl;
	} else if(stilldown) {
	//			cout << "Release " << get_time() << endl;
		stilldown=false;
		if((get_time()-timeoflastbtn)<duration)
			setStopped(!paused);
	}
#endif
	unsigned int curt=get_time();
	dirty=dirty || (curt<timeoflastrelease);
	if(!paused) {
		if(!dirty)
			return 0;
		if(curt>=timeoflastrelease) {
#ifdef TGT_HAS_LEDS
			for(unsigned int i=LEDOffset; i<LEDOffset+NumLEDs; i++)
				motman->setOutput(this,i,0.f); //blank out LEDs to avoid residual background display
#endif
#if defined(TGT_IS_CREATE) || defined(TGT_IS_CREATE2)
			motman->setOutput(this,RobotInfo::PowerRedLEDOffset,OutputCmd(0.0f, 0.5f));
			motman->setOutput(this,RobotInfo::PowerGreenLEDOffset,OutputCmd(0.5f, 0.5f));
#endif
			dirty=false;
			return 0;
		}
		float w = (curt>=timeoflastrelease) ? 0 : (static_cast<float>(timeoflastrelease-curt)/FADE_OUT_TIME);
		for(unsigned int i=0; i<NumOutputs; i++)
			cmds[i].weight=w;		
	} else {
		//immediately following a pause, just hold current position at first to prevent twitching if we were in motion
		if(curt-timeoflastfreeze>FrameTime*NumFrames*5) {
			//once joints have come to rest, respond to outside forces
			for(unsigned int i=0; i<NumPIDJoints; i++) {
				//exponential average of duty cycles to filter out noise
				piddutyavgs[i]=piddutyavgs[i]*.9f+state->pidduties[i]*.1f;
				//reset if there's something significantly out of place (perhaps we're being overridden)
				if(fabsf(state->outputs[PIDJointOffset+i]-cmds[PIDJointOffset+i].value)>.15f) {
					//if(PIDJointOffset+i==LFrLegOffset+RotatorOffset)
					//cout << "resetting from " << cmds[PIDJointOffset+i].value << " to " << state->outputs[PIDJointOffset+i] << endl;
					curPositions[PIDJointOffset+i]=cmds[PIDJointOffset+i].value=state->outputs[PIDJointOffset+i];
					dirty=true;
					targetReached=false;
				}
				//give if there's a force...
				if(fabsf(piddutyavgs[i])>pidcutoff) {
					cmds[PIDJointOffset+i].value-=piddutyavgs[PIDJointOffset+i]; //move in the direction of the force
					dirty=true;
					targetReached=false;
				}
			}
		}
	}
#ifdef TGT_HAS_LEDS
	ledengine.updateLEDs(&cmds[LEDOffset]);
#endif
#if defined(TGT_ERS7)
	//a special Battlestar Galactica inspired effect for the ERS-7
	static float acts[5];
	static bool wasPaused=false;
	if(!wasPaused && paused) {
		for(int i=0; i<5; i++)
			acts[i]=0;
		wasPaused=paused;
	}
	float t=curt;
	t/=period;
	t=(((int)t)&1)?(int)t+1-t:(t-(int)t);
	t*=8;
	const float invsigma=-6;
	const float gamma=.83f;
	const float amp=.5f;
	float imp[5];
	// w is used to fade out LEDs when releasing estop
	float w = (paused || curt>=timeoflastrelease) ? 1 : (static_cast<float>(timeoflastrelease-curt)/FADE_OUT_TIME);
	for(int i=0; i<5; i++) {
		float p=invsigma*(t-i-2)*(t-i-2);
		if(p>-10) { // only bother with impulse if big enough
			// (in particular, saw exp returning -inf instead of 0 for <-85... bug in libm?)
			imp[i]=expf(p)*w;
			acts[i]+=amp*imp[i];
		} else {
			imp[i]=0;
		}
		acts[i]*=gamma*w;
	}
	cmds[ERS7Info::FaceLEDPanelOffset+ 6]=acts[0]/2+imp[0];
	cmds[ERS7Info::FaceLEDPanelOffset+ 8]=acts[1]/2+imp[1];
	cmds[ERS7Info::FaceLEDPanelOffset+10]=acts[2]/2+imp[2];
	cmds[ERS7Info::FaceLEDPanelOffset+ 9]=acts[3]/2+imp[3];
	cmds[ERS7Info::FaceLEDPanelOffset+ 7]=acts[4]/2+imp[4];
#elif defined(TGT_IS_CREATE) || defined(TGT_IS_CREATE2)
	if (!paused) {
		cmds[RobotInfo::PowerRedLEDOffset].set(0.2f,1);
		cmds[RobotInfo::PowerGreenLEDOffset].set(1,1);
	}
#endif
	int changed=PostureMC::updateOutputs();
	dirty=(curt<timeoflastrelease);
	return changed;
}

void EmergencyStopMC::setActive(bool a) {
	if(paused) {
		if(!a && active)
			releaseJoints();
		else if(a && !active)
			freezeJoints();
	}
	active=a;
}


void EmergencyStopMC::setStopped(bool p, bool sound) {
	if(p!=paused) {
		paused=p;
		if(active) {
			if(paused) {
				freezeJoints();
				if(sound)
					sndman->playFile(config->motion.estop_on_snd);
				std::cout << "*** PAUSED ***" << std::endl;
			} else {
				releaseJoints();
				if(sound)
					sndman->playFile(config->motion.estop_off_snd);
				std::cout << "*** UNPAUSED ***" << std::endl;
			}
		}
	}
}

void EmergencyStopMC::freezeJoints() {
	dirty=true;
	targetReached=false;
	for(unsigned int i=0; i<NumOutputs; i++) {
		OutputCmd c=motman->getOutputCmd(i);
		curPositions[i]=cmds[i].value = (c.weight==0) ? state->outputs[i] : c.value;
	}
	for(unsigned int i=0; i<NumPIDJoints; i++)
		piddutyavgs[i]=0; //or: state->pidduties[i];
#ifdef TGT_HAS_WHEELS
	//Wheels need to be set to 0 in e-stop mode
	for(unsigned int i=WheelOffset; i<WheelOffset+NumWheels; i++) {
	  cmds[i].value = 0;
	  curPositions[i] = 0;
	}
#endif
#ifndef TGT_HAS_LEDS
	// no LEDs, just go all the way through in one pass...
	for(unsigned int i=0; i<NumOutputs; i++)
		cmds[i].weight=1;
#else
	for(unsigned int i=0; i<LEDOffset; i++)
		cmds[i].weight=1;
	for(unsigned int i=LEDOffset; i<LEDOffset+NumLEDs; i++)
		cmds[i].unset(); // let other commands' LEDs "show through"
	for(unsigned int i=LEDOffset+NumLEDs; i<NumOutputs; i++)
		cmds[i].weight=1;
#endif
#if defined(TGT_ERS2xx) || defined(TGT_ERS210) || defined(TGT_ERS220)
	// because of the dual-boot ability, these models have to test dynamically
	// (TGT_ERS210 and TGT_ERS220 could make the assumption, but don't want to repeat the code...)
	if(RobotName == ERS210Info::TargetName) {
		int red=capabilities.getOutputOffset(ERS210Info::outputNames[ERS210Info::TlRedLEDOffset]);
		int blue=capabilities.getOutputOffset(ERS210Info::outputNames[ERS210Info::TlBluLEDOffset]);
		cmds[red].set(0,.5f);
		cmds[blue].set(0,.5f);
	} else if(RobotName == ERS220Info::TargetName) {
		int tl=capabilities.getOutputOffset(ERS220Info::outputNames[ERS220Info::TailLeftLEDOffset]);
		int tc=capabilities.getOutputOffset(ERS220Info::outputNames[ERS220Info::TailCenterLEDOffset]);
		int tr=capabilities.getOutputOffset(ERS220Info::outputNames[ERS220Info::TailRightLEDOffset]);
		int bl1=capabilities.getOutputOffset(ERS220Info::outputNames[ERS220Info::BackLeft1LEDOffset]);
		int bl2=capabilities.getOutputOffset(ERS220Info::outputNames[ERS220Info::BackLeft2LEDOffset]);
		int bl3=capabilities.getOutputOffset(ERS220Info::outputNames[ERS220Info::BackLeft3LEDOffset]);
		int br1=capabilities.getOutputOffset(ERS220Info::outputNames[ERS220Info::BackRight1LEDOffset]);
		int br2=capabilities.getOutputOffset(ERS220Info::outputNames[ERS220Info::BackRight2LEDOffset]);
		int br3=capabilities.getOutputOffset(ERS220Info::outputNames[ERS220Info::BackRight3LEDOffset]);
		cmds[tl].set(0, .5f); cmds[tc].set(0, .5f); cmds[tr].set(0, .5f);
		cmds[bl1].set(0, .5f); cmds[bl2].set(0, .5f); cmds[bl3].set(0, .5f);
		cmds[br1].set(0, .5f); cmds[br2].set(0, .5f); cmds[br3].set(0, .5f);
	}
#elif defined(TGT_ERS7)
	cmds[MdBackColorLEDOffset].set(0,.5f);
	for(int i=6; i<6+5; i++)
		cmds[FaceLEDPanelOffset+i].set(0,0.5f);
#elif defined(TGT_CHIARA)
	cmds[ChiaraInfo::RedLEDOffset].set(0,.5f);
#elif defined(TGT_CHIARA2)
	cmds[Chiara2Info::AllLEDMask].set(0,.5f);
#elif defined(TGT_IS_QWERK)
	cmds[LEDOffset+8].set(0,.5f); // last two 'front' LEDs
	cmds[LEDOffset+9].set(0,.5f);
#  if defined(TGT_QBOTPLUS)
	cmds[LEDOffset+10].set(1,1); // first digital out: extra red LED on the side
#  endif
#elif defined(TGT_IS_CREATE) | defined(TGT_IS_CREATE2)
	cmds[RobotInfo::PowerRedLEDOffset].set(0,.5f);
	cmds[RobotInfo::PowerGreenLEDOffset].set(0,.5f);
#elif defined(TGT_HAS_LEDS)
	cmds[LEDOffset+NumLEDs-1].set(0,.5f);
#  if TGT_HAS_LEDS>1
	cmds[LEDOffset+NumLEDs-2].set(0,.5f);
#  endif
#endif
	postEvent(EventBase(EventBase::estopEGID,getID(),EventBase::activateETID,0));
	timeoflastfreeze=get_time();
}

void EmergencyStopMC::releaseJoints() {
	dirty=true;
	targetReached=false;
	unsigned int curt=get_time();
	timeoflastrelease=curt+FADE_OUT_TIME;
	postEvent(EventBase(EventBase::estopEGID,getID(),EventBase::deactivateETID,curt-timeoflastfreeze));
}

bool EmergencyStopMC::trigger() {
	const WorldState * st=WorldState::getCurrent(); // this is need because trigger is a static, so it doesn't have access to the MC 'state' instance
#if defined(TGT_ERS2xx) || defined(TGT_ERS210) || defined(TGT_ERS220)
	if(RobotName == ERS210Info::TargetName)
		return st->button_times[capabilities.getButtonOffset(ERS210Info::buttonNames[ERS210Info::BackButOffset])];
	if(RobotName == ERS220Info::TargetName)
		return st->button_times[capabilities.getButtonOffset(ERS220Info::buttonNames[ERS220Info::BackButOffset])];
#elif defined(TGT_ERS7)
	return st->button_times[ERS7Info::FrontBackButOffset]+st->button_times[ERS7Info::MiddleBackButOffset]+st->button_times[ERS7Info::RearBackButOffset];
#elif defined(TGT_IS_QWERK)
	return st->button_times[0];
#elif defined(TGT_IS_CREATE) || defined(TGT_IS_CREATE2)
	return (st->sensors[RobotInfo::ModeStateOffset] == RobotInfo::MODE_PASSIVE);
#elif defined(TGT_IS_KOBUKI)
	return(st->buttons[RobotInfo::DropLeftWheelButOffset] > 0 || st->buttons[RobotInfo::DropRightWheelButOffset] > 0);
#elif defined(TGT_IS_CHIARA)
	return st->button_times[RedButOffset];
#else
#  if defined(TGT_HAS_BUTTONS)
#    warning Target model has buttons, but EmergencyStop does not know which to use for the trigger
#  endif
	(void)st; // just to avoid unused variable warning
#endif
	return false;
}

/*! @file
 * @brief Implements EmergencyStopMC, overrides all joints, allows modelling, blinks tail
 * @author ejt (Creator)
 */

