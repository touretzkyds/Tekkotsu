//-*-c++-*-
#ifndef INCLUDED_TailWagMC_h
#define INCLUDED_TailWagMC_h

#include "Motion/MotionCommand.h"
#include "Motion/MotionManager.h"
#include "Shared/get_time.h"
#include "Shared/ERS7Info.h"
#include "Shared/ERS210Info.h"
#include "Shared/WorldState.h"
#include <cmath>

//! A simple motion command for wagging the tail - you can specify period, magnitude, and tilt
class TailWagMC : public MotionCommand {
public:

	//!constructor
	TailWagMC() :
		MotionCommand(), period(500), magnitude(22*(float)M_PI/180), 
		offset(0), active(true), last_pan(0), last_sign(0), tilt() { }

	//!constructor
	TailWagMC(unsigned int cyc_period, float cyc_magnitude) :
		MotionCommand(), period(cyc_period), magnitude(cyc_magnitude), 
		offset(0), active(true), last_pan(0), last_sign(0), tilt() { }

	//!destructor
	virtual ~TailWagMC() {}

	virtual int updateOutputs() {
		if(!active)
			return 0;

		// only ERS-210 and ERS-7 have moveable tails
		unsigned int tail_pan_offset=-1U, tail_tilt_offset=-1U;
		if(RobotName == ERS210Info::TargetName) {
			// might be in 2xx compatability mode, use dynamic lookup
			unsigned tail = capabilities.getOutputOffset(ERS210Info::outputNames[ERS210Info::TailOffset]);
			tail_pan_offset = tail+PanOffset;
			tail_tilt_offset = tail+TiltOffset;
		}
		else if(RobotName == ERS7Info::TargetName) {
			// no compatability mode, just use direct symbol:
			tail_pan_offset = ERS7Info::TailOffset+PanOffset;
			tail_tilt_offset = ERS7Info::TailOffset+TiltOffset;
		} else {
			// unknown model, see if we have a tail named after the ERS-210 tail:
			tail_pan_offset = capabilities.getOutputOffset(ERS210Info::outputNames[ERS210Info::TailOffset+PanOffset]);
			if(tail_pan_offset!=-1U) {
				// try the tilt too...
				tail_tilt_offset = capabilities.getOutputOffset(ERS210Info::outputNames[ERS210Info::TailOffset+TiltOffset]);
			} else {
				// maybe just named "tail"?
				tail_pan_offset = capabilities.getOutputOffset("Tail");
			}
		}
		if(tail_pan_offset==-1U)
			return 0;

		for(unsigned int i=0; i<NumFrames; i++) {
			float new_pan = std::sin((2*(float)M_PI*(get_time()+offset+i*FrameTime))/period)*magnitude; //bug fix thanks L.A.Olsson AT herts ac uk
			pans[i].set(new_pan);
			if ( (new_pan-last_pan >= 0 && last_sign == -1) ||
			     (new_pan-last_pan < 0 && last_sign == +1) )
				postEvent(EventBase(EventBase::motmanEGID,getID(),EventBase::statusETID));
			last_sign = new_pan-last_pan >= 0 ? 1 : -1;
			last_pan = new_pan;
		}
		motman->setOutput(this,tail_pan_offset,pans);
		if(tail_tilt_offset!=-1U)
			motman->setOutput(this,tail_tilt_offset,tilt);
		return tilt.weight>0?2:1;
	}

	virtual int isDirty() { return active; }
	virtual void setDirty() { active=true; }

	virtual int isAlive() { return true; }

	//! sets the period of time between swings, in milliseconds
	/*! a bit complicated in order to avoid jerking around when the period changes */
	void setPeriod(unsigned int p) {
		float prevPos=((get_time()+offset)%period)/(float)period;
		period=p;
		float curPos=(get_time()%period)/(float)period;
		offset=static_cast<unsigned int>((prevPos-curPos)*period);
	} 
	unsigned int getPeriod() const { return period; } //!< gets the period of time between swings, in milliseconds
	void setMagnitude(float mag) { magnitude=mag; } //!< sets the magnitude of swings, in radians
	double getMagnitude() const { return magnitude; } //!< gets the magnitude of swings, in radians
	void setTilt(float r) { tilt.set(r,1); }  //!< sets the tilt of the tail while wagging, in radians
	void unsetTilt() { tilt.unset(); } //!< makes the tilt control unspecified, will let something else control tilt
	double getTilt() const { return tilt.value; }  //!< gets the tilt of the tail while wagging, in radians
        double getPan() const { return last_pan; } //!< returns the most recent pan value of the tail while wagging, in radians

	bool getActive() { return active; } //!< returns true if this is currently trying to wag the tail

	//! turns the tail wagger on or off
	void setActive(bool a) {
		if ( active != a )
			last_sign = 0;
		active=a;
	}
  
protected:
	unsigned int period; //!< period of time between swings, in milliseconds
	float magnitude; //!< magnitude of swings, in radians
	unsigned int offset; //!< offset in the period, only used if period is changed to avoid twitching
	bool active; //!< true if this is currently trying to wag the tail
	float last_pan; //!< last tail position
	int last_sign; //!< sign of tail movement direction
	OutputCmd tilt; //!< holds current setting for the tilt joint
	OutputCmd pans[NumFrames]; //!< holds commands for planning ahead the wagging
};

/*! @file
 * @brief Defines TailWagMC, which will wag the tail on a ERS-210 robot.
 * @author ejt (Creator)
 */

#endif

