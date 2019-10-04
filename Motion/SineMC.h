//-*-c++-*-
#ifndef INCLUDED_SineMC_h_
#define INCLUDED_SineMC_h_

#include "Motion/MotionCommand.h"
#include "Motion/MotionManager.h"
#include <cmath>

//! Moves one or more outputs through a sinusoidal pattern
class SineMC : public MotionCommand {
public:
	//! constructor, sets default parameters to use full range of each joint, with a period of 2 seconds, but all weights to zero
	SineMC() : MotionCommand(), lastUpdate(get_time()+FrameTime*NumFrames) {
		for(unsigned int i=0; i<NumOutputs; ++i) {
			amp[i] = (outputRanges[i][MaxRange] - outputRanges[i][MinRange]) / 2;
			period[i]=2000;
			offset[i] = (outputRanges[i][MaxRange] + outputRanges[i][MinRange]) / 2;
			sync(i,lastUpdate);
		}
	}

	//! sets the weight of an output, zero-weighted outputs are ignored
	virtual void setWeight(unsigned int i, float w) { cmds[i].weight=w; }
	//! gets the weight of an output, zero-weighted outputs are ignored
	virtual float getWeight(unsigned int i) const { return cmds[i].weight; }
	
	//! sets the sine wave parameters
	/*! @param i the index of the output
	 *  @param amplitude the peak-to-center amplitude
	 *  @param period_time the period of the oscillation (milliseconds)
	 *  @param offset_value sets the center position of the oscillation
	 *  @param offset_time sets the phase shift of the oscillation
	 *
	 *  If the output weight is 0, it is set to 1. */
	virtual void setParams(unsigned int i, float amplitude, unsigned int period_time, float offset_value, unsigned int offset_time) {
		amp[i]=amplitude;
		period[i]=period_time;
		offset[i]=offset_value;
		start[i]=offset_time;
		if(cmds[i].weight<=0)
			cmds[i].weight=1;
	}
	//! sets the sine wave parameters
	/*! @param i the index of the output
	 *  @param amplitude the peak-to-center amplitude
	 *  @param period_time the period of the oscillation (milliseconds)
	 *  @param offset_value sets the center position of the oscillation
	 *  
	 *  The offset time is synced to maintain current position.  If the output weight is 0, it is set to 1. */
	virtual void setParams(unsigned int i, float amplitude, unsigned int period_time, float offset_value) {
		amp[i]=amplitude;
		period[i]=period_time;
		offset[i]=offset_value;
		if(cmds[i].weight<=0)
			cmds[i].weight=1;
		sync(i);
	}
	//! gets the peak-to-center amplitude
	virtual float getAmplitude(unsigned int i) const { return amp[i]; }
	//! gets the period of the oscillation (milliseconds)
	virtual unsigned int getPeriod(unsigned int i) const { return period[i]; }
	//! gets the center position of the oscillation
	virtual float getOffsetValue(unsigned int i) const { return offset[i]; }
	//! gets the phase shift of the oscillation
	virtual unsigned int getOffsetTime(unsigned int i) const { return start[i]; }
	
	//! returns the percent of period completed
	virtual float getPhase(unsigned int i) const { float x = getCount(i); return x-std::floor(x); }
	//! returns the percent of period completed
	virtual float getCount(unsigned int i) const { unsigned int t=get_time(); return float(t-start[i])/period[i]; }
	
	//! returns the current target position of the specified output
	virtual float getPosition(unsigned int i) const { return cmds[i].value; }
	//! returns the current target speed of the specified output (radians/sec)
	virtual float getSpeed(unsigned int i) const {
		if(cmds[i].weight<=0)
			return 0;
		float scale = 2*static_cast<float>(M_PI)/period[i];
		return std::cos((lastUpdate-start[i])*scale)*amp[i]*scale*1000;
	}
	//! returns the current target acceleration of the specified output (radians/sec^2)
	virtual float getAcceleration(unsigned int i) const {
		if(cmds[i].weight<=0)
			return 0;
		float scale = 2*static_cast<float>(M_PI)/period[i];
		return -std::sin((lastUpdate-start[i])*scale)*amp[i]*scale*scale*1000*1000;
	}
	
	//! sets the offset_time of a specified output to match its current position so it won't suddenly snap to a new position
	virtual void sync(unsigned int i) { sync(i,get_time()); }
	
	//! sets the offset_time of a specified output to match its current position so it won't suddenly snap to a new position
	virtual void sync(unsigned int i, unsigned int t) {
		const float TWOPI = 2*static_cast<float>(M_PI);
		float x = (state->outputs[i]-offset[i])/amp[i];
		if(x>1)
			x=1;
		else if(x<-1)
			x=-1;
		start[i] = (unsigned int)rintf(t - std::asin(x)/TWOPI*period[i]);
	}
	
	//! sets the offset_time of all non-zero weighted joints to use the closest common phase, keeping all outputs in phase
	virtual void syncAll() {
		unsigned int t=get_time();
		float avg=0, totw=0;
		for(unsigned int i=0; i<NumOutputs; ++i) {
			sync(i,t);
			avg+=cmds[i].weight * start[i];
			totw+=cmds[i].weight;
		}
		avg/=totw;
		for(unsigned int i=0; i<NumOutputs; ++i)
			start[i]=(unsigned int)rintf(avg);
	}
	
	virtual int updateOutputs() {
		const float TWOPI = 2*static_cast<float>(M_PI);
		unsigned int t=get_time();
		unsigned int cnt=0;
		// this is probably overkill to split these cases, but anyway...
		if(NumFrames==1) {
			// single frame, make direct calls to avoid extra copy
			for(unsigned int i=0; i<NumOutputs; ++i) {
				if(cmds[i].weight>0) {
					cmds[i].value=std::sin((t-start[i])*TWOPI/period[i])*amp[i]+offset[i];
					motman->setOutput(this,i,cmds[i]);
					++cnt;
				}
			}
		} else {
			// multiple frames, do them in bulk for efficiency
			OutputCmd tmp[NumFrames];
			for(unsigned int i=0; i<NumOutputs; ++i) {
				if(cmds[i].weight>0) {
					for(unsigned int f=0; f<NumFrames; ++f) {
						tmp[i].value=std::sin((t-start[i]+f*FrameTime)*TWOPI/period[i])*amp[i]+offset[i];
						tmp[i].weight=cmds[i].weight;
					}
					motman->setOutput(this,i,tmp);
					cmds[i].value = tmp[i].value;
					++cnt;
				}
			}
		}
		lastUpdate=t+(NumFrames-1)*FrameTime;
		return cnt;
	}
		
	virtual int isDirty() { return true; }
	virtual int isAlive() { return true; }

protected:
	OutputCmd cmds[NumOutputs];
	float amp[NumOutputs];
	unsigned int period[NumOutputs];
	float offset[NumOutputs];
	unsigned int start[NumOutputs];
	unsigned int lastUpdate;
};

/*! @file
 * @brief Defines SineMC, which DESCRIPTION
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
