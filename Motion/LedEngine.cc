#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_LEDS

#include "LedEngine.h"
#include "MotionManager.h"
#include "Shared/WorldState.h"
#include "Shared/ERS210Info.h"
#include "Shared/ERS220Info.h"
#include "Shared/ERS7Info.h"

#ifdef TGT_HAS_LED_PANEL
/*! This is "Mimic the number" style */
const LEDBitMask_t LedEngine::defaultMimicNumMasks[11] = {
	BotRLEDMask|BotLLEDMask|TopBrLEDMask, //0
	BotLLEDMask|MidLLEDMask|TopLLEDMask,  //1
	BotRLEDMask|BotLLEDMask|TopLLEDMask|TopBrLEDMask, //2
	BotRLEDMask|BotLLEDMask|MidRLEDMask|TopLLEDMask|TopBrLEDMask, //3
	BotLLEDMask|MidLLEDMask|TopRLEDMask|TopLLEDMask,  //4
	BotRLEDMask|BotLLEDMask|TopRLEDMask|TopBrLEDMask, //5
	BotRLEDMask|BotLLEDMask|MidRLEDMask|MidLLEDMask|TopRLEDMask|TopBrLEDMask, //6
	BotLLEDMask|MidLLEDMask|TopLLEDMask|TopBrLEDMask,  //7
	BotRLEDMask|BotLLEDMask|MidRLEDMask|MidLLEDMask|TopRLEDMask|TopLLEDMask|TopBrLEDMask, //8
	BotLLEDMask|MidLLEDMask|TopRLEDMask|TopLLEDMask|TopBrLEDMask,  //9
	BotLLEDMask //.
};
/*! This is "Count the dots" style specialized for the ERS-220 */
const LEDBitMask_t LedEngine::ERS220numMasks[11] = {
	ERS220Info::ModeLEDMask, //0

	ERS220Info::FaceBackLeftLEDMask, //1

	ERS220Info::FaceBackLeftLEDMask|ERS220Info::FaceCenterLeftLEDMask, //2

	ERS220Info::FaceBackLeftLEDMask|ERS220Info::FaceCenterLeftLEDMask|ERS220Info::FaceFrontLeftLEDMask, //3

	ERS220Info::FaceBackLeftLEDMask|ERS220Info::FaceCenterLeftLEDMask|ERS220Info::FaceFrontLeftLEDMask
	|ERS220Info::FaceFrontRightLEDMask, //4

	ERS220Info::FaceBackLeftLEDMask|ERS220Info::FaceCenterLeftLEDMask|ERS220Info::FaceFrontLeftLEDMask
	|ERS220Info::FaceFrontRightLEDMask|ERS220Info::FaceCenterRightLEDMask, //5

	ERS220Info::FaceBackLeftLEDMask|ERS220Info::FaceCenterLeftLEDMask|ERS220Info::FaceFrontLeftLEDMask
	|ERS220Info::FaceFrontRightLEDMask|ERS220Info::FaceCenterRightLEDMask|ERS220Info::FaceBackRightLEDMask, //6

	ERS220Info::FaceBackLeftLEDMask|ERS220Info::FaceCenterLeftLEDMask|ERS220Info::FaceFrontLeftLEDMask
	|ERS220Info::FaceFrontRightLEDMask|ERS220Info::FaceCenterRightLEDMask|ERS220Info::FaceBackRightLEDMask
	|ERS220Info::FaceFrontALEDMask, //7

	ERS220Info::FaceBackLeftLEDMask|ERS220Info::FaceCenterLeftLEDMask|ERS220Info::FaceFrontLeftLEDMask
	|ERS220Info::FaceFrontRightLEDMask|ERS220Info::FaceCenterRightLEDMask|ERS220Info::FaceBackRightLEDMask
	|ERS220Info::FaceFrontALEDMask|ERS220Info::FaceFrontBLEDMask, //8

	ERS220Info::FaceBackLeftLEDMask|ERS220Info::FaceCenterLeftLEDMask|ERS220Info::FaceFrontLeftLEDMask
	|ERS220Info::FaceFrontRightLEDMask|ERS220Info::FaceCenterRightLEDMask|ERS220Info::FaceBackRightLEDMask
	|ERS220Info::FaceFrontALEDMask|ERS220Info::FaceFrontBLEDMask|ERS220Info::FaceFrontCLEDMask, //9

	ERS220Info::FaceFrontLeftLEDMask //.
};
/*
/ *! This is "Mimic the number" style * /
const LEDBitMask_t LedEngine::ERS7numMasks[11] = {
	(ERS7Info::FaceLEDPanelMask<< 5)|(ERS7Info::FaceLEDPanelMask<< 3), //0
	(ERS7Info::FaceLEDPanelMask<< 7)|(ERS7Info::FaceLEDPanelMask<< 3)|(ERS7Info::FaceLEDPanelMask<< 1), //1
	(ERS7Info::FaceLEDPanelMask<<10)|(ERS7Info::FaceLEDPanelMask<< 9)|(ERS7Info::FaceLEDPanelMask<< 5)|(ERS7Info::FaceLEDPanelMask<<3), //2
	(ERS7Info::FaceLEDPanelMask<< 7)|(ERS7Info::FaceLEDPanelMask<< 3)|(ERS7Info::FaceLEDPanelMask<< 1)|(ERS7Info::FaceLEDPanelMask<<5), //3
	(ERS7Info::FaceLEDPanelMask<<10)|(ERS7Info::FaceLEDPanelMask<<11)|(ERS7Info::FaceLEDPanelMask<< 5)|(ERS7Info::FaceLEDPanelMask<<7)|(ERS7Info::FaceLEDPanelMask<<3)|(ERS7Info::FaceLEDPanelMask<<1), //4
	(ERS7Info::FaceLEDPanelMask<< 7)|(ERS7Info::FaceLEDPanelMask<< 9)|(ERS7Info::FaceLEDPanelMask<<11)|(ERS7Info::FaceLEDPanelMask<<5), //5
	(ERS7Info::FaceLEDPanelMask<< 9)|(ERS7Info::FaceLEDPanelMask<< 5)|(ERS7Info::FaceLEDPanelMask<< 3), //6
	(ERS7Info::FaceLEDPanelMask<< 9)|(ERS7Info::FaceLEDPanelMask<< 7)|(ERS7Info::FaceLEDPanelMask<< 3)|(ERS7Info::FaceLEDPanelMask<<1), //7
	(ERS7Info::FaceLEDPanelMask<<10)|(ERS7Info::FaceLEDPanelMask<<11)|(ERS7Info::FaceLEDPanelMask<< 9)|(ERS7Info::FaceLEDPanelMask<<5)|(ERS7Info::FaceLEDPanelMask<<7)|(ERS7Info::FaceLEDPanelMask<<3), //8
	(ERS7Info::FaceLEDPanelMask<< 9)|(ERS7Info::FaceLEDPanelMask<< 5)|(ERS7Info::FaceLEDPanelMask<< 7)|(ERS7Info::FaceLEDPanelMask<<3)|(ERS7Info::FaceLEDPanelMask<<1), //9
	(ERS7Info::FaceLEDPanelMask<< 1) //.
};
*/
/*! This is "Count the dots" style specialized for the ERS-7 */
const LEDBitMask_t LedEngine::ERS7numMasks[11] = {
	0, //0
	(ERS7Info::FaceLEDPanelMask<<11), //1

	(ERS7Info::FaceLEDPanelMask<< 4)|(ERS7Info::FaceLEDPanelMask<< 5), //2

	(ERS7Info::FaceLEDPanelMask<< 2)|(ERS7Info::FaceLEDPanelMask<<11)|(ERS7Info::FaceLEDPanelMask<< 3), //3

	(ERS7Info::FaceLEDPanelMask<< 2)|(ERS7Info::FaceLEDPanelMask<< 3)|(ERS7Info::FaceLEDPanelMask<< 8)
	|(ERS7Info::FaceLEDPanelMask<<9), //4

	(ERS7Info::FaceLEDPanelMask<< 2)|(ERS7Info::FaceLEDPanelMask<< 3)|(ERS7Info::FaceLEDPanelMask<< 8)
	|(ERS7Info::FaceLEDPanelMask<<9)|(ERS7Info::FaceLEDPanelMask<<11), //5

	(ERS7Info::FaceLEDPanelMask<< 0)|(ERS7Info::FaceLEDPanelMask<< 1)|(ERS7Info::FaceLEDPanelMask<< 4)
	|(ERS7Info::FaceLEDPanelMask<<5)|(ERS7Info::FaceLEDPanelMask<< 6)|(ERS7Info::FaceLEDPanelMask<< 7), //6

	(ERS7Info::FaceLEDPanelMask<< 0)|(ERS7Info::FaceLEDPanelMask<< 1)|(ERS7Info::FaceLEDPanelMask<< 4)
	|(ERS7Info::FaceLEDPanelMask<<5)|(ERS7Info::FaceLEDPanelMask<< 6)|(ERS7Info::FaceLEDPanelMask<< 7)
	|(ERS7Info::FaceLEDPanelMask<<11), //7

	(ERS7Info::FaceLEDPanelMask<< 2)|(ERS7Info::FaceLEDPanelMask<< 3)|(ERS7Info::FaceLEDPanelMask<< 4)
	|(ERS7Info::FaceLEDPanelMask<<5)|(ERS7Info::FaceLEDPanelMask<< 6)|(ERS7Info::FaceLEDPanelMask<< 7)
	|(ERS7Info::FaceLEDPanelMask<< 8)|(ERS7Info::FaceLEDPanelMask<<9), //8

	(ERS7Info::FaceLEDPanelMask<< 2)|(ERS7Info::FaceLEDPanelMask<< 3)|(ERS7Info::FaceLEDPanelMask<< 4)
	|(ERS7Info::FaceLEDPanelMask<<5)|(ERS7Info::FaceLEDPanelMask<< 6)|(ERS7Info::FaceLEDPanelMask<< 7)
	|(ERS7Info::FaceLEDPanelMask<< 8)|(ERS7Info::FaceLEDPanelMask<<9)|(ERS7Info::FaceLEDPanelMask<<11), //9

	(ERS7Info::FaceLEDPanelMask<< 1) //.
};
#endif // has led panel

/*! This is "Count the dots" style */
const LEDBitMask_t LedEngine::defaultCountNumMasks[11] = {
	(1<<0)|(1<<8), //0
	(1<<0), //1
	(1<<0)|(1<<1), //2
	(1<<0)|(1<<1)|(1<<2), //3
	(1<<0)|(1<<1)|(1<<2)|(1<<3), //4
	(1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4), //5
	(1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<5), //6
	(1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6), //7
	(1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7), //8
	(1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7)|(1<<8), //9
	(1<<1)|(1<<3)|(1<<5)|(1<<7) //.
};


LedEngine::LedEngine() : dirty(true), numCycling(0), nextFlashEnd((unsigned int)-1) {
	for(unsigned int i=0; i<NumLEDs; i++) {
		infos[i].flashtime=0;
		infos[i].starttime=0;
	}
	clear();
}

void LedEngine::recalcFlashEnd(void) {
  unsigned int t = get_time();
  nextFlashEnd=(unsigned int)-1;
  for(unsigned int i=0; i<NumLEDs; i++)
    if(infos[i].flashtime>t && nextFlashEnd>infos[i].flashtime)
      nextFlashEnd=infos[i].flashtime;
}

int LedEngine::isDirty() {
	unsigned int t = get_time();
	if(t>nextFlashEnd) {
	  dirty=true;
	  recalcFlashEnd();
	};
	return dirty;
}

int LedEngine::updateLEDs(const MotionCommand* caller, LEDBitMask_t mask/*=AllLEDMask*/) {
	unsigned int t = get_time();
	if (t>nextFlashEnd) recalcFlashEnd();
#ifdef TGT_HAS_LEDS
	for(unsigned int i=0; i<NumLEDs; i++)
		if((mask>>i)&1)
			for(unsigned int f=0; f<NumFrames; f++)
				motman->setOutput(caller, i+LEDOffset,calcValue(i,t+f*FrameTime),f);
#endif
	bool tmp=dirty;
	dirty = numCycling>0;
	return tmp;
}

int LedEngine::updateLEDs(OutputCmd cmds[NumLEDs]) {
	unsigned int t = get_time();
	if (t>nextFlashEnd) recalcFlashEnd();
	for(unsigned int i=0; i<NumLEDs; i++)
			cmds[i].value=calcValue(i,t);
	bool tmp=dirty;
	dirty = numCycling>0;
	return tmp;
}

int LedEngine::updateLEDFrames(OutputCmd cmds[NumLEDs][NumFrames]) {
	unsigned int t = get_time();
	if (t>nextFlashEnd) recalcFlashEnd();
	for(unsigned int i=0; i<NumLEDs; i++)
		for(unsigned int f=0; f<NumFrames; f++)
			cmds[i][f].value=calcValue(i,t+f*FrameTime);
	bool tmp=dirty;
	dirty = numCycling>0;
	return tmp;
}

void LedEngine::invert(LEDBitMask_t leds) {
	if(leds!=0) {
		dirty=true;
		for(unsigned int i=0; i<NumLEDs; i++) {
			if((leds>>i)&1) {
				if(infos[i].isCycling)
					infos[i].amp*=-1;
				else
					infos[i].value=1-infos[i].value;
			}
		}
	}
}
void LedEngine::set(LEDBitMask_t leds, float value) {
	if(leds!=0) {
		dirty=true;
		for(unsigned int i=0; i<NumLEDs; i++)
			if((leds>>i)&1) {
				infos[i].value=value;
				if(infos[i].isCycling) {
					numCycling--;
					infos[i].isCycling=false;
				}
			}
	}
}
void LedEngine::cflash(LEDBitMask_t leds, float value, unsigned int ms) {
	dirty=true;
	unsigned int t = get_time();
	if(t+ms<nextFlashEnd)
	  nextFlashEnd=t+ms;
	for(unsigned int i=0; i<NumLEDs; i++) {
		infos[i].flashvalue=((leds>>i)&1)*value;
		infos[i].flashtime=t+ms;
	}
}
void LedEngine::flash(LEDBitMask_t leds, float value, unsigned int ms) {
	if(leds!=0) {
		dirty=true;
		unsigned int t = get_time();
		if(t+ms<nextFlashEnd)
		  nextFlashEnd=t+ms;
		for(unsigned int i=0; i<NumLEDs; i++)
			if((leds>>i)&1) {
				infos[i].flashvalue=value;
				infos[i].flashtime=t+ms;
			}
	}
}
void LedEngine::flash(LEDBitMask_t leds, unsigned int ms) {
	if(leds!=0) {
		dirty=true;
		unsigned int t = get_time();
		if(t+ms<nextFlashEnd)
		  nextFlashEnd=t+ms;
		for(unsigned int i=0; i<NumLEDs; i++)
			if((leds>>i)&1) {
				infos[i].flashvalue=calcFlash(calcValue(i,t));
				infos[i].flashtime=t+ms;
			}
	}
}
/*!@param leds the bitmask of leds to apply this to
 * @param period the period of the cycle (milliseconds), includes an on and off
 * @param amp the amplitude of the cycle - note that this is clipped at 0 and 1.
 * @param offset the vertical offset of the cycle - simply shifts the baseline of the cycle up or down
 * @param phase the phase within the cycle to start at (specify in milliseconds)
 *
 * When this function is called, the starting time is stored as current time + phase.
 *
 * The equation used is \f[\cos(\frac{2\pi(t-starttime)}{period})*(\frac{-amp}{2})+.5+offset\f]
 * 
 * The idea is that with a amplitude=1 and offset=0, it will start at
 * 0, ramp up to 1, and then ramp down again.  The arguments to this
 * function will let you control all parameters of the cycle.
 *
 * You can get a blink-on/off instead of cycle on/off by using a very large amplitude.
 */
void LedEngine::cycle(LEDBitMask_t leds, unsigned int period, float amp, float offset, int phase) {
	//	cout << "cycle("<<leds<<","<<period<<","<<amp<<","<<offset<<","<<phase<<")"<<endl;
	if(leds!=0) {
		dirty=true;
		unsigned int start = get_time()+phase;
		for(unsigned int i=0; i<NumLEDs; i++)
			if((leds>>i)&1) {
				if(!infos[i].isCycling)
					numCycling++;
				infos[i].isCycling=true;
				infos[i].amp=amp;
				infos[i].period=period;
				infos[i].starttime=start;
				infos[i].offset=offset;
			}
	}
}
void LedEngine::clear() {
	for(unsigned int i=0; i<NumLEDs; i++) {
		infos[i].value=0;
		infos[i].flashtime=0;
		infos[i].isCycling=false;
	}
	numCycling=0;
	dirty=true;
}

void LedEngine::extendFlash(unsigned int ms) {
	for(unsigned int i=0; i<NumLEDs; i++)
		if(infos[i].flashtime!=0)
			infos[i].flashtime+=ms;
	if(nextFlashEnd!=0)
		nextFlashEnd+=ms;
	dirty=true;
}

void LedEngine::displayNumber(int x, numStyle_t style) {
	switch(style) {
	case onedigit: {
#ifdef TGT_HAS_LED_PANEL
		const LEDBitMask_t * numMasks=defaultCountNumMasks;
		if(RobotName == ERS210Info::TargetName)
			numMasks=defaultMimicNumMasks;
		else if(RobotName == ERS220Info::TargetName)
			numMasks=ERS220numMasks;
		else if(RobotName == ERS7Info::TargetName)
			numMasks=ERS7numMasks;
		if(x>9 || x<-9) {
			ccycle(FaceLEDMask&~TopBrLEDMask,333,10,-5);
			infos[TopBrLEDOffset-LEDOffset].value=x<0?1:0;
		} else {
			clear();
			if(x<0) {
				set(numMasks[-x],1);
				infos[TopBrLEDOffset-LEDOffset].value=infos[TopBrLEDOffset-LEDOffset].value*.5f+.25f;
			} else
				set(numMasks[x],1);
		}
#elif defined(TGT_HAS_LEDS)
		if(NumLEDs<9)
			return;
		const LEDBitMask_t * numMasks=defaultCountNumMasks;
		if(x>9 || x<-9) {
			ccycle((LEDBitMask_t)~(1<<(NumLEDs-1)),333,10,-5);
			infos[NumLEDs-1].value=x<0?1:0;
		} else {
			clear();
			if(x<0) {
				set(numMasks[-x],1);
				infos[NumLEDs-1].value=infos[NumLEDs-1].value*.5f+.25f;
			} else
				set(numMasks[x],1);
		}
#endif
		} break;
	case twodigit:
#ifdef TGT_HAS_LED_PANEL
		if(x>99 || x<-99) {
			ccycle(FaceLEDMask&~TopBrLEDMask,333,10,-5);
			infos[TopBrLEDOffset-LEDOffset].value=x<0?1:0;
		} else {
			clear();
			if(x<0) {
				infos[TopBrLEDOffset-LEDOffset].value=1;
				x=-x;
			}
			setOneOfTwo(x/10,BotRLEDOffset-LEDOffset,MidRLEDOffset-LEDOffset,TopRLEDOffset-LEDOffset);
			setOneOfTwo(x%10,BotLLEDOffset-LEDOffset,MidLLEDOffset-LEDOffset,TopLLEDOffset-LEDOffset);
		}
#elif defined(TGT_HAS_LEDS)
		if(NumLEDs<3)
			return;
		if(x>99 || x<-99) {
			ccycle((LEDBitMask_t)~(1<<(NumLEDs-1)),333,10,-5);
			infos[NumLEDs-1].value=x<0?1:0;
		} else {
			clear();
			if(x<0) {
				infos[NumLEDs-1].value=1;
				x=-x;
			}
			setOneOfTwo(x/10,NumLEDs-3,NumLEDs-2,NumLEDs-1);
			setOneOfTwo(x%10,2,1,0);
		}
#endif
		break;
	}
}
void LedEngine::setOneOfTwo(unsigned int x, unsigned int low, unsigned int mid, unsigned int high) {
	if(x==0)
		return;
	float bg = ((x-1)/3)/3.f;
	float fg = bg+1/3.f;
	if(RobotName == ERS7Info::TargetName)
		bg*=bg; // dim the background a bit more on ERS7
	switch(x%3) {
	case 1:
		infos[high].value=bg;
		infos[mid].value=bg;
		infos[low].value=fg;
		break;
	case 2:
		infos[high].value=bg;
		infos[mid].value=fg;
		infos[low].value=bg;
		break;
	case 0:
		infos[high].value=fg;
		infos[mid].value=bg;
		infos[low].value=bg;
		break;
	}
}

void LedEngine::displayPercent(float x, percentStyle_t left_style, percentStyle_t right_style) {
	clear();
#ifdef TGT_HAS_LED_PANEL
	if(x<0) {
		set(FaceLEDMask,.25f);
		return;
	}
	if(x>1) {
		set(FaceLEDMask,.75f);
		return;
	}
	if(left_style==major)
		setColumn(x,BotLLEDMask,MidLLEDMask,TopLLEDMask,TopBrLEDMask);
	if(right_style==major)
		setColumn(x,BotRLEDMask,MidRLEDMask,TopRLEDMask,TopBrLEDMask);
	x*=4;
	x-=(int)x;
	if(left_style==minor)
		setColumn(x,BotLLEDMask,MidLLEDMask,TopLLEDMask,TopBrLEDMask);
	if(right_style==minor)
		setColumn(x,BotRLEDMask,MidRLEDMask,TopRLEDMask,TopBrLEDMask);
#else
	if(x<0) {
		set(AllLEDMask,.25f);
		return;
	}
	if(x>1) {
		set(AllLEDMask,.75f);
		return;
	}
	if(left_style==major)
		setColumn(x,3,2,1,0);
	if(right_style==major)
		setColumn(x,NumLEDs-4,NumLEDs-3,NumLEDs-2,NumLEDs-1);
	x*=4;
	x-=(int)x;
	if(left_style==minor)
		setColumn(x,3,2,1,0);
	if(right_style==minor)
		setColumn(x,NumLEDs-4,NumLEDs-3,NumLEDs-2,NumLEDs-1);
#endif
}

void LedEngine::setColumn(float x, unsigned int low, unsigned int mid, unsigned int high, unsigned int top) {
	LEDBitMask_t solid=0;
	LEDBitMask_t partial=0;
	switch((int)(4*x)) {
	case 4:
		solid|=top;
	case 3:
		solid|=high;
	case 2:
		solid|=mid;
	case 1:
		solid|=low;
	}
	switch((int)(4*x)) {
	case 3:
		partial=top; break;
	case 2:
		partial=high; break;
	case 1:
		partial=mid; break;
	case 0:
		partial=low; break;
	}
	float partialvalue=(x*4)-(int)(x*4);
	set(partial,partialvalue);
	set(solid,1);
}

#endif

/*! @file
 * @brief Implements LedEngine, which provides basic LED effects to anything that inherits or instantiates it
 * @author ejt (Creator)
 */
