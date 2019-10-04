#ifndef INCLUDED_LedEngine_h
#define INCLUDED_LedEngine_h

#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_LEDS

#include "Shared/get_time.h"
#include "OutputCmd.h"
#include <math.h>

class MotionCommand;

// LEDBitMask_t and associated constants are defined in RobotInfo.h

//! Provides basic LED effects via subclassing (e.g. LedMC) or embedding (e.g. EmergencyStopMC)
/*! This implements a collection of LED special effects so that the code can be
 *  reused various places as feedback to to the user.
 *
 * The basic effects provided are:
 * - static brightness (set(), cset(), invert(), clear())
 * - temporary setting (flash(), cflash())
 * - pulsing/blinking (cycle(), ccycle())
 * - numeric display (displayNumber(), displayPercent())
 *
 * The 'c' prefix on a function (cset(), cflash(), ccycle()) means it will call clear() before doing its own setting.  This is
 * a quick way to replace whatever is currently displayed with whatever you are about to display.
 * 
 * A flash() will invert and override the current setting, so that it
 * will "reset" after the flash.  Flashes change mid-range values to
 * extremes to make the flash visible (ie not just (1-current)) Normal
 * invert() will do simple inverses (just (1-current)), and doesn't change back automatically.
 *
 * Cycling ("pulsing") and single-value setting are mutually exclusive;
 * one will cut off the other
 *
 * getSetting() returns value of last set();
 * getValue() returns what's actually being returned to Motion at the moment
 *
 * There's some nice functions for using the LEDs to display numbers.
 * This is handy for when you want to be free of the terminal.
 * <img src="NumberLEDs.jpg">
 *
 * The ERS-220 and ERS-7 have enough LEDs that they just use a "count
 * the lights" style of display instead of this pseudo-arabic display.
 * (look close to see that green bar LED at the top of the 210, which 
 * doesn't show up well in the camera image for some reason. )
 * <img src="NumberLEDs-ERS7.jpg">
 *
 * The ERS-7 also has a "mode" which controls how certain face LEDs are
 * interpreted.  By setting ERS7Info::LEDABModeOffset /
 * ERS7Info::LEDABModeMask, you can switch between the modes -- a value
 * of 0 is mode "A", and a value of 1 is mode "B".
 * <img src="ERS7-LED-Modes.png">
 *
 * Things to note for the ERS-7:
 * - There are many fewer LEDs than there are holes in the "grill" (the holes you see on the face are not the positions of the LEDs.
 * - Each LED covers several holes.
 * - Not all holes have LEDs behind them -- some areas of the face panel are always dark.
 * - Some LEDs (12 and 13) control two symmetrical physical LEDs.
 * - Some LEDs change color based on mode (0-1), some don't change at all (2-11), and some change position and color (12-13)
 * - We don't have individual names for all of the LEDs on the panel.  Instead you base off of FaceLEDPanelOffset or FaceLEDPanelMask to address the LEDs, e.g., to set the <i>n</i>th led to <i>x</i>:
 *   @code
 *   LedEngine::set(FaceLEDPanelMask << n, x)
 *   @endcode
 *   or (if using other classes, e.g. PostureEngine)
 *   @code
 *   PostureEngine::setOutputCmd(FaceLEDPanelOffset + n, x)
 *   @endcode
 */

class LedEngine {
 public:
	//!constructor - don't forget to call if you inherit
	LedEngine();
	//!destructor
	virtual ~LedEngine() {}
	
	//! call this from a MotionCommand's updateOutputs() - makes calls to MotionManager to update LED values
	/*! @param caller pass the "parent" motioncommand's address here (usually will pass 'this')
	 *  @param mask a bitmask of which leds to update (uses weight of 1) */
	int updateLEDs(const MotionCommand* caller,LEDBitMask_t mask=AllLEDMask);
	
	//! call this from a MotionCommand's updateOutputs() - performs the calculations to update LEDs' values
	/*! @param cmds on input, used for weight values - on return, holds the resulting OutputCmd's*/
	int updateLEDs(OutputCmd cmds[NumLEDs]);

	//! call this from a MotionCommand's updateOutputs() - performs the calculations to update LEDs' values
	/*! @param cmds on input, used for weight values - on return, holds the resulting OutputCmd's*/
	int updateLEDFrames(OutputCmd cmds[NumLEDs][NumFrames]);
	
	//! recalculates #nextFlashEnd so we can tell when a flash has completed
	void recalcFlashEnd();

	//! returns true if there are changes since the last updateLEDs()
	int isDirty();

	//!sets the leds specified by @a leds to the inverse of their current value
	void invert(LEDBitMask_t leds);
	//!sets the leds specified by @a leds to @a value, clears all the rest
	inline void cset(LEDBitMask_t leds, float value) { clear(); set(leds,value); }
	//!sets the leds specified by @a leds to @a value
	void set(LEDBitMask_t leds, float value);
	//!sets the leds specified by @a leds to @a value for @a ms milliseconds, then sets back.  Clears ~leds
	void cflash(LEDBitMask_t leds, float value, unsigned int ms);
	//!sets the leds specified by @a leds to @a value for @a ms milliseconds, then sets back.
	void flash(LEDBitMask_t leds, float value, unsigned int ms);
	//!sets the leds specified by @a leds to either a much higher or much lower value for @a ms milliseconds, then sets back.
	void flash(LEDBitMask_t leds, unsigned int ms);
	//!causes the leds specified by @a leds to cycle between low and high, clears others.  See cycle() for parameter documentation.
	inline void ccycle(LEDBitMask_t leds, unsigned int period, float amp, float offset=0, int phase=0) { clear(); cycle(leds,period,amp,offset,phase); }
	//!causes the leds specified by @a leds to cycle between low and high; values calculated for cycle will be clipped to [0,1] for more sensible blending of square wave approximations (high amplitude sine wave)
	void cycle(LEDBitMask_t leds, unsigned int period, float amp, float offset=0, int phase=0);
	//!sets all leds to 0.
	void clear();
	
	//!adds @a ms to all flash times (may resurrect previously completed flashes)
	void extendFlash(unsigned int ms);

	//!returns the current setting of the LED specified by @a led_id (the value you passed in set())
	float getSetting(LEDOffset_t led_id) {
#ifndef TGT_HAS_LEDS
		(void)led_id; // avoid unused argument warning
		return 0;
#else
		return infos[led_id-LEDOffset].value;
#endif
	}
	//!returns the current value of the LED specified by @a led_id (the value being expressed - may change if cycling for instance)
	float getValue(LEDOffset_t led_id,unsigned int planahead=0) {
#ifndef TGT_HAS_LEDS
		(void)led_id; (void)planahead; // avoid unused argument warnings
		return 0;
#else
		return calcValue(led_id-LEDOffset,get_time()+planahead);
#endif
	}
	
#ifdef TGT_HAS_LED_PANEL
	//!holds a series of bit masks for the onedigit style of numerical display (-9:9 and '.')
	/*!the hope is that these actually resemble the shapes of the numbers so people can
	 * recognize them more easily - without counting or converting base 2 in their heads. */
	static const LEDBitMask_t defaultMimicNumMasks[11];
	static const LEDBitMask_t ERS220numMasks[11]; //!< bit masks for the ondigit style of numberical display - just count the LEDs on the head
	static const LEDBitMask_t ERS7numMasks[11]; //!< bit masks for the ondigit style of numberical display - just count the LEDs on the head
#endif
	static const LEDBitMask_t defaultCountNumMasks[11]; //!< bit masks for the ondigit style of numberical display - just count the LEDs on the head
	//!Use these to specify a style for displaying numbers using displayNumber()
	enum numStyle_t {
		onedigit, //!< can display a number -9 thru 9, using the current robot model's numMask.  For negative numbers, blinks the top bar - fast if it's supposed to be on, slow if it would otherwise be off
		twodigit  //!< can display a number -99 thru 99, using setOneOfTwo().  For negative numbers, sets the top bar to 1 (off otherwise).
	};
	//!Use these to specify a style for displaying a percentage value [0-1] using displayPercent()
	enum percentStyle_t {
		major, //!< shows overall value
		minor, //!< shows value within major tick
		none   //!< if you want to leave blank
	};
		
	//!Allows convenient display of numerical information to the LEDs on the face.
	/*!If overflow occurs, the face LEDs are set to flash on and off 3 every 333 milliseconds*/
	void displayNumber(int x, numStyle_t style);
	//!Allows convenient display of percentage information to the LEDs on the face.
	/*!Besides allowing a two-digit display, the 'edge' bar for each type is blinked to
	 * denote how full it is.  So you can get up to a two-digit, base 5 display, with an
	 * extra digit of estimated value.
	 *
	 * If overflow (>1) occurs, sets everything to .75. <br>
	 * If underflow (<0) occurs, sets everything to .25.
	 * 
	 * The left and right columns are combined with an OR operation.  (they overlap on the top bar)
	 * Left and right designations are <em>dog centric!</em> */
	void displayPercent(float x, percentStyle_t left_style, percentStyle_t right_style);
	
 protected:
	//!Performs the 'invert' calculation based on current value (returns 1-value)
	static float calcInvert(float value) {
		return 1-value;
	}
	//!Performs the 'flash' calculation based on current value (uses calcInvert() if value upper or lower third, 0 or 1 otherwise)
	static float calcFlash(float value) {
		if(value>.33333 && value<.66666)
			return (value<.5 ? 1 : 0);
		else
			return calcInvert(value);
	}
	//!Performs the 'cycle' calculation based on desired period, amplituted, amplitude offset, and time since start.  See cycle()
	static float calcCycle(unsigned int period, float amp, float offset, unsigned int t) {
		//		cout << period << ',' << amp << ',' << offset << ',' << time << " -> " << x;
		float x=float(cos(t*2*M_PI/period))*(-amp/2)+.5f+offset;
		if(x<0)
			return 0;
		if(x>1)
			return 1;
		return x;
	}
	//!Calculates the current value of led @a i for current time t
	float calcValue(unsigned int i, unsigned int t) {
		if(t<infos[i].flashtime)
			return infos[i].flashvalue;
		else if(infos[i].isCycling)
			return calcCycle(infos[i].period,infos[i].amp,infos[i].offset,t-infos[i].starttime);
		else
			return infos[i].value;
	}
	//!used by displayNumber() to determine settings of LEDs when using numStyle_t::twodigit
	void setOneOfTwo(unsigned int x, unsigned int low, unsigned int mid, unsigned int high);
	//!used by displayPercent() to determine settings of LEDs
	void setColumn(float x, unsigned int low, unsigned int mid, unsigned int high, unsigned int top);

	//!Holds all the information needed by each of the LEDs
	struct LEDInfo {
		float value;           //!< the current value being expressed
		float amp;             //!< the amplitude of the cycle (if cycling)
		unsigned int period;    //!< the period of the cycle (if cycling)
		unsigned int starttime; //!< the start time of the cycle (if cycling)
		float offset;          //!< the phase shift from normal of the cycle (if cycling)
		float flashvalue;      //!< the value being 'flashed' (only valid if current time is less than flashtime
		unsigned int flashtime; //!< the time the 'flash' should retire
		bool isCycling;         //!< true if in cycle mode
	};
	
	LEDInfo infos[NumLEDs]; //!< the information regarding each of the LEDs
	bool dirty;             //!< true if changes since last updateLEDs: either at least one LED is cycling or a flash has begun/ended
	unsigned int numCycling;//!< the number of LEDs currently cycling (if non-zero, always dirty)
	unsigned int nextFlashEnd; //!< the soonest time a flash will end (and we'll become dirty)
};
	
/*! @file
 * @brief Describes LedEngine, which provides basic LED effects to anything that inherits or instantiates it
 * @author ejt (Creator)
 */

#endif // TGT_HAS_LEDS

#endif
