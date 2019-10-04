//-*-c++-*-
#ifndef INCLUDED_FlashIPAddrBehavior_h_
#define INCLUDED_FlashIPAddrBehavior_h_

#ifndef TGT_HAS_LEDS
#  error FlashIPAddrBehavior requires a target with LEDs
#endif

#include "Behaviors/BehaviorBase.h"
#include "Motion/MotionManager.h"
#include "Motion/MotionSequenceMC.h"
#include "IPC/SharedObject.h"

//! Displays IP address by speaking the digits and flashing a series of numbers on the LED face panel
/*! Will only run the display on doStart() if the flash_on_start
 *  config variable is set.  Otherwise you will need to hold down the
 *  buttons specified by #button1 and #button2 to trigger the display.
 *  Note that if the e-stop is active it will intercept the button
 *  events, so turn off e-stop first. */
class FlashIPAddrBehavior : public BehaviorBase {
public:
	//! constructor
	FlashIPAddrBehavior()
		: BehaviorBase("FlashIPAddrBehavior"), sounds(), ms(), ms_id(MotionManager::invalid_MC_ID)
	{}

	virtual void doStart(); //!< if the Config::behavior_config::flash_on_start flag is set, will setup and run
	virtual void doStop();  //!< halts any display which may be in progress

	//! Receives button events, timers, and motman manager pruning notifications
	virtual void doEvent();

	static std::string getClassDescription() {
		std::string pre="Displays IP address by flashing a series of numbers on the LED face panel;";
#ifdef TGT_HAS_BUTTONS
		pre+=" Hold down ";
		pre+=buttonNames[button1];
		pre+=" and ";
		pre+=buttonNames[button2];
		pre+=" to trigger any time while running";
#else
		pre+=" Set configuration file to enable display on boot.";
#endif
		return pre;
	}
	virtual std::string getDescription() const { return getClassDescription(); }
	
protected:
	typedef XLargeMotionSequenceMC MSMC_t; //!< used to flash the LEDs to report the IP address

	void loadSounds();    //!< loads the numeric sounds into memory
	void releaseSounds(); //!< releases the numeric sounds
	void setupSequence(); //!< construct the motion sequence for flashing leds, request timers to play corresponding sound file

#ifdef TGT_IS_AIBO
	static const unsigned int button1=ChinButOffset; //!< one of two buttons which must be pressed together to trigger the report without using the Controller
	static const unsigned int button2=HeadFrButOffset; //!< one of two buttons which must be pressed together to trigger the report without using the Controller
#elif defined(TGT_HAS_BUTTONS)
	static const unsigned int button1=0; //!< one of two buttons which must be pressed together to trigger the report without using the Controller
	static const unsigned int button2=1; //!< one of two buttons which must be pressed together to trigger the report without using the Controller
#endif
	
	static const unsigned int ACTIVATE_TIMER=-1U; //!< timer id to specify both trigger buttons have been down long enough
	std::vector<std::string> sounds; //!< sound to play, corresponding to timers to coincide with corresponding digit on the LEDs (could be done with chained sounds, but this is cooler)
	static const unsigned int delay=64; //!< time (in milliseconds) to expect #ms to be delayed before it actually starts
	
	SharedObject<MSMC_t> ms; //!< motion sequence used to control the LEDs
	MotionManager::MC_ID ms_id; //!< id number of #ms
};

/*! @file
 * @brief Describes FlashIPAddrBehavior, which displays IP address by flashing a series of numbers on the LED face panel
 * @author ejt (Creator)
 */

#endif
