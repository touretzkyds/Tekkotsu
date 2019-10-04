#ifndef INCLUDED_EmergencyStopMC_h
#define INCLUDED_EmergencyStopMC_h

#include "PostureMC.h"
#ifdef TGT_HAS_LEDS
#  include "LedEngine.h"
#endif

//!overrides all joints with high priority freeze, blinks tail pink/red/blue cycle
/*! Uses MotionCommand::kEmergencyPriority.  Monitors the feedback on joints and
 *  adjusts joints to react to pressures above a certain threshold.  This allows
 *  you to mold the body while it's in this mode, while retaining enough stiffness
 *  to hold against gravity.
 *
 *  This MotionCommand is intended to always be left running.  It polls WorldState::state
 *  for a double-tap on the back button, which causes it to set its joint values.
 *  to whatever their current state is.  LEDs are left blank, except the tail,
 *  which is used to indicate that the emergency stop is on.
 *
 *  A second double-tap will cause it to set all joints to 0 weight
 *
 *  The tail LEDs only ever go up to .5, so that if you really care whether the tail
 *  light was set by an underlying behavior/motion, you should be able to tell by
 *  looking closely (if blue is going from .5 to 1, that's because it's already set)
 */
class EmergencyStopMC : public PostureMC {
 public:
	EmergencyStopMC(); //!< constructor
	virtual ~EmergencyStopMC() {} //!< destructor
	virtual int updateOutputs(); //!< checks for feedback or double tap

	void setActive(bool a); //!< allows you to modify #active
	bool getActive() { return active; } //!< returns #active
	void setStopped(bool p, bool sound=true); //!< allows you to modify #paused
	bool getStopped() const { return paused; } //!< returns #paused
	void setDblTapDuration(unsigned int d) { duration=d; } //!< sets #duration
	unsigned int getDblTapDuration() const { return duration; } //!< returns #duration
	void setResetSensitivity(float r) { pidcutoff=r; } //!< takes a value to set #pidcutoff
	float getResetSensitivity() { return pidcutoff; } //!< returns #pidcutoff

 protected:
	void freezeJoints(); //!< code to execute when locking joints
	void releaseJoints(); //!< code to execute when releasing joints
	static bool trigger(); //!< true when the trigger condition is active
	
	static const unsigned int FADE_OUT_TIME=400; //!< number of milliseconds to fade out lock on joints

	bool paused; //!< true if the joints are current locked up
	bool stilldown; //!< true if the back button was down on last updateJointCmds
	bool active; //!< true if the EmergencyStopMC is monitoring the back button (if false, won't pause on a double-tap)
	unsigned int period; //!< period of cycles on tail LEDs
	unsigned int timeoflastbtn; //!< time of the last button press
	unsigned int timeofthisbtn; //!< time of the current button press
	unsigned int timeoflastfreeze; //!< the time estop was last turned on
	unsigned int timeoflastrelease; //!< the time estop was last turned off (may be in the future if still fading out control of joints!)
	unsigned int duration; //!<the maximum time (in milliseconds) of consecutive button-down's to count as a double tap
	float piddutyavgs[NumPIDJoints]; //!< a running average of PID feedback ("duty"), so one bad reading doesn't cause a movement, need a consistent pressure
	float pidcutoff; //!<abs pid duty cycle above which we just reset joint to current
#ifdef TGT_HAS_LEDS
	LedEngine ledengine; //!< used to do LED effects on the tail
#endif
};

/*! @file
 * @brief Describes EmergencyStopMC, overrides all joints, allows modelling, blinks tail
 * @author ejt (Creator)
 */

#endif
