//-*-c++-*-
#ifndef INCLUDED_WorldState_h
#define INCLUDED_WorldState_h

#ifdef PLATFORM_APERIOS
#  include <OPENR/core_macro.h>
#  include <OPENR/ObjcommTypes.h>
#  include <OPENR/OPENR.h>
#  include <OPENR/OPENRAPI.h>
#  include <OPENR/OPENRMessages.h>
#else
class SensorState;
#endif

#include "Shared/RobotInfo.h"
#include "IPC/ProcessID.h"
#include <vector>

class EventRouter;
class EventBase;

//The following SourceIDs are for events created by WorldState's event generators

//! holds source ID types for sensor events; see EventBase, see #SensorSourceID_t
namespace SensorSrcID {
	//! holds source ID types for sensor events
	/*! May want to add a proximity alarm for IR distance?  Probably
	 *  should do it from a separate generator to avoid screwing up
	 *  behaviors relying on the current setup
	 */
	enum SensorSourceID_t {
		UpdatedSID //!< sends status event as last event after processing a frame
	};
}

//! holds source ID types for power events; see EventBase, see #PowerSourceID_t
namespace PowerSrcID {
	//! holds source ID types for power events
	/*! Also serve as offsets into WorldState::powerFlags[].
	 *
	 *  I've never seen a lot of these events thrown by the OS.  'NS'
	 *  means never-seen, which could simply be because i haven't put it
	 *  in that situation (don't have a station-type power charger) or
	 *  because the OS doesn't actually support sending that flag.
	 *
	 *  Under normal conditions, you'll see MotorPowerSID,
	 *  BatteryConnectSID, DischargingSID, and PowerGoodSID always
	 *  active with occasional VibrationSID and UpdateSID.  When the
	 *  chest button is pushed, PauseSID is activated and MotorPowerSID
	 *  is deactivated.
	 *
	 *  The BatteryMonitorBehavior will give a warning once power begins
	 *  getting low.  The OS won't boot off a battery with less than 15%
	 *  power remaining (which is when the LowPowerWarnSID is thrown)
	 *
	 *  @note there's not a one-to-one correspondance of the events from
	 *  the OPENR power system... i map several OPENR events to fewer
	 *  Tekkotsu events, check the event's name if you want to know the
	 *  specific source (say if low battery is low current and/or low
	 *  voltage) Status ETIDS are only generated when one of a related
	 *  group goes on/off but others are still active
	 */
	enum PowerSourceID_t {
		PauseSID=0, //!< the chest button was pushed (this is not a normal button, it kills power to the motors in hardware)
		MotorPowerSID, //!< active while the motors have power
		VibrationSID, //!< triggered when the OS decides a large acceleration has occured, like falling down (or more specifically, hitting the ground afterward)
		BatteryEmptySID, //!< battery is dead
		LowPowerWarnSID, //!< triggered when sensors[PowerRemainOffset] <= 0.15 (PowerGoodSID stays on)
		BatteryFullSID,  //!< battery is full
		ExternalPowerSID, //!< receiving power from an external source (such as AC cable, may or may not include the "station", see StationConnectSID)
		ExternalPortSID,  //!< an external power source is plugged in (does not imply current is flowing however)
		BatteryConnectSID, //!< a battery is plugged in
		BatteryInitSID, //!< ? NS
		DischargingSID, //!< using power from the battery (although still stays on after hooked up to external power)
		ChargingSID, //!< you used to be able to charge while running, tho that has changed in more recent versions of OPEN-R.  In any case, I never saw this even when it did work.
		OverheatingSID, //!< in case the robot starts getting too hot NS
		PowerGoodSID, //!< there is power, either from external or battery
		ChargerStatusSID, //!< ? NS
		SuspendedSID, //!< ? NS
		OverChargedSID, //!< in case the charger screws up somehow (?) NS
		TermDischargeSID, //!< end of battery (?) NS
		TermChargeSID, //!< end of charging (?) NS
		ErrorSID, //!< general power error NS
		StationConnectSID, //!< connected to a station NS
		BatteryOverCurrentSID, //!< similar to OverChargedSID (?) NS
		DataFromStationSID, //!< ? NS
		RegisterUpdateSID, //!< ? NS
		RTCSID, //!< ? NS
		SpecialModeSID, //!< ? NS
		BMNDebugModeSID, //!< ? NS
		PlungerSID, //!< I think this is in reference to having a memorystick (?) NS
		UpdatedSID, //!< sent as last event after processing a frame
		NumPowerSIDs
	};
}

//! The state of the robot and its environment
/*! Contains sensor readings, current joint positions, etc.  Notable members are:
 * - #outputs - joint positions and current LED values
 * - #buttons - current button values
 * - #sensors - values from other sensors (IR, acceleration, temperature, power levels)
 * - #pids - current PID settings for each joint (more specifically, each PID-controlled joint)
 * - #pidduties - how hard each of the PID joints is working to get to its target value
 *
 * Generally you will use enumerated values from a robot-specific namespace to
 * index into these arrays.  Each of those members' documentation specifies where
 * to find the list of indices to use with them.
 *
 * This is a shared memory region between Main, Motion, and possibly others in the future.
 * Be very careful about including structures that use pointers in
 * this class... they will only be valid from the OObject that created
 * them, others may cause a crash if they try to access them.
 *
 * WorldState takes power and sensor updates from the system and
 * maintains the last known values in its member fields.  It throws
 * events when some of these values change, listed in the
 * SensorSourceID, PowerSourceID namespaces, and the ButtonOffset_t
 * enumeration for your robot model's info
 * namespace. (e.g. ERS7Info::ButtonOffset_t)
 *
 * Status events for buttons only generated if the
 * WorldState::alwaysGenerateStatus flag is turned on.  Otherwise, by
 * default, they are only generated when a value has changed.
 * (i.e. when the pressure sensitive buttons get a new pressure
 * reading)
 */
class WorldState {
public:
	//! constructor - sets everything to zeros
	WorldState();

	bool alwaysGenerateStatus; //!< controls whether status events are generated for the boolean buttons

	float outputs[NumOutputs];     //!< last sensed positions of joints, last commanded value of LEDs; indexes (aka offsets) are defined in the target model's namespace (e.g. "Output Offsets" section of ERS7Info)
	float buttons[NumButtons];     //!< magnitude is pressure for some, 0/1 for others; indexes are defined in the ButtonOffset_t of the target model's namespace (e.g. ERS7Info::ButtonOffset_t)
	float sensors[NumSensors];     //!< IR, Accel, Thermo, Power stuff; indexes are defined in SensorOffset_t of the target model's namespace (e.g. ERS7Info::SensorOffset_t)
	float pids[NumPIDJoints][3];   //!< current PID settings (same ordering as the #outputs), not sensed -- updated by MotionManager whenever it sends a PID setting to the system; note this is only valid for PID joints, has NumPIDJoint entries (as opposed to NumOutputs)
	float pidduties[NumPIDJoints]; //!< duty cycles - -1 means the motor is trying to move full power in negative direction, 1 means full power in positive direction, in practice, these values stay rather small - 0.15 is significant force. (same ordering as the #outputs); note this is only valid for PID joints, has NumPIDJoint entries (as opposed to NumOutputs)
	
	float vel_x; //!< the current, egocentric rate of forward locomotion, mm/second
	float vel_y; //!< the current, egocentric rate of sideways (leftward is positive) locomotion, mm/second
	float vel_a; //!< the current, egocentric rate of rotational (counterclockwise is positive) locomotion, radian/second
	unsigned int vel_time; //!< the time at which we began moving along the current velocity vector

	unsigned int robotStatus;       //!< bitmask, see OPENR/OPower.h
	unsigned int batteryStatus;     //!< bitmask, see OPENR/OPower.h
	unsigned int powerFlags[PowerSrcID::NumPowerSIDs]; //!< bitmasks of similarly-grouped items from previous two masks, corresponds to the PowerSrcID::PowerSourceID_t's

	unsigned int button_times[NumButtons]; //!< value is time of current press, 0 if not down
	
	unsigned int lastSensorUpdateTime;     //!< primarily so calcDers can determine the time difference between updates, but others might want to know this too...
	unsigned int frameNumber;              //!< the serial number of the currently available frame
	unsigned int framesProcessed;          //!< the number of sensor updates which have been processed

	static const double g;                 //!< the gravitational acceleration of objects on earth
	static const double IROORDist;         //!< If IR returns this, we're out of range

#ifdef PLATFORM_APERIOS
	void read(OSensorFrameVectorData& sensor, EventRouter* er); //!< will process a sensor reading as given by OPEN-R
	void read(const OPowerStatus& power, EventRouter* er);      //!< will process a power status update as given by OPEN-R
	
	//! returns the current WorldState instance for the running process, needed if your code may be in a shared memory region; otherwise you can directly access ::state
	/*! Generally you can access ::state directly, but if your code is running as a member of an object in a shared memory region,
	 *  this handles the shared object context switching problem on Aperios, and simply returns ::state on non-Aperios. */
	static WorldState* getCurrent();
	
#else
	//! processes incoming sensor data, generating events for buttons
	void read(const SensorState& sensor, bool sendEvents);
	
	//! returns the current WorldState instance for the running process, needed if your code may be in a shared memory region; otherwise you can directly access ::state
	/*! Generally you can access ::state directly, but if your code is running as a member of an object in a shared memory region,
	 *  this handles the shared object context switching problem on Aperios, and simply returns ::state on non-Aperios. */
	static const WorldState* getCurrent();
#endif

protected:
	unsigned int curtime; //!< set by read(OSensorFrameVectorData& sensor, EventRouter* er) for chkEvent() so each call doesn't have to

	//! Tests to see if the button status has changed and post events as needed
	void chkEvent(std::vector<EventBase>& evtBuf, unsigned int off, float newval, const char* name);

	//! Apply calibration to the sensors and joint positions (reversing motion calibration parameters)
	void applyCalibration();

	//! sets the names of the flags that will be generating events
	/*! note that this function does not actually do the event posting,
	 *  unlike chkEvent() */
	void chkPowerEvent(unsigned int sid, unsigned int cur, unsigned int mask, const char* name, 
										 std::string actname[PowerSrcID::NumPowerSIDs],
										 std::string dename[PowerSrcID::NumPowerSIDs],
										 unsigned int summask[PowerSrcID::NumPowerSIDs]) {
		if(cur&mask) {
			actname[sid]+=name;
			summask[sid]|=mask;
		} else if(powerFlags[sid]&mask)
			dename[sid]+=name;
	}

	//! given the next value, calculates and stores the next current, the velocity, and the acceleration
	/*! @param next the new value that's about to be set
	 *  @param cur the previous value
	 *  @param vel the previous 1st derivative
	 *  @param acc the previous 2nd derivative
	 *  @param invtimediff @f$1/(curtime-prevtime)@f$ in seconds*/
	inline void calcDers(double next, double& cur, double& vel, double& acc, double invtimediff) {
		double diff=next-cur;
		cur=next;
		next=diff*invtimediff;;
		diff=next-vel;
		vel=next;
		acc=diff*invtimediff;
	}
	
private:
	WorldState(const WorldState&); //!< this shouldn't be called...
	WorldState& operator=(const WorldState&); //!< this shouldn't be called...
};

#ifdef PLATFORM_APERIOS
extern WorldState * state; //!< the global state object, points into a shared memory region, created by MainObject
inline WorldState * WorldState::getCurrent() { return state; }

#else

//! This class masquerades as a simple WorldState pointer, but actually checks the process ID of the referencing thread to allow each thread group to have a separate WorldState*
/*! This is so if a behavior in Main is blocking, it doesn't prevent Motion threads from getting updated sensors. */
class WorldStateLookup {
public:
	WorldStateLookup() {} //!< constructor
	WorldState* operator->() { return &ws[ProcessID::getID()]; } //!< smart pointer to the underlying class
	WorldState& operator*() { return ws[ProcessID::getID()]; } //!< smart pointer to the underlying class
	operator WorldState*() { return &ws[ProcessID::getID()]; } //!< pretend we're a simple pointer
	
protected:
	//! This holds a separate WorldState pointer for each process
	/*! Note that under a multi-process model, each process is only ever going to reference one of these,
	 *  (so we could get away with a single global pointer), but under a uni-process model, we wind up
	 *  using the various entries to differentiate the thread groups */
	WorldState ws[ProcessID::NumProcesses];
private:
	WorldStateLookup(const WorldStateLookup&); //!< don't call this
	WorldStateLookup& operator=(const WorldStateLookup&); //!< don't call this
};
//! the global state object, points into a shared memory region, created by MainObject
extern WorldStateLookup state;

inline const WorldState* WorldState::getCurrent() { return ::state; }

#endif

/*! @file
 * @brief Describes WorldState, maintains information about the robot's environment, namely sensors and power status
 * @author ejt (Creator)
 */

#endif
