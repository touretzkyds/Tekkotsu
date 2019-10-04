//-*-c++-*-
#ifndef INCLUDED_BioloidInfo_h
#define INCLUDED_BioloidInfo_h

#include <cmath>
#include <stdlib.h>
#include "CommonInfo.h"
using namespace RobotInfo;

// see http://tekkotsu.org/porting.html#configuration for more information on TGT_HAS_* flags
#if defined(TGT_BIOLOID)
#  define TGT_IS_BIOLOID
#  define TGT_HAS_WEBCAM 1
#  ifdef BIOLOID_BEGINNER_KIT
#    define TGT_HAS_LEDS 4
#  else
#    define TGT_HAS_LEDS 18
#  endif
#endif

//! Declares configuration of generic bioloid/dynamixel based robots, such as number of joints, LEDs, etc.
namespace BioloidInfo {

	// *******************************
	//       ROBOT CONFIGURATION
	// *******************************

	extern const char* const TargetName; //!< the name of the model, to be used for logging and remote GUIs

	const unsigned int FrameTime=32;        //!< time between frames in the motion system (milliseconds)
	const unsigned int NumFrames=1;        //!< the number of frames per buffer (don't forget also double buffered)
	const unsigned int SoundBufferTime=32; //!< the number of milliseconds per sound buffer... I'm not sure if this can be changed
	
	//!@name Output Types Information
	const unsigned NumWheels      =  0;
	
	const unsigned FingerJointsPerArm = 0;
	const unsigned JointsPerArm   =  0;
	const unsigned NumArms        =  0;
	const unsigned NumArmJoints   =  JointsPerArm*NumArms;
	
	const unsigned JointsPerLeg   =  0; //!< The number of joints per leg
	const unsigned NumLegs        =  0; //!< The number of legs
	const unsigned NumLegJoints   =  JointsPerLeg*NumLegs; //!< the TOTAL number of joints on ALL legs
	const unsigned NumHeadJoints  =  0; //!< The number of joints in the pantilt
	const unsigned NumTailJoints  =  0; //!< The number of joints assigned to the tail
	const unsigned NumMouthJoints =  0; //!< the number of joints that control the mouth
	const unsigned NumEarJoints   =  0; //!< The number of joints which control the ears (NOT per ear, is total)
	const unsigned NumButtons     =  0; //!< the number of buttons that are available
	const unsigned NumSensors     =  2;  //!< the number of sensors available (voltage and temperature)
	const unsigned NumFacePanelLEDs = 0; //!< The number of face panel LEDs

#ifdef BIOLOID_BEGINNER_KIT
	const unsigned NumLEDs        =  4; //!< The number of LEDs which can be controlled (one per dynamixel servo)
	const unsigned NumPIDJoints   = 4 + NumWheels + NumArmJoints + NumLegJoints+NumHeadJoints+NumTailJoints+NumMouthJoints;; //!< servo pins
#else
	const unsigned NumLEDs        =  18; //!< The number of LEDs which can be controlled (one per dynamixel servo)
	const unsigned NumPIDJoints   = 18 + NumWheels + NumArmJoints + NumLegJoints+NumHeadJoints+NumTailJoints+NumMouthJoints;; //!< servo pins
#endif
	
	const unsigned NumOutputs     = NumWheels + NumPIDJoints + NumLEDs; //!< the total number of outputs
	const unsigned NumReferenceFrames = NumOutputs+1; //!< Since we don't know what the user built, we don't actually know any of the kinematics...

	using namespace Camera75DOF;
	//@}


	// *******************************
	//         OUTPUT OFFSETS
	// *******************************

	//!Corresponds to entries in ERS7Info::PrimitiveName, defined at the end of this file
	//!@name Output Offsets

	
	const unsigned PIDJointOffset = 0; //!< The beginning of the PID Joints
	const unsigned WheelOffset = PIDJointOffset;

	const unsigned LEDOffset   = PIDJointOffset + NumPIDJoints; //!< the offset of LEDs in WorldState::outputs and MotionCommand functions, see LedOffset_t for specific offsets

	const unsigned BaseFrameOffset   = NumOutputs; //!< Use with kinematics to refer to base reference frame

	enum WheelOffset_t {
		LWheelOffset=WheelOffset,
		RWheelOffset
	};
	
	//! The offsets of the individual LEDs -- except we don't have any idea what to 'name' the servos in a reconfigurable kit, so this is empty (just add numeric offsets)
	/*! @hideinitializer */
	enum LEDOffset_t { };
	
	typedef unsigned int LEDBitMask_t; //!< So you can be clear when you're refering to a LED bitmask
	//! LEDs for the face panel (all FaceLEDPanelMask<<(0:NumFacePanelLEDs-1) entries)
	const LEDBitMask_t FaceLEDMask = 0;
	//! selects all of the leds
	const LEDBitMask_t AllLEDMask  = (LEDBitMask_t)~0;
	//@}


	// *******************************
	//          INPUT OFFSETS
	// *******************************


	//! The order in which inputs should be stored
	//!@name Input Offsets

	//! holds offsets to different buttons in WorldState::buttons[]
	/*! Should be a straight mapping to the ButtonSourceIDs
	 *
	 *  Note that the chest (power) button is not a normal button.  It kills
	 *  power to the motors at a hardware level, and isn't sensed in the
	 *  normal way.  If you want to know when it is pressed (and you are
	 *  about to shut down) see PowerSrcID::PauseSID.
	 *
	 *  @see WorldState::buttons @see ButtonSourceID_t
	 * @hideinitializer */
	enum ButtonOffset_t { };

	//! Provides a string name for each button
	const char* const buttonNames[NumButtons+1] = { NULL }; // non-empty array to avoid gcc 3.4.2 internal error

	//! holds offset to different sensor values in WorldState::sensors[]
	/*! @see WorldState::sensors[] */
	enum SensorOffset_t {
		PowerThermoOffset, //!< degrees Celcius
		PowerVoltageOffset, //!< volts
	};

	//! Provides a string name for each sensor
	const char* const sensorNames[NumSensors+1] = {
		"PowerThermo","PowerVoltage", NULL
	};

	//@}


	//! Names for each of the outputs
	const char* const outputNames[NumReferenceFrames+1] = {
		"SERVO:000",
		"SERVO:001",
		"SERVO:002",
		"SERVO:003",
#ifndef BIOLOID_BEGINNER_KIT
		"SERVO:004",
		"SERVO:005",
		"SERVO:006",
		"SERVO:007",
		"SERVO:008",
		"SERVO:009",
		"SERVO:010",
		"SERVO:011",
		"SERVO:012",
		"SERVO:013",
		"SERVO:014",
		"SERVO:015",
		"SERVO:016",
		"SERVO:017",
#endif
		"LED:00000",
		"LED:00001",
		"LED:00002",
		"LED:00003",
#ifndef BIOLOID_BEGINNER_KIT
		"LED:00004",
		"LED:00005",
		"LED:00006",
		"LED:00007",
		"LED:00008",
		"LED:00009",
		"LED:00010",
		"LED:00011",
		"LED:00012",
		"LED:00013",
		"LED:00014",
		"LED:00015",
		"LED:00016",
		"LED:00017",
#endif
		"BaseFrame",
		NULL
	};
	
	//! allocation declared in RobotInfo.cc
	extern const Capabilities capabilities;
	
	//! Dynamixel servos don't use PID control.  Instead, these values indicate compliance slope (P), punch (add to P*error), compliance margin (min error to start applying torque) (see ServoParam_t)
	/*! I believe the torque calculation goes something like: torque = (error<compliance) ? 0 : punch + P*error
	 *  Dynamixel servos allow different values to be supplied for CW vs. CCW motion, but we just use the same value for each */
	const float DefaultPIDs[NumPIDJoints][3] = {
		{32,32,0}, {32,32,0}, {32,32,0}, {32,32,0}, 
#ifndef BIOLOID_BEGINNER_KIT
		{32,32,0}, {32,32,0}, {32,32,0}, {32,32,0}, 
		{32,32,0}, {32,32,0}, {32,32,0}, {32,32,0}, 
		{32,32,0}, {32,32,0}, {32,32,0}, {32,32,0}, 
		{32,32,0}, {32,32,0},
#endif
	};
	
	//!These values are our recommended maximum joint velocities, in rad/sec
	/*! a value <= 0 means infinite speed (e.g. LEDs)
	 *  
	 *  These limits are <b>not</b> enforced by the framework.  They are simply available for you to use as you see fit.
	 *  HeadPointerMC and PostureMC are primary examples of included classes which do respect these values (although they can be overridden)
	 *  
	 *  These values were obtained from the administrators of the Sony OPEN-R BBS */
	const float MaxOutputSpeed[NumOutputs] = {
		// servos
		0.4f,0.4f,0.4f,0.4f,
#ifndef BIOLOID_BEGINNER_KIT
		0.4f,0.4f,0.4f,0.4f, 0.4f,0.4f,0.4f,0.4f, 0.4f,0.4f,0.4f,0.4f, 0.4f,0.4f,
#endif
		// leds
		0.4f,0.4f,0.4f,0.4f,
#ifndef BIOLOID_BEGINNER_KIT
		0.4f,0.4f,0.4f,0.4f, 0.4f,0.4f,0.4f,0.4f, 0.4f,0.4f,0.4f,0.4f, 0.4f,0.4f,
#endif
	};

	#ifndef RAD
		//!Just a little macro for converting degrees to radians
	#define RAD(deg) (((deg) * (float)M_PI ) / 180.0f)
		//!a flag so we undef these after we're done - do you have a cleaner solution?
	#define __RI_RAD_FLAG
	#endif
	
	//! This table holds the software limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
	const float outputRanges[NumOutputs][2] = {
		// servos
		{RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, 
#ifndef BIOLOID_BEGINNER_KIT
		{RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, {RAD(-150),RAD(150)},
		{RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, {RAD(-150),RAD(150)},
		{RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, {RAD(-150),RAD(150)},
		{RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, 
#endif

		// LED
		{0,1}, {0,1}, {0,1}, {0,1},
#ifndef BIOLOID_BEGINNER_KIT
		{0,1}, {0,1}, {0,1}, {0,1},
		{0,1}, {0,1}, {0,1}, {0,1},
		{0,1}, {0,1}, {0,1}, {0,1},
		{0,1}, {0,1},
#endif
	};

	//! This table holds the mechanical limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
	/*! Same as #outputRanges, don't know actual values because they were never specified by Sony */
	const float mechanicalLimits[NumOutputs][2] = {
		// servos
		{RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, 
#ifndef BIOLOID_BEGINNER_KIT
		{RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, {RAD(-150),RAD(150)},
		{RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, {RAD(-150),RAD(150)},
		{RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, {RAD(-150),RAD(150)},
		{RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, 
#endif
		
		// LED
		{0,1}, {0,1}, {0,1}, {0,1},
#ifndef BIOLOID_BEGINNER_KIT
		{0,1}, {0,1}, {0,1}, {0,1},
		{0,1}, {0,1}, {0,1}, {0,1},
		{0,1}, {0,1}, {0,1}, {0,1},
		{0,1}, {0,1},
#endif
	};

#ifdef __RI_RAD_FLAG
#undef RAD
#undef __RI_RAD_FLAG
#endif
}

/*! @file
 * @brief Defines some capabilities common to generic bioloid/dynamixel based robots
 * @author ejt (Creator)
 */

#endif
