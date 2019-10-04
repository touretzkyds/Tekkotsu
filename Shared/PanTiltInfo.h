//-*-c++-*-
#ifndef INCLUDED_PanTiltInfo_h
#define INCLUDED_PanTiltInfo_h

#include <cmath>
#include <stdlib.h>
#include "CommonInfo.h"
using namespace RobotInfo;

// see http://tekkotsu.org/porting.html#configuration for more information on TGT_HAS_* flags
#if defined(TGT_PANTILT)
#  define TGT_IS_PANTILT
#  define TGT_HAS_CAMERA 1
#  define TGT_HAS_HEAD 1
#endif

//! Declares configuration of a pan/tilt camera “robot”, such as number of joints, LEDs, etc.
namespace PanTiltInfo {

	// *******************************
	//       ROBOT CONFIGURATION
	// *******************************

	extern const char* const TargetName; //!< the name of the model, to be used for logging and remote GUIs

	const unsigned int FrameTime=40;        //!< time between frames in the motion system (milliseconds)
	const unsigned int NumFrames=1;        //!< the number of frames per buffer (don't forget also double buffered)
	const unsigned int SoundBufferTime=32; //!< the number of milliseconds per sound buffer... I'm not sure if this can be changed
	
	//!@name Output Types Information
	const unsigned NumWheels      =  0;
	
	const unsigned JointsPerArm   =  0;
	const unsigned NumArms        =  0;
	const unsigned NumArmJoints   =  JointsPerArm*NumArms;
	
	const unsigned JointsPerLeg   =  0; //!< The number of joints per leg
	const unsigned NumLegs        =  0; //!< The number of legs
	const unsigned NumLegJoints   =  JointsPerLeg*NumLegs; //!< the TOTAL number of joints on ALL legs
	const unsigned NumHeadJoints  =  2; //!< The number of joints in the pantilt
	const unsigned NumTailJoints  =  0; //!< The number of joints assigned to the tail
	const unsigned NumMouthJoints =  0; //!< the number of joints that control the mouth
	const unsigned NumEarJoints   =  0; //!< The number of joints which control the ears (NOT per ear, is total)
	const unsigned NumButtons     =  0; //!< the number of buttons that are available
	const unsigned NumSensors     =  0;  //!< the number of sensors available
	const unsigned NumFacePanelLEDs = 0; //!< The number of face panel LEDs

	const unsigned NumLEDs        =  0; //!< The number of LEDs which can be controlled (one per dynamixel servo)
	const unsigned NumPIDJoints   = NumWheels + NumArmJoints + NumLegJoints+NumHeadJoints+NumTailJoints+NumMouthJoints;; //!< servo pins
	
	const unsigned NumOutputs     = NumWheels + NumPIDJoints + NumLEDs; //!< the total number of outputs
	const unsigned NumReferenceFrames = NumOutputs + 1 + 1; //!< for the base and camera reference frames

	using namespace Camera75DOF;
	//@}


	// *******************************
	//         OUTPUT OFFSETS
	// *******************************

	//!Corresponds to entries in ERS7Info::PrimitiveName, defined at the end of this file
	//!@name Output Offsets

	const unsigned PIDJointOffset = 0; //!< The beginning of the PID Joints
	const unsigned HeadOffset = PIDJointOffset; //!< The beginning of the head joints

	const unsigned LEDOffset   = PIDJointOffset + NumPIDJoints; //!< the offset of LEDs in WorldState::outputs and MotionCommand functions, see LedOffset_t for specific offsets

	const unsigned BaseFrameOffset   = NumOutputs; //!< Use with kinematics to refer to base reference frame
	const unsigned CameraFrameOffset = BaseFrameOffset+1; //!< Use with kinematics to refer to camera reference frame

	//! The offsets of appendages with pan (heading), tilt (elevation), note that this should be added to HeadOffset, otherwise use HeadOffset_t (#HeadPanOffset and #HeadTiltOffset)
	enum TPROffset_t {
		PanOffset = 0,      //!< pan/heading (horizontal)
		TiltOffset, //!< tilt/elevation (vertical)
	};
	
	//! These are 'absolute' offsets for the neck joints, don't need to add to HeadOffset like TPROffset_t values do
	enum HeadOffset_t {
		HeadPanOffset = HeadOffset,      //!< pan/heading (horizontal)
		HeadTiltOffset, //!< tilt/elevation (vertical)
	};
	
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
	enum SensorOffset_t { };

	//! Provides a string name for each sensor
	const char* const sensorNames[NumSensors+1] = { NULL }; // non-empty array to avoid gcc 3.4.2 internal error

	//@}


	//! Names for each of the outputs
	const char* const outputNames[NumReferenceFrames+1] = {
		"NECK:pan",
		"NECK:tilt",
		"BaseFrame",
		"CameraFrame",
		NULL
	};
	
	//! allocation declared in RobotInfo.cc
	extern const Capabilities capabilities;
	
	//! Dynamixel servos don't use PID control.  Instead, these values indicate compliance slope (P), punch (add to P*error), compliance margin (min error to start applying torque) (see ServoParam_t)
	/*! I believe the torque calculation goes something like: torque = (error<compliance) ? 0 : punch + P*error
	 *  Dynamixel servos allow different values to be supplied for CW vs. CCW motion, but we just use the same value for each */
	const float DefaultPIDs[NumPIDJoints][3] = {
		{32,32,0}, {32,32,0}
	};
	
	//!These values are our recommended maximum joint velocities, in rad/ms
	/*! a value <= 0 means infinite speed (e.g. LEDs)
	 *  
	 *  These limits are <b>not</b> enforced by the framework.  They are simply available for you to use as you see fit.
	 *  HeadPointerMC and PostureMC are primary examples of included classes which do respect these values (although they can be overridden)
	 */
	const float MaxOutputSpeed[NumOutputs] = {
		// servos
		0.8f,0.8f,
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
		{RAD(-150),RAD(150)}, {RAD(-92),RAD(75)},
	};

	//! This table holds the mechanical limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
	const float mechanicalLimits[NumOutputs][2] = {
		// servos
		{RAD(-150),RAD(150)}, {RAD(-92),RAD(75)},
	};

#ifdef __RI_RAD_FLAG
#undef RAD
#undef __RI_RAD_FLAG
#endif
}

/*! @file
 * @brief Defines some capabilities of a simple pan-tilt camera “robot”
 * @author ejt (Creator)
 */

#endif
