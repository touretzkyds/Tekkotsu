//-*-c++-*-
#ifndef INCLUDED_HandEyeInfo_h
#define INCLUDED_HandEyeInfo_h

#include <cmath>
#include <stdlib.h>
#include "CommonInfo.h"
using namespace RobotInfo;

// see http://tekkotsu.org/porting.html#configuration for more information on TGT_HAS_* flags
#if defined(TGT_HANDEYE)
#  define TGT_IS_BIOLOID
#  define TGT_IS_HANDEYE
#  define TGT_HAS_CAMERA 1
//UPGRADE: set TGT_HAS_LEDS to 7
#  define TGT_HAS_LEDS 5
#  define TGT_HAS_ARMS 1
#  define TGT_HAS_HEAD 1
#  define WALKMC_NO_WARN_NOOP
#endif

//! Declares configuration of the HandEye planar arm robot, such as number of joints, LEDs, etc.
namespace HandEyeInfo {

	// *******************************
	//       ROBOT CONFIGURATION
	// *******************************

	extern const char* const TargetName; //!< the name of the model, to be used for logging and remote GUIs

	const unsigned int FrameTime=32;        //!< time between frames in the motion system (milliseconds)
	const unsigned int NumFrames=1;        //!< the number of frames per buffer (don't forget also double buffered)
	const unsigned int SoundBufferTime=32; //!< the number of milliseconds per sound buffer... I'm not sure if this can be changed
	
	//!@name Output Types Information
	const unsigned NumWheels      =  0;
	
	const unsigned FingerJointsPerArm = 0; // UPGRADE: 2
	const unsigned JointsPerArm   =  3; // UPGRADE: 5;
	const unsigned NumArms        =  1;
	const unsigned NumArmJoints   =  JointsPerArm*NumArms;
	
	const unsigned JointsPerLeg   =  0; //!< The number of joints per leg
	const unsigned NumLegs        =  0; //!< The number of legs
	const unsigned NumLegJoints   =  JointsPerLeg*NumLegs; //!< the TOTAL number of joints on ALL legs
	const unsigned NumHeadJoints  =  2; //!< The number of joints in the pantilt
	const unsigned NumTailJoints  =  0; //!< The number of joints assigned to the tail
	const unsigned NumMouthJoints =  0; //!< the number of joints that control the mouth
	const unsigned NumEarJoints   =  0; //!< The number of joints which control the ears (NOT per ear, is total)
	const unsigned NumButtons     =  3; //!< the number of buttons that are available
	const unsigned NumSensors     =  2;  //!< the number of sensors available
	const unsigned NumFacePanelLEDs = 0; //!< The number of face panel LEDs

	// UPGRADE: NumLEDs to 7
	const unsigned NumLEDs        =  5; //!< The number of LEDs which can be controlled (one per dynamixel servo)
	const unsigned NumPIDJoints   = NumWheels + NumArmJoints + NumLegJoints + NumHeadJoints + NumTailJoints + NumMouthJoints;; //!< servo pins
	
	const unsigned NumOutputs     = NumWheels + NumPIDJoints + NumLEDs; //!< the total number of outputs
	const unsigned NumReferenceFrames = NumOutputs + 1 + NumArms + 1; //!< for the base, gripper (* NumArms), and camera reference frames

	using namespace Camera75DOF;
	//@}


	// *******************************
	//         OUTPUT OFFSETS
	// *******************************

	//!Corresponds to entries in ERS7Info::PrimitiveName, defined at the end of this file
	//!@name Output Offsets

	
	const unsigned PIDJointOffset = 0; //!< The beginning of the PID Joints
	
	const unsigned ArmOffset   = PIDJointOffset;  //!< beginning of arm joints, don't add anything, just use ArmOffset_t entries directly
	
	const unsigned HeadOffset  = ArmOffset+NumArmJoints;   //!< the offset of the beginning of the head joints, add TPROffset_t to get specific joint

	const unsigned LEDOffset   = PIDJointOffset + NumPIDJoints; //!< the offset of LEDs in WorldState::outputs and MotionCommand functions, see LedOffset_t for specific offsets

	const unsigned BaseFrameOffset   = NumOutputs; //!< Use with kinematics to refer to base reference frame
	const unsigned GripperFrameOffset    = BaseFrameOffset+1; //!< Use with kinematics to refer to paw reference frames (add appropriate LegOrder_t to specify which paw)
	const unsigned CameraFrameOffset = GripperFrameOffset+NumArms; //!< Use with kinematics to refer to camera reference frame

	//! These are 'absolute' offsets for the arm joints
	enum ArmOffset_t {
		ArmShoulderOffset = ArmOffset,
		ArmElbowOffset,
		WristOffset,
		WristYawOffset = WristOffset,
		//WristPitchOffset,
		//WristRollOffset,
		//GripperOffset
	};
	
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
	
	//! The offsets of the individual LEDs
	/*! @hideinitializer */
	enum LEDOffset_t {
		ArmShoulderLEDOffset = LEDOffset, //!< Small led on the shoulder servo
		ArmElbowLEDOffset, //!< Small LED on the elbow servo
		ArmWristLEDOffset, //!< Small LED on the wrist servo
		NeckPanLEDOffset, //!< Small LED on the neck pan servo
		NeckTiltLEDOffset //!< Small LED on the neck tilt servo
};
	
	typedef unsigned int LEDBitMask_t; //!< So you can be clear when you're refering to a LED bitmask

	const LEDBitMask_t ArmShoulderLEDMask = 1 << (ArmShoulderLEDOffset-LEDOffset); //!< Mask corresponding to ArmShoulderLEDOffset
	const LEDBitMask_t ArmElbowLEDMask    = 1 << (ArmElbowLEDOffset-LEDOffset); //!< Mask corresponding to ArmElbowLEDOffset
	const LEDBitMask_t ArmWristLEDMask    = 1 << (ArmWristLEDOffset-LEDOffset); //!< Mask corresponding to ArmWristLEDOffset
	const LEDBitMask_t NeckPanLEDMask     = 1 << (NeckPanLEDOffset-LEDOffset); //!< Mask corresponding to NeckPanLEDOffset
	const LEDBitMask_t NeckTiltLEDMask    = 1 << (NeckTiltLEDOffset-LEDOffset); //!< Mask corresponding to NeckTiltLEDOffset

	//! LEDs for the face panel (all FaceLEDPanelMask<<(0:NumFacePanelLEDs-1) entries)
	const LEDBitMask_t FaceLEDMask = NeckPanLEDMask | NeckTiltLEDMask;
	//! selects all of the leds
	const LEDBitMask_t AllLEDMask  = (LEDBitMask_t)~0;
	//@}


	//! Offset needed so that the centroid of the robot is correct relative to the bounding box
	const fmat::Column<3> AgentBoundingBoxBaseFrameOffset = fmat::pack(0,0,0); 

	//! Half of the length, width, and height of the robot
	const fmat::Column<3> AgentBoundingBoxHalfDims = fmat::pack(304.8/2, 304.8/2, 0);

	// *******************************
	//          INPUT OFFSETS
	// *******************************


	//! The order in which inputs should be stored
	//!@name Input Offsets

	//! holds offsets to different buttons in WorldState::buttons[]
	/*! Should be a straight mapping to the ButtonSourceIDs
	 *
	 *  @see WorldState::buttons @see ButtonSourceID_t
	 * @hideinitializer */
	enum ButtonOffset_t { GreenButOffset, RedButOffset, YellowButOffset };

	//! Provides a string name for each button
	const char* const buttonNames[NumButtons+1] = { "GreenBut", "RedBut", "YellowBut", NULL };

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
		"ARM:shldr","ARM:elbow","ARM:wrist",// UPGRADE: "ARM:wristPitch","ARM:gripper",
		"NECK:pan", "NECK:tilt",

		"LED:ARM:shldr",
		"LED:ARM:elbow",
		"LED:ARM:wrist",
		"LED:NECK:pan",
		"LED:NECK:tilt",
		//"LED:00005", // UPGRADE
		//"LED:00006", // UPGRADE
		
		"BaseFrame",
		"GripperFrame",
		"CameraFrame",
		NULL
	};
	
	//! provides polymorphic robot capability detection/mapping
	class HandEyeCapabilities : public Capabilities {
	public:
		//! constructor
		HandEyeCapabilities()
		: Capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs)
		{
			frameToIndex["ARM:WristYaw"] = WristYawOffset; // UPGRADE: may want to reverse this, so "WristYaw" is the official display name, and "Wrist" is the alias
			frameToIndex["NECK:nod"] = HeadTiltOffset;
		}
	};
	//! allocation declared in RobotInfo.cc
	extern const HandEyeCapabilities capabilities;
	
	//! Dynamixel servos don't use PID control.  Instead, these values indicate compliance slope (P), punch (add to P*error), compliance margin (min error to start applying torque) (see ServoParam_t)
	/*! I believe the torque calculation goes something like: torque = (error<compliance) ? 0 : punch + P*error
	 *  Dynamixel servos allow different values to be supplied for CW vs. CCW motion, but we just use the same value for each */
	const float DefaultPIDs[NumPIDJoints][3] = {
		{32,32,0}, {32,32,0}, {32,32,0}, // UPGRADE: {32,32,0}, {32,32,0}, 
		{32,32,0}, {32,32,0},
	};
	
	//!These values are our recommended maximum joint velocities, in rad/ms
	/*! a value <= 0 means infinite speed (e.g. LEDs)
	 *  
	 *  These limits are <b>not</b> enforced by the framework.  They are simply available for you to use as you see fit.
	 *  HeadPointerMC and PostureMC are primary examples of included classes which do respect these values (although they can be overridden)
	 */
	const float MaxOutputSpeed[NumOutputs] = {
		// servos
		0.8f, 0.8f, 0.8f, 0.8f, 0.8f,// UPGRADE: 0.8f, 0.8f,
		// leds
		0,0,0,0,0,// UPGRADE: 0,0,
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
		{RAD(-130),RAD(130)}, {RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, //UPGRADE: {0,RAD(300)}, {0,RAD(300)}, 
		{RAD(-63),RAD(63)}, {RAD(-95),RAD(68)},

		// LED
		{0,1}, {0,1}, {0,1}, // UPGRADE: {0,1}, {0,1}, 
		{0,1}, {0,1},
	};

	//! This table holds the mechanical limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
	const float mechanicalLimits[NumOutputs][2] = {
		// servos
		{RAD(-130),RAD(130)}, {RAD(-150),RAD(150)}, {RAD(-150),RAD(150)}, //UPGRADE: {0,RAD(300)}, {0,RAD(300)}, 
		{RAD(-63),RAD(63)}, {RAD(-95),RAD(68)},
		
		// LED
		{0,1}, {0,1}, {0,1}, // UPGRADE: {0,1}, {0,1}, 
		{0,1}, {0,1},
	};

#ifdef __RI_RAD_FLAG
#undef RAD
#undef __RI_RAD_FLAG
#endif
}

/*! @file
 * @brief Defines some capabilities of the HandEye planar arm robot
 * @author ejt (Creator)
 */

#endif
