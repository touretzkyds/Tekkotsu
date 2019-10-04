//-*-c++-*-
#ifndef INCLUDED_Chiara2Info_h
#define INCLUDED_Chiara2Info_h

#include <cmath>
#include <stdlib.h>
#include "CommonInfo.h"
using namespace RobotInfo;

// see http://tekkotsu.org/porting.html#configuration for more information on TGT_HAS_* flags
#if defined(TGT_CHIARA2)
#  define TGT_IS_BIOLOID
#  define TGT_IS_CHIARA
#  define TGT_IS_CHIARA2
#  define TGT_HAS_CAMERA 1
#  define TGT_HAS_LEDS 7
#  define TGT_HAS_BUTTONS 3
#  define TGT_HAS_LEGS 6
#  define TGT_HAS_SEK_LEGS 6
#  define TGT_HAS_ARMS 1
#  define TGT_HAS_HEAD 1
#  define TGT_HAS_POWER_STATUS
#  define TGT_HAS_IR_DISTANCE 3
#endif

//! Contains information about an Chiara hexapod robot, such as number of joints, LEDs, etc.
namespace Chiara2Info {

	// *******************************
	//       ROBOT CONFIGURATION
	// *******************************

	extern const char* const TargetName; //!< the name of the model, to be used for logging and remote GUIs

	const unsigned int FrameTime=32;        //!< time between frames in the motion system (milliseconds)
	const unsigned int NumFrames=1;        //!< the number of frames per buffer (don't forget also double buffered)
	const unsigned int SoundBufferTime=32; //!< the number of milliseconds per sound buffer... I'm not sure if this can be changed
	
	//!@name Output Types Information
	const unsigned NumWheels      =  0;
	
	const unsigned FingerJointsPerArm = 2;
	const unsigned JointsPerArm   =  6;
	const unsigned NumArms        =  1;
	const unsigned NumArmJoints   =  JointsPerArm*NumArms;
	
	const unsigned JointsPerLeg   =  3; //!< The number of joints per leg
	const unsigned NumLegs        =  6; //!< The number of legs
	const unsigned NumLegJoints   =  JointsPerLeg*NumLegs; //!< the TOTAL number of joints on ALL legs
	const unsigned NumHeadJoints  =  2; //!< The number of joints in the pantilt
	const unsigned NumTailJoints  =  0; //!< The number of joints assigned to the tail
	const unsigned NumMouthJoints =  0; //!< the number of joints that control the mouth
	const unsigned NumEarJoints   =  0; //!< The number of joints which control the ears (NOT per ear, is total)
	const unsigned NumButtons     =  3; //!< the number of buttons that are available
	const unsigned NumSensors     =  11;  //!< the number of sensors available
	const unsigned NumFacePanelLEDs = 0; //!< The number of face panel LEDs

	const unsigned NumLEDs        =  NumLegs + 1; //!< There's an LED on every dynamixel, but can't see most of them, so save some computational resources and only expose the visible ones: knees and pan
	const unsigned NumPIDJoints   = NumArmJoints + 1 + NumLegJoints + NumHeadJoints+NumTailJoints+NumMouthJoints;; //!< servo pins
	
	const unsigned NumOutputs     = NumWheels + NumPIDJoints + NumLEDs; //!< the total number of outputs
	const unsigned NumReferenceFrames = NumOutputs + 1 + NumLegs + NumArms + 1 + 3; //!< for the base, feet, gripper, camera, and IR distance rangefinder

	using namespace Camera75DOF;
	
	const float BallOfFootRadius=0; //!< radius of the ball of the foot
	//@}


	// *******************************
	//         OUTPUT OFFSETS
	// *******************************

	//!Corresponds to entries in ERS7Info::PrimitiveName, defined at the end of this file
	//!@name Output Offsets

	
	const unsigned PIDJointOffset = 0; //!< The beginning of the PID Joints
	const unsigned RFrRotatorOffset = PIDJointOffset; //!< The right front leg's rotator joint
	const unsigned LegOffset   = RFrRotatorOffset+1;           //!< the offset of the beginning of the regular leg joints (after the 1 rotator joint for the right front leg):  #NumLegs of #JointsPerLeg each, in #LegOrder_t order; see #LegOffset_t
	const unsigned ArmOffset  = LegOffset+NumLegJoints;   //!< the offset of the beginning of the arm joints, add #TPROffset_t to get specific joint
	const unsigned HeadOffset  = ArmOffset+NumArmJoints;   //!< the offset of the beginning of the head joints, add #TPROffset_t to get specific joint

	const unsigned LEDOffset   = PIDJointOffset + NumPIDJoints; //!< the offset of LEDs in WorldState::outputs and MotionCommand functions, see LedOffset_t for specific offsets

	const unsigned BaseFrameOffset   = NumOutputs; //!< Use with kinematics to refer to base reference frame
	const unsigned FootFrameOffset    = BaseFrameOffset+1; //!< Use with kinematics to refer to feet reference frames (add appropriate LegOrder_t to specify which paw)
	const unsigned GripperFrameOffset = FootFrameOffset+NumLegs; //!< Use with kinematics to refer to gripper reference frame
	const unsigned CameraFrameOffset = GripperFrameOffset+NumArms; //!< Use with kinematics to refer to camera reference frame
	
	const unsigned LeftIRFrameOffset = CameraFrameOffset+1; //!< Use with kinematics to refer to left IR distance rangefinder reference frame
	const unsigned CenterIRFrameOffset = LeftIRFrameOffset+1; //!< Use with kinematics to refer to center IR distance rangefinder reference frame
	const unsigned IRFrameOffset = CenterIRFrameOffset; //!< alias for CenterIRFrameOffset
	const unsigned RightIRFrameOffset = CenterIRFrameOffset+1; //!< Use with kinematics to refer to right IR distance rangefinder reference frame

	//! the ordering of legs
	enum LegOrder_t {
		RFrLegOrder = 0, //!< right front leg
		LFrLegOrder,       //!< left front leg
		RMdLegOrder,     //!< right middle leg
		LMdLegOrder,     //!< left middle leg
		RBkLegOrder,      //!< right back leg
		LBkLegOrder     //!< left back leg
	};
	
	//! The offsets within appendages (the legs)  Note that the ordering matches the actual physical ordering of joints on the appendage
	enum SEKOffset_t {
		SweepOffset=0, //!< moves leg forward or backward along body
		ElevatorOffset,  //!< moves leg toward or away from body
		KneeOffset       //!< moves knee
	};
	
	//! The offsets of appendages with tilt (elevation), pan (heading), and roll or nod joints (i.e. head/wrist)
	enum TPROffset_t {
		PanOffset=0, //!< pan/yaw/heading (horizontal)
		TiltOffset,      //!< tilt/pitch/elevation (vertical)
		NodOffset = TiltOffset, //!< replicated tilt (could be left undefined instead...)
		RollOffset //!< spin/twist final axis (can be applied to WristOffset, but Chiara neck only has pan/tilt)
	};
	
	//! The offsets of the individual legs, add #REKOffset_t value to access specific joint
	/*! @hideinitializer */
	enum LegOffset_t {
		RFrLegOffset = LegOffset+RFrLegOrder*JointsPerLeg, //!< beginning of right front leg's joints
		LFrLegOffset = LegOffset+LFrLegOrder*JointsPerLeg, //!< beginning of left front leg's joints
		RMdLegOffset = LegOffset+RMdLegOrder*JointsPerLeg, //!< beginning of right front leg's joints
		LMdLegOffset = LegOffset+LMdLegOrder*JointsPerLeg, //!< beginning of left front leg's joints
		RBkLegOffset = LegOffset+RBkLegOrder*JointsPerLeg,  //!< beginning of right back leg's joints
		LBkLegOffset = LegOffset+LBkLegOrder*JointsPerLeg, //!< beginning of left back leg's joints
	};
	
	//! These are 'absolute' offsets for the arm joints, don't need to add to ArmOffset like TPROffset_t values do
	enum ArmOffset_t {
		ArmShoulderOffset=ArmOffset,
		ArmElbowOffset,
		WristOffset,
		WristYawOffset = WristOffset,
		WristPitchOffset,
		GripperLeftOffset,
		GripperRightOffset,
		GripperOffset
	};
	
	//! These are 'absolute' offsets for the neck joints, don't need to add to HeadOffset like TPROffset_t values do
	enum HeadOffset_t {
		HeadPanOffset = HeadOffset,      //!< pan/heading (horizontal)
		HeadTiltOffset, //!< tilt/elevation (vertical)
	};
	
	//! The offsets of the individual LEDs, one LED on each dynamixel servo, these match the servo order, so this is empty
	/*! @hideinitializer */
	enum LEDOffset_t {
		RFrKneeLEDOffset = LEDOffset, //!< Small LED on the knee servo
		LFrKneeLEDOffset, //!< Small LED on the knee servo
		RMdKneeLEDOffset, //!< Small LED on the knee servo
		LMdKneeLEDOffset, //!< Small LED on the knee servo
		RBkKneeLEDOffset, //!< Small LED on the knee servo
		LBkKneeLEDOffset, //!< Small LED on the knee servo
		HeadLEDOffset  //!< Small LED on the camera's pan servo
	};
	
	typedef unsigned int LEDBitMask_t; //!< So you can be clear when you're refering to a LED bitmask
	
	const LEDBitMask_t RFrKneeLEDMask = 1<<(RFrKneeLEDOffset-LEDOffset); //!< mask corresponding to RFrKneeLEDOffset
	const LEDBitMask_t LFrKneeLEDMask = 1<<(LFrKneeLEDOffset-LEDOffset); //!< mask corresponding to LFrKneeLEDOffset
	const LEDBitMask_t RMdKneeLEDMask = 1<<(RMdKneeLEDOffset-LEDOffset); //!< mask corresponding to RMdKneeLEDOffset
	const LEDBitMask_t LMdKneeLEDMask = 1<<(LMdKneeLEDOffset-LEDOffset); //!< mask corresponding to LMdKneeLEDOffset
	const LEDBitMask_t RBkKneeLEDMask = 1<<(RBkKneeLEDOffset-LEDOffset); //!< mask corresponding to RBkKneeLEDOffset
	const LEDBitMask_t LBkKneeLEDMask = 1<<(LBkKneeLEDOffset-LEDOffset); //!< mask corresponding to LBkKneeLEDOffset
	const LEDBitMask_t HeadLEDMask = 1<<(HeadLEDOffset-LEDOffset); //!< mask corresponding to HeadLEDOffset
	
	//! LEDs for the "face panel" -- on the Chiara this is just HeadLEDMask
	const LEDBitMask_t FaceLEDMask = HeadLEDMask;
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
	enum ButtonOffset_t { GreenButOffset, RedButOffset, YellowButOffset };

	//! Provides a string name for each button
	const char* const buttonNames[NumButtons+1] = { "GreenBut", "RedBut", "YellowBut", NULL };

	//! holds offset to different sensor values in WorldState::sensors[]
	/*! @see WorldState::sensors[] */
	enum SensorOffset_t {
		LeftIRDistOffset,
		CenterIRDistOffset,
		IRDistOffset = CenterIRDistOffset,
		RightIRDistOffset,
		LeftLuminosityOffset,
		CenterLuminosityOffset,
		RightLuminosityOffset,
		MicVolumeOffset,
		MicSpikeCountOffset,
		PowerRemainOffset, //! estimate of power capacity as percentage, 0-1
		PowerThermoOffset, //!< degrees Celcius
		PowerVoltageOffset, //!< volts
	};

	//! Provides a string name for each sensor
	const char* const sensorNames[NumSensors+1] = {
		"LeftIRDist", "CenterIRDist", "RightIRDist",
		"LeftLuminosity", "CenterLuminosity", "RightLuminosity",
		"MicVolume", "MicSpikeCount",
		"PowerRemain", "PowerThermo", "PowerVoltage", NULL
	};

	//@}
	
	
	//! Names for each of the outputs
	const char* const outputNames[NumReferenceFrames+1] = {
		// servos:
		"RFr:rotor",
		"RFr:sweep","RFr:elvtr","RFr:knee",
		"LFr:sweep","LFr:elvtr","LFr:knee",
		"RMd:sweep","RMd:elvtr","RMd:knee",
		"LMd:sweep","LMd:elvtr","LMd:knee",
		"RBk:sweep","RBk:elvtr","RBk:knee",
		"LBk:sweep","LBk:elvtr","LBk:knee",
		"ARM:shldr","ARM:elbow","ARM:wristYaw","ARM:wristPitch","ARM:gripperLeft","ARM:gripperRight",
		"NECK:pan","NECK:tilt",
		
		// note we don't expose ALL of the dynamixel LEDs... can't see most of them anyway,
		// so don't bother wasting storage/computation... these are the more visible ones:
		"LED:RFr:knee","LED:LFr:knee",
		"LED:RMd:knee","LED:LMd:knee",
		"LED:RBk:knee","LED:LBk:knee",
		"LED:NECK:pan",
		
		// reference frames:
		"BaseFrame",
		"RFrFootFrame",
		"LFrFootFrame",
		"RMdFootFrame",
		"LMdFootFrame",
		"RBkFootFrame",
		"LBkFootFrame",
		"GripperFrame",
		"CameraFrame",
		"LeftIRFrame",
		"CenterIRFrame",
		"RightIRFrame",
		NULL
	};
	
	//! provides polymorphic robot capability detection/mapping
	class ChiaraCapabilities : public Capabilities {
	public:
		//! constructor
		ChiaraCapabilities()
		: Capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs)
		{
			frameToIndex["ARM:Wrist"] = WristYawOffset;
			frameToIndex["NECK:nod"] = HeadTiltOffset;
			frameToIndex["IRFrame"] = IRFrameOffset; // aliased to the center IR sensor
			sensorToIndex["IRDist"]=IRDistOffset; // aliased to the center IR sensor
		}
	};
	//! allocation declared in RobotInfo.cc
	extern const ChiaraCapabilities capabilities;
	
	//! Dynamixel servos don't use PID control.  Instead, these values indicate compliance slope (P), punch (add to P*error), compliance margin (min error to start applying torque) (see ServoParam_t)
	/*! I believe the torque calculation goes something like: torque = (error<compliance) ? 0 : punch + P*error
	 *  Dynamixel servos allow different values to be supplied for CW vs. CCW motion, but we just use the same value for each */
	const float DefaultPIDs[NumPIDJoints][3] = {
		{32,32,0},
		{32,32,0}, {32,32,0}, {32,32,0}, 
		{32,32,0}, {32,32,0}, {32,32,0}, 
		{32,32,0}, {32,32,0}, {32,32,0}, 
		{32,32,0}, {32,32,0}, {32,32,0}, 
		{32,32,0}, {32,32,0}, {32,32,0}, 
		{32,32,0}, {32,32,0}, {32,32,0}, 
		{32,32,0}, {32,32,0}, {32,32,0}, {32,32,0}, {32,32,0}, {32,32,0}, 
		{32,32,0}, {32,32,0},
	};
	
	//!These values are our recommended maximum joint velocities, in rad/ms
	/*! a value <= 0 means infinite speed (e.g. LEDs)
	 *  
	 *  These limits are <b>not</b> enforced by the framework.  They are simply available for you to use as you see fit.
	 *  HeadPointerMC and PostureMC are primary examples of included classes which do respect these values (although they can be overridden)
	 *  
	 *  These values were obtained from the administrators of the Sony OPEN-R BBS */
	const float MaxOutputSpeed[NumOutputs] = {
		// servos
		0.4f,
		0.4f,0.4f,0.4f,
		0.4f,0.4f,0.4f,
		0.4f,0.4f,0.4f,
		0.4f,0.4f,0.4f,
		0.4f,0.4f,0.4f,
		0.4f,0.4f,0.4f,
		0.4f,0.4f,0.4f,0.4f,0.4f,0.4f,
		0.4f,0.4f,
		// leds
		0,0,
		0,0,
		0,0,
		0,
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
		{RAD(-17),RAD(90)},
		{RAD(-102),RAD(67)}, {RAD(-100),RAD(100)}, {RAD(-55),RAD(160)}, // right front
		{RAD(-53),RAD(58)}, {RAD(-100),RAD(100)}, {RAD(-55),RAD(160)}, // left front
		{RAD(-53),RAD(53)}, {RAD(-100),RAD(100)}, {RAD(-55),RAD(160)}, // right mid
		{RAD(-53),RAD(53)}, {RAD(-100),RAD(100)}, {RAD(-55),RAD(160)}, // left mid
		{RAD(-53),RAD(53)}, {RAD(-100),RAD(100)}, {RAD(-55),RAD(160)}, // right back
		{RAD(-73),RAD(53)}, {RAD(-100),RAD(100)}, {RAD(-55),RAD(160)}, // left back		
		{RAD(-95),RAD(95)}, {RAD(-95),RAD(95)}, {RAD(-100),RAD(100)}, {RAD(-90),RAD(90)}, {RAD(-90),RAD(90)}, {RAD(-90),RAD(90)},
		{RAD(-150),RAD(150)}, {RAD(-92),RAD(75)}, 
		
		// LED
		{0,1}, {0,1},
		{0,1}, {0,1},
		{0,1}, {0,1},
		{0,1}, 
	};

	//! This table holds the mechanical limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
	/*! Same as #outputRanges, don't know actual values because they were never specified by Sony */
	const float mechanicalLimits[NumOutputs][2] = {
		// servos
		{RAD(-17),RAD(90)},
		{RAD(-102),RAD(67)}, {RAD(-100),RAD(100)}, {RAD(-55),RAD(160)}, // right front
		{RAD(-53),RAD(58)}, {RAD(-100),RAD(100)}, {RAD(-55),RAD(160)}, // left front
		{RAD(-53),RAD(53)}, {RAD(-100),RAD(100)}, {RAD(-55),RAD(160)}, // right mid
		{RAD(-53),RAD(53)}, {RAD(-100),RAD(100)}, {RAD(-55),RAD(160)}, // left mid
		{RAD(-53),RAD(53)}, {RAD(-100),RAD(100)}, {RAD(-55),RAD(160)}, // right back
		{RAD(-73),RAD(53)}, {RAD(-100),RAD(100)}, {RAD(-55),RAD(160)}, // left back		
		{RAD(-95),RAD(95)}, {RAD(-95),RAD(95)}, {RAD(-100),RAD(100)}, {RAD(-90),RAD(90)}, {RAD(-90),RAD(90)}, {RAD(-90),RAD(90)},
		{RAD(-150),RAD(150)}, {RAD(-92),RAD(75)}, 		

		// LED
		{0,1}, {0,1},
		{0,1}, {0,1},
		{0,1}, {0,1},
		{0,1}, 
	};

#ifdef __RI_RAD_FLAG
#undef RAD
#undef __RI_RAD_FLAG
#endif
}

/*! @file
 * @brief Defines some capabilities of the Chiara hexapod robots
 * @author ejt (Creator)
 */

#endif
