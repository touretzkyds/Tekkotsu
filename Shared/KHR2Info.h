//-*-c++-*-
#ifndef INCLUDED_KHR2Info_h
#define INCLUDED_KHR2Info_h

#include <cmath>
#include <stdlib.h>
#include "CommonInfo.h"
using namespace RobotInfo;

// see http://tekkotsu.org/porting.html#configuration for more information on TGT_HAS_* flags
#if defined(TGT_KHR2)
#  define TGT_IS_BIPED
#  define TGT_IS_HUMANOID
#  define TGT_IS_KHR2
#  define TGT_HAS_CAMERA 1
#  define TGT_HAS_LEGS 2
#  define TGT_HAS_ARMS 2
#  define TGT_HAS_HEAD 1
#endif

//! Contains information about an KHR2 humanoid robot, such as number of joints, LEDs, etc.
namespace KHR2Info {

	// *******************************
	//       ROBOT CONFIGURATION
	// *******************************

	extern const char* const TargetName; //!< the name of the model, to be used for logging and remote GUIs

	const unsigned int FrameTime=32;       //!< time between frames in the motion system (milliseconds)
	const unsigned int NumFrames=1;        //!< the number of frames per buffer (don't forget also double buffered)
	const unsigned int SoundBufferTime=32; //!< the number of milliseconds per sound buffer... I'm not sure if this can be changed
	
	//!@name Output Types Information
	const unsigned NumWheels      =  0;
	
	const unsigned FingerJointsPerArm = 0;
	const unsigned JointsPerArm   =  3;
	const unsigned NumArms        =  2;
	const unsigned NumArmJoints   =  JointsPerArm*NumArms;//                                                 6
	
	const unsigned JointsPerLeg   =  5; //!< The number of joints per leg
	const unsigned NumLegs        =  2; //!< The number of legs
	const unsigned NumLegJoints   =  JointsPerLeg*NumLegs; //!< the TOTAL number of joints on ALL legs      10
	const unsigned NumHeadJoints  =  1; //!< The number of joints in the pantilt                             1
	const unsigned NumTailJoints  =  0; //!< The number of joints assigned to the tail
	const unsigned NumMouthJoints =  0; //!< the number of joints that control the mouth
	const unsigned NumEarJoints   =  0; //!< The number of joints which control the ears (NOT per ear, is total)
	const unsigned NumButtons     =  0; //!< the number of buttons that are available
	const unsigned SensorsPerFoot = 6;
        const unsigned NumIMUAxes     = 3;
        const unsigned NumFootSensors = SensorsPerFoot*NumLegs;
	const unsigned NumSensors     =  NumArms + NumLegs*6 + 3;  // 2 grippers, 12 foot pressure sensors, 3 axis imu
	const unsigned NumFacePanelLEDs = 0; //!< The number of face panel LEDs

	const unsigned NumLEDs        =  0;//NumLegs + 1 + 4; //!< There's an LED on every dynamixel, but can't see most of them, so save some computational resources and only expose the visible ones: knees and pan
	const unsigned NumPIDJoints   = NumArmJoints+NumLegJoints+NumHeadJoints; //!< servo pins             15
	
	const unsigned NumOutputs     = NumWheels + NumPIDJoints + NumLEDs; //!< the total number of outputs    17
	const unsigned NumReferenceFrames = NumOutputs + NumLegs + NumArms + 1 + 1; //!< for the feet (NumLegs), arms, base, and camera  23

	using namespace Camera75DOF;
	
	const float BallOfFootRadius=23.433f / 2; //!< radius of the ball of the foot
	//@}


	// *******************************
	//         OUTPUT OFFSETS
	// *******************************

	//!Corresponds to entries in ERS7Info::PrimitiveName, defined at the end of this file
	//!@name Output Offsets

	const unsigned PIDJointOffset = 0; //!< The beginning of the PID Joints
	//const unsigned RFrRotatorOffset = PIDJointOffset;
	const unsigned ArmOffset   = PIDJointOffset;          //!< 0 - the offset of the beginning of the arm joints
	const unsigned LegOffset   = ArmOffset+NumArmJoints;   //!< 6 - the offset of the beginning of the leg joints
	const unsigned HeadOffset  = LegOffset+NumLegJoints;  //!< 16 - the offset of the beginning of the head joint
	const unsigned LEDOffset   = PIDJointOffset + NumPIDJoints; //!< 17 - the offset of LEDs
	const unsigned BaseFrameOffset    = NumOutputs; //!< 17 - Use with kinematics to refer to base reference frame
	const unsigned CameraFrameOffset  = BaseFrameOffset+1; //!< 18  
	const unsigned GripperFrameOffset = CameraFrameOffset+1; //!< 19 
	const unsigned FootFrameOffset    = GripperFrameOffset+NumLegs; //!< 21 

	//! the ordering of arms
	enum ArmOrder_t
	{
		LeftArmOrder = 0,   //!< left arm
		RightArmOrder       //!< right arm
	};

	//! the ordering of legs
	enum LegOrder_t
	{
		LeftLegOrder = 0,   //!< left leg
		RightLegOrder       //!< right leg
	};

	enum RLEOffset_t
	{
		ArmShoulderRotateOffset = 0,
		ArmShoulderOffset,
		ArmElbowOffset
	};

	//tilt, pan, roll
	enum TPROffset_t
	{
		PanOffset = 0,
		TiltOffset = PanOffset
	};

	enum AHKALOffset_t
	{
		LegHipAbduceOffset = 0,
		LegHipOffset,
		LegKneeOffset,
		LegAnkleOffset,
		LegAnkleLateralOffset
	};

	enum ArmOffset_t
	{
		LeftArmOffset = ArmOffset+LeftArmOrder*JointsPerArm, // 0+0*3 = 0
		RightArmOffset = ArmOffset+RightArmOrder*JointsPerArm// 0+1*3 = 3
	};

	enum LegOffset_t
	{
		LeftLegOffset = LegOffset+LeftLegOrder*JointsPerLeg, //6+0*5 = 6
		RightLegOffset = LegOffset+RightLegOrder*JointsPerLeg//6+1*5 = 11
	};

	enum HeadOffset_t
	{
		HeadPanOffset = HeadOffset //16
	};

	enum LEDOffset_t {};

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
	enum ButtonOffset_t {};

	//! Provides a string name for each button
	const char* const buttonNames[NumButtons+1] = {NULL};

	/*! holds offset to different sensor values in WorldState::sensors[]
	 *  @see WorldState::sensors[] */
	enum SensorOffset_t
	{
		LStressOffset, RStressOffset,

		LLToeOffset, LRToeOffset, LLSoleOffset, LRSoleOffset, LLBackOffset, LRBackOffset,

                //! right foot pressure sensors
                RLToeOffset, RRToeOffset, RLSoleOffset, RRSoleOffset, RLBackOffset, RRBackOffset,

		//! ... be careful about the signs on all of these...

		/*! backward acceleration, in @f$m/s^2@f$
		 *  (negative if laying on back) */
		BAccelOffset,

		/*! acceleration to the robot's left, in @f$m/s^2@f$
		 *  (negative if lying on the robot's right side) */
		LAccelOffset,

		/*! download acceleration, in @f$m/s^2@f$
		 *  (negative if standing up) */
		DAccelOffset//,

		//PowerVoltageOffset   //!< power status, in volts
	};

	//! provides a string name for each sensor
	const char* const sensorNames[NumSensors + 1] = {
		"LStress", "RStress",

		//! left foot pressure sensors
		"LLToe", "LRToe", "LLSole", "LRSole", "LLBack", "LRBack",

		//! right foot pressure sensors
		"RLToe", "RRToe", "RLSole", "RRSole", "RLBack", "RRBack",

		"BAccel", "LAccel", "DAccel"//,

		//"PowerVoltage"
	};

	//! names for each of the outputs
	const char* const outputNames[NumReferenceFrames + 1] = {

		//! names for the left arm outputs
		"LArm:Rotate", "LArm:Elevator", "LArm:Elbow",

		//! names for the right arm outputs
		"RArm:Rotate", "RArm:Elevator", "RArm:Elbow",
		
		//! names for the left leg outputs
		"LLeg:Abduce", "LLeg:Flex", "LLeg:Knee",
		"LLeg:AnkleFlex",
		 "LLeg:AnkleLateral",

		//! names for the right left outputs
		"RLeg:Abduce", "RLeg:Flex", "RLeg:Knee",
		"RLeg:AnkleFlex",
		 "RLeg:AnkleLateral",

		//! names for each of the neck outputs
		"NECK:Pan",
		
		"BaseFrame",
		"CAMERA:Eye",// 18
		"LArm:Wrist",// 19
		"RArm:Wrist",
		"LLeg:Sole",// 21
		"RLeg:Sole",
		NULL
	};
	
	//! allocation declared in RobotInfo.cc
	extern const Capabilities capabilities;
	
	//! this table holds the default PID values for each joint; see PIDMC
	const float DefaultPIDs[NumPIDJoints][3] = {
		{1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {1, 0, 0}, // PID joints
		{1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {1, 0, 0},
		{1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {1, 0, 0},
		{1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {1, 0, 0}, 
		{1, 0, 0}
	};
	
	//!These values are our recommended maximum joint velocities, in rad/ms
	/*! a value <= 0 means infinite speed (e.g. LEDs)
	 *  
	 *  These limits are <b>not</b> enforced by the framework.  They are simply available for you to use as you see fit.
	 *  HeadPointerMC and PostureMC are primary examples of included classes which do respect these values (although they can be overridden)
	 *  
	 *  These values were obtained from the administrators of the Sony OPEN-R BBS */
	//! haven't determined safe speeds for KHR2 yet
	const float MaxOutputSpeed[NumOutputs] = {

		//! PID joints
		1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f,
		1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 
		1.f
	};

	#ifndef RAD
		//!Just a little macro for converting degrees to radians
	#define RAD(deg) (((deg) * (float)M_PI ) / 180.0f)
		//!a flag so we undef these after we're done - do you have a cleaner solution?
	#define __RI_RAD_FLAG
	#endif
	
	//! This table holds the software limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
	const float outputRanges[NumOutputs][2] = {

		//! left arm
		{RAD(0), RAD(180)}, {RAD(0), RAD(180)}, {RAD(-90), RAD(90)},

		//! right arm
		{RAD(0), RAD(180)}, {RAD(0), RAD(180)}, {RAD(-90), RAD(90)},

		//! left leg
		{RAD(-10), RAD(90)}, {RAD(-90), RAD(90)}, {RAD(-90), RAD(20)},
		{RAD(-90), RAD(90)},
		 {RAD(-70), RAD(20)},

		//! right leg
		{RAD(-10), RAD(90)}, {RAD(-90), RAD(90)}, {RAD(-90), RAD(20)},
		{RAD(-90), RAD(90)},
		 {RAD(-70), RAD(20)},

		//! head (neck)
		{RAD(-90), RAD(90)}
	};

	/*! the range that can be reached by external interaction
	 * (i.e. the domain of the output feedback);
	 *  this is probably identical to #outputRanges,
	 *  but may differ if the output have some built-in safety margin... */
	const float mechanicalLimits[NumOutputs][2] = {

		//! left arm
    		{RAD(0), RAD(180)}, {RAD(0), RAD(180)}, {RAD(-90), RAD(90)},

		//! right arm
		{RAD(0), RAD(180)}, {RAD(0), RAD(180)}, {RAD(-90), RAD(90)},

		//! left leg
		{RAD(-10), RAD(90)}, {RAD(-90), RAD(90)}, {RAD(-90), RAD(20)},
		{RAD(-90), RAD(90)},
		 {RAD(-70), RAD(20)},

		//! right leg
		{RAD(-10), RAD(90)}, {RAD(-90), RAD(90)}, {RAD(-90), RAD(20)},
		{RAD(-90), RAD(90)},
		 {RAD(-70), RAD(20)},

		//! head (neck)
		{RAD(-90), RAD(90)}
	};

#ifdef __RI_RAD_FLAG
#undef RAD
#undef __RI_RAD_FLAG
#endif
}

/*! @file
 * @brief Defines some capabilities of the KHR2 humanoid robots
 * $Author: dst $
 * $Name:  $
 * $Revision: 1.5 $
 * $State: Exp $
 * $Date: 2013/05/02 03:56:40 $
 */

#endif
