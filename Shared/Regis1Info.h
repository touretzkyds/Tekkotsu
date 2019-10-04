//-*-c++-*-
#ifndef INCLUDED_Regis1Info_h
#define INCLUDED_Regis1Info_h

#include <stdlib.h>
#include <cmath>
#include "CommonInfo.h"
using namespace RobotInfo;

// see http://tekkotsu.org/porting.html#configuration for more information on TGT_HAS_* flags
#if defined(TGT_REGIS1)
#  define TGT_HAS_WHEELS 2
#  define TGT_HAS_ARMS 1
#  define TGT_HAS_CAMERA 1
#  define TGT_HAS_HEAD 1
#endif

//! Declares configuration of the Regis robot prototype, such as number of joints, LEDs, etc.
namespace Regis1Info	{

	// *******************************
	//       ROBOT CONFIGURATION
	// *******************************

	extern const char* const TargetName; //!< the name of the model, to be used for logging and remote GUIs

	const unsigned int FrameTime=15;        //!< time between frames in the motion system (milliseconds)
	const unsigned int NumFrames=1;        //!< the number of frames per buffer (don't forget also double buffered)
	const unsigned int SoundBufferTime=32; //!< the number of milliseconds per sound buffer... I'm not sure if this can be changed
	
	//!@name Output Types Information
	const unsigned NumWheels      =  2; //!<Left and Right Wheels
	
	const unsigned JointsPerArm   =  6;//!< Base, Shoulder, Elbow, Wrist (Yaw, Roll), Gripper
	const unsigned NumArms        =  1;
	const unsigned NumArmJoints   =  JointsPerArm*NumArms;
	
	const unsigned JointsPerLeg   =  0; 
	const unsigned NumLegs        =  0; 
	const unsigned NumLegJoints   =  JointsPerLeg*NumLegs;
	const unsigned NumHeadJoints  =  4;   // Goose neck
	const unsigned NumTailJoints  =  0; 
	const unsigned NumMouthJoints =  0; 
	const unsigned NumEarJoints   =  0; 
	const unsigned NumButtons     =  0; 
	const unsigned NumSensors     =  0;  
	const unsigned NumLEDs        =  0; 
	const unsigned NumFacePanelLEDs = 0; 
	
	const unsigned NumPIDJoints   = NumWheels+NumArmJoints+NumLegJoints+NumHeadJoints+NumTailJoints+NumMouthJoints; //!< The number of joints which use PID motion - everything except ears
	const unsigned NumBinJoints   = NumEarJoints; //!< The number of binary joints - just the ears
	const unsigned NumOutputs     = NumPIDJoints + NumBinJoints + NumLEDs; //!< the total number of outputs
	const unsigned NumReferenceFrames = NumOutputs + 1 + NumArms + 1; //!< for the base, gripper (* NumArms), and camera reference frames

	using namespace CameraSTX;
	//@}


	// *******************************
	//         OUTPUT OFFSETS
	// *******************************

	//!Corresponds to entries in ERS7Info::PrimitiveName, defined at the end of this file
	//!@name Output Offsets

	const unsigned PIDJointOffset = 0; //!< The beginning of the PID Joints
	const unsigned WheelOffset = PIDJointOffset; //!< beginning of wheel velocities, don't need to add anything, just use WheelOffset_t entries directly
	const unsigned ArmOffset   = WheelOffset+NumWheels;  //!< beginning of arm joints, don't add anything, just use ArmOffset_t entries directly
	const unsigned HeadOffset  = ArmOffset+NumArmJoints;   //!< the offset of the beginning of the head joints, add TPROffset_t to get specific joint

	const unsigned LEDOffset   = PIDJointOffset + NumPIDJoints; //!< the offset of LEDs in WorldState::outputs and MotionCommand functions, see LedOffset_t for specific offsets
	const unsigned BinJointOffset = LEDOffset + NumLEDs;   //!< The beginning of the binary joints
	const unsigned EarOffset   = BinJointOffset;           //!< the offset of the beginning of the ear joints - note that ears aren't sensed.  They can be flicked by the environment and you won't know.

	const unsigned BaseFrameOffset   = NumOutputs; //!< Use with kinematics to refer to base reference frame
	const unsigned GripperFrameOffset    = BaseFrameOffset+1; //!< Use with kinematics to refer to paw reference frames (add appropriate LegOrder_t to specify which paw)
	const unsigned CameraFrameOffset = GripperFrameOffset+NumArms; //!< Use with kinematics to refer to camera reference frame

	enum WheelOffset_t {
		LWheelOffset=WheelOffset,
		RWheelOffset
	};
	
	//! these are all 'absolute' offsets -- no need to add to HeadOffset
	enum HeadOffset_t {
		HeadBaseOffset = HeadOffset,
		HeadPanOffset = HeadBaseOffset, // alias HeadPanOffset
		HeadShoulderOffset,
		HeadElbowOffset,
		HeadTiltOffset = HeadElbowOffset, // alias HeadTiltOffset
		HeadWristOffset,
		HeadNodOffset = HeadWristOffset // alias HeadNodOffset
	};
	
	//! these are the traditional sub-offsets, relative to HeadOffset
	enum TPROffset_t {
		PanOffset = HeadBaseOffset - HeadOffset, //!< pan/heading (horizontal)
		TiltOffset = HeadElbowOffset - HeadOffset, //!< tilt/elevation (vertical)
		NodOffset = HeadWristOffset - HeadOffset //!< replicated tilt (could be left undefined instead...)
	};
	
	enum ArmOffset_t {
		ArmBaseOffset = ArmOffset,
		ArmShoulderOffset,
		ArmElbowOffset,
		ArmWristOffset,
		ArmWristRotateOffset,
		ArmGripperOffset
	};

	enum ArmJoint{
		ArmBase,
		ArmShoulder,
		ArmElbow,
		ArmWrist,
		ArmWristRotate,
		ArmGripper
	};

	enum ButtonOffset_t { };
	
	const char* const buttonNames[NumButtons + 1] = { NULL };
	
	enum SensorOffset_t { };
	
	const char* const sensorNames[NumSensors + 1] = { NULL };
	
	//! Names for each of the outputs
	const char* const outputNames[NumReferenceFrames+1] = {
		"WHEEL:L",
		"WHEEL:R",
		"ARM:base","ARM:shldr","ARM:elbow","ARM:wrist","ARM:wristRot","ARM:gripper",
		"GNk:Pan","GNk:Shldr","GNk:Elbow","GNk:Pitch",
		"BaseFrame","GripperFrame","CameraFrame",
		NULL
	};

	//! provides polymorphic robot capability detection/mapping
	class Regis1Capabilities : public Capabilities {
	public:
		//! constructor
		Regis1Capabilities()
		: Capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs)
		{
			// alias neck joints
			frameToIndex["NECK:tilt"]=HeadTiltOffset;
			frameToIndex["NECK:pan"]=HeadPanOffset;
			frameToIndex["NECK:nod"]=HeadNodOffset;
		}
	};
	//! allocation declared in RobotInfo.cc
	extern const Regis1Capabilities capabilities;
	
	const float DefaultPIDs[NumPIDJoints][3] = {
		{1,0,0},
		{1,0,0},
		{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},
		{1,0,0},{1,0,0},{1,0,0},{1,0,0}
	};
	
	//! haven't determined safe speeds for regis yet
	const float MaxOutputSpeed[NumOutputs] = {
		1.f,
		1.f,
		1.f, 1.f, 1.f, 1.f, 1.f, 1.f,
		1.f, 1.f, 1.f, 1.f
	};
	
	#ifndef RAD
		//!Just a little macro for converting degrees to radians
	#define RAD(deg) (((deg) * (float)M_PI ) / 180.0f)
		//!a flag so we undef these after we're done - do you have a cleaner solution?
	#define __RI_RAD_FLAG
	#endif
	
	//! the range that can be reached by the joint controller (i.e. the domain of the output commands)
	const float outputRanges[NumOutputs][2] = {
			{ -1 , 1 },//Wheels
			{ -1 , 1 },
			{ RAD(-90),RAD(90) },{ RAD(-94.45f),RAD(94.45f) },{ RAD(-94.45f),RAD(94.45f) },{ RAD(-94.45f),RAD(94.45f) },{ RAD(-90),RAD(90) },{ RAD(-90),RAD(90) }, //Arm
			{ RAD(-100),RAD(100) },{ RAD(-190),RAD(10) },{ RAD(-60),RAD(140) },{ RAD(-50),RAD(150) }//Goose Neck
	};
	
	//! the range that can be reached by external interaction (i.e. the domain of the output feedback)
	/*! This is probably identical to #outputRanges, but may differ if the outputs have some built-in safety margin... */
	const float mechanicalLimits[NumOutputs][2] = {
			{ -1 , 1 },//Wheels
			{ -1 , 1 },
			{ RAD(-90),RAD(90) },{ RAD(-90),RAD(90) },{ RAD(-90),RAD(90) },{ RAD(-90),RAD(90) },{ RAD(-90),RAD(90) },{ RAD(-90),RAD(90) }, //Arm
			{ RAD(-100),RAD(100) },{ RAD(-10),RAD(190) },{ RAD(-140),RAD(60) },{ RAD(-150),RAD(50) }//Goose Neck
	};

		
#ifdef __RI_RAD_FLAG
#undef RAD
#undef __RI_RAD_FLAG
#endif
}

/*! @file
 * @brief Defines some capabilities of the Regis robot prototype
 * @author ejt (Creator)
 */

#endif
