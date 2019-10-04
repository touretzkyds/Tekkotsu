//-*-c++-*-
#ifndef INCLUDED_CameraBotInfo_h
#define INCLUDED_CameraBotInfo_h

#include <cmath>
#include <stdlib.h>
#include "CommonInfo.h"
using namespace RobotInfo;

// see http://tekkotsu.org/porting.html#configuration for more information on TGT_HAS_* flags
#if defined(TGT_CAMERABOT)
#  define TGT_IS_CAMERABOT
#  define TGT_HAS_CAMERA 1
//#  define TGT_HAS_LEDS 0
//#  define TGT_HAS_ARMS 0
//#  define TGT_HAS_HEAD 0
#  define WALKMC_NO_WARN_NOOP
#endif

//! Declares configuration of the HandEye planar arm robot, such as number of joints, LEDs, etc.
namespace CameraBotInfo {

  using namespace CameraLifeCam;

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
	const unsigned NumTailJoints  =  1; //!< **** HACK **** The number of joints assigned to the tail
	const unsigned NumMouthJoints =  0; //!< the number of joints that control the mouth
	const unsigned NumEarJoints   =  0; //!< The number of joints which control the ears (NOT per ear, is total)
	const unsigned NumButtons     =  3; //!< the number of buttons that are available
	const unsigned NumSensors     =  0;  //!< the number of sensors available
	const unsigned NumFacePanelLEDs = 0; //!< The number of face panel LEDs

	// UPGRADE: NumLEDs to 7
	const unsigned NumLEDs        =  0; //!< The number of LEDs which can be controlled (one per dynamixel servo)
	const unsigned NumPIDJoints   = NumWheels + NumArmJoints + NumLegJoints + NumHeadJoints + NumTailJoints + NumMouthJoints;; //!< servo pins
	
	const unsigned NumOutputs     = NumWheels + NumPIDJoints + NumLEDs; //!< the total number of outputs
	const unsigned NumReferenceFrames = NumOutputs + 1 + NumArms + 1; //!< for the base, gripper (* NumArms), and camera reference frames


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



	typedef unsigned int LEDBitMask_t; //!< So you can be clear when you're refering to a LED bitmask
  enum LEDOffset_t { };
  const LEDBitMask_t AllLEDMask  = (LEDBitMask_t)~0;


	//! Offset needed so that the centroid of the robot is correct relative to the bounding box
	const fmat::Column<3> AgentBoundingBoxBaseFrameOffset = fmat::pack(0,0,0); 

	//! Half of the length, width, and height of the robot
	const fmat::Column<3> AgentBoundingBoxHalfDims = fmat::pack(75/2, 50/2, 24*25.4/2);

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

  enum SensorOffset_t {};

	//! Provides a string name for each button
	const char* const buttonNames[NumButtons+1] = { "GreenBut", "RedBut", "YellowBut", NULL };

	//! Provides a string name for each sensor
	const char* const sensorNames[1] = {
   NULL
	};
	
	//@}


	//! Names for each of the outputs
	const char* const outputNames[NumReferenceFrames+1] = {
    "Tail",
		"BaseFrame",
		"CameraFrame",
		NULL
	};
	
	//! provides polymorphic robot capability detection/mapping
	class CameraBotCapabilities : public Capabilities {
	public:
		//! constructor
		CameraBotCapabilities()
		: Capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs)
		{	}
	};
	//! allocation declared in RobotInfo.cc
	extern const CameraBotCapabilities capabilities;
	
	//! Dynamixel servos don't use PID control.  Instead, these values indicate compliance slope (P), punch (add to P*error), compliance margin (min error to start applying torque) (see ServoParam_t)
	/*! I believe the torque calculation goes something like: torque = (error<compliance) ? 0 : punch + P*error
	 *  Dynamixel servos allow different values to be supplied for CW vs. CCW motion, but we just use the same value for each */
	const float DefaultPIDs[NumPIDJoints][3] = {};

	//!These values are our recommended maximum joint velocities, in rad/ms
	/*! a value <= 0 means infinite speed (e.g. LEDs)
	 *  
	 *  These limits are <b>not</b> enforced by the framework.  They are simply available for you to use as you see fit.
	 *  HeadPointerMC and PostureMC are primary examples of included classes which do respect these values (although they can be overridden)
	 */
	const float MaxOutputSpeed[NumOutputs] = {};

	
	//! This table holds the software limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
	const float outputRanges[NumOutputs][2] = {};

	//! This table holds the mechanical limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
	const float mechanicalLimits[NumOutputs][2] = {};
}

/*! @file
 * @brief Defines some capabilities of the CameraBot
 * @author ejt (Creator)
 */

#endif
