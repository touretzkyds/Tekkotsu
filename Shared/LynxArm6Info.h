//-*-c++-*-
#ifndef INCLUDED_LynxArm6Info_h
#define INCLUDED_LynxArm6Info_h

#include <cmath>
#include <cstdlib>
#include "CommonInfo.h"
using namespace RobotInfo;

// see http://tekkotsu.org/porting.html#configuration for more information on TGT_HAS_* flags
#if defined(TGT_LYNXARM6)
#  define TGT_HAS_ARMS 1
#endif

//! Declares configuration of the 6-DOF PUMA style arm from Lynx Motion, such as number of joints, LEDs, etc.
namespace LynxArm6Info {

	// *******************************
	//       ROBOT CONFIGURATION
	// *******************************

	extern const char* const TargetName; //!< the name of the model, to be used for logging and remote GUIs

	const unsigned int FrameTime=32;        //!< time between frames in the motion system (milliseconds)
	const unsigned int NumFrames=1;        //!< the number of frames per buffer (don't forget also double buffered)
	const unsigned int SoundBufferTime=32; //!< the number of milliseconds per sound buffer... I'm not sure if this can be changed
	
	//!@name Output Types Information
	const unsigned NumWheels = 0; //!< no wheels, just legs
	
	const unsigned JointsPerArm   =  6;
	const unsigned NumArms        =  1;
	const unsigned NumArmJoints   =  JointsPerArm*NumArms;
	
	const unsigned JointsPerLeg   =  0; //!< The number of joints per leg
	const unsigned NumLegs        =  0; //!< The number of legs
	const unsigned NumLegJoints   =  JointsPerLeg*NumLegs; //!< the TOTAL number of joints on ALL legs
	const unsigned NumHeadJoints  =  0; //!< The number of joints in the neck
	const unsigned NumTailJoints  =  0; //!< The number of joints assigned to the tail
	const unsigned NumMouthJoints =  0; //!< the number of joints that control the mouth
	const unsigned NumEarJoints   =  0; //!< The number of joints which control the ears (NOT per ear, is total)
	const unsigned NumButtons     =  0; //!< the number of buttons that are available, 2 head, 4 paws, 3 back, 1 underbelly see ERS7Info::ButtonOffset_t
	const unsigned NumSensors     =  4;  //!< the four input pins
	const unsigned NumLEDs        =  0; //!< The number of LEDs which can be controlled
	const unsigned NumFacePanelLEDs = 0; //!< The number of face panel LEDs
	
	const unsigned NumPIDJoints   = NumArmJoints + NumLegJoints+NumHeadJoints+NumTailJoints+NumMouthJoints; //!< The number of joints which use PID motion - everything except ears
	const unsigned NumOutputs     = NumPIDJoints + NumLEDs; //!< the total number of outputs
	const unsigned NumReferenceFrames = NumOutputs + 1 + NumArms; //!< for the base, gripper (* NumArms)

	// webcam ?
	const float CameraHorizFOV=56.9f/180*(float)M_PI; //!< horizontal field of view (radians)
	const float CameraVertFOV=45.2f/180*(float)M_PI; //!< vertical field of view (radians)
	const float CameraFOV=CameraHorizFOV; //!< should be set to maximum of #CameraHorizFOV or #CameraVertFOV
	const unsigned int CameraResolutionX=320; //!< the number of pixels available in the 'full' layer
	const unsigned int CameraResolutionY=240; //!< the number of pixels available in the 'full' layer
	//@}


	// *******************************
	//         OUTPUT OFFSETS
	// *******************************

	//!Corresponds to entries in ERS7Info::PrimitiveName, defined at the end of this file
	//!@name Output Offsets

	const unsigned PIDJointOffset = 0; //!< The beginning of the PID Joints
	const unsigned ArmOffset   = PIDJointOffset;

	//! These are 'absolute' offsets for the arm joints, don't need to add to ArmOffset like TPROffset_t values do
	enum ArmOffset_t {
		ArmShoulderOffset=ArmOffset,
		ArmElevatorOffset,
		ArmElbowOffset,
		WristOffset,
		WristRollOffset = WristOffset,
		WristPitchOffset,
		GripperOffset
	};
	
	const unsigned BaseFrameOffset   = NumOutputs; //!< Use with kinematics to refer to base reference frame
	const unsigned GripperFrameOffset    = BaseFrameOffset+1; //!< Use with kinematics to refer to paw reference frames

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
	const char* const buttonNames[NumButtons+1] = { NULL }; // plus one and NULL entry to workaround gcc 3.4.2 internal error

	//! holds offset to different sensor values in WorldState::sensors[]
	/*! @see WorldState::sensors[] */
	enum SensorOffset_t { SensorAOffset, SensorBOffset, SensorCOffset, SensorDOffset };

	//! Provides a string name for each sensor
	const char* const sensorNames[NumSensors] = { "SensorA", "SensorB", "SensorC", "SensorD" };

	//@}


	//! Names for each of the outputs
	const char* const outputNames[NumReferenceFrames+1] = {
		"Shldr:rot","Shldr:elv","Elbow",
		"Wrist:elv","Wrist:rot","Gripper",
		
		"BaseFrame", "GripperFrame",
		NULL
	};
	
	//! allocation declared in RobotInfo.cc
	extern const Capabilities capabilities;
	
	//! This table holds the default PID values for each joint.  see PIDMC
	const float DefaultPIDs[NumPIDJoints][3] = {
		{1,0,0}, {1,0,0}, {1,0,0},
		{1,0,0}, {1,0,0}, {1,0,0},
	};
		
	//!These values are Sony's recommended maximum joint velocities, in rad/sec
	/*! a value <= 0 means infinite speed (e.g. LEDs)
	 *  
	 *  These limits are <b>not</b> enforced by the framework.  They are simply available for you to use as you see fit.
	 *  HeadPointerMC and PostureMC are primary examples of included classes which do respect these values (although they can be overridden)
	 *  
	 *  These values were obtained from the administrators of the Sony OPEN-R BBS */
	const float MaxOutputSpeed[NumOutputs] = {
		4.86510529f, //Legs LR,FB,REK
		5.27962099f,
		5.27962099f,
		4.86510529f,
		5.27962099f,
		5.27962099f
	};

	#ifndef RAD
		//!Just a little macro for converting degrees to radians
	#define RAD(deg) (((deg) * (float)M_PI ) / 180.0f)
		//!a flag so we undef these after we're done - do you have a cleaner solution?
	#define __RI_RAD_FLAG
	#endif
	
	//! This table holds the software limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
	const float outputRanges[NumOutputs][2] =
		{
			{ RAD(-90),RAD(90) },{ RAD(-90),RAD(90) },{ RAD(-90),RAD(90) },
			{ RAD(-90),RAD(90) },{ RAD(-90),RAD(90) },{ RAD(-90),RAD(90) },
		};

	//! This table holds the mechanical limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
	/*! Same as #outputRanges */
	const float mechanicalLimits[NumOutputs][2] =
		{
			{ RAD(-90),RAD(90) },{ RAD(-90),RAD(90) },{ RAD(-90),RAD(90) },
			{ RAD(-90),RAD(90) },{ RAD(-90),RAD(90) },{ RAD(-90),RAD(90) },
		};

#ifdef __RI_RAD_FLAG
#undef RAD
#undef __RI_RAD_FLAG
#endif
}

/*! @file
 * @brief Defines some capabilities of the 6-DOF PUMA style arm from Lynx Motion
 * @author ejt (Creator)
 */

#endif
