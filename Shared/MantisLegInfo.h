//-*-c++-*-
#ifndef INCLUDED_MantisLegInfo_h
#define INCLUDED_MantisLegInfo_h

#include <cmath>
#include <stdlib.h>
#include "CommonInfo.h"
using namespace RobotInfo;

// see http://tekkotsu.org/porting.html#configuration for more information on TGT_HAS_* flags
#if defined(TGT_MANTISLEG)
#  define TGT_IS_MANTISLEG
#  define TGT_HAS_LEGS 1
#  define TGT_HAS_LEDS 4
#  define TGT_HAS_POWER_STATUS
#endif

//! Contains information about an Mantis Leg robot, such as number of joints, LEDs, etc.
namespace MantisLegInfo {

	// *******************************
	//       ROBOT CONFIGURATION
	// *******************************

	extern const char* const TargetName; //!< the name of the model, to be used for logging and remote GUIs

	const unsigned int FrameTime	= 32;    //!< time between frames in the motion system (milliseconds)
	const unsigned int NumFrames	= 1;     //!< the number of frames per buffer (don't forget also double buffered)
	const unsigned int SoundBufferTime = 32; //!< the number of milliseconds per sound buffer... I'm not sure if this can be changed
	
	//!@name Output Types Information
	const unsigned NumWheels      =  0;
	
	const unsigned JointsPerArm   =  0;
	const unsigned NumArms        =  0;
	const unsigned NumArmJoints   =  JointsPerArm*NumArms;
	
	const unsigned JointsPerLeg   =  4; //!< The number of joints per leg
	const unsigned NumLegs        =  1; //!< The number of legs
	const unsigned NumLegJoints   =  JointsPerLeg*NumLegs; //!< the TOTAL number of joints on ALL legs

	const unsigned NumHeadJoints  =  0; //!< The number of joints in the pantilt
	const unsigned NumTailJoints  =  0; //!< The number of joints assigned to the tail
	const unsigned NumMouthJoints =  0; //!< the number of joints that control the mouth
	const unsigned NumEarJoints   =  0; //!< The number of joints which control the ears (NOT per ear, is total)
	const unsigned NumButtons     =  3; //!< the number of buttons that are available
	const unsigned NumSensors     =  11; //!< the number of sensors available
	const unsigned NumFacePanelLEDs = 0; //!< The number of face panel LEDs

	const unsigned NumPIDJoints   = NumArmJoints + NumLegJoints + NumHeadJoints + NumTailJoints + NumMouthJoints; //!< servo pins
	const unsigned NumLEDs        = NumPIDJoints; //!< There's an LED on every dynamixel, but can't see most of them, so save some computational resources and only expose the visible ones
	
	const unsigned NumOutputs     = NumWheels + NumPIDJoints + NumLEDs; //!< the total number of outputs
	const unsigned NumReferenceFrames = NumOutputs + NumLegs + NumArms + 1; //!< for the base, feet, gripper, camera, and IR distance rangefinder

	using namespace Camera75DOF;
	
	const float BallOfFootRadius = 0; //!< radius of the ball of the foot


	// *******************************
	//         OUTPUT OFFSETS
	// *******************************

	//!Corresponds to entries in ERS7Info::PrimitiveName, defined at the end of this file
	//!@name Output Offsets

	const unsigned PIDJointOffset	= 0; //!< The beginning of the PID Joints
	const unsigned LegOffset		= PIDJointOffset; //!< the offset of the beginning of the regular leg joints (after the 1 rotator joint for the right front leg):  #NumLegs of #JointsPerLeg each, in #LegOrder_t order; see #LegOffset_t
	const unsigned LEDOffset		= PIDJointOffset + NumPIDJoints; //!< the offset of LEDs in WorldState::outputs and MotionCommand functions, see LedOffset_t for specific offsets
	const unsigned BaseFrameOffset	= NumOutputs;		//!< Use with kinematics to refer to base reference frame
	const unsigned FootFrameOffset	= BaseFrameOffset+1;	//!< Use with kinematics to refer to feet reference frames (add appropriate LegOrder_t to specify which paw)
    const unsigned GripperFrameOffset = FootFrameOffset+NumLegs; //!< Use with kinematics to refer to gripper reference frame
	const unsigned CameraFrameOffset = GripperFrameOffset+NumArms; //!< Use with kinematics to refer to camera reference frame
    
    const unsigned LeftIRFrameOffset = CameraFrameOffset+1; //!< Use with kinematics to refer to left IR distance rangefinder reference frame
	const unsigned CenterIRFrameOffset = LeftIRFrameOffset+1; //!< Use with kinematics to refer to center IR distance rangefinder reference frame
	const unsigned IRFrameOffset = CenterIRFrameOffset; //!< alias for CenterIRFrameOffset
	const unsigned RightIRFrameOffset = CenterIRFrameOffset+1; //!< Use with kinematics to refer to right IR distance rangefinder reference frame

	
	//! the ordering of legs
	enum LegOrder_t {
		MdLegOrder = 0,	//!< middle leg
	};
	
	//! The offsets within appendages (the legs)  Note that the ordering matches the actual physical ordering of joints on the appendage
	enum SEKOffset_t {
		SweepOffset = 0,	//!< moves leg forward or backward along body
		ElevatorOffset,		//!< moves leg toward or away from body
		RotorOffset,		//!< rotates leg
		KneeOffset			//!< moves knee
	};
	
	//! The offsets of appendages with tilt (elevation), pan (heading), and roll or nod joints (i.e. head/wrist)
	enum TPROffset_t {
		PanOffset = 0,		//!< pan/yaw/heading (horizontal)
		TiltOffset,			//!< tilt/pitch/elevation (vertical)
		NodOffset = TiltOffset,	//!< replicated tilt (could be left undefined instead...)
		RollOffset			//!< spin/twist final axis
	};
	
	//! The offsets of the individual legs, add #REKOffset_t value to access specific joint
	/*! @hideinitializer */
	enum LegOffset_t {
		MdLegOffset = LegOffset+MdLegOrder*JointsPerLeg, //!< beginning of right front leg's joints
	};
    
    //! The offsets of the individual LEDs
	/*! @hideinitializer */
	enum LEDOffset_t {
		PowerRedLEDOffset=LEDOffset,
		PowerGreenLEDOffset,
		PlayLEDOffset,
		AdvanceLEDOffset
	};
    
    
    const LEDOffset_t RedLEDOffset = PowerRedLEDOffset;
	const LEDOffset_t BlueLEDOffset = AdvanceLEDOffset; 
	const LEDOffset_t GreenLEDOffset = PlayLEDOffset;
	const LEDOffset_t YellowLEDOffset = AdvanceLEDOffset;
    
	typedef unsigned int LEDBitMask_t; //!< So you can be clear when you're refering to a LED bitmask
    
	const LEDBitMask_t BlueLEDMask = (1<<(AdvanceLEDOffset-LEDOffset)) | (1<<(PowerRedLEDOffset-LEDOffset));
	const LEDBitMask_t GreenLEDMask = 1<<(GreenLEDOffset-LEDOffset); //!< mask corresponding to GreenLEDOffset
	const LEDBitMask_t YellowLEDMask = 1<<(YellowLEDOffset-LEDOffset); //!< mask corresponding to YellowLEDOffset
	const LEDBitMask_t RedLEDMask = 1<<(RedLEDOffset-LEDOffset); //!< mask corresponding to RedLEDOffset
    
	const LEDBitMask_t PowerRedLEDMask = 1<<(PowerRedLEDOffset-LEDOffset); //!< mask corresponding to BlueLEDOffset
	const LEDBitMask_t PowerGreenLEDMask = 1<<(PowerGreenLEDOffset-LEDOffset); //!< mask corresponding to GreenLEDOffset
	const LEDBitMask_t PlayLEDMask = 1<<(PlayLEDOffset-LEDOffset); //!< mask corresponding to YellowLEDOffset
	const LEDBitMask_t AdvanceLEDMask = 1<<(AdvanceLEDOffset-LEDOffset); //!< mask corresponding to RedLEDOffset
    
	//! LEDs for the face panel (all FaceLEDPanelMask<<(0:NumFacePanelLEDs-1) entries)
	const LEDBitMask_t FaceLEDMask = 0;
	//! selects all of the leds
	const LEDBitMask_t AllLEDMask  = (LEDBitMask_t)~0;
	//@}
    
    
    //! Offset needed so that the centroid of the robot is correct related to the bounding box
	const fmat::Column<3> AgentBoundingBoxBaseFrameOffset = fmat::pack(400,70,350);
    
	//! Half of the length, width, and height of the robot.
	const fmat::Column<3> AgentBoundingBoxHalfDims = fmat::pack(100,100,100);
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
		"sweep","elevator","rotor","knee",
		
        // LEDs
		"LED:sweep","LED:elevator",
		"LED:rotor","LED:knee",
        
        // Reference frames
        "BaseFrame", "FootFrame",
		  
		NULL
	};
	
	//! provides polymorphic robot capability detection/mapping
	class MantisLegCapabilities : public Capabilities {
	public:
		//! constructor
		MantisLegCapabilities()
		: Capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs)
		{
			frameToIndex["IRFrame"]	= IRFrameOffset;	// aliased to the center IR sensor
			sensorToIndex["IRDist"]	= IRDistOffset;		// aliased to the center IR sensor
		}
	};
    
	//! allocation declared in RobotInfo.cc
	extern const MantisLegCapabilities capabilities;
	
	
	//! offsets into DefaultPIDs, since Dynamixel servos don't actually use PID control, but a different set of parameters
	enum ServoParam_t {
		DYNAMIXEL_SLOPE = 0,	//!< compliance slope, the proportional control (P in PID)
		DYNAMIXEL_PUNCH,		//!< punch, a constant added to the slope once error exceeds compliance margin
		DYNAMIXEL_MARGIN		//!< compliance margin, the amount of error to tolerate before triggering a torque response
	};
	
	//! Dynamixel servos don't use PID control.  Instead, these values indicate compliance slope (P), punch (add to P*error), compliance margin (min error to start applying torque) (see ServoParam_t)
	/*! I believe the torque calculation goes something like: torque = (error<compliance) ? 0 : punch + P*error
	 *  Dynamixel servos allow different values to be supplied for CW vs. CCW motion, but we just use the same value for each */
	const float DefaultPIDs[NumPIDJoints][3] = {
		{32,32,0},{32,32,0},
        {32,32,0},{32,32,0}
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
	  0.4f, 0.4f, 0.4f, 0.4f, 
	  // leds
	  0,0,0,0 
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
		{RAD(-100),RAD(100)}, 
        {RAD(-100),RAD(100)},
		{RAD(-100),RAD(100)},
        {RAD(-100),RAD(100)},		
		
        // LED
		{0,1},{0,1},
        {0,1},{0,1},
	};

	//! This table holds the mechanical limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
	/*! Same as #outputRanges, don't know actual values because they were never specified by Sony */
	const float mechanicalLimits[NumOutputs][2] = {
		// servos
		{RAD(-100),RAD(100)}, 
        {RAD(-100),RAD(100)},
		{RAD(-100),RAD(100)},
        {RAD(-100),RAD(100)},
		// LED
		{0,1},{0,1},
        {0,1},{0,1},
	};

#ifdef __RI_RAD_FLAG
#undef RAD
#undef __RI_RAD_FLAG
#endif
}

/*! @file
 * @brief Defines some capabilities of the Mantis leg robots
 * @author ejt (Creator)
 */

#endif
