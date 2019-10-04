//-*-c++-*-
#ifndef INCLUDED_CalliopeUInfo_h
#define INCLUDED_CalliopeUInfo_h

#include <cmath>
#include <stdlib.h>
#include "CommonInfo.h"
using namespace RobotInfo;

// see http://tekkotsu.org/porting.html#configuration for more information on TGT_HAS_* flags
#if defined(TGT_CALLIOPEU)
#  define TGT_IS_KOBUKI
#  define TGT_HAS_WHEELS 2
#  define TGT_HAS_BUTTONS 15
#  define TGT_HAS_LEDS 4
#  define TGT_HAS_CAMERA 1
#endif

//! Contains information about the CalliopeU, such as number of joints, LEDs, etc.
namespace CalliopeUInfo {
	
	// *******************************
	//       ROBOT CONFIGURATION
	// *******************************

	extern const char* const TargetName; //!< the name of the model, to be used for logging and remote GUIs

	const unsigned int FrameTime=30;        //!< time between frames in the motion system (milliseconds)
	const unsigned int NumFrames=1;        //!< the number of frames per buffer (don't forget also double buffered)
	const unsigned int SoundBufferTime=32; //!< the number of milliseconds per sound buffer... I'm not sure if this can be changed
	
	//!@name Output Types Information
	const unsigned NumWheels      =  2;
	
	const unsigned FingerJointsPerArm = 0;
	const unsigned JointsPerArm   =  0;
	const unsigned NumArms        =  0;
	const unsigned NumArmJoints   =  JointsPerArm*NumArms;
	
	const unsigned JointsPerLeg   =  0; //!< The number of joints per leg
	const unsigned NumLegs        =  0; //!< The number of legs
	const unsigned NumLegJoints   =  JointsPerLeg*NumLegs; //!< the TOTAL number of joints on ALL legs
	const unsigned NumHeadJoints  =  0; //!< The number of joints in the neck
	const unsigned NumTailJoints  =  0; //!< The number of joints assigned to the tail
	const unsigned NumMouthJoints =  0; //!< the number of joints that control the mouth
	const unsigned NumEarJoints   =  0; //!< The number of joints which control the ears (NOT per ear, is total)
	const unsigned NumButtons     =  11; //!< the number of buttons that are available
	const unsigned NumSensors     =  42;  //!< the number of sensors available
	const unsigned NumLEDs        =  4; //!< The number of LEDs which can be controlled
	const unsigned NumFacePanelLEDs = 0; //!< The number of face panel LEDs
	
	const unsigned NumPIDJoints   = NumWheels; //!< servo pins
	const unsigned NumOutputs     = NumPIDJoints + NumLEDs; //!< the total number of outputs
	const unsigned NumReferenceFrames = NumOutputs + 1 + NumArms + 1; //!< for the base, gripper (* NumArms), and Camera

  const float wheelBase = 230.0; //!< Kobuki wheel base in mm
  const float wheelRadius = 35.0; //!< Kobuki wheel radius in mm
	const float tickToRad = 0.002436916871363930187454f;

	using namespace Camera75DOF;


	// *******************************
	//         OUTPUT OFFSETS
	// *******************************

	//!Corresponds to entries in ERS7Info::PrimitiveName, defined at the end of this file
	//!@name Output Offsets

	
	const unsigned PIDJointOffset = 0; //!< The beginning of the PID Joints
	const unsigned WheelOffset = PIDJointOffset;
	const unsigned HeadOffset  = WheelOffset+NumWheels;   //!< the offset of the beginning of the head joints, add TPROffset_t to get specific joint

	const unsigned LEDOffset   = HeadOffset+NumHeadJoints;
	
	const unsigned ModeOffset = LEDOffset + NumLEDs;

	const unsigned BaseFrameOffset   = NumOutputs; //!< Use with kinematics to refer to base reference frame
	const unsigned CameraFrameOffset = BaseFrameOffset + 1; //!< Use with kinematics to refer to camera reference frame
	
	//! The offsets of appendages with pan (heading), tilt (elevation), note that this should be added to HeadOffset, otherwise use HeadOffset_t (#HeadPanOffset and #HeadTiltOffset)
	enum TPROffset_t {
		PanOffset = 0,      //!< pan/heading (horizontal)
		TiltOffset, //!< tilt/elevation (vertical)
		NodOffset = TiltOffset //!< replicated tilt (could be left undefined instead...)
	};
	
	//! These are 'absolute' offsets for the neck joints, don't need to add to HeadOffset like TPROffset_t values do
	enum HeadOffset_t {
		HeadPanOffset = HeadOffset,      //!< pan/heading (horizontal)
		HeadTiltOffset, //!< tilt/elevation (vertical)
	};

	enum WheelOffset_t {
		SpeedOffset=WheelOffset,
		RadiusOffset
	};
	
	//! The offsets of the individual LEDs
	/*! @hideinitializer */
	enum LEDOffset_t {
		RedLED1Offset=LEDOffset, // PowerRed
		GreenLED1Offset, // PowerGreen
		RedLED2Offset, // Play
		GreenLED2Offset // Advance
	};

	const LEDOffset_t RedLEDOffset = RedLED1Offset;
	const LEDOffset_t YellowLEDOffset = RedLED2Offset;
	const LEDOffset_t GreenLEDOffset = GreenLED1Offset;
	const LEDOffset_t BlueLEDOffset = GreenLED2Offset; 

	typedef unsigned int LEDBitMask_t; //!< So you can be clear when you're refering to a LED bitmask
	
	const LEDBitMask_t RedLEDMask = 1<<(RedLED1Offset-LEDOffset); //!< mask corresponding to RedLEDOffset
	const LEDBitMask_t YellowLEDMask = 1<<(RedLED2Offset-LEDOffset); //!< mask corresponding to YellowLEDOffset
	const LEDBitMask_t GreenLEDMask = 1<<(GreenLED1Offset-LEDOffset); //!< mask corresponding to GreenLEDOffset
	const LEDBitMask_t BlueLEDMask = (1<<(GreenLED2Offset-LEDOffset)); //!< mask corresponding to BlueLEDOffset
	


	//! LEDs for the face panel (all FaceLEDPanelMask<<(0:NumFacePanelLEDs-1) entries)
	const LEDBitMask_t FaceLEDMask = 0;
	//! selects all of the leds
	const LEDBitMask_t AllLEDMask  = (LEDBitMask_t)~0;
	//@}

	//!Offset needed so that the centroid of the robot is correct related to the bounding box
	const fmat::Column<3> AgentBoundingBoxBaseFrameOffset = fmat::pack(0,0,0);

	//! Half of the length, width, and height of the robot without gadgets (E.G arms)
	const fmat::Column<3> AgentBoundingBoxHalfDims = fmat::pack(304.8/2, 304.8/2, 0);

	// *******************************
	//          INPUT OFFSETS
	// *******************************


	//! The order in which inputs should be stored
	//!@name Input Offsets

	//! holds offsets to different buttons in WorldState::buttons[]
	/*! Should be a straight mapping to the ButtonSourceIDs
	 *
	 *  @see WorldState::buttons
	 * @hideinitializer */
	enum ButtonOffset_t {
		B0ButOffset, //!< 1 if play button is down
		B1ButOffset, //!< 1 if advance button is down
		B2ButOffset, //!< 1 if advance button is down
		DropLeftWheelButOffset, //!< 1 if left wheel detects dropoff
		DropRightWheelButOffset, //!< 1 if right wheel detects dropoff
		BumpLeftButOffset, //!< 1 if left bumper is pressed
		BumpRightButOffset, //!< 1 if right bumper is pressed
		BumpCenterButOffset, //!< 1 if center bumber is pressed
		CliffLeftButOffset,
		CliffRightButOffset,
		CliffCenterButOffset,
		//OvercurrentOffset, //!< 1 if the left wheel is drawing more than 1 amp
	};

	const ButtonOffset_t GreenButOffset = B0ButOffset;
	const ButtonOffset_t YellowButOffset = B1ButOffset;
	const ButtonOffset_t RedButOffset = B2ButOffset;

	//! Provides a string name for each button
	const char* const buttonNames[NumButtons+1] = { 
		"Button0", 
		"Button1",
		"Button2",
		"LeftWheelDrop", 
		"RightWheelDrop",
		"BumpLeft", 
		"BumpRight", 
		"BumpCenter",
		"CliffLeft",
		"CliffRight",
		"CliffCenter",
		NULL
	};

	//! holds offset to different sensor values in WorldState::sensors[]
	/*! @see WorldState::sensors[] */
	enum SensorOffset_t {
		//! Core Sensors
		//TimestampOffset,
		//CliffOffset,
		LeftEncoderOffset,
		RightEncoderOffset,
		LeftPwmOffset,
		RightPwmOffset,
		ChargerOffset,
		BatteryOffset,
		OverCurrentOffset,

		//! Dock IR
		Docking0Offset,
		Docking1Offset,
		Docking2Offset,

		//! Inertia
		AngleOffset,
		AngleRateOffset,
		Acc0Offset,
		Acc1Offset,
		Acc2Offset,

		//! Cliff
		Bottom0Offset,
		Bottom1Offset,
		Bottom2Offset,

		//! Current
		Current0Offset,
		Current1Offset,

		//! Three Axis Gyro
		FrameIdOffset,
		FollowedDataLenghtOffset,
		GyroParam0Offset,
		GyroParam1Offset,
		GyroParam2Offset,
		GyroParam3Offset,
		GyroParam4Offset,
		GyroParam5Offset,
		GyroParam6Offset,
		GyroParam7Offset,
		GyroParam8Offset,

		//! GpInput
		DigitalInputOffset,
		AnalogInput0Offset,
		AnalogInput1Offset,
		AnalogInput2Offset,
		AnalogInput3Offset,
		AnalogInput4Offset,
		AnalogInput5Offset,
		AnalogInput6Offset,
		//ModeStateOffset, //!< one of #ModeState_t
		GPSXOffset, //!< x-coordinate of robot from GPS or Mirage
		GPSYOffset, //!< y-coordinate of robot from GPS or Mirage
		GPSHeadingOffset, //!< heading of robot from GPS or Mirage
	};

	//! Provides a string name for each sensor
	const char* const sensorNames[NumSensors+1] = { 
		//! Core Sensors
		//"Timestamp",
		//"CliffOffset",
		"LeftEncoder",
		"RightEncoder",
		"LeftPwm",
		"RightPwm",
		"Charger",
		"Battery",
		"OverCurrent",

		//! Dock IR
		"Docking0",
		"Docking1",
		"Docking2",

		//! Inertia
		"Angle",
		"AngleRate",
		"Acc0",
		"Acc1",
		"Acc2",

		//! Cliff
		"CliffBottom0",
		"CliffBottom1",
		"CliffBottom2",

		//! Current
		"Current0",
		"Current2",
	
		//! Three Axis Gyro
		"FrameId",
		"FollowedDataLenght",
		"GyroParam0",
		"GyroParam1",
		"GyroParam2",
		"GyroParam3",
		"GyroParam4",
		"GyroParam5",
		"GyroParam6",
		"GyroParam7",
		"GyroParam8",

		//! Gp Input
		"DigitalInput0",
		"AnalogInput0",
		"AnalogInput1",
		"AnalogInput2",
		"AnalogInput3",
		"AnalogInput4",
		"AnalogInput5",
		"AnalogInput6",
		"GPSX",
		"GPSY",
		"GPSHeading",
		NULL
		
	};

	//@}


	//! Names for each of the outputs
	const char* const outputNames[NumReferenceFrames+1] = {
		"Speed",
		"Radius",
		"LED1:Red",
		"LED1:Green",
		"LED2:Red",
		"LED2:Green",
		"BaseFrame",
		"CameraFrame",
		NULL
	};
	
	//! provides polymorphic robot capability detection/mapping
	class CalliopeUCapabilities : public Capabilities {
	public:
		//! constructor
		CalliopeUCapabilities()
		: Capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,
			       NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs)
		{}
	};
	//! allocation declared in RobotInfo.cc
	extern const CalliopeUCapabilities capabilities;
	
	//! This table holds the default PID values for each joint.  see PIDMC
	const float DefaultPIDs[NumPIDJoints+1][3] = {
	  {1,0,0},
	  {1,0,0},
	};
	
	//!These values are our recommended maximum joint velocities, in rad/ms
	const float MaxOutputSpeed[NumOutputs] = {
	  0, 0,  // wheels
	  0, 0, 0, 0 // LEDs
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
			{ -500 , 500 },
			{ -500 , 500 },
			{  0 , 1 }, // leds
			{  0 , 1 },
			{  0 , 1 },
			{  0 , 1 }
		};

	//! This table holds the mechanical limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
	/*! Same as #outputRanges, don't know actual values because they were never specified by Sony */
	const float mechanicalLimits[NumOutputs][2] =
		{
			{ -500 , 500 },
			{ -500 , 500 },
			{  0 , 1 },  // leds
			{  0 , 1 },
			{  0 , 1 },
			{  0 , 1 },
		};

#ifdef __RI_RAD_FLAG
#undef RAD
#undef __RI_RAD_FLAG
#endif
}

/*! @file
 * @brief Defines some capabilities of the CalliopeU robot
 * @author ejt (Creator)
 */

#endif
