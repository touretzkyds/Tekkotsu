//-*-c++-*-
#ifndef INCLUDED_CalliopeComponents_h
#define INCLUDED_CalliopeComponents_h

//**** This file defines Calliope components (arms, cameras, AX-S1
//**** sensor module) that are used by CalliopeInfo.h to define
//**** specific robot models such as Calliope2SP.

#include <cmath>
#include <stdlib.h>
#include "CommonInfo.h"
using namespace RobotInfo;

// see http://tekkotsu.org/porting.html#configuration for more information on TGT_HAS_* flags

#if defined(TGT_CALLIOPE)
#  define TGT_IS_CALLIOPE
#endif

#if defined(TGT_CALLIOPESP) || defined(TGT_CALLIOPELP) || defined(TGT_CALLIOPEKP)
#  define TGT_IS_CALLIOPE
#  define TGT_IS_CALLIOPE0
#endif

#if defined(TGT_CALLIOPE2SP) || defined(TGT_CALLIOPE2LP) || defined(TGT_CALLIOPE2KP)
#  define TGT_IS_CALLIOPE
#  define TGT_IS_CALLIOPE2
#endif

#if defined(TGT_CALLIOPE5SP) || defined(TGT_CALLIOPE5LP) || defined(TGT_CALLIOPE5KP)
#  define TGT_IS_CALLIOPE
#  define TGT_IS_CALLIOPE5
#endif

#if defined(TGT_CALLIOPEKP) || defined(TGT_CALLIOPE2KP) || defined(TGT_CALLIOPE5KP)
#  define TGT_HAS_KINECT
#endif

// Common to all Calliope models
#if defined(TGT_IS_CALLIOPE)
#  define TGT_IS_CREATE
#  define TGT_HAS_WHEELS 2
#  define TGT_HAS_BUTTONS 15
#  define TGT_HAS_LEDS 4
#  define TGT_HAS_CAMERA 1
#endif

// Everything but the base model (old TGT_CREATE) has a pan-tilt
#if !defined(TGT_CALLIOPE)
#  define TGT_HAS_HEAD 1
#endif

// 2-DOF arm with open-close gripper
#if defined(TGT_IS_CALLIOPE2)
#  define TGT_HAS_ARMS 1
#  define TGT_HAS_GRIPPER 1
#endif

// 5-DOF arm with independent fingers
#if defined(TGT_IS_CALLIOPE5)
#  define TGT_HAS_ARMS 1
#  define TGT_HAS_GRIPPER 1
#  define TGT_HAS_FINGERS 2
#endif

// Models with a head but not a Kinect have an AX-S1 with 3 IR rangefinders
#if defined(TGT_HAS_HEAD) && !defined(TGT_HAS_KINECT)
#  define TGT_HAS_IR_DISTANCE 3
#endif

#ifndef RAD
//!Just a little macro for converting degrees to radians
#define RAD(deg) (((deg) * (float)M_PI ) / 180.0f)
//!a flag so we undef these after we're done - do you have a cleaner solution?
#define __RI_RAD_FLAG
#endif


//! Contains information about a Calliope robot, such as number of joints, LEDs, etc.
namespace CalliopeComponents {
	
	// *******************************
	//       ROBOT CONFIGURATION
	// *******************************

	const unsigned int FrameTime=30;        //!< time between frames in the motion system (milliseconds)
	const unsigned int NumFrames=1;        //!< the number of frames per buffer (don't forget also double buffered)
	const unsigned int SoundBufferTime=32; //!< the number of milliseconds per sound buffer... I'm not sure if this can be changed
	
	//!@name Output Types Information
	const unsigned NumWheels      =  2;
	
	const unsigned JointsPerLeg   =  0; //!< The number of joints per leg
	const unsigned NumLegs        =  0; //!< The number of legs
	const unsigned NumLegJoints   =  JointsPerLeg*NumLegs; //!< the TOTAL number of joints on ALL legs
	const unsigned NumTailJoints  =  0; //!< The number of joints assigned to the tail
	const unsigned NumMouthJoints =  0; //!< the number of joints that control the mouth
	const unsigned NumEarJoints   =  0; //!< The number of joints which control the ears (NOT per ear, is total)
	const unsigned NumButtons     =  15; //!< the number of buttons that are available
	const unsigned NumLEDs        =  4; //!< The number of LEDs which can be controlled
	const unsigned NumFacePanelLEDs = 0; //!< The number of face panel LEDs
	//@}


	// *******************************
	//         OUTPUT OFFSETS
	// *******************************

	//!Corresponds to entries in outputNames, defined at the end of this file
	//!@name Output Offsets

	
	const unsigned PIDJointOffset = 0; //!< The beginning of the PID Joints
	const unsigned WheelOffset = PIDJointOffset;

	enum WheelOffset_t {
		LWheelOffset=WheelOffset,
		RWheelOffset
	};
	
	//! Offset needed so that the centroid of the robot is correct related to the bounding box
	const fmat::Column<3> AgentBoundingBoxBaseFrameOffset = fmat::pack(838.4/2-304.8/2,0,0);

	//! Half of the length, width, and height of the robot.
	const fmat::Column<3> AgentBoundingBoxHalfDims = fmat::pack(838.4/2, 304.8/2, 0);

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
		PlayButOffset, //!< 1 if play button is down
		AdvanceButOffset, //!< 1 if advance button is down
		WallButOffset, //!< 1 if wall is detected (note correspondence to WALL_SIGNAL_OFFSET's value, avoiding problems if the two are swapped)
		DropCasterButOffset, //!< 1 if caster detects dropoff
		DropLeftWheelButOffset, //!< 1 if left wheel detects dropoff
		DropRightWheelButOffset, //!< 1 if right wheel detects dropoff
		BumpLeftButOffset, //!< 1 if left bumper is pressed
		BumpRightButOffset, //!< 1 if right bumper is pressed
		OvercurrentLeftWheelOffset, //!< 1 if the left wheel is drawing more than 1 amp
		OvercurrentRightWheelOffset, //!< 1 if the right wheel is drawing more than 1 amp
		LowSideDriver0ButOffset, //!< 1 if low side driver 0 is pulling more than 0.5 amps
		LowSideDriver1ButOffset, //!< 1 if low side driver 1 is pulling more than 0.5 amps
		LowSideDriver2ButOffset, //!< 1 if low side driver 2 is pulling more than 1.6 amps
		BaseChargerButOffset, //!< 1 if the home base charger is available
		InternalChargerButOffset //!< 1 if the internal charger is available
	};

	const ButtonOffset_t GreenButOffset = PlayButOffset;
	const ButtonOffset_t YellowButOffset = AdvanceButOffset;
	const ButtonOffset_t RedButOffset = PlayButOffset;

	//! Provides a string name for each button
	const char* const buttonNames[NumButtons+1] = { 
		"Play", "Advance",
		"Wall",
		"CasterDrop", "LWheelDrop", "RWheelDrop",
		"LBump", "RBump",
		"LOverCurrent", "ROverCurrent",
		"DriverLow0", "DriverLow1", "DriverLow2",
		"BaseCharger", "InternalCharger",
		NULL
	};

	//! holds offset to different sensor values in WorldState::sensors[]
	/*! @see WorldState::sensors[] */
	enum SensorOffset_t {
		DigitalInput0Offset, //!< the digital input pins in bits 0 through 4
		DigitalInput1Offset, //!< the digital input pins in bits 0 through 4
		DigitalInput2Offset, //!< the digital input pins in bits 0 through 4
		DigitalInput3Offset, //!< the digital input pins in bits 0 through 4
		AnalogSignalOffset, //!< voltage on cargo bay pin 4
		WallSignalOffset, //!< strength of the wall sensor's signal (note correspondence to WALL_OFFSET's value, avoid problems if the two are swapped)
		IRCommOffset, //!< value received by the infrared communication receiver, see IRComm_t for values sent by standard hardware
		CliffLeftSignalOffset, //!< strength of the left cliff sensor
		CliffFrontLeftSignalOffset, //!< strength of the front left cliff sensor
		CliffFrontRightSignalOffset, //!< strength of the front right cliff sensor
		CliffRightSignalOffset, //!< strength of the right cliff sensor
		EncoderDistanceOffset, //!< average distance (mm) traveled by the wheels since last update
		EncoderAngleOffset, //!< average angle (radians) rotated since the last update
		VoltageOffset, //!< mV measured at battery
		CurrentOffset, //!< mA flowing into battery (negative when discharging)
		BatteryChargeOffset, //!< mAh remaining in battery (may not be accurate with alkaline battery pack)
		BatteryTempOffset, //!< degrees celsius
		ChargingStateOffset, //!< one of #ChargingState_t
		ModeStateOffset, //!< one of #ModeState_t
		GPSXOffset, //!< x-coordinate of robot from GPS or Mirage
		GPSYOffset, //!< y-coordinate of robot from GPS or Mirage
		GPSHeadingOffset, //!< heading of robot from GPS or Mirage
	};
	
	enum IRComm_t {
		IR_REMOTE_LEFT=129,
		IR_REMOTE_FORWARD,
		IR_REMOTE_RIGHT,
		IR_REMOTE_SPOT,
		IR_REMOTE_MAX,
		IR_REMOTE_SMALL,
		IR_REMOTE_MEDIUM,
		IR_REMOTE_LARGE,
		IR_REMOTE_PAUSE,
		IR_REMOTE_POWER,
		IR_REMOTE_ARC_LEFT, 
		IR_REMOTE_ARC_RIGHT,
		IR_REMOTE_STOP,
		IR_REMOTE_SEND,
		IR_REMOTE_DOCK,
		IR_BASE_RED=248,
		IR_BASE_GREEN=244,
		IR_BASE_FORCE=242,
		IR_BASE_RED_GREEN=252,
		IR_BASE_RED_FORCE=250,
		IR_BASE_GREEN_FORCE=246,
		IR_BASE_RED_GREEN_FORCE=254
	};
	/*const unsigned IR_BASE_MASK=240;
	const unsigned IR_BASE_RED_MASK=8;
	const unsigned IR_BASE_GREEN_MASK=4;
	const unsigned IR_BASE_FORCE_MASK=2;*/
	
	enum ChargingState_t {
		CHARGING_OFF,
		CHARGING_RECONDITIONING,
		CHARGING_FULL,
		CHARGING_TRICKLE,
		CHARGING_WAITING,
		CHARGING_FAULT
	};

	enum ModeState_t {
		MODE_OFF,
		MODE_PASSIVE,
		MODE_SAFE,
		MODE_FULL
	};
		
	//@}


	namespace WithoutAXS1Sensors {
		const unsigned NumSensors     =  22;  //!< the number of sensors available: 19 Create
		
		//! Provides a string name for each sensor
		const char* const sensorNames[NumSensors+1] = { 
			"DigitalIn0",
			"DigitalIn1",
			"DigitalIn2",
			"DigitalIn3",
			"AnalogIn",
			"WallSignal",
			"IR",
			"CliffLeftSignal",
			"CliffFrontLeftSignal",
			"CliffFrontRightSignal",
			"CliffRight",
			"Distance",
			"Angle",
			"BatteryVoltage",
			"BatteryCurrent",
			"BatteryCharge",
			"BatteryTemp",
			"ChargingState",
			"ModeState",
			"GPSX",
			"GPSY",
			"GPSHeading",
			NULL
		};
		
	}
	
	namespace WithAXS1Sensors {
		const unsigned NumSensors     =  WithoutAXS1Sensors::NumSensors+3+3+2;  //!< the number of sensors available: 19 Create plus 3 IR, 3 luminosity, 2 mic from AX-S1
		
		enum AXS1SensorOffset_t {
			LeftIRDistOffset = GPSHeadingOffset+1,
			CenterIRDistOffset,
			IRDistOffset = CenterIRDistOffset,
			RightIRDistOffset,
			LeftLuminosityOffset,
			CenterLuminosityOffset,
			RightLuminosityOffset,
			MicVolumeOffset,
			MicSpikeCountOffset
		};
		
		//! Provides a string name for each sensor
		const char* const sensorNames[NumSensors+1] = { 
			"DigitalIn0",
			"DigitalIn1",
			"DigitalIn2",
			"DigitalIn3",
			"AnalogIn",
			"WallSignal",
			"IR",
			"CliffLeftSignal",
			"CliffFrontLeftSignal",
			"CliffFrontRightSignal",
			"CliffRight",
			"Distance",
			"Angle",
			"BatteryVoltage",
			"BatteryCurrent",
			"BatteryCharge",
			"BatteryTemp",
			"ChargingState",
			"ModeState",
			"GPSX",
			"GPSY",
			"GPSHeading",
			"LeftIRDist", "CenterIRDist", "RightIRDist",
			"LeftLuminosity", "CenterLuminosity", "RightLuminosity",
			"MicVolume", "MicSpikeCount",
			NULL
		};
	}
	
	namespace WithHead {
		const unsigned NumHeadJoints = 2; //!< The number of joints in the pan/tilt
		const unsigned HeadOffset  = WheelOffset+NumWheels;   //!< the offset of the beginning of the head joints, add TPROffset_t to get specific joint
		
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
	}
	
	// No arm, no head
	namespace CalliopeInfo {
		const unsigned FingerJointsPerArm   =  0;
		const unsigned JointsPerArm   =  0;
		const unsigned NumArms        =  0;
		const unsigned NumArmJoints   =  JointsPerArm*NumArms;
		const unsigned NumHeadJoints = 0;
		const unsigned NumPIDJoints   = NumWheels + NumHeadJoints + NumArmJoints; //!< number of motors and servos
		const unsigned NumOutputs     = NumPIDJoints + NumLEDs + 1 /*Create mode*/; //!< the total number of outputs

		const unsigned LEDOffset   = WheelOffset+NumWheels;
		const unsigned ModeOffset = LEDOffset + NumLEDs;
		
		const unsigned BaseFrameOffset   = NumOutputs; //!< Use with kinematics to refer to base reference frame
		const unsigned CameraFrameOffset = BaseFrameOffset+1; //!< Use with kinematics to refer to camera reference frame

		//! The offsets of the individual LEDs
		/*! @hideinitializer */
		enum LEDOffset_t {
			PowerRedLEDOffset=LEDOffset,
			PowerGreenLEDOffset,
			PlayLEDOffset,
			AdvanceLEDOffset
		};
		
		const LEDOffset_t RedLEDOffset = PowerRedLEDOffset;
		const LEDOffset_t BlueLEDOffset = AdvanceLEDOffset; //!< Create has no blue LED: use Advance LED here, and Advance+PowerRED in BlueLEDMask
		const LEDOffset_t GreenLEDOffset = PlayLEDOffset;
		const LEDOffset_t YellowLEDOffset = AdvanceLEDOffset;
		
		typedef unsigned int LEDBitMask_t; //!< So you can be clear when you're refering to a LED bitmask
		
		const LEDBitMask_t BlueLEDMask = (1<<(AdvanceLEDOffset-LEDOffset)) |
		(1<<(PowerRedLEDOffset-LEDOffset)); //!< Create has no blue LED, so use Advance (green) + Power (red)
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

		//! This table holds the default PID values for each joint.  see PIDMC
		const float DefaultPIDs[NumPIDJoints][3] = {
			{1,0,0},
			{1,0,0},
		};
		
		//!These values are our recommended maximum joint velocities, in rad/ms
		const float MaxOutputSpeed[NumOutputs] = {
			0, 0,   // wheels
			0,      // leds
			0,
			0,
			0
		};
		
		//! This table holds the software limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
		const float outputRanges[NumOutputs][2] =
		{
			{ -500 , 500 }, // left wheel
			{ -500 , 500 }, // right wheel
			{  0 , 1 },
			{  0 , 1 },
			{  0 , 1 },
			{  0 , 1 },
			{ MODE_PASSIVE, MODE_SAFE }
		};
		
	}
	
	// no arm, with pan/tilt head and AX-S1
	namespace Calliope0 {
		using namespace CalliopeComponents::WithHead;
		
		const unsigned FingerJointsPerArm = 0;
		const unsigned JointsPerArm   =  0;
		const unsigned NumArms        =  0;
		const unsigned NumArmJoints   =  JointsPerArm*NumArms;
		const unsigned NumPIDJoints   = NumWheels + NumHeadJoints + NumArmJoints; //!< number of motors and servos
		const unsigned NumOutputs     = NumPIDJoints + NumLEDs + 1 /*Create mode*/; //!< the total number of outputs

		const unsigned LEDOffset   = WheelOffset+NumWheels;
		const unsigned ModeOffset = LEDOffset + NumLEDs;
		
		const unsigned BaseFrameOffset   = NumOutputs; //!< Use with kinematics to refer to base reference frame
		const unsigned CameraFrameOffset = BaseFrameOffset+1; //!< Use with kinematics to refer to camera reference frame
		const unsigned LeftIRFrameOffset = CameraFrameOffset+1; //!< Use with kinematics to refer to left IR distance rangefinder reference frame
		const unsigned CenterIRFrameOffset = LeftIRFrameOffset+1; //!< Use with kinematics to refer to center IR distance rangefinder reference frame
		const unsigned IRFrameOffset = CenterIRFrameOffset; //!< alias for CenterIRFrameOffset
		const unsigned RightIRFrameOffset = CenterIRFrameOffset+1; //!< Use with kinematics to refer to right IR distance rangefinder reference frame
		
		//! The offsets of the individual LEDs
		/*! @hideinitializer */
		enum LEDOffset_t {
			PowerRedLEDOffset=LEDOffset,
			PowerGreenLEDOffset,
			PlayLEDOffset,
			AdvanceLEDOffset
		};
		
		const LEDOffset_t RedLEDOffset = PowerRedLEDOffset;
		const LEDOffset_t BlueLEDOffset = AdvanceLEDOffset; //!< Create has no blue LED: use Advance LED here, and Advance+PowerRED in BlueLEDMask
		const LEDOffset_t GreenLEDOffset = PlayLEDOffset;
		const LEDOffset_t YellowLEDOffset = AdvanceLEDOffset;
		
		typedef unsigned int LEDBitMask_t; //!< So you can be clear when you're refering to a LED bitmask
		
		const LEDBitMask_t BlueLEDMask = (1<<(AdvanceLEDOffset-LEDOffset)) |
		(1<<(PowerRedLEDOffset-LEDOffset)); //!< Create has no blue LED, so use Advance (green) + Power (red)
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
		//! This table holds the default PID values for each joint.  see PIDMC
		const float DefaultPIDs[NumPIDJoints][3] = {
			{1,0,0},
			{1,0,0},
			{32,32,1},
			{32,32,1}
		};
		
		//!These values are our recommended maximum joint velocities, in rad/ms
		const float MaxOutputSpeed[NumOutputs] = {
			0, 0,       // wheels
			1.f, 1.f,   // head
			0,          // leds
			0,
			0,
			0
		};
		
		//! This table holds the software limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
		const float outputRanges[NumOutputs][2] =
		{
			{ -500 , 500 }, // left wheel
			{ -500 , 500 }, // right wheel
			{RAD(-150),RAD(150)}, // neck pan
			{RAD(-92),RAD(75)}, // neck tilt
			{  0 , 1 },
			{  0 , 1 },
			{  0 , 1 },
			{  0 , 1 },
			{ MODE_PASSIVE, MODE_SAFE }
		};
		
	}
	
	// 2-dof arm, with pan/tilt head and AX-S1
	namespace Calliope2 {
		using namespace CalliopeComponents::WithHead;
		
		const unsigned FingerJointsPerArm = 1;
		const unsigned JointsPerArm   =  3; //!< 2 arm joints plus gripper
		const unsigned NumArms        =  1;
		const unsigned NumArmJoints   =  JointsPerArm*NumArms;
		const unsigned NumPIDJoints   = NumWheels + NumHeadJoints + NumArmJoints; //!< number of motors and servos
		const unsigned NumOutputs     = NumPIDJoints + NumLEDs + 1 /*Create mode*/; //!< the total number of outputs

		const unsigned ArmOffset = HeadOffset+NumHeadJoints;
		const unsigned LEDOffset   = ArmOffset+NumArmJoints;
		const unsigned ModeOffset = LEDOffset + NumLEDs;
		
		const unsigned BaseFrameOffset   = NumOutputs; //!< Use with kinematics to refer to base reference frame
		const unsigned GripperFrameOffset = BaseFrameOffset + 1; //!< Use with kinematics to refer to gripper reference frame
		const unsigned CameraFrameOffset = GripperFrameOffset+NumArms; //!< Use with kinematics to refer to camera reference frame
		const unsigned LeftIRFrameOffset = CameraFrameOffset+1; //!< Use with kinematics to refer to left IR distance rangefinder reference frame
		const unsigned CenterIRFrameOffset = LeftIRFrameOffset+1; //!< Use with kinematics to refer to center IR distance rangefinder reference frame
		const unsigned IRFrameOffset = CenterIRFrameOffset; //!< alias for CenterIRFrameOffset
		const unsigned RightIRFrameOffset = CenterIRFrameOffset+1; //!< Use with kinematics to refer to right IR distance rangefinder reference frame

		//! These are 'absolute' offsets for the arm joints, don't need to add to ArmOffset like TPROffset_t values do
		enum ArmOffset_t {
			ArmBaseOffset=ArmOffset,
			ArmShoulderOffset,
			GripperOffset
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
		const LEDOffset_t BlueLEDOffset = AdvanceLEDOffset; //!< Create has no blue LED: use Advance LED here, and Advance+PowerRED in BlueLEDMask
		const LEDOffset_t GreenLEDOffset = PlayLEDOffset;
		const LEDOffset_t YellowLEDOffset = AdvanceLEDOffset;
		
		typedef unsigned int LEDBitMask_t; //!< So you can be clear when you're refering to a LED bitmask
		
		const LEDBitMask_t BlueLEDMask = (1<<(AdvanceLEDOffset-LEDOffset)) |
		(1<<(PowerRedLEDOffset-LEDOffset)); //!< Create has no blue LED, so use Advance (green) + Power (red)
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
		//! This table holds the default PID values for each joint.  see PIDMC
		const float DefaultPIDs[NumPIDJoints][3] = {
			{1,0,0},
			{1,0,0},
			{32,32,1},
			{32,32,1},
			{32,32,1},
			{32,32,1},
			{32,32,1}
		};
		
		//!These values are our recommended maximum joint velocities, in rad/ms
		const float MaxOutputSpeed[NumOutputs] = {
			0, 0,   // wheels
			1.f, 1.f,   // head
			1.f, 1.f, 1.f, // arm
			0,
			0,
			0,
			0,
			0
		};
		
		//! This table holds the software limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
		const float outputRanges[NumOutputs][2] =
		{
			{ -500 , 500 }, // left wheel
			{ -500 , 500 }, // right wheel
			{RAD(-150),RAD(150)}, // neck pan
			{RAD(-92),RAD(75)}, // neck tilt
			{RAD(-150),RAD(75)}, // arm base
			{RAD(-60),RAD(49)}, // arm shoulder
			{RAD(0),RAD(130)}, // gripper
			{  0 , 1 },
			{  0 , 1 },
			{  0 , 1 },
			{  0 , 1 },
			{ MODE_PASSIVE, MODE_SAFE }
		};
		
	}
	
	// 5-dof arm, with pan/tilt head (but see sub-spaces re: AX-S1...)
	namespace Calliope5 {
		using namespace CalliopeComponents::WithHead;
		
		const unsigned FingerJointsPerArm = 2;
		const unsigned JointsPerArm   =  7; //!< 5 arms joints plus two fingers
		const unsigned NumArms        =  1;
		const unsigned NumArmJoints   =  JointsPerArm*NumArms;
		const unsigned NumPIDJoints   = NumWheels + NumHeadJoints + NumArmJoints; //!< number of motors and servos
		const unsigned NumOutputs     = NumPIDJoints + NumLEDs + 1 /*Create mode*/; //!< the total number of outputs

		const unsigned ArmOffset = HeadOffset+NumHeadJoints;
		const unsigned LEDOffset   = ArmOffset+NumArmJoints;
		const unsigned ModeOffset = LEDOffset + NumLEDs;
		
		const unsigned BaseFrameOffset   = NumOutputs; //!< Use with kinematics to refer to base reference frame
		const unsigned GripperFrameOffset = BaseFrameOffset + 1; //!< Use with kinematics to refer to gripper reference frame
		const unsigned LeftFingerFrameOffset = GripperFrameOffset+1; //!< Use with kinematics to refer to the left finger reference frame
		const unsigned RightFingerFrameOffset = LeftFingerFrameOffset+1; //!< Use with kinematics to refer to the right finger reference frame
		const unsigned CameraFrameOffset = RightFingerFrameOffset+NumArms; //!< Use with kinematics to refer to camera reference frame

		//! These are 'absolute' offsets for the arm joints, don't need to add to ArmOffset like TPROffset_t values do
		enum ArmOffset_t {
			ArmBaseOffset=ArmOffset,
			ArmShoulderOffset,
			ArmElbowOffset,
			ArmWristOffset,
			WristRotateOffset,
			LeftFingerOffset,
			RightFingerOffset
		};
		
		namespace WithoutAXS1Sensors {
			using namespace Calliope5;
			using namespace CalliopeComponents::WithoutAXS1Sensors;
		}
		namespace WithAXS1Sensors {
			using namespace Calliope5;
			using namespace CalliopeComponents::WithAXS1Sensors;
			const unsigned LeftIRFrameOffset = CameraFrameOffset+1; //!< Use with kinematics to refer to left IR distance rangefinder reference frame
			const unsigned CenterIRFrameOffset = LeftIRFrameOffset+1; //!< Use with kinematics to refer to center IR distance rangefinder reference frame
			const unsigned IRFrameOffset = CenterIRFrameOffset; //!< alias for CenterIRFrameOffset
			const unsigned RightIRFrameOffset = CenterIRFrameOffset+1; //!< Use with kinematics to refer to right IR distance rangefinder reference frame
		}

		//! The offsets of the individual LEDs
		/*! @hideinitializer */
		enum LEDOffset_t {
			PowerRedLEDOffset=LEDOffset,
			PowerGreenLEDOffset,
			PlayLEDOffset,
			AdvanceLEDOffset
		};
		
		const LEDOffset_t RedLEDOffset = PowerRedLEDOffset;
		const LEDOffset_t BlueLEDOffset = AdvanceLEDOffset; //!< Create has no blue LED: use Advance LED here, and Advance+PowerRED in BlueLEDMask
		const LEDOffset_t GreenLEDOffset = PlayLEDOffset;
		const LEDOffset_t YellowLEDOffset = AdvanceLEDOffset;
		
		typedef unsigned int LEDBitMask_t; //!< So you can be clear when you're refering to a LED bitmask
		
		const LEDBitMask_t BlueLEDMask = (1<<(AdvanceLEDOffset-LEDOffset)) |
		(1<<(PowerRedLEDOffset-LEDOffset)); //!< Create has no blue LED, so use Advance (green) + Power (red)
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
		//! This table holds the default PID values for each joint.  see PIDMC
		const float DefaultPIDs[NumPIDJoints][3] = {
			{1,0,0},
			{1,0,0},
			{32,32,1},
			{32,32,1},
			{32,32,1},
			{32,32,1},
			{32,32,1},
			{32,32,1},
			{32,32,1},
			{32,32,1},
			{32,32,1}
		};
		
		//!These values are our recommended maximum joint velocities, in rad/s
		const float MaxOutputSpeed[NumOutputs] = {
			0, 0,       // wheels
			1.0, 1.0,   // head pan and tilt
			1.f,        // arm base
			1.f,        // shoulder
			1.f,        // elbow
			1.f,        // wrist
			1.f,        // wrist rotate
			1.f, 1.f,   // left and right finger
			0,0,0,0     // Create LEDs
		};
		
		//! This table holds the software limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
		const float outputRanges[NumOutputs][2] =
		{
			{    -500 ,    500 }, // left wheel
			{    -500 ,    500 }, // right wheel
			{RAD(-150),RAD(150)}, // neck pan
			{RAD(-99) ,RAD(107)}, // neck tilt
			{RAD( -95),RAD( 84)}, // arm base
			{RAD( -55),RAD(149)}, // arm shoulder
			{RAD(-149),RAD(149)}, // arm elbow
			{RAD(-132),RAD(130)}, // arm wrist
			{RAD(-149),RAD(149)}, // arm wristrot
			{RAD( -70),RAD( 25)}, // arm left finger
			{RAD( -25),RAD( 70)}, // arm right finger
			{       0 ,      1 }, // LED power red
			{       0 ,      1 }, // LED power green
			{       0 ,      1 }, // LED play
			{       0 ,      1 }, // LED advance
			{ MODE_PASSIVE, MODE_SAFE }
		};
		
	}
	
}

#endif
