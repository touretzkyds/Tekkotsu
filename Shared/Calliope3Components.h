//-*-c++-*-
#ifndef INCLUDED_Calliope3Components_h
#define INCLUDED_Calliope3Components_h

//**** This file defines Calliope components (arms, cameras, AX-S1
//**** sensor module) that are used by CalliopeInfo.h to define
//**** specific robot models such as Calliope2SP.

#include <cmath>
#include <stdlib.h>
#include "CommonInfo.h"
using namespace RobotInfo;

// see http://tekkotsu.org/porting.html#configuration for more information on TGT_HAS_* flags


#if defined(TGT_CALLIOPE3A) || defined(TGT_CALLIOPE3)
#  define TGT_IS_CALLIOPE3
#  define TGT_HAS_HEAD 1
#  define TGT_HAS_CAMERA 1
#  define TGT_HAS_ARMS 1
#  define TGT_HAS_GRIPPER 1
#endif


#ifdef TGT_IS_CALLIOPE3
#  define TGT_IS_CREATE2
#endif

#ifdef TGT_IS_CREATE2
#  define TGT_HAS_WHEELS 2
#  define TGT_HAS_BUTTONS 25
#  define TGT_HAS_LEDS 6 
#endif

#ifndef RAD
//!Just a little macro for converting degrees to radians
#define RAD(deg) (((deg) * (float)M_PI ) / 180.0f)
//!a flag so we undef these after we're done - do you have a cleaner solution?
#define __RI_RAD_FLAG
#endif


//! Contains information about a Calliope robot, such as number of joints, LEDs, etc.
namespace Calliope3Components {
	
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
	const unsigned NumButtons     =  31; //!< the number of buttons that are available
	const unsigned NumLEDs        =  6; //!< The number of LEDs which can be controlled
	const unsigned NumFacePanelLEDs = 0; //!< The number of face panel LEDs
	const unsigned NumSensors     =  36;  //!< the number of sensors available: 33 Create2 + 3 for GPS
	//@}

	const float wheelDiameter = 72.0; //!< Create 2 wheel diameter in mm
	const float wheelBase = 235.0; //!< Create 2 wheel base in mm
	const float encoderTicksPerRev = 508.8; //!< Number of encoder ticks per wheel revolution
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
		WallButOffset, //!< 1 if wall is detected (note correspondence to WALL_SIGNAL_OFFSET's value, avoiding problems if the two are swapped)
		DropLeftWheelButOffset, //!< 1 if left wheel detects dropoff
		DropRightWheelButOffset, //!< 1 if right wheel detects dropoff
		BumpLeftButOffset, //!< 1 if left bumper is pressed
		BumpRightButOffset, //!< 1 if right bumper is pressed
		CliffLeftButOffset, //!< 1 if cliff sensor detects dropoff
		CliffFrontLeftButOffset,
		CliffFrontRightButOffset,
		CliffRightButOffset,
		VirtualWallButOffset, //!< 1 if virtual wall is detected
		OvercurrentLeftWheelOffset, //!< 1 if the left wheel is drawing more than 1 amp
		OvercurrentRightWheelOffset, //!< 1 if the right wheel is drawing more than 1 amp
		OvercurrentMainBrushOffset, //!< 1 if the main brush is drawing more than 1 amp
		OvercurrentSideBrushOffset, //!< 1 if the side brush is drawing more than 1 amp
		CleanButOffset, //!< 1 if clean button is down
		SpotButOffset, //!< 1 if spot button is down
                DockButOffset, //!< 1 if dock button is down
		MinuteButOffset, //!< 1 if minute button is down
		HourButOffset, //!< 1 if hour button is down
		DayButOffset, //!< 1 if day button is down
		ScheduleButOffset, //!< 1 if schedule button is down
		ClockButOffset, //!< 1 if clock button is down
		BaseChargerButOffset, //!< 1 if the home base charger is available
		InternalChargerButOffset, //!< 1 if the internal charger is available
		LtBumpRightOffset, //!< 1 if the right light bumper is pressed
		LtBumpFrontRightOffset,
		LtBumpCenterRightOffset,
		LtBumpCenterLeftOffset,
		LtBumpFrontLeftOffset,
		LtBumpLeftOffset,
		StasisButOffset //!< 1 if the robot is moving forward
	};
	const ButtonOffset_t GreenButOffset = CleanButOffset;
	const ButtonOffset_t YellowButOffset = SpotButOffset;
	const ButtonOffset_t RedButOffset = DockButOffset;

	//! Provides a string name for each button
        const char* const buttonNames[NumButtons + 1] ={
		"Wall",
		"LWheelDrop", "RWheelDrop",
		"LBump", "RBump",
		"LCliff", "LFrontCliff",
		"RCliff", "RFrontCliff",
		"VirtualWall",
		"LOverCurrent", "ROverCurrent", "MBrushOverCurrent", "SBrushOverCurrent",
		"Clean", "Spot", "Dock", "Minute", "Hour", "Day", "Schedule",
		"Clock",
		"BaseCharger", "InternalCharger",
		"LtBumperRight", "LtBumperFrontRight","LtBumperCenterRight", "LtBumperCenterLeft", "LtBumperFrontLeft", "LtBumperLeft",
		"Stasis",
		NULL
	};
		
	//! holds offset to different sensor values in WorldState::sensors[]
	/*! @see WorldState::sensors[] */
	enum SensorOffset_t {
		WallSignalOffset, //!< strength of the wall sensor's signal (note correspondence to WALL_OFFSET's value, avoid problems if the two are swapped)
		DirtDetectOffset, //!< Level of dirt detect sensor
		IRCommOffset, //!< value received by the infrared communication receiver, see IRComm_t for values sent by standard hardware
		IRLeftOffset, //!< value received by the left IR communciation receiver, see IRComm_t for values
		IRRightOffset, //!< value received by the right IR communication receiver, see IRComm_t for values
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
		ReqVelocityOffset, //!< last requested velocity by a drive command
		ReqRadiusOffset, //!< last requested radius by a drive command
		ReqLeftVelocityOffset, //!< last requested velocity of left wheel
		ReqRightVelocityOffset, //!< last requested velocity of right wheel
		LeftEncoderOffset, //!< Count of the encoder of the left wheel
		RightEncoderOffset, //!< Count of the encoder of the right wheel
		LtBumpRightSignalOffset, //!< Signal strength of right light bumper
		LtBumpFrontRightSignalOffset,
		LtBumpCenterRightSignalOffset,
		LtBumpCenterLeftSignalOffset,
		LtBumpFrontLeftSignalOffset,
		LtBumpLeftSignalOffset,
		LeftMotorCurrentOffset, //!< Current of left motor in mA
		RightMotorCurrentOffset, //!< Current of right motor in mA
		SideBrushCurrentOffset, //!< Current of motor for side brush in mA
		MainBrushCurrentOffset, //!< Current of motor for main brush in mA
		GPSXOffset, //!< x-coordinate of robot from GPS or Mirage
		GPSYOffset, //!< y-coordinate of robot from GPS or Mirage
		GPSHeadingOffset, //!< heading of robot from GPS or Mirage
	};
	
	
	//! Provides a string name for each sensor
	const char* const sensorNames[NumSensors+1] = { 
		"WallSignal",
		"Dirt",
		"IR",
		"IRLeft",
		"IRRight",
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
		"RequestedVelocity",
		"RequestedRadius",
		"RequestedLeftVelocity",
		"RequestedRightVelocity",
		"LeftEncoderCount",
		"RightEncoderCount",
		"LightBumperRightSignal",
		"LightBumperFrontRightSignal",
		"LightBumperCenterRightSignal",
		"LightBumperCenterLeftSignal",
		"LightBumperFrontLeftSignal",
		"LightBumperLeftSignal",
		"LeftMotorCurrent",
		"RightMotorCurrent",
		"SideBrushCurrent",
		"MainBrushCurrent",	
		"GPSX",
		"GPSY",
		"GPSHeading",
		NULL
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
		IR_BASE_RED_GREEN_FORCE=254,
		IR_600_FORCE=161,
		IR_600_GREEN=164,
		IR_600_GREEN_FORCE,
		IR_600_RED = 168,
		IR_600_RED_FORCE,
		IR_600_RED_GREEN = 172,
		IR_600_RED_GREEN_FORCE,
		IR_WALL = 162
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


	namespace Calliope3 {
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
			{RAD(-60),RAD(65)}, // gripper
			{  0 , 1 },
			{  0 , 1 },
			{  0 , 1 },
			{  0 , 1 },
			{ MODE_PASSIVE, MODE_SAFE }
		};
	}
}
#endif
