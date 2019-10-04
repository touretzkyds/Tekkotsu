//-*-c++-*-
#ifndef INCLUDED_WiiMoteInfo_h
#define INCLUDED_WiiMoteInfo_h

#include <cmath>
#include <stdlib.h>
#include "CommonInfo.h"
using namespace RobotInfo;

// see http://tekkotsu.org/porting.html#configuration for more information on TGT_HAS_* flags
#if defined(TGT_WIIMOTE)
#  define TGT_HAS_LEDS 2
#endif

//! Contains information about the WiiMote remote control
namespace WiiMoteInfo {
	
	// *******************************
	//       ROBOT CONFIGURATION
	// *******************************

	extern const char* const TargetName; //!< the name of the model, to be used for logging and remote GUIs

	const unsigned int FrameTime=15;        //!< time between frames in the motion system (milliseconds)
	const unsigned int NumFrames=1;        //!< the number of frames per buffer (don't forget also double buffered)
	const unsigned int SoundBufferTime=32; //!< the number of milliseconds per sound buffer... I'm not sure if this can be changed
	
	//!@name Output Types Information
	const unsigned NumWheels      =  0;
	
	const unsigned JointsPerArm   =  0;
	const unsigned NumArms        =  0;
	const unsigned NumArmJoints   =  0;
	
	const unsigned JointsPerLeg   =  0; //!< The number of joints per leg
	const unsigned NumLegs        =  0; //!< The number of legs
	const unsigned NumLegJoints   =  0;
	const unsigned NumHeadJoints  =  0; //!< The number of joints in the neck
	const unsigned NumTailJoints  =  0; //!< The number of joints assigned to the tail
	const unsigned NumMouthJoints =  0; //!< the number of joints that control the mouth
	const unsigned NumEarJoints   =  0; //!< The number of joints which control the ears (NOT per ear, is total)
	const unsigned NumButtons     =  11; //!< the number of buttons that are available, 2 head, 4 paws, 3 back, 1 underbelly see ERS7Info::ButtonOffset_t
	const unsigned NumSensors     =  3;  //!< the number of sensors available
	const unsigned NumLEDs        =  4; //!< The number of LEDs which can be controlled
	const unsigned NumFacePanelLEDs = 0; //!< The number of face panel LEDs
	
	const unsigned NumPIDJoints   = NumWheels; //!< servo pins
	const unsigned NumOutputs     = NumWheels + NumLEDs; //!< the total number of outputs
	const unsigned NumReferenceFrames = NumOutputs + 1 + NumArms; //!< for the base, gripper (* NumArms)

	// webcam ?
	const float CameraHorizFOV=0;
	const float CameraVertFOV=0;
	const float CameraFOV=0;
	const unsigned int CameraResolutionX=0;
	const unsigned int CameraResolutionY=0;
	//@}


	// *******************************
	//         OUTPUT OFFSETS
	// *******************************

	//!Corresponds to entries in ERS7Info::PrimitiveName, defined at the end of this file
	//!@name Output Offsets

	
	const unsigned PIDJointOffset = 0; //!< The beginning of the PID Joints
	const unsigned WheelOffset = PIDJointOffset;

	const unsigned LEDOffset   = WheelOffset+NumWheels;
	
	const unsigned MODE_OFFSET = LEDOffset + NumLEDs;
	const unsigned DEMO_OFFSET = MODE_OFFSET+1;

	const unsigned BaseFrameOffset   = NumOutputs; //!< Use with kinematics to refer to base reference frame

	enum WheelOffset_t { };
	
	//! The offsets of the individual LEDs on the head and tail.  Note that left/right are robot's point of view.  See also LEDBitMask_t
	/*! @hideinitializer */
	enum LEDOffset_t {
		LED0_Offset=LEDOffset,
		LED1_Offset,
		LED2_Offset,
		LED3_Offset
	};
	
	typedef unsigned int LEDBitMask_t; //!< So you can be clear when you're refering to a LED bitmask
	//! LEDs for the face panel (all FaceLEDPanelMask<<(0:NumFacePanelLEDs-1) entries)
	const LEDBitMask_t FaceLEDMask = 0;
	//! selects all of the leds
	const LEDBitMask_t AllLEDMask  = (LEDBitMask_t)~0;
	//@}

	enum InterfaceMode_t {
	};

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
		BUTTON_2,
		BUTTON_1,
		BUTTON_B,
		BUTTON_A,
		BUTTON_MINUS,
		BUTTON_HOME,
		BUTTON_LEFT,
		BUTTON_RIGHT,
		BUTTON_DOWN,
		BUTTON_UP,
		BUTTON_PLUS
	};

	//! Provides a string name for each button
	const char* const buttonNames[NumButtons+1] = 
		{ "button 2", 
			"button 1", 
			"button B", 
			"button A", 
			"button minus", 
			"button home", 
			"button left", 
			"button right", 
			"button down", 
			"button up", 
			"button pus", 
			NULL
		}; // non-empty array to avoid gcc 3.4.2 internal error

	//! holds offset to different sensor values in WorldState::sensors[]
	/*! @see WorldState::sensors[] */
	enum SensorOffset_t { 
		X_OFFSET,
		Y_OFFSET,
		Z_OFFSET
	};
	
	enum IRComm_t {
		IR_X,
		IR_Y
	};
	/*const unsigned IR_BASE_MASK=240;
	const unsigned IR_BASE_RED_MASK=8;
	const unsigned IR_BASE_GREEN_MASK=4;
	const unsigned IR_BASE_FORCE_MASK=2;*/
		
	//! Provides a string name for each sensor
	const char* const sensorNames[NumSensors+1] = { 
		"x",
		"y",
		"z",
		NULL
	}; // non-empty array to avoid gcc 3.4.2 internal error

	//@}


	//! Names for each of the outputs
	const char* const outputNames[NumReferenceFrames+1] = {
		"LED 0",
		"LED 1",
		"LED 2",
		"LED 3",
		"BaseFrame",
		NULL
	};
	
	//! allocation declared in RobotInfo.cc
	extern const Capabilities capabilities;
	
	//! This table holds the default PID values for each joint.  see PIDMC
	const float DefaultPIDs[NumPIDJoints+1][3] = {
		{0,0,0} // extra value to avoid error in older versions of gcc (doesn't like empty array
	};
	
	//!These values are our recommended maximum joint velocities, in rad/ms
	const float MaxOutputSpeed[NumOutputs] = {
		0, 0, 0, 0
	};

	//! This table holds the software limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
	const float outputRanges[NumOutputs][2] =
		{
			{ -1 , 1 },
			{ -1 , 1 },
			{ -1 , 1 },
			{ -1 , 1 },
		};

	//! This table holds the mechanical limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
	/*! Same as #outputRanges, don't know actual values because they were never specified by Sony */
	const float mechanicalLimits[NumOutputs][2] =
		{
			{ -1 , 1 },
			{ -1 , 1 },
			{ -1 , 1 },
			{ -1 , 1 },
		};

}

/*! @file
 * @brief Defines some capabilities of the WiiMote
 * @author ejt (Creator)
 */

#endif
