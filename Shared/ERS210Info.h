//-*-c++-*-
#ifndef INCLUDED_ERS210Info_h
#define INCLUDED_ERS210Info_h

#include "CommonERSInfo.h"

// see http://tekkotsu.org/porting.html#configuration for more information on TGT_HAS_* flags
#if defined(TGT_ERS210)
#	define TGT_HAS_LEDS 9
#	define TGT_HAS_BUTTONS 8
#	define TGT_HAS_IR_DISTANCE 1
#endif

//! Contains information about the ERS-210 Robot, such as number of joints, PID defaults, timing information, etc.
namespace ERS210Info {

	// *******************************
	//       ROBOT CONFIGURATION
	// *******************************

	extern const char* const TargetName; //!< the name of the model, to be used for logging and remote GUIs

	const unsigned int FrameTime=8;        //!< time between frames in the motion system (milliseconds)
	const unsigned int NumFrames=4;        //!< the number of frames per buffer (don't forget also double buffered)
	const unsigned int SlowFrameTime=128;  //!< time between frames for the ears (which move slower for some reason, don't want to mix with other outputs) (milliseconds)
	const unsigned int NumSlowFrames=1;    //!< the number of frames per buffer being sent to ears (double buffered as well)
	const unsigned int SoundBufferTime=32; //!< the number of milliseconds per sound buffer... I'm not sure if this can be changed
	
	//!Corresponds to entries in ERS210Info::PrimitiveName, defined at the end of this file, these are the primary grouping
	/*!Right now all binary joints are slow, but perhaps this won't always be the case... hence the IsFast/Slow bitmasks to select which type, in order to be more general */
	//!@name Output Types Information
	const unsigned NumWheels = 0; //!< no wheels, just legs
	
	const unsigned JointsPerArm   =  0; //!< no arms, just legs
	const unsigned NumArms        =  0; //!< no arms, just legs
	const unsigned NumArmJoints   =  JointsPerArm*NumArms;
	
	const unsigned JointsPerLeg   =  3; //!< The number of joints per leg
	const unsigned NumLegs        =  4; //!< The number of legs
	const unsigned NumLegJoints   =  JointsPerLeg*NumLegs; //!< the TOTAL number of joints on ALL legs
	const unsigned NumHeadJoints  =  3; //!< The number of joints in the neck
	const unsigned NumTailJoints  =  2; //!< The number of joints assigned to the tail
	const unsigned NumMouthJoints =  1; //!< the number of joints that control the mouth
	const unsigned NumEarJoints   =  2; //!< The number of joints which control the ears (NOT per ear, is total)
	const unsigned NumButtons     =  8; //!< the number of buttons that are available, see ERS210Info::ButtonOffset_t
	const unsigned NumSensors     =  1+3+1+5;  //!< 1 dist, 3 accel, 1 thermo, 5 from power, see ERS210Info::SensorOffset_t
	const unsigned NumLEDs        =  9; //!< The number of LEDs which can be controlled
	
	const unsigned NumPIDJoints   = NumLegJoints+NumHeadJoints+NumTailJoints+NumMouthJoints; //!< The number of joints which use PID motion - everything except ears
	const unsigned NumBinJoints   = NumEarJoints; //!< The number of binary joints - just the ears (Aperios only)
	const unsigned NumOutputs     = NumPIDJoints + NumBinJoints + NumLEDs; //!< the total number of outputs
	const unsigned NumReferenceFrames = NumOutputs + 1 + NumLegs + 1 + 1; //!< for the base, paw, camera, and IR sensor reference frames

	using namespace CameraERS2xx;
	
	const float BallOfFootRadius=27.922f/2; //!< radius of the ball of the foot
	const float CylinderOfFootRadius=24.606f/2; //!< radius of the cylinder of the foot

	//! true for joints which can be updated every 32 ms (all but the ears)
	/*! @hideinitializer */
	const bool IsFastOutput[NumOutputs] = { true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,false,false };
	//@}
	
	
	
	// *******************************
	//         OUTPUT OFFSETS
	// *******************************


	//!Corresponds to entries in ERS210Info::PrimitiveName, defined at the end of this file
	//!@name Output Offsets

	const unsigned PIDJointOffset = 0; //!< The beginning of the PID Joints
	const unsigned LegOffset   = PIDJointOffset;           //!< the offset of the beginning of the leg joints
	const unsigned HeadOffset  = LegOffset+NumLegJoints;   //!< the offset of the beginning of the head joints
	const unsigned TailOffset  = HeadOffset+NumHeadJoints; //!< the offset of the beginning of the tail joints
	const unsigned MouthOffset = TailOffset+NumTailJoints; //!< the offset of the beginning of the mouth joint

	const unsigned LEDOffset   = PIDJointOffset + NumPIDJoints; //!< the offset of LEDs in WorldState::outputs and MotionCommand functions

	const unsigned BinJointOffset = LEDOffset + NumLEDs; //!< The beginning of the binary joints
	const unsigned EarOffset   = BinJointOffset;           //!< the offset of the beginning of the ear joints - note that ears aren't sensed.  They can be flicked by the environment and you won't know.  Nor will they be flicked back

	const unsigned BaseFrameOffset   = NumOutputs; //!< Use with kinematics to refer to base reference frame
	const unsigned FootFrameOffset    = BaseFrameOffset+1; //!< Use with kinematics to refer to paw reference frames (add appropriate LegOrder_t to specify which paw)
	const unsigned PawFrameOffset    = FootFrameOffset; //!< Aibo-era alias for FootFrameOffset
	const unsigned CameraFrameOffset = FootFrameOffset+NumLegs; //!< Use with kinematics to refer to camera reference frame
	const unsigned IRFrameOffset = CameraFrameOffset+1; //!< Use with kinematics to refer to infrared (distance) sensor reference frame
	
	//! the ordering of legs
	enum LegOrder_t {
		LFrLegOrder = 0, //!< left front leg
		RFrLegOrder,     //!< right front leg
		LBkLegOrder,     //!< left back leg
		RBkLegOrder      //!< right back leg
	};
	
	//! The offsets within appendages (the legs)  Note that the ordering matches the actual physical ordering of joints on the appendage (and not that of the head's TPROffset_t's)
	enum REKOffset_t {
		RotatorOffset=0, //!< moves leg forward or backward along body
		ElevatorOffset,  //!< moves leg toward or away from body
		KneeOffset       //!< moves knee
	};
	
	//! The offsets of appendages with tilt (elevation), pan (heading), and roll or nod joints (i.e. head)  Note that the ordering matches the actual physical ordering of joints on the appendage (and not that of the leg's REKOffset_t's)
	enum TPROffset_t {
		TiltOffset = 0, //!< tilt/elevation (vertical)
		PanOffset,      //!< pan/heading (horizontal)
		RollOffset,      //!< roll (rotational)
		NodOffset=RollOffset       //!< nod (second tilt)
	};
	
	//! The offsets of the individual legs
	enum LegOffset_t {
		LFrLegOffset = LegOffset+LFrLegOrder*JointsPerLeg, //!< beginning of left front leg
		RFrLegOffset = LegOffset+RFrLegOrder*JointsPerLeg, //!< beginning of right front leg
		LBkLegOffset = LegOffset+LBkLegOrder*JointsPerLeg, //!< beginning of left back leg
		RBkLegOffset = LegOffset+RBkLegOrder*JointsPerLeg  //!< beginning of right back leg
	};
	
	//@}
	
	//! The offsets of the individual LEDs on the head and tail.  Note that left/right are robot's point of view.  See also LEDBitMask_t
	enum LEDOffset_t {
		BotLLEDOffset = LEDOffset,//!< bottom left (red - sad)
		BotRLEDOffset, //!< bottom right (red - sad)
		MidLLEDOffset, //!< middle left (green - happy)
		MidRLEDOffset, //!< middle right (green - happy)
		TopLLEDOffset, //!< top left (red - angry)
		TopRLEDOffset, //!< top right (red - angry)
		TopBrLEDOffset,//!< top bar (green)
		TlRedLEDOffset,//!< red tail light
		TlBluLEDOffset, //!< blue tail light
 
		// alias for 220 cross-compatibility
		FaceFrontLeftLEDOffset = BotLLEDOffset,   //!< alias for 220 cross-compatibility
		FaceFrontRightLEDOffset = BotRLEDOffset,  //!< alias for 220 cross-compatibility
		FaceCenterLeftLEDOffset = MidLLEDOffset,  //!< alias for 220 cross-compatibility
		FaceCenterRightLEDOffset = MidRLEDOffset, //!< alias for 220 cross-compatibility
		FaceBackLeftLEDOffset = TopLLEDOffset,    //!< alias for 220 cross-compatibility
		FaceBackRightLEDOffset = TopRLEDOffset,   //!< alias for 220 cross-compatibility
		ModeLEDOffset = TopBrLEDOffset,           //!< alias for 220 cross-compatibility
		TailRightLEDOffset = TlRedLEDOffset,      //!< alias for 220 cross-compatibility
		TailLeftLEDOffset = TlBluLEDOffset        //!< alias for 220 cross-compatibility
	};
	
	//! Bitmasks for use when specifying combinations of LEDs (see LedEngine ) Note that left/right are robot's point of view
	//!@name LED Bitmasks
	typedef unsigned int LEDBitMask_t; //!< So you can be clear when you're refering to a LED bitmask
	const LEDBitMask_t BotLLEDMask = 1<<(BotLLEDOffset-LEDOffset); //!< bottom left (red - sad)
	const LEDBitMask_t BotRLEDMask = 1<<(BotRLEDOffset-LEDOffset); //!< bottom right (red - sad)
	const LEDBitMask_t MidLLEDMask = 1<<(MidLLEDOffset-LEDOffset); //!< middle left (green - happy)
	const LEDBitMask_t MidRLEDMask = 1<<(MidRLEDOffset-LEDOffset); //!< middle right (green - happy)
	const LEDBitMask_t TopLLEDMask = 1<<(TopLLEDOffset-LEDOffset); //!< top left (red - angry)
	const LEDBitMask_t TopRLEDMask = 1<<(TopRLEDOffset-LEDOffset); //!< top right (red - angry)
	const LEDBitMask_t TopBrLEDMask= 1<<(TopBrLEDOffset-LEDOffset); //!< top bar (green)
	const LEDBitMask_t TlRedLEDMask= 1<<(TlRedLEDOffset-LEDOffset); //!< red tail light
	const LEDBitMask_t TlBluLEDMask= 1<<(TlBluLEDOffset-LEDOffset); //!< blue tail light

	//! LEDs for face (all but tail)
	const LEDBitMask_t FaceLEDMask
	= BotLLEDMask	| BotRLEDMask
	| MidLLEDMask	| MidRLEDMask
	| TopLLEDMask	| TopRLEDMask
	| TopBrLEDMask;

	//! LEDs for face (all but tail)
	const LEDBitMask_t HeadLEDMask
	= BotLLEDMask | BotRLEDMask
	| MidLLEDMask | MidRLEDMask
	| TopLLEDMask | TopRLEDMask
	| TopBrLEDMask;

	const LEDBitMask_t BackLEDMask = 0; //!< 210 has no back LEDs
	const LEDBitMask_t TailLEDMask = TlRedLEDMask|TlBluLEDMask; //!< LEDs on tail
	const LEDBitMask_t AllLEDMask  = (LEDBitMask_t)~0; //!< selects all of the leds
	//@}
	
	//! Offset needed so that the centroid of the robot is correct relative to the bounding box
	const fmat::Column<3> AgentBoundingBoxBaseFrameOffset = fmat::pack(0,0,0); 

	//! Half of the length, width, and height of the robot
	const fmat::Column<3> AgentBoundingBoxHalfDims = fmat::pack(304.8/2, 304.8/2, 0);
	
	
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
	 *  @see WorldState::buttons @see ButtonSourceID_t */
	enum ButtonOffset_t {
		LFrPawOffset = LFrLegOrder,
		RFrPawOffset = RFrLegOrder,
		LBkPawOffset = LBkLegOrder,
		RBkPawOffset = RBkLegOrder,
		ChinButOffset= 4,
		BackButOffset,
		HeadFrButOffset, //!< not in reliable pressure units, but 1.0 is fairly stiff pressure, 0 is none
		HeadButOffset=HeadFrButOffset,  //!< for ERS-7 compatibility
		HeadBkButOffset //!< not in reliable pressure units, but 1.0 is fairly stiff pressure, 0 is none
	};
	
	//! Provides a string name for each button
	const char* const buttonNames[NumButtons] = {
		"LFrPaw","RFrPaw","LBkPaw","RBkPaw",
		"ChinBut","BackBut",
		"HeadFrBut","HeadBkBut"
	};

	//! holds offset to different sensor values in WorldState::sensors[]
	/*! @see WorldState::sensors[] */
	enum SensorOffset_t {
		IRDistOffset = 0,  //!< in millimeters
		BAccelOffset, //!< backward acceleration, in @f$m/s^2@f$, negative if sitting on butt (positive for faceplant)
		LAccelOffset, //!< acceleration to the robot's left, in @f$m/s^2@f$, negative if lying on robot's left side
		DAccelOffset, //!< downward acceleration, in @f$m/s^2@f$, negative if standing up... be careful about the signs on all of these...
		ThermoOffset, //!< in degrees Celcius
		PowerRemainOffset, //!< percentage, 0-1
		PowerThermoOffset, //!<  degrees Celcius
		PowerCapacityOffset, //!< milli-amp hours
		PowerVoltageOffset, //!< volts
		PowerCurrentOffset //!< milli-amp negative values (maybe positive while charging?)
	};

	//! Provides a string name for each sensor
	const char* const sensorNames[NumSensors] = {
		"IRDist",
		"BAccel","LAccel","DAccel",
		"Thermo",
		"PowerRemain","PowerThermo","PowerCapacity","PowerVoltage","PowerCurrent"
	};

	//@}


	//! Names for each of the outputs
	const char* const outputNames[NumReferenceFrames+1] = {
		"LFr:rotor",
		"LFr:elvtr",
		"LFr:knee",
		"RFr:rotor",
		"RFr:elvtr",
		"RFr:knee",
		"LBk:rotor",
		"LBk:elvtr",
		"LBk:knee",
		"RBk:rotor",
		"RBk:elvtr",
		"RBk:knee",
		
		"NECK:tilt",
		"NECK:pan",
		"NECK:roll",
		
		"TAIL:tilt",
		"TAIL:pan",
		
		"MOUTH",
		
		"LED:botL",
		"LED:botR",
		"LED:midL",
		"LED:midR",
		"LED:topL",
		"LED:topR",
		"LED:topBr",
		
		"LED:tlRed",
		"LED:tlBlu",
		
		"EAR:left",
		"EAR:right",

		"BaseFrame",
		"LFrFootFrame",
		"RFrFootFrame",
		"LBkFootFrame",
		"RBkFootFrame",
		"CameraFrame",
		"IRFrame",
		NULL
	};
	
	
	//! provides polymorphic robot capability detection/mapping
	class ERS210Capabilities : public Capabilities {
	public:
		//! constructor
		ERS210Capabilities()
		: Capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs)
		{
			// ers-7 button alias
			buttonToIndex["HeadBut"]=HeadButOffset;
			// 220 led aliases
			frameToIndex["LED:bkL1"]=TailLeftLEDOffset;
			frameToIndex["LED:bkL2"]=TailRightLEDOffset;
		}
	};
	//! allocation declared in RobotInfo.cc
	extern const ERS210Capabilities capabilities;
	
	
	//! the joint identifier strings used to refer to specific joints in OPEN-R (but not needed for others)
	/*!@showinitializer 
	 * @warning IMPORTANT!!!!  DO NOT CHANGE THE ORDER OF ITEMS IN THIS TABLE!!!\n
	 *
	 * The offset consts defined in this file correspond to this table and will make life easier
	 * if you feel the need to reorder things, but they aren't used perfect @e everywhere \n
	 * In particular, assumptions are made that the pid joints will be in slots 0-numPIDJoints
	 * and that the fast outputs (ie NOT ears) will be in slots 0-NumFastOutputs\n
	 * There may be other assumptions not noted here!!!
	 * @note These entries DON'T correspond to the CPC index numbers defined in WorldState (this only lists joints, and in a different order defined by OPEN-R, that one has sensors as well*/
	const char* const PrimitiveName [NumOutputs] = {
		"PRM:/r2/c1-Joint2:j1",       //!< the left front leg   the rotator
		"PRM:/r2/c1/c2-Joint2:j2",    //!< the left front leg   the elevator 
		"PRM:/r2/c1/c2/c3-Joint2:j3", //!< the left front leg   the knee 
		"PRM:/r4/c1-Joint2:j1",       //!< the right front leg   the rotator
		"PRM:/r4/c1/c2-Joint2:j2",    //!< the right front leg    the elevator 
		"PRM:/r4/c1/c2/c3-Joint2:j3", //!< the right front leg   the knee 
		
		"PRM:/r3/c1-Joint2:j1",       //!< the left hind leg   the rotator
		"PRM:/r3/c1/c2-Joint2:j2",    //!< the left hind leg   the elevator 
		"PRM:/r3/c1/c2/c3-Joint2:j3", //!< the left hind leg   the knee
		"PRM:/r5/c1-Joint2:j1",       //!< the right hind leg   the rotator
		"PRM:/r5/c1/c2-Joint2:j2",    //!< the right hind leg   the elevator 
		"PRM:/r5/c1/c2/c3-Joint2:j3", //!< the right hind leg   the knee 

		"PRM:/r1/c1-Joint2:j1",       //!< the neck  tilt (12)
		"PRM:/r1/c1/c2-Joint2:j2",    //!< the neck   pan 
		"PRM:/r1/c1/c2/c3-Joint2:j3", //!< the neck   roll 
		
		"PRM:/r6/c2-Joint2:j2",       //!< the tail tilt (15)       // *** NOTE ***
		"PRM:/r6/c1-Joint2:j1",       //!< the tail pan             // CHANGE_ET 12/16/01 matches neck order
		
		"PRM:/r1/c1/c2/c3/c4-Joint2:j4", //!< the mouth (17)
		
		"PRM:/r1/c1/c2/c3/l1-LED2:l1", //!< lower  left  LED (18)
		"PRM:/r1/c1/c2/c3/l4-LED2:l4", //!< lower  right LED
		"PRM:/r1/c1/c2/c3/l2-LED2:l2", //!< middle left  LED
		"PRM:/r1/c1/c2/c3/l5-LED2:l5", //!< middle right LED
		"PRM:/r1/c1/c2/c3/l3-LED2:l3", //!< upper  left  LED
		"PRM:/r1/c1/c2/c3/l6-LED2:l6", //!< upper  right LED
		"PRM:/r1/c1/c2/c3/l7-LED2:l7", //!< top          LED
		
		"PRM:/r6/l2-LED2:l2", //!< tail red  LED
		"PRM:/r6/l1-LED2:l1", //!< tail blue LED
		
		"PRM:/r1/c1/c2/c3/e1-Joint3:j5", //!< left ear (27)
		"PRM:/r1/c1/c2/c3/e2-Joint3:j6" //!< right ear
	};

	//! use to open speaker connectio with the system
	const char* const SpeakerLocator="PRM:/r1/c1/c2/c3/s1-Speaker:S1";

	//! use to open camera connection with the system
	const char* const CameraLocator="PRM:/r1/c1/c2/c3/i1-FbkImageSensor:F1";

	//Old PID table:
	/*const word Pid[NumPIDJoints][6] = {
		{ 0x16, 0x04, 0x08, 0x0E, 0x02, 0x0F },
		{ 0x14, 0x04, 0x06, 0x0E, 0x02, 0x0F },
		{ 0x23, 0x04, 0x05, 0x0E, 0x02, 0x0F },
		{ 0x16, 0x04, 0x08, 0x0E, 0x02, 0x0F },
		{ 0x14, 0x04, 0x06, 0x0E, 0x02, 0x0F },
		{ 0x23, 0x04, 0x05, 0x0E, 0x02, 0x0F },
		{ 0x16, 0x04, 0x08, 0x0E, 0x02, 0x0F },
		{ 0x14, 0x04, 0x06, 0x0E, 0x02, 0x0F },
		{ 0x23, 0x04, 0x05, 0x0E, 0x02, 0x0F },
		{ 0x16, 0x04, 0x08, 0x0E, 0x02, 0x0F },
		{ 0x14, 0x04, 0x06, 0x0E, 0x02, 0x0F },
		{ 0x23, 0x04, 0x05, 0x0E, 0x02, 0x0F },
		
		{ 0x0A, 0x08, 0x0C, 0x0E, 0x02, 0x0F },
		{ 0x0D, 0x08, 0x0B, 0x0E, 0x02, 0x0F },
		{ 0x10, 0x08, 0x0C, 0x0E, 0x02, 0x0F }, // P was 0x0C, updated as seen on https://www.openr.org/page1_2001/gain.html 8/13/2002
		
		{ 0x0A, 0x00, 0x18, 0x0E, 0x02, 0x0F },
		{ 0x07, 0x00, 0x11, 0x0E, 0x02, 0x0F },
		
		{ 0x0E, 0x08, 0x10, 0x0E, 0x02, 0x0F }, //  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	};*/

	//! This table holds the default PID values for each joint.  see PIDMC
	const float DefaultPIDs[NumPIDJoints][3] =
		{
			{ 0x16/(float)(1<<(16-0xE)), 0x04/(float)(1<<(16-0x2)), 0x08/(float)(1<<(16-0xF)) },
			{ 0x14/(float)(1<<(16-0xE)), 0x04/(float)(1<<(16-0x2)), 0x06/(float)(1<<(16-0xF)) },
			{ 0x23/(float)(1<<(16-0xE)), 0x04/(float)(1<<(16-0x2)), 0x05/(float)(1<<(16-0xF)) },
			{ 0x16/(float)(1<<(16-0xE)), 0x04/(float)(1<<(16-0x2)), 0x08/(float)(1<<(16-0xF)) },
			{ 0x14/(float)(1<<(16-0xE)), 0x04/(float)(1<<(16-0x2)), 0x06/(float)(1<<(16-0xF)) },
			{ 0x23/(float)(1<<(16-0xE)), 0x04/(float)(1<<(16-0x2)), 0x05/(float)(1<<(16-0xF)) },
			{ 0x16/(float)(1<<(16-0xE)), 0x04/(float)(1<<(16-0x2)), 0x08/(float)(1<<(16-0xF)) },
			{ 0x14/(float)(1<<(16-0xE)), 0x04/(float)(1<<(16-0x2)), 0x06/(float)(1<<(16-0xF)) },
			{ 0x23/(float)(1<<(16-0xE)), 0x04/(float)(1<<(16-0x2)), 0x05/(float)(1<<(16-0xF)) },
			{ 0x16/(float)(1<<(16-0xE)), 0x04/(float)(1<<(16-0x2)), 0x08/(float)(1<<(16-0xF)) },
			{ 0x14/(float)(1<<(16-0xE)), 0x04/(float)(1<<(16-0x2)), 0x06/(float)(1<<(16-0xF)) },
			{ 0x23/(float)(1<<(16-0xE)), 0x04/(float)(1<<(16-0x2)), 0x05/(float)(1<<(16-0xF)) },

			{ 0x0A/(float)(1<<(16-0xE)), 0x08/(float)(1<<(16-0x2)), 0x0C/(float)(1<<(16-0xF)) },
			{ 0x0D/(float)(1<<(16-0xE)), 0x08/(float)(1<<(16-0x2)), 0x0B/(float)(1<<(16-0xF)) },
			{ 0x0A/(float)(1<<(16-0xE)), 0x08/(float)(1<<(16-0x2)), 0x0C/(float)(1<<(16-0xF)) },

			{ 0x0A/(float)(1<<(16-0xE)), 0x00/(float)(1<<(16-0x2)), 0x18/(float)(1<<(16-0xF)) },
			{ 0x07/(float)(1<<(16-0xE)), 0x00/(float)(1<<(16-0x2)), 0x11/(float)(1<<(16-0xF)) },

			{ 0x0E/(float)(1<<(16-0xE)), 0x08/(float)(1<<(16-0x2)), 0x10/(float)(1<<(16-0xF)) }
		};

	//! These will control the shift values given to the system.  see PIDMC
	const unsigned char DefaultPIDShifts[3] = {0x0E, 0x02, 0x0F};
		
	//!These values are Sony's recommended maximum joint velocities, in rad/s
	/*! a value <= 0 means infinite speed (e.g. LEDs) 
	 *  
	 *  These limits are <b>not</b> enforced by the framework.  They are simply available for you to use as you see fit.
	 *  HeadPointerMC is (as of v1.6) the only included MotionCommand to actually use these values. */
	const float MaxOutputSpeed[NumOutputs] = {
		2.8143434f,     //Legs LR,FB,REK
		2.4980025f,
		2.8361600f,
		2.8143434f,
		2.4980025f,
		2.8361600f,
		2.8143434f,
		2.4980025f,
		2.8361600f,
		2.8143434f,
		2.4980025f,
		2.8361600f,
	
		2.1053034f,     //Head TPR
		3.0106930f,
		3.0106930f,
	
		4.4724062f,     //Tail
		4.4724062f,
	
		4.3742314f,     //Mouth
		
		0,0,0,0,0,0,0,0,0, //LEDs
		
		0,0                //Ears
	};

	#ifndef RAD
		//!Just a little macro for converting degrees to radians
	#define RAD(deg) (((deg) * (float)M_PI ) / 180.0f)
		//!a flag so we undef these after we're done - do you have a cleaner solution?
	#define __RI_RAD_FLAG
	#endif
	
	//! This table holds the software limits of each of the outputs
	const float outputRanges[NumOutputs][2] =
		{
			{ RAD(-117),RAD(117) },{ RAD(-11),RAD(89) },{ RAD(-27),RAD(147) }, //left front REK
			{ RAD(-117),RAD(117) },{ RAD(-11),RAD(89) },{ RAD(-27),RAD(147) }, //right front REK
			{ RAD(-117),RAD(117) },{ RAD(-11),RAD(89) },{ RAD(-27),RAD(147) }, //left back REK
			{ RAD(-117),RAD(117) },{ RAD(-11),RAD(89) },{ RAD(-27),RAD(147) }, //right back REK

			{ RAD(-82),RAD(43) },{ RAD(-89.6f),RAD(89.6f) },{ RAD(-29),RAD(29) }, //neck TPR
				
			{ RAD(-22),RAD(22) },{ RAD(-22),RAD(22) }, // tail tp

			{ RAD(-47),RAD(-3) }, //mouth

			{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1}, //LEDs
		
			{0,1},{0,1} //ears
		};

	//! This table holds the mechanical limits of each of the outputs
	const float mechanicalLimits[NumOutputs][2] =
		{
			{ RAD(-120),RAD(120) },{ RAD(-14),RAD(92) },{ RAD(-30),RAD(150) }, //left front jsk
			{ RAD(-120),RAD(120) },{ RAD(-14),RAD(92) },{ RAD(-30),RAD(150) }, //right front jsk
			{ RAD(-120),RAD(120) },{ RAD(-14),RAD(92) },{ RAD(-30),RAD(150) }, //left back jsk
			{ RAD(-120),RAD(120) },{ RAD(-14),RAD(92) },{ RAD(-30),RAD(150) }, //right back jsk

			{ RAD(-85),RAD(46) },{ RAD(-92.6f),RAD(92.6f) },{ RAD(-32),RAD(32) }, //neck tpr
				
			{ RAD(-25),RAD(25) },{ RAD(-25),RAD(25) }, // tail tp

			{ RAD(-50),RAD(0) }, //mouth

			{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1}, //LEDs
		
			{0,1},{0,1} //ears
		};

#ifdef __RI_RAD_FLAG
#undef RAD
#undef __RI_RAD_FLAG
#endif

	/*! @name CPC IDs
	 * values defined by OPEN-R, used to interface with lower level OPEN-R code to read sensors - DOESN'T correspond to ERS210Info::PrimitiveName */
	const int CPCJointNeckTilt           =  0; // Head
	const int CPCJointNeckPan            =  1;
	const int CPCJointNeckRoll           =  2;
	const int CPCSensorHeadBackPressure  =  3;
	const int CPCSensorHeadFrontPressure =  4;
	const int CPCSensorPSD               =  5;
	const int CPCJointMouth              =  6;
	const int CPCSensorChinSwitch        =  7;
	const int CPCJointLFRotator          =  8; // Left front leg
	const int CPCJointLFElevator         =  9;
	const int CPCJointLFKnee             = 10;
	const int CPCSensorLFPaw             = 11;
	const int CPCJointLHRotator          = 12; // Left hind leg
	const int CPCJointLHElevator         = 13;
	const int CPCJointLHKnee             = 14;
	const int CPCSensorLHPaw             = 15;
	const int CPCJointRFRotator          = 16; // Right front leg
	const int CPCJointRFElevator         = 17;
	const int CPCJointRFKnee             = 18;
	const int CPCSensorRFPaw             = 19;
	const int CPCJointRHRotator          = 20; // Right hind leg
	const int CPCJointRHElevator         = 21;
	const int CPCJointRHKnee             = 22;
	const int CPCSensorRHPaw             = 23;
	const int CPCJointTailPan            = 24; // Tail
	const int CPCJointTailTilt           = 25;
	const int CPCSensorThermoSensor      = 26;
	const int CPCSensorBackSwitch        = 27;
	const int CPCSensorAccelFB           = 28; //!< Front-back; see RobotInfo::BAccelOffset 
	const int CPCSensorAccelLR           = 29; //!< Left-right; see RobotInfo::LAccelOffset 
	const int CPCSensorAccelUD           = 30; //!< Up-down; see RobotInfo::DAccelOffset 
	//@}

}
	
/*! @file
 * @brief Defines RobotInfo namespace for ERS-210 models, gives some information about the robot's capabilities, such as joint counts, offsets, names and PID values
 * @author ejt (Creator)
 */

#endif
