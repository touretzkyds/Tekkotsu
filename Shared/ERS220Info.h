//-*-c++-*-

// Mad props to Daishi MORI, the 220 master chief, for porting to the 220 ;)

#ifndef INCLUDED_ERS220Info_h
#define INCLUDED_ERS220Info_h

#include "CommonERSInfo.h"

// see http://tekkotsu.org/porting.html#configuration for more information on TGT_HAS_* flags
#if defined(TGT_ERS220)
#	define TGT_HAS_LEDS 20
#	define TGT_HAS_BUTTONS 11
#	define TGT_HAS_IR_DISTANCE 1
#endif

//! Contains information about the ERS-220 Robot, such as number of joints, PID defaults, timing information, etc.
namespace ERS220Info {

	// *******************************
	//       ROBOT CONFIGURATION
	// *******************************

	extern const char* const TargetName; //!< the name of the model, to be used for logging and remote GUIs

	const unsigned int FrameTime=8;        //!< time between frames in the motion system (milliseconds)
	const unsigned int NumFrames=4;        //!< the number of frames per buffer (don't forget also double buffered)
	const unsigned int SlowFrameTime=128;  //!< time between frames for the ears (which move slower for some reason, don't want to mix with other outputs) (milliseconds)
	const unsigned int NumSlowFrames=1;    //!< the number of frames per buffer being sent to ears (double buffered as well)
	const unsigned int SoundBufferTime=32; //!< the number of milliseconds per sound buffer... I'm not sure if this can be changed
	
	//!Corresponds to entries in ERS220Info::PrimitiveName, defined at the end of this file, these are the primary grouping
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
	const unsigned NumTailJoints  =  0; //!< The number of joints assigned to the tail
	const unsigned NumMouthJoints =  0; //!< the number of joints that control the mouth
	const unsigned NumEarJoints   =  0; //!< The number of joints which control the ears (NOT per ear, is total)
	const unsigned NumButtons     =  11; //!< the number of buttons that are available, see ERS220Info::ButtonOffset_t
	const unsigned NumSensors     =  1+3+1+5;  //!< 1 dist, 3 accel, 1 thermo, 5 from power, see ERS220Info::SensorOffset_t
	const unsigned NumLEDs        = 20; //!< The number of LEDs which can be controlled
	
	const unsigned NumPIDJoints   = NumLegJoints+NumHeadJoints+NumTailJoints+NumMouthJoints; //!< The number of joints which use PID motion - everything
	const unsigned NumBinJoints   = NumEarJoints; //!< The number of binary joints - just the ears (which the 220 doesn't have) (Aperios only)
	const unsigned NumOutputs     = NumPIDJoints + NumBinJoints + NumLEDs; //!< the total number of outputs
	const unsigned NumReferenceFrames = NumOutputs + 1 + NumLegs + 1 + 1; //!< for the base, paw, camera, and IR sensor reference frames

	using namespace CameraERS2xx;
	
	const float BallOfFootRadius=27.922f/2; //!< radius of the ball of the foot
	const float CylinderOfFootRadius=24.606f/2; //!< radius of the cylinder of the foot

	//! true for joints which can be updated every 32 ms (true for all)
	const bool IsFastOutput[NumOutputs] = {
		// for PID joints
		true, true, true,
		true, true, true,
		true, true, true,
		true, true, true,
		true, true, true,
		// for LEDs
		true, true, true,           // face left side LEDs x3
		true, true, true,           // face right side LEDs x3
		true,                       // head mode LED x1
		true, true, true,           // back left multi LEDs x3
		true, true, true,           // back right multi LEDs x3
		true, true, true,           // tail LEDs x3
		true, true, true,           // face front LEDs x3
		true,                       // retractable head light x1
		// for binary joints (none supported/exist on 220)
	};



	// *******************************
	//         OUTPUT OFFSETS
	// *******************************


	//!Corresponds to entries in ERS220Info::PrimitiveName, defined at the end of this file
	//!@name Output Offsets
	const unsigned PIDJointOffset = 0; //!< The beginning of the PID Joints
	const unsigned LegOffset   = PIDJointOffset;           //!< the offset of the beginning of the leg joints
	const unsigned HeadOffset  = LegOffset+NumLegJoints;   //!< the offset of the beginning of the head joints

	const unsigned LEDOffset   = PIDJointOffset + NumPIDJoints; //!< the offset of LEDs in WorldState::outputs and MotionCommand functions

	const unsigned BinJointOffset = NumOutputs; //!< The beginning of the binary joints

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
		FaceFrontLeftLEDOffset = LEDOffset, //!< head face side light (front left - blue)
		FaceFrontRightLEDOffset,    //!< head face side light (front right - blue)
		FaceCenterLeftLEDOffset,    //!< head face side light (center left - blue)
		FaceCenterRightLEDOffset,   //!< head face side light (center right - blue)
		FaceBackLeftLEDOffset,      //!< head face side light (back left - red)
		FaceBackRightLEDOffset,     //!< head face side light (back right - red)
		ModeLEDOffset,              //!< mode indicator (back of the head - orange)
		BackLeft1LEDOffset,         //!< back multi-indicator (left #1 - blue)
		BackLeft2LEDOffset,         //!< back multi-indicator (left #2 - blue)
		BackLeft3LEDOffset,         //!< back multi-indicator (left #3 - blue)
		BackRight3LEDOffset,        //!< back multi-indicator (right #3 - blue)
		BackRight2LEDOffset,        //!< back multi-indicator (right #2 - blue)
		BackRight1LEDOffset,        //!< back multi-indicator (right #1 - blue)
		TailLeftLEDOffset,          //!< tail light (left - blue)
		TailCenterLEDOffset,        //!< tail light (center - red)
		TailRightLEDOffset,         //!< tail light (right - blue)
		FaceFrontBLEDOffset,        //!< face front light B (blue)
		FaceFrontALEDOffset,        //!< face front light A (blue)
		FaceFrontCLEDOffset,        //!< face front light C (red)
		RetractableHeadLEDOffset,   //!< retractable head light
 
		// aliases for backward compatibility
		BotLLEDOffset = FaceFrontLeftLEDOffset,   //!< bottom left (red - sad) (ERS-210)
		BotRLEDOffset = FaceFrontRightLEDOffset,  //!< bottom right (red - sad) (ERS-210)
		MidLLEDOffset = FaceCenterLeftLEDOffset,  //!< middle left (green - happy) (ERS-210)
		MidRLEDOffset = FaceCenterRightLEDOffset, //!< middle right (green - happy) (ERS-210)
		TopLLEDOffset = FaceBackLeftLEDOffset,    //!< top left (red - angry) (ERS-210)
		TopRLEDOffset = FaceBackRightLEDOffset,   //!< top right (red - angry) (ERS-210)
		TopBrLEDOffset = ModeLEDOffset,           //!< top bar (green) (ERS-210)
		TlBluLEDOffset = TailLeftLEDOffset,      //!< blue tail light (ERS-210)
		TlRedLEDOffset = TailRightLEDOffset,      //!< red tail light (ERS-210)
	};
	
	//! Bitmasks for use when specifying combinations of LEDs (see LedEngine ) Note that left/right are robot's point of view
	//!@name LED Bitmasks
	typedef unsigned int LEDBitMask_t; //!< So you can be clear when you're refering to a LED bitmask
	const LEDBitMask_t FaceFrontLeftLEDMask   = 1<<(FaceFrontLeftLEDOffset-LEDOffset);
	const LEDBitMask_t FaceFrontRightLEDMask  = 1<<(FaceFrontRightLEDOffset-LEDOffset);
	const LEDBitMask_t FaceCenterLeftLEDMask  = 1<<(FaceCenterLeftLEDOffset-LEDOffset);
	const LEDBitMask_t FaceCenterRightLEDMask = 1<<(FaceCenterRightLEDOffset-LEDOffset);
	const LEDBitMask_t FaceBackLeftLEDMask    = 1<<(FaceBackLeftLEDOffset-LEDOffset);
	const LEDBitMask_t FaceBackRightLEDMask   = 1<<(FaceBackRightLEDOffset-LEDOffset);
	const LEDBitMask_t ModeLEDMask            = 1<<(ModeLEDOffset-LEDOffset);
	const LEDBitMask_t BackLeft1LEDMask       = 1<<(BackLeft1LEDOffset-LEDOffset);
	const LEDBitMask_t BackLeft2LEDMask       = 1<<(BackLeft2LEDOffset-LEDOffset);
	const LEDBitMask_t BackLeft3LEDMask       = 1<<(BackLeft3LEDOffset-LEDOffset);
	const LEDBitMask_t BackRight3LEDMask      = 1<<(BackRight3LEDOffset-LEDOffset);
	const LEDBitMask_t BackRight2LEDMask      = 1<<(BackRight2LEDOffset-LEDOffset);
	const LEDBitMask_t BackRight1LEDMask      = 1<<(BackRight1LEDOffset-LEDOffset);
	const LEDBitMask_t TailLeftLEDMask        = 1<<(TailLeftLEDOffset-LEDOffset);
	const LEDBitMask_t TailCenterLEDMask      = 1<<(TailCenterLEDOffset-LEDOffset);
	const LEDBitMask_t TailRightLEDMask       = 1<<(TailRightLEDOffset-LEDOffset);
	const LEDBitMask_t FaceFrontBLEDMask      = 1<<(FaceFrontBLEDOffset-LEDOffset);
	const LEDBitMask_t FaceFrontALEDMask      = 1<<(FaceFrontALEDOffset-LEDOffset);
	const LEDBitMask_t FaceFrontCLEDMask      = 1<<(FaceFrontCLEDOffset-LEDOffset);
	const LEDBitMask_t RetractableHeadLEDMask = 1<<(RetractableHeadLEDOffset-LEDOffset);
	
	// aliases for backward compatibility
	const LEDBitMask_t BotLLEDMask = 1<<(BotLLEDOffset-LEDOffset); //!< bottom left (red - sad)
	const LEDBitMask_t BotRLEDMask = 1<<(BotRLEDOffset-LEDOffset); //!< bottom right (red - sad)
	const LEDBitMask_t MidLLEDMask = 1<<(MidLLEDOffset-LEDOffset); //!< middle left (green - happy)
	const LEDBitMask_t MidRLEDMask = 1<<(MidRLEDOffset-LEDOffset); //!< middle right (green - happy)
	const LEDBitMask_t TopLLEDMask = 1<<(TopLLEDOffset-LEDOffset); //!< top left (red - angry)
	const LEDBitMask_t TopRLEDMask = 1<<(TopRLEDOffset-LEDOffset); //!< top right (red - angry)
	const LEDBitMask_t TopBrLEDMask= 1<<(TopBrLEDOffset-LEDOffset); //!< top bar (green)
	const LEDBitMask_t TlRedLEDMask= 1<<(TlRedLEDOffset-LEDOffset); //!< red tail light
	const LEDBitMask_t TlBluLEDMask= 1<<(TlBluLEDOffset-LEDOffset); //!< blue tail light

	//! LEDs for face
	const LEDBitMask_t FaceLEDMask
	= FaceFrontLeftLEDMask
	| FaceFrontRightLEDMask
	| FaceCenterLeftLEDMask
	| FaceCenterRightLEDMask
	| FaceBackLeftLEDMask
	| FaceBackRightLEDMask
	| FaceFrontALEDMask
	| FaceFrontBLEDMask
	| FaceFrontCLEDMask
	| ModeLEDMask;
 
	//! LEDs on head (face plus retractable light)
	const LEDBitMask_t HeadLEDMask
	= FaceLEDMask | RetractableHeadLEDMask;   
 
	//! LEDs on back
	const LEDBitMask_t BackLEDMask
	= BackLeft1LEDMask
	| BackLeft2LEDMask
	| BackLeft3LEDMask
	| BackRight1LEDMask
	| BackRight2LEDMask
	| BackRight3LEDMask;
 
	//! LEDs for tail
	const LEDBitMask_t TailLEDMask
	= TailLeftLEDMask
	| TailCenterLEDMask
	| TailRightLEDMask;
 
	//! selects all of the leds
	const LEDBitMask_t AllLEDMask  = (LEDBitMask_t)~0;
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
		HeadFrButOffset, //!< for the "antenna" - this is <.2 if pushed back all the way
		HeadButOffset=HeadFrButOffset,  //!< for ERS-7 compatibility
		HeadBkButOffset, //!< for the "antenna" - this is >.98 if pulled forward, <.2 if pushed back partly
		TailLeftButOffset,
		TailCenterButOffset,
		TailRightButOffset,
	};

	//! Provides a string name for each button
	const char* const buttonNames[NumButtons] = {
		"LFrPaw","RFrPaw","LBkPaw","RBkPaw",
		"ChinBut","BackBut","HeadFrBut","HeadBkBut",
		"TailLeftBut","TailCenterBut","TailRightBut"
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
		
		"LED:botL",
		"LED:botR",
		"LED:midL",
		"LED:midR",
		"LED:topL",
		"LED:topR",
		"LED:topBr",
		
		"LED:bkL1",                // "LED:tlBlu" of ERS-210
		"LED:bkL2",                // "LED:tlRed" of ERS-210
		"LED:bkL3",
		"LED:bkR3",
		"LED:bkR2",
		"LED:bkR1",
		"LED:tailL",
		"LED:tailC",
		"LED:tailR",
		"LED:faceB",
		"LED:faceA",
		"LED:faceC",
		"LED:light",                 // retractable head light
		
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
	class ERS220Capabilities : public Capabilities {
	public:
		//! constructor
		ERS220Capabilities()
		: Capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs)
		{
			// ers-7 button alias
			buttonToIndex["HeadBut"]=HeadButOffset;
			// 220 led aliases
			frameToIndex["LED:tlBlu"]=TlBluLEDOffset;
			frameToIndex["LED:tlRed"]=TlRedLEDOffset;
		}
	};
	//! allocation declared in RobotInfo.cc
	extern const ERS220Capabilities capabilities;
	
	
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
				
		"PRM:/r1/c1/c2/c3/l1-LED2:l1", //!< lower  left  LED (15)
		"PRM:/r1/c1/c2/c3/l4-LED2:l4", //!< lower  right LED
		"PRM:/r1/c1/c2/c3/l2-LED2:l2", //!< middle left  LED
		"PRM:/r1/c1/c2/c3/l5-LED2:l5", //!< middle right LED
		"PRM:/r1/c1/c2/c3/l3-LED2:l3", //!< upper  left  LED
		"PRM:/r1/c1/c2/c3/l6-LED2:l6", //!< upper  right LED
		"PRM:/r1/c1/c2/c3/l7-LED2:l7", //!< top          LED
		
		"PRM:/r6/l1-LED2:l1", //!< back 1st left LED (corresponds to tail blue LED of ERS-210)
		"PRM:/r6/l2-LED2:l2", //!< back 2nd left LED (corresponds to tail red  LED of ERS-210)
		"PRM:/r6/l3-LED2:l3", //!< back 3rd left LED
		"PRM:/r6/l4-LED2:l4", //!< back 3rd right LED
		"PRM:/r6/l5-LED2:l5", //!< back 2nd right LED
		"PRM:/r6/l6-LED2:l6", //!< back 1st right LED
 
		"PRM:/r6/l9-LED2:l9", //!< tail left LED
		"PRM:/r6/l7-LED2:l7", //!< tail center LED
		"PRM:/r6/l8-LED2:l8", //!< tail right LED
 
		"PRM:/r1/c1/c2/c3/l8-LED2:l8", //!< face front LED B
		"PRM:/r1/c1/c2/c3/l9-LED2:l9", //!< face front LED A
		"PRM:/r1/c1/c2/c3/la-LED2:la", //!< face front LED C
		"PRM:/r1/c1/c2/c3/lb-LED2:lb", //!< retractable head light
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
			{ 0x0A/(float)(1<<(16-0xE)), 0x08/(float)(1<<(16-0x2)), 0x0C/(float)(1<<(16-0xF)) }

// 			{ 0x0A/(float)(1<<(16-0xE)), 0x00/(float)(1<<(16-0x2)), 0x18/(float)(1<<(16-0xF)) },
// 			{ 0x07/(float)(1<<(16-0xE)), 0x00/(float)(1<<(16-0x2)), 0x11/(float)(1<<(16-0xF)) },

// 			{ 0x0E/(float)(1<<(16-0xE)), 0x08/(float)(1<<(16-0x2)), 0x10/(float)(1<<(16-0xF)) }
		};
	
	//! These will control the shift values given to the system.  see PIDMC
	const unsigned char DefaultPIDShifts[3] = {0x0E, 0x02, 0x0F};
		
	//!These values are Sony's recommended maximum joint velocities, in rad/sec
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
	
		0,0,0,//LEDs
		0,0,0,
		0,
		0,0,0, 
		0,0,0, 
		0,0,0, 
		0,0,0, 
		0
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

			{ RAD(-88.5f),RAD(43) },{ RAD(-89.6f),RAD(89.6f) },{ RAD(-29),RAD(29) }, //neck TPR
				
			{0,1},{0,1},{0,1},        // face left side LEDs x3
			{0,1},{0,1},{0,1},        // face right side LEDs x3
			{0,1},                    // head mode LED x1
			{0,1},{0,1},{0,1},        // back left multi LEDs x3
			{0,1},{0,1},{0,1},        // back right multi LEDs x3
			{0,1},{0,1},{0,1},        // tail LEDs x3
			{0,1},{0,1},{0,1},        // face front LEDs x3
			{0,1}                     // retractable head light x1
		};

	//! This table holds the mechanical limits of each of the outputs
	const float mechanicalLimits[NumOutputs][2] =
		{
			{ RAD(-120),RAD(120) },{ RAD(-14),RAD(92) },{ RAD(-30),RAD(150) }, //left front jsk
			{ RAD(-120),RAD(120) },{ RAD(-14),RAD(92) },{ RAD(-30),RAD(150) }, //right front jsk
			{ RAD(-120),RAD(120) },{ RAD(-14),RAD(92) },{ RAD(-30),RAD(150) }, //left back jsk
			{ RAD(-120),RAD(120) },{ RAD(-14),RAD(92) },{ RAD(-30),RAD(150) }, //right back jsk

			{ RAD(-91.5f),RAD(46) },{ RAD(-92.6f),RAD(92.6f) },{ RAD(-32),RAD(32) }, //neck tpr
				
			{0,1},{0,1},{0,1},        // face left side LEDs x3
			{0,1},{0,1},{0,1},        // face right side LEDs x3
			{0,1},                    // head mode LED x1
			{0,1},{0,1},{0,1},        // back left multi LEDs x3
			{0,1},{0,1},{0,1},        // back right multi LEDs x3
			{0,1},{0,1},{0,1},        // tail LEDs x3
			{0,1},{0,1},{0,1},        // face front LEDs x3
			{0,1}                     // retractable head light x1
		};

#ifdef __RI_RAD_FLAG
#undef RAD
#undef __RI_RAD_FLAG
#endif

	/*! @name CPC IDs
	 * values defined by OPEN-R, used to interface with lower level OPEN-R code to read sensors - DOESN'T correspond to ERS220Info::PrimitiveName */
    const int CPCJointNeckTilt           =  0; // PRM:/r1/c1-Joint2:j1
    const int CPCJointNeckPan            =  1; // PRM:/r1/c1/c2-Joint2:j2
    const int CPCJointNeckRoll           =  2; // PRM:/r1/c1/c2/c3-Joint2:j3
    const int CPCSensorPSD               =  3; // PRM:/r1/c1/c2/c3/p1-Sensor:p1
    const int CPCSensorHeadBackPressure  =  4; // PRM:/r1/c1/c2/c3/f1-Sensor:f1
    const int CPCSensorHeadFrontPressure =  5; // PRM:/r1/c1/c2/c3/f2-Sensor:f2
    const int CPCSensorChinSwitch        =  6; // PRM:/r1/c1/c2/c3/c4/s5-Sensor:s5
    const int CPCJointLFRotator          =  7; // PRM:/r2/c1-Joint2:j1
    const int CPCJointLFElevator         =  8; // PRM:/r2/c1/c2-Joint2:j2
    const int CPCJointLFKnee             =  9; // PRM:/r2/c1/c2/c3-Joint2:j3
    const int CPCSensorLFPaw             = 10; // PRM:/r2/c1/c2/c3/c4-Sensor:s4
    const int CPCJointLHRotator          = 11; // PRM:/r3/c1-Joint2:j1
    const int CPCJointLHElevator         = 12; // PRM:/r3/c1/c2-Joint2:j2
    const int CPCJointLHKnee             = 13; // PRM:/r3/c1/c2/c3-Joint2:j3
    const int CPCSensorLHPaw             = 14; // PRM:/r3/c1/c2/c3/c4-Sensor:s4
    const int CPCJointRFRotator          = 15; // PRM:/r4/c1-Joint2:j1
    const int CPCJointRFElevator         = 16; // PRM:/r4/c1/c2-Joint2:j2
    const int CPCJointRFKnee             = 17; // PRM:/r4/c1/c2/c3-Joint2:j3
    const int CPCSensorRFPaw             = 18; // PRM:/r4/c1/c2/c3/c4-Sensor:s4
    const int CPCJointRHRotator          = 19; // PRM:/r5/c1-Joint2:j1
    const int CPCJointRHElevator         = 20; // PRM:/r5/c1/c2-Joint2:j2
    const int CPCJointRHKnee             = 21; // PRM:/r5/c1/c2/c3-Joint2:j3
    const int CPCSensorRHPaw             = 22; // PRM:/r5/c1/c2/c3/c4-Sensor:s4
    const int CPCSensorThermoSensor      = 23; // PRM:/r6/t1-Sensor:t1
    const int CPCSensorBackSwitch        = 24; // PRM:/r6/s1-Sensor:s1
    const int CPCSensorTailLeftSwitch    = 25; // PRM:/r6/s2-Sensor:s2  (ERS-220 only)
    const int CPCSensorTailCenterSwitch  = 26; // PRM:/r6/s3-Sensor:s3  (ERS-220 only)
    const int CPCSensorTailRightSwitch   = 27; // PRM:/r6/s4-Sensor:s4  (ERS-220 only)
    const int CPCSensorAccelFB           = 28; // PRM:/a1-Sensor:a1
    const int CPCSensorAccelLR           = 29; // PRM:/a2-Sensor:a2
    const int CPCSensorAccelUD           = 30; // PRM:/a3-Sensor:a3
	//@}

}

/*! @file
 * @brief Defines RobotInfo namespace for ERS-220 models, gives some information about the robot's capabilities, such as joint counts, offsets, names and PID values
 * @author Daishi MORI (Creator)
 *
 * Mad props to Daishi MORI, the 220 master chief, for porting to the 220 ;)
 */

#endif
