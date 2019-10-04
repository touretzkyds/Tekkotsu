//-*-c++-*-
#ifndef INCLUDED_ERS7Info_h
#define INCLUDED_ERS7Info_h

#include "Shared/CommonERSInfo.h"

// see http://tekkotsu.org/porting.html#configuration for more information on TGT_HAS_* flags
#if defined(TGT_ERS7)
#	define TGT_HAS_LEDS 27
#	define TGT_HAS_BUTTONS 10
#	define TGT_HAS_IR_DISTANCE 3
#endif

//! Contains information about the ERS-7 Robot, such as number of joints, PID defaults, timing information, etc.
/*! 
You may be particularly interested in the "Output Offsets" section, which, along with the offsets of the common RobotInfo namespace,
allows you to reference any specific joint or LED on the robot.

The "Input Offsets" section gives the index order of the buttons (#ButtonOffset_t) and sensors (#SensorOffset_t), as well as
string names for each for easier debugging (#buttonNames, #sensorNames)

"Output Types" section provides "meta-information" regarding the capabilities of the robot, such as the number of head joints, or the number of LEDs, etc.

For more information on your robot's specifications, see also #DefaultPIDs, #MaxOutputSpeed, #outputRanges, and #mechanicalLimits.

"Outputs" (i.e. Joints, LEDs) are often refered to by index ("offset") value within an array.
These values are formed by specifying a @e section offset, plus a @e specific offset.  Sections are typically general across robot models, whereas the specifics are model-dependent (but can be aliased to provide compatability).

For most joints, the positive direction is "up", and the 0 position yields a forward looking, fully extended standing posture.

- {L,R}{FrBk}LegOffset - #NumLegs combinations, each with #JointsPerLeg items, add REKOffset_t; see also LegOffset_t
  - + #RotatorOffset: positive moves "out", away from body
  - + #ElevatorOffset: positive moves up, away from body
  - + #KneeOffset: positive bends knee (slight negative possible)
- #HeadOffset - #NumHeadJoints items, add #TPROffset_t:
  - + #TiltOffset: positive looks up
  - + #PanOffset: positive looks left
  - + #NodOffset: positive looks up
- #TailOffset - #NumTailJoints items, add #TPROffset_t:
  - + #TiltOffset: positive raises the joint (lowers the tail itself)
  - + #PanOffset: positive points the tail to the Aibo's right
- MouthOffset - #NumMouthJoints items (no specific, only 1 joint)
- LEDs: #NumLEDs items, see #LEDOffset_t; these are all direct offsets, and do not need to be added to anything else
- #EarOffset - #NumEarJoints items (no specifics, first is left ear, second is right ear)

It happens that these joints can also be grouped by the @e type of joint, so there are additionally a few other offsets that can be used in order to loop across a group of joints:
- PIDJointOffset - #NumPIDJoints items, servos using PID control
- LegOffset - #NumLegJoints items, a subset of PID servos corresponding to the leg joints
- LEDOffset - #NumLEDs items
- BinJointOffset - #NumBinJoints items, solenoids, such as the ears (if any) which flip between two positions
- #NumOutputs - total number of outputs available

LEDs are often handled in groups to display patterns.  Some functions take an LEDBitMask_t
parameter, which allows you to specify a set of several LEDs in a single parameter.  
For any given LED offset @c fooLEDOffset, the corresponding bitmask constant is @c fooLEDMask.
Alternatively, you could calculate the bitmask of @c fooLEDOffset by <code>1&lt;&lt;(fooLEDOffset-LEDOffset)</code>.

@see <a href="../media/TekkotsuQuickReference_ERS7.pdf">ERS-7 Quick Reference Sheet</a>
*/
namespace ERS7Info {

	// *******************************
	//       ROBOT CONFIGURATION
	// *******************************

	extern const char* const TargetName; //!< the name of the model, to be used for logging and remote GUIs

	const unsigned int FrameTime=8;        //!< time between frames in the motion system (milliseconds)
	const unsigned int NumFrames=4;        //!< the number of frames per buffer (don't forget also double buffered)
	const unsigned int SlowFrameTime=8;  //!< time between frames for the ears (ERS-7 doesn't seem to have any "slow" joints; this only applied for the ears on the ERS-210)
	const unsigned int NumSlowFrames=4;    //!< the number of frames per buffer being sent to ears (double buffered as well)
	const unsigned int SoundBufferTime=32; //!< the number of milliseconds per sound buffer... I'm not sure if this can be changed
	
	//!Corresponds to entries in ERS7Info::PrimitiveName, defined at the end of this file, these are the primary grouping
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
	const unsigned NumButtons     =  2+4+3+1; //!< the number of buttons that are available, 2 head, 4 paws, 3 back, 1 underbelly see ERS7Info::ButtonOffset_t
	const unsigned NumSensors     =  3+3+5;  //!< 3 IR (distance), 3 accel (force), 5 from power, see ERS7Info::SensorOffset_t
	const unsigned NumLEDs        =  27; //!< The number of LEDs which can be controlled
	const unsigned NumFacePanelLEDs = 14; //!< The number of face panel LEDs
	
	const unsigned NumPIDJoints   = NumLegJoints+NumHeadJoints+NumTailJoints+NumMouthJoints; //!< The number of joints which use PID motion - everything except ears
	const unsigned NumBinJoints   = NumEarJoints; //!< The number of binary joints - just the ears
	const unsigned NumOutputs     = NumPIDJoints + NumBinJoints + NumLEDs; //!< the total number of outputs
	const unsigned NumReferenceFrames = NumOutputs + 1 + NumLegs + 1 + 3; //!< for the base, paws (NumLegs), camera, and IR sensors (3) reference frames

	const float CameraHorizFOV=56.9f/180*static_cast<float>(M_PI); //!< horizontal field of view (radians)
	const float CameraVertFOV=45.2f/180*static_cast<float>(M_PI); //!< vertical field of view (radians)
	const float CameraFOV=CameraHorizFOV; //!< should be set to maximum of #CameraHorizFOV or #CameraVertFOV
	const unsigned int CameraResolutionX=208; //!< the number of pixels available in the 'full' layer
	const unsigned int CameraResolutionY=160; //!< the number of pixels available in the 'full' layer
	extern const char CameraModelName[]; //!< specifies a name of the camera to load calibration parameters into RobotInfo::CameraHomography
	
	const float BallOfFootRadius=23.433f/2; //!< radius of the ball of the foot

	//! true for joints which can be updated every 32 ms (all joints on ERS-7)
	/*! @hideinitializer */
	const bool IsFastOutput[NumOutputs] = {
		true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true, //PID joints
		true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true, //leds
		true,true //ears
	};
	//@}
	
	
	
	// *******************************
	//         OUTPUT OFFSETS
	// *******************************


	//!Corresponds to entries in ERS7Info::PrimitiveName, defined at the end of this file
	//!@name Output Offsets

	const unsigned PIDJointOffset = 0; //!< The beginning of the PID Joints
	const unsigned LegOffset   = PIDJointOffset;           //!< the offset of the beginning of the leg joints, #NumLegs of #JointsPerLeg each, in #LegOrder_t order; see #LegOffset_t
	const unsigned HeadOffset  = LegOffset+NumLegJoints;   //!< the offset of the beginning of the head joints, add #TPROffset_t to get specific joint
	const unsigned TailOffset  = HeadOffset+NumHeadJoints; //!< the offset of the beginning of the tail joints, add #TPROffset_t to get specific joint (except RollOffset not available)
	const unsigned MouthOffset = TailOffset+NumTailJoints; //!< the offset of the beginning of the mouth joint, is specific joint

	const unsigned LEDOffset   = PIDJointOffset + NumPIDJoints; //!< the offset of LEDs in WorldState::outputs and MotionCommand functions, see LedOffset_t for specific offsets

	const unsigned BinJointOffset = LEDOffset + NumLEDs;   //!< The beginning of the binary joints
	const unsigned EarOffset   = BinJointOffset;           //!< the offset of the beginning of the ear joints - note that ears aren't sensed.  They can be flicked by the environment and you won't know.

	const unsigned BaseFrameOffset   = NumOutputs; //!< Use with kinematics to refer to base reference frame
	const unsigned FootFrameOffset    = BaseFrameOffset+1; //!< Use with kinematics to refer to paw reference frames (add appropriate LegOrder_t to specify which paw)
	const unsigned PawFrameOffset    = FootFrameOffset; //!< Aibo-era alias for FootFrameOffset
	const unsigned CameraFrameOffset = FootFrameOffset+NumLegs; //!< Use with kinematics to refer to camera reference frame
	const unsigned NearIRFrameOffset = CameraFrameOffset+1; //!< Use with kinematics to refer to short-range infrared (distance) sensor reference frame
	const unsigned IRFrameOffset = NearIRFrameOffset; //!< alias for the near IR sensor
	const unsigned FarIRFrameOffset = NearIRFrameOffset+1; //!< Use with kinematics to refer to long-range infrared (distance) sensor reference frame
	const unsigned ChestIRFrameOffset = FarIRFrameOffset+1; //!< Use with kinematics to refer to chest-mounted infrared (distance) sensor reference frame

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
	
	//! The offsets of the individual legs, add #REKOffset_t value to access specific joint
	/*! @hideinitializer */
	enum LegOffset_t {
		LFrLegOffset = LegOffset+LFrLegOrder*JointsPerLeg, //!< beginning of left front leg's joints
		RFrLegOffset = LegOffset+RFrLegOrder*JointsPerLeg, //!< beginning of right front leg's joints
		LBkLegOffset = LegOffset+LBkLegOrder*JointsPerLeg, //!< beginning of left back leg's joints
		RBkLegOffset = LegOffset+RBkLegOrder*JointsPerLeg  //!< beginning of right back leg's joints
	};
	
	//! The offsets of the individual LEDs on the head and tail.  Note that left/right are robot's point of view.  See also LEDBitMask_t
	/*! @hideinitializer */
	enum LEDOffset_t {
		HeadColorLEDOffset = LEDOffset, //!< the orange LED immediately above the head button (copositioned with #HeadWhiteLEDOffset)
		HeadWhiteLEDOffset, //!< the LED immediately above the head button (copositioned with #HeadColorLEDOffset)
		ModeRedLEDOffset, //!< controls both of the red LEDs above the ears (copositioned with #ModeGreenLEDOffset and #ModeBlueLEDOffset)
		ModeGreenLEDOffset, //!< controls both of the green LEDs above the ears (copositioned with #ModeRedLEDOffset and #ModeBlueLEDOffset)
		ModeBlueLEDOffset, //!< controls both of the blue LEDs above the ears (copositioned with #ModeRedLEDOffset and #ModeGreenLEDOffset)
		WirelessLEDOffset, //!< controls the small rectangular blue LED on the back of the head
		FaceLEDPanelOffset, //!< the first LED in the panel - add 0 up to (not including) #NumFacePanelLEDs to this to access specific face panel LEDs, see LedEngine for diagram of placement
		FrBackColorLEDOffset = FaceLEDPanelOffset+NumFacePanelLEDs, //!< @b blue/purple LED on back, closest to head (copositioned with #FrBackWhiteLEDOffset)
		FrBackWhiteLEDOffset,  //!< white LED on back, closest to head (copositioned with #FrBackColorLEDOffset)
		MdBackColorLEDOffset, //!< @b orange LED on back, in between FrBackColorLEDOffset and RrBackColorLEDOffset (copositioned with #MdBackWhiteLEDOffset)
		MdBackWhiteLEDOffset, //!< white LED on back, in between FrBackWhiteLEDOffset and RrBackWhiteLEDOffset (copositioned with #MdBackColorLEDOffset)
		RrBackColorLEDOffset, //!< @b red LED on back, farthest from head (copositioned with #RrBackWhiteLEDOffset)
		RrBackWhiteLEDOffset, //!< white LED on back, farthest from head (copositioned with #RrBackColorLEDOffset)
		LEDABModeOffset, // allows you to control A/B mode setting (this is a "virtual" LED)

		// aliases for 2xx cross-compatibility
		BotLLEDOffset = FaceLEDPanelOffset+1,//!< aliases for backward compatability with ERS-210 (use mode A); bottom left of face panel
		BotRLEDOffset = FaceLEDPanelOffset+0, //!< aliases for backward compatability with ERS-210 (use mode A); bottom right of face panel
		MidLLEDOffset = FaceLEDPanelOffset+3, //!< aliases for backward compatability with ERS-210 (use mode A); middle left of face panel
		MidRLEDOffset = FaceLEDPanelOffset+2, //!< aliases for backward compatability with ERS-210 (use mode A); middle right of face panel
		TopLLEDOffset = FaceLEDPanelOffset+7, //!< aliases for backward compatability with ERS-210 (use mode A); top left of face panel
		TopRLEDOffset = FaceLEDPanelOffset+6, //!< aliases for backward compatability  with ERS-210(use mode A); top right of face panel
		TopBrLEDOffset= HeadColorLEDOffset,//!< aliases for backward compatability with ERS-210 (use mode A); top bar (#HeadColorLEDOffset)
		TlRedLEDOffset= RrBackColorLEDOffset,//!< aliases for backward compatability with ERS-210; red tail light (#RrBackColorLEDOffset)
		TlBluLEDOffset= FrBackColorLEDOffset //!< aliases for backward compatability with ERS-210; blue tail light (#FrBackColorLEDOffset)
	};
	//@}
		
	//! Bitmasks for use when specifying combinations of LEDs (see LedEngine ) Note that left/right are robot's point of view
	//!@name LED Bitmasks
	typedef unsigned int LEDBitMask_t; //!< So you can be clear when you're refering to a LED bitmask
	
	const LEDBitMask_t HeadColorLEDMask = 1<<(HeadColorLEDOffset-LEDOffset); //!< mask corresponding to HeadColorLEDOffset
	const LEDBitMask_t HeadWhiteLEDMask = 1<<(HeadWhiteLEDOffset-LEDOffset); //!< mask corresponding to HeadWhiteLEDOffset
	const LEDBitMask_t ModeRedLEDMask = 1<<(ModeRedLEDOffset-LEDOffset); //!< mask corresponding to ModeRedLEDOffset
	const LEDBitMask_t ModeGreenLEDMask = 1<<(ModeGreenLEDOffset-LEDOffset); //!< mask corresponding to ModeGreenLEDOffset
	const LEDBitMask_t ModeBlueLEDMask = 1<<(ModeBlueLEDOffset-LEDOffset); //!< mask corresponding to ModeBlueLEDOffset
	const LEDBitMask_t WirelessLEDMask = 1<<(WirelessLEDOffset-LEDOffset); //!< mask corresponding to WirelessLEDOffset
	const LEDBitMask_t FaceLEDPanelMask = 1<<(FaceLEDPanelOffset-LEDOffset); //!< mask corresponding to FaceLEDPanelOffset, selects only the first of the panel - shift this to get the others
	const LEDBitMask_t FrBackColorLEDMask = 1<<(FrBackColorLEDOffset-LEDOffset); //!< mask corresponding to FrBackColorLEDOffset
	const LEDBitMask_t FrBackWhiteLEDMask = 1<<(FrBackWhiteLEDOffset-LEDOffset); //!< mask corresponding to FrBackWhiteLEDOffset
	const LEDBitMask_t MdBackColorLEDMask = 1<<(MdBackColorLEDOffset-LEDOffset); //!< mask corresponding to MdBackColorLEDOffset
	const LEDBitMask_t MdBackWhiteLEDMask = 1<<(MdBackWhiteLEDOffset-LEDOffset); //!< mask corresponding to MdBackWhiteLEDOffset
	const LEDBitMask_t RrBackColorLEDMask = 1<<(RrBackColorLEDOffset-LEDOffset); //!< mask corresponding to RrBackColorLEDOffset
	const LEDBitMask_t RrBackWhiteLEDMask = 1<<(RrBackWhiteLEDOffset-LEDOffset); //!< mask corresponding to RrBackWhiteLEDOffset
	const LEDBitMask_t LEDABModeMask = 1<<(LEDABModeOffset-LEDOffset); //!< mask corresponding to LEDABModeOffset

	const LEDBitMask_t BotLLEDMask = 1<<(BotLLEDOffset-LEDOffset); //!< bottom left
	const LEDBitMask_t BotRLEDMask = 1<<(BotRLEDOffset-LEDOffset); //!< bottom right
	const LEDBitMask_t MidLLEDMask = 1<<(MidLLEDOffset-LEDOffset); //!< middle left
	const LEDBitMask_t MidRLEDMask = 1<<(MidRLEDOffset-LEDOffset); //!< middle right
	const LEDBitMask_t TopLLEDMask = 1<<(TopLLEDOffset-LEDOffset); //!< top left
	const LEDBitMask_t TopRLEDMask = 1<<(TopRLEDOffset-LEDOffset); //!< top right
	const LEDBitMask_t TopBrLEDMask= 1<<(TopBrLEDOffset-LEDOffset); //!< top bar
	const LEDBitMask_t TlRedLEDMask= 1<<(TlRedLEDOffset-LEDOffset); //!< red tail light
	const LEDBitMask_t TlBluLEDMask= 1<<(TlBluLEDOffset-LEDOffset); //!< blue tail light

	//! LEDs for the face panel (all FaceLEDPanelMask<<(0:NumFacePanelLEDs-1) entries)
	/*! @hideinitializer */
	const LEDBitMask_t FaceLEDMask = (FaceLEDPanelMask<<0) | (FaceLEDPanelMask<<1) | (FaceLEDPanelMask<<2) | (FaceLEDPanelMask<<3) | (FaceLEDPanelMask<<4) | (FaceLEDPanelMask<<5) | (FaceLEDPanelMask<<6) | (FaceLEDPanelMask<<7) | (FaceLEDPanelMask<<8) | (FaceLEDPanelMask<<9) | (FaceLEDPanelMask<<10) | (FaceLEDPanelMask<<11) | (FaceLEDPanelMask<<12) | (FaceLEDPanelMask<<13);

	//! LEDs for face (all but back lights)
	const LEDBitMask_t HeadLEDMask
	= FaceLEDMask | HeadColorLEDMask | HeadWhiteLEDMask
	| ModeRedLEDMask | ModeGreenLEDMask | ModeBlueLEDMask
	| WirelessLEDMask;

	//! LEDS on the back
	const LEDBitMask_t BackLEDMask
	= FrBackColorLEDMask | FrBackWhiteLEDMask
	| MdBackColorLEDMask | MdBackWhiteLEDMask
	| RrBackColorLEDMask | RrBackWhiteLEDMask;

	//! LEDs on tail (ERS-7 has none)
	const LEDBitMask_t TailLEDMask = 0;

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
	 *  @see WorldState::buttons @see ButtonSourceID_t
	 * @hideinitializer */
	enum ButtonOffset_t {
		LFrPawOffset = LFrLegOrder,
		RFrPawOffset = RFrLegOrder,
		LBkPawOffset = LBkLegOrder,
		RBkPawOffset = RBkLegOrder,
		ChinButOffset= 4,
		HeadButOffset,
		HeadFrButOffset=HeadButOffset,
		FrontBackButOffset,
		MiddleBackButOffset,
		BackButOffset = MiddleBackButOffset,
		RearBackButOffset,
		WirelessSwOffset
	};

	//! Provides a string name for each button
	const char* const buttonNames[NumButtons] = {
		"LFrPaw","RFrPaw","LBkPaw","RBkPaw",
		"ChinBut","HeadBut",
		"FrontBackBut","MiddleBackBut","RearBackBut",
		"WirelessSw"
	};

	//! holds offset to different sensor values in WorldState::sensors[]
	/*! @see WorldState::sensors[] */
	enum SensorOffset_t {
		NearIRDistOffset = 0,  //!< in millimeters, ranges from 50 to 500
		IRDistOffset=NearIRDistOffset, //!< alias for ERS-2xx's solitary range finder
		FarIRDistOffset,  //!< in millimeters, ranges from 200 to 1500
		ChestIRDistOffset,  //!< in millimeters, ranges from 100 to 900
		BAccelOffset, //!< backward acceleration, in @f$m/s^2@f$, negative if sitting on butt (positive for faceplant)
		LAccelOffset, //!< acceleration to the robot's left, in @f$m/s^2@f$, negative if lying on robot's left side
		DAccelOffset, //!< downward acceleration, in @f$m/s^2@f$, negative if standing up... be careful about the signs on all of these...
		PowerRemainOffset, //!< percentage, 0-1
		PowerThermoOffset, //!<  degrees Celcius
		PowerCapacityOffset, //!< milli-amp hours
		PowerVoltageOffset, //!< volts
		PowerCurrentOffset //!< milli-amp negative values (maybe positive while charging?)
	};

	//! Provides a string name for each sensor
	const char* const sensorNames[NumSensors] = {
		"NearIRDist","FarIRDist","ChestIRDist",
		"BAccel","LAccel","DAccel",
		"PowerRemain","PowerThermo","PowerCapacity","PowerVoltage","PowerCurrent"
	};

	//@}


	//! Names for each of the outputs
	const char* const outputNames[NumReferenceFrames+1] = {
		"LFr:rotor","LFr:elvtr","LFr:knee",
		"RFr:rotor","RFr:elvtr","RFr:knee",
		"LBk:rotor","LBk:elvtr","LBk:knee",
		"RBk:rotor","RBk:elvtr","RBk:knee",
		
		"NECK:tilt","NECK:pan","NECK:nod",
		
		"TAIL:tilt",	"TAIL:pan",
		
		"MOUTH",
		
		"LED:headC","LED:headW",
		"LED:modeR","LED:modeG",
		"LED:modeB","LED:wless",
		"LED:faceA","LED:faceB","LED:faceC","LED:faceD","LED:faceE","LED:faceF","LED:faceG",
		"LED:faceH","LED:faceI","LED:faceJ","LED:faceK","LED:faceL","LED:faceM","LED:faceN",
		"LED:bkFrC","LED:bkFrW",
		"LED:bkMdC","LED:bkMdW",
		"LED:bkRrC","LED:bkRrW",
		"LED:ABmod",
		
		"EAR:left","EAR:right",

		"BaseFrame",
		"LFrFootFrame",
		"RFrFootFrame",
		"LBkFootFrame",
		"RBkFootFrame",
		"CameraFrame",
		"NearIRFrame",
		"FarIRFrame",
		"ChestIRFrame",
		NULL
	};
	
	
	//! provides polymorphic robot capability detection/mapping
	class ERS7Capabilities : public Capabilities {
	public:
		//! constructor
		ERS7Capabilities()
		: Capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs)
		{
			// 2xx button aliases
			buttonToIndex["HeadFrBut"]=HeadFrButOffset; // aliased to HeadButOffset
			buttonToIndex["BackBut"]=BackButOffset; //aliased to MiddleBackButOffset
			// 210 led aliases
			frameToIndex["LED:botL"]=BotLLEDOffset; // aliased to face panel...
			frameToIndex["LED:botR"]=BotRLEDOffset;
			frameToIndex["LED:midL"]=MidLLEDOffset;
			frameToIndex["LED:midR"]=MidRLEDOffset;
			frameToIndex["LED:topL"]=TopLLEDOffset;
			frameToIndex["LED:topR"]=TopRLEDOffset;
			frameToIndex["LED:topBr"]=TopBrLEDOffset;
			frameToIndex["LED:tlRed"]=TlRedLEDOffset; // aliased to the red back light
			frameToIndex["LED:tlBlu"]=TlBluLEDOffset; // aliased to the blue back light
			// 220 led aliases
			frameToIndex["LED:bkL1"]=TlBluLEDOffset; // aliased to the blue back light
			frameToIndex["LED:bkL2"]=TlRedLEDOffset; // aliased to the red back light
			// 2xx sensor aliases
			sensorToIndex["IRDist"]=IRDistOffset; // aliased to the near IR sensor
			frameToIndex["IRFrame"]=IRFrameOffset; // aliased to the near IR sensor
			
			// the AB switch is a meta-output, don't tell Aperios about it
			fakeOutputs.insert(LEDABModeOffset);
		}
	};
	//! allocation declared in RobotInfo.cc
	extern const ERS7Capabilities capabilities;
	
	
	//! the joint identifier strings used to refer to specific joints in OPEN-R (but not needed for others)
	/*!@showinitializer 
	 * The offset consts defined in this file correspond to this table and will make life easier
	 * if you feel the need to reorder things, but they can't be arbitrarily reordered... \n
	 * In particular, assumptions are made that the pid joints will be in slots 0-numPIDJoints
	 * and that the fast outputs (ie NOT ears) will be in slots 0-NumFastOutputs\n
	 * There may be other assumptions not noted here!!!
	 * @note These entries DON'T correspond to the CPC index numbers (this only lists joints for identifying joints to Aperios, CPCs are for identifying sensors from Aperios */
	const char* const PrimitiveName [NumOutputs] = {
		"PRM:/r2/c1-Joint2:21",       //!< the left front leg   the rotator
		"PRM:/r2/c1/c2-Joint2:22",    //!< the left front leg   the elevator 
		"PRM:/r2/c1/c2/c3-Joint2:23", //!< the left front leg   the knee 
		"PRM:/r4/c1-Joint2:41",       //!< the right front leg   the rotator
		"PRM:/r4/c1/c2-Joint2:42",    //!< the right front leg    the elevator 
		"PRM:/r4/c1/c2/c3-Joint2:43", //!< the right front leg   the knee 
				
		"PRM:/r3/c1-Joint2:31",       //!< the left hind leg   the rotator
		"PRM:/r3/c1/c2-Joint2:32",    //!< the left hind leg   the elevator 
		"PRM:/r3/c1/c2/c3-Joint2:33", //!< the left hind leg   the knee
		"PRM:/r5/c1-Joint2:51",       //!< the right hind leg   the rotator
		"PRM:/r5/c1/c2-Joint2:52",    //!< the right hind leg   the elevator 
		"PRM:/r5/c1/c2/c3-Joint2:53", //!< the right hind leg   the knee 
		  
		"PRM:/r1/c1-Joint2:11",       //!< the lower neck tilt (12)
		"PRM:/r1/c1/c2-Joint2:12",    //!< the neck pan 
		"PRM:/r1/c1/c2/c3-Joint2:13", //!< the upper neck tilt (nod)
				
		"PRM:/r6/c1-Joint2:61",       //!< the tail tilt
		"PRM:/r6/c2-Joint2:62",       //!< the tail rotate
				
		"PRM:/r1/c1/c2/c3/c4-Joint2:14", //!< the mouth (17)
				
		"PRM:/r1/c1/c2/c3/l1-LED2:l1", //!< Head light (color) (x6, 18)
		"PRM:/r1/c1/c2/c3/l2-LED2:l2", //!< Head light (white)
		"PRM:/r1/c1/c2/c3/l3-LED2:l3", //!< Red mode indicator
		"PRM:/r1/c1/c2/c3/l4-LED2:l4", //!< Green mode indicator
		"PRM:/r1/c1/c2/c3/l5-LED2:l5", //!< Blue mode indicator
		"PRM:/r1/c1/c2/c3/l6-LED2:l6", //!< wireless light
		
		"PRM:/r1/c1/c2/c3/la-LED3:la", //!< face lights... (x14, 24-37)
		"PRM:/r1/c1/c2/c3/lb-LED3:lb", 		
		"PRM:/r1/c1/c2/c3/lc-LED3:lc", 
		"PRM:/r1/c1/c2/c3/ld-LED3:ld", 
		"PRM:/r1/c1/c2/c3/le-LED3:le", 		
		"PRM:/r1/c1/c2/c3/lf-LED3:lf", 
		"PRM:/r1/c1/c2/c3/lg-LED3:lg", 
		"PRM:/r1/c1/c2/c3/lh-LED3:lh", 
		"PRM:/r1/c1/c2/c3/li-LED3:li", 
		"PRM:/r1/c1/c2/c3/lj-LED3:lj", 
		"PRM:/r1/c1/c2/c3/lk-LED3:lk", 
		"PRM:/r1/c1/c2/c3/ll-LED3:ll", 
		"PRM:/r1/c1/c2/c3/lm-LED3:lm", 
		"PRM:/r1/c1/c2/c3/ln-LED3:ln", //!< ...last face light (37)

		"PRM:/lu-LED3:lu", //!< front back light (color) (x6, 38)
		"PRM:/lv-LED3:lv", //!< front back light (white)
		"PRM:/lw-LED3:lw", //!< middle back light (color)
		"PRM:/lx-LED3:lx", //!< middle back light (white)
		"PRM:/ly-LED3:ly", //!< rear back light (color)
		"PRM:/lz-LED3:lz", //!< rear back light (white)
		"",  //!< the virtual mode A/B switcher
		  
		"PRM:/r1/c1/c2/c3/e5-Joint4:15", //!< left ear (44)
		"PRM:/r1/c1/c2/c3/e6-Joint4:16" //!< right ear
	};

	//! use to open speaker connection with the system
	const char* const SpeakerLocator="PRM:/s1-Speaker:S1";

	//! use to open camera connection with the system
	const char* const CameraLocator="PRM:/r1/c1/c2/c3/i1-FbkImageSensor:F1";

	//! This table holds the default PID values for each joint.  see PIDMC
	const float DefaultPIDs[NumPIDJoints][3] =
		{
			{ 0x1C/(float)(1<<(16-0xE)), 0x08/(float)(1<<(16-0x2)), 0x01/(float)(1<<(16-0xF)) },
			{ 0x14/(float)(1<<(16-0xE)), 0x04/(float)(1<<(16-0x2)), 0x01/(float)(1<<(16-0xF)) },
			{ 0x1C/(float)(1<<(16-0xE)), 0x08/(float)(1<<(16-0x2)), 0x01/(float)(1<<(16-0xF)) },
			{ 0x1C/(float)(1<<(16-0xE)), 0x08/(float)(1<<(16-0x2)), 0x01/(float)(1<<(16-0xF)) },
			{ 0x14/(float)(1<<(16-0xE)), 0x04/(float)(1<<(16-0x2)), 0x01/(float)(1<<(16-0xF)) },
			{ 0x1C/(float)(1<<(16-0xE)), 0x08/(float)(1<<(16-0x2)), 0x01/(float)(1<<(16-0xF)) },
			{ 0x1C/(float)(1<<(16-0xE)), 0x08/(float)(1<<(16-0x2)), 0x01/(float)(1<<(16-0xF)) },
			{ 0x14/(float)(1<<(16-0xE)), 0x04/(float)(1<<(16-0x2)), 0x01/(float)(1<<(16-0xF)) },
			{ 0x1C/(float)(1<<(16-0xE)), 0x08/(float)(1<<(16-0x2)), 0x01/(float)(1<<(16-0xF)) },
			{ 0x1C/(float)(1<<(16-0xE)), 0x08/(float)(1<<(16-0x2)), 0x01/(float)(1<<(16-0xF)) },
			{ 0x14/(float)(1<<(16-0xE)), 0x04/(float)(1<<(16-0x2)), 0x01/(float)(1<<(16-0xF)) },
			{ 0x1C/(float)(1<<(16-0xE)), 0x08/(float)(1<<(16-0x2)), 0x01/(float)(1<<(16-0xF)) },

			{ 0x0A/(float)(1<<(16-0xE)), 0x04/(float)(1<<(16-0x2)), 0x02/(float)(1<<(16-0xF)) },
			{ 0x08/(float)(1<<(16-0xE)), 0x02/(float)(1<<(16-0x2)), 0x04/(float)(1<<(16-0xF)) },
			{ 0x08/(float)(1<<(16-0xE)), 0x08/(float)(1<<(16-0x2)), 0x02/(float)(1<<(16-0xF)) }, // 'I' value changed to 0x08 instead of standard 0x04

			{ 0x0A/(float)(1<<(16-0xE)), 0x04/(float)(1<<(16-0x2)), 0x04/(float)(1<<(16-0xF)) },
			{ 0x0A/(float)(1<<(16-0xE)), 0x04/(float)(1<<(16-0x2)), 0x04/(float)(1<<(16-0xF)) },

			{ 0x08/(float)(1<<(16-0xE)), 0x00/(float)(1<<(16-0x2)), 0x04/(float)(1<<(16-0xF)) }
		};
		
	//! These will control the shift values given to the system.  see PIDMC
	/*! These are modified from the default values to give better range of values to the gains */
	const unsigned char DefaultPIDShifts[3] = {0x0E, 0x02-1, 0x0F-3};
		
	//!These values are Sony's recommended maximum joint velocities, in rad/ms
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
		5.27962099f,
		4.86510529f,
		5.27962099f,
		5.27962099f,
		4.86510529f,
		5.27962099f,
		5.27962099f,
	
		3.18522588f, //Head TPR
		10.0574598f,
		5.78140315f,
	
		15.1625479f, //Tail
		15.1625479f,
	
		10.1447263f, //Mouth
		
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, //LEDs
		
		0,0                //Ears
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
			{ RAD(-115),RAD(130) },{ RAD(-10),RAD(88) },{ RAD(-25),RAD(122) }, //left front REK
			{ RAD(-115),RAD(130) },{ RAD(-10),RAD(88) },{ RAD(-25),RAD(122) }, //right front REK
			{ RAD(-130),RAD(115) },{ RAD(-10),RAD(88) },{ RAD(-25),RAD(122) }, //left back REK
			{ RAD(-130),RAD(115) },{ RAD(-10),RAD(88) },{ RAD(-25),RAD(122) }, //right back REK

			{ RAD(-75),RAD(0) },{ RAD(-88),RAD(88) },{ RAD(-15),RAD(45) }, //neck Tilt-pan-nod
				
			{ RAD(5),RAD(60) },{ RAD(-45),RAD(45) }, // tail tp

			{ RAD(-55),RAD(-3) }, //mouth

			{0,1},{0,1},{0,1},{0,1},{0,1},{0,1}, //misc LEDs
			{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1}, //Face LEDs
			{0,1},{0,1},{0,1},{0,1},{0,1},{0,1}, //back LEDs
			{0,1}, //virtual mode A/B switcher
		
			{0,1},{0,1} //ears
		};

	//! This table holds the mechanical limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
	/*! Same as #outputRanges, don't know actual values because they were never specified by Sony */
	const float mechanicalLimits[NumOutputs][2] =
		{
			{ RAD(-115),RAD(130) },{ RAD(-10),RAD(88) },{ RAD(-25),RAD(122) }, //left front REK
			{ RAD(-115),RAD(130) },{ RAD(-10),RAD(88) },{ RAD(-25),RAD(122) }, //right front REK
			{ RAD(-115),RAD(130) },{ RAD(-10),RAD(88) },{ RAD(-25),RAD(122) }, //left back REK
			{ RAD(-115),RAD(130) },{ RAD(-10),RAD(88) },{ RAD(-25),RAD(122) }, //right back REK

			{ RAD(-75),RAD(0) },{ RAD(-88),RAD(88) },{ RAD(-15),RAD(45) }, //neck Tilt-pan-nod
				
			{ RAD(5),RAD(60) },{ RAD(-45),RAD(45) }, // tail tp

			{ RAD(-55),RAD(-3) }, //mouth

			{0,1},{0,1},{0,1},{0,1},{0,1},{0,1}, //misc head LEDs
			{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1}, //Face LEDs
			{0,1},{0,1},{0,1},{0,1},{0,1},{0,1}, //back LEDs
			{0,1}, //virtual mode A/B switcher

			{0,1},{0,1} //ears
		};

#ifdef __RI_RAD_FLAG
#undef RAD
#undef __RI_RAD_FLAG
#endif


	/*! @name CPC IDs
	 * Values defined by OPEN-R, used to interface with lower level OPEN-R code to read sensors - values @e don't correspond to order of ERS7Info::PrimitiveName */
	const int CPCJointMouth      =  0; //!< Mouth                           
	const int CPCSwitchChin      =  1; //!< Chin sensor                     
	const int CPCJointNeckNod    =  2; //!< Neck tilt2                      
	const int CPCSensorHead      =  3; //!< Head sensor                     
	const int CPCSensorNearPSD   =  4; //!< Head distance sensor(near)      
	const int CPCSensorFarPSD    =  5; //!< Head distance sensor(far)       
	const int CPCJointNeckPan    =  6; //!< Neck pan                        
	const int CPCJointNeckTilt   =  7; //!< Neck tilt1                      
	const int CPCSwitchLFPaw     =  8; //!< Left fore leg  paw sensor       
	const int CPCJointLFKnee     =  9; //!< Left fore legJ3                 
	const int CPCJointLFElevator = 10; //!< Left fore legJ2                 
	const int CPCJointLFRotator  = 11; //!< Left fore legJ1                 
	const int CPCSwitchLHPaw     = 12; //!< Left hind leg  paw sensor       
	const int CPCJointLHKnee     = 13; //!< Left hind legJ3                 
	const int CPCJointLHElevator = 14; //!< Left hind legJ2                 
	const int CPCJointLHRotator  = 15; //!< Left hind legJ1                 
	const int CPCSwitchRFPaw     = 16; //!< Right fore leg  paw sensor      
	const int CPCJointRFKnee     = 17; //!< Right fore legJ3                
	const int CPCJointRFElevator = 18; //!< Right fore legJ2                
	const int CPCJointRFRotator  = 19; //!< Right fore legJ1                
	const int CPCSwitchRHPaw     = 20; //!< Right hind leg  paw sensor      
	const int CPCJointRHKnee     = 21; //!< Right hind legJ3                
	const int CPCJointRHElevator = 22; //!< Right hind legJ2                
	const int CPCJointRHRotator  = 23; //!< Right hind legJ1                
	const int CPCJointTailTilt   = 24; //!< Tail tilt                       
	const int CPCJointTailPan    = 25; //!< Tail pan                        
	const int CPCSensorAccelFB   = 26; //!< Acceleration sensor(front-back) 
	const int CPCSensorAccelLR   = 27; //!< Acceleration sensor(right-left) 
	const int CPCSensorAccelUD   = 28; //!< Acceleration sensor(up-down)    
	const int CPCSensorChestPSD  = 29; //!< Chest distance sensor           
	const int CPCSwitchWireless  = 30; //!< Wireless LAN switch             
	const int CPCSensorBackRear  = 31; //!< Back sensor(rear)               
	const int CPCSensorBackMiddle= 32; //!< Back sensor(middle)             
	const int CPCSensorBackFront = 33; //!< Back sensor(front)              
	//@}

}
	
/*! @file
 * @brief Defines RobotInfo namespace for ERS-7 models, gives some information about the robot's capabilities, such as joint counts, offsets, names and PID values
 * @author ejt (Creator)
 */

#endif
