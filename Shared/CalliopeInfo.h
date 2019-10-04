//-*-c++-*-
#ifndef INCLUDED_CalliopeInfo_h
#define INCLUDED_CalliopeInfo_h

#include "CalliopeComponents.h"

//**** This file defines specific Calliope robot models such as
//**** Calliope2SP in terms of the modular components defined in
//**** CalliopeComponents.h.

//! Basic Calliope with no arm, fixed netbook camera, without AX-S1 sensors
namespace CalliopeInfo {
	extern const char* const TargetName; //!< the name of the model, to be used for logging and remote GUIs
	
	using namespace CalliopeComponents;
	using CalliopeComponents::PIDJointOffset;
	using CalliopeComponents::NumButtons;
	using CalliopeComponents::buttonNames;
	using CalliopeComponents::NumLEDs;
	
	using namespace CalliopeComponents::CalliopeInfo;
	using CalliopeComponents::CalliopeInfo::NumPIDJoints;
	using CalliopeComponents::CalliopeInfo::NumOutputs;
	using CalliopeComponents::CalliopeInfo::LEDOffset;
	
	using namespace WithoutAXS1Sensors;
	using WithoutAXS1Sensors::NumSensors;
	using WithoutAXS1Sensors::sensorNames;
	
	using namespace CameraGeneric60;
	const unsigned NumReferenceFrames = NumOutputs + 1 /*base*/ + 0 /*grippers*/ + 1 /*camera*/ + 0 /*IR sensors*/ + 0 /*fingers*/;
	
	//! Names for each of the outputs
	const char* const outputNames[NumReferenceFrames+1] = {
		"WHEEL:L",
		"WHEEL:R",
		"LED:Power(Red)",
		"LED:Power(Green)",
		"LED:Play",
		"LED:Advance",
		"DesiredMode",
		"BaseFrame",
		"CameraFrame",
		NULL
	};
	
	//! provides polymorphic robot capability detection/mapping
	class CalliopeCapabilities : public Capabilities {
	public:
		//! constructor
		CalliopeCapabilities()
		: Capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs)
		{}
	};

	//! allocation declared in RobotInfo.cc
	extern const CalliopeCapabilities capabilities;

}

//! Calliope with no arm, Sony Playstation camera on pan/tilt, AX-S1 sensors
namespace CalliopeSPInfo {
	extern const char* const TargetName; //!< the name of the model, to be used for logging and remote GUIs
	
	using namespace CalliopeComponents;
	using CalliopeComponents::PIDJointOffset;
	using CalliopeComponents::NumButtons;
	using CalliopeComponents::buttonNames;
	using CalliopeComponents::NumLEDs;
	
	using namespace Calliope0;
	using Calliope0::NumPIDJoints;
	using Calliope0::NumOutputs;
	using Calliope0::LEDOffset;
	using Calliope0::HeadTiltOffset;
	using Calliope0::IRFrameOffset;
	
	using namespace WithAXS1Sensors;
	using WithAXS1Sensors::NumSensors;
	using WithAXS1Sensors::sensorNames;
	using WithAXS1Sensors::IRDistOffset;
	
	using namespace Camera75DOF;
	
	const unsigned NumReferenceFrames = NumOutputs + 1 /*base*/ + 0 /*grippers*/ + 1 /*camera*/ + 3 /*IR sensors*/ + 0 /*fingers*/;
	
	//! Names for each of the outputs
	const char* const outputNames[NumReferenceFrames+1] = {
		"WHEEL:L",
		"WHEEL:R",
		"NECK:pan",
		"NECK:tilt",
		"LED:Power(Red)",
		"LED:Power(Green)",
		"LED:Play",
		"LED:Advance",
		"DesiredMode",
		"BaseFrame",
		"CameraFrame",
		"LeftIRFrame",
		"CenterIRFrame",
		"RightIRFrame",
		NULL
	};

	//! provides polymorphic robot capability detection/mapping
	class CalliopeSPCapabilities : public Capabilities {
	public:
		//! constructor
		CalliopeSPCapabilities()
		: Capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs)
		{
			frameToIndex["NECK:nod"] = HeadTiltOffset;
			frameToIndex["IRFrame"] = IRFrameOffset; // aliased to the center IR sensor
			sensorToIndex["IRDist"]=IRDistOffset; // aliased to the center IR sensor
		}
	};
	//! allocation declared in RobotInfo.cc
	extern const CalliopeSPCapabilities capabilities;
}

//! Calliope with no arm, Logitech camera on pan/tilt, AX-S1 sensors
namespace CalliopeLPInfo {
	extern const char* const TargetName; //!< the name of the model, to be used for logging and remote GUIs
	
	using namespace CalliopeComponents;
	using CalliopeComponents::PIDJointOffset;
	using CalliopeComponents::NumButtons;
	using CalliopeComponents::buttonNames;
	using CalliopeComponents::NumLEDs;
	
	using namespace Calliope0;
	using Calliope0::NumPIDJoints;
	using Calliope0::NumOutputs;
	using Calliope0::LEDOffset;
	using Calliope0::HeadTiltOffset;
	using Calliope0::IRFrameOffset;
	
	using namespace WithAXS1Sensors;
	using WithAXS1Sensors::NumSensors;
	using WithAXS1Sensors::sensorNames;
	using WithAXS1Sensors::IRDistOffset;
	
	using namespace Camera75DOF;
	
	const unsigned NumReferenceFrames = NumOutputs + 1 /*base*/ + 0 /*grippers*/ + 1 /*camera*/ + 3 /*IR sensors*/ + 0 /*fingers*/;
	
	//! Names for each of the outputs
	const char* const outputNames[NumReferenceFrames+1] = {
		"WHEEL:L",
		"WHEEL:R",
		"NECK:pan",
		"NECK:tilt",
		"LED:Power(Red)",
		"LED:Power(Green)",
		"LED:Play",
		"LED:Advance",
		"DesiredMode",
		"BaseFrame",
		"CameraFrame",
		"LeftIRFrame",
		"CenterIRFrame",
		"RightIRFrame",
		NULL
	};
	
	//! provides polymorphic robot capability detection/mapping
	class CalliopeLPCapabilities : public Capabilities {
	public:
		//! constructor
		CalliopeLPCapabilities()
		: Capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs)
		{
			frameToIndex["NECK:nod"] = HeadTiltOffset;
			frameToIndex["IRFrame"] = IRFrameOffset; // aliased to the center IR sensor
			sensorToIndex["IRDist"]=IRDistOffset; // aliased to the center IR sensor
		}
	};
	//! allocation declared in RobotInfo.cc
	extern const CalliopeLPCapabilities capabilities;
}

//! Calliope with 2DOF arm, Sony Playstation camera on pan/tilt, AX-S1 sensors
namespace Calliope2SPInfo {
	extern const char* const TargetName; //!< the name of the model, to be used for logging and remote GUIs
	
	using namespace CalliopeComponents;
	using CalliopeComponents::PIDJointOffset;
	using CalliopeComponents::NumButtons;
	using CalliopeComponents::buttonNames;
	using CalliopeComponents::NumLEDs;
	
	using namespace Calliope2;
	using Calliope2::NumPIDJoints;
	using Calliope2::NumOutputs;
	using Calliope2::NumArms;
	using Calliope2::LEDOffset;
	using Calliope2::HeadTiltOffset;
	using Calliope2::IRFrameOffset;
	using Calliope2::MaxOutputSpeed;
	
	using namespace WithAXS1Sensors;
	using WithAXS1Sensors::NumSensors;
	using WithAXS1Sensors::sensorNames;
	using WithAXS1Sensors::IRDistOffset;
	
	using namespace Camera75DOF;
	
	const unsigned NumReferenceFrames = NumOutputs + 1 /*base*/ + NumArms /*grippers*/ + 1 /*camera*/ + 3 /*IR sensors*/ + 0 /*fingers*/;

	//! Names for each of the outputs
	const char* const outputNames[NumReferenceFrames+1] = {
		"WHEEL:L",
		"WHEEL:R",
		"NECK:pan",
		"NECK:tilt",
		"ARM:base",
		"ARM:shoulder",
		"ARM:gripper",
		"LED:Power(Red)",
		"LED:Power(Green)",
		"LED:Play",
		"LED:Advance",
		"DesiredMode",
		"BaseFrame",
		"GripperFrame",
		"CameraFrame",
		"LeftIRFrame",
		"CenterIRFrame",
		"RightIRFrame",
		NULL
	};
	
	//! provides polymorphic robot capability detection/mapping
	class Calliope2SPCapabilities : public Capabilities {
	public:
		//! constructor
		Calliope2SPCapabilities()
		: Capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs)
		{
			frameToIndex["NECK:nod"] = HeadTiltOffset;
			frameToIndex["IRFrame"] = IRFrameOffset; // aliased to the center IR sensor
			sensorToIndex["IRDist"]=IRDistOffset; // aliased to the center IR sensor
		}
	};
	//! allocation declared in RobotInfo.cc
	extern const Calliope2SPCapabilities capabilities;
}

//! Calliope with 5DOF arm, Sony Playstation camera on pan/tilt, AX-S1 sensors
namespace Calliope5SPInfo {
	extern const char* const TargetName; //!< the name of the model, to be used for logging and remote GUIs
	
	using namespace CalliopeComponents;
	using CalliopeComponents::PIDJointOffset;
	using CalliopeComponents::NumButtons;
	using CalliopeComponents::buttonNames;
	using CalliopeComponents::NumLEDs;
	
	using namespace Calliope5;
	using Calliope5::NumPIDJoints;
	using Calliope5::NumOutputs;
	using Calliope5::NumArms;
	using Calliope5::LEDOffset;
	using Calliope5::HeadTiltOffset;
	
	using namespace Calliope5::WithAXS1Sensors;
	using Calliope5::WithAXS1Sensors::NumSensors;
	using Calliope5::WithAXS1Sensors::sensorNames;
	using Calliope5::WithAXS1Sensors::IRDistOffset;
	using Calliope5::WithAXS1Sensors::IRFrameOffset;
	
	using namespace Camera75DOF;
	
	const unsigned NumReferenceFrames = NumOutputs + 1 /*base*/ + NumArms /*grippers*/ + 1 /*camera*/ + 3 /*IR sensors*/ + 2 /*fingers*/;

	//! Names for each of the outputs
	const char* const outputNames[NumReferenceFrames+1] = {
		"WHEEL:L",
		"WHEEL:R",
		"NECK:pan",
		"NECK:tilt",
		"ARM:base",
		"ARM:shoulder",
		"ARM:elbow",
		"ARM:wrist",
		"ARM:wristrot",
		"ARM:gripperLeft",
		"ARM:gripperRight",
		"LED:Power(Red)",
		"LED:Power(Green)",
		"LED:Play",
		"LED:Advance",
		"DesiredMode",
		"BaseFrame",
		"GripperFrame",
		"LeftFingerFrame",
		"RightFingerFrame",
		"CameraFrame",
		"LeftIRFrame",
		"CenterIRFrame",
		"RightIRFrame",
		NULL
	};
	
	//! provides polymorphic robot capability detection/mapping
	class Calliope5SPCapabilities : public Capabilities {
	public:
		//! constructor
		Calliope5SPCapabilities()
		: Capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs)
		{
			frameToIndex["NECK:nod"] = HeadTiltOffset;
			frameToIndex["IRFrame"] = IRFrameOffset; // aliased to the center IR sensor
			sensorToIndex["IRDist"]=IRDistOffset; // aliased to the center IR sensor
		}
	};
	//! allocation declared in RobotInfo.cc
	extern const Calliope5SPCapabilities capabilities;
}

//! Calliope with 5DOF arm, Kinect camera on pan/tilt, without AX-S1 sensors
namespace Calliope5KPInfo {
	extern const char* const TargetName; //!< the name of the model, to be used for logging and remote GUIs
	
	using namespace CalliopeComponents;
	using CalliopeComponents::PIDJointOffset;
	using CalliopeComponents::NumButtons;
	using CalliopeComponents::buttonNames;
	using CalliopeComponents::NumLEDs;
	
	using namespace Calliope5;
	using Calliope5::NumPIDJoints;
	using Calliope5::NumOutputs;
	using Calliope5::NumArms;
	using Calliope5::LEDOffset;
	using Calliope5::HeadTiltOffset;
	
	using namespace Calliope5::WithoutAXS1Sensors;
	using Calliope5::WithoutAXS1Sensors::NumSensors;
	using Calliope5::WithoutAXS1Sensors::sensorNames;
	
	using namespace CameraKinect;
	
	const unsigned NumReferenceFrames = NumOutputs + 1 /*base*/ + NumArms /*grippers*/ + 1 /*camera*/ + 0 /*IR sensors*/ + 2 /*fingers*/;

	//! Names for each of the outputs
	const char* const outputNames[NumReferenceFrames+1] = {
		"WHEEL:L",
		"WHEEL:R",
		"NECK:pan",
		"NECK:tilt",
		"ARM:base",
		"ARM:shoulder",
		"ARM:elbow",
		"ARM:wrist",
		"ARM:wristrot",
		"ARM:gripperLeft",
		"ARM:gripperRight",
		"LED:Power(Red)",
		"LED:Power(Green)",
		"LED:Play",
		"LED:Advance",
		"DesiredMode",
		"BaseFrame",
		"GripperFrame",
		"LeftFingerFrame",
		"RightFingerFrame",
		"CameraFrame",
		NULL
	};
	
	//! provides polymorphic robot capability detection/mapping
	class Calliope5KPCapabilities : public Capabilities {
	public:
		//! constructor
		Calliope5KPCapabilities()
		: Capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs)
		{
			frameToIndex["NECK:nod"] = HeadTiltOffset;
		}
	};
	//! allocation declared in RobotInfo.cc
	extern const Calliope5KPCapabilities capabilities;
}

/*! @file
 * @brief Defines some capabilities of the Calliope robot, based on the iRobot Create
 * @author ejt (Creator)
 */

#ifdef __RI_RAD_FLAG
#  undef RAD
#  undef __RI_RAD_FLAG
#endif


#endif
