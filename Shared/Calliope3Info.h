//-*-c++-*-
#ifndef INCLUDED_Calliope3Info_h
#define INCLUDED_Calliope3Info_h

#include "Calliope3Components.h"

//**** This file defines Calliope3 robot models in terms of the modular components defined in
//**** Calliope3Components.h.
//! Calliope with 2DOF arm and Microsof LifeCam camera on pan/tilt
namespace Calliope3Info {
	extern const char* const TargetName;
	
	using namespace Calliope3Components;
	using Calliope3Components::PIDJointOffset;
	using Calliope3Components::NumButtons;
	using Calliope3Components::buttonNames;
	using Calliope3Components::NumLEDs;
	using Calliope3Components::NumSensors;
	using Calliope3Components::sensorNames;
	
	using namespace Calliope3;
	using Calliope3::NumPIDJoints;
	using Calliope3::NumOutputs;
	using Calliope3::NumArms;
	using Calliope3::LEDOffset;
	using Calliope3::HeadTiltOffset;
	using Calliope3::MaxOutputSpeed;


	using namespace CameraLifeCam;

	const unsigned NumReferenceFrames = NumOutputs + 1/*base*/ + NumArms/*grippers*/ + 1/*camera*/ + 0/*IR sensors*/ + 0/*fingers*/;

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
		"LED:Spot",
		"LED:CheckRobot",
		"LED:Debris",
		"LED:Home",
		"DesiredMode",
		"BaseFrame",
		"GripperFrame",
		"CameraFrame",
		NULL
	};
	
	class Calliope3Capabilities : public Capabilities {
	public:
		Calliope3Capabilities()
		: Capabilities(TargetName, NumReferenceFrames, outputNames, NumButtons, buttonNames, NumSensors, sensorNames, PIDJointOffset, NumPIDJoints, LEDOffset, NumLEDs, NumOutputs)
		{
			frameToIndex["NECK:nod"] = HeadTiltOffset;
		}
	};

	extern const Calliope3Capabilities capabilities;
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
