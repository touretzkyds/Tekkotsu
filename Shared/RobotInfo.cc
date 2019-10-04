#include "RobotInfo.h"
#include <iostream>

#if defined(TGT_ERS2xx) && defined(PLATFORM_APERIOS)
#  include <OPENR/OPENRAPI.h>
#endif

// collecting these static allocations here so we don't have to have a separate file for each one
// you can either make a .cc file dedicated to your Info.h, or just add an entry below...

#include "ERS210Info.h"
namespace ERS210Info {
  const char* const TargetName="ERS-210";
	const ERS210Capabilities capabilities;
}

#include "ERS220Info.h"
namespace ERS220Info {
	const char* const TargetName="ERS-220";
	const ERS220Capabilities capabilities;
}

#include "ERS2xxInfo.h"
namespace ERS2xxInfo {
	const char* const TargetName="ERS-2xx";
	const ERS2xxCapabilities capabilities;
}

#include "ERS7Info.h"
namespace ERS7Info {
	const char* const TargetName="ERS-7";
	const ERS7Capabilities capabilities;
	const char CameraModelName[] = "ERS-7-Camera";
}

#include "LynxArm6Info.h"
namespace LynxArm6Info {
	const char* const TargetName="LynxArm6";
	const Capabilities capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,0,0,NumOutputs);
}

#include "Regis1Info.h"
namespace Regis1Info {
	const char* const TargetName="Regis1";
	const Regis1Capabilities capabilities;
}

#include "QBotPlusInfo.h"
namespace QBotPlusInfo {
	const char* const TargetName="QBotPlus";
	const QBotPlusCapabilities capabilities;
}

#include "QwerkInfo.h"
namespace QwerkInfo {
	const char* const TargetName="Qwerk";
	const Capabilities capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs);
}

#include "CreateInfo.h"
namespace CreateInfo {
	const char* const TargetName="Create";
	const Capabilities capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs);
}

#include "CalliopeInfo.h"
namespace CalliopeInfo {
	const char* const TargetName="Calliope";
	const CalliopeCapabilities capabilities;
}

namespace CalliopeLPInfo {
	const char* const TargetName="CalliopeLP";
	const CalliopeLPCapabilities capabilities;
}

namespace CalliopeSPInfo {
	const char* const TargetName="CalliopeSP";
	const CalliopeSPCapabilities capabilities;
}

namespace Calliope2SPInfo {
	const char* const TargetName="Calliope2SP";
	const Calliope2SPCapabilities capabilities;
}

namespace Calliope5SPInfo {
	const char* const TargetName="Calliope5SP";
	const Calliope5SPCapabilities capabilities;
}

namespace Calliope5KPInfo {
	const char* const TargetName="Calliope5KP";
	const Calliope5KPCapabilities capabilities;
}
#include "Calliope3Info.h"
namespace Calliope3Info {
	const char* const TargetName="Calliope3";
        const Calliope3Capabilities capabilities;
}
#include "CalliopeUInfo.h"
namespace CalliopeUInfo {
	const char* const TargetName="CalliopeU";
	const CalliopeUCapabilities capabilities;
}

#include "WiiMoteInfo.h"
namespace WiiMoteInfo {
	const char* const TargetName="WiiMote";
	const Capabilities capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs);
}

#include "BioloidInfo.h"
namespace BioloidInfo {
	const char* const TargetName="Bioloid";
	const Capabilities capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs);
}

#include "PanTiltInfo.h"
namespace PanTiltInfo {
	const char* const TargetName="PanTilt";
	const Capabilities capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs);
}

#include "HandEyeInfo.h"
namespace HandEyeInfo {
	const char* const TargetName="HandEye";
	const HandEyeCapabilities capabilities;
}

#include "HandEyeZInfo.h"
namespace HandEyeZInfo {
	const char* const TargetName="HandEyeZ";
	const HandEyeZCapabilities capabilities;
}


#include "CameraBotInfo.h"
namespace CameraBotInfo {
	const char* const TargetName="CameraBot";
	const CameraBotCapabilities capabilities;
}

#include "TentacleInfo.h"
namespace TentacleInfo {
	const char* const TargetName="Tentacle";
	const Capabilities capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs);
}

#include "ChiaraInfo.h"
namespace ChiaraInfo {
	const char* const TargetName="Chiara";
	const ChiaraCapabilities capabilities;
}

#include "Chiara2Info.h"
namespace Chiara2Info {
	const char* const TargetName="Chiara2";
	const ChiaraCapabilities capabilities;
}

#include "KHR2Info.h"
namespace KHR2Info {
	const char* const TargetName="KHR2";
	const Capabilities capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs);
}

#include "MantisArmInfo.h"
namespace MantisArmInfo {
	const char* const TargetName="MantisArm";
	const MantisArmCapabilities capabilities;
}

#include "MantisLegInfo.h"
namespace MantisLegInfo {
	const char* const TargetName="MantisLeg";
	const MantisLegCapabilities capabilities;
}

#include "MantisInfo.h"
namespace MantisInfo {
	const char* const TargetName="Mantis";
	const MantisCapabilities capabilities;
}

namespace CameraGeneric60 {
	const char CameraModelName[] = "Generic60";
}

namespace Camera75DOF {
	const char CameraModelName[] = "QuickCam-Pro-9000";
}

namespace CameraKinect {
	const char CameraModelName[] = "Kinect-Camera";
}

namespace CameraSTX {
	const char CameraModelName[] = "QuickCam-STX";
}

namespace CameraERS2xx {
	const char CameraModelName[] = "ERS-2xx-Camera";
}

namespace CameraLifeCam {
        const char CameraModelName[] = "LifeCam-HD-5000";
}
// and now for RobotInfo's own stuff:
namespace RobotInfo {
	
	const char* const detectModel() {
#ifdef TGT_ERS2xx
#  ifdef PLATFORM_APERIOS
		// might be running on either 210 or 220, check
		char robotDesignStr[orobotdesignNAME_MAX + 1];
		memset(robotDesignStr, 0, sizeof(robotDesignStr));
		if (OPENR::GetRobotDesign(robotDesignStr) != oSUCCESS) {
			std::cout << "OPENR::GetRobotDesign() failed." << std::endl;
			return TargetName;
		} else {
			if(strcmp(robotDesignStr,"ERS-210")==0)
				return ERS210Info::TargetName;
			else if(strcmp(robotDesignStr,"ERS-220")==0)
				return ERS220Info::TargetName;
			else {
				std::cerr << "ERROR: Unknown name '" << robotDesignStr << "' for target ERS2xx" << std::endl;
				return TargetName;
			}
		}
#  else
#    warning TGT_ERS2xx assuming ERS-210 for simulation on local platform
		return ERS210Info::TargetName;
#  endif
		
#else
		// target is directly the robot, return the target name
		return TargetName;
#endif
	}
	
#if defined(TGT_HAS_CAMERA) || defined(TGT_HAS_WEBCAM)
	const char* CameraName = CameraModelName;
	fmat::Matrix<3,3> CameraHomography = fmat::Matrix<3,3>::identity();
#endif
	
#ifndef PLATFORM_APERIOS
	const char* const RobotName = detectModel();
#else // have to use a string because aperios is annoying like that
	const std::string RobotName = detectModel();
#endif
	
	
Capabilities::Capabilities(const char* robName, size_t numFrame, const char * const frameNames[], 
			   size_t numBut, const char * const butNames[], 
			   size_t numSen, const char * const senNames[], 
			   size_t pidOff, size_t numPID, size_t ledOff, size_t numLED, size_t numTotalOutputs) :
  name(robName),
  frames(frameNames,frameNames+numFrame), buttons(butNames,butNames+numBut), sensors(senNames,senNames+numSen),
  frameToIndex(), buttonToIndex(), sensorToIndex(),
  pidJointOffset(pidOff), numPIDJoints(numPID), 
  ledOffset(ledOff), numLEDs(numLED),
  numOutputs(numTotalOutputs), fakeOutputs() {
  for(size_t i=0; i<frames.size(); ++i)
    frameToIndex[frames[i]]=i;
  for(size_t i=0; i<buttons.size(); ++i)
    buttonToIndex[buttons[i]]=i;
  for(size_t i=0; i<sensors.size(); ++i)
    sensorToIndex[sensors[i]]=i;
  
  std::map<std::string, const Capabilities*>::const_iterator it=getCaps().find(robName);
  if(it!=getCaps().end())
    std::cerr << "WARNING: RobotInfo '" << robName 
	      << "' capabilities has already been registered!  Name conflict?  Replacing previous..." << std::endl;
  getCaps()[robName]=this;
}

std::map<std::string, const Capabilities*>& Capabilities::getCaps() {
    static std::map<std::string, const Capabilities*> caps;
    return caps;
}

} // namespace
