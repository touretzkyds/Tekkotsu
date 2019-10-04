//-*-c++-*-
#ifndef INCLUDED_RobotInfo_h
#define INCLUDED_RobotInfo_h

#include <map>
#include <string>

// If creating a new robot configuration, add a new entry in the list below

#if TGT_ERS210
#	include "Shared/ERS210Info.h"
namespace RobotInfo { using namespace ERS210Info; }

#elif TGT_ERS220
#	include "Shared/ERS220Info.h"
namespace RobotInfo { using namespace ERS220Info; }

#elif TGT_ERS2xx
#	include "Shared/ERS2xxInfo.h"
namespace RobotInfo { using namespace ERS2xxInfo; }

#elif TGT_ERS7
#	include "Shared/ERS7Info.h"
namespace RobotInfo { using namespace ERS7Info; }

#elif TGT_LYNXARM6
#	include "Shared/LynxArm6Info.h"
namespace RobotInfo { using namespace LynxArm6Info; }

#elif TGT_REGIS1
#	include "Shared/Regis1Info.h"
namespace RobotInfo { using namespace Regis1Info; }

#elif TGT_QBOTPLUS
#	include "Shared/QBotPlusInfo.h"
namespace RobotInfo { using namespace QBotPlusInfo; }

#elif TGT_QWERK
#	include "Shared/QwerkInfo.h"
namespace RobotInfo { using namespace QwerkInfo; }

#elif TGT_WIIMOTE
#	include "Shared/WiiMoteInfo.h"
namespace RobotInfo { using namespace WiiMoteInfo; }

#elif TGT_CREATE
#	include "Shared/CreateInfo.h"
namespace RobotInfo { using namespace CreateInfo; }

#elif TGT_CALLIOPE
#	include "Shared/CalliopeInfo.h"
namespace RobotInfo { using namespace CalliopeInfo; }

#elif TGT_CALLIOPELP
#	include "Shared/CalliopeInfo.h"
namespace RobotInfo { using namespace CalliopeLPInfo; }

#elif TGT_CALLIOPESP
#	include "Shared/CalliopeInfo.h"
namespace RobotInfo { using namespace CalliopeSPInfo; }

#elif TGT_CALLIOPE2SP
#	include "Shared/CalliopeInfo.h"
namespace RobotInfo { using namespace Calliope2SPInfo; }

#elif TGT_CALLIOPE5SP
#	include "Shared/CalliopeInfo.h"
namespace RobotInfo { using namespace Calliope5SPInfo; }

#elif TGT_CALLIOPE5KP
#	include "Shared/CalliopeInfo.h"
namespace RobotInfo { using namespace Calliope5KPInfo; }

#elif TGT_CALLIOPE3
#	include "Shared/Calliope3Info.h"
namespace RobotInfo { using namespace Calliope3Info; }

#elif TGT_CALLIOPE3A
#	include "Shared/Calliope3Info.h"
namespace RobotInfo { using namespace Calliope3Info; }

#elif TGT_CALLIOPEU
#	include "Shared/CalliopeUInfo.h"
namespace RobotInfo { using namespace CalliopeUInfo; }

#elif TGT_BIOLOID
#	include "Shared/BioloidInfo.h"
namespace RobotInfo { using namespace BioloidInfo; }

#elif TGT_PANTILT
#	include "Shared/PanTiltInfo.h"
namespace RobotInfo { using namespace PanTiltInfo; }

#elif TGT_HANDEYE
#	include "Shared/HandEyeInfo.h"
namespace RobotInfo { using namespace HandEyeInfo; }

#elif TGT_HANDEYEZ
#	include "Shared/HandEyeZInfo.h"
namespace RobotInfo { using namespace HandEyeZInfo; }

#elif TGT_CAMERABOT
#	include "Shared/CameraBotInfo.h"
namespace RobotInfo { using namespace CameraBotInfo; }

#elif TGT_CHIARA
#	include "Shared/ChiaraInfo.h"
namespace RobotInfo { using namespace ChiaraInfo; }

#elif TGT_CHIARA2
#	include "Shared/Chiara2Info.h"
namespace RobotInfo { using namespace Chiara2Info; }

#elif TGT_DYNAMIC
#	include "Shared/DynamicInfo.h"
namespace RobotInfo { using namespace DynamicInfo; }

#elif TGT_TENTACLE
#	include "Shared/TentacleInfo.h"
namespace RobotInfo { using namespace TentacleInfo; }

#elif TGT_KHR2
#	include "Shared/KHR2Info.h"
namespace RobotInfo { using namespace KHR2Info; }

#elif TGT_MANTISARM
#   include "Shared/MantisArmInfo.h"
namespace RobotInfo { using namespace MantisArmInfo; }

#elif TGT_MANTISLEG
#   include "Shared/MantisLegInfo.h"
namespace RobotInfo { using namespace MantisLegInfo; }

#elif TGT_MANTIS
#   include "Shared/MantisInfo.h"
namespace RobotInfo { using namespace MantisInfo; }

#else //default case, currently ERS-7
#	warning "TGT_<model> undefined or unknown model set - defaulting to ERS7"
#	include "Shared/ERS7Info.h"
namespace RobotInfo { using namespace ERS7Info; }
#endif //model selection


//! Contains information about the robot, such as number of joints, PID defaults, timing information, etc.
/*! This is just a wrapper for whichever namespace corresponds to the current
*  robot target setting (one of TGT_ERS7, TGT_ERS210, TGT_ERS220, or the cross-booting TGT_ERS2xx)
*
*  You probably should look at ERS7Info, ERS210Info, ERS220Info, or ERS2xxInfo for the actual
*  constants used for each model, although some common information shared by all of these namespaces
*  is defined in CommonInfo.h */
namespace RobotInfo {
	
	//! Accessor for Capabilities::caps, returns the Capabilities instance for a specified robot model (or NULL if robot is unknown or didn't provide a Capabilities instance)
	/*! Use this if you have a robot name in string form and want to check or map its capabilities.
	 *  (For example, if you are communicating with another robot of a different type over the network.)
	 *  If you know at compile time the type of the robot in question, you could just directly access
	 *  its 'capabilities' instance via its RobotInfo namespace.  (e.g. ERS210Info::capabilities)
	 *  If you want the capabilities for the current robot, just use the global 'capabilities' instance
	 *  as RobotInfo.h will automatically import the current robot's namespace into the global space. */
	inline const Capabilities* getCapabilities(const std::string& robName) {
		const std::map<std::string, const Capabilities*>& caps = Capabilities::getCaps();
		std::map<std::string, const Capabilities*>::const_iterator it = caps.find(robName);
		return it==caps.end() ? NULL : it->second;
	}
	
#if defined(TGT_HAS_CAMERA) || defined(TGT_HAS_WEBCAM)
	//! Name of the camera being used on the robot
	/*! Gets value from previously imported namespace, but can be overridden by DeviceDriver e.g. Mirage */
	extern const char* CameraName;
	//! The camera correction homography to utilize
	/*! Initialized from file by StartupBehavior::doStart(), using #CameraName to indicate file name. */
	extern fmat::Matrix<3,3> CameraHomography;
#endif
	
}

using namespace RobotInfo;
	
/*! @file
 * @brief Checks the define's to load the appropriate header and namespace
 * @author ejt (Creator)
 */

#endif
