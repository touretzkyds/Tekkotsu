//-*-c++-*-
#ifndef INCLUDED_Aibo3DControllerBehavior_h_
#define INCLUDED_Aibo3DControllerBehavior_h_

#include <iostream>
#include "Wireless/Wireless.h"
#include "Behaviors/BehaviorBase.h"
#include "Motion/MotionManager.h"
#include "Motion/RemoteControllerMC.h"
#include "Events/EventRouter.h"
#include "Events/EventBase.h"
#include "Shared/RobotInfo.h"
#include "Behaviors/Controller.h"
#include "Shared/WorldState.h"
#include "Shared/Config.h"

//! Listens to aibo3d control commands coming in from the command port.
class Aibo3DControllerBehavior : public BehaviorBase {
 private:
	Aibo3DControllerBehavior(const Aibo3DControllerBehavior&); //!< don't call
	Aibo3DControllerBehavior operator=(const Aibo3DControllerBehavior&); //!< don't call
	//! if true, indicates we launched the WorldState serializer, so we should stop it again if we stop
	bool launchedSerializer;
	
 public:
	//! constructor
	Aibo3DControllerBehavior() : BehaviorBase("Aibo3DControllerBehavior"),launchedSerializer(false) { }
	//! destructor
	virtual ~Aibo3DControllerBehavior() { }

	virtual void doStart();
	virtual void doStop();
	
	//! returns string corresponding to the Java GUI which should be launched
	virtual std::string getGUIType() const { return "org.tekkotsu.aibo3d.Aibo3D"; }
	//! returns port number the Java GUI should connect to
	virtual unsigned int getPort() const { return config->main.wsjoints_port; }

	static std::string getClassDescription() { 
		return "Launches a WorldStateSerializer and asks gui to load Aibo3D GUI";
		//char tmp[20];
		//sprintf(tmp,"%d",config->main.aibo3d_port);
		//return std::string("Listens to aibo3d control commands coming in from port ")+tmp;
	}
	virtual std::string getDescription() const { return getClassDescription(); }
};

/*! @file
 * @brief Defines Aibo3DControllerBehavior, which listens to commands from the Aibo3D gui and shows current state
 * @author alokl (Creator)
 */

#endif 
