#include "Shared/RobotInfo.h"
//#if defined(TGT_ERS7) || defined(TGT_ERS210) || defined(TGT_ERS2xx)

#include "Aibo3DControllerBehavior.h"
#include "Behaviors/Controls/BehaviorSwitchControl.h"

REGISTER_BEHAVIOR_MENU(Aibo3DControllerBehavior,"TekkotsuMon");

extern BehaviorSwitchControlBase * const autoRegisterWorldStateSerializer;

void Aibo3DControllerBehavior::doStart() {
	// Behavior startup
	BehaviorBase::doStart();
	
	launchedSerializer=false;
	if(autoRegisterWorldStateSerializer!=NULL) {
		if(!autoRegisterWorldStateSerializer->isRunning()) {
			autoRegisterWorldStateSerializer->start();
			launchedSerializer=true;
		}
		// open gui
		/*		std::vector<std::string> tmp;
		tmp.push_back("Aibo3D Load Instructions");
		tmp.push_back("To load Aibo3D, you will need to install java3d\nand then run Tekkotsu/tools/aibo3d/");
		tmp.back()+=getGUIType();
		Controller::loadGUI("ControllerMsg","LoadAibo3d",getPort(),tmp);*/
	}
	
	Controller::loadGUI(getGUIType(),getGUIType(),getPort());
}

void Aibo3DControllerBehavior::doStop() {
	Controller::closeGUI(getGUIType());
	if(launchedSerializer && autoRegisterWorldStateSerializer!=NULL) {
		autoRegisterWorldStateSerializer->stop();
	}
	// Total behavior stop
	BehaviorBase::doStop();
}

//#endif
