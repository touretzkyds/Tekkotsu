#include "Shared/RobotInfo.h"
#if defined(TGT_HAS_LEGS) && !defined(TGT_IS_AIBO)

#include "ConfigurationEditor.h"
#include "Behaviors/Mon/WalkController.h"
#include "Motion/XWalkMC.h"

class XWalkEdit : public ConfigurationEditor {
public:
	XWalkEdit() : ConfigurationEditor("XWalk Edit","Provides editing of an XWalkMC instance"), ctrl() {
		ctrl.setAutoDelete(false);
		setRootCollection(ctrl.getWalkMC());
		load.setRoot(::config->motion.root);
		save.setRoot(::config->motion.root);
	}
	
	~XWalkEdit() {
		if(ctrl.isActive()) {
			ctrl.setKeepGUI(true);
			ctrl.stop();
		}
	}
	
	virtual ControlBase * activate(MC_ID disp_id, Socket * gui) {
		if(!ctrl.isActive())
			ctrl.start();
		return ConfigurationEditor::activate(disp_id,gui);
	}
	virtual void deactivate() {
		ConfigurationEditor::deactivate();
	}
	
protected:
	WalkController ctrl;
};

#endif
