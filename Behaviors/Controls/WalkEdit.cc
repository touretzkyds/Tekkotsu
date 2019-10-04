#include "Shared/RobotInfo.h"
#ifdef TGT_IS_AIBO

#include "WalkEdit.h"
#include "Behaviors/Mon/WalkController.h"
#include "Behaviors/Controls/LoadWalkControl.h"
#include "Behaviors/Controls/SaveWalkControl.h"
#include "Behaviors/Controls/ValueEditControl.h"
#include "Behaviors/Controls/WalkCalibration.h"
#include "Behaviors/Controls/LoadCalibration.h"

using namespace std; 

void WalkEdit::init() {
	menuStack.push(this);
	
#if defined(PLATFORM_APERIOS) && (defined(TGT_ERS210) || defined(TGT_ERS220) || defined(TGT_ERS2xx))
	// Replace this if section with an #if 1 to re-enable
	// This functionality takes about 1MB of memory on the Aibo between loading the binary
	// code and the actual control memory usage.
	addItem(new ControlBase("Disabled to conserve memory"));
#else
	
	//Use this instance of the GUI to try out changes you have made to parameters within this editor
	//(the one in TekkotsuMon will load fresh parameters from walk.prm on disk and won't know about changes made here)
	WalkController * walker = new WalkController();
	addItem((new BehaviorSwitchControlBase("Walk GUI: test changes",walker)));
	
	addItem(new ValueEditControl<float>("Slow Motion",walker->getWalkMC()->getSlowMo()));
	
	startSubMenu("Body","Edit the walk parameters");
	{ 
		addItem(new ValueEditControl<double>("Body Height",&walker->getWalkMC()->getWP().body_height));
		addItem(new ValueEditControl<double>("Body Angle",&walker->getWalkMC()->getWP().body_angle));
		addItem(new ValueEditControl<double>("Hop",&walker->getWalkMC()->getWP().hop));
		addItem(new ValueEditControl<double>("Sway",&walker->getWalkMC()->getWP().sway));
		addItem(new ValueEditControl<int>("Period",&walker->getWalkMC()->getWP().period));
		addItem(new ValueEditControl<int>("Use Diff Drive",&walker->getWalkMC()->getWP().useDiffDrive));
		addItem(new ValueEditControl<float>("Sag",&walker->getWalkMC()->getWP().sag));
	}
	endSubMenu();
	
	startSubMenu("Neutral","Edit the walk parameters");
	{ 
		addItem(new ValueEditControl<double>("NeuLeg[xFL]",&walker->getWalkMC()->getWP().leg[0].neutral.x));
		addItem(new ValueEditControl<double>("NeuLeg[xFR]",&walker->getWalkMC()->getWP().leg[1].neutral.x));
		addItem(new ValueEditControl<double>("NeuLeg[xBL]",&walker->getWalkMC()->getWP().leg[2].neutral.x));
		addItem(new ValueEditControl<double>("NeuLeg[xBR]",&walker->getWalkMC()->getWP().leg[3].neutral.x));
		addItem(new ValueEditControl<double>("NeuLeg[yFL]",&walker->getWalkMC()->getWP().leg[0].neutral.y));
		addItem(new ValueEditControl<double>("NeuLeg[yFR]",&walker->getWalkMC()->getWP().leg[1].neutral.y));
		addItem(new ValueEditControl<double>("NeuLeg[yBL]",&walker->getWalkMC()->getWP().leg[2].neutral.y));
		addItem(new ValueEditControl<double>("NeuLeg[yBR]",&walker->getWalkMC()->getWP().leg[3].neutral.y));
		addItem(new ValueEditControl<double>("NeuLeg[zFL]",&walker->getWalkMC()->getWP().leg[0].neutral.z));
		addItem(new ValueEditControl<double>("NeuLeg[zFR]",&walker->getWalkMC()->getWP().leg[1].neutral.z));
		addItem(new ValueEditControl<double>("NeuLeg[zBL]",&walker->getWalkMC()->getWP().leg[2].neutral.z));
		addItem(new ValueEditControl<double>("NeuLeg[zBR]",&walker->getWalkMC()->getWP().leg[3].neutral.z));
	}
	endSubMenu();
	
	startSubMenu("lift_vel","Edit the walk parameters");
	{ 
		addItem(new ValueEditControl<double>("liftLeg[xFL]",&walker->getWalkMC()->getWP().leg[0].lift_vel.x));
		addItem(new ValueEditControl<double>("liftLeg[xFR]",&walker->getWalkMC()->getWP().leg[1].lift_vel.x));
		addItem(new ValueEditControl<double>("liftLeg[xBL]",&walker->getWalkMC()->getWP().leg[2].lift_vel.x));
		addItem(new ValueEditControl<double>("liftLeg[xBR]",&walker->getWalkMC()->getWP().leg[3].lift_vel.x));
		addItem(new ValueEditControl<double>("liftLeg[yFL]",&walker->getWalkMC()->getWP().leg[0].lift_vel.y));
		addItem(new ValueEditControl<double>("liftLeg[yFR]",&walker->getWalkMC()->getWP().leg[1].lift_vel.y));
		addItem(new ValueEditControl<double>("liftLeg[yBL]",&walker->getWalkMC()->getWP().leg[2].lift_vel.y));
		addItem(new ValueEditControl<double>("liftLeg[yBR]",&walker->getWalkMC()->getWP().leg[3].lift_vel.y));
		addItem(new ValueEditControl<double>("liftLeg[zFL]",&walker->getWalkMC()->getWP().leg[0].lift_vel.z));
		addItem(new ValueEditControl<double>("liftLeg[zFR]",&walker->getWalkMC()->getWP().leg[1].lift_vel.z));
		addItem(new ValueEditControl<double>("liftLeg[zBL]",&walker->getWalkMC()->getWP().leg[2].lift_vel.z));
		addItem(new ValueEditControl<double>("liftLeg[zBR]",&walker->getWalkMC()->getWP().leg[3].lift_vel.z));
	}
	endSubMenu();
	
	startSubMenu("down_vel","Edit the walk parameters");
	{ 
		addItem(new ValueEditControl<double>("downLeg[xFL]",&walker->getWalkMC()->getWP().leg[0].down_vel.x));
		addItem(new ValueEditControl<double>("downLeg[xFR]",&walker->getWalkMC()->getWP().leg[1].down_vel.x));
		addItem(new ValueEditControl<double>("downLeg[xBL]",&walker->getWalkMC()->getWP().leg[2].down_vel.x));
		addItem(new ValueEditControl<double>("downLeg[xBR]",&walker->getWalkMC()->getWP().leg[3].down_vel.x));
		addItem(new ValueEditControl<double>("downLeg[yFL]",&walker->getWalkMC()->getWP().leg[0].down_vel.y));
		addItem(new ValueEditControl<double>("downLeg[yFR]",&walker->getWalkMC()->getWP().leg[1].down_vel.y));
		addItem(new ValueEditControl<double>("downLeg[yBL]",&walker->getWalkMC()->getWP().leg[2].down_vel.y));
		addItem(new ValueEditControl<double>("downLeg[yBR]",&walker->getWalkMC()->getWP().leg[3].down_vel.y));
		addItem(new ValueEditControl<double>("downLeg[zFL]",&walker->getWalkMC()->getWP().leg[0].down_vel.z));
		addItem(new ValueEditControl<double>("downLeg[zFR]",&walker->getWalkMC()->getWP().leg[1].down_vel.z));
		addItem(new ValueEditControl<double>("downLeg[zBL]",&walker->getWalkMC()->getWP().leg[2].down_vel.z));
		addItem(new ValueEditControl<double>("downLeg[zBR]",&walker->getWalkMC()->getWP().leg[3].down_vel.z));
	}
	endSubMenu();
	
	startSubMenu("lift_time","Edit the walk parameters");
	{ 
		addItem(new ValueEditControl<double>("liftimeLeg[FL]",&walker->getWalkMC()->getWP().leg[0].lift_time));
		addItem(new ValueEditControl<double>("liftimeLeg[FR]",&walker->getWalkMC()->getWP().leg[1].lift_time));
		addItem(new ValueEditControl<double>("liftimeLeg[BL]",&walker->getWalkMC()->getWP().leg[2].lift_time));
		addItem(new ValueEditControl<double>("liftimeLeg[BR]",&walker->getWalkMC()->getWP().leg[3].lift_time));
	}
	endSubMenu();
	
	startSubMenu("down_time","Edit the walk parameters");
	{ 
		addItem(new ValueEditControl<double>("downimeLeg[FL]",&walker->getWalkMC()->getWP().leg[0].down_time));
		addItem(new ValueEditControl<double>("downimeLeg[FR]",&walker->getWalkMC()->getWP().leg[1].down_time));
		addItem(new ValueEditControl<double>("downimeLeg[BL]",&walker->getWalkMC()->getWP().leg[2].down_time));
		addItem(new ValueEditControl<double>("downimeLeg[BR]",&walker->getWalkMC()->getWP().leg[3].down_time));
	}
	endSubMenu();
	
	startSubMenu("calibration","Edit the walk parameters");
	{ 
		addItem(new WalkCalibration());
		addItem(new LoadCalibration(&walker->getWalkMC()->getCP()));
		addItem(new ValueEditControl<float>("forward max_accel",&walker->getWalkMC()->getCP().max_accel[WalkMC::CalibrationParam::forward]));
		addItem(new ValueEditControl<float>("reverse max_accel",&walker->getWalkMC()->getCP().max_accel[WalkMC::CalibrationParam::reverse]));
		addItem(new ValueEditControl<float>("strafe max_accel",&walker->getWalkMC()->getCP().max_accel[WalkMC::CalibrationParam::strafe]));
		addItem(new ValueEditControl<float>("rotate max_accel",&walker->getWalkMC()->getCP().max_accel[WalkMC::CalibrationParam::rotate]));
		addItem(new ValueEditControl<float>("forward max_vel",&walker->getWalkMC()->getCP().max_vel[WalkMC::CalibrationParam::forward]));
		addItem(new ValueEditControl<float>("reverse max_vel",&walker->getWalkMC()->getCP().max_vel[WalkMC::CalibrationParam::reverse]));
		addItem(new ValueEditControl<float>("strafe max_vel",&walker->getWalkMC()->getCP().max_vel[WalkMC::CalibrationParam::strafe]));
		addItem(new ValueEditControl<float>("rotate max_vel",&walker->getWalkMC()->getCP().max_vel[WalkMC::CalibrationParam::rotate]));
	}
	endSubMenu();
	
	addItem(new LoadWalkControl("Load Walk",walker->getWalkMC()));
	addItem(new SaveWalkControl("Save Walk",walker->getWalkMC()));
#endif
}

void WalkEdit::startSubMenu(const std::string& nm, const std::string& desc) {
	ControlBase * c = new ControlBase(nm,desc);
	menuStack.top()->pushSlot(c);
	menuStack.push(c);
}

void WalkEdit::addItem(ControlBase* control) {
	menuStack.top()->pushSlot(control);
}

void WalkEdit::endSubMenu() {
	menuStack.pop();
}

#endif

/*! @file
 * @brief Implements WalkEdit, which provides an editor for the CMPack-based Aibo walk engine
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
