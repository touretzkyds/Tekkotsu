#include "PostureEditor.h"
#include "Motion/MMAccessor.h"
#include "Motion/EmergencyStopMC.h"
#include "Motion/MotionSequenceMC.h"
#include "Sound/SoundManager.h"
#include "Events/EventRouter.h"
#include "ValueEditControl.h"
#include "NullControl.h"
#include "StringInputControl.h"
#include "FileInputControl.h"
#include "Shared/ProjectInterface.h"
#include "Motion/MotionPtr.h"
#include "Shared/Config.h"

#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_LEDS
#  include "Motion/LedMC.h"
#endif

PostureEditor::PostureEditor()
	: ControlBase("Posture Editor","Allows you to load, save, and numerically edit the posture"), 
	  pose(), reachID(invalid_MC_ID),
	  loadPose(NULL), disabledLoadPose(NULL), savePose(NULL), pauseCalled(false)
{
	// add load and save menus
	pushSlot(loadPose=new FileInputControl("Load Posture","Select a posture to open",config->portPath(config->motion.root)));
	loadPose->setFilter("*.pos");
	disabledLoadPose=new NullControl("[Load disabled by EStop]","Cannot load new postures while EStop is active");
	pushSlot(savePose=new StringInputControl("Save Posture","Please enter the filename to save to (in "+config->motion.root+")"));

	// add submenu for weight editors
	ControlBase * weights;
	pushSlot(weights=new ControlBase("Weights","Set the weights for outputs"));
	for(unsigned int i=0; i<NumOutputs; i++)
		weights->pushSlot(new ValueEditControl<float>(outputNames[i],&pose(i).weight));

	pushSlot(NULL); // a separator for clarity

	// add actual value editors
	for(unsigned int i=0; i<NumOutputs; i++)
		pushSlot(new ValueEditControl<float>(outputNames[i],&pose(i).value));
}

PostureEditor::~PostureEditor() {
	delete loadPose;
	delete disabledLoadPose;
	options[0]=NULL;
}

ControlBase *
PostureEditor::activate(MC_ID disp_id, Socket * gui) {
	//cout << "activate" << endl;
	if(reachID!=invalid_MC_ID) // was already activated
		return ControlBase::activate(disp_id,gui); // happens when estop is turned on, causing "reactivation"
	// start off with current pose
	pose.takeSnapshot();
	pose.setWeights(1);
#ifdef TGT_HAS_LEDS
	// clear the LEDs though
	for(unsigned int i=LEDOffset; i<LEDOffset+NumLEDs; i++)
		pose.setOutputCmd(i,0);
#endif
	// add it the motion sequence we'll be using to move to changes
	SharedObject<SmallMotionSequenceMC> reach;
	reachID=motman->addPersistentMotion(reach);
	// we'll need to know when estop is turned on or off
	erouter->addListener(this,EventBase::estopEGID);
	// call super class
	return ControlBase::activate(disp_id,gui);
}

void
PostureEditor::refresh() {
	//cout << "refresh" << endl;
	if(isEStopped()) {
		processEvent(EventBase(EventBase::timerEGID,1,EventBase::statusETID,0));
		erouter->addTimer(this,0,500);
		options[0]=disabledLoadPose;
	} else {
		for(unsigned int i=PIDJointOffset; i<PIDJointOffset+NumPIDJoints; ++i)
			if(state->pids[i][0]==0 && state->pids[i][1]==0 && state->pids[i][2]==0)
				pose(i).value=state->outputs[i];
		options[0]=loadPose;
	}
	if(loadPose->getLastInput().size()>0) {
		pose.loadFile(loadPose->getLastInput().c_str());
		updatePose(moveTime);
		loadPose->clearLastInput();
	} else if(savePose->getLastInput().size()>0) {
		// we just got back from the save menu
		std::string filename=savePose->getLastInput();
		if(filename.find(".")==std::string::npos)
			filename+=".pos";
		pose.saveFile(config->motion.makePath(filename).c_str());
		savePose->takeInput("");
	} else {
		updatePose(moveTime/2);
	}
	pauseCalled=false;
	ControlBase::refresh();
}

void
PostureEditor::pause() {
	//cout << "pause" << endl;
	refresh(); //one last time, in case this pause is due to un-estop putting Controller into low-profile mode
	pauseCalled=true;
	erouter->removeTimer(this);
	ControlBase::pause();
}

void
PostureEditor::deactivate() {
	//cout << "deactivate" << endl;
	//cout << "removeMotion(" << reachID << ")" << endl;
	motman->removeMotion(reachID);
	reachID=invalid_MC_ID;
	erouter->removeListener(this);
	erouter->removeTimer(this);
	ControlBase::deactivate();
}

void
PostureEditor::processEvent(const EventBase& e) {
	//cout << "processEvent(" << e.getName() << ")" << endl;
	if(e.getGeneratorID()==EventBase::estopEGID) {
		if(e.getTypeID()==EventBase::deactivateETID) {
			MMAccessor<SmallMotionSequenceMC>(reachID)->play();
			erouter->removeTimer(this);
			if(!pauseCalled)
				refresh();
		} else {
			if(!pauseCalled) {
				erouter->addTimer(this,0,500); // timer to allow updates on joint positions
				processEvent(EventBase(EventBase::timerEGID,0,EventBase::statusETID)); // but also do one right now
			}
		}
	} else if(e.getGeneratorID()==EventBase::timerEGID) {
#ifndef TGT_HAS_LEDS
		pose.takeSnapshot();
#else
		//doing a manual copy instead of just takeSnapshot() so we don't disturb the LED settings
		for(unsigned int i=0; i<LEDOffset; i++)
			pose(i).value=state->outputs[i];
		for(unsigned int i=LEDOffset+NumLEDs; i<NumOutputs; i++)
			pose(i).value=state->outputs[i];
#endif
		if(e.getSourceID()==0) // source==1 indicates it's a forged event sent from refresh -- don't inf. recurse
			refresh();
	} else {
		serr->printf("WARNING: PostureEditor unexpected event: %s\n",e.getName().c_str());
	}
}

bool
PostureEditor::isEStopped() {
	return ProjectInterface::estop->getStopped();
}

void
PostureEditor::updatePose(unsigned int delay) {
	//cout << "updatePose" << endl;
	bool paused=isEStopped();
	if ( reachID == MotionManager::invalid_MC_ID ) return;
	MMAccessor<SmallMotionSequenceMC> reach_acc(reachID);
	if(paused) {
		reach_acc->clear();
		return;
	}
	PostureEngine curpose;
	reach_acc->getPose(curpose);
	reach_acc->clear();
	//we want to keep the current pose to avoid any twitching
	reach_acc->setTime(1);
	for(unsigned int i=0; i<NumOutputs; i++) //only set weighted joints
		if(curpose(i).weight!=0)
			reach_acc->setOutputCmd(i,curpose(i));
	//now move to desired posture
	reach_acc->setTime(delay);
	reach_acc->setPose(pose);
	reach_acc->play();
}


/*! @file
 * @brief Describes PostureEditor, which allows numeric control of joints and LEDs
 * @author ejt (Creator)
 */
