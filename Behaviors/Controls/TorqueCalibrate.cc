#define TORQUE_CALIBRATE_NO_WARN_NOOP
#include "TorqueCalibrate.h"
#undef TORQUE_CALIBRATE_NO_WARN_NOOP
#include "Behaviors/Controls/NullControl.h"
#include "Shared/WorldState.h"
#include "Shared/Config.h"
#include "Events/EventRouter.h"
#include "Motion/MMAccessor.h"
#include "Motion/PIDMC.h"
#include "Motion/MotionSequenceMC.h"
#include "Sound/SoundManager.h"
#include "IPC/SharedObject.h"
#include <fstream>
#include <cmath>

//better to put this here instead of the header
using namespace std; 


//***************************//
//***** TorqueCalibrate *****//
//***************************//

void TorqueCalibrate::record(unsigned int joint, float sensorDist, float maxDuty, float maxForce) const {
	std::ofstream log(filename.c_str(),std::ios::app);
	if(!log) {
		serr->printf("ERROR: could not open %s for writing log\n",filename.c_str());
		sndman->playFile("fart.wav");
	} else {
		log << joint << '\t' << sensorDist << '\t' << maxDuty << '\t' << maxForce << std::endl;
		sndman->playFile("camera.wav");
	}
	std::cout << "DATA: " << joint << '\t' << sensorDist << '\t' << maxDuty << '\t' << maxForce << std::endl;
}

void TorqueCalibrate::refresh() {
	if(filenameInput->getLastInput()!=filename) {
		filename=filenameInput->getLastInput();
		std::string::size_type f=filename.rfind("/");
		filenameInput->setName("Storage: "+filename.substr(f==string::npos?0:f+1));
		std::string desc="Location where data will be appended to any previous contents";
		filenameInput->setDescription(desc+": "+filename);
		std::ofstream log(filename.c_str(),std::ios::app);
		if(!log)
			serr->printf("ERROR: could not open %s for writing log\n",filename.c_str());
	}
	ControlBase::refresh();
}


//******************************************************//
//***** TorqueCalibrate::TakeMeasurementControl *****//
//******************************************************//

ControlBase * TorqueCalibrate::TakeMeasurementControl::activate(MC_ID disp_id, Socket * gui) {
#ifdef TGT_HAS_BUTTONS
#  ifdef TGT_ERS7
	erouter->addListener(this,EventBase::buttonEGID,FrontBackButOffset,EventBase::activateETID);
	erouter->addListener(this,EventBase::buttonEGID,MiddleBackButOffset,EventBase::activateETID);
	erouter->addListener(this,EventBase::buttonEGID,RearBackButOffset,EventBase::activateETID);
#  else
	unsigned int offset = capabilities.findButtonOffset("BackBut");
	if(offset==-1U)
		offset=0; // we don't know what button to use, just take 'first' one
	erouter->addListener(this,EventBase::buttonEGID,offset,EventBase::activateETID);
#  endif
#endif
	cstate=ZERO_JOINT;
	SharedObject<PIDMC> pidmc(0);
	pidID=motman->addPersistentMotion(pidmc,MotionManager::kHighPriority);
	return ControlBase::activate(disp_id,gui);
}

void TorqueCalibrate::TakeMeasurementControl::processEvent(const EventBase& event) {
	if(cstate==ZERO_JOINT && event.getGeneratorID()==EventBase::buttonEGID) {
		basePosition=state->outputs[joint];
		transition(RECORD_POSITION);
	} else if(cstate==DO_PULSE && event.getGeneratorID()==EventBase::sensorEGID) {
		if(std::abs(state->pidduties[joint-PIDJointOffset]) > maxDuty)
			maxDuty=std::abs(state->pidduties[joint-PIDJointOffset]);
		std::cout << "Duty: " << state->pidduties[joint-PIDJointOffset] << std::endl;
	} else if(cstate==DO_PULSE && event.getGeneratorID()==EventBase::timerEGID) {
		erouter->removeListener(this,EventBase::sensorEGID);
	} else if(cstate==DO_PULSE && event.getGeneratorID()==EventBase::motmanEGID) {
		pulseID=invalid_MC_ID;
		std::cout << "Max duty: " << maxDuty << std::endl;
		transition(RECORD_FORCE);
	} else {
		std::cerr << "Unhandled event " << event.getName() << std::endl;
	}
}

void TorqueCalibrate::TakeMeasurementControl::refresh() {
	clearSlots();
	switch(cstate) {
		case ZERO_JOINT:
			pushSlot(new NullControl("Position the joint"));
			pushSlot(new NullControl("and press a back button"));
			break;
		case RECORD_POSITION:
			pushSlot(new NullControl("What is the length of"));
			pushSlot(new NullControl("the lever arm? (cm)"));
			pushSlot(new NullControl("(Dist. from axis of"));
			pushSlot(new NullControl("rot. to force sensor)"));
			break;
		case INPUT_PULSE:
			pushSlot(new NullControl("Enter the position"));
			pushSlot(new NullControl("offset to attempt"));
			pushSlot(new NullControl("(radians -- bigger"));
			pushSlot(new NullControl("offset means apply"));
			pushSlot(new NullControl("more force...)"));
			break;
		case DO_PULSE:
			pushSlot(new NullControl("Running..."));
			break;
		case RECORD_FORCE: {
			char res[256];
			snprintf(res,256,"Max duty was: %g",maxDuty);
			pushSlot(new NullControl(res));
			pushSlot(NULL);
			pushSlot(new NullControl("Enter the maximum"));
			pushSlot(new NullControl("force (N)"));
			break;
		}
	}
	ControlBase::refresh();
}

ControlBase * TorqueCalibrate::TakeMeasurementControl::takeInput(const std::string& msg) {
	switch(cstate) {
		case ZERO_JOINT:
		case DO_PULSE:
			break;
		case RECORD_POSITION:
			sensorDist=(float)atof(msg.c_str());
			if(sensorDist==0) {
				sndman->playFile("fart.wav");
				return NULL;
			}
			transition(INPUT_PULSE);
			break;
		case INPUT_PULSE: {
			float offset=(float)atof(msg.c_str());
			SharedObject<TinyMotionSequenceMC> ms;
			ms->advanceTime(350);
			if(offset>.1) {
				// move slowly at first in case not quite against sensor
				ms->setOutputCmd(joint,basePosition+offset/4);
				ms->advanceTime(300);
				ms->setOutputCmd(joint,basePosition+offset/2);
				ms->advanceTime(200);
				// hard push at the end
				ms->setOutputCmd(joint,basePosition+offset);
				ms->advanceTime(500);
			} else {
				ms->setOutputCmd(joint,basePosition+offset);
				ms->advanceTime(700);
			}
			ms->setOutputCmd(joint,basePosition+offset);
			ms->advanceTime(300);
			erouter->addTimer(this,0,ms->getTime(),false);
			ms->setOutputCmd(joint,basePosition);
			ms->advanceTime(200);
			ms->setOutputCmd(joint,basePosition);
			pulseID=motman->addPrunableMotion(ms);
			transition(DO_PULSE);
		} break;
		case RECORD_FORCE: {
			float f=(float)atof(msg.c_str());
			if(f!=0)
				parent.record(joint,sensorDist,maxDuty,f);
			else
				sndman->playFile("fart.wav");
			transition(INPUT_PULSE);
		} break;
	}
	return this;
}

void TorqueCalibrate::TakeMeasurementControl::deactivate() {
	erouter->remove(this);
	motman->removeMotion(pidID);
	pidID=invalid_MC_ID;
	if(pulseID!=invalid_MC_ID) {
		motman->removeMotion(pulseID);
		pulseID=invalid_MC_ID;
	}
	ControlBase::deactivate();
}

void TorqueCalibrate::TakeMeasurementControl::transition(State_t newstate) {
	MMAccessor<PIDMC>(pidID)->setAllPowerLevel(1);
	erouter->removeListener(this);
	cstate=newstate;
	if(cstate==RECORD_POSITION || cstate==INPUT_PULSE || cstate==RECORD_FORCE)
				sndman->playFile("ping.wav");
	else
				sndman->playFile("barkhigh.wav");
	refresh();
	if(cstate==DO_PULSE) {
				maxDuty=0;
				float pidSetting[] = {DefaultPIDs[joint][0],0,0};
				MMAccessor<PIDMC>(pidID)->setPID(joint,pidSetting);
				erouter->addListener(this,EventBase::sensorEGID);
				erouter->addListener(this,EventBase::motmanEGID,pulseID,EventBase::deactivateETID);
	}
}


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
