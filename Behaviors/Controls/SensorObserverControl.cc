#include "SensorObserverControl.h"
#include "Events/EventRouter.h"
#include "Motion/MMAccessor.h"
#include "Shared/WorldState.h"
#include "Sound/SoundManager.h"
#include "Shared/Config.h"

#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_LEDS
#  include "Motion/LedMC.h"
#endif

#include <sstream>

REGISTER_CONTROL(SensorObserverControl,"Status Reports");

SensorObserverControl::SensorObserverControl()
	: ControlBase("Sensor Observer","Allows you to see/log the sensor data"), logfilePath(), logfile(), helpCtl(NULL), sensorCtl(NULL), buttonCtl(NULL), outputCtl(NULL), dutyCtl(NULL), consoleCtl(NULL), fileCtl(NULL), rtCtl(NULL), rtFreqCtl(NULL), numListeners(0)
{
	pushSlot(consoleCtl=new ToggleControl("Console Output","If selected, outputs events to the console"));
	pushSlot(fileCtl=new StringInputControl("[ ] File Output","Please enter the filename to log to (in /ms/...)"));
	pushSlot(rtCtl=new RTViewControl(this));
	pushSlot(rtFreqCtl=new StringInputControl("Real-time Update Period","Please enter the time between refreshes, in milliseconds; '0' to require manual refresh"));
	pushSlot(helpCtl=new ControlBase("Help"));
	pushSlot(NULL);
	helpCtl->pushSlot(new NullControl("The indexes listed here"));
	helpCtl->pushSlot(new NullControl("correspond to offsets"));
	helpCtl->pushSlot(new NullControl("given in the __Info.h"));
	helpCtl->pushSlot(new NullControl("file for the model"));
	helpCtl->pushSlot(new NullControl("robot which you are "));
	helpCtl->pushSlot(new NullControl("currently using."));
	pushSlot(sensorCtl=new ControlBase("Sensors:","Toggles logging of various sensors"));
	for(unsigned int i=0; i<NumSensors; i++)
		sensorCtl->pushSlot(new ToggleControl(sensorNames[i],"Turns logging of this sensor on/off"));
	pushSlot(buttonCtl=new ControlBase("Buttons:","Toggles logging of various buttons"));
	for(unsigned int i=0; i<NumButtons; i++)
		buttonCtl->pushSlot(new ToggleControl(buttonNames[i],"Turns logging of this button on/off"));
	pushSlot(outputCtl=new ControlBase("Outputs:","Toggles logging of various outputs' positions"));
	for(unsigned int i=0; i<NumOutputs; i++)
		outputCtl->pushSlot(new ToggleControl(outputNames[i],"Turns logging of this output's values on/off"));
	pushSlot(dutyCtl=new ControlBase("Duties:","Toggles logging of various PID joint's duty cycles"));
	for(unsigned int i=0; i<NumPIDJoints; i++)
		dutyCtl->pushSlot(new ToggleControl(outputNames[i+PIDJointOffset],"Turns logging of how hard this output is working on/off"));
	updateRT();
}

ControlBase* SensorObserverControl::doSelect() {
	ControlBase* ans=this;
	bool wasListening=(numListeners>0);
	for(unsigned int i=0; i<hilights.size(); i++) {
		unsigned int cur=hilights[i];
		if(options[cur]==fileCtl) {
			if(options[cur]->getName()[1]!=' ') {
				logfile.close();
				options[cur]->setName("[ ] File Output");
				numListeners--;
			} else {
				ans=options[cur];
				numListeners++;
			}
		} else if(options[cur]==consoleCtl) {
			options[cur]->doSelect();
			if(consoleCtl->getStatus())
				numListeners--;
			else
				numListeners++;
		} else { //(options[cur]==helpCtl || options[cur]==sensorCtl || options[cur]==buttonCtl || options[cur]==outputCtl || options[cur]==dutyCtl || options[cur]==consoleCtl)
			ans=options[cur];
		}
	}
	sndman->playFile(config->controller.select_snd);
	if(wasListening!=(numListeners>0)) {
		if(numListeners>0)
			erouter->addListener(this,EventBase::sensorEGID,SensorSrcID::UpdatedSID);
		else
			erouter->removeListener(this);
	}
	if(ans==this)
		refresh();
	return ans;
}

void SensorObserverControl::refresh() {
	//check for change in log file
	checkLogFile();
	//check for change in real-time refresh period
	if(rtFreqCtl->getLastInput().size()>0) {
		unsigned int fr=atoi(rtFreqCtl->getLastInput().c_str());
		rtFreqCtl->clearLastInput();
		if(fr!=0 && rtCtl!=NULL)
			rtCtl->setPeriod(fr<100?100:fr); //limit to minimum period of 100ms
	}
	ControlBase::refresh();
}

void SensorObserverControl::processEvent(const EventBase& event) {
	if(event.getGeneratorID()==EventBase::sensorEGID) {
		std::ostringstream logdata;
		for(unsigned int i=0; i<NumSensors; i++)
			if(ToggleControl * tgl=dynamic_cast<ToggleControl*>(sensorCtl->getSlots()[i]))
				if(tgl->getStatus())
					logdata << state->lastSensorUpdateTime << "\tSENSOR:\t" << i << '\t' << state->sensors[i] << '\n';
		for(unsigned int i=0; i<NumButtons; i++)
			if(ToggleControl * tgl=dynamic_cast<ToggleControl*>(buttonCtl->getSlots()[i]))
				if(tgl->getStatus())
					logdata << state->lastSensorUpdateTime << "\tBUTTON:\t" << i << '\t' << state->buttons[i] << '\n';
		for(unsigned int i=0; i<NumOutputs; i++)
			if(ToggleControl * tgl=dynamic_cast<ToggleControl*>(outputCtl->getSlots()[i]))
				if(tgl->getStatus())
					logdata << state->lastSensorUpdateTime << "\tOUTPUT:\t" << i << '\t' << state->outputs[i] << '\n';
		for(unsigned int i=0; i<NumPIDJoints; i++)
			if(ToggleControl * tgl=dynamic_cast<ToggleControl*>(dutyCtl->getSlots()[i]))
				if(tgl->getStatus())
					logdata << state->lastSensorUpdateTime << "\tDUTY:\t" << i << '\t' << state->pidduties[i] << '\n';
		if(consoleCtl->getStatus())
			sout->printf("%s",logdata.str().c_str());
		checkLogFile();
		if(logfile)
			logfile << logdata.str() << std::flush;
	} else {
		serr->printf("WARNING: Unexpected event: %s\n",event.getName().c_str());
	}
}

void SensorObserverControl::checkLogFile() {
	if(fileCtl->getLastInput()!=logfilePath) {
		logfile.close();
		logfilePath=fileCtl->getLastInput();
		logfile.clear();
		if(logfilePath.size()!=0) {
			sout->printf("Opening `%s'\n",config->portPath(logfilePath).c_str());
			logfile.open(config->portPath(logfilePath).c_str());
			if(!logfile.fail()) {
				std::string tmp=fileCtl->getName();
				tmp[1]='X';
				fileCtl->setName(tmp+": "+logfilePath);
			} else {
				serr->printf("Opening `%s' failed\n",config->portPath(logfilePath).c_str());
			}
		}
	}
}

void SensorObserverControl::updateRT() {
	const unsigned int FTOS_LEN=50;
	char ftos[FTOS_LEN];
	rtCtl->clearSlots();
	bool first=true;
	for(unsigned int i=0; i<NumSensors; i++)
		if(ToggleControl * tgl=dynamic_cast<ToggleControl*>(sensorCtl->getSlots()[i]))
			if(tgl->getStatus()) {
				if(first)
					rtCtl->pushSlot(new NullControl("SENSORS:"));
				first=false;
				snprintf(ftos,FTOS_LEN,"%s: %g",sensorNames[i],state->sensors[i]);
				rtCtl->pushSlot(new NullControl(ftos));
			}
	first=true;
	for(unsigned int i=0; i<NumButtons; i++)
		if(ToggleControl * tgl=dynamic_cast<ToggleControl*>(buttonCtl->getSlots()[i]))
			if(tgl->getStatus()) {
				if(first)
					rtCtl->pushSlot(new NullControl("BUTTONS:"));
				first=false;
				snprintf(ftos,FTOS_LEN,"%s: %g",buttonNames[i],state->buttons[i]);
				rtCtl->pushSlot(new NullControl(ftos));
			}
	first=true;
	for(unsigned int i=0; i<NumOutputs; i++)
		if(ToggleControl * tgl=dynamic_cast<ToggleControl*>(outputCtl->getSlots()[i]))
			if(tgl->getStatus()) {
				if(first)
					rtCtl->pushSlot(new NullControl("OUTPUTS:"));
				first=false;
				snprintf(ftos,FTOS_LEN,"%s: %g",outputNames[i],state->outputs[i]);
				rtCtl->pushSlot(new NullControl(ftos));
			}
	first=true;
	for(unsigned int i=0; i<NumPIDJoints; i++)
		if(ToggleControl * tgl=dynamic_cast<ToggleControl*>(dutyCtl->getSlots()[i]))
			if(tgl->getStatus()) {
				if(first)
					rtCtl->pushSlot(new NullControl("DUTIES:"));
				first=false;
				snprintf(ftos,FTOS_LEN,"%s: %g",outputNames[i],state->pidduties[i]);
				rtCtl->pushSlot(new NullControl(ftos));
			}
	if(rtCtl->slotsSize()==0)
		rtCtl->pushSlot(new NullControl("No sensors selected","You need to select which sensors to view"));
}

void SensorObserverControl::RTViewControl::refresh() {
	if(parent!=NULL)
		parent->updateRT();
	if(period!=0)
		erouter->addTimer(this,0,period);
	ControlBase::refresh();
}
void SensorObserverControl::RTViewControl::pause() {
	erouter->removeTimer(this);
	ControlBase::pause();
}
void SensorObserverControl::RTViewControl::deactivate() {
	erouter->removeTimer(this);
	ControlBase::deactivate();
}
/*! The change doesn't get picked up until next call to refresh() */
void SensorObserverControl::RTViewControl::setPeriod(unsigned int x) {
	period=x;
}


/*! @file
 * @brief Describes SensorObserverControl, which allows logging of sensor information to the console or file
 * @author ejt (Creator)
 */
