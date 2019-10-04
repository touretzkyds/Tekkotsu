#include "CameraStreamBehavior.h"
#include "Wireless/Socket.h"
#include "Motion/PostureEngine.h"
#include "Events/EventRouter.h"
#include "Shared/LoadSave.h"
#include "Shared/Config.h"
#include "Shared/WorldState.h"
#include "Shared/debuget.h"

using namespace std;

void CameraStreamBehavior::doEvent() {
	ASSERTRET(event->getGeneratorID()==EventBase::sensorEGID,"unexpected event");
	sendSensors();
}

int CameraStreamBehavior::receiveData(char* data, unsigned int len) {
	std::string s(data,len);
	//cout << "Console Received: " << s << endl;

	static std::string incomplete;

	//pass a line at a time to the controller
	while(s.size()>0) {
		std::string::size_type endline=s.find('\n');
		if(endline==std::string::npos) {
			incomplete+=s;
			return 0;
		}

		//strip a \r\n or a \n
		if(endline>0 && s[endline-1]=='\r')
			incomplete+=s.substr(0,endline-1);
		else
			incomplete+=s.substr(0,endline);
		
		//is now complete
		if(incomplete=="refreshSensors") {
			sendSensors();
		} else if(incomplete=="startSensors") {
			if(sensorListeners++ == 0)
				erouter->addListener(this,EventBase::sensorEGID);
		} else if(incomplete=="stopSensors") {
			if(sensorListeners==0)
				serr->printf("WARNING: %s sensor listener underflow",getName().c_str());
			else if(--sensorListeners == 0)
				erouter->removeListener(this,EventBase::sensorEGID);
		}
		incomplete.erase();
		s=s.substr(endline+1);
	}
	return 0;
}

void CameraStreamBehavior::sendSensors() {
  if ((state->lastSensorUpdateTime - lastProcessedTime) < config->main.worldState_interval) // not enough time has gone by
    return;
  lastProcessedTime = state->lastSensorUpdateTime;

	ASSERT(LoadSave::stringpad==sizeof(unsigned int)+sizeof(char),"LoadSave::encode(string) format has changed?");
	PostureEngine pose;
	pose.takeSnapshot();
	pose.setWeights(1);
	pose.setSaveFormat(true,state);
	unsigned int len=pose.getBinSize()+LoadSave::stringpad;
	byte* buf=curSocket->getWriteBuffer(len);
	if(buf==NULL) {
		serr->printf("Unable to serialize sensor data for camera image -- network overflow");
		return;
	}
	unsigned int used;
	if((used=pose.saveBuffer((char*)buf+sizeof(unsigned int),len-LoadSave::stringpad))==0) {
		cerr << "An error occured during sensor serialization" << endl;
		curSocket->write(0);
		return;
	}
	if(used!=len-LoadSave::stringpad-1)
		std::cout << "Warning: used==" << used << " len==" << len << std::endl;
	//add the LoadSave fields (prepend length, append '\0')
	len=LoadSave::encode(used,reinterpret_cast<char*>(buf),LoadSave::getSerializedSize(used));
	if(len==0) {
		cerr << "An error occured during serialization of buffer length" << endl;
		curSocket->write(0);
		return;
	}
	buf[used+sizeof(used)]='\0';
	curSocket->write(used+LoadSave::stringpad);
	//std::cout << "Sent sensors " << used << std::endl;
}



/*! @file
 * @brief Defines CameraStreamBehavior, which is the base class for camera streaming communication classes, handles upstream communication
 * @author ejt (Creator)
 */
