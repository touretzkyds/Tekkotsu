#include "PListSensorDriver.h"
#include "Shared/DynamicRobotState.h"
#include "Shared/RobotInfo.h"
#include "Events/MoCapEvent.h"
#include "Events/EventRouter.h"

using namespace std; 

const std::string PListSensorDriver::autoRegisterDriver = DeviceDriver::getRegistry().registerType<PListSensorDriver>("PListSensor");

void PListSensorDriver::deregisterSource() {
	DataStreamDriver::deregisterSource();
	for(unsigned int i=0; i<NumOutputs; ++i) {
		if(providing[i])
			ignoringOutput(i);
		providing[i] = false;
	}
}

bool PListSensorDriver::readData(std::istream& is) {
	DynamicRobotState drs;
	if(drs.loadStream(is,true)==0) {
		testCancel();
		//std::cerr << "ERROR PListSensorDriver::readData bad loadStream" << std::endl;
		return false;
	}
	if(!paceInput) {
		char firstByte;
		while(is.readsome(&firstByte,1)>0 && is) {
			is.putback(firstByte);
			if(drs.loadStream(is,true)==0) {
				testCancel();
				//std::cerr << "ERROR PListSensorDriver::readData bad loadStream" << std::endl;
				return false;
			}
		}
	}
	timestamp = get_time();
	
	{
		MarkScope l(getSensorWriteLock());
		typedef plist::DictionaryOf<plist::Primitive<float> >::const_iterator drs_iter;
		
		// process outputs
		for(drs_iter it=drs.outputs.begin(); it!=drs.outputs.end(); ++it) {
			unsigned int offset = capabilities.findOutputOffset(it->first);
			if(offset>=NumOutputs)
				continue;
			if(!providing[offset]) {
				providing[offset] = true;
				providingOutput(offset);
			}
			setOutputValue(offset,*it->second);
		}
		
		// process buttons
		for(drs_iter it=drs.buttons.begin(); it!=drs.buttons.end(); ++it) {
			unsigned int offset = capabilities.findButtonOffset(it->first);
			if(offset>=NumButtons)
				continue;
			setButtonValue(offset, *it->second);
		}
		
		// process sensors
		for(drs_iter it=drs.sensors.begin(); it!=drs.sensors.end(); ++it) {
			unsigned int offset = capabilities.findSensorOffset(it->first);
			if(offset>=NumSensors)
				continue;
			setSensorValue(offset, *it->second);
		}
		
		// process pidduties
		for(drs_iter it=drs.torques.begin(); it!=drs.torques.end(); ++it) {
			unsigned int offset = capabilities.findOutputOffset(it->first);
			if(offset>=NumOutputs)
				continue;
			setPIDDutyValue(offset, *it->second);
		}
	}
	
	if(drs.framePositions.size()>0 || drs.frameOrientations.size()>0) {
		MoCapEvent * mce = new MoCapEvent(instanceName,reinterpret_cast<size_t>(this));
		for(plist::DictionaryOf<plist::Point>::const_iterator it=drs.framePositions.begin(); it!=drs.framePositions.end(); ++it) {
			unsigned int idx = capabilities.findFrameOffset(it->first);
			if(idx!=-1U)
				mce->positions.insert(std::pair<unsigned int, fmat::Column<3> >(idx,it->second->exportTo<fmat::Column<3> >()));
		}
		for(plist::DictionaryOf<plist::Point>::const_iterator it=drs.frameOrientations.begin(); it!=drs.frameOrientations.end(); ++it) {
			unsigned int idx = capabilities.findFrameOffset(it->first);
			if(idx!=-1U)
				mce->orientations.insert(std::pair<unsigned int, fmat::Quaternion>(idx,fmat::Quaternion::fromAxis(*it->second)));
		}
		erouter->requeueEvent(mce);
	}
	
	return true;
}


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
