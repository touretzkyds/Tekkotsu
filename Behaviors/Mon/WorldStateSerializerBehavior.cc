#include "WorldStateSerializerBehavior.h"
#include "Shared/WorldState.h"
#include "Wireless/Wireless.h"
#include "Shared/Config.h"
#include "Behaviors/Controller.h"
#include "Events/EventRouter.h"
#include "Shared/LoadSave.h"

// not using REGISTER_BEHAVIOR macro so we can specify name for Aibo3D to access
extern BehaviorSwitchControlBase * const autoRegisterWorldStateSerializer;
BehaviorSwitchControlBase * const autoRegisterWorldStateSerializer = BehaviorBase::registerControllerEntry<WorldStateSerializerBehavior>("World State Serializer","TekkotsuMon",BEH_NONEXCLUSIVE);

//! a little trick to get RobotName length under aperios, which has to use a string class instead of char*
inline size_t strlen(const std::string& st) { return st.size(); }

WorldStateSerializerBehavior::WorldStateSerializerBehavior()
: BehaviorBase("WorldStateSerializerBehavior"), wsJoints(NULL), wsPIDs(NULL), lastProcessedTime(0)
{
}

void WorldStateSerializerBehavior::doStart() {
	BehaviorBase::doStart(); // do this first
	
	const unsigned int transmitPIDSize=(NumPIDJoints*3)*LoadSave::getSerializedSize<float>()+2*LoadSave::getSerializedSize<unsigned int>();
	wsPIDs=wireless->socket(Socket::SOCK_STREAM, 1024, transmitPIDSize*4); // enough for 4 packets queued up
	wireless->setDaemon(wsPIDs);
	wireless->listen(wsPIDs, config->main.wspids_port);

	const size_t robotNameLen=strlen(RobotName)+1; //note the +1 to include '\0' at the end
	const size_t transmitJointsSize=(NumOutputs+NumSensors+NumButtons+NumPIDJoints)*LoadSave::getSerializedSize<float>()+6*LoadSave::getSerializedSize<unsigned int>()+robotNameLen;
	wsJoints=wireless->socket(Socket::SOCK_STREAM, 1024, transmitJointsSize*4); // enough for 4 packets queued up
	wireless->setDaemon(wsJoints);
	wireless->listen(wsJoints, config->main.wsjoints_port);
		
	Controller::loadGUI(getGUIType(),getGUIType(),getPort());
	
	erouter->addListener(this,EventBase::sensorEGID);
}

void WorldStateSerializerBehavior::doStop() {
	erouter->removeListener(this);
	
	Controller::closeGUI(getGUIType());
	
	wireless->setDaemon(wsJoints,false);
	wireless->close(wsJoints);
	wireless->setDaemon(wsPIDs,false);
	wireless->close(wsPIDs);
	
	BehaviorBase::doStop(); // do this last
}

void WorldStateSerializerBehavior::doEvent() {
	const size_t transmitPIDSize=(NumPIDJoints*3)*LoadSave::getSerializedSize<float>()+2*LoadSave::getSerializedSize<unsigned int>();
	if ((event->getTimeStamp() - lastProcessedTime) < config->main.worldState_interval) // not enough time has gone by
		return;
	lastProcessedTime = event->getTimeStamp();
	char *buf=(char*)wsPIDs->getWriteBuffer(transmitPIDSize);
	if(buf==NULL) {
		if(wireless->isConnected(wsPIDs->sock))
			std::cout << "WorldStateSerializer dropped pid at " << event->getTimeStamp() << std::endl;
	} else {
#if LOADSAVE_SWAPBYTES
		// if we need to swap bytes, need to use this slightly slower method
		unsigned int remain=transmitPIDSize;
		LoadSave::encodeInc(state->lastSensorUpdateTime,buf,remain);
		LoadSave::encodeInc(NumPIDJoints,buf,remain);
		for(unsigned int i=0; i<NumPIDJoints; ++i)
			for(unsigned int j=0; j<3; ++j)
				LoadSave::encodeInc(state->pids[i][j],buf,remain);
		ASSERT(remain==0,"buffer remains");
		wsPIDs->write(transmitPIDSize-remain);
#else
		// this is the original version, doesn't handle byte swapping, but faster
		copy(&buf,state->lastSensorUpdateTime);
		copy(&buf,NumPIDJoints);
		copy(&buf,state->pids,NumPIDJoints*3);
		wsPIDs->write(transmitPIDSize);
#endif
	}
	
	const size_t robotNameLen=strlen(RobotName)+1; //note the +1 to include '\0' at the end
	const size_t transmitJointsSize=(NumOutputs+NumSensors+NumButtons+NumPIDJoints)*LoadSave::getSerializedSize<float>()+6*LoadSave::getSerializedSize<unsigned int>()+robotNameLen;
	//std::cout << "transmitting at " << event->getTimeStamp() << " with " << (wsJoints->getAvailableSendSize() / float(transmitJointsSize)) << std::endl;
	buf=(char*)wsJoints->getWriteBuffer(transmitJointsSize);
	if(buf==NULL) {
		if(wireless->isConnected(wsJoints->sock))
			std::cout << "WorldStateSerializer dropped state at " << event->getTimeStamp() << std::endl;
	} else {
#if LOADSAVE_SWAPBYTES
		// if we need to swap bytes, need to use this slightly slower method
		unsigned int remain=transmitJointsSize;
		memcpy(buf,RobotName,robotNameLen);
		buf+=robotNameLen; remain-=robotNameLen;
		LoadSave::encodeInc(state->lastSensorUpdateTime,buf,remain);
		LoadSave::encodeInc(state->frameNumber,buf,remain);
		LoadSave::encodeInc(NumOutputs,buf,remain);
		for(unsigned int i=0; i<NumOutputs; ++i)
			LoadSave::encodeInc(state->outputs[i],buf,remain);
		LoadSave::encodeInc(NumSensors,buf,remain);
		for(unsigned int i=0; i<NumSensors; ++i)
			LoadSave::encodeInc(state->sensors[i],buf,remain);
		LoadSave::encodeInc(NumButtons,buf,remain);
		for(unsigned int i=0; i<NumButtons; ++i)
			LoadSave::encodeInc(state->buttons[i],buf,remain);
		LoadSave::encodeInc(NumPIDJoints,buf,remain);
		for(unsigned int i=0; i<NumPIDJoints; ++i)
			LoadSave::encodeInc(state->pidduties[i],buf,remain);
		ASSERT(remain==0,"buffer remains");
		wsJoints->write(transmitJointsSize-remain);
#else
		// this is the original version, doesn't handle byte swapping, but faster
		copy(&buf,RobotName,robotNameLen);
		copy(&buf,state->lastSensorUpdateTime);
		copy(&buf,state->frameNumber);
		copy(&buf,NumOutputs);
		copy(&buf,state->outputs,NumOutputs);
		copy(&buf,NumSensors);
		copy(&buf,state->sensors,NumSensors);
		copy(&buf,NumButtons);
		copy(&buf,state->buttons,NumButtons);
		copy(&buf,NumPIDJoints);
		copy(&buf,state->pidduties,NumPIDJoints);
		wsJoints->write(transmitJointsSize);
#endif
	}
}


/*! @file
* @brief Implements WorldStateSerializer, which copies WorldState into a buffer for transmission over the network
* @author alokl (Creator)
*/
