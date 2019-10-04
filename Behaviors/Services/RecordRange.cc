#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"
#include "Motion/PIDMC.h"
#include "IPC/SharedObject.h"

class RecordRange : public BehaviorBase {
public:
	RecordRange() : BehaviorBase("RecordRange"), pidID(MotionManager::invalid_MC_ID) {
		for(unsigned int i=0; i<NumOutputs; ++i)
			maxs[i]=mins[i]=0;
	}
	virtual void doStart() {
		BehaviorBase::doStart();
		pidID = motman->addPersistentMotion(SharedObject<PIDMC>(0));
		erouter->addListener(this,EventBase::sensorEGID);
	}
	virtual void doStop() {
		motman->removeMotion(pidID);
		for(unsigned int i=0; i<NumOutputs; ++i)
			std::cout << i << " " << outputNames[i] << ": " << mins[i] << ' ' << maxs[i] << std::endl;
		BehaviorBase::doStop();
	}
	virtual void doEvent() {
		for(unsigned int i=0; i<NumOutputs; ++i) {
			if(state->outputs[i]<mins[i])
				mins[i]=state->outputs[i];
			if(state->outputs[i]>maxs[i])
				maxs[i]=state->outputs[i];
		}
	}
protected:
	MotionManager::MC_ID pidID;
	float maxs[NumOutputs];
	float mins[NumOutputs];
};

REGISTER_BEHAVIOR_MENU_OPT(RecordRange,"Background Behaviors",BEH_NONEXCLUSIVE);

/*! @file
 * @brief Defines RecordRange, a small utility which disables all joint power and records the maximum and minimum positions
 * @author ejt (Creator)
 */
