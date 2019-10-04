#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"

class SensorMonitorBehavior : public BehaviorBase, public EventTrapper {
public:
	SensorMonitorBehavior() : BehaviorBase("SensorMonitorBehavior") {}
	virtual void doStart() {
		BehaviorBase::doStart();
		erouter->addTrapper(this,EventBase::sensorEGID);
		erouter->addTimer(this, 0, 2000, true);  // initial warning should come after just 2 secs.
	}
	
	virtual void doStop() {
		erouter->removeTrapper(this);
		BehaviorBase::doStop();
	}
	
	virtual void doEvent() {
		std::cout
		<< "********************************\n"
		<< "Warning: sensor events don't seem to be "
		<< "occurring normally.\n"
		<< "********************************\n";
		erouter->addTimer(this, 0, 30000, true);  // don't repeat more frequently than every 30 secs.
	}
	
	virtual bool trapEvent(const EventBase& ev) {
		if(ev.getGeneratorID()==EventBase::sensorEGID) {
			erouter->addTimer(this, 0, 30000, true); // reset timer
		}
		return false;
	}
};

REGISTER_BEHAVIOR_MENU_OPT(SensorMonitorBehavior,"Background Behaviors/System Daemons",BEH_NONEXCLUSIVE|BEH_START);
