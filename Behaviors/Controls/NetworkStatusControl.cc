#ifdef PLATFORM_APERIOS

#include "ControlBase.h"
#include "Motion/MMAccessor.h"
#include "NullControl.h"
#include "Events/EventRouter.h"
#include "Wireless/Wireless.h"

#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_LEDS
#  include "Motion/LedMC.h"
#endif

#ifdef PLATFORM_APERIOS
#  include <ERA201D1.h>
#endif

//! will display current network status such as wireless signal strength
class NetworkStatusControl : public ControlBase, public EventListener {
public:
	//! constructor
	NetworkStatusControl() : ControlBase("Network Status","Displays current network status such as wireless signal strength"), EventListener() {}
	
	//! constructor, pass custom name
	NetworkStatusControl(const std::string& nm) : ControlBase(nm,"Displays current network status such as wireless signal strength"), EventListener() {}
	
	//! stops listening for power events and sets display to invalid
	virtual void pause() {
		erouter->remove(this);
		ControlBase::pause();
	}
	//! calls report()
	virtual void refresh() {
		erouter->addTimer(this,0,1000,true);
#ifdef PLATFORM_APERIOS
		EtherDriverGetWLANStatisticsMsg msg;
		EtherStatus status = ERA201D1_GetWLANStatistics(&msg);
		clearSlots();
		const unsigned int TMP_SIZE=255;
		char tmp[TMP_SIZE];
		if (status == ETHER_OK) {
			snprintf(tmp,TMP_SIZE,"link: %d",msg.statistics.link);
			pushSlot(new NullControl(tmp));
			snprintf(tmp,TMP_SIZE,"signal: %d",msg.statistics.signal);
			pushSlot(new NullControl(tmp));
			snprintf(tmp,TMP_SIZE,"noise: %d",msg.statistics.noise);
			pushSlot(new NullControl(tmp));
			snprintf(tmp,TMP_SIZE,"invalidIDCount: %u",(unsigned int)msg.statistics.invalidIDCount);
			pushSlot(new NullControl(tmp));
			snprintf(tmp,TMP_SIZE,"invalidEncCount: %u",(unsigned int)msg.statistics.invalidEncCount);
			pushSlot(new NullControl(tmp));
			snprintf(tmp,TMP_SIZE,"invalidMiscCount: %u",(unsigned int)msg.statistics.invalidMiscCount);
			pushSlot(new NullControl(tmp));
			MMAccessor<LedMC> leds_acc(display_id);
			leds_acc->displayPercent(msg.statistics.signal/100.0,LedEngine::major,LedEngine::major);
		} else {
			snprintf(tmp,TMP_SIZE,"ERA201D1_GetWLANSettings() FAILED %d", status);
			pushSlot(new NullControl("Error Accessing wireless statistics",tmp));
			serr->printf("%s",tmp);
		}
#else
		// Other platforms here?
#endif
		ControlBase::refresh();
	}
	virtual void deactivate() {
		erouter->remove(this);
		ControlBase::deactivate();
	}
	//! refresh the control whenever an event is received
	virtual void processEvent(const EventBase& /*event*/) {
		refresh();
	}
	
};

REGISTER_CONTROL(NetworkStatusControl,"Status Reports");

#endif

/*! @file
 * @brief Defines NetworkStatusControl, which will display current network status such as wireless signal strength
 * @author Sony OPEN-R Sample Code
 * @author klibby and lkirtane (original port)
 * @author ejt (UI repackaging)
 */
