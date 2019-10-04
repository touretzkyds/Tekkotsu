//-*-c++-*-
#ifndef INCLUDED_LEDCounterBehavior_h_
#define INCLUDED_LEDCounterBehavior_h_

#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"
#include "Motion/LedMC.h"
#include "Motion/PostureMC.h"
#include "Motion/MMAccessor.h"

//! Counts with LEDs on back button presses
class LEDCounterBehavior : public BehaviorBase {
public:
	LEDCounterBehavior()
		: BehaviorBase("LEDCounterBehavior"),
			inc(EventBase::buttonEGID,FrontBackButOffset,EventBase::activateETID),
			dec(EventBase::buttonEGID,RearBackButOffset,EventBase::activateETID),
			sw(EventBase::buttonEGID,MiddleBackButOffset,EventBase::activateETID),
			style(LedEngine::onedigit), cnt(0), led_id(MotionManager::invalid_MC_ID) {}
	virtual void doStart() {
		BehaviorBase::doStart();
		erouter->addListener(this,inc);
		erouter->addListener(this,dec);
		erouter->addListener(this,sw);
		led_id=motman->addPersistentMotion(SharedObject<LedMC>());
		motman->addPrunableMotion(SharedObject<PostureMC>("stand.pos"));
	}
	virtual void doStop() {
		motman->removeMotion(led_id);
		erouter->removeListener(this);
		BehaviorBase::doStop();
	}
	virtual void doEvent() {
		if(inc == *event) {
			MMAccessor<LedMC>(led_id)->displayNumber(++cnt,style);
		} else if(dec == *event) {
			MMAccessor<LedMC>(led_id)->displayNumber(--cnt,style);
		} else if(sw == *event) {
			style = (style==LedEngine::onedigit ? LedEngine::twodigit : LedEngine::onedigit);
			MMAccessor<LedMC>(led_id)->displayNumber(cnt,style);
		}			
	}

	static std::string getClassDescription() { return "Counts with LEDs on back button presses"; }
	virtual std::string getDescription() const { return getClassDescription(); }

protected:
	EventBase inc,dec,sw;
	LedEngine::numStyle_t style;
	int cnt;
	MotionManager::MC_ID led_id;
};

/*! @file
 * @brief Defines LEDCounterBehavior, which counts with LEDs on back button presses
 * @author ejt (Creator)
 */

#endif
