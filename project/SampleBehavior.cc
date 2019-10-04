#include "Shared/RobotInfo.h"
#if defined(TGT_HAS_BUTTONS) && defined(TGT_HAS_LEGS) && defined(TGT_HAS_LEDS)

#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"
#include "Motion/MotionPtr.h"
#include "Motion/LedMC.h"

// == added from MotionCommand tutorial == //
#include "SampleMC.h"
// ======== //

class SampleBehavior : public BehaviorBase {
protected:
	// This is a smart pointer which creates the MC and serializes access:
	MotionPtr<LedMC> leds;
	MotionPtr<SampleMC> mirror;
	
public:
	SampleBehavior() : BehaviorBase("SampleBehavior"),
		leds(), // initializer for leds, see MotionPtr docs for more constructor usage
		mirror()
	{}
	
	virtual void doStart() {
		// subscribe to all button events
		erouter->addListener(this, EventBase::buttonEGID);
		// handoff to MotionManager to make it 'active'
		addMotion(leds);
		
		// ======== //
		addMotion(mirror); // added from MotionCommand tutorial
		// ======== //
	}
	
	// Subscribed events will be sent here:
	virtual void doEvent() {
		// to be more general, let's check that it's the right event first:
		if(event->getGeneratorID()==EventBase::buttonEGID) {
			
			std::cout << "Received: " << event->getDescription() << std::endl;
			
			// set brightness of all LEDs to magnitude of event
			leds->set(AllLEDMask, event->getMagnitude());
			
			// == added from MotionCommand tutorial == //
			// this is a bit of a hack to map from button IDs to
			// whatever happens to be the corresponding numeric leg order
			mirror->setSource((LegOrder_t)event->getSourceID());
			// ======== //

		} else {
			// should never happen
			std::cout << "Bad Event:" << event->getName() << std::endl;
		}
	}
};

REGISTER_BEHAVIOR(SampleBehavior);

#endif // check for buttons, legs, and LEDs
