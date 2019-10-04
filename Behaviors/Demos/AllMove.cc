#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"
#include "Motion/SineMC.h"
#include "Motion/PostureMC.h"
#include "IPC/SharedObject.h"

//! This class moves all joints to 0°, and then uses a SineMC to wave them back and forth
/*! Each MC is used as 'fire-and-forget', so SharedObject<T> is demonstrated here
 *  since we don't need to access it ever again.  Otherwise you would use MotionPtr<T>
 *  instead (and probably store it as a member of the class instead of local to the function!) */
class AllMove : public BehaviorBase {
public:
	//! gets into '0' position at a reasonable speed
	virtual void doStart() {
		SharedObject<PostureMC> poser;
		for(unsigned int i=0; i<NumOutputs; i++) {
			poser->setOutputCmd(i,0);
			poser->setMaxSpeed(i,.5f);
		}
		// note the ID when we add the motion so we can subscribe to its completion
		MotionManager::MC_ID poseID = addMotion(poser, PRUNABLE);
		erouter->addListener(this,EventBase::motmanEGID,poseID,EventBase::deactivateETID);
	}
	
	//! when the poser gets into position (denoted by motman deactivate event upon pruning), start moving everything
	virtual void doEvent() {
		// just in case another behavior loads a motion and the poseID gets recycled:
		erouter->removeListener(this); // don't accept any more events
		
		SharedObject<SineMC> siner;
#ifdef TGT_HAS_LEDS
		const unsigned int LAST_JOINT=LEDOffset;
#else
		const unsigned int LAST_JOINT=NumOutputs;
#endif
		for(unsigned int i=0; i<LAST_JOINT; i++)
			siner->setParams(i,.5f,5000,0);
		addMotion(siner); // notice we don't need to removeMotion because BehaviorBase will clean up for us in stop()
	}
	
	// these allow the description to be shown as a tooltip in the ControllerGUI (not required, but nice)
	static std::string getClassDescription() { return "Moves all joints to 0°, and then uses a SineMC to wave them back and forth"; }
	virtual std::string getDescription() const { return getClassDescription(); }
};

REGISTER_BEHAVIOR_MENU(AllMove,DEFAULT_TK_MENU);

/*! @file
 * @brief Defines AllMove, a simple demonstration behavior which applies a SineMC to all outputs
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
