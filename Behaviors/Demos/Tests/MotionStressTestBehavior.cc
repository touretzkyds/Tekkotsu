#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_REK_LEGS

#include "Behaviors/BehaviorBase.h"
#include "Motion/MotionManager.h"
#include "Motion/MotionSequenceMC.h"
#include "Events/EventRouter.h"
#include "IPC/SharedObject.h"
#include <queue>


//! uses a separate MotionCommand for each of several joints to test for region leaks
class MotionStressTestBehavior : public BehaviorBase {
public:
	//! constructor
	MotionStressTestBehavior() : BehaviorBase("MotionStressTestBehavior"), nextLeg(RBkLegOrder), curMotions() {}

	virtual void doStart() {
		BehaviorBase::doStart(); // do this first

		SharedObject<SmallMotionSequenceMC> ms;
		ms->setTime(3000);
#ifdef TGT_HAS_LEGS
		ms->setOutputCmd(LFrLegOffset+ElevatorOffset,outputRanges[LFrLegOffset+ElevatorOffset][MaxRange]);
		ms->setOutputCmd(RFrLegOffset+ElevatorOffset,outputRanges[RFrLegOffset+ElevatorOffset][MaxRange]);
		ms->setOutputCmd(LBkLegOffset+ElevatorOffset,outputRanges[LBkLegOffset+ElevatorOffset][MaxRange]);
		ms->setOutputCmd(RBkLegOffset+ElevatorOffset,outputRanges[RBkLegOffset+ElevatorOffset][MaxRange]);
		ms->setOutputCmd(LFrLegOffset+KneeOffset,0);
		ms->setOutputCmd(RFrLegOffset+KneeOffset,0);
		ms->setOutputCmd(LBkLegOffset+KneeOffset,0);
		ms->setOutputCmd(RBkLegOffset+KneeOffset,0);
#endif
#ifdef TGT_HAS_HEAD
		for(unsigned int i=HeadOffset; i<HeadOffset+NumHeadJoints; i++)
			ms->setOutputCmd(i,0);
#endif
		MotionManager::MC_ID id=motman->addPrunableMotion(ms);
		curMotions.push(id);
		std::cout << get_time() << "\tAdded id " << id << std::endl;
		addMS(LFrLegOrder,3000);
		addMS(RFrLegOrder,4000);
		addMS(LBkLegOrder,5000);
		erouter->addListener(this,EventBase::motmanEGID);
	}

	virtual void doStop() {
		erouter->removeListener(this);
		while(!curMotions.empty()) {
			motman->removeMotion(curMotions.front());
			curMotions.pop();
		}
		BehaviorBase::doStop(); // do this last
	}

	virtual void doEvent() {
		if(event->getTypeID()==EventBase::deactivateETID) {
			if(event->getSourceID()!=curMotions.front()) {
				std::cout << event->getSourceID() << " is not mine or is out of order" << std::endl;
			} else {
				curMotions.pop();
			}
			std::cout << get_time() << "\t              Removed id " << event->getSourceID() << std::endl;
			addMS(nextLeg,3000);
			nextLeg=static_cast<LegOrder_t>((nextLeg+1)%4);
		}
	}

	//! creates and adds a new MotionSequenceMC
	void addMS(LegOrder_t leg,unsigned int delay=0) {
		unsigned int index=leg*JointsPerLeg+RotatorOffset;
		SharedObject<SmallMotionSequenceMC> ms;
		ms->setTime(delay);
		ms->setOutputCmd(index,outputRanges[index][MaxRange]);
		ms->advanceTime(2000);
		ms->setOutputCmd(index,outputRanges[index][MinRange]);
		ms->advanceTime(4000);
		ms->setOutputCmd(index,outputRanges[index][MaxRange]);
		MotionManager::MC_ID id=motman->addPrunableMotion(ms);
		curMotions.push(id);
		std::cout << get_time() << "\tAdded id " << id << std::endl;
	}

	static std::string getClassDescription() { return "uses a separate MotionCommand for each of several joints to test for region leaks"; }
	virtual std::string getDescription() const { return getClassDescription(); }
	
protected:
	LegOrder_t nextLeg; //!< the next leg to start moving
	std::queue<MotionManager::MC_ID> curMotions; //!< a queue of IDs of SmallMotionSequenceMC's
};

REGISTER_BEHAVIOR_MENU_OPT(MotionStressTestBehavior,"Background Behaviors/Debugging Tests",BEH_NONEXCLUSIVE);

#endif

/*! @file
 * @brief Defines MotionStressTestBehavior, which uses a separate MotionCommand for each of several joints to test for region leaks
 * @author ejt (Creator)
 */
