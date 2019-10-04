#include "MCNode.h"
#include "Events/EventRouter.h"
#include "Shared/MarkScope.h"

#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_TAIL
#include "TailWagNode.h"
REGISTER_BEHAVIOR_MENU_OPT(TailWagNode,DEFAULT_TK_MENU,BEH_NONEXCLUSIVE);
#endif

const char MCNodeBase::defName[]="MCNode";
const char MCNodeBase::defDesc[]="A generic wrapper for any MotionCommand";

// These externs are declared in their respective header files, but
// defined here instead of corresponding .cc files to avoid file bloat
// (there would be nothing else in their .cc files)

//! name for HeadPointerNode to pass as template argument
extern const char defHeadPointerNodeName[]="HeadPointerNode";
//! description for HeadPointerNode to pass as template argument
extern const char defHeadPointerNodeDesc[]="Manages a HeadPointerMC to look in a given direction each time the node is activated";

//! name for PIDNode to pass as template argument
extern const char defPIDNodeName[]="PIDNode";
//! description for PIDNode to pass as template argument
extern const char defPIDNodeDesc[]="Manages a PIDMC to set joint power levels when the node is activated";

//! name for CrabArmNode to pass as template argument
extern const char defCrabArmNodeName[]="CrabArmNode";
//! description for CrabArmNode to pass as template argument
extern const char defCrabArmNodeDesc[]="Manages a CrabArmMC to reach in a given direction each time the node is activated";

//! name for ArmNode to pass as template argument
extern const char defArmNodeName[]="ArmNode";
//! description for ArmNode to pass as template argument
extern const char defArmNodeDesc[]="Manages an ArmMC to reach in a given direction each time the node is activated";

//! name for TailWagNode to pass as template argument
extern const char defTailWagNodeName[]="TailWagNode";
//! description for TailWagNode to pass as template argument
extern const char defTailWagNodeDesc[]="Wags the tail for as long as the state is active";

//! name for PostureNode to pass as template argument
extern const char defPostureNodeName[]="PostureNode";
//! description for PostureNode to pass as template argument
extern const char defPostureNodeDesc[]="Moves the body to the specified posture";

//! name for MotionSequenceNode to pass as template argument
extern const char defMotionSequenceNodeName[]="MotionSequenceNode";
//! description for MotionSequenceNode to pass as template argument
extern const char defMotionSequenceNodeDesc[]="Moves the body through the specified motion sequence";

//! name for DynamicMotionSequenceNode to pass as template argument
extern const char defDynamicMotionSequenceNodeName[]="DynamicMotionSequenceNode";
//! description for DynamicMotionSequenceNode to pass as template argument
extern const char defDynamicMotionSequenceNodeDesc[]="Moves the body through the specified dynamic motion sequence";

//! name for WalkNode to pass as template argument
extern const char defWalkNodeName[]="WalkNode";
//! description for WalkNode to pass as template argument
extern const char defWalkNodeDesc[]="Manages a WalkMC node to walk in a direction each time the node is activated.";

//! name for WaypointWalkNode to pass as template argument
extern const char defWaypointWalkNodeName[]="WaypointWalkNode";
//! description for WaypointWalkNode to pass as template argument
extern const char defWaypointWalkNodeDesc[]="Manages a WaypointWalkMC to perform a waypoint walk each time the node is activated.";


void MCNodeBase::doEvent() {
	if(mcCompletes && event->getGeneratorID()==EventBase::motmanEGID && event->getSourceID()==mc_id)
		postStateCompletion();
	else
		StateNode::doEvent();
}

void MCNodeBase::stop() {
	if(hasPrivateMC()) {
		motman->removeMotion(mc_id);
		mc_id=MotionManager::invalid_MC_ID;
	}
	StateNode::stop(); // do this last (required)
}

void MCNodeBase::setMC(MotionManager::MC_ID new_mc_id) {
	if ( new_mc_id != MotionManager::invalid_MC_ID ) {
		erouter->removeListener(this,EventBase::motmanEGID, mc_id);
		if( mc != NULL ) {
			if ( mc_id != MotionManager::invalid_MC_ID )
				motman->removeMotion(mc_id);
			delete mc;
			mc=NULL;
		}
		mc_id = new_mc_id;
		if ( isActive() )
			erouter->addListener(this,EventBase::motmanEGID,mc_id,EventBase::statusETID);
	}
}

void MCNodeBase::setPriority(const float p) {
  priority = p;
  if ( mc_id != MotionManager::invalid_MC_ID )
    motman->setPriority(mc_id, priority);
}

void MCNodeBase::motionFails() {
  // Shut down the motion command: copied from stop() because we don't
  // want to call StateNode::stop() here; that would deactivate any
  // outgoing failure transition
  if(hasPrivateMC()) {
    motman->removeMotion(mc_id);
    mc_id=MotionManager::invalid_MC_ID;
  } else if(mc_id!=MotionManager::invalid_MC_ID) {
    motman->setPriority(mc_id,MotionManager::kIgnoredPriority);
  }
  postStateFailure();
}

/*! @file
 * @brief Implement's MCNode's default name and description strings (the class is templated, so everything else has to go in the header)
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
