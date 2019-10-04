//-*-c++-*-
#ifndef INCLUDED_ExploreMachine_h_
#define INCLUDED_ExploreMachine_h_

#include "Behaviors/StateNode.h"
#include "Behaviors/Nodes/WalkNode.h"
#include "Motion/MotionManager.h"

#include "Shared/RobotInfo.h"
#ifndef TGT_HAS_IR_DISTANCE
#error ExploreMachine requires a distance rangefinder via IRDistOffset
#endif

//! A state machine for exploring an environment (or searching for an object)
class ExploreMachine : public StateNode {
public:
	//!constructor
	ExploreMachine()
		: StateNode(), start(NULL), turn(NULL), walkid(MotionManager::invalid_MC_ID)
	{}

	//!constructor
	ExploreMachine(const std::string& nm)
		: StateNode(nm), start(NULL), turn(NULL), walkid(MotionManager::invalid_MC_ID)
	{}

	//!destructor, check if we need to call our teardown
	~ExploreMachine() {
		if(issetup)
			teardown();
	}

	virtual void setup();
	virtual void doStart();
	virtual void doStop();
	virtual void teardown();

	//! called each time the turn node is activated, sets a new random turn direction and speed
	virtual void doEvent();

protected:
	StateNode * start; //!< the node to begin within on doStart() (turn)
	WalkNode * turn; //!< walk node to use when turning
	MotionManager::MC_ID walkid; //!< we want to share a walk between turning and walking nodes

private:
	ExploreMachine(const ExploreMachine&); //!< don't use
	ExploreMachine operator=(const ExploreMachine&); //!< don't use
};

/*! @file
 * @brief Describes ExploreMachine, a state machine for exploring an environment (or searching for an object)
 * @author ejt (Creator)
 */

#endif
