#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_IR_DISTANCE

#include "ExploreMachine.h"
#include "Behaviors/Nodes/WalkNode.h"
#include "Behaviors/Transitions/SmoothCompareTrans.h"
#include "Behaviors/Transitions/TimeOutTrans.h"
#include "Wireless/Socket.h"
#include "Shared/WorldState.h"

REGISTER_BEHAVIOR_MENU(ExploreMachine,DEFAULT_TK_MENU"/State Machine Demos");

void ExploreMachine::setup() {
	//cout << "Explore SETUP " << issetup << "...";
	SharedObject<WalkMC> walk;
	walkid=motman->addPersistentMotion(walk);

	WalkNode * move=NULL;
	addNode(move=new WalkNode(getName()+"::move",150,0,0));
	move->setMC(walkid);
	start=addNode(turn=new WalkNode(getName()+"::turn",0,0,0.5f));
	turn->setMC(walkid);

	move->addTransition(new SmoothCompareTrans<float>(turn,&state->sensors[IRDistOffset],CompareTrans<float>::LT,350,EventBase(EventBase::sensorEGID,SensorSrcID::UpdatedSID,EventBase::statusETID),.7f));
	turn->addTransition(new TimeOutTrans(move,2000));

	StateNode::setup();
	//cout << issetup << endl;
}

void ExploreMachine::doStart() {
	StateNode::doStart();
	start->start();
	//erouter->addListener(this,EventBase::sensorEGID,SensorSrcID::UpdatedSID);
	erouter->addListener(this,EventBase::stateMachineEGID,(size_t)turn,EventBase::activateETID);
}

void ExploreMachine::doStop() {
	erouter->removeListener(this);
	StateNode::doStop();
}

void ExploreMachine::teardown() {
	//cout << "Explore TEARDOWN " << issetup << "...";
	motman->removeMotion(walkid);
	StateNode::teardown();
	//cout << issetup << endl;
}

void ExploreMachine::doEvent() {
	//sout->printf("IR: %g\n",state->sensors[IRDistOffset]);
	float vel=rand()/(float)RAND_MAX*2.0f-1;
	if(vel<0)
		vel-=.25f;
	if(vel>0)
		vel+=.25f;
	turn->setVelocity(0,0,vel);
}

#endif

/*! @file
 * @brief Describes ExploreMachine, a state machine for exploring an environment (or searching for an object)
 * @author ejt (Creator)
 */
