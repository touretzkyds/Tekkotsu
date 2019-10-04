#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_IR_DISTANCE

#include "PaceTargetsMachine.h"
#include "Behaviors/Transition.h"
#include "Behaviors/Nodes/WalkToTargetNode.h"
#include "Behaviors/Nodes/WalkNode.h"
#include "ExploreMachine.h"
#include "Behaviors/Transitions/TimeOutTrans.h"
#include "Behaviors/Transitions/VisualTargetTrans.h"
#include "Behaviors/Nodes/OutputNode.h"
#include "Behaviors/Nodes/MotionSequenceNode.h"
#include "Behaviors/Nodes/GroupNode.h"
#include "Sound/SoundManager.h"

REGISTER_BEHAVIOR_MENU(PaceTargetsMachine,DEFAULT_TK_MENU"/State Machine Demos");

void PaceTargetsMachine::setup() {
	StateNode::setup();

	const float turnAroundSpeed=1; //1 rad/sec
	//explore and chase have their own built-in speeds

	//*****  NODE SETUP *****//

	//turns in place
	WalkNode * turnAround = new WalkNode(0,0,turnAroundSpeed);
	turnAround->setName(getName()+"::TurnAround");
	addNode(turnAround);

	//two nodes running together -- explore will wander around, panhead will look around
	GroupNode * explGrp = new GroupNode(getName()+"::ExplGroup");
	addNode(explGrp);
	{
		ExploreMachine * explore = new ExploreMachine(explGrp->getName()+"::Explore");
		explGrp->addNode(explore);
		SmallMotionSequenceNode * panhead = new SmallMotionSequenceNode(explGrp->getName()+"::PanHead","pan_head.mot",true);
		explGrp->addNode(panhead);
	}

	//follow the ball
	WalkToTargetNode * chase = new WalkToTargetNode(ProjectInterface::visPinkBallSID);
	chase->setName(getName()+"::Chase");
	addNode(chase);


	//*****  TRANSITION SETUP *****//

	Transition * tmptrans=NULL;

	//if it sees pink, chase it
	explGrp->addTransition(tmptrans=new VisualTargetTrans(chase,ProjectInterface::visPinkBallSID));
	tmptrans->setSound("cutey.wav");

	//if you lose it, explore some more
	chase->addTransition(tmptrans=chase->newDefaultLostTrans(explGrp));
	tmptrans->setSound("whimper.wav");
	
	//if you get there, turn around
	chase->addTransition(tmptrans=chase->newDefaultCloseTrans(turnAround));
	tmptrans->setSound("fart.wav");
	
	//once you've turned around, explore
	turnAround->addTransition(tmptrans=new TimeOutTrans(explGrp,(unsigned int)(M_PI/turnAroundSpeed*1000))); //turn 180 degrees (aka PI radians)
	tmptrans->setSound("barkmed.wav");
	
	//preload the sounds so we don't pause on tranisitions
	sndman->loadFile("cutey.wav");
	sndman->loadFile("barkmed.wav");
	sndman->loadFile("whimper.wav");
	sndman->loadFile("fart.wav");

	//starts out exploring
	start=explGrp;
}

void PaceTargetsMachine::doStart() {
	StateNode::doStart();
	start->start();
}

void PaceTargetsMachine::teardown() {
	//release the sounds
	sndman->releaseFile("cutey.wav");
	sndman->releaseFile("barkmed.wav");
	sndman->releaseFile("whimper.wav");
	sndman->releaseFile("fart.wav");
	StateNode::teardown();
}

#endif

/*! @file
 * @brief Implements PaceTargetsMachine, a StateMachine for walking back and forth between two visual targets
 * @author ejt (Creator)
 */
