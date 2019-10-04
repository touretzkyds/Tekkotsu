#include "Shared/RobotInfo.h"
#ifdef TGT_IS_CHIARA

#include "ChiaraDemo.h"
#include "Shared/plist.h"
#include "Motion/MotionSequenceMC.h"
#include "Motion/PostureMC.h"
#include "Behaviors/Nodes/MCNode.h"
#include "Behaviors/Nodes/MotionSequenceNode.h"
#include "Behaviors/Nodes/SoundNode.h"
#include "Behaviors/Transitions/TimeOutTrans.h"
#include "Behaviors/Transitions/CompletionTrans.h"
#include "Behaviors/Transitions/RandomTrans.h"
#include "Shared/Config.h"
#include "Motion/MotionPtr.h"

REGISTER_BEHAVIOR_MENU(ChiaraDemo,DEFAULT_TK_MENU"/Chiara Demos");

using namespace std; 

class XWalkParamNode : public MCNode<XWalkMC>, public XWalkParameters {
public:
	//! default constructor, use type name as instance name
	XWalkParamNode()
	: MCNode<XWalkMC>(false), XWalkParameters()
	{
		loadFile(::config->motion.makePath("xwalk.plist").c_str());
	}
	
	//! constructor, take an instance name
	XWalkParamNode(const std::string& nm, MotionManager::MC_ID wid = MotionManager::invalid_MC_ID)
	: MCNode<XWalkMC>(nm,false), XWalkParameters()
	{
		loadFile(::config->motion.makePath("xwalk.plist").c_str());
		if(wid!=MotionManager::invalid_MC_ID)
			setMC(wid);
	}
	
	virtual void doStart() {
		MCNode<XWalkMC>::doStart();
		MMAccessor<XWalkMC> walk = getMC();
		walk->transitionDuration = transitionDuration; // assign this first so it applies to other settings
		(XWalkParameters&)*walk = *this;
		erouter->addTimer(this, 0, transitionDuration, false);
	}
	
	virtual void doEvent() {
		postStateCompletion();
	}
};

class RandomCamNode : public StateNode {
public:
	RandomCamNode() : StateNode(), cam() {}
	
	virtual void doStart() {
		StateNode::doStart();
		addMotion(cam);
		erouter->addTimer(this, 0, 1000, false);
	}
	
	virtual void doEvent() {
		cam->clear();
		unsigned int dur = static_cast<unsigned int>(r(300,2000));
		cam->setTime(dur);
		cam->setOutputCmd(HeadPanOffset,r(-(float)M_PI/4,(float)M_PI/4));
		cam->setOutputCmd(HeadTiltOffset,r(-(float)M_PI/5,(float)M_PI/3));
		cam->play();
		unsigned int delay = dur + static_cast<unsigned int>(r(2000,8000));
		erouter->addTimer(this, 0, delay, false);
	}
	
protected:
	static float r(float min, float max) {
		float rv = random()/(float)(-1U>>1);
		return rv*(max-min)+min;
	}
	
	MotionPtr<TinyMotionSequenceMC> cam;
};

void ChiaraDemo::setup() {
	StateNode::setup();
	
	motman->addPersistentMotion(walk);
	
	XWalkParamNode * reset = addNode(new XWalkParamNode("Reset",walk->getID()));
	
	// bouncing
	StateNode * bounce=NULL;
	{
		const float height = 100;
		const float sway = 40;
		const float off = 20;
		
		XWalkParamNode * bounceL = addNode(new XWalkParamNode("BounceLeft",walk->getID()));
		bounceL->groundPlane[3]=-height;
		bounceL->offsetX=off;
		bounceL->offsetY=sway;
		
		XWalkParamNode * bounceReset = addNode(new XWalkParamNode("BounceReset",walk->getID()));
		
		XWalkParamNode * bounceR = addNode(new XWalkParamNode("BounceRight",walk->getID()));
		bounceR->groundPlane[3]=-height;
		bounceL->offsetX=off;
		bounceR->offsetY=-sway;

		XWalkParamNode * bounceReset2 = addNode(new XWalkParamNode("BounceReset2",walk->getID()));
		
		XWalkParamNode * bounceC = addNode(new XWalkParamNode("BounceCenter",walk->getID()));
		bounceC->groundPlane[3]=-150;

		bounce=bounceL;
		bounceL->addTransition(new CompletionTrans(bounceReset));
		bounceReset->addTransition(new CompletionTrans(bounceR));
		bounceR->addTransition(new CompletionTrans(bounceReset2));
		bounceReset2->addTransition(new CompletionTrans(bounceC));
		bounceC->addTransition(new TimeOutTrans(reset,6000));
	}
	
	// swiveling
	StateNode * swivel;
	{
		const float mag = 9*(float)M_PI/180;
		const unsigned int dur=5000;
		
		XWalkParamNode * swivelD = addNode(new XWalkParamNode("SwivelD",walk->getID()));
		(fmat::rotationYN<4>(-mag) * fmat::pack(0,0,1,-50)).exportTo(swivelD->groundPlane);
		swivelD->transitionDuration = dur;

		XWalkParamNode * swivelL = addNode(new XWalkParamNode("SwivelL",walk->getID()));
		(fmat::rotationXN<4>(mag) * fmat::pack(0,0,1,-50)).exportTo(swivelL->groundPlane);
		swivelL->transitionDuration = dur;

		XWalkParamNode * swivelU = addNode(new XWalkParamNode("SwivelU",walk->getID()));
		(fmat::rotationYN<4>(mag) * fmat::pack(0,0,1,-50)).exportTo(swivelU->groundPlane);
		swivelU->transitionDuration = dur;

		XWalkParamNode * swivelR = addNode(new XWalkParamNode("SwivelR",walk->getID()));
		(fmat::rotationXN<4>(-mag) * fmat::pack(0,0,1,-50)).exportTo(swivelR->groundPlane);
		swivelR->transitionDuration = dur;

		XWalkParamNode * swivelD2 = addNode(new XWalkParamNode("SwivelD2",walk->getID()));
		(fmat::rotationYN<4>(-mag) * fmat::pack(0,0,1,-50)).exportTo(swivelD2->groundPlane);
		swivelD2->transitionDuration = dur;
		
		XWalkParamNode * swivelR2 = addNode(new XWalkParamNode("SwivelR2",walk->getID()));
		(fmat::rotationXN<4>(-mag) * fmat::pack(0,0,1,-50)).exportTo(swivelR2->groundPlane);
		swivelR2->transitionDuration = dur;
		
		XWalkParamNode * swivelU2 = addNode(new XWalkParamNode("SwivelU2",walk->getID()));
		(fmat::rotationYN<4>(mag) * fmat::pack(0,0,1,-50)).exportTo(swivelU2->groundPlane);
		swivelU2->transitionDuration = dur;
		
		XWalkParamNode * swivelL2 = addNode(new XWalkParamNode("SwivelL2",walk->getID()));
		(fmat::rotationXN<4>(mag) * fmat::pack(0,0,1,-50)).exportTo(swivelL2->groundPlane);
		swivelL2->transitionDuration = dur;
		
		XWalkParamNode * swivelD3 = addNode(new XWalkParamNode("SwivelD3",walk->getID()));
		(fmat::rotationYN<4>(-mag) * fmat::pack(0,0,1,-50)).exportTo(swivelD3->groundPlane);
		swivelD3->transitionDuration = dur;
		
		swivel=swivelD;
		swivelD->addTransition(new TimeOutTrans(swivelL,dur/3));
		swivelL->addTransition(new TimeOutTrans(swivelU,dur/3));
		swivelU->addTransition(new TimeOutTrans(swivelR,dur/3));
		swivelR->addTransition(new TimeOutTrans(swivelD2,dur/3));
		swivelD2->addTransition(new TimeOutTrans(swivelR2,dur/3));
		swivelR2->addTransition(new TimeOutTrans(swivelU2,dur/3));
		swivelU2->addTransition(new TimeOutTrans(swivelL2,dur/3));
		swivelL2->addTransition(new TimeOutTrans(swivelD3,dur/3));
		swivelD3->addTransition(new TimeOutTrans(reset,dur/3));
	}
	
	StateNode * wave;
	{
		const float mag = 9*(float)M_PI/180;
		const float off = 50;
		const unsigned int dur=4000;
		
		XWalkParamNode * wave1 = addNode(new XWalkParamNode("Wave1",walk->getID()));
		(fmat::rotationYN<4>(-mag) * fmat::pack(0,0,1,-50)).exportTo(wave1->groundPlane);
		wave1->offsetX = -off;
		wave1->transitionDuration = dur;
		
		XWalkParamNode * wave2 = addNode(new XWalkParamNode("Wave2",walk->getID()));
		(fmat::rotationYN<4>(0.f) * fmat::pack(0,0,1,-50)).exportTo(wave2->groundPlane);
		wave2->transitionDuration = dur;
		
		XWalkParamNode * wave3 = addNode(new XWalkParamNode("Wave3",walk->getID()));
		(fmat::rotationYN<4>(mag) * fmat::pack(0,0,1,-50)).exportTo(wave3->groundPlane);
		wave1->offsetX = off;
		wave3->transitionDuration = dur;
		
		XWalkParamNode * wave4 = addNode(new XWalkParamNode("Wave4",walk->getID()));
		(fmat::rotationYN<4>(-mag) * fmat::pack(0,0,1,-50)).exportTo(wave4->groundPlane);
		wave1->offsetX = -off;
		wave4->transitionDuration = dur*2;
		
		XWalkParamNode * wave5 = addNode(new XWalkParamNode("Wave5",walk->getID()));
		(fmat::rotationYN<4>(0.f) * fmat::pack(0,0,1,-50)).exportTo(wave5->groundPlane);
		wave5->transitionDuration = dur;
		
		XWalkParamNode * wave6 = addNode(new XWalkParamNode("Wave6",walk->getID()));
		(fmat::rotationYN<4>(mag) * fmat::pack(0,0,1,-50)).exportTo(wave6->groundPlane);
		wave1->offsetX = off;
		wave6->transitionDuration = dur;
		
		XWalkParamNode * wave7 = addNode(new XWalkParamNode("Wave7",walk->getID()));
		(fmat::rotationYN<4>(-mag) * fmat::pack(0,0,1,-50)).exportTo(wave7->groundPlane);
		wave1->offsetX = -off;
		wave7->transitionDuration = dur*2;
		
		XWalkParamNode * wave8 = addNode(new XWalkParamNode("Wave8",walk->getID()));
		(fmat::rotationYN<4>(0.f) * fmat::pack(0,0,1,-50)).exportTo(wave8->groundPlane);
		wave8->transitionDuration = dur;
		
		wave = wave1;
		wave1->addTransition(new TimeOutTrans(wave2,dur/3));
		wave2->addTransition(new TimeOutTrans(wave3,dur/3));
		wave3->addTransition(new TimeOutTrans(wave4,dur/3));
		wave4->addTransition(new TimeOutTrans(wave5,dur/3));
		wave5->addTransition(new TimeOutTrans(wave6,dur/3));
		wave6->addTransition(new TimeOutTrans(wave7,dur/3));
		wave7->addTransition(new TimeOutTrans(wave8,dur/3));
		wave8->addTransition(new TimeOutTrans(reset,dur/3));
	}
	
	StateNode * hi;
	{
		XWalkParamNode * back = addNode(new XWalkParamNode("LeanBack",walk->getID()));
		back->offsetX=-40;
		
		SoundNode * yipper = addNode(new SoundNode("Yipper","yipper.wav"));
		
		SmallMotionSequenceNode * sayhi = addNode(new SmallMotionSequenceNode("SayHi","wavearm.mot"));
		
		hi = back;
		back->addTransition(new CompletionTrans(yipper));
		yipper->addTransition(new NullTrans(sayhi));
		sayhi->addTransition(new CompletionTrans(reset));
	}
	
	RandomTrans * decider = new RandomTrans(swivel);
	decider->addDestination(bounce);
	decider->addDestination(wave);
	decider->addDestination(hi);
	StateNode * choose = addNode(new StateNode("Choose"));
	choose->addTransition(decider);
	
	startnode = addNode(new XWalkParamNode("Stand",walk->getID()));
	
	MCNode<SmallMotionSequenceMC> * foldArm = addNode(new MCNode<SmallMotionSequenceMC>("FoldArm"));
	foldArm->getMC()->setTime(750);
	foldArm->getMC()->setOutputCmd(WristOffset,-(float)M_PI/2);
	foldArm->getMC()->setOutputCmd(ArmElbowOffset,0);
	foldArm->getMC()->advanceTime(750);
	foldArm->getMC()->setOutputCmd(ArmElbowOffset,-(float)M_PI/2);
	foldArm->getMC()->setOutputCmd(ArmShoulderOffset,0);
	foldArm->getMC()->setOutputCmd(WristOffset,-(float)M_PI/2);
	foldArm->getMC()->advanceTime(750);
	foldArm->getMC()->setOutputCmd(ArmShoulderOffset,-(float)M_PI/2);
	foldArm->getMC()->setOutputCmd(WristOffset,0);
	
	Transition * startup = new TimeOutTrans(foldArm,5000);
	startup->addDestination(addNode(new RandomCamNode));
	startnode->addTransition(startup);
	foldArm->addTransition(reset->addTransition(new TimeOutTrans(choose,5000)));
}


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
