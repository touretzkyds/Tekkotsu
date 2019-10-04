#include "Shared/RobotInfo.h"
#ifdef TGT_IS_AIBO

#include "Behaviors/StateNode.h"
#include "Behaviors/Demos/StareAtBallBehavior.h"
#include "IPC/SharedObject.h"
#include "Motion/PostureMC.h"
#include "Motion/MMAccessor.h"
#include "Motion/MotionSequenceMC.h"
#include "Motion/LedMC.h"
#include "Behaviors/Transitions/TimeOutTrans.h"
#include "Behaviors/Transitions/SmoothCompareTrans.h"
#include "Behaviors/Nodes/OutputNode.h"
#include "Sound/SoundManager.h"
#include "Shared/ProjectInterface.h"
#include "Shared/WorldState.h"

#include "Behaviors/Demos/karmedbandit.h"

//! Plays K-armed bandit
class BanditMachine : public StateNode {
public:
	//!constructor
	BanditMachine()
	: StateNode(), stare(NULL), start(NULL), liedown(MotionManager::invalid_MC_ID), bandit(2)
	{
		stare=new StareAtBallBehavior();
		stare->addReference();
	}
	//!constructor
	BanditMachine(const char* n)
	: StateNode(n), stare(), start(NULL), liedown(MotionManager::invalid_MC_ID), bandit(2)
	{
		stare=new StareAtBallBehavior();
		stare->addReference();
	}
	//!destructor
	virtual ~BanditMachine() {
		stare->removeReference();
	}
	
	static std::string getClassDescription() { return "Plays k-armed bandit with a computer"; }
	virtual std::string getDescription() const { return getClassDescription(); }
	
	virtual void setup() {
		StateNode *wait=start=addNode(new WaitNode("Wait",bandit));
		StateNode *left=addNode(new PressNode("Left",LFrLegOffset+KneeOffset));
		StateNode *right=addNode(new PressNode("Right",RFrLegOffset+KneeOffset));
		StateNode *decide=addNode(new DecideNode("Decide",bandit,left,right));
		StateNode *recoverl=addNode(new OutputNode("\nBadPressLeft",std::cout,wait));
		StateNode *recoverr=addNode(new OutputNode("\nBadPressRight",std::cout,wait));
		left->addTransition(new SmoothCompareTrans<float>(wait,&state->pidduties[LFrLegOffset+RotatorOffset],CompareTrans<float>::LT,-.07f,EventBase(EventBase::sensorEGID,SensorSrcID::UpdatedSID,EventBase::statusETID),.7f));
		right->addTransition(new SmoothCompareTrans<float>(wait,&state->pidduties[RFrLegOffset+RotatorOffset],CompareTrans<float>::LT,-.07f,EventBase(EventBase::sensorEGID,SensorSrcID::UpdatedSID,EventBase::statusETID),.7f));
		wait->addTransition(new TimeOutTrans(decide,2000));
		left->addTransition(new TimeOutTrans(recoverl,1500));
		right->addTransition(new TimeOutTrans(recoverr,1500));
		//		recover->addTransition(new TimeOutTrans(decide,500));
		StateNode::setup();
	}
	
	virtual void doStart() {
		StateNode::doStart();
		stare->start();
		start->start();
		SharedObject<PostureMC> lie("liedown.pos");
		lie->setOutputCmd(LFrLegOffset+RotatorOffset,.77f);
		lie->setOutputCmd(RFrLegOffset+RotatorOffset,.73f);
		lie->setOutputCmd(LFrLegOffset+KneeOffset,.6f);
		lie->setOutputCmd(RFrLegOffset+KneeOffset,.6f);
		liedown=motman->addPrunableMotion(lie);
	}
	
	virtual void doStop() {
		motman->removeMotion(liedown);
		stare->stop();
		StateNode::doStop();
	}
	
protected:
	//! This node is used to move a paw down using a MotionSequenceMC
	class PressNode : public StateNode {
	public:
		//! constructor
		/*! @param n name of the node
		 *  @param idx the joint index of the paw to move
		 */
		PressNode(const char* n, unsigned int idx) : StateNode(n), press_id(MotionManager::invalid_MC_ID), index(idx) {
			SharedObject<SmallMotionSequenceMC> press;
			press->setTime(0);
			press->setOutputCmd(idx,.6f);
			press->setTime(1);
			press->setOutputCmd(idx,.6f);
			press->setTime(200);
			press->setOutputCmd(idx,.3f);
			press->setTime(1500);
			press->setOutputCmd(idx,outputRanges[idx][MinRange]);
			press_id=motman->addPersistentMotion(press,MotionManager::kStdPriority+1);
		}
		//!destructor
		virtual ~PressNode() {
			motman->removeMotion(press_id);
		}
		virtual void doStart() {
			StateNode::doStart();
			MMAccessor<SmallMotionSequenceMC> press(press_id);
			press->play();
			press->setOutputCmd(index,.6f);
			//			press->setSpeed(1);
		}
		virtual void doStop() {
			MMAccessor<SmallMotionSequenceMC> press(press_id);
			//			press->setSpeed(-1);
			press->pause();
			press->setTime(0);
			StateNode::doStop();
		}
	protected:
		MotionManager::MC_ID press_id; //!< the MC_ID of the MotionSequenceMC being used to do the press
		unsigned int index; //!< the joint index of the paw to move
	};
	
	//! uses one of the algorithms in karmedbandit.h to decide which paw to press next
	class DecideNode : public StateNode {
	public:
		//! constructor
		/*! @param n name of the node
		 *  @param bandito the decision making algorithm to use (look in karmedbandit.h)
		 *  @param left the PressNode to go to if the left paw is chosen
		 *  @param right the PressNode to go to if the right paw is chosen
		 */
		DecideNode(const char* n, karmedbanditExp3_1& bandito, StateNode* left, StateNode* right)
		: StateNode(n), b(bandito), l(left), r(right)
		{}
		virtual void doStart() {
			StateNode::doStart();
			addReference();
			doStop();
			if(b.decide()==0) {
				std::cout << "Left... " << std::flush;
				l->start();
			} else {
				std::cout << "Right... " << std::flush;
				r->start();
			}
			removeReference();
		}
	protected:
		karmedbanditExp3_1& b; //!< the class implementing the k-armed bandit algorithm
		StateNode* l; //!< the node to go to if the left paw is chosen
		StateNode* r; //!< the node to go to if the right paw is chosen
	private:
		DecideNode(const DecideNode& node); //!< don't call this
		DecideNode operator=(const DecideNode& node); //!< don't call this
	};
	
	//! Waits to see if a reward is received, lights up LEDs to let the user know
	class WaitNode : public StateNode {
	public:
		//! constructor
		/* @param n name to use for the node
		 * @param bandito the class to pass the reward to (if it comes)
		 */
		WaitNode(const char* n, karmedbanditExp3_1& bandito)
		: StateNode(n), b(bandito), reward(false), leds_id(MotionManager::invalid_MC_ID)
		{
			leds_id=motman->addPersistentMotion(SharedObject<LedMC>());
		}
		//! destructor
		virtual ~WaitNode() {
			motman->removeMotion(leds_id);
		}
		virtual void doStart() {
			StateNode::doStart();
			erouter->addListener(this,EventBase::visObjEGID,ProjectInterface::visPinkBallSID);
			erouter->addTimer(this,0,1000,false);
			MMAccessor<LedMC> leds(leds_id);
#ifdef TGT_IS_AIBO
			leds->cflash(BotLLEDMask+BotRLEDMask,1,1000);
#endif
		}
		virtual void doStop() {
			erouter->removeListener(this);
			b.reward(reward);
			std::cout << std::endl;
			reward=false;
			StateNode::doStop();
		}
		virtual void doEvent() {
			if(event->getGeneratorID()==EventBase::timerEGID) {
				sndman->playFile("whimper.wav");
			} else {
				sndman->playFile("yipper.wav");
				reward=true;
				MMAccessor<LedMC> leds(leds_id);
#ifdef TGT_IS_AIBO
				leds->cflash(MidLLEDMask+MidRLEDMask,1,100);
#endif
			}
			erouter->removeListener(this);
		}
	protected:
		karmedbanditExp3_1& b; //!< the class implimenting a k-armed bandit algorithm to pass the reward back to
		bool reward; //!< true if a reward was received
		MotionManager::MC_ID leds_id; //!< MC_ID of a LedMC
	};
	
	StareAtBallBehavior* stare; //!< active as long as we're in this state so it keeps an eye on the ball
	StateNode* start; //!< used to start off by lying down before we start pressing buttons
	MotionManager::MC_ID liedown; //!< a MotionSequence which will move the dog into a lying down posture
	karmedbanditExp3_1 bandit; //!< algorithm to use in the k-armed bandit problem
	
private:
	BanditMachine(const BanditMachine& node); //!< don't call this
	BanditMachine operator=(const BanditMachine& node); //!< don't call this
};

REGISTER_BEHAVIOR_MENU(BanditMachine,DEFAULT_TK_MENU"/State Machine Demos");

#endif

/*! @file
 * @brief Defines BanditMachine, A state machine for playing k-armed bandit
 * @author ejt (Creator)
 */
