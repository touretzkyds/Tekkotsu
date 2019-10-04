#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"
#include "Shared/Gamepad.h"
#include "Motion/MotionPtr.h"
#include "Motion/WalkMC.h"
#include "Motion/ArmMC.h"
#include "Motion/HeadPointerMC.h"

using namespace GamepadSrcID;

class GamepadDemo : public BehaviorBase {
private:
	float xvel;
	float avel;
	float armxvel;
	float armyvel;
	float headxvel;
	float headyvel;
	float grippervel;

public:
	GamepadDemo() : BehaviorBase("GamepadDemo"),
									xvel(0), avel(0), armxvel(0), armyvel(0), headxvel(0), headyvel(0), grippervel(0),
									walk(), arm(), head()
	{}

	static constexpr float ForwardSpeed = 125.0;
	static constexpr float TurnSpeed = M_PI / 8;
	static constexpr float HeadJointIncrement = 0.025;
	static constexpr float ArmJointIncrement =  0.010;


	virtual void doStart() {
#if defined(TGT_HAS_WHEELS) || defined(TGT_HAS_LEGS)
		addMotion(walk);
#endif
#ifdef TGT_HAS_ARMS
		addMotion(arm);
#endif
#ifdef TGT_HAS_HEAD
		addMotion(head);
#endif
		erouter->addListener(this, EventBase::buttonEGID);
		erouter->addTimer(this, 1, 50, true);
		std::cout << "Gamepad demo started.\n";
		std::cout << "  Left stick to drive the robot.\n";
		std::cout << "  Right stick to move the head head.\n";
		std::cout << "  D-Pad to move the arm.\n";
		std::cout << "  Bumpers to open/close the gripper.\n";
	}

	virtual void doEvent() {
		switch ( event->getGeneratorID() ) {
		case EventBase::buttonEGID:
			if ( IS_GAMEPAD_SID(event->getSourceID()) ) {
				switch (event->getSourceID()) {
				case gamepadLeftJoyYSrcID:
#if defined(TGT_HAS_WHEELS) || defined(TGT_HAS_LEGS)
					xvel = event->getMagnitude() * ForwardSpeed;
#endif
					break;
				case gamepadLeftJoyXSrcID:
#if defined(TGT_HAS_WHEELS) || defined(TGT_HAS_LEGS)
					avel = event->getMagnitude() * TurnSpeed;
#endif
					break;
				case gamepadRightJoyYSrcID:
#ifdef TGT_HAS_HEAD
					headyvel = event->getMagnitude() * HeadJointIncrement;
#endif
					break;
				case gamepadRightJoyXSrcID:
#ifdef TGT_HAS_HEAD
					headxvel = event->getMagnitude() * HeadJointIncrement;
#endif
					break;
				case gamepadDPadYSrcID:
#ifdef TGT_HAS_ARMS
					armyvel = event->getMagnitude() * ArmJointIncrement;
#endif
					break;
				case gamepadDPadXSrcID:
#ifdef TGT_HAS_ARMS
					armxvel = event->getMagnitude() * ArmJointIncrement;
#endif
					break;
				case gamepadLeftBumperSrcID:
#ifdef TGT_HAS_ARMS
					grippervel = event->getMagnitude() * ArmJointIncrement;
#endif
					break;
				case gamepadRightBumperSrcID:
#ifdef TGT_HAS_ARMS
					grippervel = -1*event->getMagnitude() * ArmJointIncrement;
#endif
					break;
				}

				switch (event->getSourceID()) {
				case gamepadLeftJoyXSrcID:
				case gamepadLeftJoyYSrcID:
#if defined(TGT_HAS_WHEELS) || defined(TGT_HAS_LEGS)
					std::cout << "Walk velocity: fwd = " << std::setw(6) << -1*xvel << "  turn=" << std::setw(4) << -1*avel << std::endl;
#endif					
          break;
				case gamepadRightJoyXSrcID:
				case gamepadRightJoyYSrcID:
#ifdef TGT_HAS_HEAD
					std::cout << "Head pan = " << std::setw(5) << head->getJointValue(0)
										<< "  tilt=" << std::setw(5) << head->getJointValue(1) << std::endl;
#endif
					break;
				case gamepadDPadXSrcID:
				case gamepadDPadYSrcID:
#ifdef TGT_HAS_ARMS
					std::cout << "Arm shoulder = " << std::setw(5) << arm->getJointValue(0)
										<< "  elbow = " << std::setw(5) << arm->getJointValue(1) << std::endl;
#endif				
        break;
				case gamepadLeftBumperSrcID:
				case gamepadRightBumperSrcID:
#ifdef TGT_HAS_ARMS
					std::cout << "Gripper = " << std::setw(5) << arm->getJointValue(2) << std::endl;
#endif
					break;
				}
			}
			break;
		case EventBase::timerEGID:
#if defined(TGT_HAS_WHEELS) || defined(TGT_HAS_LEGS)
			walk->setTargetVelocity(-1*xvel, 0, -1*avel);
#endif
#ifdef TGT_HAS_HEAD
			head->setJoints(head->getJointValue(0) - headxvel,
											head->getJointValue(1) + headyvel);
#endif
#ifdef TGT_HAS_ARMS
			arm->setJoints(arm->getJointValue(0) + armxvel,
										 arm->getJointValue(1) - armyvel,
										 arm->getJointValue(2) + grippervel);
#endif
			break;
		default:
			break;
		}
	}

	virtual void doStop() {
		std::cout << "Gamepad demo ended." << std::endl;
	}

protected:
#if defined(TGT_HAS_WHEELS) || defined(TGT_HAS_LEGS)
	MotionPtr<WalkMC> walk;
#else
  int walk;
#endif
#ifdef TGT_HAS_ARMS
	MotionPtr<ArmMC> arm;
#else
  int arm;
#endif
#ifdef TGT_HAS_HEAD
	MotionPtr<HeadPointerMC> head;
#else
  int head;
#endif
};

REGISTER_BEHAVIOR_MENU(GamepadDemo, DEFAULT_TK_MENU"/Interaction Demos");


/*! @file
 * @brief A test to move the robot via gamepad
 * @author med, asf
 */
