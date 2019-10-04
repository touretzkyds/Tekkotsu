#include "Shared/RobotInfo.h"
#ifdef TGT_IS_BIOLOID

#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"
#include "Events/LocomotionEvent.h"
#include "Motion/MMAccessor.h"
#include "Motion/MotionManager.h"
#include "Motion/MotionSequenceMC.h"
#include "Motion/PIDMC.h"
#include "Motion/PostureMC.h"
#include "Shared/WorldState.h"
#include "local/DeviceDrivers/DynamixelProtocol.h"
#include "IPC/SharedObject.h"

//! Listen for Dynamixel servo errors and power down the servo for a brief period to reset it.  Also twitch the legs if they're under load and the robot is just standing around.

class ResetServos : public BehaviorBase {
public:
	ResetServos() : BehaviorBase("ResetServos"), pidmc_id(MotionManager::invalid_MC_ID) {}

	virtual void doStart() {
		pidmc_id = motman->addPersistentMotion(SharedObject<PIDMC>(), MotionManager::kHighPriority);
		erouter->addListener(this, EventBase::servoEGID);
#ifdef TGT_HAS_LEGS
		erouter->addListener(this, EventBase::locomotionEGID);
#endif
	}

	virtual void doStop() {
		motman->removeMotion(pidmc_id);
	}

	static const unsigned int  loadErrorDownTime = 2500; //!< How long to keep the servo turned off (milliseconds) after a load error
	static const unsigned int  heatErrorDownTime = 5000; //!< How long to keep the servo turned off (milliseconds) after a heat error
	static const unsigned int  fidgetInterval = 10000; //!< How long between leg fidgets

	virtual void doEvent() {
		switch ( event->getGeneratorID() ) {

		case EventBase::servoEGID: {
			const unsigned int joint = event->getSourceID();
			if ( MMAccessor<PIDMC>(pidmc_id)->getPID(joint).weight > 0 )  // We're already cooling this servo, so punt
				return;
			unsigned char errorCode = (unsigned char)event->getMagnitude();
			if ( (errorCode & DynamixelProtocol::LOAD_ERROR) && std::abs(state->pidduties[joint]) > 0.05 )
				erouter->addTimer(this, joint, loadErrorDownTime, false);
			else if ( errorCode & DynamixelProtocol::HEAT_ERROR )
				erouter->addTimer(this, joint+1000, heatErrorDownTime, false);
			else return;
			std::cout << "Joint " << joint << " (" << outputNames[joint] << ") reported ";
			for(unsigned int i=0; i<sizeof(errorCode)*8; ++i)
				if( (errorCode>>i) & 1 )
				std::cout << DynamixelProtocol::ResponseErrorNames[i];
			std::cout << ",  pidduty=" << state->pidduties[joint] << std::endl;
			MMAccessor<PIDMC>(pidmc_id)->setJointPowerLevel(joint, 0, 1);    // Turn servo off: set power level to 0 with weight of 1
			break;
		}

#ifdef TGT_HAS_LEGS
		case EventBase::locomotionEGID: {
			const LocomotionEvent& le = dynamic_cast<const LocomotionEvent&>(*event);
			if ( le.x == 0 && le.y == 0 && le.a == 0 )  // walking has stopped
				erouter->addTimer(this, 9999, fidgetInterval, true);
			else
				erouter->removeTimer(this, 9999);
		// std::cout << "ResetServos locomotion: "<< le.x <<" "<< le.y <<" "<<le.a<<std::endl;
			break;
		}
#endif

		case EventBase::timerEGID: {
			unsigned int source = event->getSourceID();
#ifdef TGT_HAS_LEGS
			if ( source == 9999 ) {
				twitchSomeLeg();
				return;
			}
#endif
			bool loadError = source < 1000;
			unsigned int joint = loadError ? source : source-1000;
			MMAccessor<PIDMC>(pidmc_id)->setJointPowerLevel(joint, 0, 0);				// Reset weight to 0 so servo can power up again
// std::cout << "Cured error in joint " << joint << std::endl;
			if ( loadError) {
			// For load errors, after cooldown, twitch the joint by briefly pulling it toward its zero degree position
				float curpos = state->outputs[joint];
				SharedObject<PostureMC> twitch_mc;
				twitch_mc->setOutputCmd(joint, curpos*state->pidduties[joint]);
				motman->addPrunableMotion(twitch_mc, MotionManager::kHighPriority);
			}
			break;
		}

		default:
			std::cout << "Unexpected event " << event->getDescription() << std::endl;
			break;
		}
	}

#ifdef TGT_HAS_LEGS
	void twitchSomeLeg() {
		unsigned int joint = 0;
		for(unsigned int i = LegOffset; i < LegOffset+NumLegJoints; i++)
			if ( fabs(state->pidduties[i]) > fabs(state->pidduties[joint]) )
			joint=i;
		if( fabs(state->pidduties[joint]) < 0.2)
			return;
		unsigned int leg = (joint-LegOffset)/JointsPerLeg;	
		unsigned int elevator = LegOffset + leg*JointsPerLeg + ElevatorOffset;
		//const float midrange = (outputRanges[elevator][MaxRange]+outputRanges[elevator][MinRange])/2;
		// std::cout << "I found that the most presure being applied is at " << outputNames[joint] << "  " << state->pidduties[joint] << std::endl;
		float curpos = state->outputs[elevator];
		float newpos = 1.3f; // (curpos > midrange) ? curpos+0.4 : curpos-0.4;					
		// std::cout<<"curpos="<<curpos<<"   newpos="<<newpos<<"  tmidrange="<<midrange<<std::endl;
		SharedObject<TinyMotionSequenceMC> fidget_mc;
		fidget_mc->advanceTime(700);
		fidget_mc->setOutputCmd(joint,newpos);
		fidget_mc->advanceTime(300);
		fidget_mc->setOutputCmd(joint,curpos);
		motman->addPrunableMotion(fidget_mc,MotionManager::kHighPriority);
	}
#endif

private:
	MotionManager::MC_ID pidmc_id;

};

REGISTER_BEHAVIOR_MENU_OPT(ResetServos,"Background Behaviors/System Daemons",BEH_NONEXCLUSIVE|BEH_START);

#endif
