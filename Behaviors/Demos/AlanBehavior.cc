#include "Shared/RobotInfo.h"
#ifdef TGT_IS_AIBO

#include "Behaviors/BehaviorBase.h"
#include "Motion/MotionManager.h"
#include "Motion/MotionPtr.h"
#include "Shared/WorldState.h"
#include "Events/EventRouter.h"
#include "Motion/PostureMC.h"
#include "Shared/ERS210Info.h"
#include "Shared/ERS7Info.h"

//! just a little demo behavior which lifts a leg higher as more pressure is put on a head button
/*! Based on an idea from Alan Chun-ho Ho for a basic demo program */
class AlanBehavior : public BehaviorBase {
public:
	//! constructor, initialize members
	AlanBehavior() : BehaviorBase(), poser() {}
	
	virtual void doStart() {
		// register our motion command so we can move the joints, BehaviorBase will auto-remove on stop()
		addMotion(poser);
		// subscribe to sensor updated events through the global EventRouter
		erouter->addListener(this,EventBase::sensorEGID,SensorSrcID::UpdatedSID);
	}
	
	virtual void doEvent() {
		// to be more general, let's check that it's the right event:
		if(event->getGeneratorID()==EventBase::sensorEGID) {
			//we'll need to specify the ERS210Info namespace below when
			//referencing the button offsets so that this will compile for
			//the ERS-7 as well (which lacks front and back head buttons),
			//but for your own code, you could leave it off if you weren't
			//worried about compiling for other models.

			//Joint offsets are defined in ERS210Info.h, ERS220Info.h, etc.
			unsigned int joint=LFrLegOffset+RotatorOffset;
			
			//state is a global instantiation of WorldState, kept up to date by framework;
			//pressure is in range 0 to 1 - we use the pressure on the front head button here
			float pressure=0;
			if(RobotName == ERS210Info::TargetName) { // ERS-210 or ERS-2xx
				pressure=state->buttons[capabilities.getButtonOffset("HeadFrBut")];
				std::cout << "HeadFrBut Pressure: " << pressure << std::endl;
			} else if(RobotName == ERS7Info::TargetName) {
				pressure=state->buttons[ERS7Info::HeadButOffset];
				std::cout << "HeadBut Pressure: " << pressure << std::endl;
			} else {
				//only really works on the ERS-210 or ERS-7 models - the others don't have a proper pressure sensor
				//(the 220's antenna-thing is close, but doesn't give a continuous range)
				std::cout << "Unsupported/unknown model" << std::endl;
				erouter->removeListener(this); // stops getting events (and timers, if we had any)
				return;
			}
			
			//outputRanges is a constant table, also defined in ERS210Info.h or ERS220Info.h
			float angle=outputRanges[joint][MaxRange]*pressure;

			// now send the joint angle to the posture motion command
			poser->setOutputCmd(joint,angle);

			//let's do the whole thing again with the other head button for the other leg:
			// (cutting out a some of the intermediary steps this time)
			joint=RFrLegOffset+RotatorOffset;
			if(RobotName == ERS210Info::TargetName)
				poser->setOutputCmd(joint,outputRanges[joint][MaxRange]*state->buttons[capabilities.getButtonOffset("HeadBkBut")]);
			else if(RobotName == ERS7Info::TargetName) //ERS7 doesn't have another head button, we'll use one of its back buttons
				poser->setOutputCmd(joint,outputRanges[joint][MaxRange]*state->buttons[ERS7Info::FrontBackButOffset]);

		} else {
			//should never happen
			std::cout << "Unhandled Event:" << event->getName() << std::endl;
		}
	}
	
	// these allow the description to be shown as a tooltip in the ControllerGUI (not required, but nice)
	static std::string getClassDescription() {
		return "Lifts the left/right front legs higher as more pressure is applied to the front/back head buttons";
	}
	virtual std::string getDescription() const { return getClassDescription(); }
	
protected:
	MotionPtr<PostureMC> poser; //!< a PostureMC to control the leg
};

REGISTER_BEHAVIOR_MENU(AlanBehavior,DEFAULT_TK_MENU);

#endif

/*! @file
 * @brief Defines AlanBehavior, a little demo behavior which lifts a leg higher as more pressure is put on the back head button
 * @author ejt (Creator)
 */
