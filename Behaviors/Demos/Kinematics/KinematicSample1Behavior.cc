#include "Shared/RobotInfo.h"
#ifdef TGT_IS_AIBO

#include "Behaviors/BehaviorBase.h"
#include "Shared/MarkScope.h"
#include "Motion/Kinematics.h"
#include "Motion/PIDMC.h"
#include "Motion/PostureMC.h"
#include "Motion/MotionManager.h"
#include "Motion/MotionPtr.h"

//! Uses kinematics to mirror leg positions (note that it mirrors the paw position - not necessarily the joint angles used to get there!)
class KinematicSample1Behavior : public BehaviorBase {
public:
	//! constructor
	KinematicSample1Behavior()
		: BehaviorBase("KinematicSample1Behavior"), lastLeg(LFrLegOrder), pose()
	{ }

	virtual void doStart() {
		addMotion(pose); // BehaviorBase::stop() will clean up
		erouter->addListener(this,EventBase::sensorEGID);
		erouter->addListener(this,EventBase::buttonEGID);
	}

	virtual void doEvent() {
		if(event->getGeneratorID()==EventBase::buttonEGID) {
			switch(event->getSourceID()) {
			case LFrPawOffset:
				lastLeg=LFrLegOrder; break;
			case RFrPawOffset:
				lastLeg=RFrLegOrder; break;
			case LBkPawOffset:
				lastLeg=LBkLegOrder; break;
			case RBkPawOffset:
				lastLeg=RBkLegOrder; break;
			default:
				return;
			}
			if(event->getTypeID()==EventBase::activateETID) {
				unsigned int lastlegoff=LegOffset+lastLeg*JointsPerLeg;
				SharedObject<PIDMC> relaxLeg(lastlegoff,lastlegoff+JointsPerLeg,0);
				motman->addPrunableMotion(relaxLeg);
				MarkScope lock(pose); // holds lock on pose for duration instead of each call
				for(unsigned int i=0; i<JointsPerLeg; i++)
					pose->setOutputCmd(lastlegoff+i,OutputCmd::unused);
			} else if(event->getTypeID()==EventBase::deactivateETID) {
				unsigned int lastlegoff=LegOffset+lastLeg*JointsPerLeg;
				SharedObject<PIDMC> tightLeg(lastlegoff,lastlegoff+JointsPerLeg,1);
				motman->addPrunableMotion(tightLeg);
			}

		} else if(event->getGeneratorID()==EventBase::sensorEGID) {
			//I'll use the pack and unpack functions here just to illustrate how to avoid 
			//needing to use NEWMAT data structures - but if you're going to be doing
			//any significant math, you'll eventually want to get comfortable with NEWMAT...

			// (Actually, I think the NEWMAT version would be more readable...)

			/******** Determine location of target position ********/
			float link_x=60,link_y=0,link_z=0; // 6cm along x axis of selected joint
			float obj_x=0, obj_y=0, obj_z=0;   // these will hold the objective position
			//this next line computes the link position, and stores result into obj_*
			Kinematics::unpack(kine->linkToBase(getIndex(lastLeg))*Kinematics::pack(link_x,link_y,link_z,1.f),
			                   obj_x,obj_y,obj_z);

			/******** Solve each leg for the point ********/
			MarkScope lock(pose); // holds lock on pose for duration instead of each call
			for(unsigned int i=0; i<NumLegs; i++)
				if(i!=(unsigned int)lastLeg) {
					float m_x=((i<2)==((unsigned int)lastLeg<2))?obj_x:-obj_x;
					float m_y=((i%2)==((unsigned int)lastLeg%2))?obj_y:-obj_y;
					pose->solveLinkPosition(Kinematics::pack(m_x,m_y,obj_z),
							getIndex((LegOrder_t)i),
							Kinematics::pack(link_x,link_y,link_z));
				}
			
			//If you would like to verify the positiions of the back toes... (relative to body center)
			//cout << "L: " << kine->getInterestPoint(BaseFrameOffset,"ToeLBkPaw").t();
			//cout << "R: " << kine->getInterestPoint(BaseFrameOffset,"ToeRBkPaw").t();
			
		} else {
			serr->printf("KinematicSample1Behavior: Unhandled event %s\n",event->getName().c_str());
		}
	}

	static std::string getClassDescription() { return "Uses kinematics to mirror leg positions (note that it mirrors the paw position - not necessarily the joint angles used to get there!)"; }
	virtual std::string getDescription() const { return getClassDescription(); }
	
protected:
	//! returns the index of the knee for the requested @a leg
	unsigned int getIndex(LegOrder_t leg) {
		//or try: return FootFrameOffset+leg;
		return LegOffset+leg*JointsPerLeg+KneeOffset;
	}
	LegOrder_t lastLeg; //!< the last leg to have its button pressed, i.e. the "source"
	MotionPtr<PostureMC> pose; //!< the PostureMC which does all the computation
};

REGISTER_BEHAVIOR_MENU(KinematicSample1Behavior,DEFAULT_TK_MENU"/Kinematics Demos");

#endif

/*! @file
 * @brief Defines KinematicSample1Behavior, which uses kinematics to mirror leg positions (note that it mirrors the paw position - not necessarily the joint angles used to get there!)
 * @author ejt (Creator)
 */
