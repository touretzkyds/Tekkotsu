#include "Shared/RobotInfo.h"
#ifdef TGT_IS_AIBO

#include "Behaviors/BehaviorBase.h"
#include "Motion/Kinematics.h"
#include "Motion/MotionManager.h"
#include "Motion/PIDMC.h"
#include "Motion/PostureMC.h"
#include "Motion/MotionPtr.h"
#include "Shared/MarkScope.h"

//! Uses kinematics to make the back toe (Toe{LR}BkPaw) touch the lower thigh (Lower{LeftBackL,RightBackR}FrThigh)
class KinematicSample2Behavior : public BehaviorBase {
public:
	//! constructor
	KinematicSample2Behavior()
		: BehaviorBase("KinematicSample2Behavior"), lastLeg(LFrLegOrder), pose()
	{ }

	virtual void doStart() {
		addMotion(pose); // BehaviorBase::stop() will clean up
		erouter->addListener(this,EventBase::sensorEGID);
		erouter->addListener(this,EventBase::buttonEGID);
	}

	virtual void doEvent() {
		using std::cout;
		using std::endl;
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
				MarkScope lock(pose); // holds lock for duration instead of each individual call
				for(unsigned int i=0; i<JointsPerLeg; i++)
					pose->setOutputCmd(lastlegoff+i,OutputCmd::unused);
			} else if(event->getTypeID()==EventBase::deactivateETID) {
				unsigned int lastlegoff=LegOffset+lastLeg*JointsPerLeg;
				SharedObject<PIDMC> tightLeg(lastlegoff,lastlegoff+JointsPerLeg,1);
				motman->addPrunableMotion(tightLeg);
			}

		} else if(event->getGeneratorID()==EventBase::sensorEGID) {
			//Plan A:
			fmat::Column<3> obj;
			switch(lastLeg) {
			case LFrLegOrder:
				obj=kine->getInterestPoint(BaseFrameOffset,"LowerInnerBackLFrThigh,LowerOuterBackLFrThigh"); break;
			case RFrLegOrder:
				obj=kine->getInterestPoint(BaseFrameOffset,"LowerInnerBackRFrThigh,LowerOuterBackRFrThigh"); break;
			case LBkLegOrder:
				obj=kine->getInterestPoint(BaseFrameOffset,"LowerInnerFrontLBkThigh"); break;
			case RBkLegOrder:
				obj=kine->getInterestPoint(BaseFrameOffset,"LowerInnerFrontRBkThigh"); break;
			}

			unsigned int solveLink=FootFrameOffset+((lastLeg+2)%NumLegs); //swap front/back
			fmat::Column<3> link;
			switch(lastLeg) {
			case LFrLegOrder:
				link=kine->getInterestPoint(solveLink,"ToeLBkPaw"); break;
			case RFrLegOrder:
				link=kine->getInterestPoint(solveLink,"ToeRBkPaw"); break;
			case LBkLegOrder:
				link=kine->getInterestPoint(solveLink,"LowerInnerBackLFrShin"); break;
			case RBkLegOrder:
				link=kine->getInterestPoint(solveLink,"LowerInnerBackRFrShin"); break;
			}
			
			//use the knee angle to assign distance from the solution point
			float dist=state->outputs[LegOffset+lastLeg*JointsPerLeg+KneeOffset];
			dist*=30/outputRanges[LegOffset+lastLeg*JointsPerLeg+KneeOffset][MaxRange]; //scale to go up to 3 cm away
			cout << "Distance is " << dist/10 << "cm" << endl;
			float curlen = fmat::SubVector<3>(link).norm();
			//Two ways to do the same thing:
			fmat::SubVector<3>(link)*=(dist+curlen)/curlen; //scale the vector components individually

			//Plan B:
			/*
			NEWMAT::ColumnVector obj(4);
			switch(lastLeg) {
			case LFrLegOrder:
				obj=kine->getInterestPoint(BaseFrameOffset,"LowerLeftBackLFrShin,LowerRightBackLFrShin"); break;
			case RFrLegOrder:
				obj=kine->getInterestPoint(BaseFrameOffset,"LowerLeftBackRFrShin,LowerRightBackRFrShin"); break;
			case LBkLegOrder:
				obj=kine->getInterestPoint(BaseFrameOffset,"ToeLBkPaw"); break;
			case RBkLegOrder:
				obj=kine->getInterestPoint(BaseFrameOffset,"ToeRBkPaw"); break;
			}
			if(obj(4)!=1)
				return;
			
			unsigned int solveLink=FootFrameOffset+((lastLeg+2)%NumLegs); //swap front/back
			NEWMAT::ColumnVector link(4);
			switch(lastLeg) {
			case LFrLegOrder:
				link=kine->getInterestPoint(solveLink,"ToeLBkPaw"); break;
			case RFrLegOrder:
				link=kine->getInterestPoint(solveLink,"ToeRBkPaw"); break;
			case LBkLegOrder:
				link=kine->getInterestPoint(solveLink,"LowerLeftBackLFrShin,LowerRightBackLFrShin"); break;
			case RBkLegOrder:
				link=kine->getInterestPoint(solveLink,"LowerLeftBackRFrShin,LowerRightBackRFrShin"); break;
			}
			if(link(4)!=1)
				return;
			*/

			pose->solveLinkPosition(obj,solveLink,link);

			//If you would like to verify the positiions of the back toes... (relative to body center)
			//cout << "L: " << kine->getInterestPoint(BaseFrameOffset,"ToeLBkPaw").t();
			//cout << "R: " << kine->getInterestPoint(BaseFrameOffset,"ToeRBkPaw").t();
			//cout << "Toe: " << pose->getInterestPoint(BaseFrameOffset,"ToeLBkPaw").t();
			//cout << "PawA: " << pose->getInterestPoint(BaseFrameOffset,"LBkPaw").t();
			//cout << "PawB: " << (pose->linkToLink(FootFrameOffset+LBkLegOrder,BaseFrameOffset)*Kinematics::pack(0,0,0)).t();
			
		} else {
			serr->printf("KinematicSample2Behavior: Unhandled event %s\n",event->getName().c_str());
		}
	}

	static std::string getClassDescription() { return "Uses kinematics to make the back toe (Toe{LR}BkPaw) touch the lower thigh (Lower{LeftBackL,RightBackR}FrThigh)"; }
	virtual std::string getDescription() const { return getClassDescription(); }
	
protected:
	LegOrder_t lastLeg; //!< the last leg to have its button pressed, i.e. the "source"
	MotionPtr<PostureMC> pose; //!< the PostureMC which does all the computation
};

REGISTER_BEHAVIOR_MENU(KinematicSample2Behavior,DEFAULT_TK_MENU"/Kinematics Demos");

#endif

/*! @file
 * @brief Defines KinematicSample2Behavior, which uses kinematics to make the back toe (Toe{LR}BkPaw) touch the lower thigh (Lower{LeftBackL,RightBackR}FrThigh)
 * @author ejt (Creator)
 */
