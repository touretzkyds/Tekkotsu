#include "Shared/RobotInfo.h"
#if defined(TGT_HAS_ARMS) && defined(TGT_HAS_HEAD)

#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"
#include "Events/VisionObjectEvent.h"
#include "Motion/PostureMC.h"
#include "Motion/HeadPointerMC.h"
#include "Motion/MotionPtr.h"
#include "Shared/MarkScope.h"
#include "Shared/ProjectInterface.h"
#include "Shared/fmat.h"


//! Attempts to move the arm to a pink target, using gradient descent, assuming an overhead camera (e.g. HandEye)
class ArmGrabTarget : public BehaviorBase {
public:
	//! constructor
	ArmGrabTarget() : BehaviorBase("ArmGrabTarget"), pose() {}

	//! set up the arm, put camera in initial pose
	virtual void doStart() {
		// limit how fast the arm moves:
		for(unsigned int i=ArmOffset; i<ArmOffset+NumArmJoints; ++i)
			pose->setMaxSpeed(i,(float)(M_PI*3/4));
		
		// set an arbitrary initial position (note we're assuming there are at least 3 arm joints...)
		ASSERTRET(NumArmJoints>=3,"Not enough arm joints on this model");
		pose->setOutputCmd(ArmOffset+0, -0.5f);
		pose->setOutputCmd(ArmOffset+1, (float)(M_PI/2));
		pose->setOutputCmd(ArmOffset+2, 0.5f);
		for(unsigned int i=ArmOffset+3; i<ArmOffset+NumArmJoints; ++i)
			pose->setOutputCmd(i,0);
		
		// same for the head...
		for(unsigned int i=HeadOffset; i<HeadOffset+NumHeadJoints; ++i) {
			pose->setMaxSpeed(i, (float)(M_PI/3));
			pose->setOutputCmd(i,0);
		}
		pose->setOutputCmd(HeadOffset+PanOffset, 0);
		pose->setOutputCmd(HeadOffset+TiltOffset, -1.45f);
		
		// activate the motion:
		addMotion(pose);
		
		// request pink blob detection
		erouter->addListener(this,EventBase::visObjEGID,ProjectInterface::visPinkBallSID,EventBase::statusETID);
	}
	
	virtual void doEvent() {
		const VisionObjectEvent* vis = dynamic_cast<const VisionObjectEvent*>(event);
		if(vis==NULL)
			return;
		
		// Find the world coordinates of the object, assuming it is sitting on the ground
		fmat::Column<4> hit = kine->projectToGround(*vis);
		std::cout << hit << " (est)" << std::endl;
		fmat::Column<3> tgt(hit);
		if(hit[3]!=1) // bad projection, (parallel to ground, on horizon)
			tgt*=1.0e5f; // for this example, handle by just stretching out in direction of target
		
		// For this demo, we want to solve for a point *in front of* the gripper, not the center of gripper
		const float X_OFFSET = 25;
		// Also offset target by height of gripper so we're solving for position on the ground
		// (an alternative is to pass object/gripper height to projectToGround() call above
		// to do the target projection to the plane of the gripper instead of the actual ground)
		const float Z_OFFSET = -( kine->getPosition(GripperFrameOffset)[2] );
		fmat::Column<3> gripPoint = fmat::pack(X_OFFSET,0,Z_OFFSET);
		
		// Claim a lock from here to the end of this scope
		// (if we left this out, a separate lock would be obtained for each access to 'pose'... your call)
		MarkScope poseLock(pose); 
		
		// Solve arm inverse kinematics (and move to point)
		bool success = pose->solveLinkPosition(tgt, GripperFrameOffset, gripPoint);
		
		// Verify and report solution
		fmat::Column<3> solvedPoint = pose->linkToBase(GripperFrameOffset) * gripPoint;
		float err = (solvedPoint - fmat::SubVector<3>(tgt)).norm();
		std::cout << solvedPoint << " (" << (success ? "reached" : "failed") << ", err " << err << ")" << std::endl;
		
		
		// Extra credit: move the head to look at the target
		// Extra-extra credit: we can use different methods for the head IK
		enum {
			NO_HEAD_TEST, //!< skip it altogether
			POINT_HEAD_TEST, //!< look directly at the object
			POINT_DIST_HEAD_TEST, //!< look directly at the object, but try to keep a certain distance
			DIR_HEAD_TEST //!< look parallel to the ray between the object and the neck
		
		// Some of these can set up oscillations with the head due to latency between camera
		// images, sensor readings, and motion output, so the default is to not use the head at all.
		// Might help to limit head motion with some kind of proportional control...
		} headTest = NO_HEAD_TEST;
		
		if(headTest!=NO_HEAD_TEST) {
			// A temporary HeadPointerMC for doing calculations… automatically initialized with current head position
			HeadPointerMC look;
			
			// only needed in case of DIR_HEAD_TEST
			fmat::Column<3> direction = pose->baseToLink(HeadOffset).translation() + fmat::SubVector<3>(tgt);
			
			// do the IK
			if(headTest==POINT_HEAD_TEST) {
				success = look.lookAtPoint(tgt[0],tgt[1],tgt[2]);
			} else if(headTest==POINT_DIST_HEAD_TEST) {
				success = look.lookAtPoint(tgt[0],tgt[1],tgt[2],300);
			} else if(headTest==DIR_HEAD_TEST) {
				success = look.lookInDirection(direction[0],direction[1],direction[2]);
			}
			
			// usually we'd just add the HeadPointerMC to the motion manager and let it move the joints itself,
			// but we want to do more calculations to verify the joint angles, so put results back in the PostureMC
			for(unsigned int i=0; i<NumHeadJoints; ++i)
				pose->setOutputCmd(HeadOffset+i, look.getJointValue(i));
			
			// now verify and report results
			if(headTest!=DIR_HEAD_TEST) {
				fmat::Column<4> camhit = pose->projectToGround(CameraFrameOffset, Kinematics::pack(0,0,1));
				fmat::Column<3> tgt_cam = pose->baseToLink(CameraFrameOffset) * fmat::SubVector<3>(tgt);
				tgt_cam/=tgt_cam.norm();
				err = std::acos(tgt_cam[2])/static_cast<float>(M_PI)*180;
				std::cout << camhit;
				if(!success)
					std::cout << " Look failed, ";
				std::cout << " Look error " << err << "°" << std::endl;
			} else {
				fmat::Column<3> tgt_cam = pose->linkToBase(CameraFrameOffset).rotation().column(2);
				fmat::Column<3> errv = tgt_cam - direction/direction.norm();
				std::cout << "Direction " << (success ? "reached" : "failed") << ", err: " << errv << ' ' << errv.norm() << std::endl;
			}
		}
	}

	static std::string getClassDescription() { return "Attempts to move the arm to a pink target"; }
	virtual std::string getDescription() const { return getClassDescription(); }
	

protected:
	MotionPtr<PostureMC> pose; //!< the PostureMC being used to control the head and arm

private:
	ArmGrabTarget(const ArmGrabTarget&); //!< don't call (copy constructor)
	ArmGrabTarget& operator=(const ArmGrabTarget&); //!< don't call (assignment operator)
};

REGISTER_BEHAVIOR_MENU(ArmGrabTarget,DEFAULT_TK_MENU"/Kinematics Demos");

#endif

/*! @file
 * @brief Defines ArmGrabTarget, which attempts to move the arm to a pink target
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
