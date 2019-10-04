#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_HEAD

#include "Behaviors/BehaviorBase.h"
#include "Motion/HeadPointerMC.h"
#include "Motion/Kinematics.h"
#include "Motion/MotionPtr.h"
#include "Shared/fmatSpatial.h"
#include "Shared/TimeET.h"

//! A simple behavior to see how well LookAtPoint (inverse kinematics) is working
//! You may want to uncomment cout's in HeadPointer::LooAtPoint before running Behavior
class LookAtRandomPoints : public BehaviorBase {
public:

	LookAtRandomPoints() : BehaviorBase("LookAtRandomPoints"), pointer(), gazePt(fmat::pack(500, 0, 0)) {}

	virtual void doStart() {
		addMotion(pointer);
		erouter->addListener(this, EventBase::motmanEGID, pointer.getID(), EventBase::statusETID);
		erouter->addTimer(this, 1, 0, false);
	}

	virtual void doEvent() {
		switch (event->getGeneratorID()) {
		case EventBase::motmanEGID:
			if (event->getSourceID() == pointer.getID()) {
				erouter->addTimer(this, 0, 1000, false); // wait 1 sec for joints to stablize
			}
			break;
		case EventBase::timerEGID:
			if (event->getSourceID() == 0) {
				// uses accelerometers to determine "down", if available
				// if target model is legged, also tries to determine which legs are on the ground!
				PlaneEquation p=kine->calculateGroundPlane();

				fmat::Column<3> ray = Kinematics::pack(0,0,1);
				fmat::Column<4> hit = kine->projectToPlane(CameraFrameOffset, ray, BaseFrameOffset, p, BaseFrameOffset);

				fmat::Column<3> camPt = kine->baseToLink(CameraFrameOffset) * gazePt;
				fmat::Column<4> groundPt = kine->projectToPlane(CameraFrameOffset, camPt, BaseFrameOffset, p, BaseFrameOffset);
				float theta = std::acos(camPt[2]/camPt.norm());
				std::cout << "Result:" << std::endl;
				std::cout << " Cam Center projected to GroundPlane: " << hit << '\n';
				std::cout << " Gaze Point projected to GroundPlane: " << groundPt << '\n';
				std::cout << " Gaze Point in CameraPlane: " << camPt << '\n';
				std::cout << " theta: " << mathutils::rad2deg(theta) << " degrees\n";
				
				gazePt = fmat::pack(rand()%1000-200, rand()%1000-500, rand()%400-200); //set next gaze point
				erouter->addTimer(this, 1, 4000, false);
			} else {
				std::cout << "\n\n\nLookAtPoint: " << gazePt << '\n';
				TimeET t;
				bool b = pointer->lookAtPoint(gazePt[0],gazePt[1],gazePt[2]);
				t=t.Age();
				std::cout << " => " << (b ? "Reachable" : "Unreachable") << " took " << t << " seconds\n";
			}
			break;
		default:
			std::cout << "LookAtRandomPoints::doEvent(): unknown event\n";
			break;
		};
	}

protected:
	MotionPtr<HeadPointerMC> pointer;
	fmat::Column<3> gazePt;
};

REGISTER_BEHAVIOR_MENU(LookAtRandomPoints,DEFAULT_TK_MENU"/Kinematics Demos");

#endif // check for TGT_HAS_HEAD

/*! @file
 * @brief Defines LookAtRandomPoints, moves the head through a series of gaze points and reports if each is reachable to test inverse kinematic's lookAtPoint
 * @author dst (Creator)
 */
