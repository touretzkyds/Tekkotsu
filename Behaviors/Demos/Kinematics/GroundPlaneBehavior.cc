#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_BUTTONS

#include "Behaviors/BehaviorBase.h"
#include "Motion/Kinematics.h"
#include "Motion/PIDMC.h"
#include "Motion/MotionManager.h"
#include "Shared/RobotInfo.h"
#include "Shared/WorldState.h"
#include "Events/EventRouter.h"
#include "Events/VisionObjectEvent.h"
#include "Shared/ProjectInterface.h"
#include "Sound/SoundManager.h"
#include "Shared/Config.h"
#include "Shared/debuget.h"
#include "IPC/SharedObject.h"

//! Reports the location of the center of the camera image on the ground plane
class GroundPlaneBehavior : public BehaviorBase {
public:
	//! constructor
	GroundPlaneBehavior()
		: BehaviorBase("GroundPlaneBehavior"),
		head_release(EventBase::buttonEGID,capabilities.findButtonOffset("HeadBut"),EventBase::activateETID,0),
		head_lock(EventBase::buttonEGID,capabilities.findButtonOffset("HeadBut"),EventBase::deactivateETID,0),
		target_toggle(EventBase::buttonEGID,capabilities.findButtonOffset("ChinBut"),EventBase::activateETID),
		targeting(false),
		visual_target(EventBase::visObjEGID,ProjectInterface::visOrangeSID,EventBase::statusETID),
		last_seen(0),
		clock(EventBase::timerEGID,0,EventBase::statusETID,250)
	{ }

	virtual void doStart() {
		BehaviorBase::doStart(); // do this first
		erouter->addListener(this,head_release);
		erouter->addListener(this,head_lock);
		erouter->addListener(this,target_toggle);
		erouter->addListener(this, EventBase::textmsgEGID);
		processEvent(clock);
	}
	
	virtual void doEvent() {
		using std::cout;
		using std::endl;
		if(clock == *event) {
#if !defined(TGT_HAS_ACCELEROMETERS) || !defined(TGT_HAS_LEGS)
			// we don't have legs or accelerometers, default to the 'basic' call
			PlaneEquation p=kine->calculateGroundPlane();
#else
			// Otherwise, let's break it down to show the intermediate steps...
			// you could just call calculateGroundPlane as shown above and this
			// will all be done for you:
			
			//This is the direction gravity is pulling - probably a good way to find out
			//the attitude of the robot, assuming it is not moving.
			//** Note that the LAccel sensor needs to be negated to match the coordinate system **//
			fmat::Column<3> down=Kinematics::pack(state->sensors[BAccelOffset],
							      -state->sensors[LAccelOffset],
							      state->sensors[DAccelOffset]);
			
			//Just for kicks, lets report which leg is off the ground
			cout << "I think leg " << kine->findUnusedLeg(down) << " is least used" << endl;

			//First we determine the ground plane
			PlaneEquation p=kine->calculateGroundPlane(down);
#endif
			cout << "Ground plane: " << p << endl;

			//Project to ground plane - we do it twice here, once for camera frame and once for base frame
			fmat::Column<3> ray = fmat::pack(0,0,1);
			//cout <<"Current head:\n" << state->outputs[HeadOffset] <<' '<< state->outputs[HeadOffset+1] <<' '<< state->outputs[HeadOffset+2] << endl;
			/*cout <<"Joint to Base Pan:\n" << kine->linkToBase(HeadPanOffset);
			cout <<"Joint to Base Tilt:\n" << kine->linkToBase(HeadTiltOffset);
			cout <<"Joint to Base Camera:\n" << kine->linkToBase(CameraFrameOffset);*/
			fmat::Column<4> hit=kine->projectToPlane(CameraFrameOffset,ray,BaseFrameOffset,p,CameraFrameOffset);
			cout << "Intersection_camera: " << hit << endl;
			hit=kine->projectToPlane(CameraFrameOffset,ray,BaseFrameOffset,p,BaseFrameOffset);
			cout << "Intersection_base: " << hit << endl;

		} else if(head_release == *event) {
#ifdef TGT_HAS_HEAD
			motman->addPrunableMotion(SharedObject<PIDMC>(HeadOffset,HeadOffset+NumHeadJoints,0));
			erouter->addTimer(this,clock);
			processEvent(clock);
#endif
		} else if(head_lock == *event) {
#ifdef TGT_HAS_HEAD
			motman->addPrunableMotion(SharedObject<PIDMC>(HeadOffset,HeadOffset+NumHeadJoints,1));
			erouter->removeTimer(this,clock);
#endif
		} else if(target_toggle==*event || event->getGeneratorID()==EventBase::textmsgEGID) {
			if(targeting) {
				erouter->removeListener(this,visual_target);
				targeting=false;
			} else {
				erouter->addListener(this,visual_target);
				targeting=true;
			}
			sndman->playFile("yap.wav");
		} else if(visual_target == *event) {
			cout << '.' << std::flush;
			if(get_time()<last_seen+1000)
				return;
			cout << endl;
			
			const VisionObjectEvent& visob = dynamic_cast<const VisionObjectEvent&>(*event);
			const float OBJ_RADIUS = 20; // height of object's centroid, we'll just assume 20mm radius
			
			cout << "Target pixel position: " << visob.getCenterX()<< " " << visob.getCenterY()
				<< " (est. dist " << visob.getDistanceEstimate(OBJ_RADIUS*2) << ", assuming object radius " << OBJ_RADIUS << "mm)" << endl;
			
			//First we determine the ground plane
			PlaneEquation p=kine->calculateGroundPlane();
			cout << "Ground plane: " << p << endl;
			
			//Project to ground plane - we do it twice here, once for camera frame and once for base frame
			fmat::Column<3> ray;
			config->vision.computeRay(visob.getCenterX(),visob.getCenterY(),ray[0],ray[1],ray[2]);
			cout << "ray_camera: " << ray << endl;
			//cout <<"Current head:\n"<<state->outputs[HeadOffset] <<' '<< state->outputs[HeadOffset+1] <<' '<< state->outputs[HeadOffset+2] << endl <<kine->getTransform(CameraFrameOffset);
			fmat::Column<4> hit=kine->projectToPlane(CameraFrameOffset,ray,BaseFrameOffset,p,CameraFrameOffset, OBJ_RADIUS);
			cout << "Intersection_camera: " << hit << " (dist " << hit.norm() << ")" << endl;
			hit=kine->projectToPlane(CameraFrameOffset,ray,BaseFrameOffset,p,BaseFrameOffset, OBJ_RADIUS);
			cout << "Intersection_base: " << hit << " (dist " << hit.norm() << ")" << endl;
			last_seen=get_time();
		} else {
			ASSERT(false,"unprocessed event " << event->getName() << endl);
		}
	}

	static std::string getClassDescription() { return "Reports the location of the center of the camera image on the ground plane"; }
	virtual std::string getDescription() const { return getClassDescription(); }
	
protected:
	EventBase head_release; //!< event template to match to signal the head's PID joints should be relaxed
	EventBase head_lock;    //!< event template to match to signal the head's PID joints should be powered up again
	EventBase target_toggle; //!< event template to indicate that the behavior should watch for visual targets are report their position
	bool targeting; //!< whether currently targeting
	EventBase visual_target; //!< event template for the object to track the position of
	unsigned int last_seen;  //!< time that the last report regarding visual_target's position was displayed
	EventBase clock;        //!< event template to match to signal a new round of calculations should be performed
};

REGISTER_BEHAVIOR_MENU(GroundPlaneBehavior,DEFAULT_TK_MENU"/Kinematics Demos");

#endif // check for TGT_HAS_BUTTONS

/*! @file
 * @brief Defines GroundPlaneBehavior, which reports the location of the center of the camera image on the ground plane
 * @author ejt (Creator)
 */
