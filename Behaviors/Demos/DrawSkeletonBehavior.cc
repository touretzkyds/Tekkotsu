#include "Shared/RobotInfo.h"
#if defined(TGT_HAS_LEGS) && defined(TGT_HAS_CAMERA)

#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"

#include "Shared/ProjectInterface.h"
#include "Events/FilterBankEvent.h"
#include "Vision/Graphics.h"
#include "Shared/Config.h"
#include "Vision/RawCameraGenerator.h"
#include "Behaviors/Mon/RawCam.h"
#include "Motion/Kinematics.h"
#include "Shared/fmatSpatial.h"

//! Draws the kinematics "skeleton" on the camera frame
class DrawSkeletonBehavior : public BehaviorBase {
public:
	virtual void doStart() {
		erouter->addListener(this, EventBase::visRawCameraEGID,ProjectInterface::visRawCameraSID,EventBase::statusETID);
	}

	virtual void doEvent() {
		//We're assuming event is a filter bank event and that its source is not null...
		//You could add error checking code for that.
		const FilterBankEvent& fbe=dynamic_cast<const FilterBankEvent&>(*event);

		unsigned int chan=RawCameraGenerator::CHAN_U;
		unsigned int layer=RawCam::getSourceLayer(chan,fbe.getNumLayers());
		unsigned char color=48;
		float originBoxSize=.1f;
		
		Graphics g(*fbe.getSource(),layer,chan);
		g.setColor(color);

		fmat::Column<4> p=Kinematics::pack(0,0,0,1);
		for(unsigned int leg=0; leg<NumLegs; leg++) {
			float lastx,lasty,curx,cury;
			bool lastFront=false;
            #if defined(TGT_IS_MANTIS)
            const unsigned int JointsPerLeg = 4; // hack! fix it later
            #endif 
			bool front=getCameraPoint(LegOffset+leg*JointsPerLeg,p,lastx,lasty);
			if(front) {
				g.setColor(-color);
				g.drawRect(lastx-originBoxSize/2,lasty-originBoxSize/2,originBoxSize,originBoxSize);
			}
			for(unsigned int j=1; j<JointsPerLeg; j++) {
				//actually, there's only one more joint for this loop to process, but just for generality...
				lastFront=front;
				front=getCameraPoint(LegOffset+leg*JointsPerLeg+j,p,curx,cury);
				if(lastFront) {
					g.setColor(front?color:-color);
					g.drawLine(lastx,lasty,curx,cury);
				}
				if(front) {
					g.setColor(-color);
					g.drawRect(curx-originBoxSize/2,cury-originBoxSize/2,originBoxSize,originBoxSize);
				}
				lastx=curx;
				lasty=cury;
			}
			lastFront=front;
			front=getCameraPoint(FootFrameOffset+leg,p,curx,cury);
			if(lastFront) {
				g.setColor(front?color:-color);
				g.drawLine(lastx,lasty,curx,cury);
			}
			if(front) {
				g.setColor(-color);
				g.drawRect(curx-originBoxSize/2,cury-originBoxSize/2,originBoxSize,originBoxSize);
			}
			lastx=curx;
			lasty=cury;
		}
	}

	static std::string getClassDescription() { return "Draws the kinematics \"skeleton\" on the camera frame"; }
	virtual std::string getDescription() const { return getClassDescription(); }
	

protected:
	//! returns the @a x and @a y coordinates with the camera image corresponding to a point (@a p) in 3d space (relative to @a jointOffset)
  bool getCameraPoint(unsigned int jointOffset, const fmat::SubVector<4, const float>& p, float& x, float& y) {
		fmat::Transform T = kine->linkToLink(jointOffset,CameraFrameOffset);
		fmat::Column<4> o=T*p; //o is now the position of point p on the link, relative to the camera
		bool front=o[2]>=0;
		config->vision.computePixel(o[0],o[1],o[2],x,y);
		//cout << jointOffset << ' ' << o.t() << " -> " << x << ' ' << y << endl;
		return front;
	}
};

REGISTER_BEHAVIOR_MENU_OPT(DrawSkeletonBehavior,"Vision Pipeline",BEH_NONEXCLUSIVE);

#endif

/*! @file
 * @brief Defines DrawSkeletonBehavior, which draws the kinematics "skeleton" on the camera frame
 * @author ejt (Creator)
 */
