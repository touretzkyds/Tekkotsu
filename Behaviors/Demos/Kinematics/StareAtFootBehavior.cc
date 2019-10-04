#include "Shared/RobotInfo.h"
#if defined(TGT_HAS_LEGS) && defined(TGT_HAS_HEAD) && !defined(TGT_IS_MANTIS)

#include "StareAtFootBehavior.h"

#include "Events/EventRouter.h"
#include "Motion/MMAccessor.h"
#include "Motion/PIDMC.h"
#include "Motion/HeadPointerMC.h"
#include "Motion/Kinematics.h"
#include "Motion/PostureMC.h"
#include "IPC/SharedObject.h"
#include "Shared/Config.h"
#include "Shared/fmat.h"
#include "Shared/WorldState.h"

// these are for drawing into the camera frame
#include "Shared/ProjectInterface.h"
#include "Vision/Graphics.h"
#include "Events/FilterBankEvent.h"
#include "Vision/RawCameraGenerator.h"
#include "Behaviors/Mon/RawCam.h"

REGISTER_BEHAVIOR_MENU(StareAtFootBehavior,DEFAULT_TK_MENU"/Kinematics Demos");

using namespace std; 

void StareAtFootBehavior::doStart() {
	addMotion(pointer);

	erouter->addListener(this,EventBase::sensorEGID); // updates head position to track foot
	erouter->addListener(this,EventBase::buttonEGID); // switches foot being tracked
	
	// draws a dot on the camera image where we think the toe is
	erouter->addListener(this,EventBase::visRawCameraEGID,ProjectInterface::visRawCameraSID,EventBase::statusETID);
}

void StareAtFootBehavior::doEvent() {
	if(event->getGeneratorID()==EventBase::buttonEGID) {
		//*******************************//
		//*** Switch the "active" toe ***//
		//*******************************//

#ifdef TGT_IS_AIBO
		if(event->getSourceID()==LFrPawOffset) {
			lastLeg=LFrLegOrder;
		} else if(event->getSourceID()==RFrPawOffset) {
			lastLeg=RFrLegOrder;
		} else
			return;
#else
		if(event->getTypeID()==EventBase::activateETID) {
			if(++lastLeg >= NumLegs)
				lastLeg=0;
		}
#endif
		if(event->getTypeID()==EventBase::activateETID) {
			std::cout << "Switching target point to " << getInterestPointName() << std::endl;
			std::cout << "Currently at " << kine->getInterestPoint(BaseFrameOffset,getInterestPointName()) << std::endl;
			
			unsigned int lastlegoff=LegOffset+lastLeg*JointsPerLeg;
			SharedObject<PIDMC> relaxLeg(lastlegoff,lastlegoff+JointsPerLeg,0);
			motman->addPrunableMotion(relaxLeg);
		} else if(event->getTypeID()==EventBase::deactivateETID) {
			unsigned int lastlegoff=LegOffset+lastLeg*JointsPerLeg;
			SharedObject<PIDMC> tightLeg(lastlegoff,lastlegoff+JointsPerLeg,1);
			motman->addPrunableMotion(tightLeg);
		}


	} else if(event->getGeneratorID()==EventBase::sensorEGID) {
		//***************************************************************************//
		//*** Update the position of the head based on new information on the toe ***//
		//***************************************************************************//

		//Ask kinematics for current location of that paw (this is the "objective" aka target)
		fmat::Column<3> Pobj=kine->getInterestPoint(BaseFrameOffset,getInterestPointName());

		//Now point the head there
		pointer->lookAtPoint(Pobj[0],Pobj[1],Pobj[2]); //keep head as far away as possible
		//Alternative method:
		// pointer->lookAtPoint(Pobj(1),Pobj(2),Pobj(3),80); //keep head 80mm away


	} else if(event->getGeneratorID()==EventBase::visRawCameraEGID) {
		//**************************************************************//
		//*** Draw into the camera frame to put a box around the toe ***//
		//**************************************************************//
		
		//same as above, but get the toe position relative to the camera
		fmat::Column<3> Pobj=kine->getInterestPoint(CameraFrameOffset,getInterestPointName());
		float x,y;
		config->vision.computePixel(Pobj[0],Pobj[1],Pobj[2],x,y);
		float r=atan2(10.f,Pobj.norm())/CameraHorizFOV/2; // size of rectangle based on distance
		
		//draw into the layer which will be sent to the gui
		//not doing any error checking on the type of the event or its source... cross your fingers ;)
		const FilterBankEvent& fbe=dynamic_cast<const FilterBankEvent&>(*event);
		unsigned chan=RawCameraGenerator::CHAN_Y;
		unsigned int layer=RawCam::getSourceLayer(chan,fbe.getNumLayers());
		Graphics g(*fbe.getSource(), layer, chan);
		//g.setColor(255);
		//g.drawRect(x-r,y-r,r*2,r*2);
			
		//this manually calculates pixel coordinates without applying
		// any camera calibration via config->vision.computePixel
		r=atan2(5.f,sqrt(Pobj.sumSq()))/CameraHorizFOV/2; //draws a rectangle half the previous size
		x= Pobj[0] / (Pobj[2]*tan(CameraHorizFOV/2));
		y= (Pobj[1] * config->vision.aspectRatio) / (Pobj[2]*tan(CameraVertFOV/2));
		g.setColor(192);
		g.drawRect(x-r,y-r,r*2,r*2);

	} else {
		serr->printf("StareAtFootBehavior: Unhandled event %s\n",event->getName().c_str());
	}
}

const char* StareAtFootBehavior::getInterestPointName() const {
#ifdef TGT_IS_AIBO
	return (lastLeg==LFrLegOrder) ? "ToeLFrPaw" : "ToeRFrPaw";
#else
	return outputNames[FootFrameOffset+lastLeg];
#endif
}

#endif // has legs

/*! @file
 * @brief Implements StareAtFootBehavior, which uses new-style ROBOOP kinematics to track the paw which last received a button press with the camera
 * @author ejt (Creator)
 */
