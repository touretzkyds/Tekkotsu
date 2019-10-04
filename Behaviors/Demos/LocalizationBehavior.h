//-*-c++-*-
#ifndef INCLUDED_LocalizationBehavior_h_
#define INCLUDED_LocalizationBehavior_h_

#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"
#include "DualCoding/DualCoding.h"
//these are included indirectly via DualCoding.h:
//#include "Shared/ParticleFilter.h"
//#include "Localization/LocalizationParticle.h"

#include "Motion/LedMC.h"
#include "Motion/MMAccessor.h"
#include "Motion/MotionManager.h"

//! A behavior which tracks the robots location using a dual-coding-based "shape sensor"
/*! The behavior draws 10% of the particles on the world map for visualization, and updates
 *  the motion model once a second if no landmarks have been seen, so you can
 *  press "refresh" in the world map and get an updated view of how the particles
 *  have moved.
 *
 *  The landmarks are expected to be arranged as defined by setupLandmarksOnMap().
 *  If you want to define what your world looks like, edit the function to provide your own layout.*/
class LocalizationBehavior : public DualCoding::VisualRoutinesBehavior {
public:
	
	//! constructor
	LocalizationBehavior() : VisualRoutinesBehavior("LocalizationBehavior"),
		mapreq(DualCoding::MapBuilderRequest::localMap), lastvis(-1U), leds_id(MotionManager::invalid_MC_ID)
	{}

	virtual void doStart() {
		VisualRoutinesBehavior::doStart(); // do this first (required)
		
		// PARTICLE FILTER SETUP
		// have to wait to do this in doStart instead of constructor because the global
		// VRmixin::particleFilter isn't created until a client behavior is started.
		
		// changing some parameters in the default resampler
		typedef DualCoding::PFShapeLocalization::LowVarianceResamplingPolicy ResamplingPolicy;
		if(ResamplingPolicy* resample = dynamic_cast<ResamplingPolicy*>(particleFilter->getResamplingPolicy())) {
			// by default, resampling occurs on each update (delay=0)
			// Since groundplane projection is quite noisy while walking, we'll need quite a few more
			// samples before we get a good estimate of particle accuracy
			resample->resampleDelay=30;
			
			// by default the minimum is -FLT_MAX (essentially no minimum)
			// we'll require particles to have at least some feasability or we'll randomize to try to re-localize
			resample->minAcceptableWeight=log(1.0e-13); // logarthmic scale, avoids numeric issues (configurable)
		}
		
		// If you want to change the resampling variance:
		// This controls how "tight" the cluster can get after we evaluate the particles
		// Regarding the typedef: every resampling policy embeds a distribution policy, the
		// default policy is specified by the particle's own DistributionPolicy typedef
		typedef DualCoding::PFShapeLocalization::particle_type::DistributionPolicy DistributionPolicy;
		if(DistributionPolicy* dp = dynamic_cast<DistributionPolicy*>(&particleFilter->getResamplingPolicy()->getDistributionPolicy())) {
			dp->positionVariance*=1;
			dp->orientationVariance*=1;
		}
		
		// specifying motion model variance (how much do particles spread over time)
		// note that if we tracked robot velocity as a particle parameter, we could use resampling variance instead...
		// (and then get an estimate of actual velocity as well!)
		typedef HolonomicMotionModel<DualCoding::PFShapeLocalization::particle_type> MotionModel;
		if(MotionModel * motion = dynamic_cast<MotionModel*>(particleFilter->getMotionModel())) {
			// these are currently the default parameters, but explicitly reset for demonstration:
			motion->setVariance(.25,.25,.25);
		}
		
		
		// MAP BUILDER REQUEST SETUP
		const int pink_index = ProjectInterface::getColorIndex("pink");
		const int yellow_index = ProjectInterface::getColorIndex("yellow");
		//const int orange_index = ProjectInterface::getColorIndex("orange");
		mapreq.objectColors[DualCoding::ellipseDataType].insert(pink_index);
		mapreq.objectColors[DualCoding::ellipseDataType].insert(yellow_index);
		//mapreq.objectColors[ellipseDataType].insert(orange_index);
		
		// a hard coded gound plane corresponding to the default walk parameters
		// better to let it figure it out from sensors while walking, but this is better if standing still... (but only in the walk position!)
		//mapreq.setCustomGroundPlane(0.20944, 115);
		mapreq.maxDist=2000;
		
		
		// EVENT LISTENER SETUP
		// only do processing on camera frames where there's possibly something to see
		erouter->addListener(this, EventBase::visObjEGID, ProjectInterface::visPinkBallSID, EventBase::statusETID);
		erouter->addListener(this, EventBase::visObjEGID, ProjectInterface::visYellowBallSID, EventBase::statusETID);
		//erouter->addListener(this, EventBase::visObjEGID, //ProjectInterface::visOrangeSID, EventBase::statusETID);
		
		erouter->addTimer(this,-1U,1000,true); // update motion model regularly (even if not seeing anything)
		
		
		// MOTION COMMAND SETUP
		// using leds for displaying confidence
		SharedObject<LedMC> leds_mc;
		leds_id = motman->addPersistentMotion(leds_mc);
	}

	virtual void doStop() {
		motman->removeMotion(leds_id);
		VisualRoutinesBehavior::doStop(); // do this last (required)
	}
	
	//! this function defines the expected layout of the world
	void setupLandmarksOnMap() {
		// a pair of ellipses (pink and yellow)
		NEW_SHAPE(pinkm,DualCoding::EllipseData,new DualCoding::EllipseData(worldShS,DualCoding::Point(65,0,0,DualCoding::allocentric),27.5,27.5));
		pinkm->setColor("pink");
		pinkm->setMobile(false);
		NEW_SHAPE(yellowm,DualCoding::EllipseData,new DualCoding::EllipseData(worldShS,DualCoding::Point(-65,0,0,DualCoding::allocentric),27.5,27.5));
		yellowm->setColor("yellow");
		yellowm->setMobile(false);
		
		// a square marked by ellipses in each corner
		// pink on the "top" edge, and yellow on the "bottom"
		/*
		NEW_SHAPE(pinkm,EllipseData,new EllipseData(worldShS,Point(700,700,0,allocentric),27.5,27.5));
		pinkm->setColor("pink");
		pinkm->setMobile(false);
		NEW_SHAPE(yellowm,EllipseData,new EllipseData(worldShS,Point(700,0,0,allocentric),27.5,27.5));
		yellowm->setColor("yellow");
		yellowm->setMobile(false);
		NEW_SHAPE(pink2m,EllipseData,new EllipseData(worldShS,Point(0,700,0,allocentric),27.5,27.5));
		pink2m->setColor("pink");
		pink2m->setMobile(false);
		NEW_SHAPE(yellow2m,EllipseData,new EllipseData(worldShS,Point(0,0,0,allocentric),27.5,27.5));
		yellow2m->setColor("yellow");
		yellow2m->setMobile(false);
		 */
	}
	
	//! by uncommenting some of the erouter calls, you can make it skip frames (currently tries to process all)
	virtual void doEvent() {
		switch (event->getGeneratorID()){
		 
			case EventBase::visObjEGID:
			{
				// update local map
				const VisionObjectEvent &visev = dynamic_cast<const VisionObjectEvent&>(*event);
				unsigned int curvis = visev.getFrame();
				if (curvis == lastvis){
					std::cout << "Current = Previous" << std::endl;
					//erouter->removeListener(this,*event);
					//erouter->addTimer(this,event->getSourceID(),1000,false);
					return ; 
				}
				lastvis = curvis;
				std::cout << "Localizing off of " << event->getName() << "... " << std::endl;
			
				//unsigned int mapreq_id =
				mapBuilder.executeRequest(mapreq);
				// we're using an "immediate" map builder request, so we don't have to wait for the event...
				//erouter->addListener(this, EventBase::mapbuilderEGID, mapreq_id, EventBase::statusETID);
				
				// now update the particle filter
				worldShS.clear(); // clear particles and other junk we may have drawn on the map as feedback
				setupLandmarksOnMap(); // refresh the landmarks
				particleFilter->update(); // ask the particle filter to match the local space against the world space for each particle
				erouter->addTimer(this,-1U,1000,true); // delay motion model update (since we just updated on previous line)
				
				// now update the display
				updateAgentOnMap();
				displayParticlesOnMap();
				ledSettings();
				
				//erouter->removeListener(this,*event);
				//erouter->addTimer(this,event->getSourceID(), 1000,false);
				break;
			}
		
			case EventBase::timerEGID:
			{
				if(event->getSourceID()==-1U) {
					std::cout << "Updating motion model!" << std::endl;
					particleFilter->updateMotion();
					
					// refresh the display
					worldShS.clear();
					setupLandmarksOnMap();
					displayParticlesOnMap();
					updateAgentOnMap();
					
				} else {
					// this won't be called unless you comment-in the lines which skip camera frames
					std::cout << "Re-enabling vision listener" << std::endl;
					erouter->addListener(this, EventBase::visObjEGID, event->getSourceID(), EventBase::statusETID);
				}
				break;
			}
		
			default:
				std::cout << "Unexpected event: " << event->getDescription() << std::endl;
				break;
		}
	}

	//! places the agent at the most likely particle
	void updateAgentOnMap() {
		const DualCoding::PFShapeLocalization::particle_type& p = particleFilter->getBestParticle();
		std::cout << "Best index: " << particleFilter->getBestIndex() << " with score: " << p.weight << std::endl;
		mapBuilder.setAgent(DualCoding::Point(p.x,p.y), p.theta);
		std::cout << "DATA1 Current positions: " << p.x << " " << p.y << " " << p.theta << std::endl;
	}
	
	//! draws a sample (10%) of the particles on the map 
	void displayParticlesOnMap() {
		particleFilter->displayParticles();
		//or manually:
		/*if(particleFilter->getParticles().size() < 10)
			cout << "CURRENT PARTICLES LESS THAN 10!" << endl;
		for(unsigned int i = 0; i < particleFilter->getParticles().size(); i+=10) {
			const ShapeLocalizationPF::particle_type& p = particleFilter->getParticles()[i];
			NEW_SHAPE(pt, PointData, new PointData(worldShS, Point(p.x,p.y,0,allocentric)));
		}*/
	}
	
	//! updates the LEDs to show the confidence interval; the smaller the confidence interval, the more LEDs will turn on
	void ledSettings() {
		float confidence = particleFilter->getConfidenceInterval();
		std::cout << std::endl << "Confidence: " << confidence << std::endl << std::endl;
		MMAccessor<LedMC> leds_acc(leds_id);
		
		float c = 0;
		if(confidence < 500 && confidence>=0) {
			//linear dropoff:
			//	c=(1-confidence/500);
			
			//or exponential dropoff:
			c=exp(-(confidence-60)/150);
		}
		leds_acc->displayPercent(c,LedEngine::major,LedEngine::major);
	}
	
	static std::string getClassDescription() { return "A behavior which tracks the robots location using a dual-coding-based \"shape sensor\""; }
	virtual std::string getDescription() const { return getClassDescription(); }


protected:
	//! used to request the DualCoding::MapBuilder project cam-space shapes to the local-space
	DualCoding::MapBuilderRequest mapreq;
	
	//! serial number of the last processed camera frame
	/*! if there are multiple landmarks in the same image, we don't want to process
	 *  the same image multiple times for each landmark.  Tracking the camera frame
	 *  makes sure we only do each frame at most once */
	unsigned int lastvis;
	
	//! used to display the confidence of the particle filter on the LEDs
	MotionManager::MC_ID leds_id;


private:
	// Providing declarations for these functions will avoid a compiler warning if
	// you have any class members which are pointers.  However, as it is, an error
	// will result if you inadvertantly cause a call to either (which is probably
	// a good thing, unless you really intended to copy/assign a behavior, in
	// which case simply provide implementations for the functions)
	LocalizationBehavior(const LocalizationBehavior&); //!< don't call (copy constructor)
	LocalizationBehavior& operator=(const LocalizationBehavior&); //!< don't call (assignment operator)
};

/*! @file
 * @brief Defines LocalizationBehavior, which tracks the robots location using a dual-coding-based "shape sensor"
 * @author Ashley Johnson and Ethan Tira-Thompson (Creators)
 */

#endif
