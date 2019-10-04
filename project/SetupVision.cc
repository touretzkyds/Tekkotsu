#include "StartupBehavior.h"
#include "Shared/RobotInfo.h"

#include "Shared/ProjectInterface.h"
#include "Shared/Config.h"

#include "Behaviors/Controls/BehaviorSwitchControl.h"
#include "Behaviors/Controls/NullControl.h"

#ifdef PLATFORM_APERIOS
#  include "Vision/RawCameraGenerator.h"
#else
#  include "Vision/BufferedImageGenerator.h"
#endif
#include "Vision/InterleavedYUVGenerator.h"
#include "Vision/JPEGGenerator.h"
#include "Vision/PNGGenerator.h"
#include "Vision/SegmentedColorGenerator.h"
#include "Vision/RLEGenerator.h"
#include "Vision/RegionGenerator.h"
#include "Vision/BallDetectionGenerator.h"
//#include "Vision/CDTGenerator.h"

using namespace ProjectInterface;

BallDetectionGenerator * pball=NULL;
BallDetectionGenerator * bball=NULL;
BallDetectionGenerator * gball=NULL;
BallDetectionGenerator * yball=NULL;
BallDetectionGenerator * oball=NULL; //aka orange

/*! We set the default vision generators and source IDs here.
 *
 * The vis*SID and def*Generator variables are defined in
 * ProjectInterface.  You can reassign their values as you see fit,
 * which allows you to reorganize and adapt the vision pipeline, but
 * still retain the ability of the included demo behaviors to access
 * vision information.
 */
void
StartupBehavior::initVision() {
	//The value you set here will define the number of resolution layers available throughout the vision pipeline
	//If you change this value, you should modify the *Layer variables in ProjectInterface (if you want to use them)
	unsigned int numLayers=6;

#ifdef PLATFORM_APERIOS
	//The top level layer will be double resolution
	//This will set how many real layers are provided by the system (3 in Aperios)
	//Any other layers will be subsamples of the lowest resolution system resolution
	unsigned int numSystemLayers=3;
	defRawCameraGenerator = new RawCameraGenerator(numSystemLayers,numLayers,visRawCameraSID,EventBase::visOFbkEGID,0);
#else //PLATFORM_LOCAL
	defRawCameraGenerator = 
	  new BufferedImageGenerator("RawBufferedImageGenerator",EventBase::visRawCameraEGID,visRawCameraSID,
				     numLayers,EventBase::visOFbkEGID,visRawCameraSID);
	defRawDepthGenerator = 
	  new BufferedImageGenerator("DepthBufferedImageGenerator",EventBase::visRawDepthEGID,visRawDepthSID,
				     numLayers,EventBase::visOFbkEGID,visRawDepthSID);
#endif
	

	// These JPEG & PNG generators select the "deactivate" stage, so they will work on
	// the potentially marked up versions of the raw camera.  The camera GUIs use
	// these to provide vision feedback without requiring an extra pipeline stage
	// or image copy.
	// Of course, if you want to analyze the compression as a part of a
	// computer vision algorithm, you'll want the un-marked up versions, and so
	// you may need to add another compressor instance that uses the
	// "activate" stage to ensure you get the original data.

	//need to do pixel interleaving for JPEG & PNG compression
	//if the results of visRawCameraEGID are already in the proper format (e.g. source layer of BufferedImageGenerator) then InterleavedYUVGenerator is a pass-through
	defInterleavedYUVGenerator = new InterleavedYUVGenerator(visInterleaveSID,RawCameraGenerator::CHAN_Y,RawCameraGenerator::CHAN_U,RawCameraGenerator::CHAN_V,defRawCameraGenerator,EventBase::deactivateETID);
	
	defColorJPEGGenerator = new JPEGGenerator(visColorJPEGSID,defInterleavedYUVGenerator,EventBase::deactivateETID);
	defColorJPEGGenerator->setName("ColorJPEGGenerator");
	defGrayscaleJPEGGenerator = new JPEGGenerator(visGrayscaleJPEGSID,defRawCameraGenerator,EventBase::deactivateETID);
	defGrayscaleJPEGGenerator->setName("GrayscaleJPEGGenerator");

	defColorPNGGenerator = new PNGGenerator(visColorPNGSID,defInterleavedYUVGenerator,EventBase::deactivateETID);
	defColorPNGGenerator->setName("ColorPNGGenerator");
	defGrayscalePNGGenerator = new PNGGenerator(visGrayscalePNGSID,defRawCameraGenerator,EventBase::deactivateETID);
	defGrayscalePNGGenerator->setName("GrayscalePNGGenerator");

	// the hardware level CDT generator allows faster, but less flexible
	// segmenting it still needs a little work though before it can be
	// hooked up to the CMVision RLE generator.  See CDTGenerator class notes.
	// defSegmentedColorGenerator = new CDTGenerator(numSystemLayers,numLayers,EventBase::visOFbkEGID,0,visSegmentSID);
	defSegmentedColorGenerator = new SegmentedColorGenerator(visSegmentSID,defRawCameraGenerator,EventBase::activateETID);
	SegmentedColorGenerator * segcol = defSegmentedColorGenerator; //just using segcol as a shorthand for the following setup
	if(config->vision.colors[0]!='\0')
		segcol->loadColorInfo(config->vision.colors);
	for(unsigned int i=0; i<config->vision.thresh.size(); i++)
		segcol->loadThresholdMap(config->vision.thresh[i]);

	// Note this uses the "activate" stage, so you can mark up segmented images as well
	defRLEGenerator = new RLEGenerator(visRLESID,segcol,EventBase::activateETID);
	
	defRegionGenerator = new RegionGenerator(visRegionSID, defRLEGenerator, EventBase::activateETID);
	
	// for lack of a better idea, the object recognizers below will all
	// use the config->vision.rlecam_channel for picking the threshold
	// file to use
	// note that this is done here just once with the initial value, but
	// the detectors won't get reloaded if you change the rlecam_channel
	// later on

	// these names match up with /ms/config/default.col - the default
	// color information... if that changes, these should change too
	unsigned int threshChan=config->vision.segcam.channel;

	// higher value reduce false events, but increase reaction time [0-inf]
	unsigned int noiseFiltering=1;

	// lower values increase sensitivity (and noise) [0-1]
	float confidenceThreshold=.8f;

	unsigned int pinkIdx=segcol->getColorIndex("pink");
	if(pinkIdx!=-1U) {
		pball = new BallDetectionGenerator(visPinkBallSID,defRegionGenerator,pinkIdx,threshChan,noiseFiltering,confidenceThreshold);
		pball->setName("PinkBallDetectionGenerator");
	}
	unsigned int blueIdx=segcol->getColorIndex("blue");
	if(blueIdx!=-1U) {
		bball = new BallDetectionGenerator(visBlueBallSID,defRegionGenerator,blueIdx,threshChan,noiseFiltering,confidenceThreshold);
		bball->setName("BlueBallDetectionGenerator");
	}
	unsigned int greenIdx=segcol->getColorIndex("green");
	if(greenIdx!=-1U) {
		gball = new BallDetectionGenerator(visGreenBallSID,defRegionGenerator,greenIdx,threshChan,noiseFiltering,confidenceThreshold);
		gball->setName("GreenBallDetectionGenerator");
	}
	unsigned int yellowIdx=segcol->getColorIndex("yellow");
	if(yellowIdx!=-1U) {
		yball = new BallDetectionGenerator(visYellowBallSID,defRegionGenerator,yellowIdx,threshChan,noiseFiltering,confidenceThreshold);
		yball->setName("YellowBallDetectionGenerator");
	}
	unsigned int skinIdx=segcol->getColorIndex("orange");
	if(skinIdx!=-1U) {
		oball = new BallDetectionGenerator(visOrangeSID,defRegionGenerator,skinIdx,threshChan,noiseFiltering,confidenceThreshold);
		oball->setName("OrangeBallDetectionGenerator");
	}
}

ControlBase*
StartupBehavior::SetupVision() {
	startSubMenu("Vision Pipeline","Start/Stop stages of the vision pipeline");
	{
		addItem(NULL);

		//check that each generator is actually defined before trying to start it
		//just as a failsafe in case some of the default generators are disabled
		
		if(defRawCameraGenerator)
			addItem((new BehaviorSwitchControlBase(defRawCameraGenerator))->start());

		if(defRawDepthGenerator)
			addItem((new BehaviorSwitchControlBase(defRawDepthGenerator))->start());

		if(defInterleavedYUVGenerator)
			addItem((new BehaviorSwitchControlBase(defInterleavedYUVGenerator))->start());

		if(defColorJPEGGenerator)
			addItem((new BehaviorSwitchControlBase(defColorJPEGGenerator))->start());
		
		if(defGrayscaleJPEGGenerator)
			addItem((new BehaviorSwitchControlBase(defGrayscaleJPEGGenerator))->start());
		
		if(defColorPNGGenerator)
			addItem((new BehaviorSwitchControlBase(defColorPNGGenerator))->start());
		
		if(defGrayscalePNGGenerator)
			addItem((new BehaviorSwitchControlBase(defGrayscalePNGGenerator))->start());
		
		if(config->vision.colors!="" && config->vision.thresh.size()>0)
			addItem((new BehaviorSwitchControlBase(defSegmentedColorGenerator))->start());
		else
			addItem(new BehaviorSwitchControlBase(defSegmentedColorGenerator));
		
		if(defRLEGenerator)
			addItem((new BehaviorSwitchControlBase(defRLEGenerator))->start());

		if(defRegionGenerator)
			addItem((new BehaviorSwitchControlBase(defRegionGenerator))->start());

		if(pball)
			addItem((new BehaviorSwitchControlBase(pball))->start());
		else
			addItem(new NullControl("Error: PinkBallDetectionGenerator","Color \"pink\" is undefined"));

		if(bball)
			addItem((new BehaviorSwitchControlBase(bball))->start());
		else
			addItem(new NullControl("Error: BlueBallDetectionGenerator","Color \"blue\" is undefined"));

		if(gball)
			addItem((new BehaviorSwitchControlBase(gball))->start());
		else
			addItem(new NullControl("Error: BlueBallDetectionGenerator","Color \"green\" is undefined"));

		if(yball)
			addItem((new BehaviorSwitchControlBase(yball))->start());
		else
			addItem(new NullControl("Error: BlueBallDetectionGenerator","Color \"yellow\" is undefined"));

		if(oball)
			addItem((new BehaviorSwitchControlBase(oball))->start());
		else
			addItem(new NullControl("Error: OrangeBallDetectionGenerator","Color \"orange\" is undefined"));
	}
	return endSubMenu();
}
