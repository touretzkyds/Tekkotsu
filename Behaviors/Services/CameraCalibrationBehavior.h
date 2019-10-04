#ifndef _CameraCalibrationBehavior_h_
#define _CameraCalibrationBehavior_h_

//#include "Motion/LedMC.h"
//#include "Motion/MMAccessor.h"
#include "Motion/MotionManager.h"

#include "DualCoding/DualCoding.h"
using namespace DualCoding;

//Timers IDs and Intervals
#define SEARCH_TIMER_ID 7753
#define SEARCH_TIME_INTERVAL 5000
#define CHECK_IS_LED_TIMER_ID 6684
#define CHECK_IS_LED_TIMER_INVERVAL 2000
#define STOP_TIMER_ID 8832
#define STOP_TIME_INTERVAL 3000

//thresholds for blob size when converting diff sketch into blobs
#define MAX_LED_AREA_THRESHOLD 800
#define LED_AREA_THRESHOLD 100
#define WEAK_LED_AREA_THRESHOLD 30

#define PARTIAL_VISION_THRESHOLD 3 //number of consecutive times of seeing
                                   //small(large) diff blobs
                                   //before we drop(raise) our diff threshold
//threshold on the diff sketch to filter out noise (may change)
#define STARTING_DIFF_THRESHOLD 40

//These values move the camera half the size of the camera frame
#define SEARCH_PAN_INCREMENT -.25f
#define SEARCH_TILT_INCREMENT .2f

#define RIGHT_LED_MASK (1)
#define LEFT_LED_MASK (1<<9)

//! A behavior to calibrate the camera using the robots own LEDs as reference
class CameraCalibrationBehavior : public VisualRoutinesBehavior {
 public:
 CameraCalibrationBehavior() 
	 : VisualRoutinesBehavior("CameraCalibrationBehavior"),
		leds_id(MotionManager::invalid_MC_ID),
		headpointer_id(MotionManager::invalid_MC_ID),
		currentPan(RobotInfo::outputRanges[HeadOffset + PanOffset][MaxRange]),
		currentTilt(RobotInfo::outputRanges[HeadOffset + TiltOffset][MinRange]),
		leftLEDPan(0),leftLEDTilt(0),
		currentLEDMask(LEFT_LED_MASK),
		partial_vision_count(0),
		diffThreshold(STARTING_DIFF_THRESHOLD){}

	virtual ~CameraCalibrationBehavior() {}

  virtual void doStart();

	virtual void doStop();

	virtual void moveToNextSearchPoint();

	virtual void doEvent();

	static std::string getClassDescription() {return "Calibrate the camera using own LEDs";}
	virtual std::string getDescription() const {return getClassDescription();}

 protected:
	MotionManager::MC_ID leds_id;
	MotionManager::MC_ID headpointer_id;
	float currentPan, currentTilt;
	float leftLEDPan, leftLEDTilt;
	int currentLEDMask;
	int partial_vision_count;
	uint diffThreshold;

};


#endif
