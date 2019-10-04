#include "Shared/RobotInfo.h"
#if defined(TGT_IS_QWERK) && defined(TGT_HAS_HEAD)

#include "Motion/LedMC.h"
#include "Motion/MMAccessor.h"
//#include "Motion/MotionManager.h"

//#include "DualCoding/DualCoding.h"
//using namespace DualCoding;

#include "CameraCalibrationBehavior.h"

void CameraCalibrationBehavior::doStart() {
	VisualRoutinesBehavior::doStart();
#ifdef TGT_QBOTPLUS
	SharedObject<LedMC> leds_mc;
	leds_mc->set(currentLEDMask, 100);
	leds_mc->set(~currentLEDMask, 0);
	leds_id = motman->addPersistentMotion(leds_mc);
	
	SharedObject<HeadPointerMC> head_mc;
	head_mc->setJoints(0, currentPan, currentTilt);
	headpointer_id = motman->addPersistentMotion(head_mc);
	
	erouter->addTimer(this, SEARCH_TIMER_ID, SEARCH_TIME_INTERVAL, false);
#else
	std::cout << "Calibration routine enabled for non-QBotPlus robot." << std::endl;
	std::cout << "Stopping routine." << std::endl;
	doStop();
#endif

}

void CameraCalibrationBehavior::doStop() {
#ifdef TGT_QBOTPLUS
	motman->removeMotion(leds_id);
	motman->removeMotion(headpointer_id);
#endif

	VisualRoutinesBehavior::doStop();
}

void CameraCalibrationBehavior::moveToNextSearchPoint() {
	if ( (currentPan+=SEARCH_PAN_INCREMENT) <
			 RobotInfo::outputRanges[HeadOffset + PanOffset][MinRange]) {
		currentPan = RobotInfo::outputRanges[HeadOffset + PanOffset][MaxRange];
		currentTilt += SEARCH_TILT_INCREMENT;
	}
	if (currentTilt > RobotInfo::outputRanges[HeadOffset + TiltOffset][MaxRange]) {
		std::cout << "COMPLETE FAILURE!"<<std::endl
							<<"NO LED COULD BE FOUND!"<<std::endl;
		stop();
	}
	partial_vision_count = 0;
	diffThreshold = STARTING_DIFF_THRESHOLD;
	
	std::cout << "Checking joint values (" << currentPan
						<< "," <<currentTilt << ")" << std::endl;
	MMAccessor<HeadPointerMC>(headpointer_id)->
		setJoints(0, currentPan, currentTilt);
	erouter->addTimer(this, SEARCH_TIMER_ID, SEARCH_TIME_INTERVAL, false);
}

void CameraCalibrationBehavior::doEvent() {
	if (event->getGeneratorID() == EventBase::timerEGID) {
		if (event->getSourceID() == STOP_TIMER_ID) {
			stop();
			return;
		} else if (event->getSourceID() ==  SEARCH_TIMER_ID) {
			//we have just moved to a new search point
			//(or this search point again)

			std::cout << "Grabbing frame" << std::endl;
			camSkS.clear();
			
			NEW_SKETCH(on, uchar, sketchFromRawY());
			on->retain();
			
			MMAccessor<LedMC>(leds_id)->set(currentLEDMask, 0);
			erouter->addTimer(this, CHECK_IS_LED_TIMER_ID,
												CHECK_IS_LED_TIMER_INVERVAL, false);
			
		} else if (event->getSourceID() == CHECK_IS_LED_TIMER_ID) {
			//we haven't moved, but now the led is off, so we can compare
			//intensity sketches
			
			//turn the led back on
			MMAccessor<LedMC>(leds_id)->set(currentLEDMask, 100);
			
			
			const uint offset = 256;
			
			NEW_SKETCH(off, uchar, sketchFromRawY());
			GET_SKETCH(on, uchar, camSkS);
			on->retain(false);
			
			//add offset to the on sketch so that a pixel that is
			//slightly brighter in the off sketch does not appear as a 
			//large difference (because 100-101 = 254 in unsigned subtraction)
			NEW_SKETCH(diff, uint, (((Sketch<uint>)on)+offset) - (Sketch<uint>)off);
			
			//This does not scale properly
			diff->setColorMap(jetMapScaled);
			
			NEW_SKETCH(diffThresh , bool, diff > diffThreshold+offset);
			
			NEW_SKETCH(areas, DualCoding::uint, visops::areacc(diffThresh));
			std::cout << "Largest area found is " << areas->max() << std::endl;
			std::cout << "  Threshold used is " << diffThreshold << std::endl;
			
			NEW_SHAPEVEC(too_big_leds, BlobData,
									 BlobData::extractBlobs(diffThresh, MAX_LED_AREA_THRESHOLD));
			NEW_SHAPEVEC(possible_leds, BlobData,
									 BlobData::extractBlobs(diffThresh, LED_AREA_THRESHOLD));
			NEW_SHAPEVEC(too_small_leds, BlobData,
									 BlobData::extractBlobs(diffThresh, WEAK_LED_AREA_THRESHOLD));
			
			if (possible_leds.size() == 1 && too_big_leds.size() == 0) {
				//The led is in the vision frame, its homing time.
				BlobData led = possible_leds[0];
				
				std::cout << "Led located at (" << led.getCentroid().coordX()
									<< "," << led.getCentroid().coordY() << ")" << std::endl;
				//center at 160,120 and upper left is 0 0
				
				//if head close enough to center
				if ( fabs(led.getCentroid().coordX() - 160) < 16 &&
						 fabs(led.getCentroid().coordY() - 120) < 12) {
					
					//store point and switch target led
					std::cout << "Led locked on!" << std::endl;
					
					if (currentLEDMask == LEFT_LED_MASK) {
						//found left led
						leftLEDPan = state->outputs[HeadOffset + PanOffset];
						leftLEDTilt = state->outputs[HeadOffset + TiltOffset];
						
						//cheat a little, jump right to the next led (hopefully)
						currentPan = leftLEDPan + SEARCH_PAN_INCREMENT * 3;
						currentTilt = leftLEDTilt;
						
						currentLEDMask = RIGHT_LED_MASK;
						MMAccessor<LedMC>(leds_id)->set(currentLEDMask, 100);
						
						moveToNextSearchPoint();
						return;
					} else {
						//found right led
						float rightLEDPan = state->outputs[HeadOffset + PanOffset];
						float rightLEDTilt = state->outputs[HeadOffset + TiltOffset];
						float avgPan = (leftLEDPan+rightLEDPan)/2;
						float avgTilt = (leftLEDTilt+rightLEDTilt)/2;
						
						std::cout << "Pan difference is " << leftLEDPan-rightLEDPan
											<< ". Average is " << avgPan << std::endl;
						std::cout << "Tilt difference is " << leftLEDTilt-rightLEDTilt
											<< ". Average is " << avgTilt << std::endl;
						
						std::string filename = "ms/config/tekkotsu.xml";
						
						//expect averages to be 0 and -1.15
						config->motion.calibration_offset[HeadOffset+PanOffset] += avgPan;
						config->motion.calibration_offset[HeadOffset+TiltOffset]  += avgTilt + 1.15f;
						config->saveFile(filename.c_str());
						
						std::cout << "Changes saved to file:" << filename << std::endl;
						
						//reset and zero
						MMAccessor<HeadPointerMC>(headpointer_id)->setJoints(0,0,0);
						MMAccessor<LedMC>(leds_id)->set(AllLEDMask, 0);
						
						//let our motion commands complete before we stop
						erouter->addTimer(this, STOP_TIMER_ID, STOP_TIME_INTERVAL, false);
						return;
					}
					
				} else {//led not centered in vision frame
					//otherwise move head closer to led
					float xdiff = (led.getCentroid().coordX() - CameraResolutionX/2)/(CameraResolutionX/2);
					float ydiff = (led.getCentroid().coordY() - CameraResolutionY/2)/CameraResolutionY/2;
					std::cout << "Adjusting by " << xdiff << "(" 
										<< xdiff*SEARCH_PAN_INCREMENT
										<< "), " << ydiff << "(" 
										<< ydiff*SEARCH_TILT_INCREMENT << ")" << std::endl;
					MMAccessor<HeadPointerMC>(headpointer_id)->
						setJoints(0,
											state->outputs[HeadOffset + PanOffset] + xdiff*SEARCH_PAN_INCREMENT, 
											state->outputs[HeadOffset + TiltOffset] + -1*ydiff*SEARCH_TILT_INCREMENT);
				}
				
				//check this point again
				erouter->addTimer(this, SEARCH_TIMER_ID, SEARCH_TIME_INTERVAL, false);
				
			} else if (possible_leds.size() > 0 || too_small_leds.size() > 0) {
				//something is not right with the vision frame
				std::cout << "I see multiple LEDS or something that might be an LED!"
									<< "  I'm going to look again." << std::endl;
				
				//there are too many small dots
				if (too_small_leds.size() > 0 && too_big_leds.size() == 0) {
					if (++partial_vision_count >= PARTIAL_VISION_THRESHOLD) {
						std::cout << "reducing diff threshold" << std::endl;
						partial_vision_count = 0;
						diffThreshold = (diffThreshold * 3)/4;
					}
				} else if (too_big_leds.size() > 0) { //we see a plob too large
					if (--partial_vision_count <= -1 * PARTIAL_VISION_THRESHOLD) {
						std::cout << "increasing diff threshold" << std::endl;
						partial_vision_count = 0;
						diffThreshold = (diffThreshold * 3)/2;
					}
				}
				
				erouter->addTimer(this, SEARCH_TIMER_ID, SEARCH_TIME_INTERVAL, false);
				
			} else { //0 blobs of size > LED_AREA_THRESHOLD, go to next point
				moveToNextSearchPoint();
			}
		}
	}
}

#endif
