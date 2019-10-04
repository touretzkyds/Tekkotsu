#include "VisualOdometry.h"
#include "Behaviors/StateMachine.h"
#include "DualCoding/VRmixin.h"

//================ Optical Flow Odometry ================

void OpticalFlowOdometry::update(bool sleepOverride) {
  if ( !sleepOverride && !VRmixin::isWalkingFlag ) {
    sleeping = true;
    lastTranslation = 0;
    lastAngle = 0;
    return;
  }
  float lastFlow = integratedFlow;  // only valid if not sleeping
  updateFlow();
  if ( ! sleeping ) {
    lastTranslation = integratedFlow - lastFlow;
    lastAngle = lastTranslation / scalingFactor;
  } else {
    lastTranslation = 0;
    lastAngle = 0;
    sleeping = false;
    // std::cout << "Was Sleeping" << std::endl;
  }
}

/*
void OpticalFlowOdometry::update(bool ignoreSleep) {
  if(ignoreSleep) {
    float lastFlow = integratedFlow;
    updateFlow();
    lastTranslation = integratedFlow - lastFlow;
    lastAngle = lastTranslation / scalingFactor;
  } else {
    update();
  }
}
*/

void OpticalFlowOdometry::setConversionParameters(float slope, float offset) {
  scalingFactor = slope;
}


//================ Image Profile Odometry ================

void ImageProfileOdometry::update(bool sleepOverride) {
  if ( !sleepOverride && !VRmixin::isWalkingFlag ) {
    sleeping = true;
    lastTranslation = 0;
    lastAngle = 0;
    return;
  }
  buildImageProfile();
  if ( ! sleeping ) {
    lastTranslation = minTranslation();
    lastAngle = translationToAngle(lastTranslation);
  } else {
    lastTranslation = 0;
    lastAngle = 0;
    sleeping = false;
    // std::cout << "Was Sleeping" << std::endl;
  }
  displayImageProfile();
  displayTranslationProfile(); // note: currently no call to buildTranslationProfile
  swapCurrentProfile();
}


/*
void ImageProfileOdometry::update() {
  if (VRmixin::isWalkingFlag) {
    buildImageProfile();

    if(sleeping) {
      lastTranslation = 0;
      lastAngle = 0;
      sleeping = false;
      std::cout << "Was Sleeping" << std::endl;
    } else {
      lastTranslation = minTranslation();
      lastAngle = translationToAngle(lastTranslation);
    }

    swapCurrentProfile();
  } else {
    sleeping = true;
  }
}
*/

void ImageProfileOdometry::buildImageProfile() {
  static Sketch<uchar> rawY(VRmixin::sketchFromRawY(), "rawY", true);
  rawY = VRmixin::sketchFromRawY();

  for(unsigned int i = 0; i < currImageProfile->size(); i++) {
    (*currImageProfile)[i] = 0;
  }

  for(unsigned int i = 0; i < currImageProfile->size(); i++) {
    for(unsigned int j = 0; j < RobotInfo::CameraResolutionY; j++) {
      (*currImageProfile)[i] += rawY(i,j);
    }
  }
}

void ImageProfileOdometry::swapCurrentProfile() {
  if ( currImageProfile == &profile1 )
    currImageProfile = &profile2;
  else
    currImageProfile = &profile1;
}

void ImageProfileOdometry::buildTranslationProfile() {
  int upper = RobotInfo::CameraResolutionX/2;
  int lower = -upper;

  float currDistance;
  int index = 0;
  for(int i = lower; i < upper; i++, index++) {
    if(currImageProfile == &profile1)
      currDistance = differenceMeasure(i);
    else
      currDistance = differenceMeasure(i);

    translationProfile[index] = currDistance;
  }

  float maxElement = *(max_element(translationProfile.begin(), translationProfile.end()));

  const unsigned int width = RobotInfo::CameraResolutionX;
  const unsigned int height = RobotInfo::CameraResolutionY;
  for(unsigned int i = 0; i < width; i++) {
    translationProfile[i] = width - floor(translationProfile[i] / (maxElement/height));
  }
}

float ImageProfileOdometry::translationToAngle(int translation) {
  if ( slope == 0.0f )
    return 0.0f;
  else
    return ((float)translation + offset)/slope;
}

void ImageProfileOdometry::setConversionParameters(float tSlope, float tOffset) {
  this->slope = tSlope;
  this->offset = tOffset;
}


/**
   Calculate the difference between the current image profile and the previous one
   shifted to the right by s.
 */
float ImageProfileOdometry::differenceMeasure(int s) {
  bool direction = currImageProfile == &profile1;
  int bounds = RobotInfo::CameraResolutionX - fabs(s) - 1;
    
  float manDifference = 0.0f;
  float differenceComponent = 0.0f;
  for(int i = 0; i < bounds; i++) {
    int currIndex = i + std::max(0,s);
    int prevIndex = i - std::min(0,s);
    
    if(direction)
      differenceComponent = profile1[currIndex] - profile2[prevIndex];
    else
      differenceComponent = profile1[prevIndex] - profile2[currIndex];

    manDifference += fabs(differenceComponent);
  }

  return manDifference / (bounds + 1);
}



/**
   Returns the translation that minimizes the differenceMeasure
   between the two stored image profiles.
 */
int ImageProfileOdometry::minTranslation() {
  int upper = RobotInfo::CameraResolutionX/2;
  int lower = -upper;

  int minTrans = lower;
  float minDistance = 255 * RobotInfo::CameraResolutionX;
  float currDistance;
  for (int i = lower; i < upper; i++) {
    currDistance = differenceMeasure(i);
    if(currDistance < minDistance) {
      minDistance = currDistance;
      minTrans = i;
    }
  }

  return minTrans;
}

void ImageProfileOdometry::displayImageProfile() {
  static Sketch<bool> hist(visops::zeros(VRmixin::camSkS), "imageProfile", true);
  hist = false;

  const unsigned int MAX_INTENSITY = 255; // assume one-byte pixels
  const unsigned int height = RobotInfo::CameraResolutionY;
  for(unsigned int i = 0; i < RobotInfo::CameraResolutionX; i++) {
    float scaledHeight = (float)(*currImageProfile)[i] / MAX_INTENSITY;
    for(int j = height-1; j > height-scaledHeight-1; j--) {
      hist(i,j) = true;
    }
  }
}

void ImageProfileOdometry::displayTranslationProfile() {
  static Sketch<bool> transDisp(visops::zeros(VRmixin::camSkS), "translationProfile", true);

  transDisp = false;

  int scaledValue;
  const unsigned int width = RobotInfo::CameraResolutionX;
  for(unsigned int bin = 0; bin < width; bin++) {
    scaledValue = translationProfile[bin];
    for(unsigned int j = width-1; j > width-scaledValue-1; j--) {
      transDisp(bin,j) = true;
    }
  }
}

const float VisualOdometry::ANGULAR_RESOLUTION =
  RobotInfo::CameraResolutionX / (RobotInfo::CameraHorizFOV / M_PI * 180);
