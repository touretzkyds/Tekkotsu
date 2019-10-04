#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include <math.h>
#include <algorithm>
#include <vector>

#include "IPC/SharedObject.h"
#include "Motion/HeadPointerMC.h"
#include "Motion/MMAccessor.h"
#include "Shared/mathutils.h"
#include "DualCoding/Sketch.h"
#include "OpticalFlow.h"

namespace DualCoding {

  //! Generic visual odometry class; can be used for multiple algorithms
  class VisualOdometry {
  public:
    VisualOdometry() : sleeping(true), lastAngle(0.0), lastTranslation(0) {};

    virtual ~VisualOdometry() {}

    virtual void update(bool overrideSleep=false) = 0;
    virtual float getAngle() { return lastAngle; }
    virtual int getTranslation() { return lastTranslation; }
    virtual float getIntegratedAngle() { return 0.0f; }

    //! Set parameters for converting from translation units (pixels) to rotation angle (radians)
    virtual void setConversionParameters(float slope, float offset) = 0;

    //! Number of milliseconds between odometry updates.
    virtual unsigned int suggestedFrameRate() const = 0;

    //! Number of pixels of translation accross the camera frame per degree of camera rotation.
    //static const float ANGULAR_RESOLUTION = 10.0f;
    static const float ANGULAR_RESOLUTION;

  protected:
    bool sleeping;  //!< True if not currently subscribed to images
    float lastAngle;
    int lastTranslation;
  };


  //! Optical flow implementation of visual odometry
  class OpticalFlowOdometry : public OpticalFlow, public VisualOdometry {
  public:
    OpticalFlowOdometry(const std::string &dummyName="") :
      OpticalFlow(), VisualOdometry(), scalingFactor(ANGULAR_RESOLUTION) {};

    virtual void update(bool overrideSleep=false);

    //! Set parameters for converting from translation units (pixels) to angle (radians)
    virtual void setConversionParameters(float slope, float offset);

    virtual float getIntegratedAngle() { return integratedFlow / scalingFactor; }

    //! Number of milliseconds between odometry updates.
    virtual unsigned int suggestedFrameRate() const { return 100; }

  private:
    float scalingFactor;
  };


  //! Image profile visual odometry.
  
  class ImageProfileOdometry : public VisualOdometry {
  public:
    ImageProfileOdometry(const std::string &dummyName="") :
      VisualOdometry(), currImageProfile(&profile1),
      profile1(RobotInfo::CameraResolutionX), profile2(RobotInfo::CameraResolutionX),
      translationProfile(RobotInfo::CameraResolutionX),
      angularResolution(ANGULAR_RESOLUTION), slope(ANGULAR_RESOLUTION), offset(0.0f)
    {}

    ImageProfileOdometry(const ImageProfileOdometry &other); //!< do not call
    virtual ~ImageProfileOdometry() {}
    virtual float getAngle();
    virtual int getTranslation();
    virtual void update(bool sleepOverride=false);

    //! Set parameters for converting from translation units (pixels) to angle (radians)
    virtual void setConversionParameters(float slope, float offset);

    //! Number of milliseconds between odometry updates.
    virtual unsigned int suggestedFrameRate() const { return  4; }

    float getSlope() { return slope; }
    float getOffset() { return offset; }

  protected:
    std::vector<float> *currImageProfile;
    std::vector<float> profile1;
    std::vector<float> profile2;
    std::vector<float> translationProfile;

  private:
    ImageProfileOdometry & operator=(const ImageProfileOdometry &rhs);
    float angularResolution;
    float slope;
    float offset;

    float differenceMeasure(int s);
    int minTranslation();

    //! Convert from translation units (pixels) to angle (radians)
    float translationToAngle(int trans);

    //! Builds a translation profile comparing how similar the two image profiles are at different translations.
    /*! The horizontal axis is translation, starting at -WIDTH/2 on
        the leftmost part of the axis and ending with WIDTH/2 on the right.  

        The vertical axis is the difference measure between the two 
        image profiles using the translation at that point on the horizontal axis.
      */
    void buildImageProfile();

    void displayImageProfile();
    void swapCurrentProfile();
    void buildTranslationProfile();
    void displayTranslationProfile();
  };

  typedef OpticalFlowOdometry CurrVisualOdometry;

}

#endif
