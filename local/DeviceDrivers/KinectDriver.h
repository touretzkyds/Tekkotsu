#ifndef _KINECT_DRIVER_H_
#define _KINECT_DRIVER_H_

/*! @file
 * @brief Driver that interfaces with OpenNI for Kinect
 * @author Jitu Das (cdas)
 */

#include <cstdio>

#include <local/DeviceDriver.h>
#include <local/DataSource.h>
#include <DualCoding/Point.h>
#include <Shared/ProjectInterface.h>

#include <XnOS.h>
#include <XnCppWrapper.h>

using namespace std;
using namespace xn;
using namespace DualCoding;

class KinectDriver : public virtual DeviceDriver {
 public:
  KinectDriver(const string& name);
  virtual ~KinectDriver();

  // DeviceDriver methods
  virtual string getClassName() const { return autoRegisterKinectDriver; }
  virtual void getImageSources(map<string,DataSource*>& sources) {
    sources.clear();
    sources["Camera"] = &rgbSource;
    sources["Depth"] = &depthSource;
  }

  Context context;
  ScriptNode scriptNode;
  ImageGenerator image;
  DepthGenerator depth;

 private:
  static const string autoRegisterKinectDriver;
	
  class RGBSource : public DataSource {
  public:
    RGBSource(KinectDriver *_driver) : DataSource(), driver(_driver), metadata(), cbHandle(NULL), frameNumber(0) { }
    RGBSource(const RGBSource&);
    ~RGBSource() { deregisterSource(); }

    RGBSource& operator= (const RGBSource&);

    // Datasource methods
    virtual bool advance();
    virtual unsigned int nextTimestamp() { return get_time(); }
    virtual const string& nextName() { return instanceName; }
    virtual void doFreeze();
    virtual void doUnfreeze();

  private:
    const static string instanceName;
    KinectDriver *driver;
    ImageMetaData metadata;
    XnCallbackHandle cbHandle;
		unsigned int frameNumber;
      
    static void imageUpdateReceived(ImageGenerator &image, RGBSource *source);
  } rgbSource;
	
  class DepthSource : public DataSource {
  public:
    DepthSource(KinectDriver * _driver) : DataSource(), driver(_driver), metadata(), cbHandle(NULL), frameNumber(0) { }
    DepthSource(const DepthSource&);
    ~DepthSource() { deregisterSource(); }

    DepthSource& operator= (const DepthSource&);

    // Datasource methods
    virtual bool advance();
    virtual unsigned int nextTimestamp() { return get_time(); }
    virtual const string& nextName() { return instanceName; }
    virtual void doFreeze();
    virtual void doUnfreeze();
    
  private:
    const static string instanceName;
    KinectDriver *driver;
    DepthMetaData metadata;
    XnCallbackHandle cbHandle;
		unsigned int frameNumber;

    static void depthUpdateReceived(DepthGenerator &depth, DepthSource *source);
  } depthSource;
};

#endif /* _KINECT_DRIVER_H_ */
