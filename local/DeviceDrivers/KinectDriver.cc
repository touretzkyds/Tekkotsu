#ifdef HAVE_OPENNI

#include <stdint.h>
#include "KinectDriver.h"

/****************
* Kinect Driver *
****************/

#define CHECK_RC(rc, what) \
  if (rc != XN_STATUS_OK) { \
	  cerr << "KinectDriver: " << what << " failed " << xnGetStatusString(rc) << endl; \
  }

#define CHECK_ERRORS(rc, errors, what) \
  if (rc == XN_STATUS_NO_NODE_PRESENT) { \
    XnChar strError[1024]; \
    errors.ToString(strError, 1024); \
    cerr << "KinectDriver: " << strError << endl; \
  }

const string KinectDriver::autoRegisterKinectDriver =
  DeviceDriver::getRegistry().registerType<KinectDriver>("Kinect");

KinectDriver::KinectDriver(const string& name) :
  DeviceDriver(autoRegisterKinectDriver,name),
  context(), scriptNode(), image(), depth(), rgbSource(this), depthSource(this) {
  // Load the OpenNI configuration file, which has the sensors available
  stringstream ss;
  ss << "ms/config/OpenNI-" << name << ".xml";
  xn::EnumerationErrors errors;
  XnStatus rc = context.InitFromXmlFile(ss.str().c_str(), scriptNode, &errors);
  CHECK_ERRORS(rc, errors, "InitFromXmlFile");
  CHECK_RC(rc, "InitFromXmlFile");

  // Fetch the image and depth generators
  context.FindExistingNode(XN_NODE_TYPE_IMAGE, image);
  context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth);

  // Start getting data
  context.StartGeneratingAll();
}

KinectDriver::~KinectDriver() { context.Release(); }

/*************
* RGB Source *
*************/

const string KinectDriver::RGBSource::instanceName = "RGBSource";

void KinectDriver::RGBSource::imageUpdateReceived(ImageGenerator &image, RGBSource *source) {
  // cout << instanceName << ": update received" << endl;
  source->advance();
}

bool KinectDriver::RGBSource::advance()
{
  driver->image.GetMetaData(metadata);
  // cout << instanceName << ": advance " << (unsigned int)metadata.FrameID() << " " << (long long)metadata.Timestamp() << endl;

  int width = metadata.XRes();
  int height = metadata.YRes();
  int components = 3;

  RCRegion* region = getUnusedRegion(width*height*components+sizeof(ImageHeader), 0);
  uint8_t * buf = reinterpret_cast<uint8_t*>(region->Base());
  new (buf) ImageHeader(ProjectInterface::visRawCameraSID, 0, width, height, components, frameNumber++, metadata.Timestamp(), nextName());
  uint8_t * yuv = buf+sizeof(ImageHeader);

  if (driver->image.GetPixelFormat() == XN_PIXEL_FORMAT_RGB24) {
    const XnRGB24Pixel *rgb = driver->image.GetRGB24ImageMap();
    for (int i=0; i<height*width; i++) {
      float R = rgb[i].nRed;
      float G = rgb[i].nGreen;
      float B = rgb[i].nBlue;
      *yuv++ = (0.257 * R) + (0.504 * G) + (0.098 * B) + 16;
      *yuv++ = -(0.148 * R) - (0.291 * G) + (0.439 * B) + 128;
      *yuv++ = (0.439 * R) - (0.368 * G) - (0.071 * B) + 128;
    }
  } else if (driver->image.GetPixelFormat() == XN_PIXEL_FORMAT_YUV422) {
    const XnYUV422DoublePixel *uyvy = driver->image.GetYUV422ImageMap();
    for (int i=0; i<height*width; i+=2) {
      *yuv++ = uyvy[i].nY1;
      *yuv++ = uyvy[i].nU;
      *yuv++ = uyvy[i].nV;
      *yuv++ = uyvy[i+1].nY2;
      *yuv++ = uyvy[i+1].nU;
      *yuv++ = uyvy[i+1].nV;
    }
  }

  setImage(region);
  return true;
}

void KinectDriver::RGBSource::doUnfreeze() {
  // listen for more data
  if ( driver->image.IsValid() )
    driver->image.RegisterToNewDataAvailable((StateChangedHandler)&imageUpdateReceived, this, cbHandle);
}

void KinectDriver::RGBSource::doFreeze() {
  // don't listen for more data
  if ( driver->image.IsValid() )
    driver->image.UnregisterFromNewDataAvailable(cbHandle);
}

/***************
* Depth Source *
***************/

const string KinectDriver::DepthSource::instanceName = "DepthSource";

void KinectDriver::DepthSource::depthUpdateReceived(DepthGenerator &depth, DepthSource *source) {
  // cout << instanceName << ": update received" << endl;
  source->advance();
}

bool KinectDriver::DepthSource::advance()
{
  driver->depth.GetMetaData(metadata);
  // cout << instanceName << ": advance " << (unsigned int)metadata.FrameID() << " " << (long long)metadata.Timestamp() << endl;

  int width = metadata.XRes();
  int height = metadata.YRes();
  int components = 2; // 2 byte short integer per pixel

  RCRegion* region = getUnusedRegion(width*height*components+sizeof(ImageHeader), 0);
  uint8_t * buf = reinterpret_cast<uint8_t*>(region->Base());
  new (buf) ImageHeader(ProjectInterface::visRawDepthSID, 0, width, height, components, frameNumber++, metadata.Timestamp(), nextName());
  uint16_t * rawmap = (uint16_t*)(buf+sizeof(ImageHeader));

  //  const XnDepthPixel *map = depth.GetDepthMap();
  const XnDepthPixel *map = metadata.Data();
  for (int i=0; i<height*width; i++) {
    *rawmap++ = map[i];
  }
  
  setImage(region);
  return true;
}

void KinectDriver::DepthSource::doUnfreeze() {
  // listen for more data
  if ( driver->depth.IsValid() )
    driver->depth.RegisterToNewDataAvailable((StateChangedHandler)&depthUpdateReceived, this, cbHandle);
}

void KinectDriver::DepthSource::doFreeze() {
  // don't listen for more data
  if ( driver->depth.IsValid() )
    driver->depth.UnregisterFromNewDataAvailable(cbHandle);
}

#endif /* HAVE_OPENNI */
