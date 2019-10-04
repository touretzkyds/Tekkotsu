#ifdef __linux__
#ifdef HAVE_FLYCAP

#include "CameraDriverPtGrey.h"
#include "Shared/debuget.h"
#include "Shared/MarkScope.h"

using namespace std; 

const std::string CameraDriverPtGrey::autoRegisterCameraDriverPtGrey = DeviceDriver::getRegistry().registerType<CameraDriverPtGrey>("CameraPtGrey");

bool CameraDriverPtGrey::advance()
{
	MarkScope l(lock);

	//	cout << "CAMERA DRIVER: advancing" << endl;

	if (!cam.IsConnected()) {
		openCam();

		if (!cam.IsConnected()) {
			return false;
		}
	}
	
	FlyCapture2::Error error;
	FlyCapture2::Image rawImage;

	// Retrieve an image
	error = cam.RetrieveBuffer( &rawImage );
	if (error != FlyCapture2::PGRERROR_OK) {
		cout << "CAMERA DRIVER: error capturing image: " << error.GetDescription() << endl;
		return false;
	}

	if (!convertImage(rawImage)) {
		return false;
	}

	++frameCount;
	return true;
}

void CameraDriverPtGrey::registerSource() {
	openCam();
	index.addPrimitiveListener(this);
}

void CameraDriverPtGrey::deregisterSource() {
	index.removePrimitiveListener(this);
	closeCam();
}

void CameraDriverPtGrey::doUnfreeze() {
	thread.start();
}

void CameraDriverPtGrey::doFreeze() {
	if(thread.isStarted())
		thread.stop().join();
}

void CameraDriverPtGrey::threadrun() {
	while(advance())
		Thread::testCurrentCancel();
}


void CameraDriverPtGrey::plistValueChanged(const plist::PrimitiveBase& pl)
{
	if (&pl == &index) {
		MarkScope l(lock);
		openCam();
	}
}

void CameraDriverPtGrey::openCam()
{
	if (cam.IsConnected())
		closeCam();
	
	FlyCapture2::BusManager busMgr;
	FlyCapture2::Error error;

	unsigned int numCameras;
	error = busMgr.GetNumOfCameras(&numCameras);
	if (error != FlyCapture2::PGRERROR_OK) {
		cout << "CAMERA DRIVER: error detecting number of cameras: " << error.GetDescription() << endl;
		return;
	}
  
	cout << "CAMERA DRIVER: number of cameras detected: " << numCameras << endl;

	if (index >= numCameras) {
		cout << "CAMERA DRIVER: index out of bounds: " << index << endl;
		return;
	}

	FlyCapture2::PGRGuid guid;
	error = busMgr.GetCameraFromIndex(index, &guid);
	if (error != FlyCapture2::PGRERROR_OK) {
		cout << "CAMERA DRIVER: error getting camera GUID: " << error.GetDescription() << endl;
		return;
	}

	// Connect to a camera
	error = cam.Connect(&guid);
	if (error != FlyCapture2::PGRERROR_OK) {
		cout << "CAMERA DRIVER: error connecting camera: " << error.GetDescription() << endl;
		cam.Disconnect();
		return;
	}

	// Get the camera information
	FlyCapture2::CameraInfo camInfo;
	error = cam.GetCameraInfo(&camInfo);
	if (error != FlyCapture2::PGRERROR_OK) {
		cout << "CAMERA DRIVER: error getting camera info: " << error.GetDescription() << endl;
	}
	else {
		printf(
					 "\nCAMERA DRIVER: Using camera:\n"
					 "Serial number - %u\n"
					 "Camera model - %s\n"
					 "Camera vendor - %s\n"
					 "Sensor - %s\n"
					 "Resolution - %s\n"
					 "Firmware version - %s\n"
					 "Firmware build time - %s\n\n",
					 camInfo.serialNumber,
					 camInfo.modelName,
					 camInfo.vendorName,
					 camInfo.sensorInfo,
					 camInfo.sensorResolution,
					 camInfo.firmwareVersion,
					 camInfo.firmwareBuildTime );
	}

	// check desired resolution (TODO support other resolutions?)
	error = cam.SetVideoModeAndFrameRate(FlyCapture2::VIDEOMODE_640x480Y8, FlyCapture2::FRAMERATE_30);
	if (error != FlyCapture2::PGRERROR_OK) {
		cout << "CAMERA DRIVER: error setting capture mode: " << error.GetDescription() << endl;
		cam.Disconnect();
		return;
	}

	// start capture
	error = cam.StartCapture();
	if (error != FlyCapture2::PGRERROR_OK) {
		cout << "CAMERA DRIVER: error starting capture: " << error.GetDescription() << endl;
		cam.Disconnect();
		return;
	}
}

void CameraDriverPtGrey::closeCam()
{
	FlyCapture2::Error error;

	error = cam.StopCapture();
	if (error != FlyCapture2::PGRERROR_OK) {
		cout << "CAMERA DRIVER: error stopping capture: " << error.GetDescription() << endl;
	}

	error = cam.Disconnect();
	if (error != FlyCapture2::PGRERROR_OK) {
		cout << "CAMERA DRIVER: error disconnecting camera: " << error.GetDescription() << endl;
	}
}

bool CameraDriverPtGrey::convertImage(FlyCapture2::Image & rawImage)
{
	FlyCapture2::Error error;

	// Create a converted image
	FlyCapture2::Image convertedImage;
	
	// Convert the raw image
	error = rawImage.Convert( FlyCapture2::PIXEL_FORMAT_RGB8, &convertedImage );
	if (error != FlyCapture2::PGRERROR_OK) {
		cout << "CAMERA DRIVER: error converting image: " << error.GetDescription() << endl;
		return false;
	}

	unsigned int layer = 0;
	unsigned int components = 3;
	unsigned int width = convertedImage.GetCols();
	unsigned int height = convertedImage.GetRows();

	ssize_t reqSize = sizeof(ImageHeader) + width * height * components;
	RCRegion* region = getUnusedRegion(reqSize,0);
	unsigned char * buf = reinterpret_cast<unsigned char*>(region->Base());
	new (buf) ImageHeader(0, layer, width, height, components, frameCount, timestamp, nextName());

	unsigned char * data = convertedImage.GetData();
	unsigned int dataSize = convertedImage.GetDataSize();

	/*
	cout << "width : " << width << " height: " << height;
	cout << "CAMERA DRIVER: data size: " << dataSize << " calculated: " << width*height*components << endl;	
	*/

	unsigned char * dst = buf + sizeof(ImageHeader);
	unsigned char * src = data;

	while (src != (data + dataSize)) {
		int r, g, b;
		
		r=*src;
		++src;
		
		g=*src;
		++src;
		
		b=*src;
		++src;
						
		// check wikipedia yuv entry for explanation
		*dst++ = ((66*r + 129*g + 25*b + 128) >> 8) + 16;
		*dst++ = ((-38*r - 74*g + 112*b + 128) >> 8) + 128;
		*dst++ = ((112*r - 94*g - 18*b + 128) >> 8) + 128;
	}

	setImage(region);
	return true;
}

/*! @file
 * @brief 
 * @author Alex Grubb (agrubb1) (V4L2 code)
 * @author Ethan Tira-Thompson (ejt) (Framework)
 */

#endif // ifdef HAVE_FLYCAP

#endif // ifdef __linux__
