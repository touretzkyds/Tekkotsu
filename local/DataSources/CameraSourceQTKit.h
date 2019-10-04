//-*-c++-*-
#ifndef INCLUDED_CameraSourceQTKit_h_
#define INCLUDED_CameraSourceQTKit_h_

#include "local/DataSource.h"
#include "Shared/plistCollections.h"
#include <QTKit/QTKit.h>
#include <CoreVideo/CVPixelBuffer.h> // for kCVPixelFormatTypes
#include <QuickTime/ImageCompression.h> // for kComponentVideoUnsigned

@class CameraSourceQTKitDelegate;

//! This interfaces with a specific camera through the QTKit API, which is the only capture interface which supports 64-bit on OS X
class CameraSourceQTKit : public DataSource, public virtual plist::Dictionary {
public:
	
	//! If true, will attempt to use Apple's Grand Central Dispatch to do block processing... this parallelizes image processing, may slightly increase total CPU usage but reduces per-frame wall time
	plist::Primitive<bool> parallel;
	
	//! If true, upsamples color channels horizontally to match Y channel, otherwise downsamples everything to common resolution (y/4, u,v/2); set to true to use full resolution of camera (either as the “full” layer or if you are accessing the “double” layer), set to false if you are using half-resolution as the resolution of the “full” layer
	plist::Primitive<bool> highRes;
	
	//! Controls the resolution layer at which the image should be processed.\n 0 indicates "automatic" mode (picks layer closest to image's resolution), positive numbers indicate the resolution layer directly.\n Negative values are relative to the number of layers marked available by the vision setup, so that typically -1 would correspond to the "double" layer, and -2 would correspond to the "full" layer.
	plist::Primitive<int> layer;
	
	//! Conversion formats supported for requesting camera output
	enum PixelFormat_t {
		TYPE_UNKNOWN=0, //!< should avoid requesting any format, see what the camera's “native” format is
		TYPE_YUVS = kComponentVideoUnsigned, //!< the native format for iSight camera on 2010 Macbook Pros, 10.6.3
		TYPE_2VUY = kCVPixelFormatType_422YpCbCr8, //!< alternative supported format of 2010 iSight camera on 10.6.3, byte reordering of #TYPE_YUVS
		TYPE_GRAY = kCVPixelFormatType_8IndexedGray_WhiteIsZero //!< alternative supported format of 2010 iSight camera on 10.6.3, Y channel only
	};
	
	//! If non-empty, requests the camera convert to the specified format (a four character code aka FourCC)
	plist::NamedEnumeration<PixelFormat_t> format;
	
	//! constructor
	CameraSourceQTKit(const std::string& srcName, QTCaptureDevice* capDevice)
		: DataSource(), parallel(true), highRes(false), layer(0), format(TYPE_UNKNOWN), name(srcName), device(capDevice), session(NULL), delegate(NULL), frame(0), lastTime(0), duration(0), oneFrame(false), frameLock(), frameCond()
	{
		init();
	}
	
	//! destructor
	~CameraSourceQTKit();
	
	virtual unsigned int nextTimestamp() { return static_cast<unsigned int>((lastTime+duration)*1000); }
	virtual const std::string& nextName() { return name; }
	
	virtual void registerSource();
	virtual void deregisterSource();
	virtual bool advance();
	
	//! called from #delegate with each frame
	void processImage(CVImageBufferRef videoFrame, QTSampleBuffer* sampleBuffer);

protected:
	void init(); //!< general initialization: add configuration entries, retain #device and create #delegate
	void doFreeze();
	void doUnfreeze();
	
	//! Returns the four-character-code as a string for display
	static std::string CC2Str(unsigned int fourcc);
	
	//! returns the display string as a four-character-code
	static unsigned int Str2CC(const std::string& fourcc);
	
	
	//! yuyv ordering, upsample by duplicating u and v columns
	/*! Uses CCIR601 range (e.g. Y ∈ [16,235] ) */
	void process_yuvs_U(const unsigned char * s, unsigned int srcWidth, unsigned int srcHeight);
	
	//! yuyv ordering, downsample to common resolution (y/4, u,v/2)
	/*! Uses CCIR601 range (e.g. Y ∈ [16,235] ) */
	void process_yuvs_D(const unsigned char * s, unsigned int srcWidth, unsigned int srcHeight);
	
	
	//! uyvy ordering, upsample by duplicating u and v columns
	/*! Uses CCIR601 range (e.g. Y ∈ [16,235] ) */
	void process_2vuy_D(const unsigned char * s, unsigned int srcWidth, unsigned int srcHeight);
	
	//! uyvy ordering, downsample to common resolution (y/4, u,v/2)
	/*! Uses CCIR601 range (e.g. Y ∈ [16,235] ) */
	void process_2vuy_U(const unsigned char * s, unsigned int srcWidth, unsigned int srcHeight);
	
	
	//! Grayscale, white is 0
	/*! Need to convert to CCIR601 (y = (255-g)·219/255+16) and fill in u=v=128 
	 *  You might think compressing the Y range would lower quality, but inspecting data from iSight indicates it originates as CCIR601,
	 *  with this setting showing gaps in the histogram as it was stretched to full range, so really this is just resetting */
	void process_grayscale_zerowhite(const unsigned char * s, unsigned int srcWidth, unsigned int srcHeight);

	std::string name;
	QTCaptureDevice* const device; //!< the camera associated with this instance
	QTCaptureSession* session; //!< the capture session created in registerSource (thus non-NULL if registered), used to start/stop capture when frozen
	CameraSourceQTKitDelegate* delegate; //!< receives per-frame callbacks from the #session, forwards calls to processImage()
	
	unsigned int frame; //!< a frame index counter
	float lastTime; //!< time in seconds of ideal frame arrival time, if drifts more than a frame duration, reset via get_time()
	float duration; //!< the frame duration encoded in the captured frame meta-data
	bool oneFrame; //!< set to true by advance(), indicates capture should stop once a frame is sent
	Thread::Lock frameLock; //!< held by advance() when waiting for a frame
	Thread::Condition frameCond; //!< condition to wake up advance when a frame has been sent
	
private:
	CameraSourceQTKit(const CameraSourceQTKit&); //!< do not copy
	CameraSourceQTKit& operator=(const CameraSourceQTKit&); //!< do not assign
};

@interface CameraSourceQTKitDelegate : NSObject {
@public
	CameraSourceQTKit * target;
}
-(CameraSourceQTKitDelegate*) initWithTarget:(CameraSourceQTKit*)tgt;
@end



/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
