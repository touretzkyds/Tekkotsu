//-*-c++-*-
#ifndef INCLUDED_CameraSourceQTSG_h_
#define INCLUDED_CameraSourceQTSG_h_

#include "local/DataSource.h"
#include "Shared/plistCollections.h"
#include "IPC/CallbackThread.h"
#include <iostream>
#include <stdexcept>
#include <map>
#include <cmath>

#define TARGET_API_MAC_CARBON 1
#include <Carbon/Carbon.h>
#include <QuickTime/QuickTime.h>

extern bool checkQTThreadInit();

//! This interfaces with a specific camera through QuickTime and the Sequence Grabber, which is deprecated.  See the alternative CameraSourceQTKit implementation.
/*! This camera source is used on pre-10.6 systems, supporting 32 bit only */
class CameraSourceQTSG : public DataSource, public virtual plist::Dictionary {
public:
	//! constructor, pass the short name, device name and input index
	CameraSourceQTSG(SeqGrabComponent grabber, const std::string& srcName, const std::string& devName, const std::string& inputName, int inputIdx)
		throw(std::pair<OSErr,const char*>)
		: DataSource(), layer(0), poller(&CameraSourceQTSG::poll,*this,TimeET(0L),TimeET(1.f/(*sensorFramerate)),true,CallbackPollThread::STOP_FALSE),
		sg(grabber), sgChan(NULL), gworld(NULL), name(srcName), deviceName(devName), devInputName(inputName), devInputIdx(inputIdx),
		frame(0), skipped(0), queuedFrames(0), grabbing(false),
		lastTime(0), duration(0), chanTimeScale(), chanTimeBase(), drawSeq(0), callbackerr(noErr),
		gworldBuf(NULL), burnIn(false)
	{
		initCamera();
			
		// plist dictionary stuff
		setLoadSavePolicy(FIXED,SYNC);
		addEntry("Layer",layer,"Controls the resolution layer at which the image should be processed.\n"
		    "0 indicates \"automatic\" mode (picks layer closest to image's resolution), positive numbers indicate the resolution layer directly.\n"
		    "Negative values are relative to the number of layers marked available by the vision setup, so that typically -1 would correspond to the \"double\" layer, and -2 would correspond to the \"full\" layer.");
	}
	
	//! destructor, free system resources
	~CameraSourceQTSG();
	
	//! accessor for the instance name
	virtual const std::string& getName() const { return name; }
	//! accessor for the device name
	virtual const std::string& getDeviceName() const { return deviceName; }
	//! accessor for the name of the input from the device
	virtual const std::string& getInputName() const { return devInputName; }
	
	virtual unsigned int nextTimestamp() { return static_cast<unsigned int>(std::ceil(lastTime+duration*1000)); }
	virtual const std::string& nextName() { return name; } //!< this is supposed to be the name for the next camera frame, we just reuse our static instance name
	
	virtual void registerSource();
	virtual void deregisterSource();
	virtual bool advance();
	
	plist::Primitive<int> layer; //!< Controls the resolution layer at which the image should be processed.\n 0 indicates "automatic" mode (picks layer closest to image's resolution), positive numbers indicate the resolution layer directly.\n Negative values are relative to the number of layers marked available by the vision setup, so that typically -1 would correspond to the "double" layer, and -2 would correspond to the "full" layer.
	
protected:
	void initCamera();
	
	void doFreeze();
	void doUnfreeze();

	
	//! converts from pascal-format string to c-format string
	static std::string p2c(unsigned char pascalStr[]) {
		unsigned char len = *pascalStr++;
		return std::string(reinterpret_cast<char*>(pascalStr),len);
	}

	static void dumpLiteral(OSType t);
	static ComponentResult setVideoChannelBounds(SGChannel videoChannel, const Rect *scaledVideoBounds);
	static pascal ComponentResult compressCompleteBottleProc(SGChannel c, UInt8 *queuedFrameCount, SGCompressInfo *ci, TimeRecord *t, long refCon);
	static pascal OSErr grabDataProc(SGChannel c, Ptr p, long len, long *offset, long chRefCon, TimeValue time, short writeType, long refCon);
	
	//! Resamples a YUV 4:2:2 (aka '2vuy') image to a YUV 4:4:4 form which is expected in getData()'s payload
	RCRegion * imgFrom2vuy(const unsigned char * s, short srcWidth, short srcHeight, short depth, long dataSize);
	//! Resamples a YUV 4:2:2 (aka 'yuv2' or 'yuvu') image to YUV 4:4:4 form which is expected in getData()'s payload
	RCRegion * imgFromyuv2(const unsigned char * s, short srcWidth, short srcHeight, short depth, long dataSize);
	
	bool poll() { Thread::NoCancelScope nc; advance(); return grabbing; }
	CallbackPollThread poller;
	
	SeqGrabComponent 	sg;	//!< sequence grabber, might need one of these globally instead of one per CameraSourceQTSG
	SGChannel			sgChan; //!< channel within the sequence grabber, I think there will be one channel per camera
	GWorldPtr 		gworld; //!< graphics buffer to store the decompressed image (RGB, will have to be converted back to YUV)
	std::string name; //!< name to use for image frames
	const std::string deviceName; //!< name of hardware device the source is connected
	const std::string devInputName; //!< name of the input on the device (in case a single device has multiple inputs)
	int devInputIdx; //!< index number of the input
		
	unsigned int frame; //!< current frame number
	unsigned int skipped; //!< number of frames skipped since last successful frame
	unsigned int queuedFrames; //!< number of frames the system indicates are waiting to be processed
	bool grabbing; //!< set to true when setDataSourceThread is called with a non-NULL value, but can be set back to false if an error occurs
	TimeValue 		 	lastTime;
	float duration;
	TimeScale 		 	chanTimeScale;
	TimeBase			chanTimeBase;
	ImageSequence 	 	drawSeq;	// unique identifier for our draw sequence
	OSErr callbackerr; //!< error value from call back, so we can tell if an error came from SGIdle itself or the callback functions it may trigger
	
	char * gworldBuf; //!< buffer used for #gworld
	bool burnIn; //!< set to true if the image should be skipped (e.g. clearing out a backlog of images when advancing while frozen
		
private:
	CameraSourceQTSG(const CameraSourceQTSG&); //!< don't call (copy constructor)
	CameraSourceQTSG& operator=(const CameraSourceQTSG&); //!< don't call (assignment operator)
};

/*! @file
 * @brief Describes CameraSourceQTSG, which interfaces with a specific camera through QuickTime and the Sequence Grabber, which is deprecated.  See the alternative CameraSourceQTKit implementation.
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
