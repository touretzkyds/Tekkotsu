#if defined(__APPLE__) && !defined(__x86_64__)

#include <AvailabilityMacros.h>
#ifndef MAC_OS_X_VERSION_10_6

#include "CameraSourceQTSG.h"
#include "Shared/LoadSave.h"
#include "Shared/get_time.h"
#include "Shared/RobotInfo.h"
#include <pthread.h>
#include <set>
#include <sstream>
#include <iostream>

using namespace std;

static pthread_key_t qtInit;
struct QTThreadInfo {};

static bool doGlobalQTInit();
static bool autoRegisterQTInit = doGlobalQTInit();

bool checkQTThreadInit() {
	if(!autoRegisterQTInit)
		return false;
	if(pthread_getspecific(qtInit)!=NULL)
		return true;
	OSErr err = EnterMoviesOnThread(kQTEnterMoviesFlagDontSetComponentsThreadMode);
	if(err!=noErr) {
		cerr << "CameraSource: EnterMoviesOnThread returned error " << err << endl;
		return false;
	}
	pthread_setspecific(qtInit,new QTThreadInfo);
	return true;
}
static void qtThreadDestructor(void* threadInfo) {
	ExitMoviesOnThread();
	delete static_cast<QTThreadInfo*>(threadInfo);
}
static bool doGlobalQTInit() {
	int err = pthread_key_create(&qtInit,qtThreadDestructor);
	if(err!=0)
		cerr << "CameraSource: error during doGlobalQTInit, pthread_key_create:" << strerror(err) << endl;
	return (err==0);
}

CameraSourceQTSG::~CameraSourceQTSG() {
	if(sgChan!=NULL)
		SGDisposeChannel(sg,sgChan);
	sgChan=NULL;
	if(gworld!=NULL)
		DisposeGWorld(gworld);
	gworld=NULL;
	// docs say that QTNewGWorldFromPtr is supposed to mark the underlying buffer
	// so that DisposeGWorld doesn't release it, but that doesn't seem to be the case...
	// If we delete here, we get double-free warnings.
	/*if(gworldBuf!=NULL)
		free(gworldBuf);*/
	gworldBuf=NULL;
	if(sg!=NULL)
		CloseComponent(sg);
	sg=NULL;
}

void CameraSourceQTSG::initCamera() {
	OSErr err=noErr;
	
	// open the sequence grabber, assuming there's only ever one component of this type listed
	sg = OpenDefaultComponent(SeqGrabComponentType, 0);
	if(sg==NULL) throw std::make_pair(err,"OpenDefaultComponent(SeqGrabComponentType,0)");
	
	// initialize the default sequence grabber component
	err = SGInitialize(sg);
	if(err!=noErr) throw std::make_pair(err,"SGInitialize");
	
	err = SGSetDataRef(sg, 0, 0, seqGrabToMemory | seqGrabDontMakeMovie | seqGrabDataProcIsInterruptSafe);
	if(err!=noErr) throw "SGSetDataRef";
		
	// this section would get the "default" capture device
	/*
	ComponentDescription searchCompDesc;
	memset(&searchCompDesc,0,sizeof(searchCompDesc));
	searchCompDesc.componentType=SeqGrabChannelType;
	searchCompDesc.componentSubType=VideoMediaType;
	Component chanComponent = FindNextComponent(chanComponent,&searchCompDesc);
	if(chanComponent==0) throw std::make_pair(err,"FindNextComponent");
		
	SGNewChannelFromComponent(sg,&sgChan,chanComponent);
	if(err!=noErr) throw std::make_pair(err,"SGNewChannelFromComponent");
	 */

	// instead, open the *specified* capture device
	// thanks Harald ( hxr AT users sourceforge net ) for 'wacaw' source to demonstrate this
	err = SGNewChannel(sg, VideoMediaType, &sgChan);
	if(err!=noErr) throw std::make_pair(err,"SGNewChannel");
	
	unsigned char pstr[256]; // sigh, convert devName to pascal-style string
	pstr[0]=deviceName.size();
	memcpy(pstr+1,deviceName.c_str(),pstr[0]);
	err = SGSetChannelDevice(sgChan, pstr);
	if(err!=noErr) throw std::make_pair(err,"SGSetChannelDevice");
	err = SGSetChannelDeviceInput(sgChan,devInputIdx);
	if(err!=noErr) throw std::make_pair(err,"SGSetChannelDeviceInput");
	
	// does this need to be done? not found in any sample code...
	// doesn't seem to work here in any case... 
	// (error -9400: noDeviceForChannel; tried before or after the SGSetChannelDevice()...)
	/*err = SGInitChannel(sgChan,sg);
	if(err!=noErr) throw std::make_pair(err,"SGInitChannel");*/
			
	// get the active rectangle 
	Rect  srcBounds;
	err = SGGetSrcVideoBounds(sgChan, &srcBounds);
	if(err!=noErr)
		std::cerr << "Warning: CameraSource SGGetSrcVideoBounds returned error " << err << std::endl;
	else {
		srcBounds.right -= srcBounds.left;
		srcBounds.bottom -= srcBounds.top;
		srcBounds.left = srcBounds.top = 0;
		//cout << "original bounds " << srcBounds.right << "x" << srcBounds.bottom << endl;
		if((unsigned int)srcBounds.right>CameraResolutionX*2 || (unsigned int)srcBounds.bottom>CameraResolutionY*2) {
			srcBounds.right=CameraResolutionX*2;
			srcBounds.bottom=CameraResolutionY*2;
			//cout << "restricting to " << srcBounds.right << "x" << srcBounds.bottom << endl;
		}
		err = SGSetChannelBounds(sgChan, &srcBounds);
		if(err!=noErr) std::cerr << "Warning: SGSetChannelBounds returned error " << err << std::endl;
	}
	
	// set options for our expected usage
	err = SGSetChannelUsage(sgChan, seqGrabRecord | seqGrabLowLatencyCapture /* | seqGrabPreview | seqGrabAlwaysUseTimeBase */);
	if(err!=noErr) throw std::make_pair(err,"SGSetChannelUsage");
	
	// don't need a gworld (yet) -- we want to grab the raw YUV bytes from the camera
	/*unsigned int width = srcBounds.right-srcBounds.left;
	unsigned int height = srcBounds.bottom-srcBounds.top;
	gworldBuf = new char[width*height*4];
	err = QTNewGWorldFromPtr(&gworld, k32ARGBPixelFormat, &srcBounds, NULL, NULL, 0, gworldBuf, width*4);
	if(err!=noErr) throw std::make_pair(err,"QTNewGWorldFromPtr"); */
	
	// still have to call SGSetGWorld() though or else SGPrepare() will complain later
	err = SGSetGWorld(sg, NULL, NULL);
	if(err!=noErr) throw std::make_pair(err,"SGSetGWorld");
	
	//SGSettingsDialog(sg, sgChan, 0, nil, 0, nil, 0);
	
	// set up the video bottlenecks so we can get our queued frame count
	VideoBottles vb;
	memset(&vb,0,sizeof(VideoBottles));
	err = SGGetVideoBottlenecks(sgChan, &vb);
	if(err!=noErr) throw std::make_pair(err,"SGGetVideoBottlenecks");
	
	vb.procCount = 9; // there are 9 bottleneck procs; this must be filled in
	vb.grabCompressCompleteProc = NewSGGrabCompressCompleteBottleUPP(compressCompleteBottleProc);
	
	err = SGSetVideoBottlenecks(sgChan, &vb);
	if(err!=noErr) throw std::make_pair(err,"SGSetVideoBottlenecks");
	
	// specify a sequence grabber data function
	err = SGSetDataProc(sg, NewSGDataUPP(grabDataProc), (long)this);
	if(err!=noErr) throw std::make_pair(err,"SGSetDataProc");
	err = SGSetChannelRefCon(sgChan, (long)this); // callback reference context
	if(err!=noErr) throw std::make_pair(err,"SGSetChannelRefCon");

	// try to switch to YUV mode
	err = SGSetVideoCompressorType(sgChan,k422YpCbCr8CodecType); // produces 2vuy images
	if(err!=noErr) {
		if(err!=noCodecErr) std::cerr << "    Could not switch to yuv codec (k422YpCbCr8CodecType), err " << err << std::endl;
		
		// try component video...
		// Actually, this is a little slower than converting each frame individually, so we'll do that instead.
		// See color space specification in NewGWorld call below...
		/*err = SGSetVideoCompressorType(sgChan,kComponentVideoCodecType); // produces yuv2/yuvu images
		if(err==noCodecErr) std::cerr << "    Could not switch to yuv codec (k422YpCbCr8CodecType or kComponentVideoCodecType), not supported by camera (?)" << std::endl;
		else if(err!=noErr) std::cerr << "    Could not switch to yuv codec (kComponentVideoCodecType), err " << err << std::endl;*/
	}
	
	// just for debugging...
	/*OSType ct;
	err = SGGetVideoCompressorType(sgChan,&ct);
	if(err!=noErr) std::cerr << "    Could not get current codec" << std::endl;
	else { std::cout << "    Current codec type is "; dumpLiteral(ct); std::cout << std::endl; }*/
}


void CameraSourceQTSG::registerSource() {
	if(!checkQTThreadInit())
		return;
	if(!grabbing) { // we're about to be used!
		// lights...camera...
		OSErr err = SGPrepare(sg, false, true);
		if(err!=noErr) {
			cerr << "CameraSource: SGPrepare returned error " << err << endl;
			return;
		}
		
		/*
		 // make sure the timebase used by the video channel is being driven by
		 // the sound clock if there is a sound channel, this has to be done
		 // after calling SGPrepare - see Q&A 1314
		 if (NULL != sgchanSound) {
		 TimeBase soundTimeBase = NULL, sgTimeBase = NULL;
		 err = SGGetTimeBase(this->seqGrab, &sgTimeBase);
		 if(noErr == err)
		 err = SGGetChannelTimeBase(sgchanSound, &soundTimeBase);
		 if (noErr == err && NULL != soundTimeBase)
		 SetTimeBaseMasterClock(sgTimeBase, (Component)GetTimeBaseMasterClock(soundTimeBase), NULL);
		 }
		 */
		
		// ...action
		err = SGStartRecord(sg);
		if(err!=noErr) {
			cerr << "CameraSource: SGStartRecord returned error " << err << endl;
			err = SGRelease(sg); // undo SGPrepare()
			if(err!=noErr)
				cerr << "CameraSource: SGRelease returned error during recovery " << err << endl;
			return;
		}
		
		grabbing=true;
	}
}

void CameraSourceQTSG::deregisterSource() {
	if(!checkQTThreadInit())
		return;
	if(grabbing) {
		OSErr err = SGStop(sg); // undo SGStartPreview or SGStartRecord
		if(err!=noErr)
			cerr << "CameraSource: SGStop returned error " << err << endl;
		err = SGRelease(sg); // undo SGPrepare()
		if(err!=noErr)
			cerr << "CameraSource: SGRelease returned error " << err << endl;
		grabbing=false;
	}
}

void CameraSourceQTSG::doUnfreeze() {
	poller.start();
}

void CameraSourceQTSG::doFreeze() { 
	if(poller.isStarted())
		poller.stop().join();
}

/*void CameraSourceQTSG::setDataSourceFramerate(float fps) {
	DataSource::setDataSourceFramerate(fps);
	float s = std::max(getTimeScale(),.1f);
	ComponentResult err = SGSetFrameRate(sgChan, FloatToFixed(framerate*s));
	if(err!=noErr)
		std::cerr << "CameraSource::setDataSourceFramerate("<<fps<<") had error calling SGSetFrameRate " << err << endl;
}*/


bool CameraSourceQTSG::advance() {
	//cout << "getData at " << get_time() << " grabbing " << grabbing << endl;
	
	if(!checkQTThreadInit() || !grabbing)
		return false;
	
	unsigned int prev=frame;
	burnIn = frozen;
	
	OSErr err = SGIdle(sg);
	if(frozen) {
		burnIn = false;
		// we just cleared the backlog, but didn't get a current frame
		for(unsigned int i=0; i<100 && err==noErr && prev==frame; ++i) {
			usleep(10*1000); // wait for a new frame
			err = SGIdle(sg);
		}
	}
	if (err!=noErr && err!=callbackerr) {
		// some error specific to SGIdle occurred - any errors returned from the
		// data proc will also show up here and we don't want to write over them
		
		// in QT 4 you would always encounter a cDepthErr error after a user drags
		// the window, this failure condition has been greatly relaxed in QT 5
		// it may still occur but should only apply to vDigs that really control
		// the screen
		
		// you don't always know where these errors originate from, some may come
		// from the VDig...
		
		//DisplayError(pMungData->pWindow, "SGIdle", err);
		cerr << "CameraSource: SGIdle error " << err << " occurred, resetting camera!" << endl;
		
		// ...to fix this we simply call SGStop and SGStartRecord again
		// calling stop allows the SG to release and re-prepare for grabbing
		// hopefully fixing any problems, this is obviously a very relaxed
		// approach
		err = SGStop(sg); // undo SGStartPreview or SGStartRecord
		if(err!=noErr)
			cerr << "CameraSource: SGStop returned error during recovery " << err << endl;
		err = SGStartRecord(sg);
		if(err!=noErr) {
			cerr << "CameraSource: SGStartRecord returned error during recovery " << err << endl;
			grabbing=false;
			err = SGRelease(sg); // undo SGPrepare()
			if(err!=noErr)
				cerr << "CameraSource: SGRelease returned error during recovery " << err << endl;
		}
	}
	
	return (prev!=frame);
}
	
void CameraSourceQTSG::dumpLiteral(OSType t) {
	union {
		OSType v;
		char s[4];
	} x;
	x.v=t;
	cout << x.s[3] << x.s[2] << x.s[1] << x.s[0];
}


/* 
* Purpose:   used to allow us to figure out how many frames are queued by the vDig
*
* Notes:     the UInt8 *queuedFrameCount replaces Boolean *done.  0 (==false) still means no frames, and 1 (==true) one,
*            but if more than one are available, the number should be returned here - The value 2 previously meant more than
*            one frame, so some VDIGs may return 2 even if more than 2 are available, and some will still return 1 as they are
*            using the original definition.
*/
pascal ComponentResult CameraSourceQTSG::compressCompleteBottleProc(SGChannel c, UInt8 *queuedFrameCount, SGCompressInfo *ci, TimeRecord *t, long refCon)
{
	OSErr err;
	CameraSourceQTSG* cam = (CameraSourceQTSG*)refCon;
	if (NULL == cam) return -1;
	
	// call the original proc; you must do this
	err = SGGrabCompressComplete(c, queuedFrameCount, ci, t);
	
	// save the queued frame count so we have it
	cam->queuedFrames = *queuedFrameCount;
	/*if(cam->queuedFrames>0)
		cout << "compressCompleteBottleProc " << cam->queuedFrames << endl;*/
	
	return err;
}


/*****************************************************
* Purpose:   sequence grabber data procedure - this is where the work is done
*
* Notes:

the sequence grabber calls the data function whenever
any of the grabber's channels write digitized data to the destination movie file.

NOTE: We really mean any, if you have an audio and video channel then the DataProc will
be called for either channel whenever data has been captured. Be sure to check which
channel is being passed in. In this example we never create an audio channel so we know
we're always dealing with video.

This data function does two things, it first decompresses captured video
data into an offscreen GWorld, draws some status information onto the frame then
transfers the frame to an onscreen window.

For more information refer to Inside Macintosh: QuickTime Components, page 5-120
c - the channel component that is writing the digitized data.
p - a pointer to the digitized data.
len - the number of bytes of digitized data.
offset - a pointer to a field that may specify where you are to write the digitized data,
and that is to receive a value indicating where you wrote the data.
chRefCon - per channel reference constant specified using SGSetChannelRefCon.
time	- the starting time of the data, in the channel's time scale.
writeType - the type of write operation being performed.
seqGrabWriteAppend - Append new data.
seqGrabWriteReserve - Do not write data. Instead, reserve space for the amount of data
specified in the len parameter.
seqGrabWriteFill - Write data into the location specified by offset. Used to fill the space
previously reserved with seqGrabWriteReserve. The Sequence Grabber may
call the DataProc several times to fill a single reserved location.
refCon - the reference constant you specified when you assigned your data function to the sequence grabber.
*/
pascal OSErr CameraSourceQTSG::grabDataProc(SGChannel c, Ptr p, long len, long *offset, long chRefCon, TimeValue time, short writeType, long refCon)
{
#pragma unused(offset,chRefCon,writeType)
	//cout << "MungGrabDataProc" << endl;
	
	CameraSourceQTSG* cam = (CameraSourceQTSG*)refCon; // might want to use chRefCon instead?
	if (NULL == cam) {
		cerr << "CameraSource::grabDataProc called without a context" << endl;
		return -1;
	}
	
	// we only care about the video	
	if (c != cam->sgChan) {
		cerr << "CameraSource::grabDataProc called for something other than our channel" << endl;
		return noErr; //not an error as far as OS is concerned...
	}
	
	if(cam->burnIn || (cam->frozen && cam->queuedFrames>1))
		return noErr; // we want a current frame when being manually advanced, skip backlog
	
	++cam->skipped; // unless we make it through all the way, in which case we'll subtract this off and add to frame instead
	
	ComponentResult err=noErr;
	TimeValue frameTimeDelta=0;
	ImageDescriptionHandle imageDesc = NULL;
	RCRegion * region = NULL;
	try {
		// apparently can't do this before we get a frame (I tried, get seqGrabInfoNotAvailable (-9407) on SGGetChannelTimeScale
		if(cam->chanTimeScale==0) {
			Fixed framesPerSecond;
			long  milliSecPerFrameIgnore, bytesPerSecondIgnore;
			
			// first time here so get the time scale & timebase
			err = SGGetChannelTimeScale(cam->sgChan, &cam->chanTimeScale);
			if(err!=noErr) throw make_pair(err,"SGGetChannelTimeScale");
			
			err = SGGetTimeBase(cam->sg, &cam->chanTimeBase);
			if(err!=noErr) throw make_pair(err,"SGGetTimeBase");
			
			err = VDGetDataRate(SGGetVideoDigitizerComponent(cam->sgChan), &milliSecPerFrameIgnore, &framesPerSecond, &bytesPerSecondIgnore);
			if(err!=noErr) throw make_pair(err,"VDGetDataRate");
			
			cam->duration = 1.f / FixedToFloat(framesPerSecond);
			cam->poller.resetPeriod(cam->duration,false);

			//SGGetFrameRate(cam->sgChan, &framesPerSecond); // returns 0 fps
			//std::cout << "SGGetFrameRate says " << (1.f / FixedToFloat(framesPerSecond)) << std::endl;
		}
		
		// retrieve a channel's current sample description, the channel returns a
		// sample description that is appropriate to the type of data being captured
		imageDesc = (ImageDescriptionHandle)NewHandle(0);
		err = SGGetChannelSampleDescription(c, (Handle)imageDesc);
		if(err!=noErr) throw "SGGetChannelSampleDescription";
		string formatName = p2c((**imageDesc).name);
		//cout << formatName << " TYPE IS "; dumpLiteral((**imageDesc).cType); cout << " " << (**imageDesc).width << "x" << (**imageDesc).height << " => " << (**imageDesc).dataSize << " @ " << (**imageDesc).depth << endl;

		if((**imageDesc).cType==k422YpCbCr8CodecType) { // aka 2vuy
		
			region = cam->imgFrom2vuy((unsigned char*)p, (**imageDesc).width, (**imageDesc).height, (**imageDesc).depth, (**imageDesc).dataSize);
			
		} else if((**imageDesc).cType==kComponentVideoCodecType) { // aka yuv2 aka yuvu
			
			region = cam->imgFromyuv2((unsigned char*)p, (**imageDesc).width, (**imageDesc).height, (**imageDesc).depth, (**imageDesc).dataSize);
			
		} else {
			// use a OS provided decompression sequence to put it in the gworld
			if (cam->drawSeq == 0) {
				
				// set up decompression sequence	
				Rect				   sourceRect = { 0, 0, 0, 0 };
				MatrixRecord		   scaleMatrix;	
				CodecFlags			   cFlags = codecNormalQuality;
				
				if(cam->gworld==NULL) {
					Rect  srcBounds;
					err = SGGetChannelBounds(cam->sgChan, &srcBounds);
					if(err!=noErr) throw "SGGetSrcVideoBounds";
					unsigned int width = srcBounds.right-srcBounds.left;
					unsigned int height = srcBounds.bottom-srcBounds.top;
					cam->gworldBuf = (char*)malloc(width*height*2); // NewPtr(width*height*4);
					// once upon a time I could've sworn I had to use k32ARGBPixelFormat and do
					// the color space conversion manually, but apparently this is working... (yay! very fast...)
					err = QTNewGWorldFromPtr(&cam->gworld, k2vuyPixelFormat, &srcBounds, NULL, NULL, 0, cam->gworldBuf, width*2);
					if(err!=noErr) throw "QTNewGWorldFromPtr";
				}
					
				// make a scaling matrix for the sequence
				sourceRect.right = (**imageDesc).width;
				sourceRect.bottom = (**imageDesc).height;
				RectMatrix(&scaleMatrix, &sourceRect, &(*GetPortPixMap(cam->gworld))->bounds);
				
				// begin the process of decompressing a sequence of frames
				// this is a set-up call and is only called once for the sequence - the ICM will interrogate different codecs
				// and construct a suitable decompression chain, as this is a time consuming process we don't want to do this
				// once per frame (eg. by using DecompressImage)
				// for more information see Ice Floe #8 http://developer.apple.com/quicktime/icefloe/dispatch008.html
				// the destination is specified as the GWorld
				err = DecompressSequenceBeginS(&cam->drawSeq,					// pointer to field to receive unique ID for sequence
					imageDesc,							// handle to image description structure
					p,									// points to the compressed image data
					len,                             		// size of the data buffer
					cam->gworld,	// port for the DESTINATION image
					NULL,									// graphics device handle, if port is set, set to NULL
					NULL,									// decompress the entire source image - no source extraction
					&scaleMatrix,							// transformation matrix
					srcCopy,								// transfer mode specifier
					(RgnHandle)NULL,						// clipping region in dest. coordinate system to use as a mask
					0,									// flags
					cFlags, 								// accuracy in decompression
					bestSpeedCodec);						// compressor identifier or special identifiers ie. bestSpeedCodec
				
				if(err!=noErr) throw "DecompressSequenceBeginS";
			}
			
			// get the TimeBase time and figure out the delta between that time and this frame time
			TimeValue timeBaseTime, timeBaseDelta;
			timeBaseTime = GetTimeBaseTime(cam->chanTimeBase, cam->chanTimeScale, NULL);
			timeBaseDelta = timeBaseTime - time;
			frameTimeDelta = time - cam->lastTime;
			
			if (timeBaseDelta < 0) return err; // probably don't need this
			
			// if we have more than one queued frame and our capture rate drops below 10 frames, skip the frame to try and catch up
			if ((cam->queuedFrames > 1) &&  ((cam->chanTimeScale / frameTimeDelta) < 10) && (cam->skipped < 15)) {
				// do nothing, skipping frame
			} else {
				CodecFlags ignore;
				
				// decompress a frame into the window - can queue a frame for async decompression when passed in a completion proc
				err = DecompressSequenceFrameS(cam->drawSeq,	// sequence ID returned by DecompressSequenceBegin
					 p,					// pointer to compressed image data
					 len,					// size of the buffer
					 0,					// in flags
					 &ignore,				// out flags
					 NULL);				// async completion proc
				
				if(err!=noErr) throw "DecompressSequenceFrameS";
				
				cam->skipped = 1;
				cam->lastTime = time;
				
				{
					PixMapPtr pm=*GetPortPixMap(cam->gworld);
					unsigned int width = pm->bounds.right - pm->bounds.left;
					unsigned int height = pm->bounds.bottom - pm->bounds.top;
					unsigned char * s = (unsigned char *)GetPixBaseAddr(&pm);
					region = cam->imgFrom2vuy(s,width,height,16,width*height*2);
				}
			} 
		}
	} catch(const char* call) {
		cerr << "CameraSource: " << call << " returned error " << err << endl;
		if(imageDesc!=NULL)
			DisposeHandle((Handle)imageDesc);
		return cam->callbackerr=err;
	}
	if(imageDesc!=NULL)
		DisposeHandle((Handle)imageDesc);
		
	/*
	// status information
	float	fps, averagefps;
	UInt8   minutes, seconds, frames;
	
	fps = (float)cam->chanTimeScale / (float)frameTimeDelta;
	averagefps = ((float)cam->frame * (float)cam->chanTimeScale) / (float)time;
	minutes = (time / cam->chanTimeScale) / 60;
	seconds = (time / cam->chanTimeScale) % 60;
	frames = (time % cam->chanTimeScale) / frameTimeDelta; //cam->duration;
	printf("t: %ld, %02d:%02d.%02d, fps:%5.1f av:%5.1f\n", time, minutes, seconds, frames, fps, averagefps);
	*/
	
	ASSERTRETVAL(region!=NULL,"region still NULL at end of CameraSource::grabDataProc",-1)
	
	// made it!  increment frame and decrement our pessimistic skip
	++cam->frame;
	--cam->skipped;
	cam->setImage(region);
	
	return err;
}


/*! 2vuy is an interleaved format, where the u and v channels are downsampled by half.
*  The first column of pixels has u values, the second column is v's.
*  For example, the first 6 pixels of a row are stored as: uy, vy, uy, vy, uy, vy, ...  */
RCRegion* CameraSourceQTSG::imgFrom2vuy(const unsigned char * s, short srcWidth, short srcHeight, short depth, long dataSize) {
#pragma unused(depth,dataSize)
	const unsigned int width=srcWidth/2;
	const unsigned int height=srcHeight/2;
	const unsigned int components=3;
	ssize_t reqSize = sizeof(ImageHeader) + width * height * components;
	RCRegion * region = getUnusedRegion(reqSize, 0);
	unsigned char * buf = reinterpret_cast<unsigned char*>(region->Base());
	new (region->Base()) ImageHeader(0, layer, width, height, components, frame, get_time(), nextName());
	
	const unsigned int srcStride=srcWidth*2;
	unsigned char * dst = buf + sizeof(ImageHeader);
	unsigned char * const dstEnd=dst+width*height*components;
	while(dst!=dstEnd) {
		unsigned char * const rowEnd=dst+width*components;
		while(dst!=rowEnd) {
			unsigned int y,u,v;
			u=*s;
			u+=*(s+srcStride);
			++s;
			
			y=*s;
			y+=*(s+srcStride);
			++s;
			
			v=*s;
			v+=*(s+srcStride);
			++s;
			
			y+=*s;
			y+=*(s+srcStride);
			++s;
			
			*dst++ = y/4;
			*dst++ = u/2;
			*dst++ = v/2;
		}
		s+=srcStride;
	}
	ASSERTRETVAL(dst-buf==reqSize,"CameraSource bad imgFrom2vuy " << reqSize << " vs " << (dst-buf),NULL);
	return region;
}

/*! Similar to 2vuy, except:
 *  - byte swapped (yu, yv instead of uy, vy)
 *  - u and v channels are signed instead of unsigned
 *  - channels use the full byte range, whereas apparently 2vuy only uses 16-235 for the y and 16-240 for the u and v
 *    (aka 601/YCbCr standard)
 */
RCRegion* CameraSourceQTSG::imgFromyuv2(const unsigned char * s, short srcWidth, short srcHeight, short depth, long dataSize) {
#pragma unused(depth,dataSize)
	const unsigned int width=srcWidth/2;
	const unsigned int height=srcHeight/2;
	const unsigned int components=3;
	ssize_t reqSize = sizeof(ImageHeader) + width * height * components;
	RCRegion * region = getUnusedRegion(reqSize, 0);
	unsigned char * buf = reinterpret_cast<unsigned char*>(region->Base());
	new (region->Base()) ImageHeader(0, layer, width, height, components, frame, get_time(), nextName());
	
	const unsigned int srcStride=srcWidth*2;
	unsigned char * dst = buf + sizeof(ImageHeader);
	unsigned char * const dstEnd=dst+width*height*components;
	while(dst!=dstEnd) {
		unsigned char * const rowEnd=dst+width*components;
		while(dst!=rowEnd) {
			unsigned int y;
			int u,v;
			y=*s;
			y+=*(s+srcStride);
			++s;
			
			u=(char)*s;
			u+=(char)*(s+srcStride);
			++s;
			
			y+=*s;
			y+=*(s+srcStride);
			++s;
			
			v=(char)*s;
			v+=(char)*(s+srcStride);
			++s;
						
			*dst++ = (y*219/255)/4 + 16;
			*dst++ = (u*224/255)/2 + 128;
			*dst++ = (v*224/255)/2 + 128;
		}
		s+=srcStride;
	}
	ASSERTRETVAL(dst-buf==reqSize,"CameraSourceOSX bad imgFromyuv2 " << reqSize << " vs " << (dst-buf),NULL);
	return region;
}

#endif // pre-10.6
#endif // Apple 32 bit

/*! @file
 * @brief Describes CameraSourceQTSG, which interfaces with a specific camera through QuickTime and the Sequence Grabber, which is deprecated.  See the alternative CameraSourceQTKit implementation.
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
