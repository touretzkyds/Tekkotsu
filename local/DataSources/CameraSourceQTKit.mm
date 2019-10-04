#if defined(__APPLE__)

#include <AvailabilityMacros.h>
#ifdef MAC_OS_X_VERSION_10_6

#include "CameraSourceQTKit.h"
#include "Shared/debuget.h"
#include <cmath>

using namespace std; 

INSTANTIATE_NAMEDENUMERATION_STATICS(CameraSourceQTKit::PixelFormat_t);

void CameraSourceQTKit::init() {
	format.setPreferredNameForVal("",TYPE_UNKNOWN);
	format.setPreferredNameForVal("yuvs",TYPE_YUVS);
	format.setPreferredNameForVal("2vuy",TYPE_2VUY);
	format.setPreferredNameForVal("gray",TYPE_GRAY);
	format.addNameForVal("grayscale",TYPE_GRAY);
	addEntry("Parallel",parallel,"If true, will attempt to use Apple's Grand Central Dispatch to do block processing.\n"
			 "This parallelizes image processing, may slightly increase total CPU usage but should reduce per-frame wall time.");
	addEntry("HighRes",highRes,"If true, upsamples color channels horizontally to match Y channel, otherwise downsamples\n"
			 "everything to common resolution (y/4, u,v/2).  Set to true to use full resolution of\n"
			 "camera (either as the “full” layer or if you are accessing the “double” layer),\n"
			 "set to false if you are using half-resolution as the resolution of the “full” layer");
	addEntry("Layer",layer,"Controls the resolution layer at which the image should be processed.\n"
		"0 indicates “automatic” mode (picks layer closest to image's resolution), positive numbers indicate the resolution layer directly.\n"
		"Negative values are relative to the number of layers marked available by the vision setup, so that typically -1 would correspond to the “double” layer, and -2 would correspond to the “full” layer.");
	addEntry("Format",format,"If non-empty string, requests the camera hardware provide images in the specified format (a four character code aka FourCC)\n"+format.getDescription());
	setLoadSavePolicy(FIXED,SYNC);
	delegate = [[CameraSourceQTKitDelegate alloc] initWithTarget:this];
	[device retain];
}

CameraSourceQTKit::~CameraSourceQTKit() {
	ASSERT(session==NULL,"Still have session instance in CameraSourceQTKit destructor?  Missed deregisterSource call?");
	NSAutoreleasePool	 *autoreleasepool = [[NSAutoreleasePool alloc] init];
	[device release];
	[delegate release];
	[autoreleasepool release];
}

void CameraSourceQTKit::registerSource() {
	ASSERTRET(session==NULL,"re-registration?");
	NSAutoreleasePool	 *autoreleasepool = [[NSAutoreleasePool alloc] init];
	
	NSError* error=NULL;
	QTCaptureDeviceInput* input=NULL;
	QTCaptureVideoPreviewOutput* output=NULL;
	try {
		if(![device open:&error])
			throw std::runtime_error("Could not open camera device " + name);
		
		session = [[QTCaptureSession alloc] init];
		input = [[QTCaptureDeviceInput alloc] initWithDevice:device];
		if(![session addInput:input error:&error])
			throw std::runtime_error("Could not add camera device " + name + " to capture session");
		
		output = [[QTCaptureVideoPreviewOutput alloc] init];
		if(![session addOutput:output error:&error])
			throw std::runtime_error("Could not add preview output to capture session for "+name);
		
		// FYI iSight 'native' is kComponentVideoUnsigned aka kYUVSPixelFormat aka  'yuvs'
		// We want kCVPixelFormatType_444YpCbCr8 aka k444YpCbCr8CodecType aka 'v308' for equal color resolution
		//[output setPixelBufferAttributes:[NSDictionary dictionaryWithObject:[NSNumber numberWithUnsignedInt:k422YpCbCr8CodecType] forKey:(NSString*)kCVPixelBufferPixelFormatTypeKey]];
		/*[output setPixelBufferAttributes:[NSDictionary dictionaryWithObjectsAndKeys:
			  [NSNumber numberWithUnsignedInt:320], (id)kCVPixelBufferWidthKey,
			  [NSNumber numberWithUnsignedInt:240], (id)kCVPixelBufferHeightKey,
			  [NSNumber numberWithUnsignedInt:kCVPixelFormatType_422YpCbCr8], (id)kCVPixelBufferPixelFormatTypeKey,
			  nil]]; */
		
		/* Conversions supported by iSight, 10.6.3:
		 kComponentVideoUnsigned 'yuvs'
		 kCVPixelFormatType_24RGB 0x18
		 kCVPixelFormatType_32ARGB 0x20
		 kCVPixelFormatType_8IndexedGray_WhiteIsZero 0x28
		 kCVPixelFormatType_32BGRA 'BGRA'
		 kCVPixelFormatType_422YpCbCr8 '2vuy'
		 kCVPixelFormatType_420YpCbCr8Planar 'y420'
		 */
		
		// Haven't found a way to detect or predict camera support, let the user configure it
		if(format!=TYPE_UNKNOWN)
			[output setPixelBufferAttributes:[NSDictionary dictionaryWithObject:[NSNumber numberWithUnsignedInt:format] forKey:(NSString*)kCVPixelBufferPixelFormatTypeKey]];
		
		[output setDelegate:delegate];

	} catch(const std::runtime_error& ex) {
		std::cerr << ex.what() << std::endl;
		if(error!=NULL) {
			std::cerr << "Description: " << [[error localizedDescription] UTF8String] << std::endl;
			std::cerr << "Reason: " << [[error localizedFailureReason] UTF8String] << std::endl;
		}
		if(session!=NULL) {
			[session release];
			session=NULL;
		}
		if([device isOpen])
			[device close];
	}
	[input release];
	[output release];
	[autoreleasepool release];
}

void CameraSourceQTKit::deregisterSource() {
	NSAutoreleasePool	 *autoreleasepool = [[NSAutoreleasePool alloc] init];
	if(session!=NULL) {
		if([session isRunning])
			[session stopRunning];
		[session release];
		session=NULL;
	}
	if([device isOpen])
		[device close];
	[autoreleasepool release];
}

bool CameraSourceQTKit::advance() {
	NSAutoreleasePool	 *autoreleasepool = [[NSAutoreleasePool alloc] init];
	if([session isRunning]) {
		std::cerr << "Advancing " << name << ", but capture session already running" << std::endl;
		[autoreleasepool release];
		return false;
	}
	MarkScope lock(frameLock);
	oneFrame=true;
	[session startRunning];
	if(![session isRunning]) {
		std::cerr << "Advancing " << name << ", but capture session already running" << std::endl;
		[autoreleasepool release];
		return false;
	}
	[autoreleasepool release];
	frameCond.wait(frameLock); // we will receive the frame on another thread, wait for the signal it has been processed
	return true;
}

void CameraSourceQTKit::doFreeze() {
	NSAutoreleasePool	 *autoreleasepool = [[NSAutoreleasePool alloc] init];
	if([session isRunning])
		[session stopRunning];
	[autoreleasepool release];
}

void CameraSourceQTKit::doUnfreeze() {
	NSAutoreleasePool	 *autoreleasepool = [[NSAutoreleasePool alloc] init];
	if(![session isRunning])
		[session startRunning];
	[autoreleasepool release];
}

void CameraSourceQTKit::processImage(CVImageBufferRef videoFrame, QTSampleBuffer* sampleBuffer) {
	if(![session isRunning]) // indicates getting a buffered callback even after we've stopped the capture session, skip it
		return;
	
	//[sampleBuffer incrementSampleUseCount];
	
	float curTime = get_time()/1000.f;
	if(std::abs(curTime - (lastTime+duration)) >= duration)
		lastTime = curTime;
	else
		lastTime += duration;
	
	NSTimeInterval dur;
	QTGetTimeInterval([sampleBuffer duration],&dur);
	duration = static_cast<float>(dur);
	
	CGSize dim = CVImageBufferGetEncodedSize(videoFrame);
	unsigned int width = static_cast<unsigned int>(dim.width);
	unsigned int height = static_cast<unsigned int>(dim.height);
	
	unsigned int cc = [[sampleBuffer formatDescription] formatType];
	switch(cc) {
		case kComponentVideoUnsigned: // aka 'yuvs'
			if(highRes) {
				process_yuvs_U(reinterpret_cast<const unsigned char*>([sampleBuffer bytesForAllSamples]),width,height);
			} else {
				process_yuvs_D(reinterpret_cast<const unsigned char*>([sampleBuffer bytesForAllSamples]),width,height);
			}
			break;
		case kCVPixelFormatType_422YpCbCr8: // aka '2vuy'
			if(highRes) {
				process_2vuy_U(reinterpret_cast<const unsigned char*>([sampleBuffer bytesForAllSamples]),width,height);
			} else {
				process_2vuy_D(reinterpret_cast<const unsigned char*>([sampleBuffer bytesForAllSamples]),width,height);
			}
			break;
		case kCVPixelFormatType_8IndexedGray_WhiteIsZero: // aka 0x28 (not really a fourCC)
			process_grayscale_zerowhite(reinterpret_cast<const unsigned char*>([sampleBuffer bytesForAllSamples]),width,height);
			break;
		default: {
			static unsigned int gaveWarning=0;
			if(gaveWarning!=cc) {
				gaveWarning=cc;
				std::cerr << "ERROR: CameraSourceQTKit doesn't know how to convert " << CC2Str(cc) << " pixel format" << std::endl;
			}
		} break;
	}
	
	if(oneFrame) { // indicates this was triggered by 'advance', we should stop capture now that we've gotten a frame
		MarkScope lock(frameLock);
		oneFrame=false;
		[session stopRunning];
		frameCond.broadcast();
	}
	
	//[sampleBuffer decrementSampleUseCount];
}

std::string CameraSourceQTKit::CC2Str(unsigned int x) {
	std::string ans;
	if(('1234' & 0xFF) == 1) {
		ans.append(1,char((x>>0) & 0xFF));
		ans.append(1,char((x>>8) & 0xFF));
		ans.append(1,char((x>>16) & 0xFF));
		ans.append(1,char((x>>24) & 0xFF));
	} else {
		ans.append(1,char((x>>24) & 0xFF));
		ans.append(1,char((x>>16) & 0xFF));
		ans.append(1,char((x>>8) & 0xFF));
		ans.append(1,char((x>>0) & 0xFF));
	}
	return ans;
}

unsigned int CameraSourceQTKit::Str2CC(const std::string& x) {
	if(('1234' & 0xFF) == 1) {
		return (x[0]<<0) | (x[1]<<8) | (x[2]<<16) | (x[3]<<24);
	} else {
		return (x[0]<<24) | (x[1]<<16) | (x[2]<<8) | (x[3]<<0);
	}
}


// When parallelizing with GCD, each dispatch should have a reasonable amount of work to do
// So group several rows together for each block to reduce overhead.
static const unsigned int ROWS_PER_BLOCK=4;

// Data needed for each block to be processed
struct BlockContext {
	const unsigned char* src; //!< start of source image
	const unsigned int srcStride; //!< length of each row (in bytes)
	unsigned char* dst; //!< start of destination image
};



/******* yuvs processing *******/

static void process_yuvs_UBlock(void* context, size_t i) {
	const BlockContext* const ctxt = reinterpret_cast<BlockContext*>(context);
	const unsigned int pixels = ctxt->srcStride * ROWS_PER_BLOCK;
	const unsigned char* src = ctxt->src+pixels*i;
	const unsigned char* const srcEnd = src+pixels;
	unsigned char* dst = ctxt->dst+pixels/2*3*i;
	while(src!=srcEnd) {
		// y₁ u y₂ v → y₁ u v y₂ u v
		dst[0] = *src++;
		dst[1] = dst[4] = *src++;
		dst[3] = *src++;
		dst[2] = dst[5] = *src++;
		dst+=6;
	}
}

void CameraSourceQTKit::process_yuvs_U(const unsigned char * s, unsigned int width, unsigned int height) {
	//TimeET p;
	const unsigned int components=3;
	ssize_t reqSize = sizeof(ImageHeader) + width * height * components;
	RCRegion * region = getUnusedRegion(reqSize, 0);
	unsigned char * buf = reinterpret_cast<unsigned char*>(region->Base());
	new (region->Base()) ImageHeader(0, layer, width, height, components, ++frame, get_time(), nextName());
	
	unsigned char * dst = buf + sizeof(ImageHeader);
	if(parallel) {
		BlockContext ctxt = { s, width*2, dst };
		dispatch_queue_t q_default = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0);
		dispatch_apply_f(height/ROWS_PER_BLOCK, q_default, &ctxt, process_yuvs_UBlock);
	} else {
		const unsigned char * const dstEnd=dst+width*height*components;
		while(dst!=dstEnd) {
			// y₁ u y₂ v → y₁ u v y₂ u v
			dst[0] = *s++;
			dst[1] = dst[4] = *s++;
			dst[3] = *s++;
			dst[2] = dst[5] = *s++;
			dst+=6;
		}
		ASSERTRET(dst-buf==reqSize,"CameraSource bad imgFromyuvs " << reqSize << " vs " << (dst-buf));
	}
	//std::cout << p.Age() << std::endl;
	setImage(region);
}

static void process_yuvs_DBlock(void* context, size_t i) {
	const BlockContext* const ctxt = reinterpret_cast<BlockContext*>(context);
	const unsigned int srcStride = ctxt->srcStride;
	const unsigned char* src = ctxt->src+srcStride*2*i*ROWS_PER_BLOCK;
	unsigned char* dst = ctxt->dst+srcStride/4*3*i*ROWS_PER_BLOCK;
	unsigned int y,u,v;
	for(unsigned int r=0; r<ROWS_PER_BLOCK; ++r) {
		const unsigned char* const srcRowEnd = src+srcStride;
		while(src!=srcRowEnd) {
			y=*src;
			y+=*(src+srcStride);
			
			u=*++src;
			u+=*(src+srcStride);
			
			y+=*++src;
			y+=*(src+srcStride);
			
			v=*++src;
			v+=*(src+srcStride);
			
			*dst++ = y/4;
			*dst++ = u/2;
			*dst++ = v/2;
			
			++src;
		}
		src+=srcStride;
	}
}

void CameraSourceQTKit::process_yuvs_D(const unsigned char * s, unsigned int srcWidth, unsigned int srcHeight) {
	//TimeET p;
	const unsigned int width=srcWidth/2;
	const unsigned int height=srcHeight/2;
	const unsigned int components=3;
	ssize_t reqSize = sizeof(ImageHeader) + width * height * components;
	RCRegion * region = getUnusedRegion(reqSize, 0);
	unsigned char * buf = reinterpret_cast<unsigned char*>(region->Base());
	new (region->Base()) ImageHeader(0, layer, width, height, components, ++frame, get_time(), nextName());
	
	BlockContext ctxt = { s, srcWidth*2, buf + sizeof(ImageHeader) };
	if(parallel) {
		dispatch_queue_t q_default = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0);
		dispatch_apply_f(height/ROWS_PER_BLOCK, q_default, &ctxt, process_yuvs_DBlock);
	} else {
		for(size_t i=0; i<height/ROWS_PER_BLOCK; ++i)
			process_yuvs_DBlock(&ctxt,i);
	}
	//std::cout << p.Age() << std::endl;
	setImage(region);
}



/******* 2vuy processing *******/

static void process_2vuy_UBlock(void* context, size_t i) {
	const BlockContext* const ctxt = reinterpret_cast<BlockContext*>(context);
	const unsigned int pixels = ctxt->srcStride * ROWS_PER_BLOCK;
	const unsigned char* src = ctxt->src+pixels*i;
	const unsigned char* const srcEnd = src+pixels;
	unsigned char* dst = ctxt->dst+pixels/2*3*i;
	while(src!=srcEnd) {
		// y₁ u y₂ v → y₁ u v y₂ u v
		dst[1] = dst[4] = *src++;
		dst[0] = *src++;
		dst[2] = dst[5] = *src++;
		dst[3] = *src++;
		dst+=6;
	}
}

void CameraSourceQTKit::process_2vuy_U(const unsigned char * s, unsigned int width, unsigned int height) {
	//TimeET p;
	const unsigned int components=3;
	ssize_t reqSize = sizeof(ImageHeader) + width * height * components;
	RCRegion * region = getUnusedRegion(reqSize, 0);
	unsigned char * buf = reinterpret_cast<unsigned char*>(region->Base());
	new (region->Base()) ImageHeader(0, layer, width, height, components, ++frame, get_time(), nextName());
	
	unsigned char * dst = buf + sizeof(ImageHeader);
	if(parallel) {
		BlockContext ctxt = { s, width*2, dst };
		dispatch_queue_t q_default = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0);
		dispatch_apply_f(height/ROWS_PER_BLOCK, q_default, &ctxt, process_2vuy_UBlock);
	} else {
		const unsigned char * const dstEnd=dst+width*height*components;
		while(dst!=dstEnd) {
			// y₁ u y₂ v → y₁ u v y₂ u v
			dst[1] = dst[4] = *s++;
			dst[0] = *s++;
			dst[2] = dst[5] = *s++;
			dst[3] = *s++;
			dst+=6;
		}
		ASSERTRET(dst-buf==reqSize,"CameraSource bad imgFromyuvs " << reqSize << " vs " << (dst-buf));
	}
	//std::cout << p.Age() << std::endl;
	setImage(region);
}

static void process_2vuy_DBlock(void* context, size_t i) {
	const BlockContext* const ctxt = reinterpret_cast<BlockContext*>(context);
	const unsigned int srcStride = ctxt->srcStride;
	const unsigned char* src = ctxt->src+srcStride*2*i*ROWS_PER_BLOCK;
	unsigned char* dst = ctxt->dst+srcStride/4*3*i*ROWS_PER_BLOCK;
	unsigned int y,u,v;
	for(unsigned int r=0; r<ROWS_PER_BLOCK; ++r) {
		const unsigned char* const srcRowEnd = src+srcStride;
		while(src!=srcRowEnd) {
			u=*src;
			u+=*(src+srcStride);
			
			y=*++src;
			y+=*(src+srcStride);
			
			v=*++src;
			v+=*(src+srcStride);
			
			y+=*++src;
			y+=*(src+srcStride);
			
			*dst++ = y/4;
			*dst++ = u/2;
			*dst++ = v/2;
			
			++src;
		}
		src+=srcStride;
	}
}

void CameraSourceQTKit::process_2vuy_D(const unsigned char * s, unsigned int srcWidth, unsigned int srcHeight) {
	//TimeET p;
	const unsigned int width=srcWidth/2;
	const unsigned int height=srcHeight/2;
	const unsigned int components=3;
	ssize_t reqSize = sizeof(ImageHeader) + width * height * components;
	RCRegion * region = getUnusedRegion(reqSize, 0);
	unsigned char * buf = reinterpret_cast<unsigned char*>(region->Base());
	new (region->Base()) ImageHeader(0, layer, width, height, components, ++frame, get_time(), nextName());
	
	BlockContext ctxt = { s, srcWidth*2, buf + sizeof(ImageHeader) };
	if(parallel) {
		dispatch_queue_t q_default = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0);
		dispatch_apply_f(height/ROWS_PER_BLOCK, q_default, &ctxt, process_2vuy_DBlock);
	} else {
		for(size_t i=0; i<height/ROWS_PER_BLOCK; ++i)
			process_2vuy_DBlock(&ctxt,i);
	}
	//std::cout << p.Age() << std::endl;
	setImage(region);
}


/******* Grayscale processing *******/

static void process_grayscale_zerowhite_Block(void* context, size_t i) {
	const BlockContext* const ctxt = reinterpret_cast<BlockContext*>(context);
	const unsigned int pixels = ctxt->srcStride * ROWS_PER_BLOCK;
	const unsigned char* src = ctxt->src+pixels*i;
	const unsigned char* const srcEnd = src+pixels;
	unsigned char* dst = ctxt->dst+pixels*3*i;
	while(src!=srcEnd) {
		// Minor speed hack: divide by 256 instead of 255, so multiply by 220 instead of 219
		//   so ranges work out the same (src 255 still maps to 219 before adding 16)
		*dst = 235 - (*src++ * 220)/256;
		dst+=3;
	}
}

void CameraSourceQTKit::process_grayscale_zerowhite(const unsigned char * s, unsigned int width, unsigned int height) {
	//TimeET p;
	const unsigned int components=3;
	ssize_t reqSize = sizeof(ImageHeader) + width * height * components;
	RCRegion * region = getUnusedRegion(reqSize, 0);
	unsigned char * buf = reinterpret_cast<unsigned char*>(region->Base());
	new (region->Base()) ImageHeader(0, layer, width, height, components, ++frame, get_time(), nextName());
	
	unsigned char* dst = buf + sizeof(ImageHeader);
	memset(dst, 128, width*height*components);
	BlockContext ctxt = { s, width, dst };
	if(parallel) {
		dispatch_queue_t q_default = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0);
		dispatch_apply_f(height/ROWS_PER_BLOCK, q_default, &ctxt, process_grayscale_zerowhite_Block);
	} else {
		for(size_t i=0; i<height/ROWS_PER_BLOCK; ++i)
			process_grayscale_zerowhite_Block(&ctxt,i);
	}
	//std::cout << p.Age() << std::endl;
	setImage(region);
}



#pragma mark-

@implementation CameraSourceQTKitDelegate
-(CameraSourceQTKitDelegate*)initWithTarget:(class CameraSourceQTKit*)tgt {
	target=tgt;
	return self;
}

-(void)captureOutput:(QTCaptureOutput *)captureOutput didOutputVideoFrame:(CVImageBufferRef)videoFrame withSampleBuffer:(QTSampleBuffer *)sampleBuffer fromConnection:(QTCaptureConnection *)connection
{
	/*
	std::cout << "Got frame " << CC2Str([[sampleBuffer formatDescription] formatType])<< " desc " << [[[sampleBuffer formatDescription] localizedFormatSummary] UTF8String]  << std::endl;
	NSDictionary* attr = [[sampleBuffer formatDescription] formatDescriptionAttributes];
	if(attr==NULL) {
		std::cerr << "no attributes" << std::endl;
	} else {
		for (id key in attr) {
			NSLog(@"key: %@, value: %@", key, [attr objectForKey:key]);
		}
	}
	 */
	target->processImage(videoFrame,sampleBuffer);
}
@end

#endif // 10.6 or later
#endif // Apple platform

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 *
 * $Author: ejt $
 * $Name:  $
 * $Revision: 1.4 $
 * $State: Exp $
 * $Date: 2011/03/10 16:45:44 $
 */
