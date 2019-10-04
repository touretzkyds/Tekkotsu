#include "ImageStreamDriver.h"
#include "Shared/get_time.h"
#include "Shared/MarkScope.h"
#include "Shared/RobotInfo.h"
#include "Shared/Config.h"
#include "Shared/ImageUtil.h"
#include "Shared/debuget.h"
#include "Shared/InverseMarkScope.h"

#include <png.h>

using namespace std; 

const char * ImageStreamDriver::formatNames[NUM_FORMATS+1] = { "yuv", "png", "jpeg", "tekkotsu", NULL };
//declare explicit instances of the NamedEnumerations we're using
//(cuts down on some pretty significant binary size / debugging symbol bloat)
INSTANTIATE_NAMEDENUMERATION_STATICS(ImageStreamDriver::format_t);

const std::string ImageStreamDriver::autoRegisterDriver = DeviceDriver::getRegistry().registerType<ImageStreamDriver>("ImageStream");

void ImageStreamDriver::plistValueChanged(const plist::PrimitiveBase& pl) {
	if(&pl==&format) {
		CommPort * comm = getComm(commName);
		if(comm!=NULL) {
			disconnect(comm);
			connect(comm);
		}
	} else {
		DataStreamDriver::plistValueChanged(pl);
	}
}

bool ImageStreamDriver::readData(std::istream& is) {
	RCRegion * region=readImage(is);
	if(region==NULL)
		return false;
	if(!paceInput) try {
		// loop to run through any buffer backlog, get most recent image
		region->AddReference(); // to mark this region 'in use' so readImage doesn't use it for the next image
		char firstByte;
		while(is.readsome(&firstByte,1)>0 && is) {
			is.putback(firstByte); // had data, put it back in the buffer so we can process it
			RCRegion * newRegion = readImage(is);
			if(newRegion!=NULL) {
				ASSERT(region!=newRegion,"ImageStreamDriver handed out same region twice");
				// we're using the new region, put the old one back in the queue for recycling
				region->RemoveReference();
				region=newRegion;
				region->AddReference();
			}
			testCancel();
		} 
		region->RemoveReference();
	} catch(...) {
		region->RemoveReference();
		throw;
	}

	setImage(region);
	return true;
}

RCRegion * ImageStreamDriver::readImage(std::istream& is) {
	format_t expectedFormat = format; // cache format in case a new one is requested, next image will still be old one
	
	// block until we have data... might cancel out while we're waiting for an image to come
	char firstByte;
	is.get(firstByte);
	if(!is)
		return NULL;
	is.unget();
	testCancel();
	timestamp = get_time();
	
	const size_t HEADER_SIZE = sizeof(ImageHeader);
	
	RCRegion* region = (payloadSize==0) ? NULL : getUnusedRegion(payloadSize, 0);
	// a reference to region is kept in the regions list, so it's OK if we return/throw/cancel, doesn't leak
	unsigned int layer=0;
	
	switch(expectedFormat) {
		case FORMAT_YUV: {
			size_t imgSize = CameraResolutionX*CameraResolutionY*3;
			payloadSize = HEADER_SIZE + imgSize;
			if(region==NULL) {
				region = getUnusedRegion(payloadSize, 0);
			} else if(region->Size()<payloadSize) {
				// not a leak - old region is still in regions list, we'll dereference it on a future getUnusedRegion call
				region = getUnusedRegion(payloadSize, 0);
			}
			char * payload = region->Base();
			is.read(payload+HEADER_SIZE, imgSize);
			testCancel();
			if(!is)
				return NULL;
			new (payload) ImageHeader(sid, layer, CameraResolutionX, CameraResolutionY, 3, ++frameNumber, timestamp, nextName());
		} break;
			
		case FORMAT_PNG: {
			if (sid == ProjectInterface::visRawDepthSID) {
				size_t w=0,h=0,c=0;
				if(region==NULL) {
					// first frame, get meta-information to find buffer size
					if(!image_util::decodePNG(is,w,h,c)) {
						cerr << "ImageStreamDriver JPEG header decompression failed" << endl;
						return NULL;
					}
					payloadSize = HEADER_SIZE + w*h*2;
					region = getUnusedRegion(payloadSize, 0);
				}
				char * img=region->Base() + HEADER_SIZE;
				size_t imgSize=region->Size()-HEADER_SIZE;
				// usually size will be the same, assume we can reuse previous buffer
				if(!image_util::decodePNGToDepth(is,w,h,c,img,imgSize)) {
					// maybe image grew...
					if(w*h*c<=imgSize) { // nope should've worked, must be corrupted
						cerr << "ImageStreamDriver PNG decompression failed" << endl;
						return NULL;
					}
					// resize buffer and try again
					payloadSize = HEADER_SIZE + w*h*2;
					// not a leak - old region is still in regions list, we'll dereference it on a future getUnusedRegion call
					region = getUnusedRegion(payloadSize, 0);
					img=region->Base() + HEADER_SIZE;
					imgSize=region->Size()-HEADER_SIZE;
					if(!image_util::decodePNGToDepth(is,w,h,c,img,imgSize)) { // ok, must be corrupted as well
						cerr << "ImageStreamDriver PNG decompression failed after resize" << endl;
						return NULL;
					}
				}
				testCancel();

				payloadSize = HEADER_SIZE + w*h*2;
				new (region->Base()) ImageHeader(sid, layer, w, h, 2, ++frameNumber, timestamp, nextName());
			}
			// Regular images
			else {
				size_t w=0,h=0,c=0;
				if(region==NULL) {
					// first frame, get meta-information to find buffer size
					if(!image_util::decodePNG(is,w,h,c)) {
						cerr << "ImageStreamDriver JPEG header decompression failed" << endl;
						return NULL;
					}
					payloadSize = HEADER_SIZE + w*h*c;
					region = getUnusedRegion(payloadSize, 0);
				}
				char * img=region->Base() + HEADER_SIZE;
				size_t imgSize=region->Size()-HEADER_SIZE;
				// usually size will be the same, assume we can reuse previous buffer
				if(!image_util::decodePNG(is,w,h,c,img,imgSize)) {
					// maybe image grew...
					if(w*h*c<=imgSize) { // nope should've worked, must be corrupted
						cerr << "ImageStreamDriver PNG decompression failed" << endl;
						return NULL;
					}
					// resize buffer and try again
					payloadSize = HEADER_SIZE + w*h*c;
					// not a leak - old region is still in regions list, we'll dereference it on a future getUnusedRegion call
					region = getUnusedRegion(payloadSize, 0);
					img=region->Base() + HEADER_SIZE;
					imgSize=region->Size()-HEADER_SIZE;
					if(!image_util::decodePNG(is,w,h,c,img,imgSize)) { // ok, must be corrupted as well
						cerr << "ImageStreamDriver PNG decompression failed after resize" << endl;
						return NULL;
					}
				}
				testCancel();

				payloadSize = HEADER_SIZE + w*h*c;
				new (region->Base()) ImageHeader(sid, layer, w, h, c, ++frameNumber, timestamp, nextName());
			}
		} break;
			
		case FORMAT_JPEG: {
			size_t w=0,h=0,c=0;
			if(region==NULL) {
				// first frame, get meta-information to find buffer size
				if(!image_util::decodeJPEG(is,w,h,c)) {
					cerr << "ImageStreamDriver JPEG header decompression failed" << endl;
					return NULL;
				}
				payloadSize = HEADER_SIZE + w*h*c;
				region = getUnusedRegion(payloadSize, 0);
			}
			char * img=region->Base() + HEADER_SIZE;
			size_t imgSize=region->Size()-HEADER_SIZE;
			// usually size will be the same, assume we can reuse previous buffer
			if(!image_util::decodeJPEG(is,w,h,c,img,imgSize)) {
				// maybe image grew...
				if(w*h*c<=imgSize) { // nope should've worked, must be corrupted
					cerr << "ImageStreamDriver JPEG decompression failed" << endl;
					return NULL;
				}
				// resize buffer and try again
				payloadSize = HEADER_SIZE + w*h*c;
				// not a leak - old region is still in regions list, we'll dereference it on a future getUnusedRegion call
				region = getUnusedRegion(payloadSize, 0);
				img=region->Base() + HEADER_SIZE;
				imgSize=region->Size()-HEADER_SIZE;
				if(!image_util::decodeJPEG(is,w,h,c,img,imgSize)) { // ok, must be corrupted as well
					cerr << "ImageStreamDriver JPEG decompression failed after resize" << endl;
					return NULL;
				}
			}
			testCancel();

			payloadSize = HEADER_SIZE + w*h*c;
			new (region->Base()) ImageHeader(sid, layer, w, h, c, ++frameNumber, timestamp, nextName());
		} break;
			
		case FORMAT_TEKKOTSU: {
			// ugh, LoadSave handles FILE* and char*, but not iostreams...
			// gonna be a little ugly :(
			
			unsigned int width=CameraResolutionX, height=CameraResolutionY, components=3;
			
			// RAW CAM BEHAVIOR HEADER
			char tmp[256];
			unsigned int len;
			string fmt;
			is.read(tmp,LoadSave::getSerializedSize(len));
			LoadSave::decode(len,tmp,LoadSave::getSerializedSize(len));
			if(len!=13) {
				cerr << "Expecting Tekkotsu image format, but image header doesn't match! (len==" << len << ")" << endl;
				return NULL;
			}
			is.read(tmp,len+1);
			if(strncmp(tmp,"TekkotsuImage",len+1)!=0) {
				tmp[len+2]='\0';
				cerr << "Expecting Tekkotsu image format, but image header doesn't match! (" << tmp << ")" << endl;
				return NULL;
			}
			
			int encoding; //Config::vision_config::RawCamConfig::encoding_t
			is.read(tmp,LoadSave::getSerializedSize(encoding));
			LoadSave::decode(encoding,tmp,LoadSave::getSerializedSize(encoding));
			if(encoding==Config::vision_config::ENCODE_SINGLE_CHANNEL)
				components=1;
			int compression; //Config::vision_config::RawCamConfig::compression_t
			is.read(tmp,LoadSave::getSerializedSize(compression));
			LoadSave::decode(compression,tmp,LoadSave::getSerializedSize(compression));
			is.read(tmp,LoadSave::getSerializedSize(width));
			LoadSave::decode(width,tmp,LoadSave::getSerializedSize(width));
			is.read(tmp,LoadSave::getSerializedSize(height));
			LoadSave::decode(height,tmp,LoadSave::getSerializedSize(height));
			unsigned int remote_timestamp;
			is.read(tmp,LoadSave::getSerializedSize(remote_timestamp));
			LoadSave::decode(remote_timestamp,tmp,LoadSave::getSerializedSize(remote_timestamp));
			unsigned int fnum;
			is.read(tmp,LoadSave::getSerializedSize(fnum));
			LoadSave::decode(fnum,tmp,LoadSave::getSerializedSize(fnum));
			
			// have everything we need to set up output header...
			payloadSize = HEADER_SIZE + width*height*components;
			if(region==NULL) {
				region = getUnusedRegion(payloadSize, 0);
			} else if(region->Size() < payloadSize) {
				// not a leak - old region is still in regions list, we'll dereference it on a future getUnusedRegion call
				region = getUnusedRegion(payloadSize, 0);
			}
			
			unsigned int remain = width*height*components;
			char * b=region->Base()+HEADER_SIZE;
			
			for(unsigned int i=0; i<components; ++i) {
				// FILTER BANK GENERATOR HEADER
				is.read(tmp,LoadSave::getSerializedSize(len));
				LoadSave::decode(len,tmp,LoadSave::getSerializedSize(len));
				if(len!=8) {
					cerr << "Expecting FbkImage image format, but header doesn't match! (len==" << len << ")" << endl;
					return NULL;
				}
				is.read(tmp,len+1);
				if(strncmp(tmp,"FbkImage",len+1)!=0) {
					tmp[len+2]='\0';
					cerr << "Expecting FbkImage image format, but image header doesn't match! (" << tmp << ")" << endl;
					return NULL;
				}
				unsigned int lwidth;
				is.read(tmp,LoadSave::getSerializedSize(lwidth));
				LoadSave::decode(lwidth,tmp,LoadSave::getSerializedSize(lwidth));
				unsigned int lheight;
				is.read(tmp,LoadSave::getSerializedSize(lheight));
				LoadSave::decode(lheight,tmp,LoadSave::getSerializedSize(lheight));
				unsigned int lyr;
				is.read(tmp,LoadSave::getSerializedSize(lyr));
				LoadSave::decode(lyr,tmp,LoadSave::getSerializedSize(lyr));
				unsigned int chan_id;
				is.read(tmp,LoadSave::getSerializedSize(chan_id));
				LoadSave::decode(chan_id,tmp,LoadSave::getSerializedSize(chan_id));
				
				// GENERATOR SUBCLASS HEADER
				is.read(tmp,LoadSave::getSerializedSize(len));
				LoadSave::decode(len,tmp,LoadSave::getSerializedSize(len));
				is.read(tmp,len+1);
				if(strcmp(tmp,"blank")==0) {
					int useChan=(components==1)?i:chan_id;
					int off=useChan;
					for(unsigned int y=0; y<height; y++) {
						for(unsigned int x=0; x<width; x++) {
							b[off]=128;
							off+=components;
						}
					}
				} else if(strcmp(tmp,"RawImage")==0) {
					int useChan=(components==1)?i:chan_id;
					vector<char> chan(lwidth*lheight);
					is.read(&chan.front(),chan.size());
					copyImage(b,width,height,components,&chan.front(),lwidth,lheight,useChan);
					
				} else if(strcmp(tmp,"JPEGGrayscale")==0) {
					int useChan=(components==1)?i:chan_id;
					is.read(tmp,LoadSave::getSerializedSize(len));
					LoadSave::decode(len,tmp,LoadSave::getSerializedSize(len));
					vector<char> jpeg(len);
					is.read(&jpeg.front(),jpeg.size());
					size_t jwidth,jheight,jcomp;
					image_util::decodeJPEG(&jpeg.front(), jpeg.size(), jwidth, jheight, jcomp);
					if(jcomp!=1) {
						cerr << "Got color image where grayscale was expected" << endl;
						return NULL;
					}
					vector<char> chan(jwidth*jheight);
					size_t tsize=chan.size();
					char* tchan=&chan.front();
					if(!image_util::decodeJPEG(&jpeg.front(), jpeg.size(), jwidth, jheight, jcomp, tchan, tsize)) {
						cerr << "JPEG decompression failed" << endl;
						return NULL;
					}
					copyImage(b,width,height,components,&chan.front(),lwidth,lheight,useChan);
					
				} else if(strcmp(tmp,"JPEGColor")==0) {
					is.read(tmp,LoadSave::getSerializedSize(len));
					LoadSave::decode(len,tmp,LoadSave::getSerializedSize(len));
					vector<char> jpeg(len);
					is.read(&jpeg.front(),jpeg.size());
					size_t jwidth,jheight,jcomp;
					size_t tsize=remain;
					if(!image_util::decodeJPEG(&jpeg.front(), jpeg.size(), jwidth, jheight, jcomp, b, tsize)) {
						cerr << "JPEG decompression failed" << endl;
						return NULL;
					}
					i=components;
					
				} else {
					cerr << "Unknown image generator " << tmp << endl;
					return NULL;
				}
			}
			
			testCancel();
			if(!is)
				return NULL;
			new (region->Base()) ImageHeader(sid, layer, width, height, components, ++frameNumber, timestamp, nextName());
		} break;
	}
	ASSERTRETVAL(region!=NULL,"image memory region is still NULL after reading, unknown format?",NULL);
	return region;
}

void ImageStreamDriver::copyImage(char * buf, unsigned int width, unsigned int height, unsigned int channels, const char * chan, unsigned int lwidth, unsigned int lheight, unsigned int c) {
	if(lwidth==width && lheight==height) {
		// same size, straight copy
		for(unsigned int y=0;y<height;y++) {
			unsigned int datarowstart=y*width*channels+c;
			unsigned int tmprowstart=y*lwidth;
			for(unsigned int x=0;x<width;x++)
				buf[datarowstart+x*channels]=chan[tmprowstart+x];
		}
	} else {
		// upsample image
		//we'll linearly interpolate between pixels

		//hold edges, interpolate through middle:
		//  if we have 2 samples, scaling up to 4
		//   index: 0   1    2   3
		// maps to: 0  1/3  2/3  1
		float xsc=(lwidth-1)/(float)(width-1);
		float ysc=(lheight-1)/(float)(height-1);
		for(unsigned int y=0;y<height-1;y++) {
			unsigned int datarowstart=y*width*channels+c;
			float ty=y*ysc;
			unsigned int ly=(int)ty; //lower pixel index
			float fy=ty-ly; //upper pixel weight
			unsigned int tmprowstart=ly*lwidth;
			for(unsigned int x=0;x<width-1;x++) {
				float tx=x*xsc;
				unsigned int lx=(int)tx; //lower pixel index
				float fx=tx-lx; //upper pixel weight
				
				float lv=((int)chan[tmprowstart+lx]&0xFF)*(1-fx)+((int)chan[tmprowstart+lx+1]&0xFF)*fx;
				float uv=((int)chan[tmprowstart+lx+lwidth]&0xFF)*(1-fx)+((int)chan[tmprowstart+lx+1+lwidth]&0xFF)*fx;
				buf[datarowstart+x*channels]=(char)(lv*(1-fy)+uv*fy);
			}
			buf[datarowstart+(width-1)*channels]=chan[tmprowstart+lwidth-1];
		}
		unsigned int datarowstart=width*(height-1)*channels+c;
		unsigned int tmprowstart=lwidth*(lheight-1);
		for(unsigned int x=0;x<width-1;x++) {
			float tx=x*xsc;
			unsigned int lx=(int)tx; //lower pixel index
			float fx=tx-lx; //upper pixel weight
			buf[datarowstart+x*channels]=(char)(((int)chan[tmprowstart+lx]&0xFF)*(1-fx)+((int)chan[tmprowstart+lx+1]&0xFF)*fx);
		}
		buf[datarowstart+(width-1)*channels]=chan[tmprowstart+lwidth-1];
	}
}

void ImageStreamDriver::connect(CommPort* comm) {
	DataStreamDriver::connect(comm);
	format.addPrimitiveListener(this);
}

void ImageStreamDriver::disconnect(CommPort* comm) {
	format.removePrimitiveListener(this);
	DataStreamDriver::disconnect(comm);
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
