#ifndef PLATFORM_APERIOS

#include "BufferedImageGenerator.h"
#include "Events/DataEvent.h"
#include "Events/FilterBankEvent.h"
#include "Wireless/Socket.h"
#include "Events/EventRouter.h"
#include "Shared/debuget.h"
#include "Shared/ProjectInterface.h"

using namespace std;

void BufferedImageGenerator::doEvent() {
	if(event->getGeneratorID()!=getListenGeneratorID() || event->getSourceID()!=getListenSourceID())
		return;
	if(event->getTypeID()==EventBase::activateETID) {
		const DataEvent<ImageSource>* data=dynamic_cast<const DataEvent<ImageSource>*>(event);
		if(data==NULL) {
			serr->printf("Error: %s(%s) received event of the wrong type",getClassName().c_str(),getName().c_str());
			return;
		}
		if(imgsrc.layer!=data->getData().layer || imgsrc.channels!=data->getData().channels) {
			//can "quietly" switch the layer of transmission as long as the width and height were scaled properly
			//just need to reset the increments if a different layer is being used.
			unsigned int i;
			for(i=0; i<data->getData().layer; i++) {
				increments[i] = 1;
				strides[i]=data->getData().width>>(data->getData().layer-i);
				skips[i]=0;
			}
			increments[i] = data->getData().channels;
			strides[i]=data->getData().width*data->getData().channels;
			skips[i]=0;
			for(++i; i<numLayers; i++) {
				increments[i] = 1;
				strides[i]=data->getData().width<<(i-data->getData().layer);
				skips[i]=0;
			}
		}
		imgsrc=data->getData();
		sysFrameNumber=frameNumber=imgsrc.frameIndex;
		invalidateCaches(); //mark everything invalid
		if(numLayers>0) {
			// have images, check if the dimensions have changed
			if(imgsrc.width!=getWidth(imgsrc.layer) || imgsrc.height!=getHeight(imgsrc.layer)) {
				freeCaches();
				setDimensions();
				if(framesProcessed==0)
					serr->printf("WARNING: the image dimensions don't match values predicted by RobotInfo consts, \"full\" layer now %dx%d\n",widths[ProjectInterface::fullLayer],heights[ProjectInterface::fullLayer]);
				else
					serr->printf("WARNING: the image dimensions have changed since last frame, \"full\" layer now %dx%d\n",widths[ProjectInterface::fullLayer],heights[ProjectInterface::fullLayer]);
				erouter->postEvent(EventBase::cameraResolutionEGID,event->getSourceID(),EventBase::statusETID);
			} else if(framesProcessed==0) {
				// first frame, always set it anyway
				setDimensions();
			}
		}
		// -- reassign to the new buffer: --
		unsigned int i=imgsrc.layer;
		for(unsigned int j=0; j<imgsrc.channels; j++) {
			if(isAllocated[i][j]) { //in case the imgsrc layer changes
				delete [] images[i][j];
				images[i][j]=NULL;
				isAllocated[i][j]=false;
			}
			imageValids[i][j]=true;
		}
		images[i][RawCameraGenerator::CHAN_Y]=imgsrc.img+0;
		images[i][RawCameraGenerator::CHAN_U]=imgsrc.img+1;
		images[i][RawCameraGenerator::CHAN_V]=imgsrc.img+2;
		framesProcessed++;
	}
	erouter->postEvent(FilterBankEvent(this,getGeneratorID(),getSourceID(),event->getTypeID()));
}

unsigned int
BufferedImageGenerator::getBinSize() const {
	unsigned int used=FilterBankGenerator::getBinSize();
	used+=getSerializedSize("RawImage");
	used+=widths[selectedSaveLayer]*heights[selectedSaveLayer];
	return used;
}

unsigned int
BufferedImageGenerator::loadBuffer(const char buf[], unsigned int len, const char* filename) {
	unsigned int origlen=len;
	if(!checkInc(FilterBankGenerator::loadBuffer(buf,len,filename),buf,len)) return 0;
	std::string tmp;
	if(decode(tmp,buf,len)) return 0;
	if(tmp!="RawImage") {
		serr->printf("Unhandled image type for BufferedImageGenerator: %s",tmp.c_str());
		return 0;
	} else if(selectedSaveLayer!=numLayers-1) {
		serr->printf("Can't load into BufferedImageGenerator layer %d!=%d",selectedSaveLayer,numLayers-1);
		return 0;
	} else {
		if(images[selectedSaveLayer][selectedSaveChannel]==NULL)
			images[selectedSaveLayer][selectedSaveChannel]=createImageCache(selectedSaveLayer,selectedSaveChannel);
		unsigned char* img=images[selectedSaveLayer][selectedSaveChannel];
		unsigned int used=widths[selectedSaveLayer]*heights[selectedSaveLayer];
		ASSERTRETVAL(used<=len,"buffer too small",0);
		memcpy(img,buf,used);
		len-=used; buf+=used;
		imageValids[selectedSaveLayer][selectedSaveChannel]=true;
		return origlen-len;	
	}
}

unsigned int
BufferedImageGenerator::saveBuffer(char buf[], unsigned int len) const {
	unsigned int origlen=len;
	if(!checkInc(FilterBankGenerator::saveBuffer(buf,len),buf,len)) return 0;
	if(!encodeInc("RawImage",buf,len)) return 0;
	
	if(images[selectedSaveLayer][selectedSaveChannel]==NULL) {
		serr->printf("BufferedImageGenerator::saveBuffer() failed because selected image is NULL -- call selectSaveImage first to make sure it's up to date\n");
		return 0;
	}
	if(!imageValids[selectedSaveLayer][selectedSaveChannel]) {
		serr->printf("BufferedImageGenerator::saveBuffer() failed because selected image is invalid -- call selectSaveImage first to make sure it's up to date\n");
		return 0;
	}
	unsigned char* img=images[selectedSaveLayer][selectedSaveChannel];
	unsigned int used=widths[selectedSaveLayer]*heights[selectedSaveLayer];
	ASSERTRETVAL(used<=len,"buffer too small " << len << ' ' <<  widths[selectedSaveLayer] << ' ' << heights[selectedSaveLayer],0);
	unsigned int inc=getIncrement(selectedSaveLayer);
	if(inc==1) {
		//special case, straight copy
		for(unsigned int y=0; y<heights[selectedSaveLayer]; y++) {
			memcpy(buf,img,widths[selectedSaveLayer]);
			buf+=widths[selectedSaveLayer];
			img+=getStride(selectedSaveLayer);
		}
	} else {
		//otherwise, interleaved or subsampling
		for(unsigned int y=0; y<heights[selectedSaveLayer]; y++) {
			unsigned char* const rowend=img+widths[selectedSaveLayer]*inc;
			while(img!=rowend) {
				*buf++=*img;
				img+=inc;
			}
			img+=getSkip(selectedSaveLayer);
		}
	}
	len-=used;
	
	return origlen-len;
}

unsigned int
BufferedImageGenerator::saveFileStream(FILE * f) const {
	unsigned int totalused=0;
	unsigned int used;
	{ //sigh, inheritance has failed me (I wouldn't want FilterBankGenerator::saveFileStream() to call the virtuals...)
		unsigned int sz=FilterBankGenerator::getBinSize();
		char * buf = new char[sz];
		memset(buf,0xF0,sz);
		if(buf==NULL) {
			std::cout << "*** WARNING could not allocate " << sz << " bytes for loadFile";
			return 0;
		}
		unsigned int resp=FilterBankGenerator::saveBuffer(buf,sz);
		if(resp==0) {
			std::cout << "*** WARNING saveBuffer didn't write any data (possibly due to overflow or other error)" << std::endl;
			fwrite(buf,1,sz,f);
		}	else {
			unsigned int wrote=fwrite(buf,1,resp,f);
			if(wrote!=resp)
				std::cout << "*** WARNING short write (wrote " << wrote << ", expected " << resp << ")" << std::endl;
		}
		delete [] buf;
		used=resp;
	}
	if(0==used) return 0;
	totalused+=used;
	if(0==(used=encode("RawImage",f))) return 0;
	totalused+=used;
	
	if(images[selectedSaveLayer][selectedSaveChannel]==NULL) {
		serr->printf("BufferedImageGenerator::saveBuffer() failed because selected image is NULL -- call selectSaveImage first to make sure it's up to date\n");
		return 0;
	}
	if(!imageValids[selectedSaveLayer][selectedSaveChannel]) {
		serr->printf("BufferedImageGenerator::saveBuffer() failed because selected image is invalid -- call selectSaveImage first to make sure it's up to date\n");
		return 0;
	}
	unsigned char* img=images[selectedSaveLayer][selectedSaveChannel];
	used=widths[selectedSaveLayer]*heights[selectedSaveLayer];
	unsigned int inc=getIncrement(selectedSaveLayer);
	if(inc==1) {
		//special case, straight copy
		sout->printf("Saving %d by %d\n",widths[selectedSaveLayer],heights[selectedSaveLayer]);
		for(unsigned int y=0; y<heights[selectedSaveLayer]; y++) {
			if(fwrite(img,widths[selectedSaveLayer],1,f)==0) {
				serr->printf("short write on image data - ran out of space?\n");
				return 0;
			}
			img+=getStride(selectedSaveLayer);
		}
	} else {
		//otherwise, interleaved or subsampling
		for(unsigned int y=0; y<heights[selectedSaveLayer]; y++) {
			unsigned char* const rowend=img+widths[selectedSaveLayer]*inc;
			while(img!=rowend) {
				if(fputc(*img,f)==EOF) {
					serr->printf("short write on image data - ran out of space?\n");
					return 0;
				}
				img+=inc;
			}
			img+=getSkip(selectedSaveLayer);
		}
	}
	totalused+=used;
	
	return totalused;
}

void BufferedImageGenerator::freeCaches() {
	FilterBankGenerator::freeCaches();
	for(unsigned int i=0; i<numLayers; i++)
		for(unsigned int j=0; j<numChannels; j++)
			isAllocated[i][j]=false;
}

void BufferedImageGenerator::invalidateCaches() {
	for(unsigned int i=0; i<numLayers; i++)
		for(unsigned int j=0; j<numChannels; j++) {
			if(!isAllocated[i][j])
				images[i][j]=NULL;
			imageValids[i][j]=false;
		}
}

unsigned char * BufferedImageGenerator::createImageCache(unsigned int layer, unsigned int channel) const {
	if(layer!=imgsrc.layer || imgsrc.channels==1) {
		isAllocated[layer][channel]=true;
		return new unsigned char[widths[layer]*heights[layer]];
	} else {
		ASSERT(channel>=imgsrc.channels,"createImageCache for image that should come from src")
		// increment is set to imgsrc.channels, so we need to allocate multiple images at once for the generate channels
		unsigned int base=(channel/imgsrc.channels)*imgsrc.channels; // round down to nearest multiple of imgsrc.channels
		if(images[layer][base]==NULL)
			images[layer][base]=new unsigned char[widths[layer]*heights[layer]*imgsrc.channels];
		for(unsigned int i=base+1; i<base+imgsrc.channels; ++i) {
			ASSERT(!isAllocated[layer][i],"createImageCache for image already allocated!");
			ASSERT(images[layer][i]==NULL,"createImageCache for image already assigned!");
			images[layer][i]=images[layer][i-1]+1;
		}
		isAllocated[layer][base]=true;
		return images[layer][channel];
	}
}
void BufferedImageGenerator::calcImage(unsigned int layer, unsigned int channel) {
	//cout << "BufferedImageGenerator::calcImage(" << layer << ',' << channel << ')' << endl;
	ASSERTRET(layer!=imgsrc.layer || channel>=imgsrc.channels, "calcImage on src channel?");
	switch(channel) {
		case RawCameraGenerator::CHAN_Y:
			if(layer>imgsrc.layer) upsampleImage(imgsrc.layer,channel,layer);
			else downsampleImage(layer,channel);
			break;
		case RawCameraGenerator::CHAN_U:
		case RawCameraGenerator::CHAN_V:
			if(imgsrc.channels>1) {
				if(layer>imgsrc.layer) upsampleImage(imgsrc.layer,channel,layer);
				else downsampleImage(layer,channel);
			} else //grayscale image, use blank U and V channels
				memset(images[layer][channel],128,widths[layer]*heights[layer]);
			break;
		case RawCameraGenerator::CHAN_Y_DX:
			calcDx(layer);
			break;
		case RawCameraGenerator::CHAN_Y_DY:
			calcDy(layer);
			break;
		case RawCameraGenerator::CHAN_Y_DXDY:
			calcDxDy(layer);
			break;
		default:
			cerr << "Bad layer selection!" << endl;
	}
}
void BufferedImageGenerator::setDimensions() {
	if(imgsrc.img==NULL) //don't have an image to set from
		return;
	// set dimensions of layers below the input layer
	for(unsigned int i=0; i<=imgsrc.layer; i++) {
		//s is the scaling factor -- 2 means *half* size
		unsigned int s=(1<<(imgsrc.layer-i));
		//width and height are scaled down (divide by s)
		widths[i]=strides[i]=imgsrc.width/s;
		heights[i]=imgsrc.height/s;
		//stride is same as width (set above) -- we allocate these layers, don't skip rows
		skips[i]=0;
		increments[i]=1;
		//cout << "setDimensions() " << widths[i] << ' ' << skips[i] << ' ' << strides[i] << endl;
	}
	// set dimensions of the input layer (interleaved -- note increment and stride)
	increments[imgsrc.layer] = imgsrc.channels;
	widths[imgsrc.layer]=imgsrc.width;
	heights[imgsrc.layer]=imgsrc.height;
	strides[imgsrc.layer]=imgsrc.width*imgsrc.channels;
	skips[imgsrc.layer]=0;
	//set dimensions for layers above the input layer
	for(unsigned int i=imgsrc.layer+1; i<numLayers; i++) {
		//s is the scaling factor -- 2 means *double* size
		unsigned int s=(1<<(i-imgsrc.layer));
		//multiply by s
		widths[i]=strides[i]=imgsrc.width*s;
		heights[i]=imgsrc.height*s;
		//stride is same as width (set above) -- we allocate these layers, don't skip rows
		skips[i]=0;
		increments[i]=1;
	}
}
void BufferedImageGenerator::destruct() {
	FilterBankGenerator::destruct();
	for(unsigned int i=0; i<numLayers; i++)
		delete [] isAllocated[i];
	delete [] isAllocated;
	isAllocated=NULL;
}
void BufferedImageGenerator::setNumImages(unsigned int nLayers, unsigned int nChannels) {
	if(nLayers==numLayers && nChannels==numChannels)
		return;
	FilterBankGenerator::setNumImages(nLayers,nChannels);
	isAllocated=new bool*[numLayers];
	for(unsigned int i=0; i<numLayers; i++) {
		isAllocated[i]=new bool[numChannels];
		for(unsigned int j=0; j<numChannels; j++)
			isAllocated[i][j]=false;
	}
	setDimensions();
}

void BufferedImageGenerator::upsampleImage(unsigned int srcLayer, unsigned int chan, unsigned int destLayer) {
	ASSERTRET(destLayer!=imgsrc.layer,"upsample into source layer")
	unsigned char * cur=images[destLayer][chan];
	ASSERTRET(cur!=NULL,"destination layer is NULL");
	unsigned char * orig=getImage(srcLayer,chan);
	ASSERTRET(orig!=NULL,"source layer is NULL");
	unsigned int width=widths[destLayer];
	unsigned int height=heights[destLayer];
	unsigned int inc=getIncrement(srcLayer);
	int power=destLayer-srcLayer;
	ASSERTRET(power>0,"upsampleImage attempting to downsample")
	
	unsigned char * const imgend=cur+width*height;
	while(cur!=imgend) {
		unsigned char * const row=cur;
		unsigned char * const rowend=cur+width;
		//upsample pixels within one row
		while(cur<rowend) {
			for(int p=1<<power; p>0; p--)
				*cur++=*orig;
			orig+=inc;
		}
		//now replicate that row 1<<power times, doubling each time
		for(int p=0; p<power; p++) {
			unsigned int avail=width*(1<<p);
			memcpy(cur,row,avail);
			cur+=avail;
		}
		orig+=getSkip(srcLayer);
	}
	imageValids[destLayer][chan]=true;
}

void BufferedImageGenerator::downsampleImage(unsigned int destLayer, unsigned int chan) {
	ASSERTRET(destLayer!=imgsrc.layer,"downsample into source layer")
	// find closest available layer to source from
	unsigned int layer=destLayer;
	while(layer<numLayers && !imageValids[layer][chan])
		layer++;
	ASSERTRET(layer<numLayers,"valid layer to downsample from could not be found!");
	if (!(layer<numLayers))
	  std::cout << "layer = " << layer << " numLayers = " << numLayers << std::endl;
	// we'll compute in-between layers as we go (easier computation and might be able to reuse them anyway)
	// layer is the current layer we're downsampling into (layer+1 is the one we're sampling from)
	for(unsigned int srcL=layer--; layer>=destLayer; srcL=layer--) {
		unsigned int srcInc=getIncrement(srcL); // destination increment is guaranteed to be 1, but source increment isn't
		unsigned char * s=getImage(srcL,chan);
		if(images[layer][chan]==NULL)
			images[layer][chan]=createImageCache(layer,chan);
		unsigned char * dst=images[layer][chan];
		unsigned char * const dstEnd=dst+widths[layer]*heights[layer];
		while(dst!=dstEnd) {
			unsigned char * const rowEnd=dst+widths[layer];
			while(dst!=rowEnd) {
				unsigned short x=*s;
				x+=*(s+strides[srcL]);
				s+=srcInc;
				x+=*s;
				x+=*(s+strides[srcL]);
				s+=srcInc;
				*dst++ = x/4;
			}
			s+=strides[srcL];
		}
		imageValids[layer][chan]=true;
	}
}

void BufferedImageGenerator::calcDx(unsigned int layer, unsigned int srcChan/*=RawCameraGenerator::CHAN_Y*/, unsigned int dstChan/*=RawCameraGenerator::CHAN_Y_DX*/) {
	unsigned char * s=getImage(layer,srcChan);
	unsigned char * dst=images[layer][dstChan];
	unsigned int inc=getIncrement(layer);
	unsigned int skip=getSkip(layer)+inc;
	unsigned char * const dstEnd=dst+getStride(layer)*heights[layer];
	unsigned int sc=2;  // i think this should be 1, but this is to provide better compatability with the OPEN-R implementation
	while(dst!=dstEnd) {
		unsigned char * const rowEnd=dst+widths[layer]*inc-inc;
		unsigned char left,right;
		while(dst!=rowEnd) {
			left=(*s)>>sc;
			s+=inc;
			right=(*s)>>sc;
			*dst=right+128-left;
			dst+=inc;
		}
		*dst=128; //rightmost column is always 128 to retain image dimensions
		dst+=skip;
		s+=skip;
	}
	imageValids[layer][dstChan]=true;
}
void BufferedImageGenerator::calcDy(unsigned int layer, unsigned int srcChan/*=RawCameraGenerator::CHAN_Y*/, unsigned int dstChan/*=RawCameraGenerator::CHAN_Y_DY*/) {
	unsigned char * s=getImage(layer,srcChan);
	unsigned char * dst=images[layer][dstChan];
	unsigned int inc=getIncrement(layer);
	unsigned int stride=getStride(layer);
	unsigned char * const dstEnd=dst+widths[layer]*inc;
	unsigned int sc=2;  // i think this should be 1, but this is to provide better compatability with the OPEN-R implementation
	while(dst!=dstEnd) {
		unsigned char * const colEnd=dst+heights[layer]*stride-stride;
		unsigned char top,bottom;
		while(dst!=colEnd) {
			top=(*s)>>sc;
			s+=stride;
			bottom=(*s)>>sc;
			*dst=bottom+128-top;
			dst+=stride;
		}
		*dst=128; //bottommost column is always 128 to retain image dimensions
		dst-=heights[layer]*stride-stride;
		s-=heights[layer]*stride-stride;
		dst+=inc;
		s+=inc;
	}
	imageValids[layer][dstChan]=true;
}
void BufferedImageGenerator::calcDxDy(unsigned int layer) {
	// if one of the dx or dy channel is already available, go from there
	if(imageValids[layer][RawCameraGenerator::CHAN_Y_DX]) {
		calcDy(layer,RawCameraGenerator::CHAN_Y_DX,RawCameraGenerator::CHAN_Y_DXDY);
		imageValids[layer][RawCameraGenerator::CHAN_Y_DXDY]=true;
	} else if(imageValids[layer][RawCameraGenerator::CHAN_Y_DY]) {
		calcDx(layer,RawCameraGenerator::CHAN_Y_DY,RawCameraGenerator::CHAN_Y_DXDY);
		imageValids[layer][RawCameraGenerator::CHAN_Y_DXDY]=true;
	} else {
		// if neither are available, calculate one of them (dx), and then the other from that
		getImage(layer,RawCameraGenerator::CHAN_Y_DX);
		calcDy(layer,RawCameraGenerator::CHAN_Y_DX,RawCameraGenerator::CHAN_Y_DXDY);
	}
}


/*! @file
 * @brief Implements BufferedImageGenerator, which receives camera frames as they are loaded by the simulator -- or eventually other sources
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
