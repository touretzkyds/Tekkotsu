#include "CDTGenerator.h"
#include "Events/DataEvent.h"
#include "Events/EventRouter.h"
#include "Events/SegmentedColorFilterBankEvent.h"
#include "Wireless/Wireless.h"
#include "Shared/Config.h"
#include "Shared/Profiler.h"
#include "Shared/ProjectInterface.h"

#include "Shared/ODataFormats.h"
#include "OFbkImage.h"

#include "Shared/debuget.h"

CDTGenerator::CDTGenerator(unsigned int numRawLayers, unsigned int numCalcLayers, unsigned int mysid, EventBase::EventGeneratorID_t gid, unsigned int sid)
	: FilterBankGenerator("CDTGenerator",EventBase::visSegmentEGID,mysid,gid,sid), numRealLayers(numRawLayers), layers(NULL), imageInfos(NULL)
{
	/* As a root stage, we need to listen to all incoming image
	 * events, even if we don't currently have listeners of our own --
	 * This is just in case user code directly accesses a generator
	 * and we have to retroactively go back and dig up the previous
	 * frame */
	unsetAutoListen();
	
	setNumImages(numCalcLayers,NUM_CHANNELS);
}

/*! The const casts in this function are regretable but necessary
 *  since the OPEN-R OFbkImage constructor requires mutable
 *  arguments, even though it shouldn't be modifying the data
 */
void
CDTGenerator::doEvent() {
	if(event->getGeneratorID()!=getListenGeneratorID() || event->getSourceID()!=getListenSourceID())
		return;
	if(event->getTypeID()==EventBase::activateETID) {
		typedef DataEvent<const OFbkImageVectorData*> OFbkEvent;
		const OFbkEvent& fbkevent=dynamic_cast<const OFbkEvent& >(*event);
		OFbkImageVectorData& fbkdat=*const_cast<OFbkImageVectorData*>(fbkevent.getData());
		for(unsigned int res=0; res<numRealLayers; res++) {
			layers[numLayers-1-res] = fbkdat.GetData(res);
			imageInfos[numLayers-1-res] = fbkdat.GetInfo(res);
		}
		{
			const OFbkImage img(const_cast<OFbkImageInfo*>(imageInfos[numLayers-2]), const_cast<unsigned char*>(layers[numLayers-2]), ofbkimageBAND_CDT);
			//I have to do this crazy thing because apparently img.FieldCounter() doesn't work
			sysFrameNumber=frameNumber=*(int*)(img.Pointer()+(img.Height()-1)*(img.Skip()+img.Width()));
		}
		unsigned int numNotRealLayers=numLayers-numRealLayers;
		bool dimchange=false;
		for(unsigned int res=numNotRealLayers; res<numLayers; res++) {
			if(widths[res]!=imageInfos[res]->width || heights[res]!=imageInfos[res]->height) {
				dimchange=true;
				serr->printf("WARNING: the image dimensions changed, now %dx%d\n",widths[numLayers-1],heights[numLayers-1]);
				widths[res] = imageInfos[res]->width;
				heights[res] = imageInfos[res]->height;
			}

			const OFbkImage img(const_cast<OFbkImageInfo*>(imageInfos[res]), const_cast<unsigned char*>(layers[res]), ofbkimageBAND_CDT);
			skips[res]=img.Skip();
			strides[res]=skips[res]+widths[res];

			ASSERT(static_cast<unsigned int>(img.Width())==getWidth(res),"Widths don't match");
			ASSERT(static_cast<unsigned int>(img.Height())==getHeight(res),"Heights don't match");
		}
		if(numNotRealLayers>0) {
			if(widths[numNotRealLayers-1]*2!=widths[numNotRealLayers] || heights[numNotRealLayers]*2!=heights[numNotRealLayers]) {
				//|| widths[numLayers-2-numRealLayers]*2!=widths[numNotRealLayers]
				//|| heights[numLayers-2-numRealLayers]*2!=heights[numNotRealLayers]) {
				//set the width and height of non-real layers (since they don't match what they should be)
				serr->printf("WARNING: the image dimensions don't match values predicted by RobotInfo consts, \"full\" layer now %dx%d\n",widths[ProjectInterface::fullLayer],heights[ProjectInterface::fullLayer]);
				dimchange=true;
			} else if(strides[0]==0) {
				dimchange=true;
			}
		}
		if(dimchange)
			setDimensions();

		invalidateCaches();
		framesProcessed++;
	}
	// todo - i wish we had some color infomation to pass here so we could use the event for CMVision's RLE, etc.
	erouter->postEvent(SegmentedColorFilterBankEvent(this,getGeneratorID(),getSourceID(),event->getTypeID(),NULL,0,NULL,NULL));
}

unsigned int
CDTGenerator::getBinSize() const {
	unsigned int used=FilterBankGenerator::getBinSize();
	// todo - once we have color information - we could make this interoperable with SegmentedColorGenerator
	// by following the same serialization format
	used+=strlen("CDTImage")+LoadSave::stringpad;
	used+=widths[selectedSaveLayer]*heights[selectedSaveLayer];
	return used;
}

unsigned int
CDTGenerator::loadBuffer(const char [] /*buf*/, unsigned int /*len*/, const char* filename) {
	//all our memory is in system controlled buffers - we probably shouldn't overwrite it...
	serr->printf("Can't load into CDTGenerator: %s",filename);
	return 0;
}

unsigned int
CDTGenerator::saveBuffer(char buf[], unsigned int len) const {
	unsigned int origlen=len;
	unsigned int used;
	if(0==(used=FilterBankGenerator::saveBuffer(buf,len))) return 0;
	len-=used; buf+=used;
	// todo - once we have color information - we could make this interoperable with SegmentedColorGenerator
	// by following the same serialization format
	if(0==(used=encode("CDTImage",buf,len))) return 0;
	len-=used; buf+=used;
	
	if(images[selectedSaveLayer][selectedSaveChannel]==NULL) {
		serr->printf("CDTImage::saveBuffer() failed because selected image is NULL -- call selectSaveImage first to make sure it's up to date\n");
		return 0;
	}
	if(!imageValids[selectedSaveLayer][selectedSaveChannel]) {
		serr->printf("CDTImage::saveBuffer() failed because selected image is invalid -- call selectSaveImage first to make sure it's up to date\n");
		return 0;
	}
	unsigned char* img=images[selectedSaveLayer][selectedSaveChannel];
	used=widths[selectedSaveLayer]*heights[selectedSaveLayer];
	if(used>len)
		return 0;
	unsigned int inc=getIncrement(selectedSaveLayer);
	if(inc==1) {
		//special case, straight copy
		for(unsigned int y=0; y<heights[selectedSaveLayer]; y++) {
			unsigned char* const rowend=img+widths[selectedSaveLayer];
			while(img!=rowend)
				*buf++=*img++;
			img+=getSkip(selectedSaveLayer);
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

void
CDTGenerator::setDimensions() {
	freeCaches();
	unsigned int numNotRealLayers=numLayers-numRealLayers;
	for(unsigned int res=0; res<numNotRealLayers; res++) {
		widths[res] = imageInfos[numNotRealLayers]->width>>(numNotRealLayers-res);
		heights[res] = imageInfos[numNotRealLayers]->height>>(numNotRealLayers-res);
		ASSERT(widths[res]*increments[res]==widths[numNotRealLayers],"widths*increments doesn't match total width");
		strides[res]=strides[numNotRealLayers]*increments[res];
		skips[res]=skips[numNotRealLayers]+strides[res]-strides[numNotRealLayers];
	}
	strides[numLayers-1]=widths[numLayers-1]=widths[numLayers-2]*2;
	heights[numLayers-1]=heights[numLayers-2]*2;
}

void 
CDTGenerator::freeCaches() {
	for(unsigned int i=0; i<numLayers; i++) {
		for(unsigned int j=0; j<numChannels; j++) {
			images[i][j]=NULL;
			imageValids[i][j]=false;
		}
	}
	FilterBankGenerator::freeCaches();
}

void
CDTGenerator::setNumImages(unsigned int nLayers, unsigned int nChannels) {
	if(nLayers==numLayers && nChannels==numChannels)
		return;
	FilterBankGenerator::setNumImages(nLayers,nChannels);
	layers=new unsigned char*[numLayers];
	imageInfos=new const OFbkImageInfo*[numLayers];
	unsigned int numNotRealLayers=numLayers-numRealLayers;
	for(unsigned int res=0; res<numLayers; res++) {
		layers[res]=NULL;
		imageInfos[res]=NULL;
		if(res<numNotRealLayers)
			increments[res]=1<<(numNotRealLayers-res);
	}
}

unsigned char *
CDTGenerator::createImageCache(unsigned int /*layer*/, unsigned int /*chan*/) const {
	return NULL; // calcImage will set the cache itself
}

void
CDTGenerator::calcImage(unsigned int layer, unsigned int chan) {
	PROFSECTION("CDTGenerator::calcImage(...)",*mainProfiler);
	unsigned int numNotRealLayers=numLayers-numRealLayers;
	if(layer>=numNotRealLayers) {
		unsigned int fbkdatChan=ofbkimageBAND_CDT;
		const OFbkImage img(const_cast<OFbkImageInfo*>(imageInfos[layer]), const_cast<unsigned char*>(layers[layer]), fbkdatChan);
		images[layer][chan]=img.Pointer();

		//I'm not sure if this is needed for CDT images themselves - haven't used it yet
		//so this section is commented out for now just in case.
		/*
			//this part restores pixels over written with the CDT table and
			//frame count.  Yes, we are modifying the original image passed
			//from the system here...
			if(config->vision.restore_image) {
				const unsigned int numPix=16;
				if(layer==numLayers-2) {
					unsigned char * s=images[layer][chan]+getStride(layer)*(getHeight(layer)-2);
					unsigned char * d=images[layer][chan]+getStride(layer)*(getHeight(layer)-1);
					for(unsigned int i=0; i<numPix; i++)
						*d++=*s++;
				} else {
					unsigned int inc=1<<(numLayers-2-layer);
					unsigned char * s;
					//unsigned char * s=getImage(numLayers-2,chan)+getStride(numLayers-2)*(getHeight(numLayers-2)-inc);
					//...or an attempt to possibly avoid a trivial amount of recomputation....
					if(!imageValids[numLayers-2][chan]) {
						const OFbkImage simg(const_cast<OFbkImageInfo*>(imageInfos[numLayers-2]), const_cast<unsigned char*>(layers[numLayers-2]), fbkdatChan);
						s=simg.Pointer();
					} else {
						s=images[numLayers-2][chan];
					}
					s+=getStride(numLayers-2)*(getHeight(numLayers-2)-inc);
					unsigned char * d=images[layer][chan]+getStride(layer)*(getHeight(layer)-1);
					for(unsigned int i=0; i<numPix; i++) {
						*d++=*s;
						s+=inc;
					}
				}
			}
		*/
	} else {
		//we don't need to do the restoration in the previous section
		//here because these layers skip the last row
		unsigned int fbkdatChan=ofbkimageBAND_CDT;
		const OFbkImage img(const_cast<OFbkImageInfo*>(imageInfos[numNotRealLayers]), const_cast<unsigned char*>(layers[numNotRealLayers]), fbkdatChan);
		images[layer][chan]=img.Pointer();
	}
	imageValids[layer][chan]=true;
}

void
CDTGenerator::destruct() {
	FilterBankGenerator::destruct();
	delete [] layers;
	layers=NULL;
	delete [] imageInfos;
	imageInfos=NULL;
}

/*! @file
 * @brief Implements CDTGenerator, which generates SegmentedColorFilterBankEvents with images provided from the system
 * @author ejt (Creator)
 */

