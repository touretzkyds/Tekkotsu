#include "FilterBankGenerator.h"
#include "Events/FilterBankEvent.h"
#include "Events/EventRouter.h"
#include "Wireless/Socket.h"
#include "Shared/RobotInfo.h"
#include "Shared/ProjectInterface.h"

using namespace std;

unsigned int FilterBankGenerator::sysFrameNumber=-1U;

unsigned char *
FilterBankGenerator::getImage(unsigned int layer, unsigned int channel) {
	//for(int i=0;i<1000;i++)
	//std::cout << i << " FBG::getImage(" << layer<<'/'<<numLayers  << ',' << channel<<'/'<<numChannels << ");" << std::endl;
	if(!refresh())
		return NULL;
	if(!imageValids[layer][channel]) {
		if(images[layer][channel]==NULL)
			images[layer][channel]=createImageCache(layer,channel);
		calcImage(layer,channel);
	}
	return images[layer][channel];
}

void 
FilterBankGenerator::freeCaches() {
	invalidateCaches();
	for(unsigned int i=0; i<numLayers; i++)
		for(unsigned int j=0; j<numChannels; j++) {
			delete [] images[i][j];
			images[i][j]=NULL;
		}
}

void
FilterBankGenerator::invalidateCaches() {
	for(unsigned int i=0; i<numLayers; i++)
	  for(unsigned int j=0; j<numChannels; j++)
	    imageValids[i][j]=false;
}

void
FilterBankGenerator::doEvent() {
	// This code is overriden by BufferedImageGenerator, except in APERIOS
	if(event->getGeneratorID()==getListenGeneratorID() && event->getSourceID()==getListenSourceID()) {
		const FilterBankEvent& fbkevent=dynamic_cast<const FilterBankEvent& >(*event);
		if(fbkevent.getSource()==NULL) {
			serr->printf("%s %s received event with null source\n",getClassName().c_str(),getName().c_str());
			return;
		}
		src=fbkevent.getSource();
		refresh();
	} else {
		EventGeneratorBase::doEvent();
	}
}

unsigned int FilterBankGenerator::getBinSize() const {
	unsigned int used=0;
	used+=creatorSize("FbkImage");
	used+=sizeof(widths[selectedSaveLayer]);
	used+=sizeof(heights[selectedSaveLayer]);
	used+=sizeof(selectedSaveLayer);
	used+=sizeof(selectedSaveChannel);
	return used;
}

/*! The loadBuffer() functions of the included subclasses aren't tested, so don't assume they'll work without a little debugging... */
unsigned int FilterBankGenerator::loadBuffer(const char buf[], unsigned int len, const char* filename) {
	unsigned int origlen=len;
	if(!checkCreatorInc("FbkImage",buf,len,true)) return 0;
	if(!decodeInc(widths[selectedSaveLayer],buf,len)) return 0;
	if(!decodeInc(heights[selectedSaveLayer],buf,len)) return 0;
	if(!decodeInc(selectedSaveLayer,buf,len)) return 0;
	if(!decodeInc(selectedSaveChannel,buf,len)) return 0;
	return origlen-len;	
}

unsigned int FilterBankGenerator::saveBuffer(char buf[], unsigned int len) const {
	unsigned int origlen=len;
	if(!saveCreatorInc("FbkImage",buf,len)) return 0;
	if(!encodeInc(widths[selectedSaveLayer],buf,len)) return 0;
	if(!encodeInc(heights[selectedSaveLayer],buf,len)) return 0;
	if(!encodeInc(selectedSaveLayer,buf,len)) return 0;
	if(!encodeInc(selectedSaveChannel,buf,len)) return 0;
	return origlen-len;
}

void
FilterBankGenerator::setNumImages(unsigned int nLayers, unsigned int nChannels) {
	if(nLayers==numLayers && nChannels==numChannels)
		return;
	freeCaches();
	destruct();
	numLayers=nLayers;
	numChannels=nChannels;
	
	widths=new unsigned int[numLayers];
	heights=new unsigned int[numLayers];
	if(numLayers>0) {
		for(unsigned int res=0; res<numLayers-1; res++) {
			widths[res] = CameraResolutionX>>(numLayers-2-res);
			heights[res] = CameraResolutionY>>(numLayers-2-res);
		}
		widths[numLayers-1]=CameraResolutionX*2;
		heights[numLayers-1]=CameraResolutionY*2;
	}

	skips=new unsigned int[numLayers];
	strides=new unsigned int[numLayers];
	increments=new unsigned int[numLayers];
	images=new unsigned char**[numLayers];
	imageValids=new bool*[numLayers];
	for(unsigned int i=0; i<numLayers; i++) {
		skips[i]=strides[i]=0;
		increments[i]=1;
		images[i]=new unsigned char*[numChannels];
		imageValids[i]=new bool[numChannels];
		for(unsigned int j=0; j<numChannels; j++) {
			images[i][j]=NULL;
			imageValids[i][j]=false;
		}
	}
}

void
FilterBankGenerator::setDimensions() {
	if(src==NULL)
		return;
	for(unsigned int i=0; i<numLayers; i++) {
		widths[i]=src->getWidth(i);
		heights[i]=src->getHeight(i);
	}
	freeCaches();
}

void
FilterBankGenerator::destruct() {
	delete [] widths;
	widths=NULL;
	delete [] heights;
	heights=NULL;
	delete [] skips;
	skips=NULL;
	delete [] strides;
	strides=NULL;
	delete [] increments;
	increments=NULL;
	for(unsigned int i=0; i<numLayers; i++) {
		delete [] images[i];
		delete [] imageValids[i];
	}
	delete [] images;
	images=NULL;
	delete [] imageValids;
	imageValids=NULL;
	numLayers=numChannels=0;
}

bool FilterBankGenerator::refresh() {
	if(sysFrameNumber==-1U) {
		serr->printf("ERROR: attempted access to camera image before any data was available\n");
		return false;
	}
	if(frameNumber==sysFrameNumber)
		return true;
	if(src==NULL) {
		// BufferedImageGenerator leaves src at 0.  The frameNumber might not match
		// sysFrameNumber if we have depth images and camera images arriving asynchronously.
		if ( frameNumber > 0 )
			return true; // still have good data
		serr->printf("ERROR: attempted access to camera image before stage had a known source\n");
		return false;
	}
	if(!src->refresh())
		return false;
	if(frameNumber==src->getFrameNumber())
		return true;
	frameNumber=src->getFrameNumber();
	setNumImages(src->getNumLayers(),src->getNumChannels());
	if(numLayers<=0 || numChannels<=0) {
		serr->printf("ERROR: attempted to access image in empty FilterBankGenerator\n");
		return false;
	}
	if(src->getWidth(numLayers-1)!=getWidth(numLayers-1) || src->getHeight(numLayers-1)!=getHeight(numLayers-1)) {
		setDimensions();
		if(framesProcessed==0)
			serr->printf("WARNING: %s image dimensions don't match values predicted by RobotInfo consts, \"full\" layer now %dx%d\n",getName().c_str(),widths[ProjectInterface::fullLayer],heights[ProjectInterface::fullLayer]);
		else
			serr->printf("WARNING: %s image dimensions have changed since last frame, \"full\" layer now %dx%d\n",getName().c_str(),widths[ProjectInterface::fullLayer],heights[ProjectInterface::fullLayer]);
	} else if(framesProcessed==0) {
		setDimensions();
	}
	invalidateCaches();
	framesProcessed++;
	return true;
}


/*! @file
 * @brief Implements abstract base class for generators of FilterBankEvent's
 * @author ejt (Creator)
 */

