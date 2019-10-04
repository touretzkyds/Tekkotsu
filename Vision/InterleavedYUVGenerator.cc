#include "InterleavedYUVGenerator.h"
#include "Events/EventRouter.h"
#include "Events/FilterBankEvent.h"
#include "Wireless/Wireless.h"
#include "Shared/Profiler.h"

#include "Shared/debuget.h"

InterleavedYUVGenerator::InterleavedYUVGenerator(unsigned int mysid,FilterBankGenerator* fbg, EventBase::EventTypeID_t tid)
	: FilterBankGenerator("InterleavedYUVGenerator",EventBase::visInterleaveEGID,mysid,fbg,tid), srcYChan(0), srcUChan(1), srcVChan(2), isAllocated(NULL)
{
	//this part is only necessary if you override setNumImages yourself
	if(fbg!=NULL) {
		numLayers=numChannels=0; //this is to force setNumImages to override settings provided by FilterBankGenerator
		setNumImages(fbg->getNumLayers(),fbg->getNumChannels()); //channels gets overridden to '1' in setNumImages
	}
}

InterleavedYUVGenerator::InterleavedYUVGenerator(unsigned int mysid, unsigned int syc, unsigned int suc, unsigned int svc,FilterBankGenerator* fbg, EventBase::EventTypeID_t tid)
	: FilterBankGenerator("InterleavedYUVGenerator",EventBase::visInterleaveEGID,mysid,fbg,tid), srcYChan(syc), srcUChan(suc), srcVChan(svc), isAllocated(NULL)
{
	//this part is only necessary if you override setNumImages yourself
	if(fbg!=NULL) {
		numLayers=numChannels=0; //this is to force setNumImages to override settings provided by FilterBankGenerator
		setNumImages(fbg->getNumLayers(),fbg->getNumChannels()); //channels gets overridden to '1' in setNumImages
	}
}

void
InterleavedYUVGenerator::doEvent() {
	if(event->getGeneratorID()==getListenGeneratorID() && event->getSourceID()==getListenSourceID()) {
		FilterBankEvent fbkev(this,getGeneratorID(),getSourceID(),EventBase::activateETID);
		erouter->postEvent(fbkev);
		fbkev.setTypeID(EventBase::statusETID);
		erouter->postEvent(fbkev);
		fbkev.setTypeID(EventBase::deactivateETID);
		erouter->postEvent(fbkev);
	}
}

unsigned int
InterleavedYUVGenerator::getBinSize() const {
	unsigned int used=FilterBankGenerator::getBinSize();
	used+=strlen("InterleavedYUVImage")+LoadSave::stringpad;
	used+=widths[selectedSaveLayer]*heights[selectedSaveLayer]*3;
	return used;
}

unsigned int
InterleavedYUVGenerator::loadBuffer(const char buf[], unsigned int len, const char* filename) {
	unsigned int origlen=len;
	if(!checkInc(FilterBankGenerator::loadBuffer(buf,len,filename),buf,len)) return 0;
	std::string tmp;
	if(!decodeInc(tmp,buf,len)) return 0;
	if(tmp!="InterleavedYUVImage") {
		serr->printf("Unhandled image type for InterleavedYUVGenerator: %s",tmp.c_str());
		return 0;
	} else {
		unsigned int used=widths[selectedSaveLayer]*heights[selectedSaveLayer]*3;
		if(used>len)
			return 0;
		if(images[selectedSaveLayer][selectedSaveChannel]==NULL)
			images[selectedSaveLayer][selectedSaveChannel]=createImageCache(selectedSaveLayer,selectedSaveChannel);
		unsigned char* img=images[selectedSaveLayer][selectedSaveChannel];
		if(img==NULL)
			return 0;
		memcpy(img,buf,used);
		len-=used; buf+=used;
		imageValids[selectedSaveLayer][selectedSaveChannel]=true;
		return origlen-len;	
	}
}

unsigned int
InterleavedYUVGenerator::saveBuffer(char buf[], unsigned int len) const {
	unsigned int origlen=len;
	if(!checkInc(FilterBankGenerator::saveBuffer(buf,len),buf,len)) return 0;
	if(!encodeInc("InterleavedYUVImage",buf,len)) return 0;
	
	unsigned int used=widths[selectedSaveLayer]*heights[selectedSaveLayer]*3;
	if(used>len)
		return 0;
	if(images[selectedSaveLayer][selectedSaveChannel]==NULL) {
		serr->printf("InterleavedYUVGenerator::saveBuffer() failed because selected image is NULL -- call selectSaveImage first to make sure it's up to date\n");
		return 0;
	}
	if(!imageValids[selectedSaveLayer][selectedSaveChannel]) {
		serr->printf("InterleavedYUVGenerator::saveBuffer() failed because selected image is invalid -- call selectSaveImage first to make sure it's up to date\n");
		return 0;
	}
	unsigned char* img=images[selectedSaveLayer][selectedSaveChannel];
	if(img==NULL)
		return 0;
	memcpy(buf,img,used);
	len-=used;
	return origlen-len;
}

void
InterleavedYUVGenerator::setDimensions() {
	FilterBankGenerator::setDimensions();
	for(unsigned int i=0; i<numLayers; i++)
		strides[i]=widths[i]*3;
}

void
InterleavedYUVGenerator::destruct() {
	FilterBankGenerator::destruct();
	for(unsigned int i=0; i<numLayers; i++)
		delete [] isAllocated[i];
	delete [] isAllocated;
	isAllocated=NULL;
}

void
InterleavedYUVGenerator::setNumImages(unsigned int nLayers, unsigned int /*nChannels*/) {
	if(nLayers==numLayers && 1==numChannels) // this generator only has 1 channel
		return;
	FilterBankGenerator::setNumImages(nLayers,1);
	isAllocated=new bool*[numLayers];
	for(unsigned int res=0; res<numLayers; res++) {
		increments[res]=3;
		isAllocated[res]=new bool[numChannels];
		for(unsigned int c=0; c<numChannels; c++)
			isAllocated[res][c]=false;
	}
}

void
InterleavedYUVGenerator::freeCaches() {
	FilterBankGenerator::freeCaches();
	for(unsigned int i=0; i<numLayers; i++)
		for(unsigned int j=0; j<numChannels; j++)
			isAllocated[i][j]=false;
}

void
InterleavedYUVGenerator::invalidateCaches() {
	for(unsigned int i=0; i<numLayers; i++)
		for(unsigned int j=0; j<numChannels; j++) {
			if(!isAllocated[i][j])
				images[i][j]=NULL;
			imageValids[i][j]=false;
		}
}

unsigned char *
InterleavedYUVGenerator::createImageCache(unsigned int layer, unsigned int chan) const {
	if(src->getStride(layer)==getStride(layer) && src->getIncrement(layer)==getIncrement(layer)) {
		//already interleaved, pass through
		ASSERT(!isAllocated[layer][chan],"pass through over allocated image!")
		imageValids[layer][chan]=true;
		return src->getImage(layer,srcYChan);
	} else {
		isAllocated[layer][chan]=true;
		return new unsigned char[widths[layer]*heights[layer]*3];
	}
}

void
InterleavedYUVGenerator::calcImage(unsigned int layer, unsigned int chan) {
	PROFSECTION("InterleavedYUVGenerator::calcImage(...)",*mainProfiler);
	if(imageValids[layer][chan]) //check if createCache set valid flag
		return; //indicates pass through from previous stage
	
	unsigned char* dimg=images[layer][chan];
	const unsigned char* syimg=src->getImage(layer,srcYChan);
	const unsigned char* suimg=src->getImage(layer,srcUChan);
	const unsigned char* svimg=src->getImage(layer,srcVChan);
	const unsigned int inc=src->getIncrement(layer);
	const unsigned int skip=src->getSkip(layer);
	//std::cout << src->getWidth(layer) << " inc=" << inc << " skip=" << src->getSkip(layer) << " stride=" << src->getStride(layer) << std::endl;
	for(unsigned int y=0; y<getHeight(layer); y++) {
		for(unsigned int x=0; x<getWidth(layer); x++) {
			*dimg++=*syimg;
			*dimg++=*suimg;
			*dimg++=*svimg;
			syimg+=inc;
			suimg+=inc;
			svimg+=inc;
		}
		syimg+=skip;
		suimg+=skip;
		svimg+=skip;
	}
	imageValids[layer][chan]=true;
}

/*! @file
 * @brief Implements InterleavedYUVGenerator, which generates FilterBankEvents containing raw camera images with interleaved pixels (YUVYUVYUV... instead of YYY...UUU...VVV...)
 * @author ejt (Creator)
 */

