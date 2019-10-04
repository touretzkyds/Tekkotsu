#include "RLEGenerator.h"
#include "Events/EventRouter.h"
#include "Events/SegmentedColorFilterBankEvent.h"
#include "Wireless/Wireless.h"
#include "Shared/Profiler.h"

#include "Shared/debuget.h"

/*! TODO - after RLEGraphics is inplace, replace the 'tid' parameter in FBG's constructor call! */
RLEGenerator::RLEGenerator(unsigned int mysid, FilterBankGenerator* fbg, EventBase::EventTypeID_t /*tid*/)
	: FilterBankGenerator("RLEGenerator",EventBase::visRLEEGID,mysid,fbg), numRuns(NULL), maxRuns(NULL)
{
	//this part is only necessary if you override setNumImages yourself
	if(fbg!=NULL) {
		numLayers=numChannels=0; //this is to force setNumImages to override settings provided by FilterBankGenerator
		setNumImages(fbg->getNumLayers(),fbg->getNumChannels());
	}
}

void
RLEGenerator::doEvent() {
	if(event->getGeneratorID()==getListenGeneratorID() && event->getSourceID()==getListenSourceID()) {
		//Technically, we should do things this way:
		/*
		if(const SegmentedColorFilterBankEvent * segsrc=dynamic_cast<const SegmentedColorFilterBankEvent *>(event)) {
			SegmentedColorFilterBankEvent segev(this,getGeneratorID(),getSourceID(),EventBase::activateETID,*segsrc);
			erouter->postEvent(fbkev);
			segev.setTypeID(EventBase::statusETID);
			erouter->postEvent(fbkev);
			segev.setTypeID(EventBase::deactivateETID);
			erouter->postEvent(fbkev);
		} else {
			FilterBankEvent fbkev(this,getGeneratorID(),getSourceID(),EventBase::activateETID);
			erouter->postEvent(fbkev);
			fbkev.setTypeID(EventBase::statusETID);
			erouter->postEvent(fbkev);
			fbkev.setTypeID(EventBase::deactivateETID);
			erouter->postEvent(fbkev);
		}
		*/
		//But until RLEGraphics is in place, we'll do it this way so recompression can be triggered
		//after drawing into segmented image
		if(const SegmentedColorFilterBankEvent * segsrc=dynamic_cast<const SegmentedColorFilterBankEvent *>(event)) {
			SegmentedColorFilterBankEvent segev(this,getGeneratorID(),getSourceID(),event->getTypeID(),*segsrc);
			erouter->postEvent(segev);
		} else {
			FilterBankEvent fbkev(this,getGeneratorID(),getSourceID(),event->getTypeID());
			erouter->postEvent(fbkev);
		}
	}
}

unsigned int
RLEGenerator::getBinSize() const {
	unsigned int used=FilterBankGenerator::getBinSize();
	used+=strlen("RLEImage")+LoadSave::stringpad;
	used+=sizeof(unsigned int);
	if(imageValids[selectedSaveLayer][selectedSaveChannel])
		used+=XMIT_BYTES_PER_RUN*numRuns[selectedSaveLayer][selectedSaveChannel];
	else
		used+=XMIT_BYTES_PER_RUN*maxRuns[selectedSaveLayer];
	return used;
}

/*! this isn't really tested, don't rely on it working without a little debugging... specifically, doesn't set parent or next fields*/
unsigned int
RLEGenerator::loadBuffer(const char buf[], unsigned int len, const char* filename) {
	unsigned int origlen=len;
	std::string tmp;
	if(!checkInc(FilterBankGenerator::loadBuffer(buf,len,filename),buf,len)) return 0;
	if(!decodeInc(tmp,buf,len)) return 0;
	if(tmp!="RLEImage") {
		serr->printf("Unhandled image type for RLEGenerator: %s",tmp.c_str());
		return 0;
	} else {
		if(!decodeInc(numRuns[selectedSaveLayer][selectedSaveChannel],buf,len)) return 0;
		if(maxRuns[selectedSaveLayer]<numRuns[selectedSaveLayer][selectedSaveChannel])
			return 0;
		if(images[selectedSaveLayer][selectedSaveChannel]==NULL)
			images[selectedSaveLayer][selectedSaveChannel]=createImageCache(selectedSaveLayer,selectedSaveChannel);
		run * runs=reinterpret_cast<run*>(images[selectedSaveLayer][selectedSaveChannel]);
		if(runs==NULL)
			return 0;
		unsigned int y=0;
		for(unsigned int i=0; i<numRuns[selectedSaveLayer][selectedSaveChannel]; i++) {
			if(!decodeInc(runs[i].color,buf,len)) return 0;
			if(!decodeInc(runs[i].x,buf,len)) return 0;
			if(!decodeInc(runs[i].width,buf,len)) return 0;
			if((unsigned int)(runs[i].x+runs[i].width)>=getWidth(selectedSaveLayer))
				y++;
			runs[i].y=y;
		}
		imageValids[selectedSaveLayer][selectedSaveChannel]=true;
		return origlen-len;	
	}
}

unsigned int
RLEGenerator::saveBuffer(char buf[], unsigned int len) const {
	unsigned int origlen=len;
	if(!checkInc(FilterBankGenerator::saveBuffer(buf,len),buf,len)) return 0;
	if(!encodeInc("RLEImage",buf,len)) return 0;
	
	if(images[selectedSaveLayer][selectedSaveChannel]==NULL) {
		serr->printf("RLEGenerator::saveBuffer() failed because selected image is NULL -- call selectSaveImage first to make sure it's up to date\n");
		return 0;
	}
	if(!imageValids[selectedSaveLayer][selectedSaveChannel]) {
		serr->printf("RLEGenerator::saveBuffer() failed because selected image is invalid -- call selectSaveImage first to make sure it's up to date\n");
		return 0;
	}
	unsigned char* img=images[selectedSaveLayer][selectedSaveChannel];
	run * runs=reinterpret_cast<run*>(img);
	if(runs==NULL)
		return 0;
	if(!encodeInc(numRuns[selectedSaveLayer][selectedSaveChannel],buf,len)) return 0;
	for(unsigned int i=0; i<numRuns[selectedSaveLayer][selectedSaveChannel]; i++) {
		if(!encodeInc(runs[i].color,buf,len)) return 0;
		if(!encodeInc(runs[i].x,buf,len)) return 0;
		if(!encodeInc(runs[i].width,buf,len)) return 0;
	}
	return origlen-len;
}

void
RLEGenerator::setDimensions() {
	FilterBankGenerator::setDimensions();
	for(unsigned int i=0; i<numLayers; i++)
		maxRuns[i]=calcExpMaxRuns(i);
}

void
RLEGenerator::setNumImages(unsigned int nLayers, unsigned int nChannels) {
	if(nLayers==numLayers && nChannels==numChannels)
		return;
	FilterBankGenerator::setNumImages(nLayers,nChannels); //calls destruct()...
	maxRuns=new unsigned int[numLayers];
	numRuns=new unsigned int*[numLayers];
	for(unsigned int i=0; i<numLayers; i++) {
		maxRuns[i]=calcExpMaxRuns(i);
		numRuns[i]=new unsigned int[numChannels];
	}
}

//! simply creates a new data region and returns it
unsigned char *
RLEGenerator::createImageCache(unsigned int layer, unsigned int /*chan*/) const {
	return reinterpret_cast<unsigned char*>(new run[maxRuns[layer]]);
}

//! a single call to the CMVision library to do the work, and we're done.
void
RLEGenerator::calcImage(unsigned int layer, unsigned int chan) {
	PROFSECTION("RLEGenerator::calcImage(...)",*mainProfiler);
  numRuns[layer][chan] = CMVision::EncodeRuns(reinterpret_cast<run*>(images[layer][chan]),src->getImage(layer,chan),getWidth(layer),getHeight(layer),maxRuns[layer]);
	imageValids[layer][chan]=true; // <--- don't forget to do this, otherwise you'll recompute on every access, even if the cache is still valid
}

void
RLEGenerator::destruct() {
	FilterBankGenerator::destruct();
	for(unsigned int i=0; i<numLayers; i++)
		delete [] numRuns[i];
	delete [] numRuns;
	numRuns=NULL;
	delete [] maxRuns;
	maxRuns=NULL;
}

/*! @file
 * @brief Implements RLEGenerator, which generates RLE compressed FilterBankEvents (generally from indexed color images from, say, SegmentedColorGenerator)
 * @author alokl (Creator)
 * @author ejt (reorganized)
 */

