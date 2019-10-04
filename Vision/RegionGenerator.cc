#include "RegionGenerator.h"
#include "Events/EventRouter.h"
#include "Wireless/Wireless.h"
#include "Shared/Profiler.h"
#include "SegmentedColorGenerator.h"

#include "Vision/RLEGenerator.h"

#include "Shared/debuget.h"

RegionGenerator::RegionGenerator(unsigned int mysid, RLEGenerator* rleg, EventBase::EventTypeID_t tid)
	: FilterBankGenerator("RegionGenerator",EventBase::visRegionEGID,mysid,rleg,tid), srcNumColors(0), srcColors(NULL), regions(NULL)
{
	if(rleg!=NULL) {
		numLayers=numChannels=0; //this is to force setNumImages to override settings provided by FilterBankGenerator
		setNumImages(rleg->getNumLayers(),rleg->getNumChannels());
		if(const SegmentedColorGenerator* seg=dynamic_cast<const SegmentedColorGenerator*>(rleg->getSourceGenerator())) {
			srcNumColors=seg->getNumColors();
			srcColors=seg->getColors();
		} else {
			serr->printf("WARNING: Region generator expects it's source's source to be a SegmentedColorGenerator so that it can access color table information.\n");
		}
	}
}

void
RegionGenerator::freeCaches() {
	FilterBankGenerator::freeCaches();
	for(unsigned int l=0; l<numLayers; l++)
		for(unsigned int c=0; c<numChannels; c++) {
			delete [] regions[l][c];
			regions[l][c]=NULL;
		}
}

void
RegionGenerator::doEvent() {
	if(event->getGeneratorID()==getListenGeneratorID() && event->getSourceID()==getListenSourceID()) {
		const RLEGenerator * rle=dynamic_cast<const RLEGenerator*>(src);
		if(NULL==rle) {
			serr->printf("RegionGenerator's event %s is not from RLEGenerator\n",event->getName().c_str());
			return;
		}
		const SegmentedColorFilterBankEvent * segev=dynamic_cast<const SegmentedColorFilterBankEvent*>(event);
		if(NULL==segev) {
			serr->printf("RegionGenerator's event %s is not SegmentedColorFilterBankEvent\n",event->getName().c_str());
			return;
		}
		if(srcNumColors!=segev->getNumColors())
			freeCaches();
		srcNumColors=segev->getNumColors();
		srcColors=segev->getColors();
		SegmentedColorFilterBankEvent fbkev(this,getGeneratorID(),getSourceID(),EventBase::activateETID,*segev);
		erouter->postEvent(fbkev);
		fbkev.setTypeID(EventBase::statusETID);
		erouter->postEvent(fbkev);
		fbkev.setTypeID(EventBase::deactivateETID);
		erouter->postEvent(fbkev);
	}
}

unsigned int
RegionGenerator::getBinSize() const {
	unsigned int used=FilterBankGenerator::getBinSize();
	used+=strlen("RegionImage")+LoadSave::stringpad;
	used+=sizeof(unsigned int); //srcNumColors
	used+=sizeof(unsigned int)*srcNumColors; //stats[i].num (for each color i)
	unsigned int xmit_bytes_per_run=5*sizeof(int)+2*sizeof(float)+2*sizeof(int);
	if(imageValids[selectedSaveLayer][selectedSaveChannel]) {
		unsigned char* img=images[selectedSaveLayer][selectedSaveChannel];
		region_stats * stats=reinterpret_cast<region_stats*>(img);
		if(stats==NULL)
			return 0;
		for(unsigned int i=0; i<srcNumColors; i++)
			used+=xmit_bytes_per_run*stats[i].num;
	} else {
		used+=xmit_bytes_per_run*MAX_REGIONS;
	}
	used+=sizeof(unsigned int)*srcNumColors; //stats[i].min_area (for each color i)
	used+=sizeof(unsigned int)*srcNumColors; //stats[i].total_area (for each color i)
	used+=sizeof(float)*srcNumColors; //stats[i].merge_threshold (for each color i)
	return used;
}

unsigned int
RegionGenerator::loadBuffer(const char buf[], unsigned int len, const char* filename) {
	unsigned int origlen=len;
	if(!checkInc(FilterBankGenerator::loadBuffer(buf,len,filename),buf,len)) return 0;
	std::string tmp;
	if(!decodeInc(tmp,buf,len)) return 0;
	if(tmp!="RegionImage") {
		serr->printf("Unhandled image type for RegionGenerator: %s",tmp.c_str());
		return 0;
	} else {
		unsigned int tmpNumClr=0;
		if(!decodeInc(tmpNumClr,buf,len)) return 0;
		if(tmpNumClr!=srcNumColors)
			freeCaches();
		srcNumColors=tmpNumClr;
		if(images[selectedSaveLayer][selectedSaveChannel]==NULL)
			images[selectedSaveLayer][selectedSaveChannel]=createImageCache(selectedSaveLayer,selectedSaveChannel);
		region_stats * stats=reinterpret_cast<region_stats*>(images[selectedSaveLayer][selectedSaveChannel]);
		if(stats==NULL)
			return 0;
		for(unsigned int i=0; i<srcNumColors; i++) {
			unsigned int tmpNumReg=0;
			if(!decodeInc(tmpNumReg,buf,len)) return 0;
			if(MAX_REGIONS<=tmpNumReg)
				return 0;
			for(unsigned int j=0; j<tmpNumReg; j++) {
				region * curreg=&regions[selectedSaveLayer][selectedSaveChannel][j];
				if(!decodeInc(curreg->color,buf,len)) return 0;
				if(!decodeInc(curreg->x1,buf,len)) return 0;
				if(!decodeInc(curreg->y1,buf,len)) return 0;
				if(!decodeInc(curreg->x2,buf,len)) return 0;
				if(!decodeInc(curreg->y2,buf,len)) return 0;
				if(!decodeInc(curreg->cen_x,buf,len)) return 0;
				if(!decodeInc(curreg->cen_y,buf,len)) return 0;
				if(!decodeInc(curreg->area,buf,len)) return 0;
				if(!decodeInc(curreg->run_start,buf,len)) return 0;
				if(j==tmpNumReg-1)
					curreg->next=NULL;
				else
					curreg->next=&regions[selectedSaveLayer][selectedSaveChannel][j+1];
			}
			//we're only going to save the region part (not the color info)
			if(!decodeInc(stats[i].min_area,buf,len)) return 0;
			if(!decodeInc(stats[i].total_area,buf,len)) return 0;
			if(!decodeInc(stats[i].merge_threshold,buf,len)) return 0;
			stats[i].list=regions[selectedSaveLayer][selectedSaveChannel];
			stats[i].num=tmpNumReg;
		}
		imageValids[selectedSaveLayer][selectedSaveChannel]=true;
		return origlen-len;	
	}
}

unsigned int
RegionGenerator::saveBuffer(char buf[], unsigned int len) const {
	unsigned int origlen=len;
	if(!checkInc(FilterBankGenerator::saveBuffer(buf,len),buf,len)) return 0;
	if(!encodeInc("RegionImage",buf,len)) return 0;
	
	if(images[selectedSaveLayer][selectedSaveChannel]==NULL) {
		serr->printf("RegionGenerator::saveBuffer() failed because selected image is NULL -- call selectSaveImage first to make sure it's up to date\n");
		return 0;
	}
	if(!imageValids[selectedSaveLayer][selectedSaveChannel]) {
		serr->printf("RegionGenerator::saveBuffer() failed because selected image is invalid -- call selectSaveImage first to make sure it's up to date\n");
		return 0;
	}
	unsigned char* img=images[selectedSaveLayer][selectedSaveChannel];
	region_stats * stats=reinterpret_cast<region_stats*>(img);
	if(stats==NULL)
		return 0;
	if(!encodeInc(srcNumColors,buf,len)) return 0;
	for(unsigned int i=0; i<srcNumColors; i++) {
		if(!encodeInc(stats[i].num,buf,len)) return 0;
		region * curreg=stats[i].list;
		for(int j=0; j<stats[i].num; j++) {
			if(!encodeInc(curreg->color,buf,len)) return 0;
			if(!encodeInc(curreg->x1,buf,len)) return 0;
			if(!encodeInc(curreg->y1,buf,len)) return 0;
			if(!encodeInc(curreg->x2,buf,len)) return 0;
			if(!encodeInc(curreg->y2,buf,len)) return 0;
			if(!encodeInc(curreg->cen_x,buf,len)) return 0;
			if(!encodeInc(curreg->cen_y,buf,len)) return 0;
			if(!encodeInc(curreg->area,buf,len)) return 0;
			if(!encodeInc(curreg->run_start,buf,len)) return 0;
			curreg=curreg->next;
		}
		//we're only going to save the region part (not the color info)
		if(!encodeInc(stats[i].min_area,buf,len)) return 0;
		if(!encodeInc(stats[i].total_area,buf,len)) return 0;
		if(!encodeInc(stats[i].merge_threshold,buf,len)) return 0;
	}
	return origlen-len;
}

void
RegionGenerator::setNumImages(unsigned int nLayers, unsigned int nChannels) {
	if(nLayers==numLayers && nChannels==numChannels)
		return;
	FilterBankGenerator::setNumImages(nLayers,nChannels);
	regions = new region**[numLayers];
	for(unsigned int i=0; i<numLayers; i++) {
		regions[i] = new region*[numChannels];
		for(unsigned int j=0; j<numChannels; j++)
			regions[i][j]=NULL;
	}
}

unsigned char *
RegionGenerator::createImageCache(unsigned int layer, unsigned int chan) const {
	//this is where we'll store the linked list of regions for this image
	regions[layer][chan]=new region[MAX_REGIONS];

	//this is where we store the head of each list as well as some general stats of each image
	//this is what will be returned as getImage()
	region_stats * stats=new region_stats[srcNumColors];
	
	//initialize the region parameters (held in color definition...)
	memcpy(stats,srcColors,srcNumColors*sizeof(region_stats));
	
	return reinterpret_cast<unsigned char*>(stats);
}

void
RegionGenerator::calcImage(unsigned int layer, unsigned int chan) {
	PROFSECTION("RegionGenerator::calcImage(...)",*mainProfiler);
	
	//some shorthand to make the rest of the code more readable
	RLEGenerator& srcRLE=dynamic_cast<RLEGenerator&>(*src); //source generator
	RLEGenerator::run * rmap=reinterpret_cast<RLEGenerator::run*>(srcRLE.getImage(layer,chan)); //the RLE encoded image from source
	region_stats * stats=reinterpret_cast<region_stats*>(images[layer][chan]); //our top level region stats array (which is what users get from getImage())

	//do the actual calculations
  CMVision::ConnectComponents(rmap,srcRLE.getNumRuns(layer,chan));
  unsigned int numReg = CMVision::ExtractRegions(regions[layer][chan],MAX_REGIONS,rmap,srcRLE.getNumRuns(layer,chan));
  unsigned int maxArea = CMVision::SeparateRegions(stats,srcNumColors,regions[layer][chan],numReg);
  CMVision::SortRegions(stats,srcNumColors,maxArea);
  CMVision::MergeRegions(stats,(int)srcNumColors,rmap); //yes that (int) is necessary or compiler complains of ambiguous function call
  CMVision::CalcTotalArea(stats,srcNumColors);

	//and set the flag so we don't recompute if getImage() is called again before the next frame
	imageValids[layer][chan]=true;
}

void
RegionGenerator::destruct() {
	FilterBankGenerator::destruct();
	for(unsigned int i=0; i<numLayers; i++)
		delete [] regions[i];
	delete [] regions;
	regions=NULL;
}

/*! @file
 * @brief Implements RegionGenerator, which connects regions of CMVision format runs in RLEGenerator
 * @author alokl (Creator)
 * @author ejt (reorganized)
 */

