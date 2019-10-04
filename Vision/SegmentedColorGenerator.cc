#include "SegmentedColorGenerator.h"
#include "Events/EventRouter.h"
#include "Events/SegmentedColorFilterBankEvent.h"
#include "Wireless/Wireless.h"
#include "Shared/Profiler.h"
#include "Shared/Config.h"

#include "Shared/debuget.h"

SegmentedColorGenerator::SegmentedColorGenerator(unsigned int mysid, FilterBankGenerator* fbg, EventBase::EventTypeID_t tid)
	: FilterBankGenerator("SegmentedColorGenerator",EventBase::visSegmentEGID,mysid,fbg,tid), srcYChan(0), srcUChan(1), srcVChan(2), tmaps(), tmapNames(), numColors(0), colorNames()
{
	//this part is only necessary if you override setNumImages yourself
	if(fbg!=NULL) {
		numLayers=numChannels=0; //this is to force setNumImages to override settings provided by FilterBankGenerator
		setNumImages(fbg->getNumLayers(),fbg->getNumChannels());
	}
}

SegmentedColorGenerator::SegmentedColorGenerator(unsigned int mysid, FilterBankGenerator* fbg, EventBase::EventTypeID_t tid, unsigned int syc, unsigned int suc, unsigned int svc)
	: FilterBankGenerator("SegmentedColorGenerator",EventBase::visSegmentEGID,mysid,fbg,tid), srcYChan(syc), srcUChan(suc), srcVChan(svc), tmaps(), tmapNames(), numColors(0), colorNames()
{
	if(fbg!=NULL) {
		numLayers=numChannels=0; //this is to force setNumImages to override settings provided by FilterBankGenerator
		setNumImages(fbg->getNumLayers(),fbg->getNumChannels());
	}
}

SegmentedColorGenerator::~SegmentedColorGenerator() {
	freeCaches();
	destruct();
	for(unsigned int i=0; i<tmaps.size(); i++)
		delete [] tmaps[i];

	//hashmap::iterator it=colorNames.begin(); //not the way i'd like to iterate, but there seems to be a bug in the current hashmap implementation we're using...
	//for(unsigned int i=0; i<colorNames.size(); it++,i++)
	//free(const_cast<char*>(it->first));
  //colorNames.clear();

  colorNames.clear(); //we're leaking the memory of the names themselves...
}

void
SegmentedColorGenerator::doEvent() {
	if(event->getGeneratorID()==getListenGeneratorID() && event->getSourceID()==getListenSourceID()) {
		SegmentedColorFilterBankEvent segev(this,getGeneratorID(),getSourceID(),EventBase::activateETID,this,getNumColors(),getColors(),&colorNames);
		erouter->postEvent(segev);
		segev.setTypeID(EventBase::statusETID);
		erouter->postEvent(segev);
		segev.setTypeID(EventBase::deactivateETID);
		erouter->postEvent(segev);
	}
}

unsigned int
SegmentedColorGenerator::loadThresholdMap(const std::string& tm_file) {
  const unsigned int size = 1 << (BITS_Y + BITS_U + BITS_V);
	cmap_t * tmap = new cmap_t[size];
  if(!CMVision::LoadThresholdFile(tmap,NUM_Y,NUM_U,NUM_V,config->portPath(tm_file).c_str())) {
    serr->printf("  ERROR: Could not load threshold file '%s'.\n",tm_file.c_str());
		delete [] tmap;
		return -1U;
  }
	if(numColors!=0) {
		//we've already loaded color info, check against that for invalid indexes
		int remapped=CMVision::CheckTMapColors(tmap,NUM_Y,NUM_U,NUM_V,numColors,0);
		if(remapped>0)
			serr->printf("remapped %d colors in threshold file '%s'\n",remapped, tm_file.c_str());
	}

	tmapNames.push_back(tm_file);
	tmaps.push_back(tmap);
	setNumImages(numLayers,tmaps.size());
	return tmaps.size()-1;
}

bool
SegmentedColorGenerator::loadColorInfo(const std::string& col_file) {
	//hashmap::iterator it=colorNames.begin(); //not the way i'd like to iterate, but there seems to be a bug in the current hashmap implementation we're using...
	//for(unsigned int i=0; i<colorNames.size(); it++,i++)
	//free(const_cast<char*>(it->first));
	//colorNames.clear();

	colorNames.clear(); //we're leaking the memory of the names themselves...

	numColors=CMVision::LoadColorInformation(colors,MAX_COLORS,config->portPath(col_file).c_str(),colorNames);
	if(numColors <= 0){
		serr->printf("  ERROR: Could not load colors file:%s\n", col_file.c_str());
		return false;
	}

	//we'd better check the already loaded thresholds (if any) for out of bound indexes
	for(unsigned int i=0; i<tmaps.size(); i++) {
		int remapped=CMVision::CheckTMapColors(tmaps[i],NUM_Y,NUM_U,NUM_V,numColors,0);
		if(remapped>0)
			serr->printf("remapped %d colors in threshold file '%s'\n",remapped, tmapNames[i].c_str());
	}
	return true;
}


unsigned int
SegmentedColorGenerator::getBinSize() const {
	unsigned int used=FilterBankGenerator::getBinSize();
	used+=creatorSize("SegColorImage");
	used+=widths[selectedSaveLayer]*heights[selectedSaveLayer];
	return used;
}

unsigned int
SegmentedColorGenerator::loadBuffer(const char buf[], unsigned int len, const char* filename) {
	unsigned int origlen=len;
	if(!checkInc(FilterBankGenerator::loadBuffer(buf,len,filename),buf,len)) return 0;
	if(!checkCreatorInc("SegColorImage",buf,len)) {
		serr->printf("Unhandled image type for SegmentedColorGenerator: %s",buf+getSerializedSize<unsigned int>());
		return 0;
	} else {
		// actual image
		unsigned int used=widths[selectedSaveLayer]*heights[selectedSaveLayer];
		if(used>len)
			return 0;
		if(images[selectedSaveLayer][selectedSaveChannel]==NULL)
			images[selectedSaveLayer][selectedSaveChannel]=createImageCache(selectedSaveLayer,selectedSaveChannel);
		unsigned char* img=images[selectedSaveLayer][selectedSaveChannel];
		if(img==NULL)
			return 0;
		memcpy(img,buf,used);
		len-=used; buf+=used;

		// color table
		if(!decodeColorsInc(buf,len)) return 0;
		
		imageValids[selectedSaveLayer][selectedSaveChannel]=true;
		return origlen-len;	
	}
}

unsigned int
SegmentedColorGenerator::saveBuffer(char buf[], unsigned int len) const {
	unsigned int origlen=len;
	if(!checkInc(FilterBankGenerator::saveBuffer(buf,len),buf,len)) return 0;
	if(!saveCreatorInc("SegColorImage",buf,len)) return 0;
	
	// actual image
	unsigned int used=widths[selectedSaveLayer]*heights[selectedSaveLayer];
	if(used>len)
		return 0;
	if(images[selectedSaveLayer][selectedSaveChannel]==NULL) {
		serr->printf("SegmentedColorGenerator::saveBuffer() failed because selected image is NULL -- call selectSaveImage first to make sure it's up to date\n");
		return 0;
	}
	if(!imageValids[selectedSaveLayer][selectedSaveChannel]) {
		serr->printf("SegmentedColorGenerator::saveBuffer() failed because selected image is invalid -- call selectSaveImage first to make sure it's up to date\n");
		return 0;
	}
	unsigned char* img=images[selectedSaveLayer][selectedSaveChannel];
	if(img==NULL)
		return 0;
	memcpy(buf,img,used);
	len-=used; buf+=used;

	// color table
	if(!encodeColorsInc(buf,len)) return 0;
	
	return origlen-len;
}

bool
SegmentedColorGenerator::encodeColorsInc(char*& buf, unsigned int& len) const {
	if(!encodeInc(numColors,buf,len)) return false;
	for(unsigned int i=0; i<numColors; i++) {
		if(!encodeInc(colors[i].color.red,buf,len)) return false;
		if(!encodeInc(colors[i].color.green,buf,len)) return false;
		if(!encodeInc(colors[i].color.blue,buf,len)) return false;
	}
	return true;
}

bool
SegmentedColorGenerator::decodeColorsInc(const char*& buf, unsigned int& len) {
	if(!decodeInc(numColors,buf,len)) return false;
	for(unsigned int i=0; i<numColors; i++) {
		if(!decodeInc(colors[i].color.red,buf,len)) return false;
		if(!decodeInc(colors[i].color.green,buf,len)) return false;
		if(!decodeInc(colors[i].color.blue,buf,len)) return false;
	}
	return true;
}

void
SegmentedColorGenerator::setDimensions() {
	FilterBankGenerator::setDimensions();
	for(unsigned int i=0; i<numLayers; i++)
		strides[i]=widths[i];
}

void
SegmentedColorGenerator::setNumImages(unsigned int nLayers, unsigned int /*nChannels*/) {
	FilterBankGenerator::setNumImages(nLayers,tmaps.size());
}

unsigned char *
SegmentedColorGenerator::createImageCache(unsigned int layer, unsigned int /*chan*/) const {
	// notice the +1 !!!
	// this is because CMVision is a little naughty and writes an unused, terminator flag at the one-past-end of each row
	// if we didn't add one, this last byte would be beyond the end of the array
	return new unsigned char[widths[layer]*heights[layer]+1];
}

void
SegmentedColorGenerator::calcImage(unsigned int layer, unsigned int chan) {
	if(tmaps.size()==0)
		throw NoThresholdException();
	PROFSECTION("SegmentedColorGenerator::calcImage(...)",*mainProfiler);
	CMVision::image_yuv<const cmap_t> img;
	img.buf_y=src->getImage(layer,srcYChan);
	img.buf_u=src->getImage(layer,srcUChan);
	img.buf_v=src->getImage(layer,srcVChan);
	img.width=getWidth(layer);
	img.height=getHeight(layer);
	img.row_stride=src->getStride(layer);
	img.col_stride=src->getIncrement(layer);

	CMVision::ThresholdImageYUVPlanar<cmap_t,CMVision::image_yuv<const cmap_t>,const cmap_t,BITS_Y,BITS_U,BITS_V>(images[layer][chan],img,tmaps[chan]);
	imageValids[layer][chan]=true;
}

/*! @file
 * @brief Implements SegmentedColorGenerator, which generates FilterBankEvents indexed color images based on a color threshold file
 * @author alokl (Creator)
 * @author ejt (reorganized)
 */

