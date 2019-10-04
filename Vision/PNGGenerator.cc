#include "PNGGenerator.h"
#include "InterleavedYUVGenerator.h"
#include "Events/DataEvent.h"
#include "Events/EventRouter.h"
#include "Events/FilterBankEvent.h"
#include "Shared/Profiler.h"
#include "Wireless/Socket.h"
#include "Shared/ImageUtil.h"
#include "Shared/debuget.h"

using namespace std; 

PNGGenerator::PNGGenerator(unsigned int mysid, FilterBankGenerator* fbg, EventBase::EventTypeID_t tid)
: FilterBankGenerator("PNGGenerator",EventBase::visPNGEGID,mysid,fbg,tid), srcMode(SRC_AUTO), curMode(SRC_AUTO), bytesUsed(NULL)
{
	if(dynamic_cast<const InterleavedYUVGenerator*>(src)!=NULL)
		curMode=SRC_COLOR;
	else
		curMode=SRC_GRAYSCALE;
	
	//this part is only necessary if you override setNumImages yourself
	if(fbg!=NULL) {
		numLayers=numChannels=0; //this is to force setNumImages to override settings provided by FilterBankGenerator
		setNumImages(fbg->getNumLayers(),fbg->getNumChannels());
	}
}

PNGGenerator::PNGGenerator(unsigned int mysid, src_mode_t mode, FilterBankGenerator* fbg, EventBase::EventTypeID_t tid)
: FilterBankGenerator("PNGGenerator",EventBase::visPNGEGID,mysid,fbg,tid), srcMode(mode), curMode(mode), bytesUsed(NULL)
{
	if(srcMode==SRC_AUTO) {
		if(dynamic_cast<const InterleavedYUVGenerator*>(src)!=NULL)
			curMode=SRC_COLOR;
		else
			curMode=SRC_GRAYSCALE;
	}
	
	//this part is only necessary if you override setNumImages yourself
	if(fbg!=NULL) {
		numLayers=numChannels=0; //this is to force setNumImages to override settings provided by FilterBankGenerator
		setNumImages(fbg->getNumLayers(),fbg->getNumChannels());
	}
}

PNGGenerator::~PNGGenerator() {
	freeCaches();
	destruct();
}


/*! The const casts in this function are regretable but necessary
*  since the corresponding OPEN-R functions require mutable
*  arguments, even though they shouldn't be modifying the data
*/
void
PNGGenerator::doEvent() {
	if(event->getGeneratorID()==getListenGeneratorID() && event->getSourceID()==getListenSourceID()) {
		if(getSourceMode()==SRC_AUTO) { //if not auto, curMode was already set when srcMode was set
			if(dynamic_cast<const InterleavedYUVGenerator*>(src)!=NULL)
				curMode=SRC_COLOR;
			else
				curMode=SRC_GRAYSCALE;
		}
		FilterBankEvent fbkev(this,getGeneratorID(),getSourceID(),EventBase::activateETID);
		erouter->postEvent(fbkev);
		fbkev.setTypeID(EventBase::statusETID);
		erouter->postEvent(fbkev);
		fbkev.setTypeID(EventBase::deactivateETID);
		erouter->postEvent(fbkev);
	}
}

unsigned int
PNGGenerator::getBinSize() const {
	unsigned int used=FilterBankGenerator::getBinSize();
	const char * type;
	if(getCurrentSourceFormat()==SRC_COLOR)
		type="PNGColor";
	else if(getCurrentSourceFormat()==SRC_GRAYSCALE)
		type="PNGGrayscale";
	else {
		serr->printf("getBinSize failed - unsuitable or unknown mode/generator pair");
		return 0;
	}
	used+=strlen(type)+LoadSave::stringpad;
	if(bytesUsed[selectedSaveLayer][selectedSaveChannel]!=0)
		used+=bytesUsed[selectedSaveLayer][selectedSaveChannel];
	else
		used+=widths[selectedSaveLayer]*heights[selectedSaveLayer]*3+PNG_HEADER_PAD;
	return used;
}

unsigned int
PNGGenerator::loadBuffer(const char buf[], unsigned int len, const char* filename) {
	unsigned int origlen=len;
	if(!FilterBankGenerator::loadBuffer(buf,len,filename)) return 0;
	std::string tmp;
	if(!decodeInc(tmp,buf,len)) return 0;
	if(tmp!="PNGColor" && tmp!="PNGGrayscale") {
		serr->printf("Unhandled image type for PNGGenerator: %s",tmp.c_str());
		return 0;
	} else {
		if(tmp=="PNGColor" && getCurrentSourceFormat()==SRC_GRAYSCALE)
			serr->printf("Warning: loading grayscale into color image");
		if(tmp=="PNGGrayscale" && getCurrentSourceFormat()==SRC_COLOR)
			serr->printf("Warning: loading color into grayscale image");
		unsigned int tmpL;
		if(!decodeInc(tmpL,buf,len)) return 0;
		if(tmpL>len)
			return 0;
		if(images[selectedSaveLayer][selectedSaveChannel]!=NULL)
			delete [] images[selectedSaveLayer][selectedSaveChannel];
		images[selectedSaveLayer][selectedSaveChannel]=createImageCache(selectedSaveLayer,selectedSaveChannel);
		unsigned int used=bytesUsed[selectedSaveLayer][selectedSaveChannel]=tmpL;
		unsigned char* img=getImage(selectedSaveLayer,selectedSaveChannel);
		if(img==NULL)
			return 0;
		memcpy(img,buf,used);
		len-=used; buf+=used;
		imageValids[selectedSaveLayer][selectedSaveChannel]=true;
		return origlen-len;	
	}
}

unsigned int
PNGGenerator::saveBuffer(char buf[], unsigned int len) const {
	unsigned int origlen=len;
	if(!checkInc(FilterBankGenerator::saveBuffer(buf,len),buf,len)) return 0;
	
	const char * type;
	if(getCurrentSourceFormat()==SRC_COLOR)
		type="PNGColor";
	else if(getCurrentSourceFormat()==SRC_GRAYSCALE)
		type="PNGGrayscale";
	else {
		serr->printf("saveBuffer failed - unsuitable or unknown mode/generator pair");
		return 0;
	}
	if(!encodeInc(type,buf,len)) return 0;
	
	if(images[selectedSaveLayer][selectedSaveChannel]==NULL) {
		serr->printf("PNGGenerator::saveBuffer() failed because selected image is NULL -- call selectSaveImage first to make sure it's up to date\n");
		return 0;
	}
	if(!imageValids[selectedSaveLayer][selectedSaveChannel]) {
		serr->printf("PNGGenerator::saveBuffer() failed because selected image is invalid -- call selectSaveImage first to make sure it's up to date\n");
		return 0;
	}
	unsigned char* img=images[selectedSaveLayer][selectedSaveChannel];
	if(img==NULL)
		return 0;
	if(!encodeInc(bytesUsed[selectedSaveLayer][selectedSaveChannel],buf,len)) return 0;
	unsigned int used=bytesUsed[selectedSaveLayer][selectedSaveChannel];
	if(used>len)
		return 0;
	memcpy(buf,img,used);
	len-=used;
	return origlen-len;
}

void
PNGGenerator::setNumImages(unsigned int nLayers, unsigned int nChannels) {
	if(nLayers==numLayers && nChannels==numChannels)
		return;
	FilterBankGenerator::setNumImages(nLayers,nChannels);
	for(unsigned int i=0; i<numLayers; i++)
		strides[i]=skips[i]=0;
	bytesUsed=new unsigned int*[numLayers];
	for(unsigned int res=0; res<numLayers; res++) {
		increments[res]=3;
		bytesUsed[res]=new unsigned int[numChannels];
		for(unsigned int i=0; i<numChannels; i++)
			bytesUsed[res][i]=0;
	}
}

unsigned char *
PNGGenerator::createImageCache(unsigned int layer, unsigned int /*chan*/) const {
	return new unsigned char[widths[layer]*heights[layer]*3+PNG_HEADER_PAD];
}

void
PNGGenerator::calcImage(unsigned int layer, unsigned int chan) {
	PROFSECTION("PNGGenerator::calcImage(...)",*mainProfiler);
	
	// input configuration
	char* inbuf = reinterpret_cast<char*>(src->getImage(layer,chan));
	size_t inbufSize = src->getWidth(layer)*src->getIncrement(layer)*src->getHeight(layer);
	size_t srcChans = src->getIncrement(layer);
	
	// output configuration
	ASSERT(images[layer][chan]!=NULL,"image was not allocated");
	char*& outbuf = reinterpret_cast<char*&>(images[layer][chan]);
	size_t outbufSize = widths[layer]*heights[layer]*3+PNG_HEADER_PAD;
	size_t dstChans;
	if(getCurrentSourceFormat()==SRC_COLOR ) {
		dstChans=3;
	} else if(getCurrentSourceFormat()==SRC_GRAYSCALE) {
		dstChans=1;
	} else {
		serr->printf("%s %s Compression failed - unsuitable or unknown mode/generator pair",getClassName().c_str(),getName().c_str());
		return;
	}
	
	// do it!
	bytesUsed[layer][chan] = image_util::encodePNG(inbuf,inbufSize,widths[layer],heights[layer],srcChans,outbuf,outbufSize,dstChans);
	imageValids[layer][chan] = (bytesUsed[layer][chan]!=0);
}

void
PNGGenerator::destruct() {
	FilterBankGenerator::destruct();
	for(unsigned int res=0; res<numLayers; res++)
		delete [] bytesUsed[res];
	delete [] bytesUsed;
	bytesUsed=NULL;
}

/*! @file
 * @brief Implements PNGGenerator, which generates FilterBankEvents containing PNG compressed images
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
