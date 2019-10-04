#include "JPEGGenerator.h"
#include "InterleavedYUVGenerator.h"
#include "Events/DataEvent.h"
#include "Events/EventRouter.h"
#include "Events/FilterBankEvent.h"
#include "Wireless/Wireless.h"
#include "Shared/Config.h"
#include "Shared/Profiler.h"
#include "Shared/ImageUtil.h"
#include "Shared/debuget.h"

JPEGGenerator::JPEGGenerator(unsigned int mysid, FilterBankGenerator* fbg, EventBase::EventTypeID_t tid)
	: FilterBankGenerator("JPEGGenerator",EventBase::visJPEGEGID,mysid,fbg,tid), srcMode(SRC_AUTO), curMode(SRC_AUTO), bytesUsed(NULL), cinfo(), jerr(), quality(-1U)
{
	if(dynamic_cast<const InterleavedYUVGenerator*>(src)!=NULL)
		curMode=SRC_COLOR;
	else
		curMode=SRC_GRAYSCALE;

	// We set the err object before we create the compress...  the idea
	// is if the creation fails, we can still get the error as to why it
	// failed.
	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);

	//this part is only necessary if you override setNumImages yourself
	if(fbg!=NULL) {
		numLayers=numChannels=0; //this is to force setNumImages to override settings provided by FilterBankGenerator
		setNumImages(fbg->getNumLayers(),fbg->getNumChannels());
	}
}

JPEGGenerator::JPEGGenerator(unsigned int mysid, src_mode_t mode, FilterBankGenerator* fbg, EventBase::EventTypeID_t tid)
	: FilterBankGenerator("JPEGGenerator",EventBase::visJPEGEGID,mysid,fbg,tid), srcMode(mode), curMode(mode), bytesUsed(NULL), cinfo(), jerr(), quality(-1U)
{
	if(srcMode==SRC_AUTO) {
		if(dynamic_cast<const InterleavedYUVGenerator*>(src)!=NULL)
			curMode=SRC_COLOR;
		else
			curMode=SRC_GRAYSCALE;
	}
	
	// We set the err object before we create the compress...  the idea
	// is if the creation fails, we can still get the error as to why it
	// failed.
	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);

	//this part is only necessary if you override setNumImages yourself
	if(fbg!=NULL) {
		numLayers=numChannels=0; //this is to force setNumImages to override settings provided by FilterBankGenerator
		setNumImages(fbg->getNumLayers(),fbg->getNumChannels());
	}
}

JPEGGenerator::~JPEGGenerator() {
	freeCaches();
	destruct();
	jpeg_destroy_compress(&cinfo);
}


/*! The const casts in this function are regretable but necessary
 *  since the corresponding OPEN-R functions require mutable
 *  arguments, even though they shouldn't be modifying the data
 */
void
JPEGGenerator::doEvent() {
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
JPEGGenerator::getBinSize() const {
	unsigned int used=FilterBankGenerator::getBinSize();
	const char * type;
	if(getCurrentSourceFormat()==SRC_COLOR)
		type="JPEGColor";
	else if(getCurrentSourceFormat()==SRC_GRAYSCALE)
		type="JPEGGrayscale";
	else {
		serr->printf("getBinSize failed - unsuitable or unknown mode/generator pair");
		return 0;
	}
	used+=strlen(type)+LoadSave::stringpad;
	if(bytesUsed[selectedSaveLayer][selectedSaveChannel]!=0)
		used+=bytesUsed[selectedSaveLayer][selectedSaveChannel];
	else
		used+=widths[selectedSaveLayer]*heights[selectedSaveLayer]*3+JPEG_HEADER_PAD;
	return used;
}

unsigned int
JPEGGenerator::loadBuffer(const char buf[], unsigned int len, const char* filename) {
	unsigned int origlen=len;
	if(!checkInc(FilterBankGenerator::loadBuffer(buf,len,filename),buf,len)) return 0;
	std::string tmp;
	if(!decodeInc(tmp,buf,len)) return 0;
	if(tmp!="JPEGColor" && tmp!="JPEGGrayscale") {
		serr->printf("Unhandled image type for JPEGGenerator: %s",tmp.c_str());
		return 0;
	} else {
		if(tmp=="JPEGColor" && getCurrentSourceFormat()==SRC_GRAYSCALE)
			serr->printf("Warning: loading grayscale into color image");
		if(tmp=="JPEGGrayscale" && getCurrentSourceFormat()==SRC_COLOR)
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
JPEGGenerator::saveBuffer(char buf[], unsigned int len) const {
	unsigned int origlen=len;
	if(!checkInc(FilterBankGenerator::saveBuffer(buf,len),buf,len)) return 0;

	const char * type;
	if(getCurrentSourceFormat()==SRC_COLOR)
		type="JPEGColor";
	else if(getCurrentSourceFormat()==SRC_GRAYSCALE)
		type="JPEGGrayscale";
	else {
		serr->printf("saveBuffer failed - unsuitable or unknown mode/generator pair");
		return 0;
	}
	if(!encodeInc(type,buf,len)) return 0;
	
	if(images[selectedSaveLayer][selectedSaveChannel]==NULL) {
		serr->printf("JPEGGenerator::saveBuffer() failed because selected image is NULL -- call selectSaveImage first to make sure it's up to date\n");
		return 0;
	}
	if(!imageValids[selectedSaveLayer][selectedSaveChannel]) {
		serr->printf("JPEGGenerator::saveBuffer() failed because selected image is invalid -- call selectSaveImage first to make sure it's up to date\n");
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
JPEGGenerator::setNumImages(unsigned int nLayers, unsigned int nChannels) {
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
JPEGGenerator::createImageCache(unsigned int layer, unsigned int /*chan*/) const {
	return new unsigned char[widths[layer]*heights[layer]*3+JPEG_HEADER_PAD];
}

void
JPEGGenerator::calcImage(unsigned int layer, unsigned int chan) {
	PROFSECTION("JPEGGenerator::calcImage(...)",*mainProfiler);
	
	// input configuration
	char* inbuf = reinterpret_cast<char*>(src->getImage(layer,chan));
	size_t inbufSize = src->getWidth(layer)*src->getIncrement(layer)*src->getHeight(layer);
	size_t srcChans = src->getIncrement(layer);
	
	// output configuration
	ASSERT(images[layer][chan]!=NULL,"image was not allocated");
	char*& outbuf = reinterpret_cast<char*&>(images[layer][chan]);
	size_t outbufSize = widths[layer]*heights[layer]*3+JPEG_HEADER_PAD;
	size_t dstChans;
	if(getCurrentSourceFormat()==SRC_COLOR ) {
		dstChans=3;
	} else if(getCurrentSourceFormat()==SRC_GRAYSCALE) {
		dstChans=1;
	} else {
		serr->printf("%s %s Compression failed - unsuitable or unknown mode/generator pair",getClassName().c_str(),getName().c_str());
		return;
	}
	unsigned int qual = (quality==-1U?*config->vision.rawcam.compress_quality:quality);
	unsigned int yskip = config->vision.rawcam.y_skip;
	unsigned int uvskip = config->vision.rawcam.uv_skip;
	
	// do it!
	bytesUsed[layer][chan] = image_util::encodeJPEG(inbuf,inbufSize,widths[layer],heights[layer],srcChans,outbuf,outbufSize,dstChans,qual,yskip,uvskip,cinfo);
	imageValids[layer][chan] = (bytesUsed[layer][chan]!=0);
}

void
JPEGGenerator::destruct() {
	FilterBankGenerator::destruct();
	for(unsigned int res=0; res<numLayers; res++)
		delete [] bytesUsed[res];
	delete [] bytesUsed;
	bytesUsed=NULL;
}

/*! @file
 * @brief Implements JPEGGenerator, which generates FilterBankEvents containing JPEG compressed images
 * @author ejt (Creator)
 */

