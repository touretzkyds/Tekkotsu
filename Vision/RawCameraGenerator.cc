#include "RawCameraGenerator.h"
#include "Events/DataEvent.h"
#include "Events/EventRouter.h"
#include "Events/FilterBankEvent.h"
#include "Wireless/Wireless.h"
#include "Shared/Config.h"
#include "Shared/Profiler.h"
#include "Shared/ProjectInterface.h"

#include "Shared/ODataFormats.h"
#include "OFbkImage.h"

#include "Shared/debuget.h"

#include <float.h>

RawCameraGenerator::RawCameraGenerator(unsigned int numRawLayers, unsigned int numCalcLayers,
				       unsigned int mysid, EventBase::EventGeneratorID_t gid, unsigned int sid)
  : FilterBankGenerator("RawCameraGenerator",EventBase::visRawCameraEGID,mysid,gid,sid),
    numRealLayers(numRawLayers), layers(NULL), imageInfos(NULL)
{
  /* As a root stage, we need to listen to all incoming image
   * events, even if we don't currently have listeners of our own --
   * This is just in case user code directly accesses a generator
   * and we have to retroactively go back and dig up the previous
   * frame */
  unsetAutoListen();
	
  setNumImages(numCalcLayers,NUM_CHANNELS);
}

RawCameraGenerator::~RawCameraGenerator() {
  freeCaches();
  destruct();
}

/*! The const casts in this function are regretable but necessary
 *  since the OPEN-R OFbkImage constructor requires mutable
 *  arguments, even though it shouldn't be modifying the data
 */
void RawCameraGenerator::doEvent() {
  if(event->getGeneratorID()!=getListenGeneratorID() || event->getSourceID()!=getListenSourceID())
    return;
  if(event->getTypeID()==EventBase::activateETID) {
    typedef DataEvent<const OFbkImageVectorData*> OFbkEvent;
    const OFbkEvent& fbkevent=dynamic_cast<const OFbkEvent& >(*event);
    OFbkImageVectorData& fbkdat=*const_cast<OFbkImageVectorData*>(fbkevent.getData());
    for(unsigned int res=0; res<numRealLayers; res++) {
      layers[numLayers-2-res] = fbkdat.GetData(res);
      imageInfos[numLayers-2-res] = fbkdat.GetInfo(res);
    }
    {
      const OFbkImage img(const_cast<OFbkImageInfo*>(imageInfos[numLayers-2]), const_cast<unsigned char*>(layers[numLayers-2]), ofbkimageBAND_Y);
      //I have to do this crazy thing because apparently img.FieldCounter() doesn't work
      sysFrameNumber=frameNumber=*(int*)(img.Pointer()+(img.Height()-1)*(img.Skip()+img.Width()));
    }
    unsigned int numNotRealLayers=numLayers-1-numRealLayers;
    bool dimchange=false;
    for(unsigned int res=numNotRealLayers; res<numLayers-1; res++) {
      if(widths[res]!=imageInfos[res]->width || heights[res]!=imageInfos[res]->height) {
	dimchange=true;
	serr->printf("WARNING: the image dimensions changed, now %dx%d\n",widths[numLayers-1],heights[numLayers-1]);
	widths[res] = imageInfos[res]->width;
	heights[res] = imageInfos[res]->height;
      }

      const OFbkImage img(const_cast<OFbkImageInfo*>(imageInfos[res]), const_cast<unsigned char*>(layers[res]), ofbkimageBAND_Y);
      skips[res]=img.Skip();
      strides[res]=skips[res]+widths[res];

      ASSERT(static_cast<unsigned int>(img.Width())==getWidth(res),"Widths don't agree for layer "<<res);
      ASSERT(static_cast<unsigned int>(img.Height())==getHeight(res),"Heights don't agree for layer "<<res);
    }
    if(widths[numLayers-2]*2!=widths[numLayers-1] || heights[numLayers-2]*2!=heights[numLayers-1]) {
      //|| widths[numLayers-2-numRealLayers]*2!=widths[numNotRealLayers]
      //|| heights[numLayers-2-numRealLayers]*2!=heights[numNotRealLayers]) {
      //set the width and height of non-real layers (since they don't match what they should be)
      serr->printf("WARNING: the image dimensions don't match values predicted by RobotInfo consts, \"full\" layer now %dx%d\n",widths[ProjectInterface::fullLayer],heights[ProjectInterface::fullLayer]);
      freeCaches();
      dimchange=true;
    } else if(strides[numLayers-1]==0) {
      // first frame
      dimchange=true;
    }
    if(dimchange)
      setDimensions();
    float testaspect=widths[numLayers-2]/(float)heights[numLayers-2];
    if(fabs(testaspect-config->vision.aspectRatio)>FLT_EPSILON) {
      serr->printf("WARNING: the image aspect ratio changed, was %g, now %g (diff %g)\n",*config->vision.aspectRatio,testaspect,testaspect-*config->vision.aspectRatio);
      config->vision.aspectRatio=testaspect;
    }
		
    invalidateCaches();	
    framesProcessed++;
  }
  erouter->postEvent(FilterBankEvent(this,getGeneratorID(),getSourceID(),event->getTypeID()));
}

unsigned int RawCameraGenerator::getBinSize() const {
  unsigned int used=FilterBankGenerator::getBinSize();
  used+=strlen("RawImage")+LoadSave::stringpad;
  used+=widths[selectedSaveLayer]*heights[selectedSaveLayer];
  return used;
}

unsigned int RawCameraGenerator::loadBuffer(const char buf[], unsigned int len, const char* filename) {
  unsigned int origlen=len;
  std::string tmp;
  if(!checkInc(FilterBankGenerator::loadBuffer(buf,len,filename),buf,len)) return 0;
  if(!decodeInc(tmp,buf,len)) return 0;
  if(tmp!="RawImage") {
    serr->printf("Unhandled image type for RawCameraGenerator: %s",tmp.c_str());
    return 0;
  } else if(selectedSaveLayer!=numLayers-1) {
    serr->printf("Can't load into RawCameraGenerator layer %d!=%d",selectedSaveLayer,numLayers-1);
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

unsigned int RawCameraGenerator::saveBuffer(char buf[], unsigned int len) const {
  unsigned int origlen=len;
  if(!checkInc(FilterBankGenerator::saveBuffer(buf,len),buf,len)) return 0;
  if(!encodeInc("RawImage",buf,len)) return 0;
	
  if(images[selectedSaveLayer][selectedSaveChannel]==NULL) {
    serr->printf("RawCameraGenerator::saveBuffer() failed because selected image is NULL -- call selectSaveImage first to make sure it's up to date\n");
    return 0;
  }
  if(!imageValids[selectedSaveLayer][selectedSaveChannel]) {
    serr->printf("RawCameraGenerator::saveBuffer() failed because selected image is invalid -- call selectSaveImage first to make sure it's up to date\n");
    return 0;
  }
  unsigned char* img=images[selectedSaveLayer][selectedSaveChannel];
  unsigned int used=widths[selectedSaveLayer]*heights[selectedSaveLayer];
  ASSERTRETVAL(used<=len,"buffer too small",0);
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

unsigned int RawCameraGenerator::saveFileStream(FILE * f) const {
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
    serr->printf("RawCameraGenerator::saveBuffer() failed because selected image is NULL -- call selectSaveImage first to make sure it's up to date\n");
    return 0;
  }
  if(!imageValids[selectedSaveLayer][selectedSaveChannel]) {
    serr->printf("RawCameraGenerator::saveBuffer() failed because selected image is invalid -- call selectSaveImage first to make sure it's up to date\n");
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

void RawCameraGenerator::setDimensions() {
  freeCaches();
  unsigned int numNotRealLayers=numLayers-1-numRealLayers;
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

void  RawCameraGenerator::freeCaches() {
#ifdef PLATFORM_APERIOS
  const unsigned int numRefLayers=numLayers;
#else
  const unsigned int numRefLayers=numLayers-1;
#endif
  for(unsigned int i=0; i<numRefLayers; i++) {
    for(unsigned int j=0; j<numChannels; j++) {
      images[i][j]=NULL;
      imageValids[i][j]=false;
    }
  }
  FilterBankGenerator::freeCaches();
}

void RawCameraGenerator::setNumImages(unsigned int nLayers, unsigned int nChannels) {
  if(nLayers==numLayers && nChannels==numChannels)
    return;
  FilterBankGenerator::setNumImages(nLayers,nChannels);
  layers=new unsigned char*[numLayers];
  imageInfos=new const OFbkImageInfo*[numLayers];
  unsigned int numNotRealLayers=numLayers-1-numRealLayers;
  for(unsigned int res=0; res<numLayers; res++) {
    layers[res]=NULL;
    imageInfos[res]=NULL;
    if(res<numNotRealLayers)
      increments[res]=1<<(numNotRealLayers-res);
  }
}

unsigned char* RawCameraGenerator::createImageCache(unsigned int layer, unsigned int chan) const {
  if(layer==numLayers-1) {
#ifdef PLATFORM_APERIOS
    return const_cast<unsigned char*>(&dblRes[chan][0][0]);
#else
    return new unsigned char[widths[layer]*heights[layer]];	
#endif
  } else
    return NULL; // calcImage will set the cache itself
}

void RawCameraGenerator::calcImage(unsigned int layer, unsigned int chan) {
  PROFSECTION("RawCameraGenerator::calcImage(...)",*mainProfiler);
  unsigned int numNotRealLayers=numLayers-1-numRealLayers;
  if(layer==numLayers-1) {
    //This is the only layer for which we calculate and store any data of our own...
    if(chan==CHAN_Y)
      reconstructImage();
    else
      upsampleImage(static_cast<channel_id_t>(chan));
  } else {
    if(layer>=numNotRealLayers) {
      unsigned int fbkdatChan=mapChannelID(static_cast<channel_id_t>(chan));
      const OFbkImage img(const_cast<OFbkImageInfo*>(imageInfos[layer]), const_cast<unsigned char*>(layers[layer]), fbkdatChan);
      images[layer][chan]=img.Pointer();

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
    } else {
      //we don't need to do the restoration in the previous section
      //here because these layers skip the last row
      unsigned int fbkdatChan=mapChannelID(static_cast<channel_id_t>(chan));
      const OFbkImage img(const_cast<OFbkImageInfo*>(imageInfos[numNotRealLayers]), const_cast<unsigned char*>(layers[numNotRealLayers]), fbkdatChan);
      images[layer][chan]=img.Pointer();
    }
  }
  imageValids[layer][chan]=true;
}

void RawCameraGenerator::destruct() {
  FilterBankGenerator::destruct();
  delete [] layers;
  layers=NULL;
  delete [] imageInfos;
  imageInfos=NULL;
}

unsigned int RawCameraGenerator::mapChannelID(channel_id_t chan) {
  switch(chan) {
  case CHAN_Y:
    return ofbkimageBAND_Y;
  case CHAN_U:
    return ofbkimageBAND_Cb;
  case CHAN_V:
    return ofbkimageBAND_Cr;
  case CHAN_Y_DY:
    return ofbkimageBAND_Y_LH;
  case CHAN_Y_DX:
    return ofbkimageBAND_Y_HL;
  case CHAN_Y_DXDY:
    return ofbkimageBAND_Y_HH;
  default:
    std::cout << "RawCameraGenerator::mapChannelID bad channel" << std::endl;
    return ofbkimageBAND_Y;;
  }
}

void RawCameraGenerator::upsampleImage(channel_id_t chan) {
  const unsigned int dblLayer=numLayers-1;
  const unsigned int srcLayer=dblLayer-1;
  const unsigned int width=widths[dblLayer];
  const unsigned int height=heights[dblLayer];

  unsigned char * cur=images[dblLayer][chan];
  ASSERTRET(cur!=NULL,"destination layer is NULL");
  unsigned char * orig=getImage(srcLayer,chan);
  ASSERTRET(orig!=NULL,"source layer is NULL");

  unsigned char * const imgend=cur+width*height;
  while(cur!=imgend) {
    unsigned char * const row=cur;
    unsigned char * const rowend=cur+width;
    while(cur!=rowend) {
      *cur++=*orig;
      *cur++=*orig++;
    }
    memcpy(cur,row,width);
    cur+=width;
    orig+=getSkip(srcLayer);
  }
}



/*! This function is lifted from Sony's ImageObserver sample code.
  Here's Sony's original license for the file (ImageObserver.cc) that contained this function:
  <pre>
  Copyright 2002,2003 Sony Corporation 
		
  Permission to use, copy, modify, and redistribute this software for
  non-commercial use is hereby granted.
		
  This software is provided "as is" without warranty of any kind,
  either expressed or implied, including but not limited to the
  implied warranties of fitness for a particular purpose.
  </pre>
*/
void RawCameraGenerator::reconstructImage() {
  byte* yLLPtr = getImage(numLayers-2,CHAN_Y);
  byte* yLHPtr = getImage(numLayers-2,CHAN_Y_DY);
  byte* yHLPtr = getImage(numLayers-2,CHAN_Y_DX);
  byte* yHHPtr = getImage(numLayers-2,CHAN_Y_DXDY);
	
  unsigned int w = getWidth(numLayers-2);
  unsigned int h = getWidth(numLayers-2);
  unsigned int skip = getSkip(numLayers-2);
	
  unsigned char* img = images[numLayers-1][CHAN_Y];
  ASSERTRET(img!=NULL,"image destination NULL");

  unsigned char* iptr0 = img;
  unsigned char* iptr1 = iptr0 + 2*w;
    
  for (unsigned int y = 0; y < h; y++) {
    for (unsigned int x = 0; x < w; x++) {
      //
      // yLH, yHL, yHH : offset binary [0, 255] -> signed int [-128, 127]
      //
      short yLL = (short)*yLLPtr++;
      short yLH = (short)*yLHPtr++ - 128;
      short yHL = (short)*yHLPtr++ - 128;
      short yHH = (short)*yHHPtr++ - 128;

      short a = yLL + yLH + yHL + yHH; // ypix11
      short b = 2 * (yLL + yLH);       // ypix11 + ypix01
      short c = 2 * (yLL + yHL);       // ypix11 + ypix10
      short d = 2 * (yLL + yHH);       // ypix11 + ypix00
            
      *iptr0++ = clipRange(d - a);
      *iptr0++ = clipRange(c - a);
      *iptr1++ = clipRange(b - a);
      *iptr1++ = clipRange(a);
    }
    yLLPtr += skip;
    yLHPtr += skip;
    yHLPtr += skip;
    yHHPtr += skip;
    iptr0  = iptr1;
    iptr1  += 2*w;
  }
}


/*! @file
 * @brief Implements RawCameraGenerator, which generates FilterBankEvents containing raw camera images
 * @author ejt (Creator)
 */

