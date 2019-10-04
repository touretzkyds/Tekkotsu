#include "ImageBuffer.h"

#include "Shared/ImageUtil.h"

using namespace std; 

bool ImageBuffer::getYUV(const char*& buf, size_t& size) {
	updateYUV();
	buf=yuv.getBuffer();
	size=yuv.used;
	return true;
}

bool ImageBuffer::getPNG(const char*& buf, size_t& size, int compressionLevel) {
	if(!png.valid) {
		updateYUV();
		if(png.size()==0)
			png.resize(static_cast<size_t>(width*height*3*1.005 + 100));
		char * out = png.getBuffer();
		size_t outsize = png.size();
		png.used = image_util::encodePNG(yuv.getBuffer(), yuv.used, width, height, 3, out, outsize, 3, compressionLevel);
		png.valid=true;
	}
	buf=png.getBuffer();
	size=png.used;
	return size>0;
}

bool ImageBuffer::getJPG(const char*& buf, size_t& size, int quality) {
	if(!jpg.valid) {
		updateYUV();
		if(jpg.size()==0)
			jpg.resize(width*height*3 + 1024);
		char * out = jpg.getBuffer();
		size_t outsize = jpg.size();
		jpg.used = image_util::encodeJPEG(yuv.getBuffer(), yuv.used, width, height, 3, out, outsize, 3, quality);
		jpg.valid=true;
	}
	buf=jpg.getBuffer();
	size=jpg.used;
	return size>0;
}

//static inline unsigned char clip(unsigned short x) { return (x<=255 ? x : (x>(unsigned short)0x7FFF ? 0 : 255)); }
static inline unsigned char clip(unsigned short x) { return (x<=255 ? x : ((short)~x)>>16); }

/*! ideally this would be done by a Cg or GLSL during rendering, so our input would already be YUV444.
 * For now, convert on CPU (eww) */
void ImageBuffer::updateYUV() {
	if(!yuv.valid) {
		if(yuv.size()==0) {
			yuv.used = width*height*3;
			yuv.resize(yuv.used);
		}
		char* buf = yuv.getBuffer();
		const unsigned char * c = (unsigned char *)&pixels[0];	 
		const unsigned int stride=width*3;	 
		const unsigned char * ye = c + height*stride;	 
		while(c<ye) {	 
			while(c<ye) {	 
				short r = *c++; // red	 
				short g = *c++; // green	 
				short b = *c++; // blue	 
				
				// Conversion to YCbCr (aka digital YUV)	 
				// The Y channel is scaled to [16-235] (inclusive)	 
				// these formulas are from http://en.wikipedia.org/wiki/YCbCr
				/*
				 *buf++ = 16  + (   65.738  * r +  129.057  * g +  25.064  * b)/256;
				 *buf++ = 128 + ( - 37.945  * r -   74.494  * g + 112.439  * b)/256;
				 *buf++ = 128 + (  112.439  * r -   94.154  * g -  18.285  * b)/256;
				 */
				// here's a non-floating point form:
				*buf++ = clip(((66*r +  129*g +  25*b)>>8)+16); // Y	 
				*buf++ = clip(((-38*r -   74*g + 112*b)>>8)+128); // Cb	 
				*buf++ = clip(((112*r -   94*g -  18*b)>>8)+128); // Cr	 
			}	 
		}
		yuv.valid=true;
	}
}

/*! @file
 * @brief Implements ImageBuffer, for storing a rendered image and its compressed forms for transmission to a camera subscriber (CommThread::Subscription)
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
