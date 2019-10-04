#ifndef __IMAGEBUFFER_H
#define __IMAGEBUFFER_H

#include "PGMImg.h"

/**
 ImageBuffer contains information about a gray-scale image.
 width:     width of the image
 height:    height of the image
 byteArray: row-major values of the pixels in the image, should be of size width*height
 **/
class ImageBuffer{
public:
	int width;
	int height;
	unsigned char* byteArray;
};

void convertPGMImgToImageBuffer(PGMImg& pgmImg, ImageBuffer& buffer);

#endif
