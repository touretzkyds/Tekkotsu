#include "ImageBuffer.h"
#include <cstdlib>

void convertPGMImgToImageBuffer(PGMImg& pgmImg, ImageBuffer& buffer){
	buffer.width = pgmImg.getWidth();
	buffer.height = pgmImg.getHeight();
	buffer.byteArray = (unsigned char*)malloc(sizeof(unsigned char) * buffer.width * buffer.height);
	int ptr = 0;
	for (int i = 0; i < buffer.height; i++){
		for (int j = 0; j < buffer.width; j++){
			buffer.byteArray[ptr++] = pgmImg.getPixel(j, i);
		}
	}
}
