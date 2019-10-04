#include "SIFTImage.h"
#include <cstdlib>

SIFTImage::SIFTImage(int i) : id(i), buffer(), gaussianSpace(), keypoints() {}

void SIFTImage::clearImage(){
	if (buffer.byteArray != NULL) {
		free(buffer.byteArray);
		buffer.byteArray = NULL;
	}
}
