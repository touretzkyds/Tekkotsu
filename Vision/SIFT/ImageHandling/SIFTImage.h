#ifndef __SIFTIMAGE_H
#define __SIFTIMAGE_H

#include <vector>

#include "ImageBuffer.h"

class SIFTImage {
public:
	int id;
	ImageBuffer buffer;
	std::vector< std::vector< std::vector<int> > > gaussianSpace;
	std::vector<keypoint*> keypoints;
	
	SIFTImage(int i = -1);
	
	void clearImage();
};

#endif
