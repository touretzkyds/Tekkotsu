#ifndef __SIFTMATCH_H
#define __SIFTMATCH_H

#include "DualCoding/Point.h"
#include "Vision/SIFT/SIFTDatabase/keypointpair.h"
#include <vector>
#include <string>

class SiftMatch{
public:
	SiftMatch();
	
	void print(const char* frontpadding);
	
	int objectID;
	std::string objectName;
	int modelID;
	std::string modelName;
	double probOfMatch;
	double error;
	double scale, orientation, columnTranslation, rowTranslation;
	std::vector<keypointPair> inliers;
	// A bounding box normally requires just topLeft and bottomRight
	// points, but we eventually want to use a rotated bounding box
	// showing the model's orientation in the image; then the edges
	// won't be axis-aligned.
	DualCoding::Point topLeft, topRight, bottomLeft, bottomRight;
	
	void computeBoundingBox();
};

#endif
