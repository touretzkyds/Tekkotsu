#include "SiftMatch.h"
#include "Vision/SIFT/SIFTDatabase/keypoint.h"
#include <iostream>

using namespace std;

SiftMatch::SiftMatch()
	: objectID(-1), objectName(), modelID(-1), modelName(), probOfMatch(), error(), 
	scale(), orientation(), columnTranslation(), rowTranslation(), inliers(),
	topLeft(), topRight(), bottomLeft(), bottomRight()
{}

void SiftMatch::print(const char* frontpadding) {
	cout << frontpadding << "objectID:          " << objectID << endl;
	cout << frontpadding << "objectName:        " << objectName << endl;
	cout << frontpadding << "modelID:           " << modelID << endl;
	cout << frontpadding << "modelName:         " << modelName << endl;
	cout << frontpadding << "probOfMatch:       " << probOfMatch << endl;
	cout << frontpadding << "error:             " << error << endl;
	cout << frontpadding << "scale:             " << scale << endl;
	cout << frontpadding << "orientation:       " << orientation << endl;
	cout << frontpadding << "columnTranslation: " << columnTranslation << endl;
	cout << frontpadding << "rowTranslation:    " << rowTranslation << endl;
	cout << frontpadding << "topLeft:           " << topLeft << endl;
	cout << frontpadding << "topRight:          " << topRight << endl;
	cout << frontpadding << "bottomLeft:        " << bottomLeft << endl;
	cout << frontpadding << "bottomRight:       " << bottomRight << endl;
}

void SiftMatch::computeBoundingBox() {
	double minX = inliers[0].getKey1()->modelX;
	double minY = inliers[0].getKey1()->modelY;
	double maxX = minX, maxY = minY;
	for ( vector<keypointPair>::const_iterator it = inliers.begin(); it != inliers.end(); it++ ) {
		double x = it->getKey1()->modelX;
		double y = it->getKey1()->modelY;
		if ( x < minX )
			minX = x;
		else if ( x > maxX )
			maxX = x;
		if ( y < minY )
			minY = y;
		else if ( y > maxY )
			maxY = y;
	}
	topLeft = DualCoding::Point(minX,minY,0,DualCoding::camcentric);
	topRight = DualCoding::Point(maxX,minY,0,DualCoding::camcentric);
	bottomLeft = DualCoding::Point(minX,maxY,0,DualCoding::camcentric);
	bottomRight = DualCoding::Point(maxX,maxY,0,DualCoding::camcentric);
}
