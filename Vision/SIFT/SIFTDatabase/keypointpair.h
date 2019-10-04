#ifndef __KEYPOINTPAIR_H
#define __KEYPOINTPAIR_H

class keypoint;

class keypointPair {
private:
	keypoint* key1;
	keypoint* key2;
	double xDiff, yDiff, scaleZoom, orientationDiff;
	
public:
	
	keypointPair(keypoint* k1, keypoint* k2);
	
	// copy constructor
	keypointPair(const keypointPair& o) : key1(o.key1), key2(o.key2), xDiff(o.xDiff), yDiff(o.yDiff), scaleZoom(o.scaleZoom), orientationDiff(o.orientationDiff) {}
	keypointPair(keypointPair* k);
	keypointPair& operator=(const keypointPair& o) {
		key1=o.key1;
		key2=o.key2;
		xDiff=o.xDiff;
		yDiff=o.yDiff;
		scaleZoom=o.scaleZoom;
		orientationDiff=o.orientationDiff;
		return *this;
	}
	
	keypoint* getKey1() const { return key1; }
	keypoint* getKey2() const { return key2; }
	
	void getForwardTransform(double* xTranslate, double* yTranslate, double* zoom, double* rotation);
	void getBackwardTransform(double* xTranslate, double* yTranslate, double* zoom, double* rotation);
};

#endif
