#include "keypointpair.h"
#include "keypoint.h"
#include <cmath>

keypointPair::keypointPair(keypoint* k1, keypoint* k2)
	: key1(k1), key2(k2),
	xDiff(key2->modelX - key1->imageX),
	yDiff(key2->modelY - key1->imageY),
	scaleZoom(key2->modelScale / key1->imageScale),
	orientationDiff(key2->modelOrientation - key1->imageOrientation)
{
	while (orientationDiff > M_PI) orientationDiff -= (2 * M_PI);
	while (orientationDiff <= -M_PI) orientationDiff += (2 * M_PI);
}

keypointPair::keypointPair(keypointPair* k)
: key1(k->key1), key2(k->key2), 
xDiff(k->xDiff), yDiff(k->yDiff), 
scaleZoom(k->scaleZoom), orientationDiff(k->orientationDiff) {}

void keypointPair::getForwardTransform(double* xTranslate, double* yTranslate, double* zoom, double* rotation){
	*xTranslate = xDiff;
	*yTranslate = yDiff;
	*zoom = scaleZoom;
	*rotation = orientationDiff;
}

void keypointPair::getBackwardTransform(double* xTranslate, double* yTranslate, double* zoom, double* rotation){
	*xTranslate = -xDiff;
	*yTranslate = -yDiff;
	*zoom = 1.0/scaleZoom;
	*rotation = -orientationDiff;
}

