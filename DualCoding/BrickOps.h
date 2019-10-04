//-*-c++-*-
#ifndef _BRICK_OPS_H_
#define _BRICK_OPS_H_

#include "Point.h"
#include "Sketch.h"
#include "BlobData.h"

namespace DualCoding {

  /* A collection of functions used in extracting corners from blobs 
   * and extracting bricks and pyramids from the image. 
   */
  
  
  Point findEdgePoint(Point start, Point end, Sketch<bool>& rendering);
  
  int findRadialDistancesFromPoint(Point center, float radius, 
				   Sketch<bool>& rendering,
				   float distances[], std::vector<Point>& points);
  
  void takeDerivative(float x[], float dx[], int len) ;
  
  void drawHist(float distances[], unsigned int len, Sketch<bool>& parent) ;
  
  void applyGaussian(float x[], float gx[], int len);

  float getBoundingQuadrilateralScore(BlobData &blob, std::vector<Point>& corners, Sketch<bool> edgeImage, 
				      int& borderCount, ShapeSpace &space);

  float getBoundingQuadrilateralInteriorPointRatio(BlobData &blob, 
						   std::vector<Point>& corners, 
						   ShapeSpace &space);

  float getQuadrilateralArea(std::vector<Point>& corners);

  int countBorderPixelFit(BlobData &blob, const std::vector<Point> &corners, 
			  Sketch<bool> edges, ShapeSpace &space);
     
  int pickMove(std::vector<float> scores, float weight);
}

#endif /* _BRICK_OPS_H_ */
