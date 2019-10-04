#ifndef RAW_IMAGE_H
#define RAW_IMAGE_H

#include "Shared/fmat.h"
#include "Vision/RawCameraGenerator.h"

class RawImage {

public:
  RawImage(unsigned int width, unsigned int height);
  
  float& operator() (int x, int y);
  float& operator() (unsigned int x, unsigned int y);
  float operator() (float x, float y);

  fmat::Column<2> gradient(float x, float y);
  fmat::Matrix<2,2> gradientCov(float x, float y);
  fmat::Matrix<2,2> spatialGradientMatrix(float center_x,
					  float center_y,
					  int window);

  float imageScore(float x, float y, int window);

  void loadFromRawY();
  void buildNextLayer(RawImage &img);

  unsigned int getHeight() { return height; }
  unsigned int getWidth() { return width; }

  void printImage();

private:
  unsigned int width;
  unsigned int height;
  std::vector<float> imageData;

  unsigned int clampX(int x);
  unsigned int clampY(int y);
};

#endif
