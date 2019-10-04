#include "RawImage.h"

#include "DualCoding/DualCoding.h"
#include "Shared/RobotInfo.h"

using namespace DualCoding;

RawImage::RawImage(unsigned int pWidth, unsigned int pHeight) : 
  width(pWidth), height(pHeight),
  imageData(pWidth * pHeight) {}

unsigned int RawImage::clampX(int x) {
  if (x < 0)
    return 0;
  else if ((unsigned int)x >= width)
    return width-1;
  else
    return x;
}

unsigned int RawImage::clampY(int y) {
  if (y < 0)
    return 0;
  else if ((unsigned int)y >= height)
    return height-1;
  else
    return y;
}

/**

   Very Important Note:
   
   This works in x,y coordinates instead of in row,width coordinates.

 */
float& RawImage::operator() (int x, int y) {
  //y - row
  //x - column
  return imageData[clampY(y) * width + clampX(x)];
}

float& RawImage::operator() (unsigned int x, unsigned int y) {
  return imageData[clampY(y) * width + clampX(x)];
}


/**
   This is an implementation of bilinear interpolation in
   order to handle non-integer pixel indices.
 */
float RawImage::operator() (float x, float y) {
  int x_int, y_int;
  float x_remain, y_remain;

  x_int = (int)floor(x);
  y_int = (int)floor(y);
  x_remain = x - x_int;
  y_remain = y - y_int;

  return (1-x_remain)*(1-y_remain)*(*this)(x_int, y_int) +
    x_remain*(1-y_remain)*(*this)(x_int +1,y_int) + 
    (1-x_remain)*y_remain*(*this)(x_int,y_int + 1) +
    x_remain*y_remain*(*this)(x_int+1,y_int+1);
}

void RawImage::loadFromRawY() {
  int const incr = ProjectInterface::defRawCameraGenerator->getIncrement(ProjectInterface::halfLayer);
  int const skip = ProjectInterface::defRawCameraGenerator->getSkip(ProjectInterface::halfLayer);
  uchar* chan_ptr = ProjectInterface::defRawCameraGenerator->getImage(ProjectInterface::halfLayer, 
								      RawCameraGenerator::CHAN_Y);

  if (chan_ptr != NULL) {
    chan_ptr -= incr;

    for (unsigned int row = 0; row < height; row++) {
      for (unsigned int col = 0; col < width; col++) {
	imageData[row * width + col] = (float)*(chan_ptr += incr);
      }
      chan_ptr += skip;
    }
  }
}

void RawImage::buildNextLayer(RawImage &img) {
  //note that img must have dimensions width+1/2,height+1/2
  unsigned int nextWidth = (width+1)/2;
  unsigned int nextHeight = (height+1)/2;

  for (unsigned int x = 0; x < nextWidth; x++) {
    for (unsigned int y = 0; y < nextHeight; y++) {
      img(x,y) = .25 * (*this)(2*x,2*y) +
	.125 * ((*this)(2*x - 1, 2*y) + (*this)(2*x+1,2*y) +
		(*this)(2*x,2*y-1) + (*this)(2*x,2*y+1)) +
	.0625 * ((*this)(2*x-1,2*y-1) + (*this)(2*x+1,2*y+1) +
		 (*this)(2*x-1,2*y+1) + (*this)(2*x+1,2*y-1));
    }
  }
}

void RawImage::printImage() {
  for (unsigned int x = 0; x < width; x++) {
    for (unsigned int y = 0; y < height; y++) {
      cout << (*this)(x,y) << endl;
    }
  }
}

fmat::Column<2> RawImage::gradient(float x, float y) {
  float gradX = ( (*this)(x+1,y) - (*this)(x-1,y) ) / 2.0;
  float gradY = ( (*this)(x,y+1) - (*this)(x,y-1) ) / 2.0;

  return fmat::pack(gradX, gradY);
}

fmat::Matrix<2,2> RawImage::gradientCov(float x, float y) {
  float gradX = ( (*this)(x+1,y) - (*this)(x-1,y) ) / 2.0;
  float gradY = ( (*this)(x,y+1) - (*this)(x,y-1) ) / 2.0;

  fmat::Matrix<2,2> cov;
  cov(0,0) = gradX*gradX;
  cov(1,1) = gradY*gradY;
  cov(0,1) = gradX*gradY;
  cov(1,0) =  cov(0,1);
  
  return cov;
}

fmat::Matrix<2,2> RawImage::spatialGradientMatrix(float center_x,
						  float center_y,
						  int window) {
  fmat::Matrix<2,2> M;
  for (float x = center_x - window; x <= center_x + window; x++) {
    for (float y = center_y - window; y <= center_y + window; y++) {
      M += gradientCov(x,y);
    }
  }

  return M;
}

float RawImage::imageScore(float x, float y, int window) {
  fmat::Matrix<2,2> M = spatialGradientMatrix(x,y,window);
  float d = fmat::det(M);
  float b = M(0,0) + M(1,1);

  if (d == 0.0f || (b*b - 4 * d) < 0)
    return 0;
  else
    return min(fabs((b + sqrt(b*b - 4 * d))/2), fabs((b - sqrt(b*b - 4 * d))/2));
}
