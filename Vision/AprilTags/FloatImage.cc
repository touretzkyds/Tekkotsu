#include "FloatImage.h"
#include "Gaussian.h"
#include "DualCoding/Sketch.h"

namespace AprilTags {

FloatImage::FloatImage() : width(0), height(0), pixels() {}

FloatImage::FloatImage(int widthArg, int heightArg) 
  : width(widthArg), height(heightArg), pixels(widthArg*heightArg) {}

FloatImage::FloatImage(int widthArg, int heightArg, const std::vector<float>& pArg) 
  : width(widthArg), height(heightArg), pixels(pArg) {}

FloatImage::FloatImage(const DualCoding::Sketch<DualCoding::uchar>& sketch) 
  : width(sketch.width), height(sketch.height), pixels(sketchToFloats(sketch)) {}

FloatImage& FloatImage::operator=(const FloatImage& other) {
  width = other.width;
  height = other.height;
  if (pixels.size() != other.pixels.size())
    pixels.resize(other.pixels.size());
  pixels = other.pixels;
  return *this;
}

std::vector<float> FloatImage::sketchToFloats(const DualCoding::Sketch<DualCoding::uchar>& sketch) {
  const unsigned char *pix = sketch.data->getRawPixels();
  std::vector<float> result(sketch.width*sketch.height);
  // converts raw pixels to float array
  for (unsigned int i = 0; i < result.size(); i++)
    result[i] = float(pix[i]) / 255.0f;
  return result;
}

void FloatImage::decimateAvg() {
  int nWidth = width/2;
  int nHeight = height/2;

  for (int y = 0; y < nHeight; y++)
    for (int x = 0; x < nWidth; x++)
      pixels[y*nWidth+x] = pixels[(2*y)*width + (2*x)];

  width = nWidth;
  height = nHeight;
  pixels.resize(nWidth*nHeight);
}

void FloatImage::normalize() {
  const float maxVal = *max_element(pixels.begin(),pixels.end());
  const float minVal = *min_element(pixels.begin(),pixels.end());
  const float range = maxVal - minVal;
  const float rescale = 1 / range;
  for ( unsigned int i = 0; i < pixels.size(); i++ )
    pixels[i] = (pixels[i]-minVal) * rescale;
}

void FloatImage::filterFactoredCentered(const std::vector<float>& fhoriz, const std::vector<float>& fvert) {
  // do horizontal
  std::vector<float> r(pixels);

  for (int y = 0; y < height; y++) {
    Gaussian::convolveSymmetricCentered(pixels, y*width, width, fhoriz, r, y*width);
  }

  // do vertical
  std::vector<float> tmp(height); // column before convolution
  std::vector<float> tmp2(height); // column after convolution

  for (int x = 0; x < width; x++) {

    // copy the column out for locality
    for (int y = 0; y < height; y++)
      tmp[y] = r[y*width + x];

    Gaussian::convolveSymmetricCentered(tmp, 0, height, fvert, tmp2, 0);

    for (int y = 0; y < height; y++)
      pixels[y*width + x] = tmp2[y];
  }
}

void FloatImage::printMinMax() const {
  std::cout << "Min: " << *min_element(pixels.begin(),pixels.end()) << ", Max: " << *max_element(pixels.begin(),pixels.end()) << std::endl;
  //for (int i = 0; i < getNumFloatImagePixels(); i++)
  //  std::cout << "Index[" << i << "]: " << this->normalize().getFloatImagePixels()[i] << endl;
}

} // namespace
