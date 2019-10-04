#ifndef RAW_IMAGE_PYRAMID_H
#define RAW_IMAGE_PYRAMID_H

#include "RawImage.h"

class RawImagePyramid {

public:
  static const unsigned int NUM_PYRAMID_LAYERS = 4;

  RawImagePyramid();
  ~RawImagePyramid();

  RawImage& operator[] (unsigned int index) { return *layers[index]; }

  void loadFromRawY();

  void printLayer(unsigned int layer);

private:
  std::vector<RawImage*> layers;

};


#endif
