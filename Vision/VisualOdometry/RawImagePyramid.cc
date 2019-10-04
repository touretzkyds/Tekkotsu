#include "RawImagePyramid.h"

#include "Shared/RobotInfo.h"

RawImagePyramid::RawImagePyramid() : layers(NUM_PYRAMID_LAYERS) {
  int currWidth = RobotInfo::CameraResolutionX/2;
  int currHeight = RobotInfo::CameraResolutionY/2;

  for(unsigned int layer = 0; layer < NUM_PYRAMID_LAYERS; layer++) {
    layers[layer] = new RawImage(currWidth,currHeight);

    currWidth = (currWidth+1)/2;
    currHeight = (currHeight+1)/2;
  }
}

RawImagePyramid::~RawImagePyramid() {
  for(unsigned int i = 0; i < NUM_PYRAMID_LAYERS; i++) {
    delete layers[i];
  }

  layers.clear();
}

void RawImagePyramid::loadFromRawY() {
  layers[0]->loadFromRawY();

  for(unsigned int i = 1; i < NUM_PYRAMID_LAYERS; i++) {
    layers[i-1]->buildNextLayer(*layers[i]);
  }
}

void RawImagePyramid::printLayer(unsigned int layer) {
  layers[layer]->printImage();
}

