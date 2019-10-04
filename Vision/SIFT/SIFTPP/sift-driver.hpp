#ifndef __SIFTDRIVER_H
#define __SIFTDRIVER_H

#include "sift.hpp"
#include <vector>

// VL::Sift* siftdriver(int argc, char** argv, int* numKeypoints);
// VL::Sift* siftdriver(int argc, char** argv, int* numKeypoints, vector<double>* keyValues);
VL::Sift* siftdriver(int argc, char** argv, int* numKeypoints, std::vector<double>* keyValues, std::vector< std::vector< std::vector<int> > >& gaussianSpace);
VL::Sift* siftdriver(int argc, char** argv, int* numKeypoints, std::vector<double>* keyValues, std::vector< std::vector< std::vector<int> > >& gaussianSpace, VL::pixel_t* data, int width, int height);

#endif
