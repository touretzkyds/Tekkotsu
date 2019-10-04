//-*-c++-*-
#ifndef INCLUDED_SketchTypes_h
#define INCLUDED_SketchTypes_h

#include <string>

namespace DualCoding {

typedef unsigned char uchar;
typedef unsigned short int usint;
typedef unsigned int uint;

#if defined(TGT_ERS7) || defined(TGT_ERS210) || defined(TGT_ERS220) // save memory when running onboard aibo
  typedef unsigned short int skindex;
#else
  typedef unsigned int skindex;
#endif

enum SketchType_t {
  sketchUnknown = 0,
  sketchBool,
  sketchUchar,
  sketchUsint,
  sketchUint,
  sketchFloat,
  sketchYUV,
};

const std::string SketchTypeNames[] = {"unknown","bool","uchar","usint","uint","float"};

enum ColorMapType_t {
  segMap = 0,	// use the color segmentation table (default)
  grayMap = 1,	// gray scale image
  jetMap = 2,	// Matlab-style blue to red color map
  jetMapScaled = 3  // blue to red map that  scales the spectrum to the image
};

} // namespace

using DualCoding::uint;

#endif
