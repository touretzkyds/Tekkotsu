#ifndef TAGDETECTOR_H
#define TAGDETECTOR_H

#include <vector>

#include "Vision/AprilTags/TagDetection.h"
#include "Vision/AprilTags/TagFamily.h"

class FloatImage;
namespace DualCoding {
	typedef unsigned char uchar;
	template<typename T> class Sketch;
}

namespace AprilTags {

class TagDetector {
public:
	
	//! Constructor
	TagDetector(const TagFamily &tagFamily) : thisTagFamily(tagFamily) {}
	
	const TagFamily &thisTagFamily;
	
	std::vector<TagDetection> extractTags(const DualCoding::Sketch<DualCoding::uchar> &rawY);
	
	std::vector<TagDetection> extractTags(const FloatImage& fimOrig);
	
};

} // namespace

#endif
