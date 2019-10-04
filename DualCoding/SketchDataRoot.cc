// These have to go in the .cc file because SketchSpace.h circularly depends
// on SketchData.h, so we can't reference members of SketchSpace in the
// SketchDataRoot.h file.

#include "SketchDataRoot.h"
#include "SketchSpace.h"  // can't include this in SketchDataRoot.h due to circularities
#include "ShapeRoot.h"
#include "SketchSpace.h"
#include "ViewerConnection.h"

#include "Shared/LoadSave.h"
#include "Shared/Config.h"
#include "Shared/get_time.h"
#include "Shared/ProjectInterface.h" // needed for defSegmentedColorGenerator
#include "Vision/SegmentedColorGenerator.h"

namespace DualCoding {

ShapeSpace& SketchDataRoot::getDualSpace() const { return space->getDualSpace(); }

const unsigned int SketchDataRoot::getWidth() const { return space->getWidth(); }
const unsigned int SketchDataRoot::getHeight() const { return space->getHeight(); }
const unsigned int SketchDataRoot::getNumPixels() const { return space->getNumPixels(); }

void SketchDataRoot::inheritFrom(const SketchDataRoot &parent) {
  setParentId(parent.getViewableId());
  setColor(parent.getColor());
  if ( getType() == sketchBool )
    setColorMap(segMap);
  else
    setColorMap(parent.getColorMap());
}

void SketchDataRoot::inheritFrom(const ShapeRoot &parent) {
  setParentId(parent->getViewableId());
  setColor(parent->getColor());
}

void SketchDataRoot::inheritFrom(const BaseData &parent) {
  setParentId(parent.getViewableId());
  setColor(parent.getColor());
}

void SketchDataRoot::V(std::string const &_name) {
  setViewable(true);
  if ( !_name.empty() ) setName(_name);
}

void SketchDataRoot::N(std::string const &_name) {
  setViewable(false);
  if ( !_name.empty() ) setName(_name);
}

void SketchDataRoot:: setColor(const std::string &colorname) {
  setColor(ProjectInterface::getColorRGB(colorname));
}

void SketchDataRoot::setColor(const color_index cindex) {
  setColor(ProjectInterface::getColorRGB(cindex));
}

Point SketchDataRoot::indexPoint(const int index) {
  float x = (float)indexX(index);
  float y = (float)indexY(index);
  fmat::Column<3> pt = space->getTmatinv() * fmat::pack(x,y,0);
  return Point(pt, space->getDualSpace().getRefFrameType());
}

#define SKETCHDATA_ENCODE(a) \
if(!LoadSave::encodeInc(a,buf,avail,"SketchData encode ran out of space at %s:%u\n",__FILE__,__LINE__)) return 0;
	
unsigned int SketchDataRoot::saveBuffer(char buf[], unsigned int avail) const
{
	char* packet = buf; // beginning of packet
	static int frameNum = 0; // should this become a static member variable?
	
	SKETCHDATA_ENCODE("TekkotsuImage");
	SKETCHDATA_ENCODE(Config::vision_config::ENCODE_SINGLE_CHANNEL);
	SKETCHDATA_ENCODE(Config::vision_config::RawCamConfig::COMPRESS_NONE);
	SKETCHDATA_ENCODE(getWidth());
	SKETCHDATA_ENCODE(getHeight());
	SKETCHDATA_ENCODE(get_time()); // is this what should be used for time stamp?
	SKETCHDATA_ENCODE(frameNum++);
	
	// encode filterbank info
	SKETCHDATA_ENCODE("FbkImage");
	SKETCHDATA_ENCODE(getWidth());
	SKETCHDATA_ENCODE(getHeight());
	SKETCHDATA_ENCODE(CAM_LAYER);
	SKETCHDATA_ENCODE(CAM_CHANNEL);
	
	// encode actual image data
	SKETCHDATA_ENCODE("SketchImage");
	SKETCHDATA_ENCODE((unsigned char)getType());
	const unsigned int imglength  = savePixels(buf,avail);
	if(imglength==0)
		return 0; // savePixels should have already reported the error
	avail-=imglength;
	buf+=imglength;
	
	// encode color table(same as color table of segmentedcolorgenerator for now
	(dynamic_cast<SegmentedColorGenerator*>(ProjectInterface::defSegmentedColorGenerator))->encodeColorsInc(buf,avail);
	return buf-packet;
}

} // namespace
