//-*-c++-*-
#ifndef INCLUDED_RegionGenerator_h_
#define INCLUDED_RegionGenerator_h_

#include "Vision/FilterBankGenerator.h"
#include "Events/SegmentedColorFilterBankEvent.h"
#include "Vision/cmvision.h"

class RLEGenerator;

//! Connects regions of CMVision format runs in RLEGenerator
/*! Uses the CMVision library for main processing.
 *
 *  getImage() will return an array of RegionGenerator::region_stats -
 *  one entry per color.  This will give you some statistics on colors
 *  present, as well as the head of a linked list through regions of
 *  each color in the image.
 *  
 *  Uses fields in the RLEGenerator's runs to store region
 *  information.  This means we don't have to make an extra copy of
 *  the data, but unfortunately also means these two stages are
 *  tightly coupled...
 *
 *  Similarly, this also accesses the color information of the runs,
 *  so the events received must be a SegmentedColorFilterBankEvents so
 *  that it can interpret the color information.  Some statistical
 *  information will then be stored in the color struct to report the
 *  min region area, max region area, and number of regions.
 */
class RegionGenerator : public FilterBankGenerator {
public:
	typedef CMVision::region region; //!< using the CMVision library's region information
	typedef CMVision::color_class_state region_stats; //!< using the CMVision library's color struct to store the region info
	
	//! constructor
	RegionGenerator(unsigned int mysid, RLEGenerator* rleg, EventBase::EventTypeID_t tid);
	
	//! destructor
	virtual ~RegionGenerator() {
		freeCaches();
		destruct();
	}

	static std::string getClassDescription() { return "Connects runs in an RLE compressed image into regions"; }

	virtual void freeCaches();

	//! see class notes above for what data this can handle
	virtual void doEvent();

	virtual unsigned int getBinSize() const;
	virtual unsigned int loadBuffer(const char buf[], unsigned int len, const char* filename=NULL);
	virtual unsigned int saveBuffer(char buf[], unsigned int len) const;

	virtual size_t getImageSize(unsigned int /*layer*/, unsigned int /*chan*/) const { return sizeof(region_stats); }
	
protected:
	typedef SegmentedColorFilterBankEvent::color_class_state color_class_state; //!< use the same color info as SegmentedColorFilterBankEvent (since that's what's supplying the color info)
	static const unsigned int MAX_REGIONS=640*480/16;  //!< maximum number of regions, value is from historical Vision sources

	virtual void setNumImages(unsigned int nLayers, unsigned int nChannels);
	virtual unsigned char * createImageCache(unsigned int layer, unsigned int chan) const;
	virtual void calcImage(unsigned int layer, unsigned int chan);
	virtual void destruct();

	unsigned int srcNumColors; //!< number of colors available (from src->src, which should be SegmentedColorGenerator...)
	const color_class_state * srcColors; //!< colors information (from src->src, which should be SegmentedColorGenerator...)
	region *** regions; //!< data storage for region information

private:
	RegionGenerator(const RegionGenerator& fbk); //!< don't call
	const RegionGenerator& operator=(const RegionGenerator& fbk); //!< don't call
};

/*! @file 
 * @brief Describes RegionGenerator, which connects regions of CMVision format runs in RLEGenerator
 * @author alokl (Creator)
 * @author ejt (reorganized)
 */

#endif
