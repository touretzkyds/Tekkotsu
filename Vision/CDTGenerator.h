//-*-c++-*-
#ifndef INCLUDED_CDTGenerator_h_
#define INCLUDED_CDTGenerator_h_

#include "Vision/FilterBankGenerator.h"

class OFbkImageInfo;

//! Generates SegmentedColorFilterBankEvents with images provided from the system
/*! The Aibo has hardware-level color segmentation based on
 *  rectangular regions of UV color space.  This less flexible than
 *  the CMVision segmentation routines, which allow arbitrary shapes
 *  in color space, but the CDT (Color Detection Table) implementation
 *  has the major advantage of being done in hardware.
 *
 *  This class doesn't do any processing of its own, it simply
 *  provides access to the CDT segmented images generated by the
 *  system.
 *  
 *  To use this, you will first have to send the system the color
 *  regions that define the CDT.  You'll need to look in the OPEN-R
 *  documentation regarding the OCdtVectorData data structure and the
 *  New/Set/DeleteCdtVectorData() functions.
 *  
 *  For now, the color information in the Segmented Color Events will
 *  be NULL, so you won't be able to connect it to the rest of the
 *  CMVision stages unless you fill it in.  Filling in the color
 *  information will require modifications of this source - I'm
 *  consciously cutting some corners to get this out the door sooner.
 *  Hopefully someone who needs this capability can pick up where I
 *  left off.  It would be nice if someone made a version of
 *  CMVision's .col file format which also held the CDT information so
 *  the whole thing could be set up with one file load.
 *
 *  Only the actual layers sent by the system will be referenced - no
 *  double resolution layer.  Much like RawCameraGenerator, the extra
 *  subsampled images are simply using a larger interleave value.  If
 *  you want the images in continuous memory, you'll need to pass this
 *  through a de-interleaver stage.
 *  
 *  Only one channel is available.
 */
class CDTGenerator : public FilterBankGenerator {
public:
	//!constructor
	CDTGenerator(unsigned int numRawLayers, unsigned int numCalcLayers, unsigned int mysid, EventBase::EventGeneratorID_t gid, unsigned int sid);

	//! destructor
	virtual ~CDTGenerator() {
		freeCaches();
		destruct();
	}
	
	static std::string getClassDescription() { return "Extracts the segmented image channel (CDT) from the system's OFbkImageVectorData"; }

	//! holds id values for specifying image channel/bands
	enum channel_id_t {
		CHAN_CDT,    //!< Color Detection Table, segmented image
		NUM_CHANNELS //!< number of channels per resolution layer
	};

	//! need to override EventGeneratorBase's lazy listening -- as a root stage, need to remember each frame, just in case it might be used
	virtual void doStart() { FilterBankGenerator::doStart(); addSrcListener(); }

	//! called with system's image info
	virtual void doEvent();
	
	//! the memory for all layers was allocated by system, so we just set them to NULL before calling FilterBankGenerator::freeCaches() so it won't try to delete them
	virtual void freeCaches();

	virtual unsigned int getBinSize() const;

	virtual unsigned int loadBuffer(const char buf[], unsigned int len, const char* filename=NULL);

	virtual unsigned int saveBuffer(char buf[], unsigned int len) const;

protected:
	virtual void setNumImages(unsigned int nLayers, unsigned int nChannels);

	//! resets the current width, height, stride, and skip for all of the layers
	virtual void setDimensions();
	
	virtual unsigned char * createImageCache(unsigned int layer, unsigned int chan) const;
	virtual void calcImage(unsigned int layer, unsigned int chan);
	virtual void destruct();
	
	unsigned int numRealLayers; //!< the number of actual layers to expect from the system
	
	unsigned char ** layers; //!< an array of pointers to actual system memory for each layer
	const OFbkImageInfo ** imageInfos; //!< information about image properties

private:
	CDTGenerator(const CDTGenerator& fbk); //!< don't call
	const CDTGenerator& operator=(const CDTGenerator& fbk); //!< don't call
};

/*! @file 
 * @brief Describes CDTGenerator, which generates SegmentedColorFilterBankEvents with images provided from the system
 * @author ejt (Creator)
 */

#endif