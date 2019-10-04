//-*-c++-*-
#ifndef INCLUDED_RawCameraGenerator_h_
#define INCLUDED_RawCameraGenerator_h_

#include "Vision/FilterBankGenerator.h"

#ifdef PLATFORM_APERIOS
// to get the default camera resolution for the static allocation hack
#  include "Shared/RobotInfo.h"
#endif

class OFbkImageInfo;

//! Generates FilterBankEvents containing raw camera images directly from the system (doesn't make a copy)
class RawCameraGenerator : public FilterBankGenerator {
public:
	//! constructor, numRawLayers is the number of real layers passed from the system, numCalcLayers is the total number of layers to make available
	/*! The extra calculated layers are simply created by giving larger
	 *  increments, they reference the same data (no computational cost,
	 *  marginal memory costs... it's just nice to have more layers in
	 *  the image pyramid.
	 *  
	 *  However you'll have to write your code to use the getIncrement()
	 *  value properly if you want it to work.  Otherwise, you could
	 *  write a de-interlace generator which will do the resampling.
	 *
	 *  The top most layer (largest index) is a double-scale image.  For
	 *  the Y-channel, it is reconstructed from the 4 Y channels.  For
	 *  the other channels, it just does a fast scaling (but these
	 *  operations do cost, so use the top layer conservatively.
	 *
	 *  @see FilterBankGenerator for information on serialization format
	 */
	RawCameraGenerator(unsigned int numRawLayers, unsigned int numCalcLayers,
			   unsigned int mysid, EventBase::EventGeneratorID_t gid, unsigned int sid);

	//! destructor
	virtual ~RawCameraGenerator();

	static std::string getClassDescription() { return "Translates OFbkImageVectorData objects from the system into a slightly more accessible FilterBankEvent for further processing"; }

	//! holds id values for specifying image channel/bands
	enum channel_id_t {
		CHAN_Y,      //!< Y (intensity) channel
		CHAN_U,      //!< Cb (approx. blue,green,yellow) channel
		CHAN_V,      //!< Cr (approx. pink,red,yellow) channel
		CHAN_Y_DY,   //!< vertical derivative of Y channel (aka LH)
		CHAN_Y_DX,   //!< horizontal derivative of Y channel (aka HL)
		CHAN_Y_DXDY, //!< vert. & horiz. derivatives of Y channel (aka HH)
		NUM_CHANNELS //!< number of channels per resolution layer
	};

	//! need to override EventGeneratorBase's lazy listening -- as a root stage, need to remember each frame, just in case it might be used
	virtual void doStart() { FilterBankGenerator::doStart(); addSrcListener(); }

	//! called with system's image info
	virtual void doEvent();
	
	//! the memory for all layers except the double layer was allocated by system, so we just set them to NULL before calling FilterBankGenerator::freeCaches() so it won't try to delete them
	virtual void freeCaches();

	virtual unsigned int getBinSize() const;

	virtual unsigned int loadBuffer(const char buf[], unsigned int len, const char* filename=NULL);

	virtual unsigned int saveBuffer(char buf[], unsigned int len) const;

	virtual unsigned int saveFileStream(FILE* f) const; //!< overrridden to allow saving direct to file without an extra buffer copy

protected:
#ifdef PLATFORM_APERIOS
	//! ohh, this is so yucky.  But we need to get around heap allocation bugs with large regions in Aperios :(
	/*! So we store the double size images statically within the class.
	 *  What a waste of memory since we're probably not using these very
	 *  much, and they are rather large... :(
	 * 
	 *  The 2.0 release had the proper dynamic allocation
	 *  implementation, but has been replaced in 2.0.1 with this static
	 *  allocation so it won't crash.*/
	unsigned char dblRes[NUM_CHANNELS][CameraResolutionX*2][CameraResolutionY*2]; 
#endif

	virtual void setNumImages(unsigned int nLayers, unsigned int nChannels);

	virtual void setDimensions(); //!< resets the width, height, skip and stride values
	virtual unsigned char * createImageCache(unsigned int layer, unsigned int chan) const;
	virtual void calcImage(unsigned int layer, unsigned int chan);
	virtual void destruct();

	//! returns the value to pass to OPENR functions corresponding to the desired channel_id_t.
	inline static unsigned int mapChannelID(channel_id_t chan);

	//! Creates a double resolution version of a channel, doesn't do any smoothing
	void upsampleImage(channel_id_t chan);

	//! Creates a double resolution Y-channel from the Y and Y gradient channels
	void reconstructImage();

	//! helper functions for reconstructImage() - see documentation for that function
	static inline unsigned char clipRange(int val) {
		if (val < 0)        { return 0;         }
		else if (val > 255) { return 255;       }
		else                { return (unsigned char)val; }
	}

	unsigned int numRealLayers; //!< the number of "real" layers being sent from the system

	unsigned char ** layers; //!< points to the image data supplied by the system, one per layer (all channels are interleaved by row)
	const OFbkImageInfo ** imageInfos; //!< image info provided by the system

private:
	RawCameraGenerator(const RawCameraGenerator& fbk); //!< don't call
	const RawCameraGenerator& operator=(const RawCameraGenerator& fbk); //!< don't call
};

/*! @file 
 * @brief Describes RawCameraGenerator, which generates FilterBankEvents containing raw camera images
 * @author ejt (Creator)
 */

#endif
