//-*-c++-*-
#ifndef INCLUDED_InterleavedYUVGenerator_h_
#define INCLUDED_InterleavedYUVGenerator_h_

#include "Vision/FilterBankGenerator.h"

//! Generates FilterBankEvents containing raw camera images with interleaved pixels (YUVYUVYUV... instead of YYY...UUU...VVV...)
/*!
 *  There's only one channel, which holds the interleaved data.  The
 *  increment is set to 3, but if you want to access each component in
 *  order, just use 1 instead (as you would expect hopefully, since
 *  the very specific memory layout is the whole point of this class)
 *  
 *  The row
 *  skip is always 0, and the row stride is always width*3.  But it
 *  would be better to use the proper accessor functions to be more
 *  general.
 *
 *	should receive FilterBankEvents from any standard format FilterBankGenerator (like RawCameraGenerator)
 *
 *  @see FilterBankGenerator for information on serialization format
 */
class InterleavedYUVGenerator : public FilterBankGenerator {
public:
	//! constructor
	InterleavedYUVGenerator(unsigned int mysid, FilterBankGenerator* fbg, EventBase::EventTypeID_t tid);
	//! constructor, you can pass which channels to interleave
	InterleavedYUVGenerator(unsigned int mysid, unsigned int syc, unsigned int suc, unsigned int svc, FilterBankGenerator* fbg, EventBase::EventTypeID_t tid);

	//! destructor
	virtual ~InterleavedYUVGenerator() {
		freeCaches();
		destruct();
	}

	static const unsigned int CHAN_YUV=0; //!< so you can refer to the YUV channel symbolically. (as opposed to others that might be added?)

	static std::string getClassDescription() { return "Converts a FilterBankGenerator's data into interleaved format"; }

	//! should receive FilterBankEvents from any standard format FilterBankGenerator (like RawCameraGenerator)
	virtual void doEvent();
	
	virtual unsigned int getBinSize() const;

	virtual unsigned int loadBuffer(const char buf[], unsigned int len, const char* filename=NULL);

	virtual unsigned int saveBuffer(char buf[], unsigned int len) const;

	virtual void freeCaches();
	virtual void invalidateCaches();

protected:
	virtual void setNumImages(unsigned int nLayers, unsigned int nChannels);
	virtual void setDimensions(); //!< resets stride parameter (to correspond to width*3 from FilterBankGenerator::setDimensions())
	virtual void destruct();
	virtual unsigned char * createImageCache(unsigned int layer, unsigned int chan) const;
	virtual void calcImage(unsigned int layer, unsigned int chan);

	unsigned int srcYChan; //!< the channel of the source's Y channel
	unsigned int srcUChan; //!< the channel of the source's U channel
	unsigned int srcVChan; //!< the channel of the source's V channel
	
	bool ** isAllocated; //!< flag for each image, set to true if the corresponding value in #images will need to be freed, or false if it's a passthrough from the previous stage

private:
	InterleavedYUVGenerator(const InterleavedYUVGenerator& fbk); //!< don't call
	const InterleavedYUVGenerator& operator=(const InterleavedYUVGenerator& fbk); //!< don't call
};

/*! @file 
 * @brief Describes InterleavedYUVGenerator, which generates FilterBankEvents containing raw camera images with interleaved pixels (YUVYUVYUV... instead of YYY...UUU...VVV...)
 * @author ejt (Creator)
 */

#endif
