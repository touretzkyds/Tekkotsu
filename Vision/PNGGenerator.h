//-*-c++-*-
#ifndef INCLUDED_PNGGenerator_h_
#define INCLUDED_PNGGenerator_h_

#include "Vision/FilterBankGenerator.h"
	
//! Generates FilterBankEvents containing PNG compressed images
/*!
 *  This class shares a lot of non-PNG specific code with JPEGGenerator,
 *  so you may want to replicate any changes made in that class as well.
 *
 *  @todo create a common super class with JPEGGenerator to hold common setup code
*/
class PNGGenerator : public FilterBankGenerator {
public:
	static const unsigned int PNG_HEADER_PAD=500; //!< add a bit to the expected size in getBinSize just to leave a little extra room for small images
	
	//! defines how to interpret the source images (see #srcMode and #curMode)
	enum src_mode_t {
		SRC_AUTO,       //!< if src is not a InterleavedYUVGenerator, SRC_GREYSCALE, otherwise SRC_COLOR
		SRC_GRAYSCALE,  //!< indicates each channel of source should be compressed individually as grayscale images
		SRC_COLOR       //!< indicates first channel should be in an interleaved layout which can be compressed into a "color" image
	};
	
	//! constructor
	PNGGenerator(unsigned int mysid, FilterBankGenerator* fbg, EventBase::EventTypeID_t tid);
	//! constructor
	PNGGenerator(unsigned int mysid, PNGGenerator::src_mode_t sMode, FilterBankGenerator* fbg, EventBase::EventTypeID_t tid);

	//! destructor
	virtual ~PNGGenerator();

	//! set #srcMode and #curMode as well if @a mode==SRC_AUTO
	virtual void setSourceMode(src_mode_t mode) { srcMode=mode; if(mode!=SRC_AUTO) curMode=mode;}
	//! returns #srcMode
	virtual src_mode_t getSourceMode() const { return srcMode; }
	//! returns #curMode
	virtual src_mode_t getCurrentSourceFormat() const { return curMode; }
	
	static std::string getClassDescription() { return "Compresses its source FilterBankGenerator's data into PNG format"; }
	
	//! should receive FilterBankEvents from a RawCameraGenerator (or a subclass thereof)
	virtual void doEvent();
	
	//! if we don't already know bytesUsed, let's assume the size will be smaller than the original uncompressed. If we fail this assumption, probably better to fail anyway.
	virtual unsigned int getBinSize() const;
	
	virtual unsigned int loadBuffer(const char buf[], unsigned int len, const char* filename=NULL);
	
	//! you probably don't want to be calling this to access the PNG -- use getImage() instead (saveBuffer will prepend some header information before the actual image data)
	virtual unsigned int saveBuffer(char buf[], unsigned int len) const;

	//! returns the number of bytes used for the image returned by getImage() - will return 0 if the image hasn't been calculated yet (so call it @e after getImage())
	virtual size_t getImageSize(unsigned int layer, unsigned int chan) const { return bytesUsed[layer][chan]; }
	
protected:
	virtual void setNumImages(unsigned int nLayers, unsigned int nChannels);
	virtual unsigned char * createImageCache(unsigned int layer, unsigned int chan) const;
	virtual void calcImage(unsigned int layer, unsigned int chan);
	virtual void destruct();
	
	src_mode_t srcMode;   //!< how to interpret source channel of next filter bank event
	src_mode_t curMode;   //!< how to interpret getImage's current image
	
	unsigned int ** bytesUsed; //!< number of bytes used per image to actually store data;
		
private:
	PNGGenerator(const PNGGenerator& fbk); //!< don't call
	const PNGGenerator& operator=(const PNGGenerator& fbk); //!< don't call
};

/*! @file
 * @brief Describes PNGGenerator, which generates FilterBankEvents containing PNG compressed images
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
