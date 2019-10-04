//-*-c++-*-
#ifndef INCLUDED_JPEGGenerator_h_
#define INCLUDED_JPEGGenerator_h_

#include "Vision/FilterBankGenerator.h"
extern "C" {
#include <jpeglib.h>
}

//! Generates FilterBankEvents containing JPEG compressed images
/*! There's only one channel per resolution layer, which holds the
 *  compressed data.  This is mostly intended for being sent over
 *  wireless, but if you know how JPEG works, you may be able to
 *  interpret the compressed data directly and glean useful
 *  information from it.  After all, compression is all about throwing
 *  away unimportant details and storing the salient information.
 *  
 *  The generated events use 0 for their event source IDs.  The row
 *  skip and row stride are 0. (they don't really apply here)
 *
 *  This can either compress a greyscale image or an interleaved YUV
 *  image.  If the source generator's type is not
 *  InterleavedYUVGenerator, it will assume greyscale.  Call
 *  setSource() to override this.
 *
 *  The InterleavedYUVGenerator is separated from this because it
 *  wouldn't really make things much faster to combine the algorithms,
 *  and others might find the interleaved format handy for passing to
 *  other libraries which expect that format, such as what happened
 *  with libjpeg.
 *  
 *  This class shares a lot of non-JPEG specific code with PNGGenerator,
 *  so you may want to replicate any changes made in that class as well.
 *
 *  @todo possible speedup by using jpeg_write_raw_data
 *  @todo create a common super class with PNGGenerator to hold common setup code
 */
class JPEGGenerator : public FilterBankGenerator {
public:
	static const unsigned int JPEG_HEADER_PAD=500; //!< add a bit to the expected size in getBinSize just to leave a little extra room for small images
	
	//! defines how to interpret the source images (see #srcMode and #curMode)
	enum src_mode_t {
		SRC_AUTO,       //!< if src is not a InterleavedYUVGenerator, SRC_GREYSCALE, otherwise SRC_COLOR
		SRC_GRAYSCALE,  //!< indicates each channel of source should be compressed individually as grayscale images
		SRC_COLOR       //!< indicates first channel should be in an interleaved layout which can be compressed into a "color" image
	};

	//! constructor
	JPEGGenerator(unsigned int mysid, FilterBankGenerator* fbg, EventBase::EventTypeID_t tid);
	//! constructor
	JPEGGenerator(unsigned int mysid, JPEGGenerator::src_mode_t sMode, FilterBankGenerator* fbg, EventBase::EventTypeID_t tid);

	//! destructor
	virtual ~JPEGGenerator();

	//! set #srcMode and #curMode as well if @a mode==SRC_AUTO
	virtual void setSourceMode(src_mode_t mode) { srcMode=mode; if(mode!=SRC_AUTO) curMode=mode;}
	//! returns #srcMode
	virtual src_mode_t getSourceMode() const { return srcMode; }
	//! returns #curMode
	virtual src_mode_t getCurrentSourceFormat() const { return curMode; }

	static std::string getClassDescription() { return "Compresses its source FilterBankGenerator's data into JPEG format"; }

	//! should receive FilterBankEvents from a RawCameraGenerator (or a subclass thereof)
	virtual void doEvent();
	
	//! if we don't already know bytesUsed, let's assume the size will be smaller than the original uncompressed. If we fail this assumption, probably better to fail anyway.
	virtual unsigned int getBinSize() const;
	
	virtual unsigned int loadBuffer(const char buf[], unsigned int len, const char* filename=NULL);
	
	//! you probably don't want to be calling this to access the JPEG -- use getImage() instead (saveBuffer will prepend some header information before the actual image data)
	virtual unsigned int saveBuffer(char buf[], unsigned int len) const;

	//! returns #quality
	virtual unsigned int getQuality() const { return quality; }
	//! sets #quality; this will invalidate the cache if @a q does not equal current #quality
	virtual void setQuality(unsigned int q) {
		if(quality!=q) {
			quality=q; 
			invalidateCaches();
		}
	}

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

	struct jpeg_compress_struct cinfo; //!< used to interface with libjpeg - holds compression parameters and state
	struct jpeg_error_mgr jerr;        //!< used to interface with libjpeg - gives us access to error information

	unsigned int quality; //!< quality level to pass to libjpeg; -1U causes Config::vision_config::RawCam::compress_quality to be used
	
private:
	JPEGGenerator(const JPEGGenerator& fbk); //!< don't call
	const JPEGGenerator& operator=(const JPEGGenerator& fbk); //!< don't call
};

/*! @file 
 * @brief Describes JPEGGenerator, which generates FilterBankEvents containing JPEG compressed images
 * @author ejt (Creator)
 */

#endif
