//-*-c++-*-
#ifndef INCLUDED_BufferedImageGenerator_h_
#define INCLUDED_BufferedImageGenerator_h_

#include "FilterBankGenerator.h"
#include "RawCameraGenerator.h"

//! Receives camera frames as they are loaded by the simulator -- or eventually other sources
class BufferedImageGenerator : public FilterBankGenerator {
public:
	//!Stores information about the current frame, (not the image itself, but meta data a pointer to it)
	struct ImageSource {
		//!constructor
		ImageSource() : width(0), height(0), channels(0), frameIndex(0), layer(0), img(NULL) {}
		//!copy constructor
		ImageSource(const ImageSource& src) : width(src.width), height(src.height), channels(src.channels), frameIndex(src.frameIndex), layer(src.layer), img(src.img) {}
		//!assignment operator
		ImageSource& operator=(const ImageSource& src) { width=src.width; height=src.height; channels=src.channels; frameIndex=src.frameIndex; layer=src.layer; img=src.img; return *this; } 
		unsigned int width; //!< the width of #img
		unsigned int height; //!< the height of #img
		unsigned int channels; //!< the number of color channels in #img
		unsigned int frameIndex; //!< the serial number of the current frame (should be a unique, increasing ID)

		//! indicates what resolution layer of the pipeline this should be used at
		/*! Negative values are interpreted as "from the top", so -1 is the topmost layer, -2 is next-to-top, and so on.\n
		 *  Non-negative values are interpreted as direct layer values, so 0 indicates bottommost layer, 1 indicates next-to-bottom, and so on. */
		unsigned int layer; 

		//! pointer to the first byte of the image buffer
		/*! img should be stored in a channel-interleaved format, e.g. RGBRGBRGB... */
		unsigned char * img;
	};

	//! constructor
	BufferedImageGenerator(const std::string& name,EventBase::EventGeneratorID_t mgid, unsigned int msid, unsigned int nLayers, EventBase::EventGeneratorID_t srcgid, unsigned int srcsid)
		: FilterBankGenerator(name,mgid,msid,srcgid,srcsid), imgsrc(), isAllocated(NULL)
	{ 
		/* As a root stage, we need to listen to all incoming image
		 * events, even if we don't currently have listeners of our own --
		 * This is just in case user code directly accesses a generator
		 * and we have to retroactively go back and dig up the previous
		 * frame */
		unsetAutoListen();
		
		setNumImages(nLayers,RawCameraGenerator::NUM_CHANNELS);
	}
	//!destructor
	virtual ~BufferedImageGenerator() { destruct(); }
	
	//! need to override EventGeneratorBase's lazy listening -- as a root stage, need to remember each frame, just in case it might be used
	virtual void doStart() { FilterBankGenerator::doStart(); addSrcListener(); }

	virtual void doEvent();
	
	virtual unsigned int getBinSize() const;
	
	virtual unsigned int loadBuffer(const char buf[], unsigned int len, const char* filename=NULL);
	
	virtual unsigned int saveBuffer(char buf[], unsigned int len) const;
	
	virtual unsigned int saveFileStream(FILE* f) const; //!< overrridden to allow saving direct to file without an extra buffer copy

	virtual void freeCaches();
	virtual void invalidateCaches();
	
protected:
	virtual unsigned char * createImageCache(unsigned int layer, unsigned int channel) const;
	virtual void calcImage(unsigned int layer, unsigned int channel) ;
	virtual void setDimensions();
	virtual void destruct();
	virtual void setNumImages(unsigned int nLayers, unsigned int nChannels);

	//! duplicates pixels to make a higher resolution version of @a srcLayer, @a chan into @a destLayer, @a chan
	/*! Doesn't do anything fancy like blurring or smoothing */
	virtual void upsampleImage(unsigned int srcLayer, unsigned int chan, unsigned int destLayer);
	//! averages blocks of pixels to make smaller images
	virtual void downsampleImage(unsigned int destLayer, unsigned int chan);
	//! calculates the x-derivative
	virtual void calcDx(unsigned int layer, unsigned int srcChan=RawCameraGenerator::CHAN_Y, unsigned int dstChan=RawCameraGenerator::CHAN_Y_DX);
	//! calculates the x-derivative
	virtual void calcDy(unsigned int layer, unsigned int srcChan=RawCameraGenerator::CHAN_Y, unsigned int dstChan=RawCameraGenerator::CHAN_Y_DY);
	//! calculates the diagonal derivative
	virtual void calcDxDy(unsigned int layer);
	
	ImageSource imgsrc; //!< the data storage of the current image (not the image itself, but meta data a pointer to it)
	bool ** isAllocated; //!< for each image in the filterbank, a bool to account whether the pointer is to an external resource or a self-allocated resource
	
private:
	BufferedImageGenerator(const BufferedImageGenerator&); //!< don't call
	BufferedImageGenerator& operator=(const BufferedImageGenerator&); //!< don't call
};

//! input from std stream, no-op except eat a pointer value
inline std::istream& operator>>(std::istream& s, BufferedImageGenerator::ImageSource& i) { void * x; return s>>x; }
//! output to std stream, just dumps address of structure
inline std::ostream& operator<<(std::ostream& s, const BufferedImageGenerator::ImageSource& i) { return s << &i; }

/*! @file
 * @brief Describes BufferedImageGenerator, which receives camera frames as they are loaded by the simulator -- or eventually other sources
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
