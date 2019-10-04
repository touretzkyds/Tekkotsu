//-*-c++-*-
#ifndef INCLUDED_RLEGenerator_h_
#define INCLUDED_RLEGenerator_h_

#include "Vision/FilterBankGenerator.h"
#include "Vision/cmvision.h"
#include <vector>

//! Generates RLE compressed FilterBankEvents (generally from indexed color images from, say, SegmentedColorGenerator)
/*! Uses the CMVision library for main processing.
 *
 *  getImage() will return the first run, from the upper left hand
 *  corner.  The type is RLEGenerator::run.
 *
 *  The RLE produced isn't quite optimal in terms of size.  To make it
 *  easier to directly process the RLE for recognition tasks, each run
 *  will be broken at the end of the row.  So a solid image will still
 *  contain <var>height</var> runs instead of just one.
 *
 *  Also, the run structures contain extra fields to be used for
 *  region connecting.  These fields aren't sent over wireless, and
 *  are filled in by the RegionGenerator.  I don't necessarily like
 *  the tight coupling between the RLE and Region Generators that this
 *  requires, but it saves a copy of the data and allows us to use
 *  CMVision instead of rewriting.
 *
 *  Note that since the amount of data for each row is variable
 *  (depends on the complexity of that row) the row stride and skip
 *  are useless.  You'll have to process the RLE yourself to find a
 *  given index.
 *
 *  If the incoming events is a SegmentedColorFilterBankEvents, then
 *  it will post a SegmentedColorFilterBank to retain additional color
 *  information.  If the event is of a different format, it will post
 *  a regular FilterBankEvent.
 *  
 *  Note that although you could hook this class up to a raw intensity
 *  image, it is primarily of use with segmented color images because
 *  it doesn't handle gradients or noise well at all - this type of
 *  encoding/compression assumes cartoonish images of large blocks of
 *  flat color.  However, if you have some kind of other preprocessing
 *  that also provides suitable data, this can encode it for you.
 *  
 *  The format used for serialization is: (code is in saveBuffer())
 *  - <@c FilterBankGenerator: superclass header> <i>(First saves the superclass's info)</i>
 *  - <@c string: "RLEImage">  <i>(remember a 'string' is len+str+0; so this is the literal "\010\0\0\0RLEImage\0"; also remember "\010" is octal for 8)</i>
 *  - <@c unsigned @c int: num_runs> <i>(how many runs will follow)</i>
 *  - for each of num_runs:
 *    - <@c char: color> <i>(index value of color of run)</i>
 *    - <@c short: x> <i>(x position of start of run ("unknown" runs are skipped - assume index 0 for pixels which are jumped))</i>
 *    - <@c short: width> <i>(length of run, will not exceed remaining width of image)</i> 
 *
 *  Note that the RLEGenerator doesn't save the color infomation
 *  regarding what each index value "means".  This is just a
 *  compression stage, pure and simple.  You'll need to look at the
 *  RLEGenerator's source (@e probably SegmentedColorGenerator, but
 *  doesn't @e have to be) to determine how to interpret the indicies.
 *
 *  @see SegCamBehavior for information on transmission over wireless.
 *
 *  @see FilterBankGenerator more information on serialization
 */
class RLEGenerator : public FilterBankGenerator {
public:
	typedef CMVision::uchar cmap_t; //!< the type to use for a color index
	typedef CMVision::run<cmap_t> run; //!< use the CMVision library's run structure
	
	//! constructor
	RLEGenerator(unsigned int mysid, FilterBankGenerator * fbg, EventBase::EventTypeID_t tid);
	
	//! destructor
	virtual ~RLEGenerator() {
		freeCaches();
		destruct();
	}

	static std::string getClassDescription() { return "Compresses a FilterBankGenerator's channels using run length encoding"; }

	//! should receive FilterBankEvents from any standard format FilterBankGenerator (like RawCameraGenerator)
	virtual void doEvent();

	virtual unsigned int getBinSize() const;
	virtual unsigned int loadBuffer(const char buf[], unsigned int len, const char* filename=NULL);
	virtual unsigned int saveBuffer(char buf[], unsigned int len) const;

	//! returns the number of runs for the image
	virtual unsigned int getNumRuns(unsigned int layer, unsigned int chan) { if(!imageValids[layer][chan]) getImage(layer,chan); return numRuns[layer][chan]; }
	//! returns the actual runs of the specified image -- returned pointer is to internal storage; do not free
	virtual run * getRuns(unsigned int layer, unsigned int chan) { return reinterpret_cast<run*>(getImage(layer,chan)); }
	//! returns a specific run of the specified image
	virtual const run& getRun(unsigned int layer, unsigned int chan, unsigned int i) { return reinterpret_cast<const run*>(getImage(layer,chan))[i]; }

	//! in case you have inserted or removed runs during postprocessing, call this to update the count
	virtual void setNumRuns(unsigned int layer, unsigned int chan, unsigned int num) { numRuns[layer][chan]=num; }

	virtual size_t getImageSize(unsigned int layer, unsigned int chan) const { return numRuns[layer][chan]*sizeof(run); }

protected:
	static const unsigned int MIN_EXP_RUN_LENGTH=8; //!< The expected minimum average length of each run
	static const unsigned int XMIT_BYTES_PER_RUN=sizeof(cmap_t)+sizeof(short)+sizeof(short); //!< number of bytes needed to send each run

	virtual void setDimensions(); //!< sets the width, height, skip and stride, as well as #maxRuns
	virtual void setNumImages(unsigned int nLayers, unsigned int nChannels);
	virtual unsigned char * createImageCache(unsigned int layer, unsigned int chan) const;
	virtual void calcImage(unsigned int layer, unsigned int chan);
	virtual void destruct();
	//! uses a heuristic to predict the maximum number of runs expected per layer
	unsigned int calcExpMaxRuns(unsigned int layer) const { return getWidth(layer)*getHeight(layer)/MIN_EXP_RUN_LENGTH; }

	unsigned int ** numRuns; //!< a matrix of ints, holds the number of used runs for each image
	unsigned int * maxRuns; //!< the maximum number of runs possible for each layer

private:
	RLEGenerator(const RLEGenerator& fbk); //!< don't call
	const RLEGenerator& operator=(const RLEGenerator& fbk); //!< don't call
};

/*! @file 
 * @brief Describes RLEGenerator, which generates RLE compressed FilterBankEvents (generally from indexed color images from, say, SegmentedColorGenerator)
 * @author alokl (Creator)
 * @author ejt (reorganized)
 */

#endif
