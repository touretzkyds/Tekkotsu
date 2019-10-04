//-*-c++-*-
#ifndef INCLUDED_FilterBankGenerator_h_
#define INCLUDED_FilterBankGenerator_h_

#include "Events/EventGeneratorBase.h"
#include "Shared/LoadSave.h"

//! Abstract base class for generators of FilterBankEvent's
/*! This is needed to provide an interface for the FilterBankEvent to
 *  call back when the actual image data is requested from it.  This
 *  facilitates lazy calculation of image data...  no sense in
 *  processing layers or channels which aren't actually going to be
 *  used...
 *
 *  Also this way we save on allocating/deallocating large memory
 *  blocks on each event... the buffers allocated here can be reused
 *  frame to frame.
 *
 *  Larger layer indicies generally indicate higher resolution images
 *  in a scaling pyramid, but you are free to store your own data
 *  however you wish.
 *
 *  <h3>Serialization Format</h3>
 *
 *  First, be sure to get a good overview of the LoadSave style.  Most
 *  serialization is handled using this interface.
 *
 *  When, for instance, RawCameraGenerator::saveBuffer() is called, it
 *  first calls it's super class, FilterBankGenerator::saveBuffer(),
 *  which will write out the general image information, common to all
 *  subclasses of FilterBankGenerator. (i'll cover the specifics in a
 *  second) Once that's done, the RawCameraGenerator adds it's own bit
 *  of header and then saves the image data itself.
 *
 *  Note that only a single channel is being saved at this point.  So
 *  for instance, all the Y information.  No interleaving is going
 *  on. (unless you're saving from InterleavedYUVGenerator of course,
 *  which treats the 3 interleaved channels as a single image)
 *  Otherwise,only one image (selected with selectSaveImage()) of the
 *  bank will loaded or saved at a time.
 *  
 *  So, anyway.  The first header will be the same for all
 *  FilterBankGenerator subclasses.  In the specification below, I'm
 *  going to use one field per line (the new lines are not literal,
 *  it's a binary stream).  Each field is of the form '<@c type:name>
 *  <i>(notes)</i>'
 *  
 *  FilterBankGenerator Header: (from FilterBankGenerator::saveBuffer())
 *  - <@c string: "FbkImage">  <i>(remember a 'string' is len+str+0; so this is the literal "\010\0\0\0FbkImage\0"; also remember "\010" is octal for 8)</i>
 *  - <@c unsigned @c int: width> 
 *  - <@c unsigned @c int: height> 
 *  - <@c unsigned @c int: image layer> 
 *  - <@c unsigned @c int: image channel> <i>(so notice you can tell which channel it is after it's been saved)</i>
 * 
 *  Generator Specific Header (selected examples follow, or similarly, any of the other generators)
 *  
 *  - RawCameraGenerator: (from RawCameraGenerator::saveBuffer())
 *    - <@c string: "RawImage">
 *    - <<tt>char[</tt>width<tt>*</tt>height<tt>]</tt>: image data> <i>(note, just once channel being stored)</i>
 *  - InterleavedYUVGenerator: (from InterleavedYUVGenerator::saveBuffer())
 *    - <@c string: "InterleavedYUVImage">
 *    - <<tt>char[</tt>width<tt>*</tt>height<tt>*3]</tt>: image data> <i>(in YVU order, technically YCbCr)</i>
 *  - SegmentedColorGenerator: (from SegmentedColorGenerator::saveBuffer())
 *    - <@c string: "SegColorImage">
 *    - <<tt>char[</tt>width<tt>*</tt>height<tt>]</tt>: image data> <i>(one byte per sample)</i>
 *    - <@c unsigned @c int: num_cols> <i>(number of different colors available)</i>
 *    - for each of num_col:
 *      - <@c char: red> <i>red color to use for display of this index</i>
 *      - <@c char: green> <i>green color to use for display of this index</i>
 *      - <@c char: blue> <i>blue color to use for display of this index</i>
 *  - RLEGenerator: (from RLEGenerator::saveBuffer())
 *    - <@c string: "RLEImage">  <i>(remember a 'string' is len+str+0; so this is the literal "\010\0\0\0RLEImage\0"; also remember "\010" is octal for 8)</i>
 *    - <@c unsigned @c int: num_runs> <i>(how many runs will follow)</i>
 *    - for each of num_runs:
 *      - <@c char: color> <i>(index value of color of run)</i>
 *      - <@c short: x> <i>(x position of start of run ("unknown" runs are skipped - assume index 0 for pixels which are jumped))</i>
 *      - <@c short: width> <i>(length of run, will not exceed remaining width of image)</i>
 *    - <i>notice there's no color information from RLE - it's not (shouldn't be) assuming anything about the data being compressed)</i>
 *
 *  However, while we're on the topic, I'll mention that although this
 *  is the same image format used for streaming to VisionGUI, there's
 *  a few more fields added by RawCam behavior or SegCam behavior at the
 *  beginning of each packet.  See those classes for more information
 *  on the wireless protocol. That should tell you everything you need
 *  to know to interpret the vision stream as well.
 *
 *  <h3>Adding New FilterBankGenerator Subclasses</h3>
 *
 *  If you're doing fancy memory stuff, you probably want to override
 *  the freeCaches() and destruct() functions so that the default
 *  implementation won't try to free something it shouldn't.  Don't
 *  forget to call them from your own destructor though, otherwise
 *  your versions won't get called before the default implementation's
 *  does.
 *
 *  If you want to be able to transmit or save your images, you will
 *  need to override the LoadSave functions (listed below) to provide
 *  your own code for interpreting the image data itself, and then
 *  create or modify a behavior to open a socket and transmit the
 *  information.  (you could do that from within the generator itself
 *  if you like)
 *
 *  You will probably also want to add a few extra functions to allow
 *  users to set compression/data format parameters.
 *
 *  @see RawCameraGenerator, SegmentedColorGenerator for the basic
 *  image access
 * 
 *  @see RLEGenerator, RegionGenerator for some relatively simple
 *  examples of vision stages if you want to make some of your own.
 */
class FilterBankGenerator : public EventGeneratorBase, public LoadSave {
public:
	// Constructors are all protected - doesn't make sense to
	// instantiate this class directly, you want to use a subclass

	//! destructor
	/*! Your own subclasses should also have destructors which call
	 *  freeCaches() and destruct().  Otherwise, if you override these
	 *  functions to delete any custom memory you allocate, those
	 *  implementations won't be called by this destructor... a
	 *  destructor ignores virtual functions, only calls at its own
	 *  class level.\n
	 *  So it really doesn't matter if you aren't allocating any extra
	 *  memory other than what's in the image cache, but it's still good
	 *  form just in case you add stuff later so you won't forget and
	 *  leak memory everywhere */
	virtual ~FilterBankGenerator() {
		freeCaches();
		destruct();
	}

	//! returns the generator this is receiving its events from (or the last one anyway)
	virtual const FilterBankGenerator * getSourceGenerator() const { return src; }

	//! returns the number of image layers (e.g. different resolutions available)
	virtual unsigned int getNumLayers() const { return numLayers; }

	//! returns the number of channels per image (e.g. Y, U, or V components)
	virtual unsigned int getNumChannels() const { return numChannels; }
	
	//! returns pointer to the beginning of the image data for the specified layer and channel
	/*! this will cause the data to be calculated and cached if it's not already available */
	virtual unsigned char * getImage(unsigned int layer, unsigned int channel);

	//! returns the number of bytes used for the data returned by getImage() - if the data varies in size (e.g. jpeg compression), will return 0 if the image hasn't been calculated yet (so call it @e after getImage())
	virtual size_t getImageSize(unsigned int layer, unsigned int /*chan*/) const { return widths[layer]*heights[layer]; }
	
	//! returns whether or not an image has already been calculated for the current frame
	/*! If you call this immediately after getImage() and this still returns false, 
	 *  then an error must have occurred during processing */
	virtual bool getImageCached(unsigned int layer, unsigned int channel) const { return imageValids[layer][channel]; }

	//! returns width (in samples) of the image in a given layer
	unsigned int getWidth(unsigned int layer) const { return widths[layer]; }

	//! returns height (in samples) of the image in a given layer
	unsigned int getHeight(unsigned int layer) const { return heights[layer]; }
	
	//! returns the bytes to skip from the one-past-end of a row to get the beginning of the next
	unsigned int getSkip(unsigned int layer) const { return skips[layer]; }
	
	//! returns the bytes to skip from the beginning of one row to get the beginning of the next
	/*! This is just for convenience; the stride is just the skip plus the width, but it's precomputed for you for speed and clarity */
	unsigned int getStride(unsigned int layer) const { return strides[layer]; }

	//! returns the increment (in bytes) to use to go from one sample to the next
	unsigned int getIncrement(unsigned int layer) const { return increments[layer]; }
	
	//! returns the frame number of the current frame, see #frameNumber
	unsigned int getFrameNumber() const { return frameNumber; }
	
	//! returns the number of frames processed, see #framesProcessed
	unsigned int getFramesProcessed() const { return framesProcessed; }
	
	//! returns a pointer to a particular sample; if you are using this in an inner loop, consider using the getSkip() and getIncrement() values to iterate with better performance
	/*! @param px      the horizontal pizel position, relative to left edge; no boundary checking is done, ranges 0 through width-1
	 *  @param py      the vertical pixel position, relative to top edge; no boundary checking is done, ranges 0 through height-1
	 *  @param layer   the resolution layer to extract from
	 *  @param channel the image channel to extract from */
	unsigned char * getPixel(unsigned int px, unsigned int py, unsigned int layer, unsigned int channel) { return getImage(layer,channel)+py*getStride(layer)+px*getIncrement(layer); }
	
	//! returns a pointer to a particular sample; if you are using this in an inner loop, consider using the getSkip() and getIncrement() values to iterate with better performance
	/*! @param x       the horizontal position, relative to center of the image, left edge is -1 and right edge is 1; no boundary checking is done
	 *  @param y       the vertical pixel position, relative to center of the image, top edge is the negative aspect ratio, bottom edge is positive aspect ratio; no boundary checking is done
	 *  @param layer   the resolution layer to extract from
	 *  @param channel the image channel to extract from
	 *
	 *  To keep the coordinate system square, the x is defined to range -1,1, but y's range depends on the
	 *  aspect ratio of the image, height/width.  Thus typically y will approx. -.75,.75 */
	unsigned char * getPixel(float x, float y, unsigned int layer, unsigned int channel) {
		unsigned int px,py;
		getPixelCoordinates(px,py,x,y,layer);
		return getPixel(px,py,layer,channel);
	}

	//! sets the pixel-coordinate px and py parameters to the corresponding value of x and y
	/*! @param[out] px      the pixel position, relative to left edge, positive right, ranges 0 through width-1
	 *  @param[out] py      the pixel position, relative to top edge, positive down, ranges 0 through height-1
	 *  @param[in]  x       the horizontal position, relative to center of the image, left edge is -1 and right edge is 1; no boundary checking is done
	 *  @param[in]  y       the vertical pixel position, relative to center of the image, top edge is the negative aspect ratio, bottom edge is positive aspect ratio; no boundary checking is done
	 *  @param[in]  layer   the resolution layer the pixel coordinates are relative to
	 *
	 *  To keep the coordinate system square, the x is defined to range -1,1, but y's range depends on the
	 *  aspect ratio of the image, height/width.  Thus typically y will approx. -.75,.75 */
	void getPixelCoordinates(unsigned int& px, unsigned int& py, float x, float y, unsigned int layer) const {
		//note width sets the scale for both, so coordinate system is square... is good? I'm up for debate.
		px=(unsigned int)((getWidth(layer)-1)*(x+1)/2+.5f); //+.5 to round to nearest
		float aspect=getHeight(layer)/(float)getWidth(layer);
		py=(unsigned int)((getHeight(layer)-1)*(y+aspect)/(aspect*2)+.5f);
	}
	
	//! sets the x and y parameters from the pixel-coordinates px and py
	/*! @param[out] x       the horizontal position, relative to center of the image, left edge is -1 and right edge is 1; no boundary checking is done
	 *  @param[out] y       the vertical pixel position, relative to center of the image, top edge is the negative aspect ratio, bottom edge is positive aspect ratio; no boundary checking is done
	 *  @param[in]  px      the pixel position, relative to left edge, positive right, ranges 0 through width-1
	 *  @param[in]  py      the pixel position, relative to top edge, positive down, ranges 0 through height-1
	 *  @param[in]  layer   the resolution layer the pixel coordinates are relative to
	 *
	 *  To keep the coordinate system square, the x is defined to range -1,1, but y's range depends on the
	 *  aspect ratio of the image, height/width.  Thus typically y will approx. -.75,.75 */
	void getRealCoordinates(float& x, float& y, unsigned int px, unsigned int py, unsigned int layer) const {
		//note width sets the scale for both, so coordinate system is square... is good? I'm up for debate.
		x=px/(float)(getWidth(layer)-1)*2-1;
		float aspect=getHeight(layer)/(float)getWidth(layer);
		y=py/(float)(getHeight(layer)-1)*aspect*2-aspect;
	}
	
	//! deletes storage of cached images and marks it invalid
	/*! you should override this if the images cache pointer isn't actually an array of bytes... 
	 *  Don't forget to call it in your subclass's destructor or your version won't get called... */
	virtual void freeCaches();

	//! marks all of the cached images as invalid (but doesn't free their memory)
	/*! You probably want to call this right before you send the FilterBankEvent */
	virtual void invalidateCaches();

	//! default implementation does a few common housekeeping chores for you - probably should just take a look at its code
	/*! It doesn't throw any events for you - that's probably the main
	 *  reason you'd still want to override it\n
	 *  Also, if your class has a set number of layers or channels - for
	 *  instance, always 1 channel like InterleavedYUVGenerator, you
	 *  should override setNumImages() to enforce that constraint by
	 *  throwing away the appropriate argument and passing the your own
	 *  value to the superclass implementation.*/
	virtual void doEvent();
	
	//!@name LoadSave interface

	virtual unsigned int getBinSize() const;

	virtual unsigned int loadBuffer(const char buf[], unsigned int len, const char* filename=NULL);

	virtual unsigned int saveBuffer(char buf[], unsigned int len) const;

	//! Not actually part of the LoadSave interface, but allows you to select which image of the bank will be saved
	/*! Calling this will also cause the image data for that image to be calculated,
	 *  otherwise saveBuffer won't have up-to-date data to save.
	 *  
	 *  When loading, the saved image's layer and channel will reset this */
	virtual void selectSaveImage(unsigned int layer, unsigned int channel) { selectedSaveLayer=layer; selectedSaveChannel=channel; getImage(layer,channel);}

	virtual unsigned int getSelectedSaveLayer() const { return selectedSaveLayer; } //!< returns layer to be saved, or layer of last image loaded
	virtual unsigned int getSelectedSaveChannel() const { return selectedSaveChannel; } //!< returns channel to be saved, or channel of last image loaded

	//@}


protected:
	//! constructor, separate class and instance names, with a raw event specification, excluding type typically for stages which reference the previous stage's data
	FilterBankGenerator(const std::string& instancename, EventBase::EventGeneratorID_t mgid, unsigned int msid, EventBase::EventGeneratorID_t srcegid, unsigned int srcsrc)
		: EventGeneratorBase(instancename, mgid, msid, srcegid, srcsrc),
			src(NULL), numLayers(0), numChannels(0), widths(NULL), heights(NULL), skips(NULL),
			strides(NULL), increments(NULL), images(NULL), imageValids(NULL), selectedSaveLayer(0),
			selectedSaveChannel(0), frameNumber(0), framesProcessed(0)
	{ }

	//! constructor, separate class and instance names, with a raw event specification, including type typically for stages which will store their own copy of the data
	FilterBankGenerator(const std::string& instancename, EventBase::EventGeneratorID_t mgid, unsigned int msid, EventBase::EventGeneratorID_t srcegid, unsigned int srcsrc, EventBase::EventTypeID_t srcetid)
		: EventGeneratorBase(instancename, mgid, msid, srcegid, srcsrc, srcetid),
			src(NULL), numLayers(0), numChannels(0), widths(NULL), heights(NULL), skips(NULL),
			strides(NULL), increments(NULL), images(NULL), imageValids(NULL), selectedSaveLayer(0),
			selectedSaveChannel(0), frameNumber(0), framesProcessed(0)
	{ }

	//! constructor, separate class and instance names, with a filter bank source, passes on all types typically for stages which reference the previous stage's data
	FilterBankGenerator(const std::string& instancename, EventBase::EventGeneratorID_t mgid, unsigned int msid, FilterBankGenerator * fbgsrc)
		: EventGeneratorBase(instancename, mgid, msid, fbgsrc!=NULL?fbgsrc->getGeneratorID():EventBase::numEGIDs, fbgsrc!=NULL?fbgsrc->getSourceID():0),
			src(fbgsrc), numLayers(0), numChannels(0), widths(NULL), heights(NULL), skips(NULL),
			strides(NULL), increments(NULL), images(NULL), imageValids(NULL), selectedSaveLayer(0),
			selectedSaveChannel(0), frameNumber(0), framesProcessed(0)
	{
		if(src!=NULL)
			setNumImages(src->getNumLayers(),src->getNumChannels());
	}

	//! constructor, separate class and instance names, with a filter bank source, accepts a particular type typically for stages which will store their own data
	FilterBankGenerator(const std::string& instancename, EventBase::EventGeneratorID_t mgid, unsigned int msid, FilterBankGenerator * fbgsrc, EventBase::EventTypeID_t etid)
		: EventGeneratorBase(instancename, mgid, msid, fbgsrc!=NULL?fbgsrc->getGeneratorID():EventBase::numEGIDs, fbgsrc!=NULL?fbgsrc->getSourceID():0,etid),
			src(fbgsrc), numLayers(0), numChannels(0), widths(NULL), heights(NULL), skips(NULL),
			strides(NULL), increments(NULL), images(NULL), imageValids(NULL), selectedSaveLayer(0),
			selectedSaveChannel(0), frameNumber(0), framesProcessed(0)
	{
		if(src!=NULL)
			setNumImages(src->getNumLayers(),src->getNumChannels());
	}

	//! resizes the filter bank information storage area, you should override this to do your setup and call it from your constructor
	/*! In general, it isn't expected that FilterBankGenerator's should
	 *  necessarily be dynamically resizeable (although it would be
	 *  nice), which is why this isn't public.  If yours is, just add
	 *  some pubic accessor functions which call this.  In general, the
	 *  included subclasses should be able to handle being resized, but
	 *  there's no reason to do so since the system won't be changing
	 *  its available resolutions at run time. 
	 *
	 *  The default implementation is a no-op if(numLayers==nLayers && numChannels==nChannels)
	 */
	virtual void setNumImages(unsigned int nLayers, unsigned int nChannels);

	//! resets width and height parameters to that of the #src
	/*! You'll probably want to override this to also set #skips and #strides */
	virtual void setDimensions();
	
	//! create new image data storage area for the cache - this called by getImage() only when the corresponding entry in images is NULL
	/*! You should return the pointer you want stored in images to be
	 *  returned by any calls to getFirstRow.  Interpretation of the
	 *  data it points to is dependant on the the generator which
	 *  creates it */
	virtual unsigned char * createImageCache(unsigned int layer, unsigned int channel) const=0;

	//! should calculate new image data, called by getImage() only when #imageValids indicates the image being requested is dirty (and only after getImage() has already called createImageCache())
	/*! This is where you'll want to put your user-specific code for calculating the image data */
	virtual void calcImage(unsigned int layer, unsigned int channel) =0;

	//! deletes the arrays
	virtual void destruct();

	//! updates the image data to make sure its up to date with what's available from the source
	/*! If someone calls getImage on a stage which hadn't been listening for
	 *  events (an optimization to save time when it doesn't have any listeners
	 *  of its own -- see EventGeneratorBase), then this will retroactively
	 *  pull image data from the source even though the event for it was missed
	 *
	 *  @return false if no image data is available yet, true otherwise*/
	virtual bool refresh();


	FilterBankGenerator * src; //!< the generator of the last FilterBankEvent received

	unsigned int numLayers;   //!< current number of layers available
	unsigned int numChannels; //!< current number of channels available

	unsigned int * widths;    //!< an array of size numLayers, width (in samples) in pixels of each layer
	unsigned int * heights;   //!< an array of size numLayers, height (in samples) in pixels of each layer
	unsigned int * skips;     //!< an array of size numLayers, skip (in bytes) from row end to next row begin
	unsigned int * strides;   //!< an array of size numLayers, stride (in bytes) from a given column in one row to the same column in the next row
	unsigned int * increments;//!< an array of size numLayers, increment (in bytes) to use to get from one sample to the next
	
	mutable unsigned char *** images; //!< an array [numLayers][numChannels], stores pointer to cached image data
	mutable bool ** imageValids;      //!< an array [numLayers][numChannels], entry is true if cached data is still valid

	unsigned int selectedSaveLayer;   //!< layer to be manipulated with the LoadSave functions
	unsigned int selectedSaveChannel; //!< channel to be manipulated with the LoadSave functions

	//! the frame number of last frame received by doEvent - subclasses will need to set to the source's frameNumber if they don't call FilterBankGenerator::doEvent()
	/*! The idea is to use this as a unique serial number for each
	 *	frame.  That way you can know if the current image in different
	 *	generators is actually the same camera image before you try to
	 *	compare or combine them.
	 *
	 *  You could also figure out the number of dropped frames by
	 *  subtracting framesProcessed from this value.  Give some leeway
	 *  however, because it takes the first 40-70 frames just to boot up
	 *  (when running on the aibo), so there's no way they can be
	 *  processed.
	 */
	unsigned int frameNumber; 

	//! the current frame number available from the system - subclasses which receive data directly from the system should set this (and should not use EventGeneratorBase's auto-listen to ensure this is accurate)
	static unsigned int sysFrameNumber;

	//! subclasses should increment this any time they make a new filter bank available
	/*! this is automatically incremented if you use the FilterBankGenerator::doEvent() */
	unsigned int framesProcessed; 

private:
	FilterBankGenerator(const FilterBankGenerator& fbk); //!< don't call
	const FilterBankGenerator& operator=(const FilterBankGenerator& fbk); //!< don't call
};

/*! @file
 * @brief Describes abstract base class for generators of FilterBankEvent's
 * @author ejt (Creator)
 */

#endif
