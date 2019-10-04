//-*-c++-*-
#ifndef INCLUDED_SegmentedColorGenerator_h_
#define INCLUDED_SegmentedColorGenerator_h_

#include "Vision/FilterBankGenerator.h"
#include "Vision/cmvision.h"
#include "Vision/colors.h"
#include <vector>

//! Generates FilterBankEvents indexed color images based on a color threshold file
/*! Pretty simple idea - use a big mapping of YUV values to lookup
 *  index values.
 *
 *  Threshold files are 16x64x64 = 64KB.  So each Y component is
 *  discretized into 16 levels, U and V into 64 each.  Then the
 *  appropriate element of the 3D matrix is looked up, which holds the
 *  desired index for that color.  The threshold files are generated
 *  offline. See http://www.tekkotsu.org/CameraSetup.html
 *
 *  The color information is shared for all threshold files in this
 *  object.
 *
 *  The row skip is always 0, and the row stride is always width.
 *  But it would be better to use the proper accessor functions to be
 *  more general.
 *
 *	Should receive FilterBankEvents from any standard format
 *	FilterBankGenerator (like RawCameraGenerator) <em>However</em>,
 *	images that use an increment!=1 will break.
 *
 *  The events which are produced are SegmentedColorFilterBankEvents,
 *  which will allow you to reference the color information later on.
 *  Keep in mind that the region and area statistic fields are not
 *  filled out at this stage... the RegionGenerator will complete the
 *  processing if you want that info as well.
 *
 *  Uses the CMVision library for main processing
 *
 *  The format used for serialization is: (code is in saveBuffer())
 *  - <@c FilterBankGenerator: superclass header> <i>(First saves the superclass's info)</i>
 *  - <@c string: "SegColorImage"> <i>(remember a 'string' is len+str+0; so this is the literal "\015\0\0\0SegColorImage\0"; also remember "\015" is octal for 13)</i>
 *  - <<tt>char[</tt>width<tt>*</tt>height<tt>]</tt>: image data> <i>(one byte per sample)</i>
 *  - <@c unsigned @c int: num_cols> <i>(number of different colors available)</i>
 *  - for each of num_col:
 *    - <@c char: red> <i>red color to use for display of this index</i>
 *    - <@c char: green> <i>green color to use for display of this index</i>
 *    - <@c char: blue> <i>blue color to use for display of this index</i>
 *
 *  For more information on serialization, see FilterBankGenerator
 *
 */
class SegmentedColorGenerator : public FilterBankGenerator {
public:
	typedef CMVision::uchar cmap_t; //!< type to use for color indexes
	typedef CMVision::color_class_state color_class_state; //!< use CMVision's color structure
	typedef CMVision::color_name_map color_name_map; //!< shorthand for CMVision's color name lookup data structure

	//! constructor
	SegmentedColorGenerator(unsigned int mysid, FilterBankGenerator* fbg, EventBase::EventTypeID_t tid);
	//! constructor, you can pass which channels to use as Y, U, & V channels
	SegmentedColorGenerator(unsigned int mysid, FilterBankGenerator* fbg, EventBase::EventTypeID_t tid, unsigned int syc, unsigned int suc, unsigned int svc);
	//! destructor
	virtual ~SegmentedColorGenerator();

	static std::string getClassDescription() { return "Converts a FilterBankGenerator's data into indexed color"; }

	//! should receive FilterBankEvents from any standard format FilterBankGenerator (like RawCameraGenerator)
	virtual void doEvent();

	//! loads a threshold map into memory from a file, returns -1U if failed, otherwise returns corresponding channel
	virtual unsigned int loadThresholdMap(const std::string& tm_file);

	//! loads color information from a file, returns false if failed, true otherwise
	virtual bool loadColorInfo(const std::string& col_file);

	//! returns the number of different colors available
	virtual unsigned int getNumColors() const { return numColors; }

	//! gives direct access to the color information
	virtual const color_class_state * getColors() const { return colors; }

	//! gives direct access to the color information
	virtual color_class_state * getColors() { return colors; }

	//! returns index of color corresponding to a string (uses a fast hash lookup), or -1U if not found
	unsigned int getColorIndex(const char * name) const {
		color_name_map::const_iterator i = colorNames.find(name);
		return (i==colorNames.end())?-1U:i->second;
	}

	//! returns index of color corresponding to a string (uses a fast hash lookup), or -1U if not found
	unsigned int getColorIndex(const std::string& name) const { return getColorIndex(name.c_str()); }

	//! returns index of color corresponding to a specific rgb color, or -1U if not found
	unsigned int getColorIndex(const rgb color) const {
		for(unsigned int index = 0; index < getNumColors(); index++)
			if(getColorRGB((int)index) == color)
				return index;
		return -1U;
	}


	//! returns rgb struct (from colors.h) corresponding to an int index.  Returns black if index is invalid.
	rgb getColorRGB(const unsigned int index) const {
		return (index>=numColors ? rgb() : getColors()[index].color);
	}

	//! returns rgb struct (from colors.h) corresponding to a string.  Returns black if index is invalid.
	rgb getColorRGB(const char * name) const {
		return getColorRGB(getColorIndex(name));
	}

	//! returns rgb struct (from colors.h) corresponding to a string.  Returns black if index is invalid.
	rgb getColorRGB(const std::string& name) const {
		return getColorRGB(name.c_str());
	}

        //! returns the name of a color given its index
       const char* getColorName(const unsigned int index) const {
	             return (index>=numColors ? NULL : getColors()[index].name);
	}

	virtual unsigned int getBinSize() const;
	virtual unsigned int loadBuffer(const char buf[], unsigned int len, const char* filename=NULL);
	virtual unsigned int saveBuffer(char buf[], unsigned int len) const;
	virtual bool encodeColorsInc(char*& buf, unsigned int& len) const; //!< in case you want to only save the color info but not the image (this is binary - *not* the same format as what's read in loadColorInfo)
	virtual bool decodeColorsInc(const char*& buf, unsigned int& len); //!< in case you want to only load the color info but not the image (this is binary - *not* the same format as what's read in loadColorInfo)


protected:
	//! thrown if no threshold maps are available
	class NoThresholdException : public std::exception {
	public:
		//! returns descriptive error string
		virtual const char * what() const throw() { return "SegmentedColorGenerator::calcImage(): can't segment image without any loaded threshold maps"; }
	};

	static const unsigned int BITS_Y = 4; //!< bits of discretization for Y channel in the threshold map
	static const unsigned int BITS_U = 6; //!< bits of discretization for U channel in the threshold map
	static const unsigned int BITS_V = 6; //!< bits of discretization for V channel in the threshold map
	static const unsigned int NUM_Y = 1 << BITS_Y; //!< levels of discretization for Y channel in the threshold map
	static const unsigned int NUM_U = 1 << BITS_U; //!< levels of discretization for U channel in the threshold map
	static const unsigned int NUM_V = 1 << BITS_V; //!< levels of discretization for V channel in the threshold map
	static const unsigned int MAX_COLORS = 20; //!< maximum number of different colors that can be segmented

	//! ignores @a nChannels - the number of channels is always the number of loaded threshold maps
	virtual void setNumImages(unsigned int nLayers, unsigned int nChannels);
	virtual void setDimensions(); //!< sets stride parameter to width (as set by FilterBankGenerator::setDimensions())
	//! creates the image cache width[layer]*height[layer] + 1 -- why plus one?  Because CMVision temporarily scribbles one-past end of each row
	virtual unsigned char * createImageCache(unsigned int layer, unsigned int chan) const;
	virtual void calcImage(unsigned int layer, unsigned int chan);

	unsigned int srcYChan; //!< the channel of the source's Y channel
	unsigned int srcUChan; //!< the channel of the source's U channel
	unsigned int srcVChan; //!< the channel of the source's V channel

	std::vector<cmap_t*> tmaps; //!< list of threshold maps so you can segment the same source different ways
	std::vector<std::string> tmapNames; //!< filename of each tmap;

	unsigned int numColors; //!< number of available colors
	color_class_state colors[MAX_COLORS]; //!< array of available colors
	color_name_map colorNames; //!< look up color indexes corresponding to names

private:
	SegmentedColorGenerator(const SegmentedColorGenerator& fbk); //!< don't call
	const SegmentedColorGenerator& operator=(const SegmentedColorGenerator& fbk); //!< don't call
};

/*! @file
 * @brief Describes SegmentedColorGenerator, which generates FilterBankEvents indexed color images based on a color threshold file
 * @author alokl (Creator)
 * @author ejt (reorganized)
 */

#endif
