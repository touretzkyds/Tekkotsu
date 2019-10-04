//-*-c++-*-
#ifndef INCLUDED_SegmentedColorFilterBankEvent_h_
#define INCLUDED_SegmentedColorFilterBankEvent_h_

#include "Events/FilterBankEvent.h"
#include "Vision/cmvision.h"

//! This event provides some additional color information over its superclass for image banks made up of indexed colors
/*! The color information is stored using the CMVision library's data structures
 *
 *  Don't bother trying to access the region information which is also
 *  held in the color_class_state structures.  The RegionGenerator,
 *  doesn't fill in the global color information because each layer
 *  and channel is going to need its own region processing, so a
 *  single global structure only makes sense for the colors.
 */
class SegmentedColorFilterBankEvent : public FilterBankEvent {
public:
	typedef CMVision::color_class_state color_class_state; //!< use CMVision's color structure
	typedef CMVision::color_name_map color_name_map; //!< shorthand for CMVision's color name lookup data structure

	//! constructor, to be used when first segmented, later stages should use the other constructor
	SegmentedColorFilterBankEvent(FilterBankGenerator* creator,EventBase::EventGeneratorID_t gid,size_t sid,EventBase::EventTypeID_t tid, FilterBankGenerator* segColorSrc, unsigned int nColors, color_class_state * colorInfos, const color_name_map * clrNames)
		: FilterBankEvent(creator,gid,sid,tid), segsrc(segColorSrc), numColors(nColors), colors(colorInfos), colorNames(clrNames)
	{}

	//! constructor, allows you to pass along color information to later stages
	SegmentedColorFilterBankEvent(FilterBankGenerator* creator,EventBase::EventGeneratorID_t gid,size_t sid,EventBase::EventTypeID_t tid, const SegmentedColorFilterBankEvent& segevt )
		: FilterBankEvent(creator,gid,sid,tid), segsrc(segevt.segsrc), numColors(segevt.numColors), colors(segevt.colors), colorNames(segevt.colorNames)
	{}

	//! copy constructor (shallow copy -- the generator shouldn't be going anywhere)
	SegmentedColorFilterBankEvent(const SegmentedColorFilterBankEvent& fbk)
		: FilterBankEvent(fbk), segsrc(fbk.segsrc), numColors(fbk.numColors),
			colors(fbk.colors), colorNames(fbk.colorNames)
	{}
	
	//! assignment operator  (shallow copy -- the generator shouldn't be going anywhere)
	const SegmentedColorFilterBankEvent& operator=(const SegmentedColorFilterBankEvent& fbk) {
		EventBase::operator=(fbk);
		segsrc=fbk.segsrc; numColors=fbk.numColors;
		colors=fbk.colors; colorNames=fbk.colorNames;
		return *this;
	}

	virtual EventBase* clone() const { return new SegmentedColorFilterBankEvent(*this); }

	//don't do this until we actually override the LoadSave interface
	//virtual unsigned int getClassTypeID() const { return makeClassTypeID("SFBK"); }

	//! Gives access to underlying generator
	inline FilterBankGenerator* getSegmentedColorSource() const { return segsrc; }

	//! returns the number of different colors available
	inline unsigned int getNumColors() const { return numColors; }

	//! gives direct access to the color information
	inline const color_class_state * getColors() const { return colors; }

	//! gives direct access to the color information
	inline const color_class_state& getColor(unsigned int i) const { return colors[i]; }

	//! returns index of color corresponding to a string
	inline unsigned int getColorIndex(const char * name) const {
		color_name_map::const_iterator i = colorNames->find(name);
		return (i==colorNames->end())?-1U:i->second;
	}
	
	//! returns index of color corresponding to a string
	inline unsigned int getColorIndex(const std::string& name) const { return getColorIndex(name.c_str()); }
	
	virtual classTypeID_t getClassTypeID() const { return autoRegisterSegmentedColorFilterBankEvent; }
	
protected:
	//! pointer to generator which did the segmentation and therefore holds the color information
	FilterBankGenerator* segsrc;
	unsigned int numColors; //!< number of available colors
	const color_class_state * colors; //!< array of available colors
	const color_name_map * colorNames; //!< look up index from name

	//! causes class type id to automatically be regsitered with EventBase's FamilyFactory (getTypeRegistry())
	/*! This is instantiated in EventBase.cc to save on file bloat */
	static const EventBase::classTypeID_t autoRegisterSegmentedColorFilterBankEvent;
	
	//! default constructor, only intended to be called from the FamilyFactory, followed by a loadXML...
	SegmentedColorFilterBankEvent() : FilterBankEvent(), segsrc(NULL), numColors(), colors(), colorNames() {}
	friend struct Factory0Arg<EventBase>::Factory<SegmentedColorFilterBankEvent>;
};

/*! @file 
 * @brief Defines SegmentedColorFilterBankEvent, an event provides some additional color information over its superclass for image banks made up of indexed colors
 * @author ejt (Creator)
 */

#endif
