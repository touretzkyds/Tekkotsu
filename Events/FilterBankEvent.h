//-*-c++-*-
#ifndef INCLUDED_FilterBankEvent_h_
#define INCLUDED_FilterBankEvent_h_

#include "Events/EventBase.h"
#include "Vision/FilterBankGenerator.h"
#include "Shared/Factories.h"

//! This event gives access to a set of image channels at different resolutions, see FilterBankGenerator
class FilterBankEvent : public EventBase {
public:
	//! constructor
	FilterBankEvent(FilterBankGenerator* creator,EventBase::EventGeneratorID_t gid,size_t sid,EventBase::EventTypeID_t tid)
		: EventBase(gid,sid,tid,0,creator->getName()), src(creator)
	{}

	//! copy constructor (shallow copy -- the generator shouldn't be going anywhere)
	FilterBankEvent(const FilterBankEvent& fbk) : EventBase(fbk), src(fbk.src) {}
	
	//! assignment operator  (shallow copy -- the generator shouldn't be going anywhere)
	const FilterBankEvent& operator=(const FilterBankEvent& fbk) { EventBase::operator=(fbk); src=fbk.src; return *this; }

	virtual EventBase* clone() const { return new FilterBankEvent(*this); }

	//don't do this until we actually override the LoadSave interface
	//virtual unsigned int getClassTypeID() const { return makeClassTypeID("FBKE"); }

	//! Gives access to underlying generator
	inline FilterBankGenerator* getSource() const { return src; }

	//! returns the number of image layers (e.g. different resolutions available)
	inline unsigned int getNumLayers() const { return src->getNumLayers(); }

	//! returns the number of channels per image (e.g. Y, U, or V components)
	inline unsigned int getNumChannels() const { return src->getNumChannels(); }

	//! returns pointer to the beginning of the image data for the specified layer and channel
	/*! this will cause the data to be calculated and cached if it's not already available */
	inline unsigned char * getImage(unsigned int layer, unsigned int channel) const { return src->getImage(layer,channel); }

	//! returns the number of bytes used for the data returned by getImage() - if the data varies in size (e.g. jpeg compression), will return 0 if the image hasn't been calculated yet (so call it @e after getImage())
	virtual size_t getImageSize(unsigned int layer, unsigned int chan) const { return src->getImageSize(layer,chan); }
	
	//! returns width of the image in a given layer
	inline unsigned int getWidth(unsigned int layer) const { return src->getWidth(layer); }
	//! returns height of the image in a given layer
	inline unsigned int getHeight(unsigned int layer) const { return src->getHeight(layer); }
	//! returns the bytes to skip from the one-past-end of a row to get the beginning of the next
	inline unsigned int getSkip(unsigned int layer) const { return src->getSkip(layer); }
	//! returns the bytes to skip from the beginning of one row to get the beginning of the next
	/*! This is just for convenience; the stride is just the skip plus the width, but it's precomputed for you for speed and clarity */
	inline unsigned int getStride(unsigned int layer) const { return src->getStride(layer); }
	//! returns the increment to use to go from one sample to the next
	inline unsigned int getIncrement(unsigned int layer) const { return src->getIncrement(layer); }

	//! returns the frame number, see FilterBankGenerator::frameNumber
	inline unsigned int getFrameNumber() const { return src->getFrameNumber(); }
	//! returns the number of frames processed by the generator, see FilterBankGenerator::framesProcessed
	inline unsigned int getFramesProcessed() const { return src->getFramesProcessed(); }
	
	virtual classTypeID_t getClassTypeID() const { return autoRegisterFilterBankEvent; }
	
protected:
	//! pointer to generator which created this event
	/*! the generator holds all the actual image data to be more memory efficient */
	FilterBankGenerator* src;

	//! causes class type id to automatically be regsitered with EventBase's FamilyFactory (getTypeRegistry())
	/*! This is instantiated in EventBase.cc to save on file bloat */
	static const EventBase::classTypeID_t autoRegisterFilterBankEvent;
	
	//! default constructor, only intended to be called from the FamilyFactory, followed by a loadXML...
	FilterBankEvent() : EventBase(), src(NULL) {}
	friend struct Factory0Arg<EventBase>::Factory<FilterBankEvent>;
};

/*! @file 
 * @brief Defines FilterBankEvent, an event for distributing raw image data
 * @author ejt (Creator)
 */

#endif
