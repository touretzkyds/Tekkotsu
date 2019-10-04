//-*-c++-*-
#ifndef INCLUDED_PitchEvent_h_
#define INCLUDED_PitchEvent_h_

#include "EventBase.h"

//! Provides information about a tone detected from the microphone(s)
class PitchEvent : public EventBase {
public:
	//! default constructor, sets generator ID, but nothing else
	PitchEvent() : EventBase(), freq(), amplitude(), confidence() {genID=EventBase::micPitchEGID;}
	
	//!constructor
	PitchEvent(size_t sid, EventTypeID_t type, const float freq_, const char *name_, const float amplitude_, const unsigned int duration_, const float confidence_) 
	: EventBase(EventBase::micPitchEGID, sid, type,duration_,name_,(type==deactivateETID) ? 0 : confidence_*amplitude_), freq(freq_), amplitude(amplitude_), confidence(confidence_)
	{}
	
	//! copy constructor (does what you'd expect, explicit to satisify compiler warning)
	PitchEvent(const PitchEvent &p)
	: EventBase(p), freq(p.freq), amplitude(p.amplitude), confidence(p.confidence)
	{}
	
	//! assignment operator (does what you'd expect, explicit to satisify compiler warning)
	PitchEvent & operator=(const PitchEvent &p) {
		EventBase::operator=(p); freq=p.freq; amplitude=p.amplitude; confidence=p.confidence;
		return *this;
	}
	
	virtual EventBase* clone() const { return new PitchEvent(*this); }
	
	virtual std::string getDescription(bool showTypeSpecific=true, unsigned int verbosity=0) const;

	virtual unsigned int getBinSize() const;
	virtual unsigned int loadBinaryBuffer(const char buf[], unsigned int len);
	virtual unsigned int saveBinaryBuffer(char buf[], unsigned int len) const;
	virtual void loadXML(xmlNode* node);
	virtual void saveXML(xmlNode * node) const;
	
	float getFreq(void) const { return freq; } //!< returns #freq
	float getAmplitude(void) const { return amplitude; } //!< returns #amplitude
	float getConfidence(void) const { return confidence; } //!< returns #confidence
	
	virtual classTypeID_t getClassTypeID() const { return autoRegisterPitchEvent; }

protected:
	float freq; //!< the frequency (Hz) being detected
	float amplitude; //!< indicates how loud the signal is -- can be both loud and noisy, loud doesn't necessarily mean "strong"
	float confidence; //!< indicates how much variance is being detected

	//! causes class type id to automatically be regsitered with EventBase's FamilyFactory (getTypeRegistry())
	static const EventBase::classTypeID_t autoRegisterPitchEvent;
};

/*! @file
 * @brief Describes PitchEvent, which provides information about a tone detected from the microphone(s)
 * @author Matus Telgarsky and Jonah Sherman (Creators)
 * @author Ethan Tira-Thompson (imported into framework)
 *
 * Originally written as a part of a final project at Carnegie Mellon (15-494 Cognitive Robotics, Spring 2006)
 */

#endif
