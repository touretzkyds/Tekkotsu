//-*-c++-*-
#ifndef INCLUDED_TextMsgEvent_h
#define INCLUDED_TextMsgEvent_h

#include "EventBase.h"

//! Extends EventBase to also include actual message text
class TextMsgEvent : public EventBase {
 public:
	//! Constructor
	TextMsgEvent() : EventBase(EventBase::textmsgEGID,(size_t)-1, EventBase::statusETID,0),_text("")/*,_token(0)*/ {  }

	//! Constructor, pass a text msg
	TextMsgEvent(const std::string& text, size_t srcID=-1U) : EventBase(EventBase::textmsgEGID,srcID, EventBase::statusETID,0),_text(text)/*,_token(0)*/ { }
  
	virtual EventBase* clone() const { return new TextMsgEvent(*this); }

	virtual unsigned int getClassTypeID() const { return autoRegisterTextMsgEvent; }

	std::string getText() const { return _text; } //!< returns the text
	TextMsgEvent& setText(const std::string& text) { _text=text; return *this; } //!< sets the text
  
	std::string getDescription(bool showTypeSpecific=true, unsigned int verbosity=0) const;
	
	virtual unsigned int getBinSize() const;
	virtual unsigned int loadBinaryBuffer(const char buf[], unsigned int len);
	virtual unsigned int saveBinaryBuffer(char buf[], unsigned int len) const;
	virtual void loadXML(xmlNode* node);
	virtual void saveXML(xmlNode * node) const;
	
 protected:
	std::string _text; //!< the unmodified arguments passed to the command
	
	//! causes class type id to automatically be regsitered with EventBase's FamilyFactory (getTypeRegistry())
	static const EventBase::classTypeID_t autoRegisterTextMsgEvent;
};

/*! @file
 * @brief Describes TextMsgEvent, which extends EventBase to also include actual message text
 * @author ejt (Creator)
 */

#endif
