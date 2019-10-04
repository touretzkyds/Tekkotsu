#include "TextMsgEvent.h"
#include <sstream>
#include <libxml/tree.h>

const EventBase::classTypeID_t TextMsgEvent::autoRegisterTextMsgEvent=getTypeRegistry().registerType<TextMsgEvent>(makeClassTypeID("TXTM"));

std::string
TextMsgEvent::getDescription(bool showTypeSpecific/*=true*/, unsigned int verbosity/*=0*/) const {
	if(!showTypeSpecific)
		return EventBase::getDescription(showTypeSpecific,verbosity);
	std::ostringstream logdata;
	logdata << EventBase::getDescription(showTypeSpecific,verbosity) << '\t' << _text;
	return logdata.str();
}

unsigned int
TextMsgEvent::getBinSize() const {
	unsigned int used=EventBase::getBinSize();
	if(saveFormat==XML)
		return used; //if using XML, the XMLLoadSave::getBinSize (called by EventBase::getBinSize) is all we need
	//otherwise need to add our own fields
	used+=creatorSize("EventBase::TextMsgEvent");
	used+=getSerializedSize(_text);
	//used+=sizeof(_token);
	return used;
}

unsigned int
TextMsgEvent::loadBinaryBuffer(const char buf[], unsigned int len) {
	unsigned int origlen=len;
	if(!checkInc(EventBase::loadBinaryBuffer(buf,len),buf,len)) return 0;
	if(!checkCreatorInc("EventBase::TextMsgEvent",buf,len,true)) return 0;
	if(!decodeInc(_text,buf,len)) return 0;
	//if(!decodeInc(_token,buf,len)) return 0;
	//len-=used; buf+=used;
	return origlen-len;	
}

unsigned int
TextMsgEvent::saveBinaryBuffer(char buf[], unsigned int len) const {
	unsigned int origlen=len;
	if(!checkInc(EventBase::saveBinaryBuffer(buf,len),buf,len)) return 0;
	if(!saveCreatorInc("EventBase::TextMsgEvent",buf,len)) return 0;
	if(!encodeInc(_text,buf,len)) return 0;
	//if(!encodeInc(_token,buf,len)) return 0;
	return origlen-len;
}

void TextMsgEvent::loadXML(xmlNode* node) {
	if(node==NULL)
		return;
	
	EventBase::loadXML(node);
	
	for(xmlNode* cur = skipToElement(node->children); cur!=NULL; cur = skipToElement(cur->next)) {
		if(xmlStrcmp(cur->name, (const xmlChar *)"param"))
			continue;
		
		xmlChar * name = xmlGetProp(cur,(const xmlChar*)"name");
		if(name==NULL)
			throw bad_format(cur,"property missing name");
		
		xmlChar * val = xmlGetProp(cur,(const xmlChar*)"value");
		if(val==NULL)
			throw bad_format(cur,"property missing value");
		
		if(xmlStrcmp(name, (const xmlChar *)"text")==0)
			_text=(const char*)val;
		
		xmlFree(val);
		xmlFree(name);
	}
}

void TextMsgEvent::saveXML(xmlNode * node) const {
	if(node==NULL)
		return;
	EventBase::saveXML(node);
	
	//clear old params first
	for(xmlNode* cur = skipToElement(node->children); cur!=NULL; ) {
		if(xmlStrcmp(cur->name, (const xmlChar *)"param")==0) {
			xmlUnlinkNode(cur);
			xmlFreeNode(cur);
			cur = skipToElement(node->children); //restart the search (boo)
		} else
			cur = skipToElement(cur->next);
	}
	
	xmlNode* cur=xmlNewChild(node,NULL,(const xmlChar*)"param",NULL);
	if(cur==NULL)
		throw bad_format(node,"Error: TextMsgEvent xml error on saving param");
	xmlSetProp(cur,(const xmlChar*)"name",(const xmlChar*)"text");
	xmlSetProp(cur,(const xmlChar*)"value",(const xmlChar*)_text.c_str());
}

/*! @file
 * @brief Implements TextMsgEvent, which extends EventBase to also include actual message text
 * @author ejt (Creator)
 */
