#include "PitchEvent.h"
#include <sstream>
#include <libxml/tree.h>

using namespace std; 

const EventBase::classTypeID_t PitchEvent::autoRegisterPitchEvent=getTypeRegistry().registerType<PitchEvent>(makeClassTypeID("PITC"));

std::string
PitchEvent::getDescription(bool showTypeSpecific/*=true*/, unsigned int verbosity/*=0*/) const {
	if(!showTypeSpecific)
		return EventBase::getDescription(showTypeSpecific,verbosity);
	std::ostringstream logdata;
	logdata << EventBase::getDescription(showTypeSpecific,verbosity) << '\t' << freq << '\t' << amplitude << '\t' << confidence;
	return logdata.str();
}

unsigned int
PitchEvent::getBinSize() const {
	unsigned int used=EventBase::getBinSize();
	if(saveFormat==XML)
		return used; //if using XML, the XMLLoadSave::getBinSize (called by EventBase::getBinSize) is all we need
	//otherwise need to add our own fields
	used+=creatorSize("EventBase::PitchEvent");
	used+=getSerializedSize(freq);
	used+=getSerializedSize(amplitude);
	used+=getSerializedSize(confidence);
	return used;
}

unsigned int
PitchEvent::loadBinaryBuffer(const char buf[], unsigned int len) {
	unsigned int origlen=len;
	if(!checkInc(EventBase::loadBinaryBuffer(buf,len),buf,len)) return 0;
	if(!checkCreatorInc("EventBase::PitchEvent",buf,len,true)) return 0;
	if(!decodeInc(freq,buf,len)) return 0;
	if(!decodeInc(amplitude,buf,len)) return 0;
	if(!decodeInc(confidence,buf,len)) return 0;
	return origlen-len;	
}

unsigned int
PitchEvent::saveBinaryBuffer(char buf[], unsigned int len) const {
	unsigned int origlen=len;
	if(!checkInc(EventBase::saveBinaryBuffer(buf,len),buf,len)) return 0;
	if(!saveCreatorInc("EventBase::PitchEvent",buf,len)) return 0;
	if(!encodeInc(freq,buf,len)) return 0;
	if(!encodeInc(amplitude,buf,len)) return 0;
	if(!encodeInc(confidence,buf,len)) return 0;
	return origlen-len;
}

void
PitchEvent::loadXML(xmlNode* node) {
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
		
		//cout << "loadXML: " << name << "=" << val << endl;
		
		if(xmlStrcmp(name, (const xmlChar *)"freq")==0)
			freq=(float)atof((const char*)val);
		else if(xmlStrcmp(name, (const xmlChar *)"amplitude")==0)
			amplitude=(float)atof((const char*)val);
		else if(xmlStrcmp(name, (const xmlChar *)"confidence")==0)
			confidence=(float)atof((const char*)val);
		
		xmlFree(val);
		xmlFree(name);
	}
}

//! a little local macro to make saving fields easier
#define SAVE_PARAM(name) { \
	xmlNode* cur=xmlNewChild(node,NULL,(const xmlChar*)"param",NULL); \
	if(cur==NULL) \
		throw bad_format(node,"Error: PitchEvent xml error on saving param"); \
	xmlSetProp(cur,(const xmlChar*)"name",(const xmlChar*)#name); \
	char valbuf[20]; \
	snprintf(valbuf,20,"%g",name); \
	xmlSetProp(cur,(const xmlChar*)"value",(const xmlChar*)valbuf); }

void
PitchEvent::saveXML(xmlNode * node) const {
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
	
	//cout << "saveXML: " << x << ' ' << y << ' ' << a << endl;
	
	SAVE_PARAM(freq);
	SAVE_PARAM(amplitude);
	SAVE_PARAM(confidence);
}

/*! @file
 * @brief Implements PitchEvent, which provides information about a tone detected from the microphone(s)
 * @author Matus Telgarsky and Jonah Sherman (Creators)
 * @author Ethan Tira-Thompson (imported into framework)
 *
 * Originally written as a part of a final project at Carnegie Mellon (15-494 Cognitive Robotics, Spring 2006)
 */
