#include "LocomotionEvent.h"
#include <sstream>
#include <libxml/tree.h>
#include <iostream>

using namespace std;

const EventBase::classTypeID_t LocomotionEvent::autoRegisterLocomotionEvent=getTypeRegistry().registerType<LocomotionEvent>(makeClassTypeID("LOCO"));

std::string
LocomotionEvent::getDescription(bool showTypeSpecific/*=true*/, unsigned int verbosity/*=0*/) const {
	if(!showTypeSpecific)
		return EventBase::getDescription(showTypeSpecific,verbosity);
	std::ostringstream logdata;
	logdata << EventBase::getDescription(showTypeSpecific,verbosity) << '\t' << x << '\t' << y << '\t' << a;
	return logdata.str();
}
	
unsigned int
LocomotionEvent::getBinSize() const {
	unsigned int used=EventBase::getBinSize();
	if(saveFormat==XML)
		return used; //if using XML, the XMLLoadSave::getBinSize (called by EventBase::getBinSize) is all we need
	//otherwise need to add our own fields
	used+=creatorSize("EventBase::LocomotionEvent");
	used+=getSerializedSize(x);
	used+=getSerializedSize(y);
	used+=getSerializedSize(a);
	return used;
}

unsigned int
LocomotionEvent::loadBinaryBuffer(const char buf[], unsigned int len) {
	unsigned int origlen=len;
	if(!checkInc(EventBase::loadBinaryBuffer(buf,len),buf,len)) return 0;
	if(!checkCreatorInc("EventBase::LocomotionEvent",buf,len,true)) return 0;
	if(!decodeInc(x,buf,len)) return 0;
	if(!decodeInc(y,buf,len)) return 0;
	if(!decodeInc(a,buf,len)) return 0;
	return origlen-len;	
}

unsigned int
LocomotionEvent::saveBinaryBuffer(char buf[], unsigned int len) const {
	unsigned int origlen=len;
	if(!checkInc(EventBase::saveBinaryBuffer(buf,len),buf,len)) return 0;
	if(!saveCreatorInc("EventBase::LocomotionEvent",buf,len)) return 0;
	if(!encodeInc(x,buf,len)) return 0;
	if(!encodeInc(y,buf,len)) return 0;
	if(!encodeInc(a,buf,len)) return 0;
	return origlen-len;
}

void
LocomotionEvent::loadXML(xmlNode* node) {
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
		
		if(xmlStrcmp(name, (const xmlChar *)"x")==0)
			x=(float)atof((const char*)val);
		else if(xmlStrcmp(name, (const xmlChar *)"y")==0)
			y=(float)atof((const char*)val);
		else if(xmlStrcmp(name, (const xmlChar *)"a")==0)
			a=(float)atof((const char*)val);
		
		xmlFree(val);
		xmlFree(name);
	}
}

//! a little local macro to make saving fields easier
#define SAVE_PARAM(name) { \
xmlNode* cur=xmlNewChild(node,NULL,(const xmlChar*)"param",NULL); \
if(cur==NULL) \
throw bad_format(node,"Error: LocomotionEvent xml error on saving param"); \
xmlSetProp(cur,(const xmlChar*)"name",(const xmlChar*)#name); \
char valbuf[20]; \
snprintf(valbuf,20,"%g",name); \
xmlSetProp(cur,(const xmlChar*)"value",(const xmlChar*)valbuf); }

void
LocomotionEvent::saveXML(xmlNode * node) const {
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

	SAVE_PARAM(x);
	SAVE_PARAM(y);
	SAVE_PARAM(a);
}

/*! @file
 * @brief Implements LocomotionEvent, which gives updates regarding the current movement of the robot through the world
 * @author ejt (Creator)
 */
