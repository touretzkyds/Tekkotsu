#include "TimerEvent.h"
#include "Behaviors/BehaviorBase.h"
#include <sstream>
#include <libxml/tree.h>

using namespace std; 

const EventBase::classTypeID_t TimerEvent::autoRegisterTimerEvent=getTypeRegistry().registerType<TimerEvent>(makeClassTypeID("TIMR"));

std::string
TimerEvent::getDescription(bool showTypeSpecific/*=true*/, unsigned int verbosity/*=0*/) const {
	if(!showTypeSpecific)
		return EventBase::getDescription(showTypeSpecific,verbosity);
	std::ostringstream logdata;
	logdata << EventBase::getDescription(showTypeSpecific,verbosity) << '\t';
	if(BehaviorBase * beh=dynamic_cast<BehaviorBase*>(target)) {
		logdata << beh->getClassName() << "(" << beh->getName() << ")@" << beh;
	} else {
		logdata << "Listener@" << target << endl;
	}
	return logdata.str();
}

unsigned int
TimerEvent::getBinSize() const {
	unsigned int used=EventBase::getBinSize();
	if(saveFormat==XML)
		return used; //if using XML, the XMLLoadSave::getBinSize (called by EventBase::getBinSize) is all we need
	//otherwise need to add our own fields
	used+=creatorSize("EventBase::TimerEvent");
	used+=getSerializedSize(target); // not that a pointer is of any direct use once loaded externally, but still useful as an identifier
	return used;
}

unsigned int
TimerEvent::loadBinaryBuffer(const char buf[], unsigned int len) {
	unsigned int origlen=len;
	if(!checkInc(EventBase::loadBinaryBuffer(buf,len),buf,len)) return 0;
	if(!checkCreatorInc("EventBase::TimerEvent",buf,len,true)) return 0;
	unsigned long long tgt;
	if(!decodeInc(tgt,buf,len)) return 0;
	target=reinterpret_cast<EventListener*>(tgt);
	return origlen-len;	
}

unsigned int
TimerEvent::saveBinaryBuffer(char buf[], unsigned int len) const {
	unsigned int origlen=len;
	if(!checkInc(EventBase::saveBinaryBuffer(buf,len),buf,len)) return 0;
	if(!saveCreatorInc("EventBase::TimerEvent",buf,len)) return 0;
	if(!encodeInc(reinterpret_cast<unsigned long long>(target),buf,len)) return 0;
	return origlen-len;
}

void TimerEvent::loadXML(xmlNode* node) {
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
		
		if(xmlStrcmp(name, (const xmlChar *)"target")==0)
			target=reinterpret_cast<EventListener*>(strtol((char*)val,NULL,0));
		
		xmlFree(val);
		xmlFree(name);
	}
}

void TimerEvent::saveXML(xmlNode * node) const {
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
		throw bad_format(node,"Error: VisionObjectEvent xml error on saving param");
	xmlSetProp(cur,(const xmlChar*)"name",(const xmlChar*)"target");
	char tmp[20];
	snprintf(tmp,20,"%p",target);
	xmlSetProp(cur,(const xmlChar*)"value",(const xmlChar*)tmp);
}


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
