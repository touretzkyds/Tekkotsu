#include "VisionObjectEvent.h"
#include "Shared/RobotInfo.h"
#include <sstream>
#include <libxml/tree.h>
#include <cmath>

const EventBase::classTypeID_t VisionObjectEvent::autoRegisterVisionObjectEvent=getTypeRegistry().registerType<VisionObjectEvent>(makeClassTypeID("VISO"));

std::string
VisionObjectEvent::getDescription(bool showTypeSpecific/*=true*/, unsigned int verbosity/*=0*/) const {
	if(!showTypeSpecific)
		return EventBase::getDescription(showTypeSpecific,verbosity);
	std::ostringstream logdata;
	logdata << EventBase::getDescription(showTypeSpecific,verbosity) << '\t' << _x1 <<  '\t' <<_x2 <<'\t' <<_y1 <<'\t' <<_y2<<'\t'<<_frame ;
	return logdata.str();
}

float
VisionObjectEvent::getDistanceEstimate(float diaMajor, float diaMinor) const {
	float diaX,diaY;
	float w=getWidth();
	float h=getHeight();
	if(w>h) {
		diaX=diaMajor;
		diaY=diaMinor;
	} else {
		diaX=diaMinor;
		diaY=diaMajor;
	}
	// divide dim by two because range magnitude is two: ±1 maps to ±FOV/2
	float xest=diaX>0?calcDistance(getWidth()*CameraHorizFOV/2,diaX):0;
	float yest=diaY>0?calcDistance(getHeight()*CameraHorizFOV/2,diaY):0;
	if(xest>0 && yest>0) {
		return (xest+yest)/2;
	} else if(xest>0) {
		return xest;
	} else if(yest>0) {
		return yest;
	}
	return 0;
}

float
VisionObjectEvent::calcDistance(float visArc, float physDia) {
	float r=std::tan(visArc/2);
	if(r==0)
		return 0;
	return physDia/(2*r);
}

unsigned int
VisionObjectEvent::getBinSize() const {
	unsigned int used=EventBase::getBinSize();
	if(saveFormat==XML)
		return used; //if using XML, the XMLLoadSave::getBinSize (called by EventBase::getBinSize) is all we need
	//otherwise need to add our own fields
	used+=creatorSize("EventBase::VisionObjectEvent");
	used+=getSerializedSize(_x1);
	used+=getSerializedSize(_x2);
	used+=getSerializedSize(_y1);
	used+=getSerializedSize(_y2);
	return used;
}

unsigned int
VisionObjectEvent::loadBinaryBuffer(const char buf[], unsigned int len) {
	unsigned int origlen=len;
	if(!checkInc(EventBase::loadBinaryBuffer(buf,len),buf,len)) return 0;
	if(!checkCreatorInc("EventBase::VisionObjectEvent",buf,len,true)) return 0;
	if(!decodeInc(_x1,buf,len)) return 0;
	if(!decodeInc(_x2,buf,len)) return 0;
	if(!decodeInc(_y1,buf,len)) return 0;
	if(!decodeInc(_y2,buf,len)) return 0;
	return origlen-len;	
}

unsigned int
VisionObjectEvent::saveBinaryBuffer(char buf[], unsigned int len) const {
	unsigned int origlen=len;
	if(!checkInc(EventBase::saveBinaryBuffer(buf,len),buf,len)) return 0;
	if(!saveCreatorInc("EventBase::VisionObjectEvent",buf,len)) return 0;
	if(!encodeInc(_x1,buf,len)) return 0;
	if(!encodeInc(_x2,buf,len)) return 0;
	if(!encodeInc(_y1,buf,len)) return 0;
	if(!encodeInc(_y2,buf,len)) return 0;
	return origlen-len;
}

void VisionObjectEvent::loadXML(xmlNode* node) {
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

		if(xmlStrcmp(name, (const xmlChar *)"x1")==0)
			_x1=(float)atof((const char*)val);
		else if(xmlStrcmp(name, (const xmlChar *)"x2")==0)
			_x2=(float)atof((const char*)val);
		else if(xmlStrcmp(name, (const xmlChar *)"y1")==0)
			_y1=(float)atof((const char*)val);
		else if(xmlStrcmp(name, (const xmlChar *)"y2")==0)
			_y2=(float)atof((const char*)val);
		else if(xmlStrcmp(name, (const xmlChar *)"clipLeft")==0)
			_clipLeft=atoi((const char*)val);
		else if(xmlStrcmp(name, (const xmlChar *)"clipRight")==0)
			_clipRight=atoi((const char*)val);
		else if(xmlStrcmp(name, (const xmlChar *)"clipTop")==0)
			_clipTop=atoi((const char*)val);
		else if(xmlStrcmp(name, (const xmlChar *)"clipBottom")==0)
			_clipBottom=atoi((const char*)val);
		
		xmlFree(val);
		xmlFree(name);
	}
}


//! a little local macro to make saving fields easier
#define SAVE_PARAM(strname,varname,format) {\
xmlNode* cur=xmlNewChild(node,NULL,(const xmlChar*)"param",NULL); \
if(cur==NULL) \
throw bad_format(node,"Error: VisionObjectEvent xml error on saving param"); \
xmlSetProp(cur,(const xmlChar*)"name",(const xmlChar*)strname); \
char valbuf[20]; \
snprintf(valbuf,20,format,varname); \
xmlSetProp(cur,(const xmlChar*)"value",(const xmlChar*)valbuf); }

void VisionObjectEvent::saveXML(xmlNode * node) const {
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
	
	SAVE_PARAM("x1",_x1,"%g");
	SAVE_PARAM("y1",_y1,"%g");
	SAVE_PARAM("x2",_x2,"%g");
	SAVE_PARAM("y2",_y2,"%g");
	SAVE_PARAM("clipLeft",_clipLeft,"%d");
	SAVE_PARAM("clipRight",_clipRight,"%d");
	SAVE_PARAM("clipTop",_clipTop,"%d");
	SAVE_PARAM("clipBottom",_clipBottom,"%d");
}

/*! @file
 * @brief Implements VisionObjectEvent, which provides information about objects recognized in the camera image
 * @author alokl (Creator)
 * @author Ignacio Herrero Reder &lt; nhr at dte uma es &gt; (VisionObjectInfo Boundary Box - bug 74)
 */
