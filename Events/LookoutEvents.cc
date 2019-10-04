#include "LookoutEvents.h"

#include <sstream>
#include <libxml/tree.h>
#include <iostream>

using namespace std;

const EventBase::classTypeID_t LookoutPointAtEvent::autoRegisterLookoutPointAtEvent=getTypeRegistry().registerType<LookoutPointAtEvent>(makeClassTypeID("LOLA"));
const EventBase::classTypeID_t LookoutSketchEvent::autoRegisterLookoutSketchEvent=getTypeRegistry().registerType<LookoutSketchEvent>(makeClassTypeID("LSKC"));
const EventBase::classTypeID_t LookoutIREvent::autoRegisterLookoutIREvent=getTypeRegistry().registerType<LookoutIREvent>(makeClassTypeID("LOIR"));
const EventBase::classTypeID_t LookoutScanEvent::autoRegisterLookoutScanEvent=getTypeRegistry().registerType<LookoutScanEvent>(makeClassTypeID("LSCN"));


//================ LookoutPointAtEvent

std::string
LookoutPointAtEvent::getDescription(bool showTypeSpecific/*=true*/, unsigned int verbosity/*=0*/) const {
  if(!showTypeSpecific)
    return EventBase::getDescription(showTypeSpecific,verbosity);
  std::ostringstream logdata;
  logdata << EventBase::getDescription(showTypeSpecific,verbosity)
	  << "toBaseMatrix=\n" << toBaseMatrix << '\n';
  return logdata.str();
}

unsigned int
LookoutPointAtEvent::getBinSize() const {
  unsigned int used=EventBase::getBinSize();
  if(saveFormat==XML)
    return used; //if using XML, the XMLLoadSave::getBinSize (called by EventBase::getBinSize) is all we need
  //otherwise need to add our own fields
  used+=creatorSize("EventBase::LookoutPointAtEvent");
  used+=sizeof(toBaseMatrix);
  return used;
}

unsigned int
LookoutPointAtEvent::loadBinaryBuffer(const char buf[], unsigned int len) {
  unsigned int origlen=len;
  unsigned int used;
  if(0==(used=EventBase::loadBinaryBuffer(buf,len))) return 0;
  len-=used; buf+=used;
  if(0==(used=checkCreator("EventBase::LookoutPointAtEvent",buf,len,true))) return 0;
  len-=used; buf+=used;
  for (int i = 0; i < 16; i++) {
    if(0==(used=decode(toBaseMatrix(1+i/4,1+(i%4)),buf,len))) return 0;
    len-=used; buf+=used;
  }
  return origlen-len;	
}

unsigned int
LookoutPointAtEvent::saveBinaryBuffer(char buf[], unsigned int len) const {
  unsigned int origlen=len;
  unsigned int used;
  if(0==(used=EventBase::saveBinaryBuffer(buf,len))) return 0;
  len-=used; buf+=used;
  if(0==(used=saveCreator("EventBase::LookoutPointAtEvent",buf,len))) return 0;
  len-=used; buf+=used;
  for (int i = 0; i < 16; i++) {
    if(0==(used=encode(toBaseMatrix(1+i/4,1+(i%4)),buf,len))) return 0;
    len-=used; buf+=used;
  }
  return origlen-len;
}

void LookoutPointAtEvent::loadXML(xmlNode* node) {
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
    
    cout << "loadXML: " << name << "=" << val << endl;
    
    if(xmlStrcmp(name, (const xmlChar *)"toBaseMatrix")==0) {
      const string valStr = (const char*) val;
      string::size_type pos = valStr.find_first_of(' ', 0);
      string::size_type prev_pos = ++pos;
      for (unsigned int i = 0; i < 16; i++) {
	pos = valStr.find_first_of(' ', pos);
	toBaseMatrix(1+i/4,1+(i%4)) = (float)atof(valStr.substr(prev_pos, pos-prev_pos).c_str());
	prev_pos = ++pos;
	if (prev_pos == string::npos)
	  break;
      }
    }
    
    xmlFree(val);
    xmlFree(name);
  }
}

void LookoutPointAtEvent::saveXML(xmlNode * node) const {
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
	
  //  cout << "saveXML: " << toBaseMatrix << endl;

  xmlNode* cur=xmlNewChild(node,NULL,(const xmlChar*)"param",NULL);
  if(cur==NULL)
    throw bad_format(node,"Error: LookoutPointAtEvent xml error on saving param");
  xmlSetProp(cur,(const xmlChar*)"name",(const xmlChar*)"toBaseMatrix");
  std::ostringstream valbuf;
  for (int i = 0; i < 16; i++)
    valbuf << toBaseMatrix(1+i/4,1+(i%4)) << ' ';
  xmlSetProp(cur,(const xmlChar*)"value",(const xmlChar*)valbuf.str().c_str());
}


//================ LookoutIREvent

std::string
LookoutIREvent::getDescription(bool showTypeSpecific/*=true*/, unsigned int verbosity/*=0*/) const {
  if(!showTypeSpecific)
    return LookoutPointAtEvent::getDescription(showTypeSpecific,verbosity);
  std::ostringstream logdata;
  logdata << LookoutPointAtEvent::getDescription(showTypeSpecific,verbosity)
	  << '\t' << distance;
  return logdata.str();
}

unsigned int
LookoutIREvent::getBinSize() const {
  unsigned int used=LookoutPointAtEvent::getBinSize();
  if(saveFormat==XML)
    return used; //if using XML, the XMLLoadSave::getBinSize (called by LookoutPointAtEvent::getBinSize) is all we need
  //otherwise need to add our own fields
  used+=creatorSize("LookoutPointAtEvent::LookoutIREvent");
  used+=sizeof(distance);
  return used;
}

unsigned int
LookoutIREvent::loadBinaryBuffer(const char buf[], unsigned int len) {
  unsigned int origlen=len;
  unsigned int used;
  if(0==(used=LookoutPointAtEvent::loadBinaryBuffer(buf,len))) return 0;
  len-=used; buf+=used;
  if(0==(used=checkCreator("LookoutPointAtEvent::LookoutIREvent",buf,len,true))) return 0;
  len-=used; buf+=used;
  if(0==(used=decode(distance,buf,len))) return 0;
  len-=used; buf+=used;
  return origlen-len;	
}

unsigned int
LookoutIREvent::saveBinaryBuffer(char buf[], unsigned int len) const {
  unsigned int origlen=len;
  unsigned int used;
  if(0==(used=LookoutPointAtEvent::saveBinaryBuffer(buf,len))) return 0;
  len-=used; buf+=used;
  if(0==(used=saveCreator("LookoutPointAtEvent::LookoutIREvent",buf,len))) return 0;
  len-=used; buf+=used;
  if(0==(used=encode(distance,buf,len))) return 0;
  len-=used; buf+=used;
  return origlen-len;
}

void LookoutIREvent::loadXML(xmlNode* node) {
  if(node==NULL)
    return;
  
  LookoutPointAtEvent::loadXML(node);
  
  for(xmlNode* cur = skipToElement(node->children); cur!=NULL; cur = skipToElement(cur->next)) {
    if(xmlStrcmp(cur->name, (const xmlChar *)"param"))
      continue;
    
    xmlChar * name = xmlGetProp(cur,(const xmlChar*)"name");
    if(name==NULL)
      throw bad_format(cur,"property missing name");
    
    xmlChar * val = xmlGetProp(cur,(const xmlChar*)"value");
    if(val==NULL)
      throw bad_format(cur,"property missing value");
    
    cout << "loadXML: " << name << "=" << val << endl;
    
    
    if(xmlStrcmp(name, (const xmlChar *)"distance")==0)
      distance=(float)atof((const char*)val);
    
    xmlFree(val);
    xmlFree(name);
  }
}

void LookoutIREvent::saveXML(xmlNode * node) const {
  if(node==NULL)
    return;
  LookoutPointAtEvent::saveXML(node);
	
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
    throw bad_format(node,"Error: LocomotionEvent xml error on saving param");
  xmlSetProp(cur,(const xmlChar*)"name",(const xmlChar*)"distance");
  char valbuf[20];
  snprintf(valbuf,20,"%g",distance);
  xmlSetProp(cur,(const xmlChar*)"value",(const xmlChar*)valbuf);
}
