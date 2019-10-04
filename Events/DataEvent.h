//-*-c++-*-
#ifndef INCLUDED_DataEvent_h_
#define INCLUDED_DataEvent_h_

#include "Events/EventBase.h"
#include <sstream>
#include <libxml/tree.h>

//! Event type for passing around data (or pointers to data).  In a state machine, use a SignalTrans to test for a specific data value and make the sid the address of the node posting the event.
template<class T, int TID=-1>
class DataEvent : public EventBase {
public:
	//!@name Constructors
	//!
	DataEvent() : EventBase(), data() {}
	DataEvent(const T& d, EventGeneratorID_t gid, size_t sid, EventTypeID_t tid, unsigned int dur=0) : EventBase(gid,sid,tid,dur), data(d) {}
	DataEvent(const T& d, EventGeneratorID_t gid, size_t sid, EventTypeID_t tid, unsigned int dur, const std::string& n, float mag) : EventBase(gid,sid,tid,dur,n,mag), data(d) {}

	//! copy constructor
	DataEvent(const DataEvent& evt) : EventBase(evt), data(evt.data) {}
	
	//! assignment
	const DataEvent& operator=(const DataEvent& evt) { EventBase::operator=(evt); data=evt.data; return *this; }

	virtual EventBase* clone() const { return new DataEvent<T>(*this); }
	//@}

	void setData(const T& d) { data=d; } //!< assigns @a d to the internal #data
	const T& getData() const { return data; } //!< returns #data
	T& getData() { return data; } //!< returns #data
	
	inline static unsigned int encode(const T& x, char buf[], unsigned int cap) { if(cap<sizeof(T)) return 0; memcpy(buf,(const void*)&x,sizeof(x)); return sizeof(x); }
	inline static unsigned int decode(T& x, const char buf[], unsigned int cap) { if(cap<sizeof(T)) return 0; memcpy((void*)&x,buf,sizeof(x)); return sizeof(x); }
	
	virtual unsigned int getBinSize() const;
	virtual unsigned int loadBinaryBuffer(const char buf[], unsigned int len);
	virtual unsigned int saveBinaryBuffer(char buf[], unsigned int len) const;
	virtual void loadXML(xmlNode* node);
	virtual void saveXML(xmlNode * node) const;
	
	virtual classTypeID_t getClassTypeID() const { return autoRegisterDataEvent; }

	virtual void getDataFromString(std::stringstream &s);
	virtual void sendDataToString(std::stringstream &s) const;

protected:
	T data; //!< the data being communicated
	
	//! this silliness is to work around parsing issue in gcc 3.3 branch
	static EventBase::classTypeID_t registerDataType(EventBase::classTypeID_t classid) {
#if !defined(__GNUC__) || __GNUC__>3 || __GNUC__==3 && __GNUC_MINOR__>3
		// if here, using gcc 3.4 or later...
		// gcc 3.3 won't let me call this templated member function?!?!
		registry_t& reg = getTypeRegistry();
		return reg.registerType<DataEvent>(classid);
#else // using gcc 3.3.x or prior
		// instead I have to wind up registering my own instance instead of using the FamilyFactory's instance generation
		return getTypeRegistry().registerFactory(classid,new EventBase::registry_t::FactoryType< DataEvent<T,TID> >);
#endif
	}
	static const EventBase::classTypeID_t autoRegisterDataEvent; //!< causes class type id to automatically be regsitered with EventBase's FamilyFactory (getTypeRegistry())
};

#define DATAEVENT_IMPLEMENTATION(_abstract_type, _implementation) \
  template<> void DataEvent<_abstract_type, -1>::getDataFromString(std::stringstream &ss) \
   { _implementation x; ss >> x; data = (_abstract_type)x; } \
  template<> void DataEvent<_abstract_type, -1>::sendDataToString(std::stringstream &ss) const { ss << (_implementation)data; }

#define DATAEVENT_IMPLEMENTATION_H(_abstract_type, _implementation) \
  template<> void DataEvent<_abstract_type, -1>::getDataFromString(std::stringstream &ss); \
  template<> void DataEvent<_abstract_type, -1>::sendDataToString(std::stringstream &ss) const;

#define DATAEVENT_IMPLEMENTATION_CC(_abstract_type, _implementation) \
  template<> void DataEvent<_abstract_type, -1>::getDataFromString(std::stringstream &ss) \
   { _implementation x; ss >> x; data = (_abstract_type)x; } \
  template<> void DataEvent<_abstract_type, -1>::sendDataToString(std::stringstream &ss) const { ss << (_implementation)data; }


//**************** IMPLEMENTATION ****************

template<class T, int TID>
const EventBase::classTypeID_t DataEvent<T,TID>::autoRegisterDataEvent=DataEvent<T,TID>::registerDataType(makeClassTypeID("DATA")+(TID<0?static_cast<EventBase::classTypeID_t>(getTypeRegistry().getNumTypes()):static_cast<EventBase::classTypeID_t>(TID)));

template<class T, int TID>
unsigned int DataEvent<T,TID>::getBinSize() const {
	unsigned int used=EventBase::getBinSize();
	if(saveFormat==XML)
		return used; //if using XML, the XMLLoadSave::getBinSize (called by EventBase::getBinSize) is all we need
	//otherwise need to add our own fields
	used+=creatorSize("EventBase::DataEvent");
	used+=getSerializedSize(data);
	return used;
}

template<class T, int TID>
unsigned int DataEvent<T,TID>::loadBinaryBuffer(const char buf[], unsigned int len) {
  unsigned int origlen=len;
  if(!checkInc(EventBase::loadBinaryBuffer(buf,len),buf,len)) return 0;
  if(!checkCreatorInc("EventBase::DataEvent",buf,len,true)) return 0;
  unsigned int used = decode(data,buf,len);
  if(used==0) return 0; else { buf+=used; len-=used; }
  return origlen-len;	
}

template<class T, int TID>
unsigned int DataEvent<T,TID>::saveBinaryBuffer(char buf[], unsigned int len) const {
  unsigned int origlen=len;
  if(!checkInc(EventBase::saveBinaryBuffer(buf,len),buf,len)) return 0;
  if(!saveCreatorInc("EventBase::DataEvent",buf,len)) return 0;
  unsigned int used = encode(data,buf,len);
  if(used==0) return 0; else { buf+=used; len-=used; }
  return origlen-len;
}

template<class T, int TID>
void DataEvent<T,TID>::loadXML(xmlNode* node) {
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
    
    if(xmlStrcmp(name, (const xmlChar *)"data")==0) {
			std::stringstream ss((const char*)val);
			DataEvent<T,-1>::getDataFromString(ss);
      xmlFree(val);
      xmlFree(name);
    }
  }
}

template<class T, int TID>
void DataEvent<T,TID>::getDataFromString(std::stringstream&) {
  std::cout << "DataEvent failure: don't know how to read data of this type." << std::endl;
}

template<> void DataEvent<unsigned char, -1>::getDataFromString(std::stringstream &ss);
template<> void DataEvent<unsigned short int, -1>::getDataFromString(std::stringstream &ss);
template<> void DataEvent<unsigned int, -1>::getDataFromString(std::stringstream &ss);
template<> void DataEvent<int, -1>::getDataFromString(std::stringstream &ss);
template<> void DataEvent<float, -1>::getDataFromString(std::stringstream &ss);
template<> void DataEvent<double, -1>::getDataFromString(std::stringstream &ss);

template<class T, int TID>
void DataEvent<T,TID>::sendDataToString(std::stringstream&) const {
  std::cout << "DataEvent failure: don't know how to encode data of this type as a string." << std::endl;
}

template<> void DataEvent<unsigned char, -1>::sendDataToString(std::stringstream &ss) const;
template<> void DataEvent<unsigned short int, -1>::sendDataToString(std::stringstream &ss) const;
template<> void DataEvent<unsigned int, -1>::sendDataToString(std::stringstream &ss) const;
template<> void DataEvent<int, -1>::sendDataToString(std::stringstream &ss) const;
template<> void DataEvent<float, -1>::sendDataToString(std::stringstream &ss) const;
template<> void DataEvent<double, -1>::sendDataToString(std::stringstream &ss) const;

template<class T, int TID>
void DataEvent<T,TID>::saveXML(xmlNode* node) const {
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
  
  // write new params
  xmlNode* cur=xmlNewChild(node,NULL,(const xmlChar*)"param",NULL);	\
  if(cur==NULL)								\
    throw bad_format(node,"Error: DataEvent xml error on saving param"); \
  xmlSetProp(cur,(const xmlChar*)"name",(const xmlChar*)"data");		\
  std::stringstream ss;
  DataEvent<T,-1>::sendDataToString(ss);
  xmlSetProp(cur,(const xmlChar*)"value",(const xmlChar*)ss.str().c_str());
}

/*! @file
 * @brief Defines DataEvent, for passing around data (or pointers to data)
 * @author ejt (Creator)
 */

#endif
