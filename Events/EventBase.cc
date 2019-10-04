#include "EventBase.h"
#include <stdio.h>
#include <sstream>
#include <libxml/tree.h>
#include "Shared/debuget.h"
#include "Shared/string_util.h"
#include "Shared/get_time.h"

// just to give linkage for the auto register:
const EventBase::classTypeID_t EventBase::autoRegisterEventBase=getTypeRegistry().registerType<EventBase>(makeClassTypeID("BASE"));
#include "FilterBankEvent.h"
const EventBase::classTypeID_t FilterBankEvent::autoRegisterFilterBankEvent=getTypeRegistry().registerType<FilterBankEvent>(makeClassTypeID("FBKE"));
#include "SegmentedColorFilterBankEvent.h"
const EventBase::classTypeID_t SegmentedColorFilterBankEvent::autoRegisterSegmentedColorFilterBankEvent=getTypeRegistry().registerType<SegmentedColorFilterBankEvent>(makeClassTypeID("SFBK"));

const char* const EventBase::EventGeneratorNames[numEGIDs+1] = {
	"unknownEGID",
	"aiEGID",
	"audioEGID",
	"buttonEGID",
	"cameraResolutionEGID",
	"erouterEGID",
	"estopEGID",
	"grasperEGID",
    "koduEGID",
	"locomotionEGID",
	"lookoutEGID",
	"mapbuilderEGID",
	"micOSndEGID",
	"micRawEGID",
	"micFFTEGID",
	"micPitchEGID",
	"mocapEGID",
	"motmanEGID",
	"pilotEGID",
	"powerEGID",
	"remoteStateEGID",
	"runtimeEGID",
	"sensorEGID",
	"servoEGID",
	"stateMachineEGID",
	"stateSignalEGID",
	"stateTransitionEGID",
	"textmsgEGID",
	"timerEGID",
	"userEGID",
	"visInterleaveEGID",
	"visJPEGEGID",
	"visObjEGID",
	"visOFbkEGID",
	"visPNGEGID",
	"visRawCameraEGID",
	"visRawDepthEGID",
	"visRegionEGID",
	"visRLEEGID",
	"visSegmentEGID",
	"wmVarEGID",
	"worldModelEGID",
	NULL // extra entry as termination marker for plist::NamedEnumeration construction
};

const char* const EventBase::EventTypeNames[numETIDs+1] = {
	"activate",
	"status",
	"deactivate",
	NULL // extra entry as termination marker for plist::NamedEnumeration construction
};

const char* const EventBase::EventTypeAbbr[numETIDs] = { "A", "S", "D" };

int doNameChecks() {
	if(EventBase::EventGeneratorNames[EventBase::numEGIDs-1]==NULL) {
		std::cerr << "\nWARNING: EventBase::EventGeneratorNames ends early - did you add an entry to EventGeneratorID_t but forget to provide a display name in EventBase.cc?  This will probably crash.\n" << std::endl;
	}
	if(EventBase::EventTypeNames[EventBase::numETIDs-1]==NULL) {
		std::cerr << "\nWARNING: EventBase::EventTypeNames ends early - did you add an entry to EventTypeID_t but forget to provide a display name in EventBase.cc?  This will probably crash.\n" << std::endl;
	}
	if(EventBase::EventTypeAbbr[EventBase::numETIDs-1]==NULL) {
		std::cerr << "\nWARNING: EventBase::EventTypeAbbr ends early - did you add an entry to EventTypeID_t but forget to provide an abbreviation in EventBase.cc?  This will probably crash.\n" << std::endl;
	}
	return 0;
}
const int NAME_CHECKS = doNameChecks();

EventBase::registry_t& EventBase::getTypeRegistry() {
	static registry_t typeRegistry;
	return typeRegistry;
}

EventBase::EventBase()
	: XMLLoadSave(), stim_id(), magnitude(0), timestamp(get_time()), saveFormat(XML),
	  nameisgen(true), genID(unknownEGID), typeID(statusETID), sourceID((size_t)-1),
	  hostID(-1), duration(0)
{
	genName();
}

EventBase::EventBase(EventGeneratorID_t gid, size_t sid, EventTypeID_t tid, unsigned int dur)
	: XMLLoadSave(), stim_id(), magnitude(0), timestamp(get_time()), saveFormat(XML), nameisgen(true), genID(gid), typeID(tid), sourceID(sid), hostID(-1), duration(dur)
{
	genName();
	if(tid==deactivateETID)
		setMagnitude(0);
	else
		setMagnitude(1);
}

EventBase::EventBase(EventGeneratorID_t gid, size_t sid, EventTypeID_t tid, unsigned int dur, const std::string& n)
	: XMLLoadSave(), stim_id(), magnitude(0), timestamp(get_time()), saveFormat(XML), nameisgen(true), genID(gid), typeID(tid), sourceID(sid), hostID(-1), duration(dur)
{
	setName(n);
	if(tid==deactivateETID)
		setMagnitude(0);
	else
		setMagnitude(1);
}

EventBase::EventBase(EventGeneratorID_t gid, size_t sid, EventTypeID_t tid, unsigned int dur, const std::string& n, float mag)
	: XMLLoadSave(), stim_id(), magnitude(mag), timestamp(get_time()), saveFormat(XML), nameisgen(true), genID(gid), typeID(tid), sourceID(sid), hostID(-1), duration(dur)
{
	setName(n);
}

EventBase&
EventBase::setName(const std::string& sourcename) {
	stim_id='(';
	if(genID<numEGIDs) {
		stim_id+=EventGeneratorNames[genID];
	} else {
		stim_id+=std::string("InvalidGen");
		char tmp2[16];
		snprintf(tmp2,16,"(%d)",genID);
		stim_id+=tmp2; 
	}
	stim_id+=',';
	stim_id+=sourcename;
	stim_id+=',';
	stim_id+=EventTypeAbbr[getTypeID()];

	if (hostID != -1) {
		stim_id += ',';
		stim_id += string_util::intToStringIP(hostID);
	}
	
	stim_id+=')';
	nameisgen=false;
	return *this;
}

std::string
EventBase::getDescription(bool /*showTypeSpecific=true*/, unsigned int verbosity/*=0*/) const {
	std::ostringstream logdata;
	logdata << getName();
	if(verbosity>=1)
		logdata << '\t' << getGeneratorID() << '\t' << getSourceID() << '\t' << getTypeID();
	if(verbosity>=2)
		logdata << '\t' << getDuration() << '\t' << getTimeStamp();
	if(verbosity>=3)
		logdata << '\t' << getMagnitude();
	return logdata.str();
}

unsigned int
EventBase::getBinSize() const {
	if(saveFormat==XML)
		return XMLLoadSave::getBinSize();
	unsigned int used=0;
	used+=creatorSize("EventBase");
	used+=getSerializedSize(stim_id);
	used+=getSerializedSize(magnitude);
	used+=getSerializedSize(timestamp);
	used+=getSerializedSize(nameisgen);
	used+=getSerializedSize<char>(); //genID is an enum, override to char
	used+=getSerializedSize<char>(); //typeID is an enum, override to char
	used+=getSerializedSize(sourceID);
	used+=getSerializedSize(hostID);
	used+=getSerializedSize(duration);
	return used;
}

unsigned int
EventBase::loadBinaryBuffer(const char buf[], unsigned int len) {
	unsigned int origlen=len;
	if(!checkCreatorInc("EventBase",buf,len,true)) return 0;
	if(!decodeInc(stim_id,buf,len)) return 0;
	if(!decodeInc(magnitude,buf,len)) return 0;
	if(!decodeInc(timestamp,buf,len)) return 0;
	if(!decodeInc(nameisgen,buf,len)) return 0;
	char tmp;
	if(!decodeInc(tmp,buf,len)) return 0;
	genID=(EventGeneratorID_t)tmp;
	if(!decodeInc(tmp,buf,len)) return 0;
	typeID=(EventTypeID_t)tmp;
	if(!decodeInc(sourceID,buf,len)) return 0;
	if(!decodeInc(hostID,buf,len)) return 0;
	if(!decodeInc(duration,buf,len)) return 0;

	genName();
	return origlen-len;	
}

unsigned int
EventBase::saveBinaryBuffer(char buf[], unsigned int len) const {
	unsigned int origlen=len;
	if(!saveCreatorInc("EventBase",buf,len)) return 0;
	if(!encodeInc(stim_id,buf,len)) return 0;
	if(!encodeInc(magnitude,buf,len)) return 0;
	if(!encodeInc(timestamp,buf,len)) return 0;
	if(!encodeInc(nameisgen,buf,len)) return 0;
	if(!encodeInc((char)genID,buf,len)) return 0;
	if(!encodeInc((char)typeID,buf,len)) return 0;
	if(!encodeInc(sourceID,buf,len)) return 0;
	if(!encodeInc(hostID,buf,len)) return 0;
	if(!encodeInc(duration,buf,len)) return 0;
	return origlen-len;
}

void EventBase::loadXML(xmlNode* node) {
	if(xmlStrcmp(node->name, (const xmlChar *)"event"))
		throw bad_format(node,"Load of the wrong type -- expecting 'event' node");
	unsigned int i;
	xmlChar* str = xmlGetProp(node,(const xmlChar*)"egid");
	if(str==NULL)
		throw bad_format(node,"missing generator id");
	for(i=0; i<numEGIDs; i++) {
		if(xmlStrcmp(str,(const xmlChar*)EventGeneratorNames[i])==0)
			break;
	}
	xmlFree(str);
	if(i==numEGIDs)
		throw bad_format(node,"bad event generator name");
	genID=static_cast<EventGeneratorID_t>(i);
	
	str = xmlGetProp(node,(const xmlChar*)"sid");
	if(str==NULL)
		throw bad_format(node,"missing source id");
	std::stringstream((const char*)str) >> sourceID;
	xmlFree(str);

	str = xmlGetProp(node,(const xmlChar*)"hid");
	if(str==NULL)
		throw bad_format(node,"missing host id");
	hostID=atoi((const char*)str);
	xmlFree(str);
	
	str = xmlGetProp(node,(const xmlChar*)"etid");
	if(str==NULL)
		throw bad_format(node,"missing type id");
	for(i=0; i<numETIDs; i++) {
		if(xmlStrcmp(str,(const xmlChar*)EventTypeAbbr[i])==0)
			break;
		if(xmlStrcmp(str,(const xmlChar*)EventTypeNames[i])==0)
			break;
	}
	xmlFree(str);
	if(i==numETIDs)
		throw bad_format(node,"bad event type name");
	typeID=static_cast<EventTypeID_t>(i);
	str = xmlGetProp(node,(const xmlChar*)"time");
	if(str==NULL)
		throw bad_format(node,"missing timestamp");
	timestamp=atoi((const char*)str);
	xmlFree(str);
	str = xmlGetProp(node,(const xmlChar*)"duration");
	if(str==NULL)
		throw bad_format(node,"missing duration");
	duration=atoi((const char*)str);
	xmlFree(str);
	str = xmlGetProp(node,(const xmlChar*)"magnitude");
	if(str==NULL)
		throw bad_format(node,"missing magnitude");
	magnitude=atoi((const char*)str);
	xmlFree(str);
	str = xmlGetProp(node,(const xmlChar*)"custom_name");
	if(str!=NULL) {
		setName((const char*)str);
		xmlFree(str);
	} else
		genName();
}
void EventBase::saveXML(xmlNode * node) const {
	xmlNodeSetName(node,(const xmlChar*)"event");
	xmlSetProp(node,(const xmlChar*)"egid",(const xmlChar*)EventGeneratorNames[genID]);
	const size_t BUFSIZE=30;
	char buf[BUFSIZE];
	
	snprintf(buf,BUFSIZE,"%lu",static_cast<unsigned long>(sourceID));
	xmlSetProp(node,(const xmlChar*)"sid",(const xmlChar*)buf);

	snprintf(buf,BUFSIZE,"%u",hostID);
	xmlSetProp(node,(const xmlChar*)"hid",(const xmlChar*)buf);
	
	xmlSetProp(node,(const xmlChar*)"etid",(const xmlChar*)EventTypeAbbr[typeID]);
	snprintf(buf,BUFSIZE,"%u",timestamp);
	xmlSetProp(node,(const xmlChar*)"time",(const xmlChar*)buf);
	snprintf(buf,BUFSIZE,"%u",duration);
	xmlSetProp(node,(const xmlChar*)"duration",(const xmlChar*)buf);
	snprintf(buf,BUFSIZE,"%g",magnitude);
	xmlSetProp(node,(const xmlChar*)"magnitude",(const xmlChar*)buf);
	if(!nameisgen) {
		//extract custom name
		size_t b=stim_id.find(',');
		ASSERTRET(b!=std::string::npos,"malformed event name! (no comma)");
		size_t e=stim_id.find(',',++b);
		ASSERTRET(e!=std::string::npos,"malformed event name! (no 2nd comma)");
		xmlSetProp(node,(const xmlChar*)"custom_name",(const xmlChar*)stim_id.substr(b,e-b).c_str());
	}
}

void
EventBase::genName() {
	if(!nameisgen)
		return;
	const size_t BUFSIZE=30;
	char tmp[BUFSIZE];
	snprintf(tmp,BUFSIZE,"%lu",static_cast<unsigned long>(sourceID));
	setName(tmp);
	nameisgen=true;
}

/*! @file
 * @brief Implements EventBase, the basic class for sending events around the system
 * @author ejt (Creator)
 */

