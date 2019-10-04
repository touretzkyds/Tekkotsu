#include "Events/Kodu/KoduSayEvent.h"

const char* KoduSayEvent::classID = "KoduEventBase::KoduSayEvent";

std::string KoduSayEvent::getDescription(bool showTypeSpecific, unsigned int verbosity) const {
	if (!showTypeSpecific)
		return KoduEventBase::getDescription(showTypeSpecific,verbosity);
	std::ostringstream logdata;
	logdata << KoduEventBase::getDescription(showTypeSpecific,verbosity) << '\t' << _phrase;
	return logdata.str();
}

unsigned int KoduSayEvent::getBinSize() const {
	unsigned int used = KoduEventBase::getBinSize();
	// If using XML, the XMLLoadSave::getBinSize (called by EventBase::getBinSize) is all we need.
	if (saveFormat==XML)
		return used;
	// Otherwise need to add our own fields.
	used += creatorSize(KoduSayEvent::classID);
	used += getSerializedSize(_phrase);
	return used;
}

unsigned int KoduSayEvent::saveBinaryBuffer(char buf[], unsigned int len) const {
	unsigned int origlen = len;
	if (!checkInc(KoduEventBase::saveBinaryBuffer(buf,len),buf,len)) return 0;
	if (!saveCreatorInc(KoduSayEvent::classID,buf,len)) return 0;
	if (!encodeInc(_phrase,buf,len)) return 0;
	return origlen-len;
}

unsigned int KoduSayEvent::loadBinaryBuffer(const char buf[], unsigned int len) {
	unsigned int origlen = len;
	if (!checkInc(KoduEventBase::loadBinaryBuffer(buf,len),buf,len)) return 0;
	if (!checkCreatorInc(KoduSayEvent::classID,buf,len,true)) return 0;
	if (!decodeInc(_phrase,buf,len)) return 0;
	return origlen-len; 
}

std::string KoduSayEvent::getPhrase() const {
	return _phrase;
}

/*! @file
 * 
 * @brief Kodu says something in the world
 *
 * @author afranchu
 * @author medee
 */
