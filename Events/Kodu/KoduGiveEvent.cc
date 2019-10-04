#include "Events/Kodu/KoduGiveEvent.h"

const char* KoduGiveEvent::classID = "KoduEventBase::KoduGiveEvent";

std::string KoduGiveEvent::getDescription(bool showTypeSpecific, unsigned int verbosity) const {
	if (!showTypeSpecific)
		return KoduEventBase::getDescription(showTypeSpecific,verbosity);
	std::ostringstream logdata;
	logdata << KoduEventBase::getDescription(showTypeSpecific,verbosity) << "Give";
	return logdata.str();
}

unsigned int KoduGiveEvent::getBinSize() const {
	unsigned int used = KoduEventBase::getBinSize();
	// If using XML, the XMLLoadSave::getBinSize (called by EventBase::getBinSize) is all we need.
	if (saveFormat==XML)
		return used;
	// Otherwise need to add our own fields.
	used += creatorSize(KoduGiveEvent::classID);
	return used;
}

unsigned int KoduGiveEvent::saveBinaryBuffer(char buf[], unsigned int len) const {
	unsigned int origlen = len;
	if (!checkInc(KoduEventBase::saveBinaryBuffer(buf,len),buf,len)) return 0;
	if (!saveCreatorInc(KoduGiveEvent::classID,buf,len)) return 0;
    if (!encodeInc(_turn,buf,len)) return 0;
    if (!encodeInc(_objType,buf,len)) return 0;
	return origlen-len;
}

unsigned int KoduGiveEvent::loadBinaryBuffer(const char buf[], unsigned int len) {
	unsigned int origlen = len;
	if (!checkInc(KoduEventBase::loadBinaryBuffer(buf,len),buf,len)) return 0;
	if (!checkCreatorInc(KoduGiveEvent::classID,buf,len,true)) return 0;
    if (!decodeInc(_turn,buf,len)) return 0;
    if (!decodeInc(_objType,buf,len)) return 0;
	return origlen-len;
}

void KoduGiveEvent::setTurnAndObject(float turn, std::string objType) {
    _turn = turn;
    _objType = objType;
}

float KoduGiveEvent::getTurn() const {
    return _turn;
}

std::string KoduGiveEvent::getObjType() const {
    return _objType;
}

/*! @file
 *
 * @brief Kodu gives something to another robot
 *
 * @author afranchu
 * @author medee
 */
