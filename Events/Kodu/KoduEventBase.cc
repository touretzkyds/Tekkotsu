#include <iostream>

#include "KoduSayEvent.h"

using namespace std;

const char* KoduEventBase::classID = "EventBase::KoduEvent";

KoduEventBase* KoduEventBase::getInstance(const char buf[], unsigned int size) {
	KoduEventBase eb;
	if (!eb.checkInc((int)eb.loadBinaryBuffer(buf,size), buf, size)) {
			cout << "Error: Received event is not a KoduEvent" << endl;
			return NULL;
    }

    if (eb.checkCreator(KoduSayEvent::classID, buf, size, false)) {
			return new KoduSayEvent();
    }
    else
			return NULL;
}

int KoduEventBase::getDestination() {
    return _destID;
}

void KoduEventBase::setDestination(int destID) {
    _destID = destID;
}

std::string KoduEventBase::getDescription(bool showTypeSpecific, unsigned int verbosity) const {
    if ( !showTypeSpecific )
        return EventBase::getDescription(showTypeSpecific,verbosity);
    std::ostringstream logdata;
    logdata << EventBase::getDescription(showTypeSpecific,verbosity) << '\t' << "KoduEventBase" << '\t' << _destID;
    return logdata.str();
}

unsigned int KoduEventBase::getBinSize() const {
    unsigned int used=EventBase::getBinSize();
    if(saveFormat==XML)
        return used; //if using XML, the XMLLoadSave::getBinSize (called by EventBase::getBinSize) is all we need
    //otherwise need to add our own fields
    used+=creatorSize(KoduEventBase::classID);
    used+=getSerializedSize(_destID);
    return used;
}

unsigned int KoduEventBase::saveBinaryBuffer(char buf[], unsigned int len) const {
    unsigned int origlen=len;
    if (!checkInc(EventBase::saveBinaryBuffer(buf,len),buf,len)) return 0;
    if (!saveCreatorInc(KoduEventBase::classID,buf,len)) return 0;
    if (!encodeInc(_destID,buf,len)) return 0;
    return origlen-len;
}

unsigned int KoduEventBase::loadBinaryBuffer(const char buf[], unsigned int len) {
    unsigned int origlen = len;
    if (!checkInc(EventBase::loadBinaryBuffer(buf,len),buf,len)) return 0;
    if (!checkCreatorInc(KoduEventBase::classID,buf,len,true)) return 0;
    if (!decodeInc(_destID,buf,len)) return 0;
    return origlen-len; 
}

/**
 * @file
 *
 * @brief Events involved with the Kodu Interpreter
 *
 * @author afranchu
 * @author medee
 */
