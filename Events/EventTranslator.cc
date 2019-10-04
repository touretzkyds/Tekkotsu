#include "EventTranslator.h"
#include "Events/EventRouter.h"
#include "Shared/debuget.h"
#include "Shared/ProjectInterface.h"
#include <iostream>

#ifdef PLATFORM_APERIOS
#  include <OPENR/OSubject.h>
#else
#  include "IPC/MessageQueue.h"
#endif
#include "IPC/RCRegion.h"

using namespace std;

void
EventTranslator::encodeEvent(const EventBase& event, bool onlyReady/*=false*/) {
	event.setSaveFormat(EventBase::BINARY);
	EventBase::classTypeID_t header=event.getClassTypeID();
	const unsigned int bufsize=LoadSave::getSerializedSize(header)+event.getBinSize();
	char * const buf=bufferRequest(bufsize);
	if(buf==NULL) {
		cerr << "ERROR: EventTranslator unable to transmit event because requested buffer was NULL" << endl;
		return;
	}
	char * cur=buf;
	unsigned int remain=bufsize;
	if(!LoadSave::encodeInc(header,cur,remain,"Ran out of space encoding event header")) return;
	unsigned int used=event.saveBuffer(cur,remain);
	if(used==0) {
		cerr << "ERROR: EventTranslator unable to transmit event because EventBase::saveBuffer failed (buffer==" << (void*)(cur) << ", size==" << remain << ")" << endl;
		post(buf,0,onlyReady);
		return;
	}
	cur+=used;
	remain-=used;
	ASSERT(cur-buf==(int)(bufsize-remain),"used count does not match offset");
	post(buf,bufsize-remain,onlyReady);
	return;
}

EventBase*
EventTranslator::decodeEvent(const char * entry, unsigned int size) {
	EventBase::classTypeID_t header;
	if(!LoadSave::decodeInc(header,entry,size,"Ran out of space decoding event")) return NULL;
	//cout << "decodeEvent(" << (void*)entry << ","<< size << ") header is " << header << " (" << entry[0] << entry[1] << entry[2] << entry[3] << ")" << endl;
	EventBase* evt=EventBase::getTypeRegistry().create(header);
	if(evt==NULL) {
		cerr << "ERROR: EventTranslator unable to translate buffer because header does not match a previously registered class type id" << endl;
		return NULL;
	}
	evt->setSaveFormat(EventBase::BINARY);
	if(evt->loadBuffer(entry,size)==0) {
		cerr << "ERROR: EventTranlator unable to translate buffer because data is malformed (EventBase::loadBuffer failed)" << endl;
		return NULL;
	}
	//cout << "decodes to " << evt->getDescription() << endl;
	return evt;
}

void
NoOpEventTranslator::encodeEvent(const EventBase& event, bool /*onlyReady=false*/) {
	evtRouter.postEvent(event);
}

char*
IPCEventTranslator::bufferRequest(unsigned int size) {
	ASSERT(curRegion==NULL,"WARNING: IPCEventTranslator::bufferRequest() curRegion was not NULL");
	try {
		lock.lock(ProcessID::getID());
		curRegion = new RCRegion(size);
		return curRegion->Base();
	} catch(...) {
		lock.unlock();
		curRegion=NULL;
		throw;
	}
}

void
IPCEventTranslator::post(const char* buf, unsigned int size, bool onlyReady) {
	ASSERTRET(curRegion!=NULL,"ERROR: IPCEventTranslator::post(buf,size) was NULL");
	if(size==0) {
		curRegion->RemoveReference();
		curRegion=NULL;
		return;
	}
	if(buf!=curRegion->Base()) {
		cerr << "ERROR: IPCEventTranslator::post(buf,size) buf does not match value given from previous bufferRequest()" << endl;
		return;
	}
#ifdef PLATFORM_APERIOS
	if(!onlyReady) {
		subject.SetData(curRegion);
		subject.NotifyObservers();
	} else {
		for(ObserverConstIterator it=subject.begin(); it!=subject.end(); ++it) {
			if(subject.IsReady(*it)) {
				subject.SetData(*it,curRegion);
				subject.NotifyObserver(*it);
			}
		}
	}
#else
	if(!onlyReady || subject.getMessageSN(subject.newest())==subject.getMessagesRead()) {
		try {
			subject.sendMessage(curRegion);
		} catch(const std::exception& ex) {
			static char errmsg[256];
			strncpy(errmsg,("Occurred during IPCEventTranslator::post(), dropping interprocess event "+curName).c_str(),256);
			ProjectInterface::uncaughtException(__FILE__,__LINE__,errmsg,&ex);
		} catch(...) {
			static char errmsg[256];
			strncpy(errmsg,("Occurred during IPCEventTranslator::post(), dropping interprocess event "+curName).c_str(),256);
			ProjectInterface::uncaughtException(__FILE__,__LINE__,errmsg,NULL);
		}
	}
#endif
	curRegion->RemoveReference();
	curRegion=NULL;
	lock.unlock();
}
	

/*! @file
 * @brief Implements EventTranslator, which receives events from EventRouters in non-Main processes and adds them into a SharedQueue for Main to pick up
 * @author ejt (Creator)
 */

