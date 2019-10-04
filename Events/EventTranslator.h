//-*-c++-*-
#ifndef INCLUDED_EventTranslator_h_
#define INCLUDED_EventTranslator_h_

#include "Events/EventTrapper.h"
#include "Events/EventListener.h"
#include "IPC/MutexLock.h"
#include <map>

//! EventTranslator receives events from EventRouters in non-Main processes and adds them into a SharedQueue for Main to pick up
class EventTranslator : public EventTrapper, public EventListener {
public:
	//!constructor
	EventTranslator() : trapRet(false) {}

	//!destructor
	virtual ~EventTranslator() {}

	//! Call this with events which should be forwarded to other processes
	/*! @param event the event to serialize and send
	 *  @param onlyReady if true, only send the event to observers which do not have any message backlog (if supported by transfer mechanism) */
	virtual void encodeEvent(const EventBase& event, bool onlyReady=false);

	//! Call this with events which should be forwarded to other processes (redirects to encodeEvent())
	/*! By providing an EventTrapper interface, you can directly
	 *  register this class with an EventRouter instead of having to
	 *  manually forward events (although you could do that as well)
	 *  @return #trapRet, which you can set via setTrapEventValue() */
	virtual bool trapEvent(const EventBase& event) { encodeEvent(event); return trapRet; }

	//! Call this with events which should be forwarded to other processes (redirects to encodeEvent())
	/*! By providing an EventListener interface, you can directly
	 *  register this class with an EventRouter instead of having to
	 *  manually forward events (although you could do that as well) */
	virtual void processEvent(const EventBase& event) { encodeEvent(event); }

	//! Called with buffers containing incoming events which should be reconstituted
	/*! @return the reconstituted event, or NULL if an error occured (malformed data) */
	static EventBase* decodeEvent(const char* buf, unsigned int size);

	//! set #trapRet, which can let you decide whether trapped events should be filtered or not
	virtual void setTrapEventValue(bool v) { trapRet=v; }
	

protected:

	//! Called by encodeEvent() to request a buffer for serializing into, must be at least @a size
	/*! This buffer will then be sent to post(), which should free
	 *  it (or recycle it for usage by a later bufferRequest()) */
	virtual char* bufferRequest(unsigned int size)=0;

	//! Called by encodeEvent() after serialization is complete for communication to other processes
	/*! @param buf the data to be sent, will be a buffer previously requested from #bufferRequest
	 *  @param size the number of bytes to send
	 *  @param onlyReady if true, only send the event to observers which do not have any message backlog (if supported by transfer mechanism)
	 *
	 *  You will always get this callback after each call to bufferRequest(), even
	 *  in the event of an error during saving.  If an error occured, the callback
	 *  will receive 0 for size.*/
	virtual void post(const char* buf, unsigned int size, bool onlyReady)=0;

	//! The value which trapEvent() should return
	bool trapRet;

private:
	EventTranslator(const EventTranslator&); //!< don't call
	EventTranslator& operator=(const EventTranslator&); //!< don't call
};

class EventRouter;

//! For completeness, if you want to have events be piped directly to the local erouter instead having to be encoded and decoded
/*! Unfortunately, this still entails a memory copy of the event since
 *  we have to make a new event for posting to the event router.  We
 *  could avoid this if events were reference counted or if there was
 *  a way to direct the EventRouter not to free the event after
 *  processing.
 *
 *  Beware of subscribing this class as a listener to the same
 *  EventRouter that it is sending to -- could cause infinite
 *  recursion */
class NoOpEventTranslator : public EventTranslator {
public:
	//! constructor
	explicit NoOpEventTranslator(EventRouter& er) : EventTranslator(), evtRouter(er) {}

	virtual void encodeEvent(const EventBase& event, bool onlyReady=false);

protected:
	//! should never be called, only included to satisfy interface
	virtual char* bufferRequest(unsigned int /*size*/) { return NULL; }
	//! should never be called, only included to satisfy interface
	virtual void post(const char* /*buf*/, unsigned int /*size*/, bool /*onlyReady*/) { }
	
	EventRouter& evtRouter; //!< the EventRouter to send events to
};


#ifdef PLATFORM_APERIOS
class OSubject;
#else
class MessageQueueBase;
#endif
class RCRegion;

//! An implementation of EventTranslator which will forward events using the inter-process mechanisms of the current platform
/*! The current implementation creates an RCRegion for each event and
 *  then releases its reference to the region after it is sent.  A
 *  more efficient implementation might retain a queue of recycled
 *  RCRegions to reduce allocation costs */
class IPCEventTranslator : public EventTranslator {
public:

#ifdef PLATFORM_APERIOS
	typedef OSubject IPCSender_t; //!< the class for sending IPC messages on aperios
#else
	typedef MessageQueueBase IPCSender_t; //!< the class for sending IPC messages on unix-based systems
#endif

	//! constructor
	explicit IPCEventTranslator(IPCSender_t& subj) : EventTranslator(), subject(subj), curRegion(NULL), curName(), lock() {}
	
	//! extends base class's implementation to store @a event.getName() into #curName 
	virtual void encodeEvent(const EventBase& event, bool onlyReady=false) {
		curName=event.getName();
		EventTranslator::encodeEvent(event,onlyReady);
	}

protected:
	virtual char* bufferRequest(unsigned int size);
	virtual void post(const char* buf, unsigned int size, bool onlyReady);
	
	IPCSender_t& subject; //!< where to post messages upon serialization, set by constructor
	RCRegion* curRegion; //!< the region currently being serialized into, only valid between call to bufferRequest() and following post()
	std::string curName; //!< name of current event being posted (for error messages)
	MutexLock<ProcessID::NumProcesses> lock; //!< prevent concurrent posts, held for the duration of #curRegion

private:
	IPCEventTranslator(const IPCEventTranslator&); //!< don't call
	IPCEventTranslator operator=(const IPCEventTranslator&); //!< don't call
};

/*! @file
 * @brief Describes EventTranslator and IPCEventTranslator, which receives events from EventRouters in non-Main processes and adds them into a SharedQueue for Main to pick up
 * @author ejt (Creator)
 */

#endif
