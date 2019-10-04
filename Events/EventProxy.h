#ifndef EVENTPROXY_H_
#define EVENTPROXY_H_
#include "Wireless/Socket.h"
#include "Wireless/Wireless.h"
#include "Events/EventListener.h"
#include "Shared/RemoteState.h"
#include "Events/RemoteEvents.h"

/*! This class serves as the host for subscribing to remote events. It
 *  should never be directly manipulated by the user; all interaction
 *  with this class is handled by EventRouter
 */
class EventProxy : public EventListener, public RemoteEvents {
	public:
	EventProxy(int port); //!< EventProxy constructor, takes a port to listen on
	virtual ~EventProxy(); //!< EventProxy destructor

	//! Returns true if the EventProxy is still waiting for a connection or is connected
	bool isActive();

	//! Sends the requested state information to the client robot
	void sendState(RemoteState::StateType dtype);

	//! Forwards any recieved events on to the client robot
	void processEvent(const EventBase& event);

	//! Handles any incoming requests from the client robot
	int processData(char *data, int bytes);

	protected:

	//! Called by processData, handles an oncoming data request packet
	void handleRemoteRequest(RemoteRequest *info);
	
	
	bool listening;
	
	EventProxy(EventProxy&);
	EventProxy &operator=(const EventProxy&);
};

#endif /*EVENTPROXY_H_*/
