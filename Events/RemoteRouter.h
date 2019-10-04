#ifndef REMOTEROUTER_H_
#define REMOTEROUTER_H_

#include <string>
#include "Wireless/Socket.h"
#include "Wireless/Wireless.h"
#include "Events/RemoteEvents.h"
#include "Events/EventProxy.h"
#include "Events/EventListener.h"
#include "Shared/RemoteState.h"

#include <vector>
#include <list>

class RemoteState;

/* ! This class allows a client robot to subscribe to events and state updates on a remote robot. It receives events and state from EventProxy on the 
  * remote server robot. It is started automatically. Subscribe to events on remote robots using EventRouter::addRemoteListener() and state updates using
  * EventRouter::requestRemoteStateUpdates()
  */
class RemoteRouter : public RemoteEvents, public EventListener {
	public:

	//! constructor
	RemoteRouter(int host);
	
	//! destructor
	virtual ~RemoteRouter();

	//! Request state data from a remote robot every 'interval' amount of time. Use EventRouter::requestRemoteStateUpdates() rather than calling this directly.
	void requestStateUpdates(RemoteState::StateType type, unsigned int interval);
	
	//! Cancels state data updates from a remote dog for the given StateType. Use EventRouter::stopRemoteStateUpdates() rather than calling this directly.
	void stopStateUpdates(RemoteState::StateType type);

	//! Add remote listener by EGID. Use EventRouter::addRemoteListener() rather than calling this directly.
	void addListener(EventBase::EventGeneratorID_t egid);

	//! Add remote listener by EGID and SID. Use EventRouter::addRemoteListener() rather than calling this directly.
	void addListener(EventBase::EventGeneratorID_t egid,
					 size_t sid);

	//! Add remote listener by EGID, SID, and ETID. Use EventRouter::addRemoteListener() rather than calling this directly.
	void addListener(EventBase::EventGeneratorID_t egid,
					 size_t sid,
					 EventBase::EventTypeID_t etid);

	//! Remove remote listener by EGID. Use EventRouter::removeRemoteListener() rather than calling this directly.
	void removeListener(EventBase::EventGeneratorID_t egid);

	//! Remove remote listener by EGID and SID. Use EventRouter::removeRemoteListener() rather than calling this directly.
	void removeListener(EventBase::EventGeneratorID_t egid,
						size_t sid);

	//! Remove remote listener by EGID, SID, and ETID. Use EventRouter::removeRemoteListener() rather than calling this directly.
	void removeListener(EventBase::EventGeneratorID_t egid,
						size_t sid,
						EventBase::EventTypeID_t etid);

	RemoteState *rstate;

	//! Processes timer events which wait for connections to remote EventProxy.
	void processEvent(const EventBase& event);
	
	//! Receives data from remote EventProxy and forwards it to the correct function according to the data type (Event or State)
	int processData(char *data, int bytes);

	protected:

	//! Returns true when robot is connected to remote robot
	bool isReady() {
		return !waitingForPort && isConnected();
	}

	//! Connect robot on specified port
	void connect(int port);

	bool waitingForPort;
	std::list<RemoteRequest> requestQueue;
	bool timerActive;
	int waitTime;
	
	int remoteHost;
	
	//! Add a remote request to the request queue
	void sendRemoteRequest(RemoteRequest& info);
	
	//! Decode the event from the buffer and post it locally
	void forwardEvent(std::vector<char>& event);
	
	RemoteRouter(RemoteRouter&);
	RemoteRouter &operator=(const RemoteRouter&);
};

#endif /*REMOTEROUTER_H_*/
