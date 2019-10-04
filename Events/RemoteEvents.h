#ifndef _REMOTEEVENTS_H_
#define _REMOTEEVENTS_H_

#include "Wireless/Socket.h"
#include "Wireless/Wireless.h"
#include "Wireless/SocketListener.h"
#include "Events/EventBase.h"
#include "Events/NetworkBuffer.h"
#include "Shared/RemoteState.h"
#include <vector>
#include <string>


/*! This class contains the network code common between RemoteRouter
 *  and EventProxy. It contains methods for sending and receiving
 *  events, state updates, and requests to recieve the former two
 *  things. */
class RemoteEvents : public SocketListener {
    public:

	//! This is sent in the header of any data sent over the wireless,
	//it indicates what type of data it is
    enum BufferType {
        Invalid,
        StateData,
        EventData,
        RequestData
    };

	//! This is sent in the header of any requests for remote events
	//or state updates
	enum RequestType {
		EventListenerRequest,
		StateUpdateRequest,
		RemoveEventListenerRequest,
		StopStateUpdateRequest
	};
	
	//! Returns true of the socket is connected
    bool isConnected();

	//! Returns the remote IP address as a string
	std::string remoteIPString();

	//! Returns the remote IP address as an int
	int remoteIPInt();

	
	static const int defaultBufferSize = 1024;
	
    protected:

    //This is so the class can't be instantiated by itself
    RemoteEvents();
    virtual ~RemoteEvents();
    
    Socket *sck;
    
    //Methods and variables for receiving data------------------
    int sizeLeft;
    std::vector<char> vecbuf;
    BufferType bufType;
    
    bool readSize(char* &data, int &bytes);
    bool readType(char* &data, int &bytes);
    bool readData(char* &data, int &bytes);
    //-------------------------------------------------------
	
    RemoteEvents(RemoteEvents&);
    RemoteEvents &operator=(const RemoteEvents&);
};

/*! This struct holds the information required for a request to a
 *  server robot for events for state updates.  */
struct RemoteRequest {
	RemoteEvents::RequestType type;

	//Event subscription
	int numElements;
	EventBase::EventGeneratorID_t egid;
	size_t sid;
	EventBase::EventTypeID_t etid;

	//State updates
	RemoteState::StateType sType;
	unsigned int interval;
};

std::ostream& operator<<(std::ostream &os, const RemoteRequest &req);

#endif
