#include "Events/RemoteRouter.h"
#include "Events/EventRouter.h"

#include "Events/TextMsgEvent.h"
#include "Events/TimerEvent.h"
#include "Events/FilterBankEvent.h"
#include "Events/LocomotionEvent.h"
#include "Events/LookoutEvents.h"
#include "Events/PitchEvent.h"
#include "Events/VisionObjectEvent.h"
#include "Events/Kodu/KoduEventBase.h"
#include "Shared/string_util.h"

using namespace std;

RemoteRouter::RemoteRouter(int host) : RemoteEvents(),
									   rstate(NULL), waitingForPort(true),
									   requestQueue(),
									   timerActive(false), waitTime(0),
									   remoteHost(0) {
	rstate = new RemoteState(this);
	
	remoteHost = host;
	
	connect(EventRouter::defaultPort);
	erouter->addTimer(this, 1, 500, true);
}

RemoteRouter::~RemoteRouter() {
	delete rstate;
	
	wireless->close(sck);
}

void RemoteRouter::forwardEvent(std::vector<char> &evec) {
	//Decode the event from the buffer and send it

	unsigned int size = evec.size();
	char *buf = &evec[0];
	EventBase etest, *event = NULL;
	
	if (!etest.checkInc((int)etest.loadBinaryBuffer(buf, size), buf, size)) {
		cout << "Error: Received event is not a subclass of EventBase" << endl;
		return;
	}
	
	//If there are bytes left, it's not just an EventBase
	if (size) {

		if (etest.checkCreator("EventBase::TextMsgEvent",
							   buf, size, false)) {
			event = new TextMsgEvent();
		} else if (etest.checkCreator("EventBase::TimerEvent",
									  buf, size, false)) {
			event = new TimerEvent();
		} else if (etest.checkCreator("EventBase::LocomotionEvent",
									  buf, size, false)) {
			event = new LocomotionEvent();
		} else if (etest.checkCreator("EventBase::VisionObjectEvent",
									  buf, size, false)) {
			event = new VisionObjectEvent();
		} else if (etest.checkCreator(KoduEventBase::classID, buf, size, false)) {
            event = KoduEventBase::getInstance(&evec[0], evec.size());
            if (event == NULL) {
                // Could not read the event
                return;
            }
		} else {
			cout << "Buffer isn't a recognized event type. " << endl;
		}
		
	} else {
		event = new EventBase();
	}

	//Load the buffer
	if (event) {
		if (!event->loadBinaryBuffer(&evec[0], evec.size())) {
			cout << "Error loading from buffer" << endl;
		} else {
 			// cout << "Created event object successfully. Posting event from host "
 			//	 << string_util::intToStringIP(remoteHost) << endl;
			event->setHostID(remoteHost);
			erouter->postEvent(*event);
			delete event;
		}
	}
}

void RemoteRouter::connect(int port) {
	std::string ip = string_util::intToStringIP(remoteHost);
	cout << "RemoteRouter: Connecting to " << ip << " on port "
		 << port << endl;
	sck = wireless->socket(Socket::SOCK_STREAM);
	wireless->setReceiver(sck, this);
	if (wireless->connect(sck, ip.c_str(), port)) {
		cout << "RemoteRouter: error connecting to remote host" << endl;
	}
}

int RemoteRouter::processData(char *data, int bytes) {
	if (waitingForPort) {
		if (bytes != sizeof(int)) {
			cout << "RemoteRouter: unexpected data" << endl;
			return -1;
		}

		wireless->close(sck);
		int port = *(int *)data;
		connect(port);
		waitingForPort = false;
		return 0;
	}
	
	while (bytes) {
		if (bufType == Invalid) {
			//Get the buffer type
			if (!readType(data, bytes))
				cout << "Error reading buffer type" << endl;

		} else if (!sizeLeft) {
			//Get the size
			if (!readSize(data, bytes))
				cout << "Error reading buffer size" << endl;
				
		} else {
			//Read some data
			if (readData(data, bytes)) {
				//Dispatch the chunk of data
				switch(bufType) {
				case EventData:
					forwardEvent(vecbuf);
					break;
				case StateData:
					rstate->update(&vecbuf[0]);
					break;
				case Invalid:
					cout << "Error: invalid data. This should never happen." << endl;
					return -1;
				default:
					cout << "Error: data came in that wasn't expected" << endl;
					return -1;
				}
				bufType = Invalid;
			}
		}
		
	}

	return 0;
}

void RemoteRouter::requestStateUpdates(RemoteState::StateType type,
											 unsigned int interval) {
	RemoteRequest info;
	info.type = StateUpdateRequest;
	info.sType = type;
	info.interval = interval;
	sendRemoteRequest(info);
}

void RemoteRouter::stopStateUpdates(RemoteState::StateType type) {
	RemoteRequest info;
	info.type = StopStateUpdateRequest;
	info.sType = type;
	sendRemoteRequest(info);
}

void RemoteRouter::addListener(EventBase::EventGeneratorID_t egid) {
	RemoteRequest info;
	info.type = EventListenerRequest;
	info.numElements = 1;
	info.egid = egid;
	sendRemoteRequest(info);
}

void RemoteRouter::addListener(EventBase::EventGeneratorID_t egid,
							   size_t sid) {
	RemoteRequest info;
	info.type = EventListenerRequest;
	info.numElements = 2;
	info.egid = egid;
	info.sid = sid;
	sendRemoteRequest(info);
}

void RemoteRouter::addListener(EventBase::EventGeneratorID_t egid,
							   size_t sid,
							   EventBase::EventTypeID_t etid) {
	RemoteRequest info;
	info.type = EventListenerRequest;
	info.numElements = 3;
	info.egid = egid;
	info.sid = sid;
	info.etid = etid;
	sendRemoteRequest(info);
}

void RemoteRouter::removeListener(EventBase::EventGeneratorID_t egid) {
	RemoteRequest info;
	info.type = RemoveEventListenerRequest;
	info.numElements = 1;
	info.egid = egid;
	sendRemoteRequest(info);
}

void RemoteRouter::removeListener(EventBase::EventGeneratorID_t egid,
								  size_t sid) {
	RemoteRequest info;
	info.type = RemoveEventListenerRequest;
	info.numElements = 2;
	info.egid = egid;
	info.sid = sid;
	sendRemoteRequest(info);
}

void RemoteRouter::removeListener(EventBase::EventGeneratorID_t egid,
								  size_t sid,
								  EventBase::EventTypeID_t etid) {
	RemoteRequest info;
	info.type = RemoveEventListenerRequest;
	info.numElements = 3;
	info.egid = egid;
	info.sid = sid;
	info.etid = etid;
	sendRemoteRequest(info);
}

void RemoteRouter::processEvent(const EventBase& event) {
	if (event.getGeneratorID() == EventBase::timerEGID ) {
		switch(event.getSourceID()) {
		case 0:
			if (isReady()) {
				cout << "Connected! Sending queue of requests" << endl;
				while (requestQueue.size()) {
					sendRemoteRequest(requestQueue.front());
					requestQueue.pop_front();
				}
				
				erouter->removeTimer(this, 0);
				timerActive = false;
			} else {
				waitTime += 500;
				if (waitTime == 5000) {
					cout << "RemoteRouter has been waiting for 5 seconds to connect, "
						 << "are you sure you specified the right host?" << endl;
				}
			}
			break;

		case 1:
			if (isConnected()) {
				int foo = 0;
				sck->write((byte *)&foo, sizeof(int));
				erouter->removeTimer(this, 1);
			}
			break;

		default:
			cout << "RemoteRouter got unknown timer event" << endl;
		}
	}
}

void RemoteRouter::sendRemoteRequest(RemoteRequest &info) {
	if (!isReady()) {
		cout << "Tried to send remote request but not connected! Queuing RemoteRequest..." << endl;

		requestQueue.push_back(info);
		if (!timerActive) {
			erouter->addTimer(this, 0, 500, true);
			timerActive = true;
			waitTime = 0;
		}
		return;
	}

	NetworkBuffer nBuf;

	nBuf.addItem(RequestData);
	int const remoteRequestSize = sizeof(RemoteRequest); // sizeof(...) returns a long; we require an int for addItem
	nBuf.addItem(remoteRequestSize);
	nBuf.addItem(info);

	if (!nBuf.send(sck)) {
		cout << "Error sending remote request" << endl;
		return;
	}	
}
