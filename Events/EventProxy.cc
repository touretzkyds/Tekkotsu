#include "Events/EventProxy.h"
#include "Shared/WorldState.h"
#include "Events/RemoteRouter.h"
#include "Events/EventRouter.h"

#include "Shared/debuget.h"

using namespace std;

EventProxy::EventProxy(int port) : RemoteEvents(), listening(true) {
	sck = wireless->socket(Socket::SOCK_STREAM, 4096, 4096);
	wireless->setReceiver(sck, this);
	wireless->setDaemon(sck, true);
	wireless->listen(sck, port);
	cout << "Adding timer" << endl;
}

EventProxy::~EventProxy() {
	if (isConnected())
		wireless->close(sck);
}

bool EventProxy::isActive() {
	return listening || isConnected();
}

int EventProxy::processData(char *data, int bytes) {
	listening = false;
	while (bytes) {
		if (bufType == Invalid) {
			//Get the buffer type
			if (!readType(data, bytes)) {
				cout << "Error reading buffer type" << endl;
			}
		} else if (!sizeLeft) {
			//Get the snize
			if (!readSize(data, bytes)) {
				cout << "Error reading buffer size" << endl;
			}
		} else {
			//Read some data
			if (readData(data, bytes)) {
				//Dispatch the chunk of data
				switch(bufType) {
				case RequestData:
					//Dispatch the data
					handleRemoteRequest((RemoteRequest *)&vecbuf[0]);
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

/* Encodes and sends the received event */
void EventProxy::processEvent(const EventBase &event) {
	
	if (event.getGeneratorID() != EventBase::timerEGID) {
		if (!isConnected()) {
			cout << "Got an event but not connected!" << endl;
			return;
		}

		// Ignore events that didn't originate with us
		if (event.getHostID() != -1)
			return;
	
		//Send Event to connected robot
	
		int esize = 0;
		byte *ebuf = new byte[defaultBufferSize];
		
		if ( (esize = event.saveBinaryBuffer((char *)ebuf, defaultBufferSize)) ) {
			NetworkBuffer nBuf;
			nBuf.addItem(EventData);
			nBuf.addBuffer(ebuf, esize);
			
			if (!nBuf.send(sck)) {
				cout << "Error sending event to remote robot." << endl;
				return;
			}
		} else {
			cout << "Unable to save event to a buffer, aborting transmission." << endl;
		}
		
		delete[] ebuf;
	} else {
		//Send state information
		sendState((RemoteState::StateType)event.getSourceID());
	}
}

void EventProxy::handleRemoteRequest(RemoteRequest *info) {
	// std::cout << "Got " << *info << std::endl;  // **** DEBUG
	switch (info->type) {
	case EventListenerRequest:
 		cout << "Adding remote event listener request: " << info->numElements
			 << " elements for host " << remoteIPString() << endl;
		
		switch (info->numElements) {
		case 1:
			erouter->addListener(this, info->egid);
			break;
			
		case 2:
			erouter->addListener(this, info->egid, info->sid);
			break;
			
		case 3:
			erouter->addListener(this, info->egid, info->sid, info->etid);
			break;
			
		default:
			cout << "Invalid number of elements (" << info->numElements << ") in event listener request." << endl;
			cout << *(float*)(NULL);
			break;
		}
		break;

		
	case RemoveEventListenerRequest:
		cout << "Removing remote event listener: " << info->numElements
			 << " for host " << remoteIPString() << endl;

		switch (info->numElements) {
		case 1:
			erouter->removeListener(this, info->egid);
			break;
			
		case 2:
			erouter->removeListener(this, info->egid, info->sid);
			break;
			
		case 3:
			erouter->removeListener(this, info->egid, info->sid, info->etid);
			break;
			
		default:
			cout << "Invalid number of elements (" << info->numElements << ") in event listener request." << endl;
			break;
		}
		break;

		
	case StateUpdateRequest:
 		cout << "Adding remote state update request for host "
			 << remoteIPString() << endl;

		erouter->addTimer(this, info->sType, info->interval, true);
		break;

	case StopStateUpdateRequest:
		cout << "Removing remote state update request" << endl;
		
		erouter->removeTimer(this, info->sType);
		break;
		
	}
}

/* Encodes and sends the requested state info */
void EventProxy::sendState(RemoteState::StateType stype) {
	if (!isConnected()) {
		cout << "Got a request to send state data but not connected!" << endl;
		return;
	}
	
	float *src = NULL;
	int size = RemoteState::sizes[stype];

	/* Get the source of the data */
	switch (stype) {
	case RemoteState::OutputState:
		src = state->outputs;
		break;
		
	case RemoteState::ButtonState:
		src = state->buttons;
		break;
		
	case RemoteState::SensorState:
		src = state->sensors;
		break;
		
	default:
		cout << "Unrecognized state type, aborting" << endl;
		return;
	}
	
	NetworkBuffer nBuf;
	nBuf.addItem(StateData);
	nBuf.addItem(size + 2*sizeof(int));
	nBuf.addItem(stype);
	nBuf.addBuffer((byte *)src, size);
	
	if (!nBuf.send(sck)) {
		cout << "Error sending state buffer" << endl;
	}
}
