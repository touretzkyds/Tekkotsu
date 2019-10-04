#include "Events/RemoteEvents.h"
#include "Events/EventRouter.h"
#include "Shared/string_util.h"

RemoteEvents::RemoteEvents() : sck(NULL),
                               sizeLeft(0), vecbuf(), bufType(Invalid) {

}

RemoteEvents::~RemoteEvents() {

}

std::string RemoteEvents::remoteIPString() {
	return string_util::intToStringIP(remoteIPInt());
}

int RemoteEvents::remoteIPInt() {
	return sck->getPeerAddress();
}

//Receiving data-------------------------------------------

/* Reads in the buffer type header */
bool RemoteEvents::readType(char* &data, int &bytes) {
	//printf("Got pointer: %x, %d bytes\n", data, bytes);
    if ((unsigned)bytes < sizeof(BufferType))
        return false;

    bufType = *(BufferType *)data;
    data += sizeof(BufferType);
    bytes -= sizeof(BufferType);
    return true;
}

/* Reads in a size header from the data pointer. */
bool  RemoteEvents::readSize(char* &data, int &bytes) {
    //Return an error if there's not enough data there
    if ((unsigned)bytes < sizeof(int))
        return false;

    //Reset the buffer
    vecbuf.clear();

    //Read the size and increment/decrement things as appropriate
    sizeLeft = *(int *)data;
    data += sizeof(int);
    bytes -= sizeof(int);
    return true;
}

/* Reads in data from the given pointer until the target size is
 * reached, or bytes becomes zero. Return true if the whole desired
 * chunk was read, false otherwise. */
bool RemoteEvents::readData(char* &data, int &bytes) {
    while (bytes) {
        //If sizeLeft is zero it's done reading the data
        if (!sizeLeft)
            return true;
        
        //Read a byte
        vecbuf.push_back(*data++);
        bytes--;
        sizeLeft--;
    }
    //Return whether or not the whole chunk was read
    return !sizeLeft;
}
//------------------------------------------------------

bool RemoteEvents::isConnected() {
	return sck==NULL ? false : wireless->isConnected(sck->sock);
}

std::ostream& operator<<(std::ostream &os, const RemoteRequest &req) {
	os << "RemoteRequest[type=" << req.type
		 << ",elts=" << req.numElements
		 << ",egid=" << req.egid
		 << ",sid=" << req.sid
		 << ",etid=" << req.etid
		 << ",sType=" << req.sType
		 << ",interval=" << req.interval
		 << "]";
	return os;
}
