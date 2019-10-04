#ifndef SOCKETLISTENER_H_
#define SOCKETLISTENER_H_

//! interface for notifications from Wireless
class SocketListener {
public:
	//! destructor (just to satisfy compiler warning that we do indeed intend to use this as a base class)
	virtual ~SocketListener() {}
	
	//! called by wireless whenever new data is available on the Socket this was registered with
	/*! @see Wireless::setReceiver() */
	virtual int processData(char *data, int bytes) = 0;
};

#endif
