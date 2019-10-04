#include "DataStreamDriver.h"

using namespace std; 

unsigned int DataStreamDriver::nextTimestamp() {
	CommPort * comm = getComm(commName);
	if(comm==NULL || !comm->isReadable() || srcFrameRate<=0)
		return -1U;
	return static_cast<unsigned int>(timestamp + 1000/srcFrameRate + 0.5f);
}

bool DataStreamDriver::advance() {
	ASSERTRETVAL(!isStarted(), "DataStreamDriver::advance() called in realtime mode",false);
	
	CommPort * comm = getComm(commName);
	if(comm==NULL || !comm->isReadable() || srcFrameRate<=0)
		return false;
	
	MarkScope autolock(*comm);
	std::istream is(&comm->getReadStreambuf());
	is.exceptions(ios_base::badbit);
	// make sure we have data... might cancel out while we're waiting for an image to come
	char firstByte;
	std::streamsize numread = is.readsome(&firstByte,1);
	if(!is)
		return false;
	if(numread==0) { // no data
		if(requestFrame(*comm) || getTimeScale()<=0) { // maybe we need to ask for it (subclass hook)
			// returned true, wait for image
			is.get(firstByte);
			if(!is)
				return false;
		} else {
			// just block on the next image
			is.read(&firstByte,1);
			testCancel();
			if(!is) { // still no data, return to allow heartbeat on previous image
				timestamp = get_time();
				return false;
			}
		}
	}
	
	is.putback(firstByte);
	return readData(is);
}

void DataStreamDriver::registerSource() {
	registered=true;
	connect(getComm(commName));
}

void DataStreamDriver::deregisterSource() {
	registered=false;
	disconnect(getComm(commName));
}

void DataStreamDriver::doUnfreeze() {
	realtime=true;
	srcFrameRate.addPrimitiveListener(this);
	ASSERTRET(!isStarted(),"DataStreamDriver was already running on call to enteringRealtime()");
	if(srcFrameRate>0)
		start();
}

void DataStreamDriver::doFreeze() {
	realtime=false;
	if(isStarted())
		stop().join();
	srcFrameRate.removePrimitiveListener(this);
}

void DataStreamDriver::plistValueChanged(const plist::PrimitiveBase& pl) {
	if(&pl==&commName) {
		// if here, then setDataSourceThread has been called, thus when commName changes,
		// need to close old one and reopen new one
		disconnect(getComm(commName.getPreviousValue()));
		connect(getComm(commName));
	} else if(&pl==&srcFrameRate) {
		if(srcFrameRate>0 && realtime && !isStarted())
			start();
		else if(srcFrameRate<=0 && isStarted())
			stop();
	} else {
		std::cerr << "Unhandled value change in " << getClassName() << ": " << pl.get() << std::endl;
	}
}

void* DataStreamDriver::run() {
	while(true) {
		CommPort * comm = getComm(commName);
		if(comm==NULL || srcFrameRate<=0)
			return NULL;
		
		testCancel();
		if(!comm->isReadable()) {
			usleep(500*1000);
			continue;
		}
		MarkScope autolock(*comm);
		std::istream is(&comm->getReadStreambuf());
		is.exceptions(ios_base::badbit);
		readData(is);
		if(is && paceInput) {
			unsigned int next = nextTimestamp();
			unsigned int cur = get_time();
			if(next>cur)
				usleep(static_cast<useconds_t>((next-cur)*1000/getTimeScale()+.5));
		}
	}
}

void DataStreamDriver::connect(CommPort* comm) {
	if(comm!=NULL) {
		MarkScope autolock(*comm);
		comm->open();
		if(getTimeScale>0 && realtime)
			start();
	}
	commName.addPrimitiveListener(this);
	srcFrameRate.addPrimitiveListener(this);
}

void DataStreamDriver::disconnect(CommPort* comm) {
	if(comm!=NULL) {
		if(isStarted())
			stop().join();
		MarkScope autolock(*comm);
		comm->close();
	}
	commName.removePrimitiveListener(this);
	srcFrameRate.removePrimitiveListener(this);
}


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
