#if 0
#include "BufferSource.h"
#include "Shared/MarkScope.h"
#include <pthread.h>

//better to put this here instead of the header
using namespace std; 

unsigned int BufferSource::getData(const char *& payload, unsigned int& payloadSize, unsigned int& t, std::string& n) {
	if(thread==NULL)
		return frame;
	MarkScope l(swapLock);
	while(!isReady) {
		readySignal.wait(swapLock);
		swapLock.lock();
		if(thread==NULL) { // in case we shut down while waiting
			payload=NULL;
			payloadSize=0;
			return frame;
		}
	}
	payload=&readyBuffer.front();
	payloadSize=readyBuffer.size();
	t=readyTimestamp;
	n=readyName;
	isReady=false;
	return frame;
}

void BufferSource::signalReady() {
	MarkScope l(swapLock);
	swap(timestamp,readyTimestamp);
	swap(name,readyName);
	buffer.swap(readyBuffer); // could use std::swap() and rely on STL providing specialization, but let's be explicit
	++frame;
	readySignal.broadcast();
}


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
#endif
