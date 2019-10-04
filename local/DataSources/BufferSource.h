//-*-c++-*-
#ifndef INCLUDED_BufferSource_h_
#define INCLUDED_BufferSource_h_

#include "local/DataSource.h"
#include "IPC/Thread.h"
#include <vector>
#include <string>

//! description of BufferSource
class BufferSource : public virtual DataSource {
public:
	//! constructor
	BufferSource()
	: DataSource(), timestamp(-1U), name("BufferSource"), buffer(), reqTime(-1U),
	swapLock(), readySignal(), readyTimestamp(-1U), readyName(), readyBuffer(), isReady(false),
	frame(0)
	{}
	
	unsigned int timestamp; //!< value to return as timestamp of #buffer (or nextTimestamp())
	std::string name; //!< value to return as name of #buffer (or nextName())
	std::vector<char> buffer; //!< pointer to buffer for writing
	unsigned int reqTime; //!< requested timestamp by last call to getData(), or -1U if not called or no LoadDataThread registered
	
	virtual unsigned int nextTimestamp() { return timestamp; }
	virtual const std::string& nextName() { return name; }
	virtual unsigned int getData(const char *& payload, unsigned int& payloadSize, unsigned int& t, std::string& n);
	
	virtual void signalReady(); //!< swaps timestamp, name, and buffer with their "ready" counterparts
	
	virtual void setDataSourceThread(LoadDataThread* th) {
		if(th==NULL)
			reqTime=-1U;
		DataSource::setDataSourceThread(th);
	}
	
protected:
	Thread::Lock swapLock; //!< lock while swapping during signalReady()
	Thread::Condition readySignal; //!< used to wake up sleeping thread in getData()
	unsigned int readyTimestamp; //!< the timestamp to report for #readyBuffer in getData()
	std::string readyName; //!< the name to report for #readyBuffer in getData()
	std::vector<char> readyBuffer; //!< the payload data to return in getData()
	bool isReady; //!< set to true when signalReady() is called, cleared by getData()
	unsigned int frame; //!< incremented for each call to signalReady();
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
