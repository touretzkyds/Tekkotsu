//-*-c++-*-
#ifndef INCLUDED_MessageReceiver_h_
#define INCLUDED_MessageReceiver_h_

#ifdef PLATFORM_APERIOS
#  warning MessageReceiver is not Aperios compatable
#else

#include "MessageQueue.h"
#include "Thread.h"

//! Spawns a thread for monitoring a MessageQueue, calls a specified function when new messages are available
/*! Uses a semaphore which is raised by the MessageQueue itself when a new message is posted.
 *  This should have almost no overhead, and fairly low latency (at least, much lower latency
 *  than you would get by running multiple busy loops polling for new messages)
 *
 *  Keep in mind that the monitor runs in a separate thread, so you will need to consider mutex issues
 *  when the callback is executing. */
class MessageReceiver : public Thread {
public:
	//! constructor, indicate the message queue, and optional callback function and whether to start the monitor right away
	/*! @param mq is the message queue that the receiver will register with
	 *  @param callback is the function to call when messages are received
	 *  @param startThread controls whether the thread will be launched by the constructor
	 *  @param subscribe only applies if @a startThread is false, indicates whether the receiver should register as a listener even though the thread isn't checking (yet)
	 *  This last parameter allows you to avoid missing messages that come in before you're ready to process them */
	explicit MessageReceiver(MessageQueueBase& mq, bool (*callback) (RCRegion*)=NULL, bool startThread=true, bool subscribe=true);
	//! destructor, stops and joins thread
	virtual ~MessageReceiver();
	
	//! returns the next unread message without marking it read, or NULL if there are currently no more messages.  MessageReceiver retains reference.
	virtual RCRegion * peekNextMessage();
	//! returns the next unread message, marking it as read.  Caller inherits reference, and should call removeReference when done.
	virtual RCRegion * getNextMessage();
	//! marks the current message as read, and allows MessageQueue to process next unread message
	void markRead() { markRead(true); }
	
	//! thread control -- stop monitoring (can call start() later to resume)
	virtual Thread& stop();
	//! thread control -- stop(), join(), and process any final messages in the queue; unsubscribes as a listener of the MessageQueue
	virtual void finish();
	
	//! allows you to change the callback function -- should be set before the thread is started (otherwise, why bother starting it?)
	virtual void setCallback(bool (*callback) (RCRegion*)) { process=callback; }
	
protected:
	typedef MessageQueueBase::index_t index_t; //!< shorthand for the message id type

	virtual void findCurrentMessage(); //!< sets #curit to the oldest message which hasn't been marked read
	virtual bool launched(); //!< register as a listener with the queue, if we haven't already (retains listener status between stop/start)
	virtual unsigned int runloop(); //!< wait for a new message, and then process it
	virtual bool waitNextMessage(); //!< wait for #semid to be raised to indicate a new message is in the queue (or at least, that it needs to be checked); returns false if interrupted
	virtual bool processNextMessage(); //!< gets the next message and processes it
	virtual void markRead(bool checkNext); //!< if @a checksNext is set, raises #semid so that if additional messages came in while we were processing the current one, they will be picked up
	
	MessageQueueBase& queue; //!< the MessageQueue being monitored
	SemaphoreManager::semid_t semid; //!< the semaphore raised when the queue should be checked for new messages
	unsigned int nextMessage; //!< the expected serial number of the next message to be sent
	unsigned int lastProcessedMessage; //!< the serial number of the last received message
	bool (*process) (RCRegion*); //!< the client callback function
	index_t curit; //!< the message id of the last received message (currently being processed)
	
private:
	MessageReceiver(const MessageReceiver& r); //!< don't call
	MessageReceiver& operator=(const MessageReceiver& r); //!< don't call
};

/*! @file
 * @brief 
 * @author ejt (Creator)
 */

#endif //Aperios check

#endif //INCLUDED

