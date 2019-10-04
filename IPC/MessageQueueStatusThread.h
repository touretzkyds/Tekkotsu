//-*-c++-*-
#ifndef INCLUDED_MessageQueueStatusThread_h_
#define INCLUDED_MessageQueueStatusThread_h_

#ifdef PLATFORM_APERIOS
#  warning MessageQueueStatusThread is not Aperios compatable, this is not going to compile
#else

#include "IPC/SemaphoreManager.h"
#include "IPC/Thread.h"
#include <list>

class MessageQueueBase;

//! description of MessageQueueStatusThread
class MessageQueueStatusThread : protected Thread {
public:
	//!An interface to allow you to receive callbacks when a message has been read from a MessageQueue, subscribed via an external class which is monitoring the queue's MessageQueueBase::pollStatus() (e.g. LoadFileThread)
	class StatusListener {
	public:
		//! destructor -- does nothing
		virtual ~StatusListener() {}
		
		//! Called after a message has been read by all receivers, and thus has been removed from the queue
		/*! Don't assume that because you receive this callback there is space in
		*  the queue -- an earlier listener may have already added a message, or
		*  the queue might have been already waiting to send a message if
		*  the queue's overflowPolicy is MessageQueueBase::WAIT
		*
		*  @param which The MessageQueueBase which has had message(s) read
		*  @param howmany The number of message which have been cleared */
		virtual void messagesRead(MessageQueueBase& which, unsigned int howmany)=0;
	};
	
	
	//! constructor
	MessageQueueStatusThread()
		: Thread(), statusListeners(), queue(NULL), semid(), numRead()
	{}
	//! constructor, automatically starts the thread with the specified queue, and an optional initial listener
	explicit MessageQueueStatusThread(MessageQueueBase& mq, StatusListener* listener=NULL)
		: Thread(), statusListeners(), queue(NULL), semid(), numRead()
	{
		setMessageQueue(mq);
		addStatusListener(listener);
	}
	//! destructor, remove ourself from queue
	~MessageQueueStatusThread();

	//! Request updates to StatusListener callbacks
	virtual void addStatusListener(StatusListener* l);
	//! Unsubscribes a StatusListener from future updates
	virtual void removeStatusListener(StatusListener* l);
	
	//! (re)sets the message queue being listened to
	virtual void setMessageQueue(MessageQueueBase& mq);
	//! returns the current queue
	virtual MessageQueueBase* getMessageQueue();
	
protected:
	//! start the thread
	virtual bool launched();
	//! wait for the queue's message read semaphore to be raised, and notify listeners
	virtual void* run();
	//! indicates it's time to stop monitoring the queue (need to raise the semaphore so run() will notice the stop)
	virtual Thread& stop();
	//! cleanup
	virtual void cancelled();
	
	//! Notifies statusListeners that a message has been read by all MessageQueue receivers
	virtual void fireMessagesRead(unsigned int howmany);

	//! MessageQueueBase::StatusListeners currently subscribed from addStatusListener()
	std::list<StatusListener*> statusListeners;
	
	//! The MessageQueue that this thread is monitoring
	MessageQueueBase* queue;
	
	//! the semaphore which is being monitored, raised by #queue when a message is read
	SemaphoreManager::semid_t semid;
	
	//! the number of messages read sent last time the semaphore (#semid) was raised
	unsigned int numRead;
	
private:
	MessageQueueStatusThread(const MessageQueueStatusThread&); //!< don't call, shouldn't copy
	MessageQueueStatusThread& operator=(const MessageQueueStatusThread&); //!< don't call, shouldn't assign
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
#endif
