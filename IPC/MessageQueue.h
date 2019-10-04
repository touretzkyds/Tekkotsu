//-*-c++-*-
#ifndef INCLUDED_MessageQueue_h_
#define INCLUDED_MessageQueue_h_

#ifdef PLATFORM_APERIOS
#  warning MessageQueue is not Aperios compatable
#else

#include "ListMemBuf.h"
#include "RCRegion.h"
#include "SemaphoreManager.h"
#include "MutexLock.h"
#include "Shared/MarkScope.h"
#include "Shared/attributes.h"
#include <exception>
#include <stdlib.h>
#include <unistd.h> // for usleep

#include "Shared/TimeET.h"

//! Defines the interface for sending new shared memory regions between processes
/*! This base class holds all of the template-independent code to allow general
 *  operations on MessageQueues.  The templated version of MessageQueue provides
 *  concrete implementation, which is what you would instantiate.
 *  
 *  Each message entails its own shared memory region, as compared to
 *  SharedQueue, where a single large buffer is maintained, and all messages are
 *  copied into the common buffer.  This class is better for large regions since
 *  it can avoid copying data around.
 * 
 *  @see MessageQueue, MessageQueueStatusListener, MessageReceiver */
class MessageQueueBase {
public:

	//! an interface for filtering (or otherwise monitoring) messages being sent through a MessageQueue, see MessageQueueBase::addMessageFilter()
	class MessageFilter {
	public:
		//! called immediately prior to sending a message -- return true to pass the message into the queue, false to drop it
		virtual bool filterSendRequest(RCRegion* rcr)=0;
		//! to make compiler warning happy
		virtual ~MessageFilter() {}
	};
	
	//!constructor
	MessageQueueBase()
		: lock(), overflowPolicy(THROW_BAD_ALLOC), isClosed(false), reportDroppings(false), numMessages(0),
			numReceivers(0), messagesRead(0)
	{
		for(unsigned int i=0; i<ProcessID::NumProcesses; ++i)
			filters[i]=NULL;
	}
	//!destructor
	virtual ~MessageQueueBase() {}
	
	
	//! The storage type for message entry indicies
	/*! This index is to be used with accessor functions, but may be recycled for
	 *  a new message after all receivers have read the previous message.  If you
	 *  wish to have a unique message identifier, see getMessageSN() */
	typedef unsigned short index_t;
	
	
	//! add one to the receiver reference count
	virtual SemaphoreManager::semid_t addReceiver() ATTR_must_check =0;
	//! remove one from the receiver reference count
	virtual void removeReceiver(SemaphoreManager::semid_t rcvr)=0;
	//! return the receiver reference count
	virtual unsigned int getNumReceivers() const { return numReceivers; }

	//! registers a semaphore which should be raised whenever a message is marked read
	/*! The number of these are limited to the MAX_SENDERS template parameter of
		*  MessageQueue... returns false if too many are already registered
		*  
		*  You probably don't want to call this directly, use a MessageQueueStatusThread */
	virtual SemaphoreManager::semid_t addReadStatusListener() ATTR_must_check =0;
	//! removes a semaphore from the status listener list
	virtual void removeReadStatusListener(SemaphoreManager::semid_t sem)=0;
	
	
	//! post a message into the queue -- a shared reference is added, the caller retains control current reference
	/*! Thus, if you are sending a region and do not intend to use it again, either pass
	 *  true for autoDereference or call RCRegion::removeReference() after sending
	 *  to free the sender's memory.
	 *  
	 *  If no one dereferences the region, you can continue to access the region,
	 *  even as the receiver accesses it as well.  Thus if both sides retain references,
	 *  you can use the region as a shared memory area for future communication.
	 *  (beware of race conditions!)
	 *
	 *  If @a rcr is NULL, an empty message will be sent (there's still some overhead
	 *  to this -- may want to consider a semaphore instead of a MessageQueue if all
	 *  you're going to do is 'ping' another process with empty messages) */
	virtual void sendMessage(RCRegion * rcr, bool autoDereference=false)=0;
	//! request access to a particular message, increments read counter -- do not call more than once per receiver!
	/*! The message is marked read and will be popped from the queue if all
	 *  receivers have read the message as well.  The caller inherits a reference
	 *  to the returned region -- call removeReference when you are done with
	 *  it */
	virtual RCRegion * readMessage(index_t msg, SemaphoreManager::semid_t rcvr)=0;
	//! request access to a particular message, does not mark message -- call as often as you like
	/*! The caller inherits a reference to the returned region -- call
	 *  removeReference when you are done with it */
	virtual RCRegion * peekMessage(index_t msg)=0;
	//! increments read counter -- do not call more than once per receiver per message!
	virtual void markRead(index_t msg, SemaphoreManager::semid_t rcvr)=0;
	//! do not allow any new messages to be posted
	virtual void close() { AutoLock autolock(lock); isClosed=true; }

	//! sets #reportDroppings
	virtual void setReportDroppings(bool report) { reportDroppings=report; }
	//! gets #reportDroppings
	virtual bool getReportDroppings() const { return reportDroppings; }
	
	
	//! Each message gets a unique, monotonically increasing serial number; this function returns that number (MessageQueue::serialNumber)
	virtual unsigned int getMessageSN(index_t msg)=0;
	
	//! Checks to see how many messages have been processed (read by all receivers and removed from queue)
	virtual unsigned int getMessagesRead() { return messagesRead; }
	
	//! Returns the number of messages which have been sent
	virtual unsigned int getMessagesSent() { return numMessages; }
	
	//! Returns the number of messages which have been sent but not yet read
	virtual unsigned int getMessagesUnread() { return getMessagesSent() - getMessagesRead(); }
	
	//! a typedef to make it easier to obtain a lock on the queue for the extent of a scope
	typedef MarkScope AutoLock;
	//! returns a reference to the queue's inter-process lock
	MutexLock<ProcessID::NumProcesses>& getLock() const { return lock; }

	
	virtual index_t oldest() const=0;          //!< return oldest message still in the queue (may or may not have been read by this process)
	virtual index_t newer(index_t it) const=0; //!< return the next message in the queue (may or may not have been read by this process)
	virtual index_t older(index_t it) const=0; //!< return the previous message in the queue (may or may not have been read by this process)
	virtual index_t newest() const=0;          //!< return most recent message added to the queue (may or may not have been read by this process)
	virtual bool isEnd(index_t it) const=0;    //!< returns true if @a it is the one-past-the-end of the queue
	
	//! an enumerations of policies for dealing with overflow, pass to setOverflowPolicy()
	enum OverflowPolicy_t {
		DROP_OLDEST,     //!< the oldest unread message is dropped
		DROP_NEWEST,     //!< the most recently added message is dropped (i.e. the overflowing message is ignored)
		WAIT,            //!< the adding process/thread polls until space is available
		THROW_BAD_ALLOC  //!< throw a std::bad_alloc exception (falls through to abort() if you don't catch it)
	};
	//! allows you to pick how to handle running out of space in the queue, see OverflowPolicy_t
	void setOverflowPolicy(OverflowPolicy_t op) { overflowPolicy=op; }
	//! returns the current overflow policy, see OverflowPolicy_t
	OverflowPolicy_t getOverflowPolicy() const { return overflowPolicy; }
	
	//! sets #semgr
	static void setSemaphoreManager(SemaphoreManager* mgr) { semgr=mgr; }
	//! gets #semgr
	static SemaphoreManager* getSemaphoreManager() { return semgr; }
	
	//! once called, any messages put into the queue must pass through @a filter first (note: there can only be one filter per process!)
	/*! if a filter was previously registered, it is replaced with the new @a filter */
	void addMessageFilter(MessageFilter& filter) {
		filters[ProcessID::getID()]=&filter;
	}
	//! removes the current filter in place, if any
	void removeMessageFilter() {
		filters[ProcessID::getID()]=NULL;
	}
protected:
	//! the global semaphore manager, needs to be set (once, globally) via setSemaphoreManager() before any receivers are added
	static SemaphoreManager* semgr;
	
	mutable MutexLock<ProcessID::NumProcesses> lock; //!< a lock to grant serial access to the queue
	volatile OverflowPolicy_t overflowPolicy; //!< the choice of how to handle message overflow -- see OverflowPolicy_t
	bool isClosed; //!< if true, new messages will be rejected
	bool reportDroppings; //!< if true, output will be sent on cerr when overflow occurs
	unsigned int numMessages; //!< number of messages which have been sent (serial number of next message)
	unsigned int numReceivers; //!< how many receivers to expect
	unsigned int messagesRead; //!< number of messages which have been read and removed from queue
	MessageFilter* filters[ProcessID::NumProcesses]; //!< provides storage of one message filter per process
private:
	MessageQueueBase(const MessageQueueBase&); //!< this shouldn't be called...
	MessageQueueBase& operator=(const MessageQueueBase&); //!< this shouldn't be called...
};

//! An implementation of MessageQueueBase, which provides mechanisms for sending shared memory regions between processes
/*! MAX_UNREAD is assigned to #CAPACITY, MAX_RECEIVERS is assigned to #RECEIVER_CAPACITY, and MAX_SENDERS is assigned to #SENDER_CAPACITY
 *  @see MessageQueueBase, MessageQueueStatusListener, MessageReceiver */
template<unsigned int MAX_UNREAD, unsigned int MAX_RECEIVERS=10, unsigned int MAX_SENDERS=10>
class MessageQueue : public MessageQueueBase {
public:
	//! total number of messages which can be backed up in the queue
	static const unsigned int CAPACITY=MAX_UNREAD;
	//! total number of receivers which can be registered
	static const unsigned int RECEIVER_CAPACITY=MAX_RECEIVERS;
	//! total number of senders which can be registered
	/*! More specifically, this is the maximum number of StatusListeners -- anyone
	 *  can call sendMessage(), but only this number can get direct notification when
	 *  messages are received. */
	static const unsigned int SENDER_CAPACITY=MAX_SENDERS;
	
	//! constructor
	MessageQueue() : MessageQueueBase(), mq(), rcvrs(), sndrs() {}
	
	//! destructor
	virtual ~MessageQueue();
	
	virtual SemaphoreManager::semid_t addReadStatusListener() ATTR_must_check;
	virtual void removeReadStatusListener(SemaphoreManager::semid_t sem);

	virtual SemaphoreManager::semid_t addReceiver() ATTR_must_check;
	virtual void removeReceiver(SemaphoreManager::semid_t rcvr);
	
	virtual void sendMessage(RCRegion * rcr, bool autoDereference=false);
	virtual RCRegion * readMessage(index_t msg, SemaphoreManager::semid_t rcvr);
	virtual RCRegion * peekMessage(index_t msg);
	virtual void markRead(index_t msg, SemaphoreManager::semid_t rcvr);

	virtual unsigned int getMessageSN(index_t msg) { /*AutoLock autolock(lock);*/ return mq[msg].sn; }
	
	virtual index_t oldest() const { AutoLock autolock(lock); return mq.begin(); }
	virtual index_t newer(index_t it) const { AutoLock autolock(lock); return mq.next(it); }
	virtual index_t older(index_t it) const { AutoLock autolock(lock); return mq.prev(it); }
	virtual index_t newest() const { AutoLock autolock(lock); return mq.prev(mq.end()); }
	virtual bool isEnd(index_t it) const { AutoLock autolock(lock); return it==mq.end() || it>=mq_t::MAX_ENTRIES; }
	
protected:
	//! data storage needed for each message
	struct entry {
		entry() : id(), sn(), numRead(0) { memset(readFlags,0,sizeof(readFlags)); } //!< constructor
		entry(unsigned int serialNumber, RCRegion* r)
		: id(r->ID()), sn(serialNumber), numRead(0) { memset(readFlags,0,sizeof(readFlags)); } //!< constructor, pass message info
		RCRegion::Identifier id; //!< the identifier for the shared memory region so that other regions can attach it
		unsigned int sn; //!< serial number for this message (not the same as its index in the queue -- indicies are reused, this id is unique to this message
		bool readFlags[MAX_RECEIVERS]; //!< a flag for each receiver to indicate if they have read it
		unsigned int numRead; //!< a count of the number of receivers which have read this message (should always equal sum(readFlags))
	};
	
	//! shorthand for the type of data storage of message entries
	typedef ListMemBuf<entry,MAX_UNREAD,index_t> mq_t;
	//! the data storage of message entries
	mq_t mq;

	//! shorthand for the type of data storage of message entries
	typedef ListMemBuf<SemaphoreManager::semid_t,MAX_RECEIVERS,index_t> rcvrs_t;
	//! the data storage of receiver semaphores
	rcvrs_t rcvrs;

	//! returns the index within #rcvrs of the receiver id @a rcvr
	typename rcvrs_t::index_t lookupReceiver(SemaphoreManager::semid_t rcvr) const;
	
	//! shorthand for the type of data storage of message entries
	typedef ListMemBuf<SemaphoreManager::semid_t,MAX_SENDERS,index_t> sndrs_t;
	//! the data storage of receiver semaphores
	sndrs_t sndrs;
};

template<unsigned int MAX_UNREAD, unsigned int MAX_RECEIVERS, unsigned int MAX_SENDERS>
MessageQueue<MAX_UNREAD,MAX_RECEIVERS,MAX_SENDERS>::~MessageQueue() {
		//lock shouldn't be necessary -- refcount should ensure the containing
		//region isn't deleted until only one process has access anyway
		//AutoLock autolock(lock);
		while(!mq.empty()) {
			RCRegion * rcr = RCRegion::attach(mq.front().id);
			rcr->RemoveSharedReference();
			rcr->RemoveReference();
			mq.pop_front();
		}
}

template<unsigned int MAX_UNREAD, unsigned int MAX_RECEIVERS, unsigned int MAX_SENDERS>
SemaphoreManager::semid_t MessageQueue<MAX_UNREAD,MAX_RECEIVERS,MAX_SENDERS>::addReadStatusListener() {
		AutoLock autolock(lock);
		SemaphoreManager::semid_t sem=semgr->getSemaphore();
		if(sem==semgr->invalid()) {
			std::cerr << "ERROR: unable to add read status listener to message queue because semaphore manager is out of semaphores" << std::endl;
			return semgr->invalid();
		}
		if(sndrs.push_back(sem)==sndrs.end()) {
			std::cerr << "ERROR: unable to add read status listener to message queue because message queue can't register any more senders (MAX_SENDERS)" << std::endl;
			semgr->releaseSemaphore(sem);
			return semgr->invalid();
		}
		return sem;
}

template<unsigned int MAX_UNREAD, unsigned int MAX_RECEIVERS, unsigned int MAX_SENDERS>
void MessageQueue<MAX_UNREAD,MAX_RECEIVERS,MAX_SENDERS>::removeReadStatusListener(SemaphoreManager::semid_t sem) {
		AutoLock autolock(lock);
		for(index_t it=sndrs.begin(); it!=sndrs.end(); it=sndrs.next(it))
			if(sndrs[it]==sem) {
				sndrs.erase(it);
				semgr->releaseSemaphore(sem);
				break;
			}
}

template<unsigned int MAX_UNREAD, unsigned int MAX_RECEIVERS, unsigned int MAX_SENDERS>
SemaphoreManager::semid_t MessageQueue<MAX_UNREAD,MAX_RECEIVERS,MAX_SENDERS>::addReceiver() {
		AutoLock autolock(lock);
		SemaphoreManager::semid_t sem=semgr->getSemaphore();
		if(sem==semgr->invalid()) {
			std::cerr << "ERROR: unable to add receiver to message queue because semaphore manager is out of semaphores" << std::endl;
			return semgr->invalid();
		}
		if(rcvrs.push_back(sem)==rcvrs.end()) {
			std::cerr << "ERROR: unable to add receiver to message queue because message queue can't register any more receivers (MAX_RECEIVERS)" << std::endl;
			semgr->releaseSemaphore(sem);
			return semgr->invalid();
		}
		numReceivers++;
		return sem;
}

template<unsigned int MAX_UNREAD, unsigned int MAX_RECEIVERS, unsigned int MAX_SENDERS>
void MessageQueue<MAX_UNREAD,MAX_RECEIVERS,MAX_SENDERS>::removeReceiver(SemaphoreManager::semid_t rcvr) {
		AutoLock autolock(lock);
		index_t rcvr_id=rcvrs.begin();
		for(; rcvr_id!=rcvrs.end(); rcvr_id=rcvrs.next(rcvr_id))
			if(rcvrs[rcvr_id]==rcvr)
				break;
		if(rcvr_id==rcvrs.end()) {
			std::cerr << "WARNING: tried to remove message queue receiver " << rcvr << ", which is not registered as a receiver for this queue" << std::endl;
			return;
		}
		rcvrs.erase(rcvr_id);
		semgr->releaseSemaphore(rcvr);
		numReceivers--;
		for(index_t it=mq.begin(); it!=mq.end(); it=mq.next(it)) {
			if(mq[it].readFlags[rcvr_id]) {
				// the removed receiver had read this message, decrement the read count
				mq[it].readFlags[rcvr_id]=false;
				mq[it].numRead--;
			} else if(mq[it].numRead==numReceivers) {
				//all *remaining* processes have gotten a look, remove the neutral MessageQueue reference
				RCRegion * rcr = RCRegion::attach(mq[it].id);
				rcr->RemoveSharedReference();
				rcr->RemoveReference();
				it=mq.prev(it);
				mq.erase(mq.next(it));
				messagesRead++;
				for(index_t sit=sndrs.begin(); sit!=sndrs.end(); sit=sndrs.next(sit))
					semgr->raise(sndrs[sit],1);
			}
		}
}

template<unsigned int MAX_UNREAD, unsigned int MAX_RECEIVERS, unsigned int MAX_SENDERS>
void MessageQueue<MAX_UNREAD,MAX_RECEIVERS,MAX_SENDERS>::sendMessage(RCRegion * rcr, bool autoDereference/*=false*/) {
		AutoLock autolock(lock);
		if(rcr==NULL) {
			rcr=new RCRegion(0);
			autoDereference=true;
		}
		if(filters[ProcessID::getID()]!=NULL && !filters[ProcessID::getID()]->filterSendRequest(rcr)) {
			if(autoDereference)
				rcr->RemoveReference();
			return;
		}
		if(numReceivers==0) {
			//if(reportDroppings)
			//std::cerr << "Warning: MessageQueue dropping " << rcr->ID().key << " because there are no receivers" << std::endl;
			messagesRead++; // counts as a read message (read by all 0 readers is still read by all readers!)
			for(index_t sit=sndrs.begin(); sit!=sndrs.end(); sit=sndrs.next(sit))
				semgr->raise(sndrs[sit],1);
			if(autoDereference)
				rcr->RemoveReference();
			return;
		}
		if(isClosed) {
			if(reportDroppings)
				std::cerr << "Warning: MessageQueue dropping " << rcr->ID().key << " because queue is closed" << std::endl;
			if(autoDereference)
				rcr->RemoveReference();
			return;
		}
		if(mq.size()==mq.getMaxCapacity()) {
			switch(overflowPolicy) {
				case DROP_OLDEST: {
					if(reportDroppings)
						std::cerr << "WARNING: MessageQueue full, dropping oldest unread message (" << mq.front().id.key << ")" << std::endl;
					RCRegion * eldest = RCRegion::attach(mq.front().id);
					eldest->RemoveSharedReference();
					mq.pop_front();
					eldest->RemoveReference();
				} break;
				case DROP_NEWEST:
					if(reportDroppings)
						std::cerr << "WARNING: MessageQueue full, dropping newest unread message (" << rcr->ID().key << ")" << std::endl;
					if(autoDereference)
						rcr->RemoveReference();
					return;
				case WAIT:
					if(reportDroppings)
						std::cerr << "WARNING: MessageQueue full, waiting for readers to catch up" << std::endl;
					while(mq.size()==mq.getMaxCapacity()) {
						//have to release locks so readers can get access
						unsigned int ll=lock.get_lock_level();
						lock.releaseAll();
						usleep(MutexLockBase::usleep_granularity*15);
						for(unsigned int i=0; i<ll; i++)
							lock.lock(ProcessID::getID());
						if(overflowPolicy!=WAIT) { //may have been changed by a different thread while we were waiting
							sendMessage(rcr,autoDereference); //retry with the new policy
							return;
						}
					}
					break;
				case THROW_BAD_ALLOC:
					if(reportDroppings)
						std::cerr << "WARNING: MessageQueue full, throwing bad_alloc exception" << std::endl;
					throw std::bad_alloc();
					break;
			}
		}
		rcr->AddSharedReference();
		if(mq.push_back(entry(numMessages++,rcr))==mq.end()) {
			//our overflow policy should've prevented this
			std::cerr << "ERROR: MessageQueue unable to add message; buggy overflow policy?" << std::endl;
			exit(EXIT_FAILURE);
		}
		
		//std::cout << Process::getName() << " sent " << (numMessages-1) << " at " << TimeET() << std::endl;
		//notify receivers
		for(index_t it=rcvrs.begin(); it!=rcvrs.end(); it=rcvrs.next(it))
			semgr->raise(rcvrs[it],1);
		
		if(autoDereference)
			rcr->RemoveReference();
}

template<unsigned int MAX_UNREAD, unsigned int MAX_RECEIVERS, unsigned int MAX_SENDERS>
RCRegion * MessageQueue<MAX_UNREAD,MAX_RECEIVERS,MAX_SENDERS>::readMessage(index_t msg, SemaphoreManager::semid_t rcvr) {
		AutoLock autolock(lock);
		RCRegion * rcr = RCRegion::attach(mq[msg].id);
		index_t rcvr_id=lookupReceiver(rcvr);
		if(rcvr_id==rcvrs.end())
			return rcr;
		if(mq[msg].readFlags[rcvr_id]) {
			std::cerr << "WARNING: MessageQueue::readMessage(): Receiver re-reading message, could be recycled/invalidated any time" << std::endl;
			return rcr; // already read, just return it
		}
		mq[msg].readFlags[rcvr_id]=true;
		mq[msg].numRead++;
		if(mq[msg].numRead==numReceivers) {
			//all processes have gotten a look, remove the neutral MessageQueue reference
			rcr->RemoveSharedReference();
			mq.erase(msg);
			messagesRead++;
			for(index_t sit=sndrs.begin(); sit!=sndrs.end(); sit=sndrs.next(sit))
				semgr->raise(sndrs[sit],1);
		}
		return rcr;
}

template<unsigned int MAX_UNREAD, unsigned int MAX_RECEIVERS, unsigned int MAX_SENDERS>
RCRegion * MessageQueue<MAX_UNREAD,MAX_RECEIVERS,MAX_SENDERS>::peekMessage(index_t msg) {
		//AutoLock autolock(lock); //I don't think a lock is necessary here
		return RCRegion::attach(mq[msg].id);
}

template<unsigned int MAX_UNREAD, unsigned int MAX_RECEIVERS, unsigned int MAX_SENDERS>
void MessageQueue<MAX_UNREAD,MAX_RECEIVERS,MAX_SENDERS>::markRead(index_t msg, SemaphoreManager::semid_t rcvr) {
		AutoLock autolock(lock);
		index_t rcvr_id=lookupReceiver(rcvr);
		if(rcvr_id==rcvrs.end())
			return;
		if(mq[msg].readFlags[rcvr_id]) {
			std::cerr << "WARNING: MessageQueue::markRead(): Receiver re-reading message, could be recycled/invalidated any time" << std::endl;
			return; // already read, just return it
		}
		mq[msg].readFlags[rcvr_id]=true;
		mq[msg].numRead++;
		if(mq[msg].numRead==numReceivers) {
			//all processes have gotten a look, remove the neutral MessageQueue reference
			RCRegion * rcr = RCRegion::attach(mq[msg].id);
			rcr->RemoveSharedReference();
			rcr->RemoveReference();
			mq.erase(msg);
			messagesRead++;
			for(index_t sit=sndrs.begin(); sit!=sndrs.end(); sit=sndrs.next(sit))
				semgr->raise(sndrs[sit],1);
		}
}

template<unsigned int MAX_UNREAD, unsigned int MAX_RECEIVERS, unsigned int MAX_SENDERS>
typename MessageQueue<MAX_UNREAD,MAX_RECEIVERS,MAX_SENDERS>::rcvrs_t::index_t
MessageQueue<MAX_UNREAD,MAX_RECEIVERS,MAX_SENDERS>::lookupReceiver(SemaphoreManager::semid_t rcvr) const {
	for(index_t rcvr_id=rcvrs.begin(); rcvr_id!=rcvrs.end(); rcvr_id=rcvrs.next(rcvr_id))
		if(rcvrs[rcvr_id]==rcvr)
			return rcvr_id;
	std::cerr << "WARNING: tried to look up queue receiver " << rcvr << ", which is not registered as a receiver for this queue" << std::endl;
	return rcvrs.end();
}

/*! @file
 * @brief Defines MessageQueue, which provides mechanisms for sending shared memory regions between processes
 * @author ejt (Creator)
 */

#endif //APERIOS check

#endif //INCLUDED
