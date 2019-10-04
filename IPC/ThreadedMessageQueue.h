#ifndef INCLUDED_ThreadedMessageQueue_h_
#define INCLUDED_ThreadedMessageQueue_h_

#include "Thread.h"
#include "Shared/MarkScope.h"
#include <list>
#include <stdexcept>
#include <algorithm>

//! Provides a mechanism for exchanging messages between threads
/*! Receivers are responsible for message cleanup/deletion. */
template<class T>
class ThreadedMessageQueue {
public:
	//! Constructor
	ThreadedMessageQueue() : lock(), signal(), msgs(), receiver(NULL) {}
	
	//! Destructor, calls finishCallback()
	~ThreadedMessageQueue() { finishCallback(); }
	
	//! Adds a message to the queue
	void send(const T& msg) { MarkScope l(lock); msgs.push_back(msg); signal.broadcast(); }
	
	//! Applies the specified predicate to each of the messages (you can modify the queue value itself via pass-by-reference)
	template<class F> void for_each(const F& f) { std::for_each(msgs.begin(),msgs.end(),f); }
	
	//! Applies the specified predicate to each of the message, removing those for which the predicate returns true
	template<class F> void remove(const F& f) { MarkScope l(lock); msgs.remove_if(f); }

	//! Removes an element from the queue
	void remove(const T e) { MarkScope l(lock); msgs.remove(e); }
	
	//! Returns number of messages in #msgs
	size_t size() const { return msgs.size(); }
	
	//! Clears any backlog
	void clear() { MarkScope l(lock); msgs.clear(); }
	
	//! Returns the next message (does not remove from the queue), blocking until available
	const T& front() const {
		MarkScope l(lock); 
		while(msgs.size()==0)
			signal.wait(lock);
		return msgs.front();
	}
	
	//! Removes the front message, if any
	void pop() { MarkScope l(lock); if(msgs.size()>0) msgs.pop_front(); }
	
	//! Spawns a thread to trigger a class member callback with each message - only a single receiver is supported, so this replaces any previous receiver
	template<typename F, typename C>
	void spawnCallback(F fn, C& cl) {
		stopCallback();
		receiver = new ReceiverThread<F,C>(*this, fn, cl);
		receiver->start();
	}
	
	//! Sends a thread cancellation to the receiver thread to halt processing
	void stopCallback() {
		if(receiver!=NULL) {
			{
#ifdef USE_SIGNAL_TO_CANCEL_THREAD
				// lock is necessary on stop to ensure cancellation does
				// not arrive between the testCancel and the system wait call
				MarkScope l(lock);
#endif
				receiver->stop();
				receiver->keepRunning=false;
			} // must release lock for wait cancellation to go through
			receiver->join();
			delete receiver;
			receiver=NULL;
		}
	}
	
	//! Sets a flag to exit the receiver thread at the completion of the current callback
	/*! If no callback is active, cancels the receiver immediately -- doesn't wait
	 *  for another message first */
	void finishCallback() {
		if(receiver==NULL)
			return;
		{
			MarkScope l(lock);
			if(msgs.size()==0)
				receiver->stop();
			receiver->keepRunning=false;
		} // must release lock for wait cancellation to go through
		receiver->join();
		delete receiver;
		receiver=NULL;
	}
	
	//! Sets a flag that when the receiver gets to the end of the queue, it will exit
	/*! If the receiver is already blocking at the end of the queue, stops the thread now */
	void finishQueue() {
		if(receiver==NULL)
			return;
		{
			MarkScope l(lock);
			if(msgs.size()==0)
				receiver->stop();
			receiver->block=false;
		} // must release lock for wait cancellation to go through
		receiver->join();
		delete receiver;
		receiver=NULL;
	}
	
protected:
 public: // *** debug
	mutable Thread::Lock lock; //!< provides mutual exclusion on #msgs operations and #signal reception
	Thread::Condition signal; //!< connects notification of send() in #receiver
	std::list<T> msgs; //!< unprocessed messages
	
	//! Holds controller flags for the receiver thread to indicate exit conditions
	class ReceiverThreadBase : public Thread {
	public:
		ReceiverThreadBase() : Thread(), keepRunning(true), block(true) {} //!< constructor
		bool keepRunning; //!< if cleared, indicates receiver should stop processing messages and exit its thread
		bool block; //!< if cleared, indiciates receiver should exit its thread if/when it runs out of messages to process
	};
	
	//! Pulls messages out of the queue and gives them to the specified callback.
	template<typename F, class C>
	class ReceiverThread : public ReceiverThreadBase {
	public:
		//! constructor
		ReceiverThread(ThreadedMessageQueue<T>& tmq, F f, C& c) : ReceiverThreadBase(), q(tmq), fn(f), cl(c) {}
		
	protected:
		ThreadedMessageQueue& q; //!< the queue being monitored
		F fn; //!< function pointer
		C& cl; //!< class pointer
		
		virtual void* run() {
			while(ReceiverThreadBase::keepRunning && (ReceiverThreadBase::block || q.size()>0)) {
				// Note: we must remove the item from from the queue before
				// invoking the callback, because the callback will delete the
				// message.  We never want a queue to contain a pointer to a
				// deleted message, since another thread could find it.  This
				// is what was causing Tekkotsu to crash when running with
				// Mirage.
				T item = q.front();
				q.pop();
				(cl.*fn)(item);
			}
			return NULL;
		}
	};
	ReceiverThreadBase* receiver; //!< currently only a single receiver is supported
	
private:
	ThreadedMessageQueue(const ThreadedMessageQueue& o); //!< Do not call
	ThreadedMessageQueue& operator=(const ThreadedMessageQueue& o); //!< Do not call
};

/*! @file
 * @brief Describes ThreadedMessageQueue, which 
 * @author ejt (Creator)
 */

#endif
