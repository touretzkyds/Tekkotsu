#ifndef PLATFORM_APERIOS
#include "MessageQueueStatusThread.h"
#include "MessageQueue.h"
#include "Shared/debuget.h"
#include <algorithm>

using namespace std; 

MessageQueueStatusThread::~MessageQueueStatusThread() {
	if(isStarted()) {
		stop();
		//join(); // join turns out to be a bad idea here -- the thread being stopped may be waiting on a lock we currently hold
	}
}

void MessageQueueStatusThread::addStatusListener(StatusListener* l) {
	if(l==NULL)
		return;
	if(find(statusListeners.begin(),statusListeners.end(),l)==statusListeners.end()) {
		//not already added
		statusListeners.push_back(l);
		if(!isStarted()) {
			if(queue==NULL)
				return;
			semid=queue->addReadStatusListener();
			if(semid==queue->getSemaphoreManager()->invalid()) {
				std::cerr << "ERROR: could not start MessageQueueStatusThread -- out of semaphore IDs" << std::endl;
				return;
			}
			//cout << "MessageQueueStatusThread added MessageQueue read listener" << endl;
			numRead=queue->getMessagesRead();
			start();
		}
	}
}

void MessageQueueStatusThread::removeStatusListener(StatusListener* l) {
	std::list<StatusListener*>::iterator it=find(statusListeners.begin(),statusListeners.end(),l);
	if(it!=statusListeners.end())
		statusListeners.erase(it);
	if(isStarted() && statusListeners.size()==0) {
		stop();
		//join(); // join turns out to be a bad idea here -- the thread being stopped may be waiting on a lock we currently hold
	}
}

void MessageQueueStatusThread::setMessageQueue(MessageQueueBase& mq) {
	if(running) {
		stop();
		//join(); // join turns out to be a bad idea here -- the thread being stopped may be waiting on a lock we currently hold
	}
	queue=&mq;
	if(statusListeners.size()!=0)
		start();
	/*
	MessageQueueBase* oldqueue=queue;
	SemaphoreManager::semid_t oldsem=semid;
	queue=&mq;
	semid=queue->addReadStatusListener();
	if(semid==queue->getSemaphoreManager()->invalid()) {
		std::cerr << "ERROR: could not switch MessageQueue -- new queue out of semaphores, stopping thread" << std::endl;
		queue=oldqueue;
		semid=oldsem;
		if(running)
			stop();
		return;
	}
	numRead=queue->getMessagesRead();
	if(oldqueue!=NULL && oldsem!=queue->getSemaphoreManager()->invalid()) {
		if(running)
			oldqueue->getSemaphoreManager()->raise(oldsem,1); //so run will notice the switchover
		oldqueue->removeReadStatusListener(oldsem);
	}*/
}

MessageQueueBase* MessageQueueStatusThread::getMessageQueue() {
	return queue;
}

bool MessageQueueStatusThread::launched() {
	return Thread::launched();
}

void * MessageQueueStatusThread::run() {
	for(;;) {
		queue->getSemaphoreManager()->lower(semid,1,true);
		//there might be a few reads, handle them as a group
		unsigned int more=queue->getSemaphoreManager()->getValue(semid);
		if(more>0)
			if(!queue->getSemaphoreManager()->lower(semid,more,false))
				std::cerr << "WARNING: MessageQueueStatusThread had a message notification disappear (is someone else using the semaphore?  Get your own!)" << std::endl;
		testCancel();
#ifdef DEBUG
		// This part just for sanity checking -- could do away with numRead altogether otherwise
		unsigned int nowRead=queue->getMessagesRead();
		unsigned int read=nowRead-numRead;
		numRead=nowRead;
		ASSERT(read==more+1,"WARNING: MessageQueueStatusThread's semaphore count does not match queue's read count ("<< (more+1) << " vs " << read<<")");
#endif
		//ok, notify the listeners
		fireMessagesRead(more+1);
	}
	return NULL; // not going to happen, just to make compiler happy
}

Thread& MessageQueueStatusThread::stop() {
	Thread::stop();
	if(semid!=queue->getSemaphoreManager()->invalid()) //if semid is still invalid, probably canceling before the launch got off
		queue->getSemaphoreManager()->raise(semid,1); //so run will notice the stop request
	return *this;
}

void MessageQueueStatusThread::cancelled() {
	if(queue==NULL)
		return;
	//cout << "MessageQueueStatusThread removing MessageQueue read listener" << endl;
	queue->removeReadStatusListener(semid);
	semid=queue->getSemaphoreManager()->invalid();
}

void MessageQueueStatusThread::fireMessagesRead(unsigned int howmany) {
	if(howmany==0)
		return;
	std::list<StatusListener*>::iterator it=statusListeners.begin();
	while(it!=statusListeners.end()) {
		std::list<StatusListener*>::iterator cur=it++; //increment early in case the listener changes subscription
		(*cur)->messagesRead(*queue,howmany);
	}
}


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
#endif
