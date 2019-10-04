#ifndef PLATFORM_APERIOS

#include "MessageReceiver.h"
#include "Shared/debuget.h"
//#include "local/sim/Process.h"
//#include <iostream>
//using namespace std;

MessageReceiver::MessageReceiver(MessageQueueBase& mq, bool (*callback) (RCRegion*)/*=NULL*/, bool startThread/*=true*/, bool subscribe/*=true*/)
: Thread(), queue(mq), semid(mq.getSemaphoreManager()->invalid()),
nextMessage(0), lastProcessedMessage(-1U), process(callback), curit((index_t)-1)
{
	if(startThread)
		start();
	else if(subscribe) {
		ASSERTRET(semid==queue.getSemaphoreManager()->invalid(),"semid is already set?");
		semid=queue.addReceiver();
		if(semid==queue.getSemaphoreManager()->invalid())
			std::cerr << "ERROR: could not start MessageReceiver -- out of semaphore IDs" << std::endl;
	}
}

MessageReceiver::~MessageReceiver() {
	if(!isStarted())
		return;
	stop();
	join();
	queue.removeReceiver(semid);
	semid=queue.getSemaphoreManager()->invalid();
}

RCRegion * MessageReceiver::peekNextMessage() {
	MessageQueueBase::AutoLock autolock(queue.getLock());
	findCurrentMessage();
	if(queue.isEnd(curit))
		return NULL;
	return queue.peekMessage(curit);
}

RCRegion * MessageReceiver::getNextMessage() {
	MessageQueueBase::AutoLock autolock(queue.getLock());
	findCurrentMessage();
	if(queue.isEnd(curit))
		return NULL;
	nextMessage=queue.getMessageSN(curit)+1;
	curit=queue.newer(curit); //next time, start on (or peek at) the one after this
	return queue.readMessage(curit,semid);
}

Thread& MessageReceiver::stop() {
	Thread::stop();
	queue.getSemaphoreManager()->raise(semid,1); //trigger a check so the thread will notice the stop
	return *this;
}

void MessageReceiver::findCurrentMessage() {
	if(queue.isEnd(curit)) {
		curit=queue.newest(); //start with the newest
		while(!queue.isEnd(curit) && queue.getMessageSN(curit)>=nextMessage)
			curit=queue.older(curit); //scan back to the first already read by this process
		curit=queue.newer(curit); //go to the following one (first unread)
	} else {
		while(!queue.isEnd(curit) && queue.getMessageSN(curit)<nextMessage)
			curit=queue.newer(curit); //scan forward to next message not read by this process
	}
}

void MessageReceiver::finish() {
	if(isStarted()) {
		stop();
		join();
	}
	if(semid!=queue.getSemaphoreManager()->invalid()) {
		//cout << Process::getName() << " finish" << endl;
		while(processNextMessage()) {}
		queue.removeReceiver(semid);
		semid=queue.getSemaphoreManager()->invalid();
	}
}

bool MessageReceiver::launched() {
	if(semid==queue.getSemaphoreManager()->invalid())
		semid=queue.addReceiver();
	if(semid==queue.getSemaphoreManager()->invalid()) {
		std::cerr << "ERROR: could not start MessageReceiver -- out of semaphore IDs" << std::endl;
		return false;
	}
	return Thread::launched();
}

unsigned int MessageReceiver::runloop() {
	//cout << Process::getName() << " runloop" << endl;
	pushNoCancel();
	waitNextMessage();
	while(processNextMessage()) { //get everything else in the queue
		queue.getSemaphoreManager()->lower(semid,1,false);
	}
	popNoCancel();
	return 0;
}

bool MessageReceiver::waitNextMessage() {
	return queue.getSemaphoreManager()->lower(semid,1,true);
}

bool MessageReceiver::processNextMessage() {
	RCRegion * msg=peekNextMessage();
	if(msg==NULL)
		return false;
	//cout << Process::getName() << " got " << msg->ID().key << ' ' << lastProcessedMessage << ' ' << queue.getMessageSN(curit) << ' ' << curit << endl;
	bool used=false;
	if(lastProcessedMessage!=queue.getMessageSN(curit)) {
		lastProcessedMessage=queue.getMessageSN(curit);
		//cout << Process::getName() << " process received " << lastProcessedMessage << " at " << TimeET() << endl;
		used=process(msg);
		if(used)
			markRead(false); // message was consumed, mark it read
		//cout << used << ' ' << curit;
		//if(!queue.isEnd(curit))
		//cout << lastProcessedMessage << ' ' << queue.getMessageSN(curit);
		//cout << endl;
	}
	msg->RemoveReference();
	return used;
}

void MessageReceiver::markRead(bool checkNext) {
	findCurrentMessage();
	if(queue.isEnd(curit))
		return;
	nextMessage=queue.getMessageSN(curit)+1;
	queue.markRead(curit,semid);
	curit=queue.newer(curit); //next time, start on (or peek at) the one after this
	if(checkNext && !queue.isEnd(curit))
		queue.getSemaphoreManager()->raise(semid,1); //trigger a check if there are more waiting in the queue
}



/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif //PLATFORM_APERIOS check (aperios doesn't support pthreads...)
