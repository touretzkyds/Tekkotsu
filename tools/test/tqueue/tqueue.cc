#include "IPC/Thread.h"
#include "IPC/ThreadedMessageQueue.h"
#include "Shared/MarkScope.h"
#include "Shared/TimeET.h"
#include <list>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <cstdlib>

size_t N = 6000; // number of iterations to perform
size_t T = 3; // number of sending threads to use
std::list<size_t> in, out;
Thread::Lock inLock;
ThreadedMessageQueue<size_t> q;

class SendThread : public Thread {
public:
	SendThread() : Thread(), sent(0) {}
	size_t getCount() const { return sent; }
	
protected:
	size_t getMsg() {
		MarkScope l(inLock);
		if(in.size()==0)
			throw std::underflow_error("done");
		size_t m = in.front();
		in.pop_front();
		return m;
	}
	virtual void* run() {
		try {
			for(;;) {
				size_t m = getMsg();
				q.send(m);
				++sent;
			}
		} catch(const std::exception&) {}
		return NULL;
	}
	size_t sent;
};

struct Receiver {
	void gotMsg(size_t m) {
		out.push_back(m);
		//std::cout << m << std::endl;
		usleep(100);
	}
};

size_t sendMessages() {
	q.clear();
	out.clear();
	in.clear();
	for(size_t i=0; i<N; ++i)
		in.push_back(i);
	
	TimeET sendTime;
	std::vector<SendThread*> threads;
	for(size_t i=0; i<T; ++i) {
		threads.push_back(new SendThread);
		threads.back()->start();
	}
	size_t sent=0;
	for(size_t i=0; i<T; ++i) {
		threads[i]->join();
		std::cout << "Thread " << i << " sent @VAR " << threads[i]->getCount() << std::endl;
		sent+=threads[i]->getCount();
		delete threads[i];
	}
	std::cout << "Sent in @VAR " << sendTime.Age() << std::endl;
	return sent;
}

int main(int argc, const char* argv[]) {
	Thread::initMainThread();
	Receiver r;
	size_t sent=0;
	
	std::cout << "ThreadedMessageQueue::stopCallback() test:" << std::endl;
	q.spawnCallback(&Receiver::gotMsg,r);
	sent=sendMessages();
	usleep(50000);
	q.stopCallback();
	std::cout << "Messages sent: " << sent << std::endl;
	std::cout << "Messages received: @VAR " << out.size() << std::endl;
	if(out.size()>sent) std::cout << "ERROR: phantom messages!" << std::endl;
	std::cout << std::endl;
	
	std::cout << "ThreadedMessageQueue::finishCallback test:" << std::endl;
	q.spawnCallback(&Receiver::gotMsg,r);
	sent=sendMessages();
	usleep(50000);
	q.finishCallback();
	std::cout << "Messages sent: " << sent << std::endl;
	std::cout << "Messages received: @VAR " << out.size() << std::endl;
	if(out.size()>sent) std::cout << "ERROR: phantom messages!" << std::endl;
	std::cout << std::endl;
	
	std::cout << "ThreadedMessageQueue::finishQueue() test:" << std::endl;
	q.spawnCallback(&Receiver::gotMsg,r);
	sent=sendMessages();
	q.finishQueue();
	std::cout << "Messages sent: " << sent << std::endl;
	std::cout << "Messages received: " << out.size() << std::endl;
	if(out.size()>sent) std::cout << "ERROR: phantom messages!" << std::endl;
	std::cout << std::endl;
}
