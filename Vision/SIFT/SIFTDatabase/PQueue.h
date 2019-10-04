#ifndef __PQUEUE_H
#define __PQUEUE_H

#include <vector>
#include <algorithm>

//! Convenient priority queue implemented using the STL's heap algorithm
template <class T>
class PQueue {
public:
	PQueue() : heap() {}
	void push(const T& obj);
	T pop ();
	bool isEmpty();
	
protected:
	std::vector<T> heap;
};

template <class T>
void PQueue<T>::push(const T& obj) {
	heap.push_back(obj);
	push_heap(heap.begin(), heap.end());
}

template <class T>
T PQueue<T>::pop () {
	pop_heap(heap.begin(), heap.end());
	T returnVal = heap.back();
	heap.pop_back();
	return returnVal;
}

template <class T>
bool PQueue<T>::isEmpty() { return (heap.size() == 0); }

#endif
