#include "Buffer.h"
#include <string.h>

Buffer::Buffer(int size)
	: data(0), capacity(0), limit(0), position(0) {

	if (size > 0) {
		data = new char[size];
		capacity = size;
	} else {
		capacity = 0;
	}
	limit = capacity;
}

Buffer::Buffer(const Buffer& rhs)
	: data(0), capacity(0), limit(0), position(0) {
		
	*this = rhs;
}

Buffer::~Buffer() {
	delete[] data;
}

Buffer& Buffer::operator=(const Buffer& rhs) {
		
	if (this != &rhs) {
		if (capacity != rhs.GetCapacity()) {
			delete[] data;
			data = 0;
			capacity = rhs.GetCapacity();
			data = new char[capacity];
		}
		
		if (capacity > 0) {			
			memcpy(data, rhs.GetData(), capacity);
		}
		
		position = rhs.GetPosition();
		limit = rhs.GetLimit();
	}
	return *this;
}

void Buffer::SetPosition(int pos) {
	if (pos < 0) {
		position = 0;
	} else {
		position = min(pos, limit);
	}
}

void Buffer::SetLimit(int lim) {
	if (lim < 0) {
		limit = 0;
	} else {
		limit = min(lim, capacity);
	}
	position = min(position, limit);
}

void Buffer::SetCapacity(int size) {
	if (size == capacity) {
		return;
	}
	if (size > 0) {
		char* newData = new char[size];
		if (data != 0) {
			memcpy(newData, data, min(capacity, size)); 
		}
		capacity = size;
		delete[] data;
		data = newData;
	} else {
		capacity = 0;
		delete[] data;
		data = 0;
	}  
	limit = min(limit, capacity);
	position = min(position, limit);
}

bool Buffer::Fill(const char*&src, int& srcLen) {
	if ((src == 0) || (srcLen < 1)) {
		return IsFull();
	}
	
	const int charsToRead = limit - position;
	if (charsToRead < 1) {
		return true;
	}
	
	if (charsToRead > srcLen) {
		memcpy(&data[position], src, srcLen);
		position += srcLen;
		src += srcLen;
		srcLen = 0;
		return false;
	} else {
		memcpy(&data[position], src, charsToRead);
		position += charsToRead;
		src += charsToRead;
		srcLen -= charsToRead;
		return true;
	}
}

/*! @file
 * @brief Describes Buffer, a general memory management data structure, featuring position, limit, and capacity marks
 * @author Alexander Klyubin (A.Kljubin AT herts ac uk) (Creator)
 * Submitted as part of "Full-duplex Audio Streaming" patch
 */
