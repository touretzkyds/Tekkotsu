#ifndef NETWORK_BUFFER_H_
#define NETWORK_BUFFER_H_

/*! This is used to incrementally build up a buffer to be sent over
 *  the network , so that only one call to the socket write function
 *  needs to be made. */
class NetworkBuffer {
public:
	
	//! constructor
	NetworkBuffer() : buf(NULL), offset(0), bufSize(1024) {
		buf = new byte[bufSize];
	}

	//! destructor
	virtual ~NetworkBuffer() {
		delete[] buf;
	}

	//! Template for adding a single item to the buffer, such as a struct or an int
	template <class T> bool addItem(T item) {
		if (offset+sizeof(T) > (unsigned int)bufSize)
			return false;
		
		*(T *)(buf+offset) = item;
		offset += sizeof(T);
		return true;
	}

	//! Template for adding a buffer such with a size to the network buffer
	bool addBuffer(byte *src, int size) {
		if (!addItem(size) || (offset+size > bufSize))
			return false;

		memcpy(buf+offset, src, size);
		offset += size;
		return true;
	}

	//! Returns the current size of the buffer
	int getSize() { return offset; }

	//! Returns a const pointer to the buffer, for debugging networking code
  const byte* getBytes() { return buf; }

	//! Sends the buffer over the given socket
	bool send(Socket *sck) {
		if (sck->write(buf, offset) != static_cast<int>(offset)) {
			std::cout << "Error sending buffer" << std::endl;
			return false;
		}
		
		return true;
	}
	
protected:
	byte *buf; //!< the buffer being built
	size_t offset; //!< current position in #buf
	size_t bufSize; //!< size of memory region at #buf

private:
	NetworkBuffer(NetworkBuffer&); //!< do not call
	NetworkBuffer &operator=(const NetworkBuffer&); //!< do not call
};

#endif
