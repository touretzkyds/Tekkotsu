//-*-c++-*-
#ifndef INCLUDED_Buffer_h_
#define INCLUDED_Buffer_h_

//! Buffer.
/*! A buffer has three main properties: position, capacity and limit.
 * Capacity is the real size of the underlying array.
 * Position is the index of the current element in the buffer (used only by
 * buffer filling operations at the moment).
 * Limit is the virtual size of the buffer. Operations such as filling up
 * the buffer, seeking and so on never go over the limit mark of the buffer. 
 *
 * 0 <= position <= limit <= capacity.
 */
class Buffer {
	public:
	  //! Constructs a new buffer of specified capacity and limit
		Buffer(int size);
		//! Constructs a copy of the buffer
		Buffer(const Buffer& rhs);
		//! Makes this buffer a copy of the rhs buffer
		Buffer& operator=(const Buffer& rhs);
		//! destructor, deletes #data
		virtual ~Buffer();
		
		//! Gets the pointer to the first element of the underlying array.
		const char* GetData() const { return data; }
		//! Gets the pointer to the first element of the underlying array.
		char* GetData() { return data; }
		//! Gets the capacity of the buffer.
		int GetCapacity() const { return capacity; }
		//! Sets the capacity of the buffer. The underlying array grows and shrinks.
		void SetCapacity(int size);
		//! Gets the current position. position <= limit.
		int GetPosition() const { return position; }
		//! Gets the limit mark of the buffer. limit <= capacity
		int GetLimit() const { return limit; }
		//! Sets the current position.
		void SetPosition(int pos);
		//! Sets the limit mark. limit <= capacity
		void SetLimit(int lim);
		//! Tries to fill the buffer from current position up to the limit mark. Advances the position, src and srcLen. Returns true if the buffer has been filled.
		bool Fill(const char*&src, int& srcLen);
		//! Tries to fill the buffer from current position up to the limit mark. Advances the position, src and srcLen. Returns true if the buffer has been filled.
		bool Fill(char*& src, int& srcLen) { return Fill(const_cast<const char*&>(src), srcLen); }
		//! Checks whether the buffer is full, that is position == limit.
		bool IsFull() const { return (position >= limit); }
	private:
		char* data;   //!< the buffer itself
		int capacity; //!< the the real size of the underlying array.
		int limit; //!< Position is the index of the current element in the buffer (used only by buffer filling operations at the moment)
		int position; //!< Limit is the virtual size of the buffer. Operations such as filling up the buffer, seeking and so on never go over the limit mark of the buffer. 
		
		//! returns the lesser of @a a or @a b
		static int min(int a, int b) { return ((a < b) ? a : b); }
};


/*! @file
 * @brief Describes Buffer, a general memory management data structure, featuring position, limit, and capacity marks
 * @author Alexander Klyubin (A.Kljubin AT herts ac uk) (Creator)
 * Submitted as part of "Full-duplex Audio Streaming" patch
 */

#endif
