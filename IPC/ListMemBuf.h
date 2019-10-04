//-*-c++-*-
#ifndef INCLUDED_ListMemBuf_h
#define INCLUDED_ListMemBuf_h

//! Provides some degree of dynamic allocation of a templated type from a buffer of set size.
/*! Think of this as a self-contained mini-malloc...
	  
    This is handy for classes which inhabit a shared memory region, where
    it's a bad idea to have pointers to other memory.  By instantiating one
    of these in your class, you can allocate space internally for up to
    MAX objects of type T_t.  ListMemBuf will worry about keeping track
    of which ones are in use or are free.

    Each time you request a entry to be created, the destructor will be
    called followed by the the defaul constructor before it is given to
    you, so the fields should be reliably 'fresh', not what was in the
    entry last time it was used.
*/
template < class T_t, unsigned int MAX, class idx_t=unsigned short >
class ListMemBuf {
	public:
	
	ListMemBuf(); //!<constructor
	~ListMemBuf(); //!<destructor
	
	//!Allows outside access to storage type
	typedef T_t T;
	//!Allows outside access to number of entries
	static const unsigned int MAX_ENTRIES = MAX;
	//!Allows outside access to index type
	typedef idx_t index_t;
	
	static index_t getMaxCapacity() { return MAX_ENTRIES; } //!<returns the maximum number of objects which can be used at any given time
	index_t size() const { return cursize; } //!<returns the current number of objects in use
	index_t countf() const; //!< for debugging, should equal size
	index_t countb() const; //!< for debugging, should equal size
	bool empty() const { return cursize==0; } //!<returns true if no objects are in use

	// the funny '(T*)(void*)foo' double-casts below are to avoid gcc warnings
	// regarding 'dereferencing type-punned pointer will break strict-aliasing rules'
	
	inline T& operator[](unsigned int x) { return *(T*)(void*)(entries[x].data); } //!<allows direct access to elements - be careful, can access 'free' elements this way
	inline const T& operator[](unsigned int x) const { return *(const T*)(const void*)(entries[x].data); } //!<allows direct access to elements - be careful, can access 'free' elements this way

	inline index_t begin() const { return activeBegin; } //!<returns index of first used entry
	T& front() { return *(T*)(void*)(entries[activeBegin].data); } //!< returns reference to first used entry
	const T& front() const { return *(const T*)(const void*)(entries[activeBegin].data); } //!< returns const reference to first used entry

	inline index_t end() const { return (index_t)-1; } //!<returns the one-past-end index
	T& back() { return *(T*)(void*)(entries[activeBack].data); } //!<returns reference to last used entry
	const T& back() const { return *(const T*)(const void*)(entries[activeBack].data); } //!<returns const reference to last used entry

	index_t new_front(); //!<pushes a 'blank' entry on the front of the used list
	index_t push_front(const T& data) { index_t tmp=new_front(); if(tmp!=end()) operator[](tmp)=data; return tmp; } //!<pushes an entry on the front of the used chain and assigns @a data to it
	void pop_front(); //!<pops the front of the used chain
	void pop_front(T& ret) { ret=front(); pop_front(); } //!<pops the front of the chain into @a ret

	index_t new_back(); //!<pushes a 'blank' entry on the back of the used list
	index_t push_back(const T& data) { index_t tmp=new_back(); if(tmp!=end()) operator[](tmp)=data; return tmp; } //!<pushes an entry on the back of the used chain and assigns @a data to it
	void pop_back(); //!<pops the last of the used chain
	void pop_back(T& ret) { ret=back(); pop_back(); } //!<pops the last of the used chain into @a ret

	index_t new_before(index_t x); //!<inserts a 'blank' entry before element @a x in the used chain
	index_t push_before(index_t x, const T& data) { index_t tmp=new_before(x); if(tmp!=end()) operator[](tmp)=data; return tmp; } //!<inserts a 'blank' entry before element @a x in the used chain and assigns @a data to it

	index_t new_after(index_t x) { return new_before(next(x)); } //!<inserts a 'blank' entry after element @a x in the used chain
	index_t push_after(index_t x, const T& data) { index_t tmp=new_after(x); if(tmp!=end()) operator[](tmp)=data; return tmp; } //!<inserts a 'blank' entry after element @a x in the used chain and assigns @a data to it

	void erase(index_t x); //!<removes element @a x from the used chain
	void clear(); //!<frees all used entries

	void swap(index_t a, index_t b); //!<swaps the two entries' position in the list

	index_t next(index_t x) const { return x==end()?activeBegin:entries[x].next; } //!< returns the next used element following @a x
	index_t prev(index_t x) const { return x==end()?activeBack:entries[x].prev; } //!< returns the preceeding used element following @a x
 
	protected:
	index_t pop_free(); //!<removes an element from the front of the free list, returns its index
	void push_free(index_t x); //!<pushes @a x onto the back of the free list
	
	//!holds data about an entry in the free/used lists
	struct entry_t {
		//!constructor
		entry_t() : next(static_cast<index_t>(-1)), prev(static_cast<index_t>(-1)) {}
		double data[(sizeof(T)-1)/sizeof(double)+1]; //!<The data being stored, not actually an instantiation of T, but big enough to hold it.  (Funky array size is to ensure proper alignment of contents)
		index_t next; //!<The next element in the used or free chain
		index_t prev; //!<The previous element in the used chain, invalid if in the free chain
	};
	entry_t entries[MAX_ENTRIES==0?1:MAX_ENTRIES]; //!<the main block of data; must have at least 1 element due to limitation of older compilers
	index_t activeBegin; //!<beginning of used chain
	index_t activeBack; //!<end of used chain
	index_t freeBegin; //!<beginning of free chain
	index_t freeBack; //!<end of free chain
	index_t cursize; //!< current number of used elements
};

template < class T, unsigned int MAX, class index_t >
ListMemBuf<T,MAX,index_t>::ListMemBuf()
	: activeBegin(end()), activeBack(end()), freeBegin(end()), freeBack(end()), cursize(0)
{
	for(unsigned int x=0; x+1<MAX_ENTRIES; x++)
		entries[x].next=x+1;
	entries[MAX_ENTRIES-1].next=end();
	freeBegin=0;
	freeBack=MAX_ENTRIES-1;
}

template < class T, unsigned int MAX, class index_t >
ListMemBuf<T,MAX,index_t>::~ListMemBuf() {
	clear();
}

template < class T, unsigned int MAX, class index_t>
index_t
ListMemBuf<T,MAX,index_t>::countf() const {
	int x=0;
	for(index_t c=begin(); c!=end(); c=next(c))
		x++;
	return x;
}

template < class T, unsigned int MAX, class index_t>
index_t
ListMemBuf<T,MAX,index_t>::countb() const {
	int x=0;
	for(index_t c=end(); c!=begin(); c=prev(c))
		x++;
	return x;
}

template < class T, unsigned int MAX, class index_t >
index_t
ListMemBuf<T,MAX,index_t>::new_front() {
	index_t tmp = pop_free();
	if(tmp==end())
		return end();
	entries[tmp].prev=end();
	entries[tmp].next=activeBegin;
	if(activeBegin!=end())
		entries[activeBegin].prev=tmp;
	else
		activeBack=tmp;
	activeBegin=tmp;
	return tmp;
}

template < class T, unsigned int MAX, class index_t >
index_t
ListMemBuf<T,MAX,index_t>::new_back() {
	index_t tmp = pop_free();
	if(tmp==end())
		return end();
	entries[tmp].prev=activeBack;
	entries[tmp].next=end();
	if(activeBack!=end())
		entries[activeBack].next=tmp;
	else
		activeBegin=tmp;
	activeBack=tmp;
	return tmp;
}

template < class T, unsigned int MAX, class index_t >
index_t
ListMemBuf<T,MAX,index_t>::new_before(index_t x) {
	if(x==end())
		return new_back();
	if(entries[x].prev==end())
		return new_front();
	index_t tmp = pop_free();
	if(tmp==end())
		return end();
	entries[tmp].next=x;
	entries[tmp].prev=entries[x].prev;
	entries[x].prev=tmp;
	entries[ entries[tmp].prev ].next = tmp;
	return tmp;
}

template < class T, unsigned int MAX, class index_t >
void
ListMemBuf<T,MAX,index_t>::pop_front() {
	index_t tmp = activeBegin;
	activeBegin = entries[activeBegin].next;
	if(activeBegin==end())
		activeBack=end();
	else
		entries[activeBegin].prev = end();
	push_free(tmp);
}

template < class T, unsigned int MAX, class index_t >
void
ListMemBuf<T,MAX,index_t>::pop_back() {
	index_t tmp = activeBack;
	activeBack = entries[activeBack].prev;
	if(activeBack==end())
		activeBegin=end();
	else
		entries[activeBack].next = end();
	push_free(tmp);
}

template < class T, unsigned int MAX, class index_t >
void
ListMemBuf<T,MAX,index_t>::erase(index_t x) {
	if(x==activeBegin) {
		pop_front();
		return;
	}
	if(x==activeBack) {
		pop_back();
		return;
	}
	entries[ entries[x].next ].prev = entries[x].prev;
	entries[ entries[x].prev ].next = entries[x].next;
	push_free(x);
}

template < class T, unsigned int MAX, class index_t >
void
ListMemBuf<T,MAX,index_t>::clear() {
	if(cursize!=0) {
		for(index_t it=activeBegin; it!=end(); it=entries[it].next)
			operator[](it).~T();
		if(freeBack==end())
			freeBegin=activeBegin;
		else
			entries[freeBack].next=activeBegin;
		freeBack=activeBack;
		activeBegin=activeBack=end();
	}
	cursize=0;
}

template < class T, unsigned int MAX, class index_t >
void
ListMemBuf<T,MAX,index_t>::swap(index_t a, index_t b) {
	if(a==b || a==end() || b==end())
		return;
	if(entries[a].prev==b) {
		entries[a].prev=entries[b].prev;
		entries[b].next=entries[a].next;
		entries[a].next=b;
		entries[b].prev=a;
		if(entries[a].prev!=end())
			entries[entries[a].prev].next=a;
		else
			activeBegin=a;
		if(entries[b].next!=end())
			entries[entries[b].next].prev=b;
		else
			activeBack=b;
	} else if(entries[a].next==b) {
		entries[a].next=entries[b].next;
		entries[b].prev=entries[a].prev;
		entries[a].prev=b;
		entries[b].next=a;
		if(entries[a].next!=end())
			entries[entries[a].next].prev=a;
		else
			activeBack=a;
		if(entries[b].prev!=end())
			entries[entries[b].prev].next=b;
		else
			activeBegin=b;
	} else {
		index_t tmpp=entries[a].prev, tmpn=entries[a].next;
		entries[a].prev=entries[b].prev;
		entries[a].next=entries[b].next;
		entries[b].prev=tmpp;
		entries[b].next=tmpn;
		if(entries[a].prev!=end())
			entries[entries[a].prev].next=a;
		else
			activeBegin=a;
		if(entries[a].next!=end())
			entries[entries[a].next].prev=a;
		else
			activeBack=a;
		if(entries[b].prev!=end())
			entries[entries[b].prev].next=b;
		else
			activeBegin=b;
		if(entries[b].next!=end())
			entries[entries[b].next].prev=b;
		else
			activeBack=b;
	}
	/*
	// Front Back => Front Back
	//  a     b       b     a
	//  a     -       b     -
	//  b     a       a     b
	//  b     -       a     -
	//  -     a       -     b
	//  -     b       -     a
	if(activeBegin==a) {
		activeBegin=b;
		if(activeBack==b)
			activeBack=a;
	} else if(activeBegin==b) {
		activeBegin=a;
		if(activeBack==a)
			activeBack=b;
	} else {
		if(activeBack==a)
			activeBack=b;
		else if(activeBack=b)
			activeBack==a;
	}
	*/
}

/*! free list is a queue... pop front, push back - hopefully more robust with multi-threads
    is purposely sloppy with unused links, a little faster*/
template < class T, unsigned int MAX, class index_t >
index_t
ListMemBuf<T,MAX,index_t>::pop_free() {
	if(freeBegin==end())
		return end();
	index_t tmp=freeBegin;
	if(freeBegin==freeBack)
		freeBegin=freeBack=end();
	else
		freeBegin=entries[freeBegin].next;
	cursize++;
	new (entries[tmp].data) T;  //calls constructor so that the data is "fresh"
	return tmp;
}

/*! @see pop_free() */
template < class T, unsigned int MAX, class index_t >
void
ListMemBuf<T,MAX,index_t>::push_free(index_t x) {
	if(freeBack==end())
		freeBegin=x;
	else
		entries[freeBack].next=x;
	freeBack=x;
	cursize--;
	operator[](x).~T(); //to match the constructor call in pop_free() (or the entry_t constructor during initialization)
}

/*! @file
 * @brief Defines ListMemBuf, which provides some degree of dynamic allocation of a templated type from a buffer of set size.
 * @author ejt (Creator)
 */
 
 #endif
