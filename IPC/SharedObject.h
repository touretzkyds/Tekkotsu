//-*-c++-*-
#ifndef INCLUDED_SharedObject_h
#define INCLUDED_SharedObject_h

#include "RCRegion.h"
#include <stdexcept>
#include <typeinfo>

//! It's nice to have a parent class of SharedObject (which is what you probably want to be reading) so that you can pass around the data structure without worrying about what type is inside the shared memory region.
/*! See MotionManager for an example on how to use this. */
class SharedObjectBase {
public:
	//! when passed to the SharedObject constructor, indicates that no RCRegion should be created
	struct NoInit {};
	
	void* data() const { return rcr->Base(); } //!< returns a pointer to the data region
	RCRegion * getRegion() const { return rcr; } //!< returns the OPEN-R memory region, should you need it
	
	virtual ~SharedObjectBase() {} //!< destructor
	
#ifndef PLATFORM_APERIOS
	//! return the next region serial number -- doesn't actually increment it though, repeated calls will return the same value until the value is actually used
	static unsigned int getNextKey() { return serialNumber+1; }
#endif

	//! assignment, adds a reference to the existing region (shallow copy)
	SharedObjectBase& operator=(const SharedObjectBase& sob) {
		if(rcr==sob.rcr)
			return *this;
		removeRef();
		rcr=sob.rcr;
		if(rcr!=NULL)
			rcr->AddReference();
		return *this;
	}
	
protected:
	//! constructor, protected because you shouldn't need to create this directly, just a common interface to all templates of SharedObject
	SharedObjectBase() : rcr(NULL) {}
	//! copy constructor, adds a reference to the existing region (shallow copy)
	SharedObjectBase(const SharedObjectBase& sob) : rcr(sob.rcr) {
		if(rcr!=NULL)
			rcr->AddReference();
	}

	//!removes a reference from #rcr, and if necessary, destructs its data
	virtual void removeRef()=0;
	
	RCRegion * rcr; //!< the pointer to the shared memory region this is in charge of
	
#ifndef PLATFORM_APERIOS
	static unsigned int serialNumber; //!< incremented for each region created so they will all have unique IDs
#endif
};	


//! This templated class allows convenient creation of any type of class wrapped in a shared memory region
/*! @see MotionManager for an example on how to use this.*/
template<class MC>
class SharedObject : public SharedObjectBase {
public:
	
	//!if you really need more than 5 arguments for your class, well, you're one crazy puppy but if you really want to, just make more like shown... (yay templates!)
	//!@name templated contructors - allows you to pass constructor arguments on to the object being created

	//! Creates the class with the default constructor
	SharedObject() : SharedObjectBase() {
		rcr=createRCRegion();
		new (rcr->Base()) MC;
	}
	//! Creates the class, passing its constructor t1
	template<class T1> explicit SharedObject(const T1& t1) : SharedObjectBase() {
		rcr=createRCRegion();
		new (rcr->Base()) MC(t1);
	}
	//! Creates the class, passing its constructor t1 and t2
	template<class T1, class T2> SharedObject(const T1& t1, const T2& t2) : SharedObjectBase(){
		rcr=createRCRegion();
		new (rcr->Base()) MC(t1,t2);
	}
	//! Creates the class, passing its constructor t1, t2, and t3
	template<class T1, class T2, class T3> SharedObject(const T1& t1, const T2& t2, const T3& t3) : SharedObjectBase(){
		rcr=createRCRegion();
		new (rcr->Base()) MC(t1,t2,t3);
	}
	//! Creates the class, passing its constructor t1, t2, t3 and t4
	template<class T1, class T2, class T3, class T4> SharedObject(const T1& t1, const T2& t2, const T3& t3, const T4& t4) : SharedObjectBase(){
		rcr=createRCRegion();
		new (rcr->Base()) MC(t1,t2,t3,t4);
	}
	//! Creates the class, passing its constructor t1, t2, t3, t4 and t5 - if you need more arguments, just add them
	template<class T1, class T2, class T3, class T4, class T5> SharedObject(const T1& t1, const T2& t2, const T3& t3, const T4& t4, const T5& t5) : SharedObjectBase(){
		rcr=createRCRegion();
		new (rcr->Base()) MC(t1,t2,t3,t4,t5);
	}
	//@}

	//! a no-op constructor, so you can transfer a SharedObject created elsewhere with operator=() later
	SharedObject(const SharedObjectBase::NoInit&) : SharedObjectBase() {}
	
	//! copy constructor adds a reference to the underlying RCRegion, allowing the SharedObject instances to "share" the region
	SharedObject(const SharedObject& sh) : SharedObjectBase(sh) {}
	
	//! Constructs from a pre-existing region, laying claim to the caller's reference to the region - region's creator is responsible for initialization
	/*! In other words, this SharedObject doesn't addReference, but will
	 *  removeReference when the time is right (upon destruction).  If
	 *  you want to maintain an reference of your own to the region, you
	 *  will need to call addReference yourself.
	 *
	 *  In comparison, the copy constructor will add a reference to the
	 *  underlying RCRegion and "share" the RCRegion with the original
	 *  SharedObject */
	explicit SharedObject(RCRegion * r) : SharedObjectBase() {
		rcr=r;
		if(rcr->Size()!=sizeof(MC)) {
#ifdef PLATFORM_APERIOS
			std::cerr << "ERROR: SharedObject(RCRegion* r) region size ("<<rcr->Size()<<") does not match size of SharedObject type ("<<sizeof(MC)<<")" << std::endl;
#else
			std::cerr << "ERROR: SharedObject(RCRegion* r) region "<<rcr->ID().key<<" size ("<<rcr->Size()<<") does not match size of SharedObject type ("<<sizeof(MC)<<")" << std::endl;
#endif
			throw std::invalid_argument("SharedObject(RCRegion* r): region size does not match sizeof(type)");
		}
	}
	
	//! destructor, removes reference to shared region
	virtual ~SharedObject() { removeRef(); }
	
	MC* operator->() const { return dataCasted(); } //!< smart pointer to the underlying class
	MC& operator*() const { return *dataCasted(); } //!< smart pointer to the underlying class
	MC& operator[](int i) const { return dataCasted()[i]; } //!< smart pointer to the underlying class
	
protected:
	MC* dataCasted() const { return static_cast<MC*>(data()); } //!< returns a correctly typed pointer to the object's memory
	
	//!removes a reference from #rcr, and if necessary, destructs its data
	virtual void removeRef() {
		if(rcr!=NULL) {
			//std::cout << "~SharedObjectBase(): rcr->NumberOfReference()==" << rcr->NumberOfReference() << std::endl;
			if(rcr->NumberOfReference()>0) {
				if(rcr->NumberOfReference()==1)
					dataCasted()->~MC(); //call destructor
				rcr->RemoveReference();
			} else
				std::cerr << "WARNING: SharedObjectBase destructed without reference" << std::endl;
			rcr=NULL;
		}
	}
	
	//! creates and returns RCRegion of correct size for current class.  Adds a reference (which is removed in the destructor)
	static RCRegion * createRCRegion() {
#ifdef PLATFORM_APERIOS
		RCRegion * r = new RCRegion(calcsize());
#else
		char name[RCRegion::MAX_NAME_LEN];
		unsigned int suffixlen=snprintf(name,RCRegion::MAX_NAME_LEN,".%d.%d",ProcessID::getID(),++serialNumber);
		if(suffixlen>RCRegion::MAX_NAME_LEN)
			suffixlen=RCRegion::MAX_NAME_LEN;
		snprintf(name,RCRegion::MAX_NAME_LEN,"Sh.%.*s.%d.%d",RCRegion::MAX_NAME_LEN-suffixlen-3,typeid(MC).name(),ProcessID::getID(),++serialNumber);
		name[RCRegion::MAX_NAME_LEN-1]='\0';
		RCRegion * r = new RCRegion(name,calcsize());
#endif
		//cout << "SIZE is " << r->Size() << endl;
		//cout << "BASE is " << (void*)r->Base() << endl;
		//std::cout << "createRCRegion(): rcr->NumberOfReference()==" << r->NumberOfReference() << std::endl;
		//r->addReference();
		//std::cout << "createRCRegion()NOW: rcr->NumberOfReference()==" << r->NumberOfReference() << std::endl;
		return r;
	}

	//!Calculates the size of the memory region to be used, (on Aperios, rounding up to the nearest page size)
	/*! Not sure this is completely necessary, but may be nice.  Of course, this also means even
	 *  small regions are going to be at least 4K (current page size)  If memory gets tight or we
	 *  get a lot of little regions floating around, this might be worth checking into */
	static unsigned int calcsize() {
#ifndef PLATFORM_APERIOS
		return sizeof(MC);
#else
		size_t size = sizeof(MC);
		sError error;
		size_t page_size;
		error = GetPageSize(&page_size);
		if (error != sSUCCESS) {
			cout << "error: " << error << " getting page size in SharedMem" << endl;
			page_size = 4096;
		}
		
		int new_size,num_pages;
		num_pages = (size+page_size-1)/page_size;
		new_size = num_pages*page_size;
		//cout << "req" << size << "new" << new_size << "ps" << page_size << endl;
		/*
		cout << "data size is " << sizeof(MC) << endl;
		cout << "PAGE is " << page_size << endl;
		cout << "reserved " << new_size << " bytes" << endl;
		*/
		return new_size;
#endif //!PLATFORM_APERIOS
	}
};

/*! @file
 * @brief Defines SharedObject, a wrapper for objects in order to facilitate sending them between processes
 * @author ejt (Creator)
 */

#endif //INCLUDED_SharedObject_h
