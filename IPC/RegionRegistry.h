//-*-c++-*-
#ifndef INCLUDED_RegionRegistry_h_
#define INCLUDED_RegionRegistry_h_

#ifdef PLATFORM_APERIOS
#  warning RegionRegistry is not Aperios compatable
#else

#include "ListMemBuf.h"
#include "RCRegion.h"
#include "Shared/MarkScope.h"
#include "ProcessID.h"

//! Keeps track of currently available shared memory regions
template<unsigned int MAX_REGIONS, unsigned int NAME_LEN>
class RegionRegistry {
protected:
	//! makes sure we only have one registration in progress at a time
	mutable MutexLock<ProcessID::NumProcesses> lock;
	
	//! Holds information regarding a shared memory region available for listening
	struct entry {
		//! constructor, provides an empty name
		entry() : id() { name[0]='\0'; }
		//! constructor, pass name and pointer to region
		entry(const char n[], RCRegion* r) : id(r->ID()) {
			strncpy(name,n,NAME_LEN);
			name[NAME_LEN]='\0';
		}
		char name[NAME_LEN+1]; //!< the name for the region
		RCRegion::Identifier id; //!< stores information needed to identify the region to the system
	};
	typedef ListMemBuf<entry,MAX_REGIONS> registry_t;  //!< type of the collection managing the regions
	registry_t avail; //!< collection of available memory regions

public:
	static const unsigned int CAPACITY=MAX_REGIONS; //!< allows access to template parameters
	static const unsigned int REGION_NAME_LEN=NAME_LEN; //!< allows access to template parameters
	typedef typename registry_t::index_t index_t; //!< shorthand for the index type used to reference regions

	//! constructor
	RegionRegistry() : lock(), avail() {}
	
	//! destructor
	~RegionRegistry() {
		MarkScope autolock(lock);
		avail.clear();
	}
	
	//! Searches for the region specified by @a name, returns end() if not found
	index_t findRegion(const std::string& name) const {
		MarkScope autolock(lock);
		if(name.size()>NAME_LEN)
			std::cerr << "WARNING: RegionRegistry::attach("<<name<<") is too long, max is " << NAME_LEN << std::endl;
		for(index_t it=begin(); it!=end(); it=next(it))
			if(name==avail[it].name)
				return it;
		return avail.end();
	}

	//! Registers a region, returns the index of the region if successful, or end() if out of space or conflicting registration is found.
	/*! You can re-register the same region under the same name, it simply returns the same index again. */
	index_t registerRegion(const std::string& name, const RCRegion * region) {
		MarkScope autolock(lock,ProcessID::getID());
		index_t it=findRegion(name);
		if(it!=end()) { //found, already registered
			if(avail[it].regions[ProcessID::getID()]==region)
				return it; //same one, just return it
			return end(); // conflict, return invalid
		}
		//not found, make a new one
		return avail.push_back(entry(name.c_str(),region));
	}
	
	//! Creates and registers a new region of the specified size — if the region is already registered, the previous one is returned.
	/*! No checking is done on the size of the region... if it was already registered as a different size, that is ignored. */
	RCRegion * registerRegion(const std::string& name, size_t size) {
		MarkScope autolock(lock);
		index_t it=findRegion(name);
		if(it!=end()) {
			//found, already registered
			return RCRegion::attach(avail[it].id);
		} else {
			//not found:
			RCRegion * region = new RCRegion(name,size);
			avail.push_back(entry(name.c_str(),region));
			return region;
		}
	}
	
	//! Access a region by index
	RCRegion * operator[](index_t it) const {
		MarkScope autolock(lock,ProcessID::getID());
		if(it==end())
			return NULL;
		return RCRegion::attach(avail[it].id);
	}
	
	//! Erases the registration, but doesn't dereference the region (the registry doesn't claim references on the regions...)
	void erase(index_t it) {
		MarkScope autolock(lock,ProcessID::getID());
		avail.erase(it);
	}
	
	index_t begin() const { return avail.begin(); } //!< provides iteration through the entries, be sure to use next(), the index_t is not a 'proper' iterator (no ++ to increment)
	index_t next(index_t it) const { return avail.next(it); } //!< provides the index of the next entry
	index_t end() const { return avail.end(); } //!< one-past-end marker
};

/*! @file
 * @brief Defines RegionRegistry, which keeps track of currently available shared memory regions
 * @author ejt (Creator)
 */

#endif //Aperios check

#endif //INCLUDED

