//-*-c++-*-
#ifndef INCLUDED_Resource_h_
#define INCLUDED_Resource_h_

//! Provides a generic interface for resources which need to keep track of when they are in use, such as mutex locks
class Resource {
public:
	//! base class for holding data required for requesting to use/release the resource
	class Data {
	public:
		virtual ~Data() {} //!< empty virtual destructor to mark this as a base class
	};
	static Data emptyData; //!< to use as the data reference when none is needed/specified
	
	//! destructor (does nothing -- up to subclass if they need to release resource)
	virtual ~Resource() {}
	
	virtual void useResource(Data& d)=0; //!< marks the resource as in use
	virtual void releaseResource(Data& d)=0; //!< releases the resource
};

//! a no-op resource, since we don't want Resource itself to be directly instantiatable
class EmptyResource : public Resource {
public:	
	virtual void useResource(Data&) {} //!< would mark the resource in use, here is a no-op
	virtual void releaseResource(Data&) {} //!< would mark the resource no longer in use, here is a no-op
};
extern EmptyResource emptyResource; //!< a global instance of the empty resource, for no-op situations

/*! @file
 * @brief Describes Resource (and EmptyResource), which provides a generic interface for resources which need to keep track of when they are in use, such as mutex locks
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
