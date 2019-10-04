//-*-c++-*-
#ifndef INCLUDED_MarkScope_h_
#define INCLUDED_MarkScope_h_

#include "Resource.h"

//! Provides a way to mark a resource as used for the duration of the instance's scope
/*! This is handy because we don't have to worry about releasing the resource
 *  if there are multiple return points, exception handling, or other such issues
 *  which might otherwise cause you to forget to release it -- let C++ do it for you! */
class MarkScope {
public:
	//! constructor, for marking resources which require no data
	MarkScope(Resource& r) : rsrc(r), data(&Resource::emptyData) {
		rsrc.useResource(*data);
	}
	//! constructor, accepts data parameter to pass to Resource::useResource()
	MarkScope(Resource& r, Resource::Data& d) : rsrc(r), data(&d) {
		rsrc.useResource(*data);
	}
	//! copy constructor, marks resource used, copying ms's data reference (better make sure the Resource support recursive usage...)
	MarkScope(const MarkScope& ms) : rsrc(ms.rsrc), data(ms.data) {
		rsrc.useResource(*data);
	}
	//! copy constructor, accepts additional data parameter to pass to Resource::useResource()
	MarkScope(const MarkScope& ms, Resource::Data& d) : rsrc(ms.rsrc), data(&d) {
		rsrc.useResource(*data);
	}
	//! destructor, releases resource
	~MarkScope() {
		rsrc.releaseResource(*data);
	}
	
	//! accessor to return the resource being marked
	Resource& getResource() const { return rsrc; }
	//! accessor to return the data used to access the resource
	Resource::Data& getData() const { return *data; }
	//! renew the resource usage -- call release and use again, with the same data
	void reset() { rsrc.releaseResource(*data); rsrc.useResource(*data); }
	//! renew the resource usage -- call release and use again with the new data
	void reset(Resource::Data& d) { rsrc.releaseResource(*data); data=&d; rsrc.useResource(*data); }
	
protected:
	Resource& rsrc; //!<the resource we're using
	Resource::Data * data; //!< data passed to resource when using it and releasing it
	
private:
	MarkScope& operator=(const MarkScope&); //!< assignment prohibited (can't reassign the reference we already hold)
};

/*! @file
* @brief Defines MarkScope, which provides a way to mark a resource as used for the duration of the instance's scope
* @author Ethan Tira-Thompson (ejt) (Creator)
*/

#endif
