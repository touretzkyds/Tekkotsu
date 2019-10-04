//-*-c++-*-
#ifndef INCLUDED_InverseMarkScope_h_
#define INCLUDED_InverseMarkScope_h_

#include "Resource.h"

//! Provides a way to mark a resource as @e unused for the duration of the instance's scope, thus negating MarkScope for a portion
/*! This is handy because we don't have to worry about re-securing the resource
 *  if there are multiple return points, exception handling, or other such issues
 *  which might otherwise cause you to forget to release it -- let C++ do it for you! */
class InverseMarkScope {
public:
	//! constructor, for marking resources which require no data
	InverseMarkScope(Resource& r) : rsrc(r), data(&Resource::emptyData) {
		rsrc.releaseResource(*data);
	}
	//! constructor, accepts data parameter to pass to Resource::releaseResource()
	InverseMarkScope(Resource& r, Resource::Data& d) : rsrc(r), data(&d) {
		rsrc.releaseResource(*data);
	}
	//! copy constructor, marks resource used, copying ms's data reference (better make sure the Resource support recursive usage...)
	InverseMarkScope(const InverseMarkScope& ms) : rsrc(ms.rsrc), data(ms.data) {
		rsrc.releaseResource(*data);
	}
	//! copy constructor, accepts additional data parameter to pass to Resource::releaseResource()
	InverseMarkScope(const InverseMarkScope& ms, Resource::Data& d) : rsrc(ms.rsrc), data(&d) {
		rsrc.releaseResource(*data);
	}
	//! destructor, re-acquires resource
	~InverseMarkScope() {
		rsrc.useResource(*data);
	}
	
	//! accessor to return the resource being marked
	Resource& getResource() const { return rsrc; }
	//! accessor to return the data used to access the resource
	Resource::Data& getData() const { return *data; }
	//! renew the resource usage -- call use and release again, with the same data
	void reset() { rsrc.useResource(*data); rsrc.releaseResource(*data); }
	//! renew the resource usage -- call use and release again with the new data
	void reset(Resource::Data& d) { rsrc.useResource(*data); data=&d; rsrc.releaseResource(*data); }
	
protected:
	Resource& rsrc; //!<the resource we're using
	Resource::Data * data; //!< data passed to resource when using it and releasing it
	
private:
	InverseMarkScope& operator=(const InverseMarkScope&); //!< assignment prohibited (can't reassign the reference we already hold)
};

/*! @file
* @brief Defines InverseMarkScope, which provides a way to mark a resource as @e unused for the duration of the instance's scope (cancels a MarkScope for a portion)
* @author Ethan Tira-Thompson (ejt) (Creator)
*/

#endif
