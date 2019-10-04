//-*-c++-*-
#ifndef INCLUDED_ResourceAccessor_h_
#define INCLUDED_ResourceAccessor_h_

#include "MarkScope.h"

//! A variation of MarkScope which allows you to forward access to the resource via the '->' operator, smart-pointer style
template <class R>
class ResourceAccessor : public MarkScope {
public:
	//! constructor, for marking resources which require no data
	ResourceAccessor(R& r) : MarkScope(r) {}
	//! constructor, accepts data parameter to pass to Resource::useResource()
	ResourceAccessor(R& r, Resource::Data& d) : MarkScope(r,d) {}
	//! copy constructor, marks resource used with default (empty) data
	ResourceAccessor(const ResourceAccessor& ra) : MarkScope(ra) {}
	//! copy constructor, accepts additional data parameter to pass to Resource::useResource()
	ResourceAccessor(const ResourceAccessor& ra, Resource::Data& d) : MarkScope(ra,d) {}
	
#if !defined(__GNUC__) || __GNUC__ > 3 || (__GNUC__ == 3 && (__GNUC_MINOR__ > 3))
	//! returns #rsrc cast as R
	inline R& accessResource() const __attribute__ ((warn_unused_result)) {
		return dynamic_cast<R&>(rsrc);
	}
#else
	//! returns #rsrc cast as R
	inline R& accessResource() const {
		return dynamic_cast<R&>(rsrc);
	}
#endif
	
	R* operator->() { return &accessResource(); } //!< smart pointer to the underlying Resource
	const R* operator->() const { return &accessResource(); } //!< smart pointer to the underlying Resource
	R& operator*() { return accessResource(); } //!< smart pointer to the underlying Resource
	const R& operator*() const { return accessResource(); } //!< smart pointer to the underlying Resource
	R& operator[](int i) { return (&accessResource())[i]; } //!< smart pointer to the underlying Resource
	const R& operator[](int i) const { return (&accessResource())[i]; } //!< smart pointer to the underlying Resource
};

/*! @file
 * @brief Describes ResourceAccessor, a variation of MarkScope which allows you to forward access to the resource via the '->' operator, smart-pointer style
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
