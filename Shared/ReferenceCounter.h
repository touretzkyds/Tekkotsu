//-*-c++-*-
#ifndef INCLUDED_ReferenceCounter_h_
#define INCLUDED_ReferenceCounter_h_

#include <iostream>
#include <stdexcept>

//! Performs simple reference counting, will delete the object when removing the last reference
/*! Remember to call setAutoDelete(false) if you
 *  instantiate a subclass on the stack instead of the heap -- don't want to try to free memory
 *  on the stack if/when last reference is removed!  (The stack limits the allocation of the behavior
 *  to the current scope, so reference counting is moot.)
 *
 *  @todo It would be nice if there was a way for ReferenceCounter to automatically know whether it
 *  has been allocated on the stack... is that possible?
 */
class ReferenceCounter {
 public:
	//! constructor
	ReferenceCounter() : references(0),RC_autodelete(true) {}
	//! copy constructor - uses autodelete setting of @a rc, but references will still start at 0
	ReferenceCounter(const ReferenceCounter& rc) : references(0),RC_autodelete(rc.RC_autodelete) {}
	//! assignment operator - does nothing because the reference count shouldn't be copied
	ReferenceCounter& operator=(const ReferenceCounter& /*rc*/) {return *this;}

	//! destructor - will std::cout a warning if still has references
	virtual ~ReferenceCounter() {
		if(references>0)
			std::cout << "*** WARNING ReferenceCounter was deleted with " << references << " references" << std::endl;
	}

	//! adds one to #references
	virtual void addReference() { ++references; }
	//! subtracts one from #references AND DELETES the object IF ZERO (and RC_autodelete is set)
	/*! @return true if the last reference was removed, false otherwise */
	virtual bool removeReference() {
		if(--references==0) {
			if(RC_autodelete)
				delete this;
			return true;
		} else if(references==(unsigned int)-1) {
			//std::cout << "*** WARNING ReferenceCounter went negative" << std::endl;
			throw std::underflow_error("ReferenceCounter went negative");
		} else
			return false;
	}
	//! returns the number of references
	/*! @return references */
	virtual unsigned int getReferences() const { return references; }

	//! if true, next time a removeReference() causes #references to hit 0, the object will delete itself
	void setAutoDelete(bool b) {RC_autodelete=b;}

	bool getAutoDelete() { return RC_autodelete; } //!< returns RC_autodelete
	
 protected:
	//! the current number of references
	unsigned int references;

	//! if false, prevents deletion when counter hits 0 (needed in case of stack allocation or if the subclass is a direct member variable of another class)
	bool RC_autodelete;
};

/*
#include "Behaviors/BehaviorBase.h"

const char* findname(ReferenceCounter* x) {
	BehaviorBase* beh=dynamic_cast<BehaviorBase*>(x);
	if(beh==NULL) {
		static char s[100];
		sprintf(s,"Uknown @ %x",x);
		return s;
	} else {
		static char s2[100];
		sprintf(s2," @ %x",x);
		return (beh->getName()+s2).c_str();
	}
}

*/

/*! @file
 * @brief Defines the ReferenceCounter base class, which allows for automatic memory deallocation
 * @author ejt (Creator)
 */

#endif

