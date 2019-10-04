//-*-c++-*-
#ifndef INCLUDED_EntryPoint_h_
#define INCLUDED_EntryPoint_h_

#include "Shared/Resource.h"
#include "Shared/debuget.h"
#include "Shared/StackTrace.h"
#include "IPC/ProcessID.h"
#include <list>
#include <typeinfo>

//! manages a thread lock to serialize behavior computation and mark whether ::state is being read or written
class EntryPoint : public Resource {
public:
	//! constructor
	explicit EntryPoint() : Resource(), epFrame(NULL), data() {}
	
	virtual void useResource(Data& d);
	virtual void releaseResource(Data& d);
	
protected:
	stacktrace::StackFrame * epFrame; //!< stores root stack frame so ProcessID::getID can tell processes apart through virtual calls on shared memory regions
	std::list<Data*> data; //!< data supplied when resource was marked used
	
private:
	EntryPoint(const EntryPoint&); //!< not implemented
	EntryPoint operator=(const EntryPoint&); //!< not implemented
};


/*! @file
* @brief 
* @author Ethan Tira-Thompson (ejt) (Creator)
*/

#endif
