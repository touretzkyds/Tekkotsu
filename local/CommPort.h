//-*-c++-*-
#ifndef INCLUDED_CommPort_h_
#define INCLUDED_CommPort_h_

#include "Shared/plistCollections.h"
#include "Shared/InstanceTracker.h"
#include "Shared/Resource.h"
#include "IPC/Thread.h"
#include <string>
#include <streambuf>
#include <ios>

//! A CommPort provides an abstract interface to a communication resource, based on wrapping a standard library stream buffer
/*! Key extensions provided beyond the std::basic_streambuf are a mutual exclusion lock,
 *  explicitly separate read/write buffers (which, depending on implementation, could refer
 *  to the same stream), recursive open/close, plist-based configuration parameters, and
 *  integration with an InstanceTracker registry and factory for dynamic reconfigurability.
 *
 *  Usually you can get by using one of the standard stream buffers (streambuf, filebuf, stringbuf)
 *  but if you need to implement a custom stream, these links may help
 *  get you started:
 *  - An article by Paul Grenyer stepping through the process:
 *    http://accu.org/index.php/journals/264
 *  - Sample code from Chris Frey for a network class:
 *    http://www.netdirect.ca/~cdfrey/software/sockstream.cc.txt
 * 
 *  See also our own network stream class, ionetstream (Wireless/netstream.h/.cc).
 *  Although intended for networking, you can pass it any file descriptor, which makes
 *  it handy for pipes as well.
 *
 *  Clients should be careful to use the locking mechanism if there is a possibility of
 *  confusing query-responses or competing command/query-polls!
 *
 * Example usage: look up an instance named "Foo", and send it a query.
 * @code
 * CommPort* comm = CommPort::getRegistry().getInstance("Foo");
 * std::ostream is(&comm->getReadStreambuf());
 * std::ostream os(&comm->getWriteStreambuf());
 * is.tie(&os); // fancy failsafe -- make sure 'os' is flushed anytime we read from 'is'
 *
 * // Locking a CommPort across query-response pairs:
 * int value;
 * {
 *     MarkScope autolock(*comm); // marks the comm port as "in use" until the end of its scope
 *     os << "query-value-command" << endl;
 *     is >> value;
 *     // because we have the lock, we know 'value' is in response
 *     // to the 'query-value-command', and not a response to any other thread
 * }
 * @endcode
 *
 * Advanced locking: try to get a lock, then transfer it to a MarkScope to
 * ensure exception-safety.
 * @code
 * Thread::Lock& l = comm->getLock();
 * if(l.trylock()) {
 *     MarkScope autolock(l); l.unlock(); // transfer lock to MarkScope
 *     // use comm ...
 * }
 * @endcode
 */
class CommPort : public virtual plist::Dictionary, public Resource {
public:
	//! destructor, removes from registry in case we're deleting it from some other source than registry's own destroy()
	virtual ~CommPort() { getRegistry().destroy(instanceName); }
	
	//! the streambuf which does the actual work should inherit from basic_streambuf, using the system's default character type
	typedef std::basic_streambuf<std::ios::char_type> streambuf;
	
	//! Returns the name of the class (aka its type)
	/*! Suggested implementation is to declare a static string member, set it to the result of
	 *  calling the registry's registerType, and then return that member here */
	virtual std::string getClassName() const=0;
	
	//! Provides serialized access to the comm port
	/*! Multiple drivers might be using the same comm port, callers should get the
	 *  lock when doing operations on the comm port, particularly across sending
	 *  a command and waiting for the reply.  See MarkScope for usage. */
	virtual Thread::Lock& getLock() { return lock; }
	
	//! Called when communication is about to begin, should handle recursive open/close calls
	/*! The subclass is expected to have its own configuration settings
	 *  which define the parameters of what is to be "opened".
	 *  Hence, no arguments are passed.
	 *
	 *  You should be able to handle recursive levels of open/close in case multiple
	 *  drivers are using the same CommPort.
	 *
	 *  @return true if successful (or already open) */
	virtual bool open()=0;
	
	//! Called when communication is complete, should handle recursive open/close calls
	/*! @return true if successful, false if still open (in use elsewhere) */
	virtual bool close()=0;
	
	//! Allows you to check whether the reference from getReadStreambuf() is currently functional (if checking is supported!)
	/*! For streambufs which don't have a way to check this, always returns true. */
	virtual bool isReadable() { return true; }
	
	//! Allows you to check whether the reference from getWriteStreambuf() is currently functional (if checking is supported!)
	/*! For streambufs which don't have a way to check this, always returns true. */
	virtual bool isWriteable() { return true; }
	
	//! Returns a std::basic_streambuf, which is expected to implement the actual work
	/*! You can pass this to an istream to use the nice C++ style input and output,
	 *  or you can call the streambuf functions directly.  However, if you're going
	 *  the latter route, probably easier to just call CommPort's own read() and write().
	 *
	 *  Depending on implementation, the streambuf this returns might be a
	 *  different instance than getWriteStreambuf.  If they are the same instance,
	 *  then you could use an iostream instead of separate istream and ostream.*/
	virtual streambuf& getReadStreambuf()=0;
	
	//! Returns a std::basic_streambuf, which is expected to implement the actual work
	/*! You can pass this to an ostream to use the nice C++ style input and output,
	 *  or you can call the streambuf functions directly.  However, if you're going
	 *  the latter route, probably easier to just call CommPort's own read() and write().
	 *
	 *  Depending on implementation, the streambuf this returns might be a
	 *  different instance than getReadStreambuf.  If they are the same instance,
	 *  then you could use an iostream instead of separate istream and ostream.*/
	virtual streambuf& getWriteStreambuf()=0;
	
	//! returns up to @a n bytes from the streambuf, returns the number read
	virtual size_t read(char* buf, size_t n) { return getReadStreambuf().sgetn(buf,n); }
	//! writes up to @a n bytes from the streambuf, returns the number written
	virtual size_t write(const char* buf, size_t n) { return getWriteStreambuf().sputn(buf,n); }
	
	//! reads all available data from getReadStreambuf()
	virtual void read(std::string& s) {
		s.clear();
		const size_t BUFSIZE=256;
		char buf[BUFSIZE];
		size_t nread=read(buf,BUFSIZE);
		while(nread!=0) {
			s.append(buf,nread);
			if(nread!=BUFSIZE || getReadStreambuf().in_avail()<=0)
				break;
			nread=read(buf,BUFSIZE);
		}
	}
	//! writes the string into getWriteStreambuf()
	virtual size_t write(const std::string& s) { return write(s.c_str(),s.size()); }
	
	//! short hand for the instance tracker, which allows dynamic reconfiguration of CommPort instances
	typedef InstanceTracker<CommPort,std::string,Factory1Arg<CommPort,std::string> > registry_t;
	//! registry from which current instances can be discovered and new instances allocated based on their class names
	static registry_t& getRegistry() { static registry_t registry; return registry; }
	
protected:
	//! constructor, pass the name of the class's type so we can use it in error messages, and a name for the instance so we can register it for MotionHook's to lookup
	CommPort(const std::string& /*classname*/, const std::string& instancename)
	: plist::Dictionary(), Resource(), instanceName(instancename), lock()
	{
		setLoadSavePolicy(FIXED,SYNC);
	}
	
	//! To be called be "deepest" subclass constructor at the end of construction
	/*! Don't want to register until completed construction!  plist::Collection listeners would be
	 *  triggered and might start performing operations on instance while partially constructed */
	virtual void registerInstance() {
		if(CommPort * inst=getRegistry().getInstance(instanceName)) {
			std::cerr << "Warning: registration of CommPort " << getClassName() << " named " << instanceName << " @ " << this
			<< " blocked by previous " << inst->getClassName() << " instance of same name @ " << inst << std::endl;
		}
		if(!getRegistry().registerInstance(getClassName(),instanceName,this))
			std::cerr << "Error: failed to register " << getClassName() << " named " << instanceName << " @ " << this;
		//addEntry(".type",new plist::Primitive<std::string>(className),"Stores the typename of the comm port so it can be re-instantiated on load.\n** Do not edit ** ");
	}
		
	//! Provides a Resource interface, allowing you to use MarkScope directly on the CommPort instead of calling through getLock().
	/*! Users don't need to call this directly... either pass the CommPort to a MarkScope, or call getLock(). */
	virtual void useResource(Data& d) { static_cast<Resource&>(lock).useResource(d); }
	//! provides a Resource interface, allowing you to use MarkScope directly on the CommPort instead of calling through getLock().
	/*! Users don't need to call this directly... either pass the CommPort to a MarkScope, or call getLock(). */
	virtual void releaseResource(Data& d) { static_cast<Resource&>(lock).releaseResource(d); }
	
	virtual void opened() {} //!< should be called by open() once the connection is successfully made, so deeper subclasses can do initialization
	virtual void closing() {} //!< should be called by close() before the connection is closed, so deeper subclasses can do cleanup
	
	const std::string instanceName; //!< holds the name of this instance of CommPort (mainly for error message reporting by the class itself)
	
	//!< ensures that serialized access is maintained (assuming clients use the lock...)
	/*! Often devices have either half-duplex communication, or may give responses to
	 *  command strings.  It is important to get a lock across a query-response pair so that
	 *  there is no risk of a second thread attempting a competing command or query. */
	Thread::Lock lock;
	
private:
	CommPort(const CommPort&); // no copy, don't call
	CommPort& operator=(const CommPort&); // no copy, don't call
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
