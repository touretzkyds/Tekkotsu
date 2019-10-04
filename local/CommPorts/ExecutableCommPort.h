//-*-c++-*-
#ifndef INCLUDED_ExecutableCommPort_h_
#define INCLUDED_ExecutableCommPort_h_

#include "local/CommPort.h"
#include "Wireless/netstream.h"

//! Run a specified executable, with the comm port connected to its stdin and stdout
/*! This can be handy for testing with a device simulator, or if you want to
 *  interface with a device as an external process.  It's more efficient to run your
 *  driver as an DeviceDriver subclass than to spawn an external process however.
 *
 *  If you need to keep your external program running across instances of the
 *  simulator (or just want to launch it externally), you'll need to use file system
 *  fifos (see mkfifo command), probably separate ones for reading and writing
 *  with a RedirectionCommPort to combine them (unless your platform supports
 *  bidirectional pipes... most don't) */
class ExecutableCommPort : public CommPort, public virtual plist::PrimitiveListener {
public:
	explicit ExecutableCommPort(const std::string& name)
		: CommPort(autoRegisterExecutableCommPort,name),
		command(), shell("sh"), child(0), rbuf(), wbuf(), openedCnt(0)
	{
		addEntry("Command",command,"Specifies the shell command to run, stdio from this process will be piped through the comm port");
		addEntry("Shell",shell,"The shell executable to use for executing the command, can be found via PATH, or explicit path\n"
						 "The shell will be passed '-c' and then your command");
	}
	
	//! destructor, checks that child process is no longer running, kills it if it is (after some delay)
	virtual ~ExecutableCommPort();
	
	virtual std::string getClassName() const { return autoRegisterExecutableCommPort; }
	
	virtual streambuf& getReadStreambuf() { return rbuf; }
	virtual streambuf& getWriteStreambuf() { return wbuf; }
	virtual bool isWriteable() { wbuf.update_status(); return wbuf.is_open(); }
	virtual bool isReadable() { rbuf.update_status(); return rbuf.is_open(); }
	
	//! launches the executable, connecting its stdin and stdout to this comm port
	virtual bool open();
	
	//! sends kill signal to the child process
	virtual bool close();
	
	//! if #command is modified, close current connection (if running) and launch new one
	virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	
	plist::Primitive<std::string> command; //!< shell command
	plist::Primitive<std::string> shell; //!< shell name
	
protected:
	pid_t child; //!< process ID of the child (0 if not launched, -1 if error)
	
	basic_netbuf<std::ios::char_type> rbuf; //!< reads from child process
	basic_netbuf<std::ios::char_type> wbuf; //!< writes to child process
	
	unsigned int openedCnt; //!< reference count of the number of times we've been opened (i.e. pending close()s)
	
	enum {
		READPIPE=0, //!< pipe() system call returns read end on the first entry, lets make it symbolic so it's more clear
		WRITEPIPE=1 //!< pipe() system call returns write end on the second entry, lets make it symbolic so it's more clear
	};
	
	//! waits for child process to exit for t milliseconds, polling status every p millseconds, returns true if no longer running
	bool waitChild(unsigned int t, unsigned int p);
	
	//! returns true if child is still running
	bool isChildRunning() const;

	//! holds the class name, set via registration with the CommPort registry
	static const std::string autoRegisterExecutableCommPort;
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
