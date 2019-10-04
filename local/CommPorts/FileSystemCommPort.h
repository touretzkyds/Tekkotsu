//-*-c++-*-
#ifndef INCLUDED_FileSystemCommPort_h_
#define INCLUDED_FileSystemCommPort_h_

#include "local/CommPort.h"
#include "Shared/plist.h"
#include <Wireless/netstream.h>
#include <ios>

//! Provides CommPort interface to file system devices, such as serial ports
/*! Pass a path to use a file system device, or see NetworkCommPort for a network interface */
class FileSystemCommPort : public CommPort, public virtual plist::PrimitiveListener {
public:
	//! constructor (see also sub-class constructor in protected section)
	explicit FileSystemCommPort(const std::string& name)
		: CommPort(autoRegisterFileSystemCommPort,name),
		path(), mode(std::ios_base::in | std::ios_base::out | std::ios_base::trunc | std::ios_base::binary),
		rbuf(), wbuf(), curloc(), curmode(), openedCnt(0)
	{
		addEntry("Path",path,"Path of file system object being accessed");
		addEntry("Mode",mode,"Mode bitmask to pass to the open() call, defaults to 'w+b': in|out|trunc|binary (see std::ios_base::openmode)");
	}
	
	//! destructor, checks that the file descriptor has already been closed
	virtual ~FileSystemCommPort() {
		lock.lock();
		if(openedCnt>0)
			connectionError("File descriptor still open in FileSystemCommPort destructor",true,NULL,false);
	}
	
	virtual std::string getClassName() const { return autoRegisterFileSystemCommPort; }
	
	virtual basic_netbuf<std::ios::char_type>& getReadStreambuf() { return rbuf; }
	virtual basic_netbuf<std::ios::char_type>& getWriteStreambuf() { return wbuf; }
	virtual bool isReadable() { return rbuf.is_open(); }
	virtual bool isWriteable() { return wbuf.is_open(); }
	
	//! tries to have #rbuf and/or #wbuf open #path, subject to #mode 
	virtual bool open();
	
	//! closes #rbuf and #wbuf
	virtual bool close();
	
	//! watches #path, triggers a close() and re-open() if it changes
	virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	
	plist::Primitive<std::string> path; //!< path of file system object being accessed
	plist::Primitive<int> mode; //!< mode bitmask to pass to the open() call (see std::ios_base::openmode)
	
protected:
	explicit FileSystemCommPort(const std::string& classname, const std::string& instancename)
		: CommPort(classname,instancename), 
		path(), mode(std::ios_base::in | std::ios_base::out | std::ios_base::trunc | std::ios_base::binary),
		rbuf(), wbuf(), curloc(), curmode(), openedCnt(0)
	{
		addEntry("Path",path,"Path of file system object being accessed");
		addEntry("Mode",mode,"Mode bitmask to pass to the open() call, defaults to 'w+b': in|out|trunc|binary (see std::ios_base::openmode)");
	}
	
	//! Displays message on stderr and if @a fatal is set, calls closeFD()
	/*! Need the useClassName parameter because we can't trust autoRegisterFileSystemCommPort to
	 *  still be valid during static destruction, so that's false when called from destructor */
	virtual void connectionError(const std::string& msg, bool fatal, const char* sysmsg, bool useClassName=true) {
		if(useClassName && getClassName()!=instanceName)
			std::cerr << getClassName() << " \"" << instanceName << "\": ";
		else
			std::cerr << instanceName << ": ";
		std::cerr << msg;
		if(sysmsg!=NULL && sysmsg[0]!='\0')
			std::cerr << ": " << sysmsg;
		std::cerr << std::endl;
		if(fatal && (rbuf.is_open() || wbuf.is_open())) {
			openedCnt=1;
			close();
		}
	}
	
	basic_netbuf<std::ios::char_type> rbuf;
	basic_netbuf<std::ios::char_type> wbuf;
	std::string curloc;
	std::ios_base::openmode curmode;
	unsigned int openedCnt;
	
	//! holds the class name, set via registration with the CommPort registry
	static const std::string autoRegisterFileSystemCommPort;
};

/*! @file
 * @brief Describes FileDescriptorMotionHook, which provides a file descriptor interface for hardware "drivers"
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
