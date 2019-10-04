//-*-c++-*-
#ifndef INCLUDED_RedirectionCommPort_h_
#define INCLUDED_RedirectionCommPort_h_

#include "local/CommPort.h"

//! Allows you to recombine the input/output of other CommPorts in various ways
/*! This will take input from one CommPort, and can send output to a different CommPort.
 *  It's not a 'pipe' from the input to the output, it just changes where the inputs and outputs
 *  are coming from. */
class RedirectionCommPort : public CommPort, public virtual plist::PrimitiveListener {
public:
	//! constructor
	explicit RedirectionCommPort(const std::string& name)
	: CommPort(autoRegisterRedirectionCommPort,name), input(), output(), curin(), curout(), openedCnt(0)
	{
		addEntry("Input",input,"Name of comm port from which to get input");
		addEntry("Output",output,"Name of comm port into which to send output");
	}

	//! destructor, checks that the file descriptor has already been closed
	virtual ~RedirectionCommPort() {
		if(openedCnt>0)
			std::cerr << "Still open in RedirectionCommPort destructor" << std::endl;
	}

	virtual std::string getClassName() const { return autoRegisterRedirectionCommPort; }

	virtual streambuf& getReadStreambuf() { CommPort * cp=getInputCP(); return (cp!=NULL) ? cp->getReadStreambuf() : invalid; }
	virtual streambuf& getWriteStreambuf() { CommPort * cp=getOutputCP(); return (cp!=NULL) ? cp->getWriteStreambuf() : invalid; }
	virtual bool isReadable() { CommPort * cp=getInputCP(); return (cp!=NULL) ? cp->isReadable() : false; }
	virtual bool isWriteable() { CommPort * cp=getOutputCP(); return (cp!=NULL) ? cp->isWriteable() : false; }

	virtual bool open();
	virtual bool close();

	//! watches #input and #output, triggers a close() and re-open() as needed
	virtual void plistValueChanged(const plist::PrimitiveBase& pl);

	plist::Primitive<std::string> input; //!< Name of comm port from which to get input
	plist::Primitive<std::string> output; //!< Name of comm port into which to send output

protected:
	//! convenience function to lookup the current input comm port instance
	CommPort * getInputCP() { return (curin.size()==0) ? NULL : CommPort::getRegistry().getInstance(curin); }
	//! convenience function to lookup the current output comm port instance
	CommPort * getOutputCP() { return (curout.size()==0) ? NULL : CommPort::getRegistry().getInstance(curout); }
	
	std::string curin; //!< name of the current input comm port (stored separately from #input so if that changes, we can close the old one)
	std::string curout; //!< name of the current output comm port (stored separately from #output so if that changes, we can close the old one)
	unsigned int openedCnt; //!< open() call depth, tracks multiple usage
	
	//! std::streambuf uses a protected constructor, so we have to inherit to make an empty streambuf for #invalid
	class invalid_streambuf : public std::streambuf {};
	//! needed so we have something to return when the RedirectionCommPort doesn't have a valid external comm port to take a streambuf from
	static invalid_streambuf invalid;
	
	//! holds the class name, set via registration with the CommPort registry
	static const std::string autoRegisterRedirectionCommPort;
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
