//-*-c++-*-
#ifndef INCLUDED_NetworkCommPort_h_
#define INCLUDED_NetworkCommPort_h_

#include "local/CommPort.h"
#include "Wireless/netstream.h"
#include "Shared/Config.h"
#include "Shared/TimeET.h"
#include "IPC/CallbackThread.h"

//! Interfaces to a machine over the internet, TCP/IP or UDP/IP.
/*! You probably want to use UDP if you're sending high-frequency, full-state updates, and
*  use TCP if you're sending low-frequency or partial-state updates. */
class NetworkCommPort : public CommPort, public virtual plist::PrimitiveListener {
public:
	explicit NetworkCommPort(const std::string& name)
		: CommPort(autoRegisterNetworkCommPort,name),
		host(), port(), transport(Config::TCP,Config::transport_names), server(false), verbose(false), block(false),
		sbuf(), curaddr(), curtrans(Config::TCP), openedCnt(0), recon(&NetworkCommPort::keepOpen,*this)
	{
		sbuf.setEcho(verbose);
		addEntry("Host",host,"Hostname to connect to, or interface to listen on (blank for INADDR_ANY)");
		addEntry("Port",port,"Port number to connect to or listen on");
		addEntry("Transport",transport,"Transport protocol to use");
		addEntry("Server",server,"If true, should listen for incoming connections instead of making an outgoing one.");
		addEntry("Verbose",verbose,"If true, all traffic will be echoed to the terminal (handy for debugging plain-text protocols)");
		addEntry("Block",block,"If true, will block until connection is (re)established on initial open or after losing the connection.");
		verbose.addPrimitiveListener(this);
	}
	
	//! destructor, checks that #sbuf has already been closed
	virtual ~NetworkCommPort() {
		if(openedCnt>0)
			connectionError("Connection still open in NetworkCommPort destructor",true);
	}
	
	virtual std::string getClassName() const { return autoRegisterNetworkCommPort; }
	
	virtual streambuf& getReadStreambuf() { return sbuf; }
	virtual streambuf& getWriteStreambuf() { return sbuf; }
	virtual bool isWriteable() { return isReadable(); }
	virtual bool isReadable();
	
	//! activates the #sbuf based on the current configuration settings
	virtual bool open();
	
	//! closes #sbuf
	virtual bool close();
	
	virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	
	plist::Primitive<std::string> host;
	plist::Primitive<unsigned short> port;
	plist::NamedEnumeration<Config::transports> transport;
	plist::Primitive<bool> server;
	plist::Primitive<bool> verbose;
	plist::Primitive<bool> block;
	
protected:
	//! Displays message on stderr and if @a fatal is set, calls closeFD()
	virtual void connectionError(const std::string& msg, bool fatal) {
		std::cerr << msg << std::endl;
		if(fatal && sbuf.is_open())
			close();
	}
	
	//! attempts to make a connection, checking that the previous attempt wasn't too recent
	virtual bool doOpen(bool dispError);
	
	virtual void keepOpen();
		
	basic_netbuf<std::ios::char_type> sbuf;
	IPaddr curaddr;
	Config::transports curtrans;
	unsigned int openedCnt;
	CallbackThread recon;
	
	//! holds the class name, set via registration with the CommPort registry
	static const std::string autoRegisterNetworkCommPort;
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
