//-*-c++-*-
#ifndef INCLUDED_Wireless_h_
#define INCLUDED_Wireless_h_

#ifdef PLATFORM_APERIOS
#  include <OPENR/OObject.h>
#  include <OPENR/OSubject.h>
#  include <OPENR/OObserver.h>
#  include <ant.h>
#else
#  include "IPC/Thread.h"
#  include "Shared/Resource.h"
#  include <stdint.h>
typedef uint32_t uint32;
#endif
#include "Socket.h"
#include "DummySocket.h"
#include <list>

class SocketListener;

//! Tekkotsu wireless class
/*!
 * For more information on using wireless, please read the following tutorials:
 * - <a href="../TekkotsuMon.html">TekkotsuMon</a>
 * - <a href="../Wireless.html">TCP/IP</a>
 *
 * The networking interface needs more documentation.  It also needs a
 * cleanup.  In the mean time, take a look at the TekkotsuMon objects
 * in <i>Tekkotsu</i><tt>/Behaviors/Mon</tt>.  They all listen for new
 * connections.  Unfortunately, at the momement there are no examples
 * of outgoing connections, but it should give you a pretty good idea
 * how to start moving.
 */
class Wireless {
public:
	//! Maximum number of sockets which can be created
	static const int WIRELESS_MAX_SOCKETS=100;

	//! Default number of bytes to use for receive buffers (overridden by value passed to socket())
	static const int WIRELESS_DEF_RECV_SIZE=1024;

	//! Default number of bytes to use for send buffers (overridden by value passed to socket())
	static const int WIRELESS_DEF_SEND_SIZE=1024;

	//! constructor - only one wireless object is required per Aperios process.
	/*! MMCombo already creates one. The (global) instance is called wireless,
	 * and you can access it by including Wireless/Wireless.h (this file) in
	 * your code
	 */
	Wireless();
	~Wireless(); //!< destructor

	//@{
	//! Creates a new socket
	/*! @return pointer to Socket object created
	 * @param ttype selects between TCP and UDP
	 * @see WIRELESS_DEF_RECV_SIZE, WIRELESS_DEF_SEND_SIZE */
	Socket* socket(Socket::TransportType_t ttype);
	/*!@param ttype selects between TCP and UDP
	 * @param recvsize size of input buffer
	 * @param sendsize size of output buffer
	 */
	Socket* socket(Socket::TransportType_t ttype, int recvsize, int sendsize);
	//@}

	//! The socket waits for incoming connections.
	/*! That is, it acts like a server. If a connection is established and
	 * later broken, it resumes waiting for new connections if the
	 * socket's daemon flag is set.
	 */
	int listen(int sock, int port);

	//! The socket tries to connect to a specific
	int connect(int sock, const char* ipaddr, int port);

	//! sets receiver callback for a socket
	void setReceiver(int sock, int (*rcvcbckfn) (char*, int) );

	//! sets receiver callback for a socket, this version requiring the SocketListener interface (more powerful, as this lets us tell connections apart)
	void setReceiver(int sock, SocketListener *listener);

	//! sets the socket to be a daemon (recycles on close)
	void setDaemon(int sock, bool val=true) { sockets[sock]->daemon=val; }
	//! sets the socket to be a daemon (recycles on close)
	bool getDaemon(int sock) { return sockets[sock]->daemon; }
	//! closes and destroys non server, daemon sockets
	void close(int sock);

	//@{
	//! utility function that you can use if you're curious about the state of the socket.
	/*! You shouldn't need to use it, since asking sockets for write
	 * and read buffers does the necessary sanity checks
	 */
	bool isConnected(int sock) {
		return sockets[sock]==NULL ? false : sockets[sock]->state==Socket::CONNECTION_CONNECTED;
	}
	bool isError(int sock) {
		return sockets[sock]==NULL ? false : sockets[sock]->state==Socket::CONNECTION_ERROR;
	}

	bool isReady(int sock) { return !sockets[sock]->tx; }
	bool hasData(int sock) { return !sockets[sock]->rx; }
	//@}

	//@{
	//! helper function for the function with the same name that takes a socket descriptor (int)
	void setReceiver(Socket &sobj, int (*rcvcbckfn) (char*, int) )
    { setReceiver(sobj.sock, rcvcbckfn); }
	void setReceiver(Socket *sobj, int (*rcvcbckfn) (char*, int) )
    { setReceiver(sobj->sock, rcvcbckfn); }
	void setReceiver(Socket &sobj, SocketListener *listener)
	{ setReceiver(sobj.sock, listener); }
	void setReceiver(Socket *sobj, SocketListener *listener)
	{ setReceiver(sobj->sock, listener); }
	void setDaemon(Socket &sobj, bool val=true) { setDaemon(sobj.sock, val); }
	void setDaemon(Socket *sobj, bool val=true) { setDaemon(sobj->sock, val); }
	bool getDaemon(Socket &sobj) { return getDaemon(sobj.sock); }
	bool getDaemon(Socket *sobj) { return getDaemon(sobj->sock); }
	int listen(Socket &sobj, int port) { return listen(sobj.sock, port); }
	int listen(Socket *sobj, int port) { return listen(sobj->sock, port); }
	int connect(Socket &sobj, const char* ipaddr, int port)
    { return connect (sobj.sock, ipaddr, port); }
	int connect(Socket *sobj, const char* ipaddr, int port)
    { return connect (sobj->sock, ipaddr, port); }
	void close(Socket &sobj) { close(sobj.sock); }
	void close(Socket *sobj) { close(sobj->sock); }
	unsigned int getNumInterfaces() { return 1; }
	uint32 getIPAddress(unsigned int idx=0);
	uint32 getIFAddress(const char*);
	//@}

	//@{
	//! function for internal and Socket use. You should not call this
	void receive(int sock, int (*rcvcbckfn) (char*, int) );
	void receive(int sock);
	//@}

	//@{
	//! function called by the Socket objects to actually write
	//! data to the network. You should not call this.
	void send(int sock);
	void blockingSend(int sock);
	//@}

#ifdef PLATFORM_APERIOS
	//@{
	//! callback function for communicating
	//! with Aperios Networking Toolkit. You should not call this.
	void ListenCont (void* msg);
	void BindCont   (void* msg);
	void ConnectCont(void* msg);
	void SendCont   (void* msg);
	void ReceiveCont(void* msg);
	void CloseCont  (void* msg);
	//@}

#else
	void pollSetup(); //!< on non-aperios, set up structures to be checked in pollTest()
	bool pollTest(struct timeval* tv); //!< on non-aperios, check to see any network communication has occurred
	void pollProcess(); //!< on non-aperios, process callbacks and state changes as signaled in pollTest()
	void wakeup(Socket * del=NULL); //!< writes @a del on #interruptCtl, breaking out of a pending pollTest() and thus giving an opportunity to change the contents of the FD sets being used;

	void setCallbackLock(Resource& l); //!< sets #callbackLock
	void clearCallbackLock(); //!< resets #callbackLock to a self-defined lock, which you can request from getCallbackLock() (there's always a callbackLock, the only question is it internally or externally instantiated)
	Resource& getCallbackLock() const { static Thread::Lock cl; return callbackLock==NULL ? cl : *callbackLock; } //!< returns #callbackLock
#endif

protected:
	friend class Socket; //so socket can lock as well
	static const int MAXCONNECTIONS = 5; //!< the maximum number of connections which can be queued when listening

	//@{
	//!private ALOKL_TODO
#ifdef PLATFORM_APERIOS
	antStackRef ipstackRef;
	OID myOID;
#else
	static Resource& getLock(); //!< returns the lock to use during @e all wireless operations (not just callbacks, this is more general)
	Resource* callbackLock; //!< this lock will be aquired during any callbacks which might occur during pollProcess()
	int interruptChk; //!< a socket, connected to #interruptCtl, which allows pollTest() to be interrupted if new sockets need to be polled
	int interruptCtl; //!< a socket, connected to #interruptChk, which allows pollTest() to be interrupted if new sockets need to be polled
	fd_set rfds; //!< a set of file descriptors which should be polled for readable data; set up by pollSetup(), watched (blocking) by pollTest(), and processed by pollProcess()
	fd_set wfds; //!< a set of file descriptors which should be polled for write-complete; set up by pollSetup(), watched (blocking) by pollTest(), and processed by pollProcess()
	fd_set efds; //!< a set of file descriptors which should be polled for errors; set up by pollSetup(), watched (blocking) by pollTest(), and processed by pollProcess()
	int fdsMax; //!< maximum file descriptor value in the #rfds, #wfds, #efds fd_set's
#endif
	Socket* sockets[WIRELESS_MAX_SOCKETS];
	std::list<int> freeSockets;
	std::list<int> usedSockets;
	bool usedSocketsInvalidated; //!< set to true at modifcation of #usedSockets, cleared prior to callbacks so we can tell if callback invalidates active iterators
	//@}

private:
	Wireless(const Wireless&); //!< don't call
	Wireless& operator= (const Wireless&); //!< don't call
};

//! the global wireless object - you'll want to make your function calls on this
extern Wireless* wireless;

/*! @file
 * @brief Interacts with the system to provide networking services
 * @author alokl (Creator)
 *
 * @verbinclude CMPack_license.txt
 */

#endif // Wireless_h_DEFINED
