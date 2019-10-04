//-*-c++-*-
#ifndef INCLUDED_Socket_h_
#define INCLUDED_Socket_h_

#ifdef PLATFORM_APERIOS
#  include <ant.h>
#  include <Types.h>
#else
#  include <sys/types.h>
#  include <sys/socket.h>
typedef unsigned char byte;
#endif
#include <stdarg.h>
#include <stdlib.h>
#include <string>

class SocketListener;

// Sigh... I hate when people define constants via macros...
#ifdef PLATFORM_APERIOS
// no socket stuff at all, define it ourselves!
enum {
	SOCK_STREAM, //!< aperios doesn't provide SOCK_STREAM, so we will initialize them to these values
	SOCK_DGRAM //!< aperios doesn't provide SOCK_DGRAM, so we will initialize them to these values
};

#elif !defined(__DOXYGEN__)

// some platforms give both a 'real' definition and a macro of the same name... need to detect that
#define doTestSelfRef(foo) defined(x##foo)
#define testSelfRef(foo) doTestSelfRef(foo)

#define xSOCK_STREAM
#if defined(SOCK_STREAM) && !testSelfRef(SOCK_STREAM)
// looks like a macro-only definition, reset it
enum { _SYS_SOCK_STREAM=SOCK_STREAM };
#undef SOCK_STREAM
enum { SOCK_STREAM=_SYS_SOCK_STREAM };
#define SOCK_STREAM SOCK_STREAM
#endif
#undef xSOCK_STREAM

#define xSOCK_DGRAM
#if defined(SOCK_DGRAM) && !testSelfRef(SOCK_DGRAM)
// looks like a macro-only definition, reset it
enum { _SYS_SOCK_DGRAM=SOCK_DGRAM };
#undef SOCK_DGRAM
enum { SOCK_DGRAM=_SYS_SOCK_DGRAM };
#define SOCK_DGRAM SOCK_DGRAM
#endif
#undef xSOCK_DGRAM

#undef testSelfRef
#undef doTestSelfRef

#endif

//! Tekkotsu wireless Socket class
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

class Socket {
  friend class Wireless;

public:
  int sock; //!< unique non-negative integer representing socket. Serves as index into socket Objects array

  //! Specifies transport type. TCP is usually a good idea
  enum TransportType_t {
    SOCK_STREAM=::SOCK_STREAM, //!< TCP: guaranteed delivery, higher overhead
    SOCK_DGRAM=::SOCK_DGRAM     //!< UDP: no guarantees, low overhead
  };
	
  //! Internal TCP/UDP Connection State
  enum ConnectionState {
    CONNECTION_CLOSED,
    CONNECTION_CONNECTING,
    CONNECTION_CONNECTED,
    CONNECTION_LISTENING,
    CONNECTION_CLOSING,
    CONNECTION_ERROR
  };
	
  //! Chooses between blocking and non-blocking Wireless Input, Output. Blocking wireless output from the main process will affect the performance of the Aibo, and should only be used for debugging purposes
  enum FlushType_t {
    FLUSH_NONBLOCKING=0, //!< Writes and Reads return immediately, and are processed by another process, so Main can continue to run. Non-blocking reads require specifying a callback function to handle data received
    FLUSH_BLOCKING       //!< Blocking writes are a good idea for debugging - a blocking write will be transmitted before execution continues to the next statement. Blocking reads should be avoided, since they'll cause a significant slow down in the main process
  };
	
public:
  //! constructor
  explicit Socket(int sockn)
		: sock(sockn), trType(), flType(FLUSH_NONBLOCKING), verbosity(0), 
			endpoint(), state(CONNECTION_CLOSED), sendBufSize(), recvBufSize(),
			sendSize(0), sentSize(0), recvSize(0), writeSize(0), readSize(0),
			tx(false), rx(false), sendBuffer(), sendData(NULL), writeData(NULL), 
			recvBuffer(), recvData(NULL), readData(NULL), server_port(0), 
      rcvcbckfn(NULL), peer_addr(-1), peer_port(-1), textForward(false), textForwardBuf(NULL),
		  forwardSock(NULL), daemon(false), sckListener(NULL)
	{
#ifndef PLATFORM_APERIOS
		endpoint=-1;
#endif
	}
  virtual ~Socket(); //!< destructor

  //! use getWriteBuffer to get a memory address to write bytes to, for
  //! subsequent writing to a connection.
  /*!
   * The getWriteBuffer-write(int) combo eliminates one buffer copy. You
   * don't need to use getWriteBuffer with write(byte*, int)
   * @return pointer to the current position in the current write buffer for this socket or NULL on error
   * @param bytesreq maximum number of bytes the caller intends to set before the write method is called */
  byte* getWriteBuffer(int bytesreq);

  //! writes the specified number of bytes starting at the pointer returned.
  /*!
   * in a (prior) call to getWriteBufer
   * @param size number of bytes to be sent from the current write buffer
   */
  void write(int size);

  //! Blocking read. NOT IMPLEMENTED
  /*!
   * Tries to read upto receive buffer size worth of data from this socket.
   *
   * Blocking read is currently broken - it will be fixed in the next release
   * @return number of bytes read or -1 on error
   */
  int read();

  //! getReadBuffer is used with blocking read's NOT IMPLEMENTED
  /*!
   * The read(void) and getReadBuffer combo eliminates one buffer copy. You
   * don't need to use getReadBuffer with read(byte*, int)
   *
   * Blocking read is currently broken - it will be fixed in the next release
   * @return pointer to the buffer the previous call to blocking read wrote into or NULL if no data was read
   */
  byte* getReadBuffer();
	
	unsigned int getMaxSendSize() const { return sendBufSize; } //!< returns the maximum buffer size for a send
	unsigned int getMaxReceiveSize() const { return recvBufSize; } //!< returns the maximum buffer size for a receive
	unsigned int getAvailableSendSize() const { return sendBufSize-writeSize; } //!< returns the maximum *currently available* buffer size for a send
	unsigned int getAvailableReceiveSize() const { return recvBufSize; } //!< returns the maximum *currently available* buffer size for a receive

  //! initialize socket member variables. This is different from the constructor since sockets are reused
  void init(); 

  //! Chooses between blocking and non-blocking input, output.
  /*! This function
   * can only be called when a socket is disconnected, since it is a bad
   * idea to mix blocking and non-blocking input, output.
   * The default for a socket is non-blocking
   * @return 0 on success
   */
  int setFlushType(FlushType_t fType);
  //! returns flush mode
  FlushType_t getFlushType() const { return flType; }
	
  //! can choose between different transports; will reset the socket
  int setTransport(TransportType_t tr);
  //! returns the transport in use
  TransportType_t getTransport() const { return trType; }

	//!causes this socket to forward output to stdout if it is not connected, call setForward(NULL) to unset
  void setTextForward() { textForward=true; forwardSock=NULL; }

	//!causes this socket to forward output to @a sock if it is not connected, pass NULL to unset
  void setForward(Socket * forsock) { forwardSock=forsock; textForward=false; }

  //! Picks a level of verbosity for filtering pprintf commands.
  /*! The higher the
   * verbosity, the more the number of messages printed. This is useful
   * for filtering out non-important messages with very little processor
   * cost. Default is 0.
   * @param verbose the higher the value of verbose, the more the output
   */
  void setVerbosity(int verbose) { verbosity=verbose; }

  //! Standard write - writes specified amount of data from a buffer to a
  //! connection
  /*! You might want to consider the getWriteBuffer-write(int) combo if you
   * call this often
   * @param buf buffer to write from
   * @param size number of bytes to write
   * @return the number of bytes actually written or -1 on error
   */
  int write(const byte *buf, int size);

  //! Blocking read (NOT IMPLEMENTED)
  /*! You might want to consider the read(void) and getReadBuffer combo if you
   * call this often
   *
   * Blocking read is currently broken - it will be fixed in the next release
   * @param buf buffer to write from
   * @param size number of bytes to write
   * @return number of bytes actually read
   */
  int read(byte *buf, int size);

  //! It's standard stuff. man 3 printf on most systems should give you more
  //! information
  int printf(const char *fmt, ...) __attribute__((format(printf,2,3)));

  //! It's standard stuff. man 3 printf on most systems should give you more
  //! information
  int vprintf(const char *fmt, va_list al) __attribute__ ((format (printf, 2, 0)));

  //! Similar to printf, except it takes an extra first argument.
  /*! If vlevel is than or equal to the current #verbosity level,
   *  the string will be printed else it will be ignored.
   *  @param vlevel if (vlevel<=verbosity) print, else ignore
	 *  @param fmt same as the standard printf's format string
   */
  int pprintf(int vlevel, const char *fmt, ...) __attribute__ ((format (printf, 3, 4)));
  
  //! Initiate blocking or nonblocking write transfer depending on the type
  //! of socket.
  /*! All write commands on the socket will implicity call this. You
   * don't need to call it, unless you're implementing your own write
   */
  void flush();
	
  //! returns #daemon
  bool getDaemon() const { return daemon; }
	
	int getPeerAddress() const { return peer_addr; } //!< returns the address of the remote host in local host byte order
	std::string getPeerAddressAsString() const; //!< returns the address of the remote host as a human-readable string
	int getPeerPort() const { return peer_port; } //!< returns the port number that the remote host's socket is on

protected:
  TransportType_t trType; //!< stores choice between transports (UDP or TCP (aka Datagram or Stream))
  FlushType_t flType; //!< blocking or non-blocking flushes... note that blocking flushes only block on the handoff to the system, not actual transmission (at least under aperios)

  int verbosity; //!< can be used to filter calls to pprintf

#ifdef PLATFORM_APERIOS
  typedef antSharedBuffer buf_t; //!< the Aibo Networking Toolkit buffer data structure
  typedef antModuleRef endp_t; //!< the Aibo Networking Toolkit endpoint data structure
#else
  typedef char* buf_t; //!< a general buffer
  typedef int endp_t; //!< a unix socket descriptor
#endif

  endp_t endpoint; //!< holds the endpoint data structure for the host OS
  ConnectionState state; //!< an enum representing the current state of the socket

  int sendBufSize; //!< the size of the buffer for #sendData and #writeData
  int recvBufSize; //!< the size of the buffer for #readData and #recvData
  int sendSize; //!< the size of #sendData (total amount of data from last flush)
  int sentSize; //!< the sent portion of #sendData (amount of data which has been sent to system so far)
  int recvSize; //!< the size of #recvData (total amount of data returned by system)
  int writeSize; //!< the size of #writeData (amount of data so far ready to be flushed)
  int readSize; //!< the size of #readData (not used)
  bool tx; //!< a flag set when #sendData is in the process of being sent to the system
  bool rx; //!< not used, see #readData

  buf_t sendBuffer; //!< under aperios, a pointer to a shared region with the ip stack; under other OS, a pointer to a normal char * buffer
  byte *sendData; //!< a region within #sendBuffer, holding data in the process of being sent
  byte *writeData; //!< a region within #sendBuffer, holds data still being filled in by user code, not yet flushed

  buf_t recvBuffer; //!< under aperios, a pointer to a shared region with the ip stack; under other OS, a pointer to a normal char * buffer
  byte *recvData; //!< a region within #recvBuffer, holding data either just filled in by ip stack (during call to #rcvcbckfn), or in the process of being filled in (any other time)
  byte *readData; //!< not used (available for double buffering, but not implemented)
  
  int server_port; //!< if the socket is a server socket, this is the port it is listening on
  int (*rcvcbckfn) (char*, int); //!< pointer to callback function, called after #recvData has been filled in
	
  int peer_addr; //!< inet address of peer (if connected) or last message sender (if udp and bound); -1 otherwise (in host byte-order, not network byte-order!)
  int peer_port; //!< port of peer (if connected) or last message sender (if udp and bound); -1 otherwise

  bool textForward; //!< if true, when data is sent to the socket and the socket is not current connected, the data will be sent to stdout (overridden by #forwardSock)
  char* textForwardBuf; //!< temporary buffer allocated in getWriteBuffer() and freed in write(), if the output is destined for stdout (#textForward)
  Socket * forwardSock; //!< if non-NULL, output will be sent to this socket if the current socket is not otherwise connected (overrides #textForward)

  bool daemon; //!< if true, the socket will automatically be reopened after any closure (manual or otherwise)

	SocketListener *sckListener; //!< if non-null, class based callback interface to provide notification when new data is available for reading
protected:
  Socket(const Socket&); //!< copy constructor, don't call
  Socket& operator= (const Socket&); //!< assignment operator, don't call
};

extern Socket* sout;  //!< the standard tekkotsu in/out console (default port 10001)
extern Socket* serr;  //!< the standard tekkotsu error output (default port 10002)

/*! @file
 * @brief Defines Tekkotsu wireless Socket class, also sout and serr
 * @author alokl (Creator)
 */

#endif
