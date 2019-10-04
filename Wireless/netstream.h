#ifndef INCLUDED_netstream_h_
#define INCLUDED_netstream_h_

#ifdef PLATFORM_APERIOS
#error netstream not yet supported on AIBO/Aperios
#endif

#include <cstdio>
#include <cstring>
#include <iostream>
#include <iosfwd>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <stdexcept>

#include <errno.h>
#include <signal.h>
//#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <cstring>
#include <string>

#if defined(__FreeBSD__) || defined(__APPLE__)
#  include <sys/event.h>
#else
#  include <poll.h>
#endif

#define OTAssert(str, b) if(!b) std::cerr << "ERR " << __FILE__ << '(' << __LINE__ << "): " << str << std::endl;

class IPaddr {
public:
	static const IPaddr ANY;
	static const IPaddr BROADCAST;
	
	typedef std::string ipname_t;
	typedef unsigned int ipnum_t;
	typedef unsigned short ipport_t;
	const static unsigned int maxHostNameLen;
	
	IPaddr();
	explicit IPaddr(const ipnum_t& num);
	explicit IPaddr(const ipname_t& name);
	IPaddr(const ipnum_t& num, const ipport_t& port);
	IPaddr(const ipname_t& name, const ipport_t& port);
	virtual ~IPaddr() {}

	virtual bool set_num(const ipnum_t& num);
	virtual bool set_name(const ipname_t& name);
	virtual bool set_addr(const ipnum_t& num, const ipport_t& port) { return set_num(num) && set_port(port); }
	virtual bool set_addr(const ipname_t& name, const ipport_t& port) { return set_name(name) && set_port(port); }
	virtual bool set_port(const ipport_t& port) { ipport=port; server.sin_port=htons(port); return true; }
	virtual ipnum_t get_num() const { return ntohl(server.sin_addr.s_addr); }
	virtual const ipname_t& get_name() const { return ipname; }
	virtual ipname_t get_display_num() const;
	virtual ipname_t get_rname() const;
	virtual ipport_t get_port() const { return ipport; }
	
	virtual bool is_valid() { return server.sin_addr.s_addr!=INADDR_NONE; }

	virtual const sockaddr_in& get_addr() const { return server; }
	
	bool operator==(const IPaddr& a) const { return server.sin_addr.s_addr==a.server.sin_addr.s_addr && server.sin_port==a.server.sin_port; }
protected:
	void Init();
	
	struct sockaddr_in server;
	ipname_t ipname;
	ipport_t ipport;
};


class netstream_server {
public:
	netstream_server() : tgtAddress(), datagram(false), opsock(-1) {}
	netstream_server(unsigned int aPort, bool useDatagram=false) : tgtAddress(), datagram(useDatagram), opsock(-1) { serve(aPort,datagram); }
	netstream_server(const IPaddr& addr, bool useDatagram=false) : tgtAddress(), datagram(useDatagram), opsock(-1) { serve(addr,datagram); }
	~netstream_server() { close(); }
	
	bool serve(unsigned int aPort, bool useDatagram=false) { return serve(IPaddr(ntohl(INADDR_ANY),aPort),useDatagram); }
	bool serve(const IPaddr& addr, bool useDatagram=false);
	
	template<class T> bool accept(T& stream) const; //!< blocks until a connection is available, then transfers it to the specified stream
	void close() { ::close(opsock); opsock=-1; }
	
	bool isServing() const { return opsock>=0; }
	
protected:
	static const int BACKLOG=5;
	IPaddr tgtAddress;
	bool datagram;
	int opsock;
};


template <class charT, class traits=std::char_traits<charT> >
class basic_netbuf : public std::basic_streambuf<charT,traits> {
	friend class netstream_server;
public:
//  Types:
	typedef charT                     char_type;
	typedef typename traits::int_type int_type;
	typedef typename traits::pos_type pos_type;
	typedef typename traits::off_type off_type;
	typedef traits                    traits_type;
	
//Constructors/Destructors:
	basic_netbuf();
	basic_netbuf(const IPaddr& addr, bool datagram=false);
	basic_netbuf(const IPaddr::ipname_t& host, const IPaddr::ipport_t port, bool datagram=false);
	basic_netbuf(size_t buf_in_size, size_t buf_out_size);
	virtual ~basic_netbuf();
	
	
//basic_netbuf specific Functions:
public:
	virtual bool			open(const IPaddr& addr, bool datagram=false);
	virtual bool			open(const IPaddr::ipname_t& str, bool datagram=false);
	virtual bool			open(const IPaddr::ipname_t& ahost, unsigned int aPort, bool datagram=false) { return open(IPaddr(ahost,aPort),datagram); }
	virtual bool			listen(unsigned int aPort, bool datagram=false) { return listen(IPaddr(ntohl(INADDR_ANY),aPort),datagram); }
	virtual bool			listen(const IPaddr& addr, bool datagram=false);
	virtual bool			is_connecting() const { return openInProgress; }
	virtual bool			is_open() const { return (sock!=INVALID_SOCKET); }
	virtual void			update_status();
	virtual void			close();
	virtual void			reset() { close(); reconnect(); }
	
	virtual int			adoptFD(int fd) { int old=sock; sock=fd; update_status(); return old; }

	virtual void			setReconnect(bool doReconnect) { autoReconnect = doReconnect; }
	virtual bool			getReconnect() const { return autoReconnect; }
	
	virtual void			setEcho(bool val = true) { is_echoing = val; }
	virtual bool			getEcho() { return is_echoing; }
	
	virtual const IPaddr&		getPeerAddress() const { return peerAddress; }
	virtual const IPaddr&		getLocalAddress() const { return localAddress; }
	
protected:
	virtual void			reconnect();
	static void			printBuffer(const char* buf, int buflen, const char* header, const char* prefix);
	void					Init() { Init(def_buf_in_size, def_buf_out_size); }
	void					Init(size_t insize, size_t outsize);
	basic_netbuf* rdbuf() { return *this; } // for generic use with netstream_server
	

//Inherited Functions:
public:
	virtual void			in_sync(); //users shouldn't need to call this directly... but can if want to
	virtual void			out_flush();

protected:
	using std::basic_streambuf<charT,traits>::eback;
	using std::basic_streambuf<charT,traits>::gptr;
	using std::basic_streambuf<charT,traits>::egptr;
	using std::basic_streambuf<charT,traits>::gbump;
	using std::basic_streambuf<charT,traits>::pbase;
	using std::basic_streambuf<charT,traits>::pptr;
	using std::basic_streambuf<charT,traits>::epptr;
	using std::basic_streambuf<charT,traits>::pbump;
	
	//  lib.streambuf.virt.get Get area:
	virtual std::streamsize	showmanyc();
	//	virtual streamsize xsgetn(char_type* s, streamsize n);
	virtual int_type		underflow();
	virtual int_type		uflow();
	
	//  lib.streambuf.virt.pback Putback:
	//	virtual int_type pbackfail(int_type c = traits::eof() );
	//  lib.streambuf.virt.put Put area:
	//	virtual streamsize xsputn(const char_type* s, streamsize n);
	virtual int_type		overflow(int_type c  = traits::eof());
	
	//  lib.streambuf.virt.buffer Buffer management and positioning:
	//	virtual _Myt basic_netbuf<char_type, traits_type>* setbuf(char_type* s, streamsize n);
	virtual pos_type seekoff(off_type off, std::ios_base::seekdir way, std::ios_base::openmode which = std::ios_base::in | std::ios_base::out);
	virtual pos_type seekpos(pos_type sp, std::ios_base::openmode which = std::ios_base::in | std::ios_base::out) {	 return seekoff(sp,std::ios_base::beg,which); }
	virtual int			sync();
	//  lib.streambuf.virt.locales Locales:
	//	virtual void imbue(const locale &loc);
	
//Data:
protected:
	//! this allows us to reset status variables like openInProgress if a thread cancel occurs
	template<typename T> struct SetScope {
		SetScope(T& target, const T& enter, const T& leave) : tgt(target), dtr(leave) { target=enter; }
		~SetScope() { tgt=dtr; }
		T& tgt;
		T dtr;
	};
	static const int INVALID_SOCKET=-1;
	charT *buf_in, *buf_out;
	bool using_buf_in, using_buf_out;
	static const size_t def_buf_in_size;
	static const size_t def_buf_out_size;
	int sock;
	bool openInProgress;
	bool canRead;
	bool canWrite;
	bool is_echoing;
	bool autoReconnect;
	bool sockFromServer;
	
	IPaddr peerAddress;
	IPaddr localAddress;
	IPaddr tgtAddress;
	bool isDatagram;

private:
	basic_netbuf(const basic_netbuf&); // copy constructor, don't call
	basic_netbuf& operator=(const basic_netbuf&); //!< assignment, don't call
};
template <class charT, class traits>
const size_t basic_netbuf<charT,traits>::def_buf_in_size=(1<<14)-28;
template <class charT, class traits>
const size_t basic_netbuf<charT,traits>::def_buf_out_size=(1<<14)-28;


template <class charT, class traits=std::char_traits<charT> >
class basic_netbuf_interface {
public:
	basic_netbuf_interface() : nb() {}
	basic_netbuf_interface(const IPaddr::ipname_t& host, const IPaddr::ipport_t port, bool datagram) : nb(host,port,datagram) {}
	basic_netbuf_interface(const IPaddr& addr, bool datagram) : nb(addr,datagram) {}
	basic_netbuf_interface(size_t buf_in_size, size_t buf_out_size) : nb(buf_in_size,buf_out_size) {}
	
	inline bool			open(const IPaddr& addr, bool datagram=false) { return nb.open(addr,datagram); }
	inline bool			open(const IPaddr::ipname_t& str, bool datagram=false) { return nb.open(str,datagram); }
	inline bool			open(const IPaddr::ipname_t& ahost, unsigned int aPort, bool datagram=false) { return nb.open(ahost,aPort,datagram); }
	inline bool			listen(unsigned int aPort, bool datagram=false) { return nb.listen(aPort,datagram); }
	inline bool			listen(const IPaddr& addr, bool datagram=false) { return nb.listen(addr,datagram); }
	
	inline bool			is_connecting() const { return nb.is_connecting(); }
	inline bool			is_open() const { return nb.is_open(); }
	inline void			update_status() { nb.update_status(); }
	
	inline void			close() { nb.close(); }
	inline void			reset() { nb.close(); nb.reconnect(); }
	
	inline void			setReconnect(bool reconnect) { nb.setReconnect(reconnect); }
	inline bool			getReconnect() const { return nb.getReconnect(); }
	
	inline void			setEcho(bool val=true) { nb.setEcho(val); }
	inline bool			getEcho() { return nb.getEcho(); }
	
	inline const IPaddr&		getPeerAddress() const { return nb.getPeerAddress(); }
	inline const IPaddr&		getLocalAddress() const { return nb.getLocalAddress(); }
	
	inline basic_netbuf<charT, traits>* rdbuf() const { return const_cast<basic_netbuf<charT, traits>*>(&nb); }

protected:
	virtual ~basic_netbuf_interface() {}
	basic_netbuf<charT, traits> nb;
};

template <class charT, class traits=std::char_traits<charT> >
class basic_inetstream : public virtual basic_netbuf_interface<charT,traits>, public std::basic_istream<charT,traits>
{
public:
	typedef charT                     char_type;
	typedef typename traits::int_type int_type;
	typedef typename traits::pos_type pos_type;
	typedef typename traits::off_type off_type;
	typedef traits                    traits_type;
	//  lib.ifstream.cons Constructors:
	basic_inetstream() : basic_netbuf_interface<charT,traits>(), std::basic_istream<charT,traits>(basic_netbuf_interface<charT,traits>::rdbuf()) {}
	basic_inetstream(const IPaddr& addr, bool datagram=false) : basic_netbuf_interface<charT,traits>(addr,datagram), std::basic_istream<charT,traits>(basic_netbuf_interface<charT,traits>::rdbuf()) {}
	basic_inetstream(const IPaddr::ipname_t& host, const IPaddr::ipport_t port, bool datagram=false) : basic_netbuf_interface<charT,traits>(host,port,datagram), std::basic_istream<charT,traits>(basic_netbuf_interface<charT,traits>::rdbuf()) {}
	basic_inetstream(size_t buf_in_size, size_t buf_out_size) : basic_netbuf_interface<charT,traits>(buf_in_size,buf_out_size), std::basic_istream<charT,traits>(basic_netbuf_interface<charT,traits>::rdbuf()) {}
	using basic_netbuf_interface<charT,traits>::rdbuf;
};


template <class charT, class traits=std::char_traits<charT> >
class basic_onetstream : public virtual basic_netbuf_interface<charT,traits>, public std::basic_ostream<charT,traits>
{
public:
	typedef charT                     char_type;
	typedef typename traits::int_type int_type;
	typedef typename traits::pos_type pos_type;
	typedef typename traits::off_type off_type;
	typedef traits                    traits_type;
	//  lib.ifstream.cons Constructors:
	basic_onetstream() : basic_netbuf_interface<charT,traits>(), std::basic_ostream<charT,traits>(basic_netbuf_interface<charT,traits>::rdbuf()) {}
	basic_onetstream(const IPaddr& addr, bool datagram=false) : basic_netbuf_interface<charT,traits>(addr,datagram), std::basic_ostream<charT,traits>(basic_netbuf_interface<charT,traits>::rdbuf()) {}
	basic_onetstream(const IPaddr::ipname_t& host, const IPaddr::ipport_t port, bool datagram=false) : basic_netbuf_interface<charT,traits>(host,port,datagram), std::basic_ostream<charT,traits>(basic_netbuf_interface<charT,traits>::rdbuf()) {}
	basic_onetstream(size_t buf_in_size, size_t buf_out_size) : basic_netbuf_interface<charT,traits>(buf_in_size,buf_out_size) , std::basic_ostream<charT,traits>(basic_netbuf_interface<charT,traits>::rdbuf()){}
	using basic_netbuf_interface<charT,traits>::rdbuf;
};


template <class charT, class traits=std::char_traits<charT> >
class basic_ionetstream : public virtual basic_netbuf_interface<charT,traits>, public std::basic_iostream<charT,traits>
{
public:
	typedef charT                     char_type;
	typedef typename traits::int_type int_type;
	typedef typename traits::pos_type pos_type;
	typedef typename traits::off_type off_type;
	typedef traits                    traits_type;
	//  lib.ifstream.cons Constructors:
	basic_ionetstream() : basic_netbuf_interface<charT,traits>(), std::basic_iostream<charT,traits>(basic_netbuf_interface<charT,traits>::rdbuf()) {}
	basic_ionetstream(const IPaddr& addr, bool datagram=false) : basic_netbuf_interface<charT,traits>(addr,datagram), std::basic_iostream<charT,traits>(basic_netbuf_interface<charT,traits>::rdbuf()) {}
	basic_ionetstream(const IPaddr::ipname_t& host, const IPaddr::ipport_t port, bool datagram=false) : basic_netbuf_interface<charT,traits>(host,port,datagram), std::basic_iostream<charT,traits>(basic_netbuf_interface<charT,traits>::rdbuf()) {}
	basic_ionetstream(size_t buf_in_size, size_t buf_out_size) : basic_netbuf_interface<charT,traits>(buf_in_size,buf_out_size) , std::basic_iostream<charT,traits>(basic_netbuf_interface<charT,traits>::rdbuf()){}
	using basic_netbuf_interface<charT,traits>::rdbuf;
};

/*
template<class T>
class char_traits {
public:
  typedef T char_type;
  typedef int int_type;
  typedef T* pos_type;
  typedef unsigned int off_type;
  static void copy(pos_type dst, pos_type src, off_type size) {
    memcpy(dst,src,size);
  }
  static void move(pos_type dst, pos_type src, off_type size) {
    memmove(dst,src,size);
  }
  static int to_int_type(T c) { return c; }
  static int eof() { return EOF; }
};*/

typedef basic_netbuf<char, std::char_traits<char> > netbuf;
typedef basic_inetstream<char, std::char_traits<char> > inetstream;
typedef basic_onetstream<char, std::char_traits<char> > onetstream;
typedef basic_ionetstream<char, std::char_traits<char> > ionetstream;


template<class T> bool netstream_server::accept(T& stream) const {
	if(opsock<0)
		return false;
	stream.close();
	
	while(!stream.is_open()) {
		// block until connection
		sockaddr tmp;
		socklen_t tmplen=sizeof(tmp);
		int sock2 = ::accept(opsock, &tmp, &tmplen);
		if(sock2 < 0) {
			switch(errno) {
				default: // generally report errors
					perror("netstream accept"); 
				// except for cancellation or socket was closed by another thread
				case EINTR: case EBADF: case ECONNABORTED:
					return false;
			}
		} else {
			//replace with accepted socket
			stream.rdbuf()->sock=sock2;
			stream.rdbuf()->sockFromServer=true;
			stream.rdbuf()->isDatagram=datagram;
			stream.rdbuf()->tgtAddress = tgtAddress;
			sockaddr_in addr;
			socklen_t addr_size=sizeof(addr);
			int err = ::getpeername(sock2, reinterpret_cast<sockaddr *>(&addr), &addr_size);
			if(err<0)
				perror("netstream getpeername");
				else {
					stream.rdbuf()->peerAddress.set_addr(ntohl(addr.sin_addr.s_addr), ntohs(addr.sin_port));
				}
			err = ::getsockname(sock2, reinterpret_cast<sockaddr *>(&addr), &addr_size);
			if(err<0)
				perror("netstream getsockname");
				else {
					stream.rdbuf()->localAddress.set_addr(ntohl(addr.sin_addr.s_addr), ntohs(addr.sin_port));
				}
			//cout << " netstream connected to " << getPeerAddress().get_display_num() << ":" << getPeerAddress().get_port()
			//	<< " from " << getLocalAddress().get_display_num() << ":" << getLocalAddress().get_port() << endl;
		}
	}
	
	if(datagram) {
		const int sndbuf=(stream.rdbuf()->epptr() - stream.rdbuf()->pbase());
		if ( ::setsockopt ( stream.rdbuf()->sock, SOL_SOCKET, SO_SNDBUF, ( const char* ) &sndbuf, sizeof ( sndbuf ) ) == -1 ) {
			perror("netstream SO_SNDBUF setsockopt");
		}
	}
	
	return true;
}


template <class charT, class traits>
basic_netbuf<charT,traits>::basic_netbuf() 
  : std::basic_streambuf<charT,traits>(), buf_in(NULL), buf_out(NULL), using_buf_in(false), using_buf_out(false),
  sock(INVALID_SOCKET), openInProgress(false), canRead(false), canWrite(false), is_echoing(false), autoReconnect(false), sockFromServer(),
	peerAddress(), localAddress(), tgtAddress(), isDatagram() {
  Init();
}
template <class charT, class traits>
basic_netbuf<charT,traits>::basic_netbuf(const IPaddr& addr, bool datagram)
  : std::basic_streambuf<charT,traits>(), buf_in(NULL), buf_out(NULL), using_buf_in(false), using_buf_out(false),
	sock(INVALID_SOCKET), openInProgress(false), canRead(false), canWrite(false), is_echoing(false), autoReconnect(false), sockFromServer(),
	peerAddress(), localAddress(), tgtAddress(), isDatagram()  {
  Init();
  open(addr,datagram);
}

template <class charT, class traits>
basic_netbuf<charT,traits>::basic_netbuf(const IPaddr::ipname_t& host, const IPaddr::ipport_t port, bool datagram)
  : std::basic_streambuf<charT,traits>(), buf_in(NULL), buf_out(NULL), using_buf_in(false), using_buf_out(false),
	sock(INVALID_SOCKET), openInProgress(false), canRead(false), canWrite(false), is_echoing(false), autoReconnect(false), sockFromServer(),
	peerAddress(), localAddress(), tgtAddress(), isDatagram()  {
  Init();
  open(host,port,datagram);
}

template <class charT, class traits>
basic_netbuf<charT,traits>::basic_netbuf(size_t buf_in_size, size_t buf_out_size) 
  : std::basic_streambuf<charT,traits>(), buf_in(NULL), buf_out(NULL), using_buf_in(false), using_buf_out(false),
	sock(INVALID_SOCKET), openInProgress(false), canRead(false), canWrite(false), is_echoing(false), autoReconnect(false), sockFromServer(),
	peerAddress(), localAddress(), tgtAddress(), isDatagram()  {
  Init();
}

template <class charT, class traits>
basic_netbuf<charT,traits>::~basic_netbuf() {
	autoReconnect=false;
	if(is_open()) {
		out_flush();
		close();
	}
	if(using_buf_in)
		delete [] buf_in;
	if(using_buf_out)
		delete [] buf_out;
}

template<class charT, class traits>
void
basic_netbuf<charT,traits>::Init(size_t insize, size_t outsize) {
  buf_in = new charT[insize];
  buf_out = new charT[outsize];
  using_buf_in = using_buf_out = true;
  this->setg(buf_in,buf_in+insize,buf_in+insize);
  this->setp(buf_out,buf_out+outsize);
	//  cout << "Buffer is:" << endl;
	//  cout << "Input buffer: " << egptr() << " to " << egptr() << " at " << gptr() << endl;
	//  cout << "Output buffer: " << pbase() << " to " << epptr() << " at " << pptr() << endl;
}

template<class charT, class traits>
void
basic_netbuf<charT,traits>::update_status() {
	if(sock==INVALID_SOCKET) {
		canRead=canWrite=false;
		return;
	}
#if defined(__FreeBSD__) || defined(__APPLE__)
	int kq = kqueue();
	struct kevent ke[2];
	EV_SET(&ke[0],sock,EVFILT_READ,EV_ADD,0,0,0);
	EV_SET(&ke[1],sock,EVFILT_WRITE,EV_ADD,0,0,0);
	struct kevent ko[10];
	timespec tv = {0,0};
	int res = kevent(kq,ke,2,ko,10,&tv);
	if(res<0) {
		perror("basic_netbuf kevent");
	}
	bool eof=false;
	canRead=canWrite=false;
	for(int i=0; i<res; ++i) {
		//std::cout << ko[i].ident << ' ' << ko[i].filter << ' ' << ko[i].flags << ' ' << ko[i].fflags << ' ' << ko[i].data << std::endl;
		u_short errorflag = (ko[i].flags&EV_ERROR);
		if(errorflag && ko[i].data==ENOTSUP) {
			errorflag=0;
		} else if(ko[i].filter==EVFILT_READ)
			canRead = ko[i].data>0;
		else if(ko[i].filter==EVFILT_WRITE)
			canWrite = true;
		eof |= (ko[i].flags & (EV_EOF|errorflag));
	}
	::close(kq);
	int type;
	socklen_t optlen = sizeof(type);
	res = getsockopt(sock, SOL_SOCKET, SO_TYPE, &type, &optlen);
	bool notsock = (res<0 && errno==ENOTSOCK);
	if(res<0 && errno!=ENOTSOCK)
		perror("basic_netbuf getsockopt");
	if(!canRead && ((notsock && !canWrite && eof) || (!notsock && (!canWrite || eof)))) {
		close();
		if(autoReconnect)
			reconnect();
		if(is_open())
			update_status();
	}
#else
	pollfd pfd;
	pfd.fd=sock;
	pfd.events = POLLIN | POLLOUT;
	if(poll(&pfd,1,0)==-1)
		perror("basic_netbuf poll");
	if(pfd.revents&(POLLERR|POLLHUP|POLLNVAL)) {
		close();
		if(autoReconnect)
			reconnect();
		if(is_open())
			update_status();
	} else {
		canRead = (pfd.revents&POLLIN);
		canWrite = (pfd.revents&POLLOUT);
	}
#endif
}

template <class charT, class traits>
bool
basic_netbuf<charT,traits>::open(const IPaddr& addr, bool datagram) {
	if(is_open())
		close();
	tgtAddress=addr;
	SetScope<bool> progress(openInProgress,true,false);
	
	while(!is_open()) {
		//cout << "netstream opening " << addr.get_display_num() << ":" << addr.get_port() << endl;
		// create socket
		int opsock = ::socket(AF_INET, datagram ? (int)SOCK_DGRAM : (int)SOCK_STREAM, 0);
		if(opsock < 0) {
			perror("netstream socket()");
			//std::cerr << "netstream error: socket failed to create stream socket" << std::endl;
			return false;
		}
		int on=1;
		if ( ::setsockopt ( opsock, SOL_SOCKET, SO_REUSEADDR, ( const char* ) &on, sizeof ( on ) ) == -1 ) {
			perror("netstream SO_REUSEADDR setsockopt");
		}
#ifdef __APPLE__
		if ( ::setsockopt ( opsock, SOL_SOCKET, SO_NOSIGPIPE, ( const char* ) &on, sizeof ( on ) ) == -1 ) {	 
			perror("netstream SO_NOSIGPIPE setsockopt");	 
		}
#endif
		if(datagram) {
			if ( ::setsockopt ( opsock, SOL_SOCKET, SO_BROADCAST, ( const char* ) &on, sizeof ( on ) ) == -1 ) {
				perror("netstream SO_BROADCAST setsockopt");
			}
			const int sndbuf=(epptr()-pbase());
			if ( ::setsockopt ( opsock, SOL_SOCKET, SO_SNDBUF, ( const char* ) &sndbuf, sizeof ( sndbuf ) ) == -1 ) {
				perror("netstream SO_SNDBUF setsockopt");
			}
		}
		
		if(datagram && addr.get_num()==IPaddr::BROADCAST.get_num()) {
			// don't connect, use sendto so we can support broadcast
			sock=opsock;
			sockFromServer=false;
			isDatagram=datagram;
			peerAddress=addr;
			sockaddr_in server;
			socklen_t server_size=sizeof(server);
			int err = ::getsockname(sock, reinterpret_cast<sockaddr *>(&server), &server_size);
			if(err<0)
				perror("netstream getsockname");
			else {
				localAddress.set_addr(ntohl(server.sin_addr.s_addr), ntohs(server.sin_port));
			}
			return true;
		}
		
		// connect to server.
		sockaddr_in server = addr.get_addr();
		int err = ::connect(opsock, (const sockaddr *) &server, sizeof(server));
		if(err < 0) {
			//perror("netstream connect()");
			//cout << "netstream error: connect failed to connect to requested address" << endl;
			::close(opsock);
			if(!autoReconnect) {
				return false;
			}
			usleep(750000); // don't try to reconnect too fast
		} else {
			sock=opsock;
			sockFromServer=false;
			isDatagram=datagram;
			socklen_t server_size=sizeof(server);
			err = ::getpeername(sock, reinterpret_cast<sockaddr *>(&server), &server_size);
			if(err<0)
				perror("netstream getpeername");
			else {
				peerAddress.set_addr(ntohl(server.sin_addr.s_addr), ntohs(server.sin_port));
			}
			err = ::getsockname(sock, reinterpret_cast<sockaddr *>(&server), &server_size);
			if(err<0)
				perror("netstream getsockname");
			else {
				localAddress.set_addr(ntohl(server.sin_addr.s_addr), ntohs(server.sin_port));
			}
			//cout << " netstream connected to " << getPeerAddress().get_display_num() << ":" << getPeerAddress().get_port()
			//	<< " from " << getLocalAddress().get_display_num() << ":" << getLocalAddress().get_port() << endl;
		}
	}
	
	return true;
}

template <class charT, class traits>
bool
basic_netbuf<charT,traits>::listen(const IPaddr& addr, bool datagram) {
	if(is_open())
		close();
	tgtAddress=addr;
	SetScope<bool> progress(openInProgress,true,false);
	
	// create socket
	int opsock = ::socket(AF_INET, datagram ? (int)SOCK_DGRAM : (int)SOCK_STREAM, 0);
	if(opsock < 0) {
		perror("netstream socket");
		//cout << "netstream error: socket failed to create stream socket" << endl;
		return false;
	}
	int on=1;
	if ( ::setsockopt ( opsock, SOL_SOCKET, SO_REUSEADDR, ( const char* ) &on, sizeof ( on ) ) == -1 ) {
		perror("netstream SO_REUSEADDR setsockopt");
	}
#ifdef __APPLE__
	if ( ::setsockopt ( opsock, SOL_SOCKET, SO_REUSEPORT, ( const char* ) &on, sizeof ( on ) ) == -1 ) {	 
		perror("netstream SO_NOSIGPIPE setsockopt");	 
	}
	if ( ::setsockopt ( opsock, SOL_SOCKET, SO_NOSIGPIPE, ( const char* ) &on, sizeof ( on ) ) == -1 ) {	 
		perror("netstream SO_NOSIGPIPE setsockopt");	 
	}
#endif
	if(datagram) {
		if ( ::setsockopt ( opsock, SOL_SOCKET, SO_BROADCAST, ( const char* ) &on, sizeof ( on ) ) == -1 ) {
			perror("netstream SO_BROADCAST setsockopt");
		}
		const int sndbuf=(epptr()-pbase());
		if ( ::setsockopt ( opsock, SOL_SOCKET, SO_SNDBUF, ( const char* ) &sndbuf, sizeof ( sndbuf ) ) == -1 ) {
			perror("netstream SO_SNDBUF setsockopt");
		}
	}
	
	sockaddr_in server = addr.get_addr();
	
	// bind socket to specified address
	if(::bind(opsock, (const sockaddr *) &server, sizeof(server)) != 0) {
		//perror("netstream bind");
		::close(opsock);
		return false;
	}
	
	if(datagram) {
		// datagram is not connection based, so we are now 'connected'
		sock = opsock;
		sockFromServer = true;
		isDatagram = datagram;
		socklen_t server_size=sizeof(server);
		int err = ::getsockname(sock, reinterpret_cast<sockaddr *>(&server), &server_size);
		if(err<0)
			perror("netstream getsockname");
		else {
			localAddress.set_addr(ntohl(server.sin_addr.s_addr), ntohs(server.sin_port));
		}
		return true;
	}
	
	// tell OS to start listening
	if(::listen(opsock, 0) != 0) {
		perror("netstream listen");
		::close(opsock);
		return false;
	}
	
	while(!is_open()) {
		// block until connection
		sockaddr tmp;
		socklen_t tmplen=sizeof(tmp);
		int sock2 = ::accept(opsock, &tmp, &tmplen);
		if(sock2 < 0) {
			if(errno!=EINTR)
				perror("netstream accept");
			if(!autoReconnect) {
				::close(opsock);
				return false;
			}
		} else {
			// close server socket
			::close(opsock);
			//replace with accepted socket
			sock=sock2;
			sockFromServer=true;
			isDatagram=datagram;
			socklen_t server_size=sizeof(server);
			int err = ::getpeername(sock, reinterpret_cast<sockaddr *>(&server), &server_size);
			if(err<0)
				perror("netstream getpeername");
			else {
				peerAddress.set_addr(ntohl(server.sin_addr.s_addr), ntohs(server.sin_port));
			}
			err = ::getsockname(sock, reinterpret_cast<sockaddr *>(&server), &server_size);
			if(err<0)
				perror("netstream getsockname");
			else {
				localAddress.set_addr(ntohl(server.sin_addr.s_addr), ntohs(server.sin_port));
			}
			//cout << " netstream connected to " << getPeerAddress().get_display_num() << ":" << getPeerAddress().get_port()
			//	<< " from " << getLocalAddress().get_display_num() << ":" << getLocalAddress().get_port() << endl;
		}
	}
	
	return true;
}

template <class charT, class traits>
bool
basic_netbuf<charT,traits>::open(const IPaddr::ipname_t& str, bool datagram) {
	std::string::size_type colon = str.rfind(':');
	if(colon==std::string::npos)
		return false;
	IPaddr::ipport_t port = atoi(str.substr(colon+1).c_str());
	bool res = open(str.substr(0,colon),port,datagram);
	return res;
}

template <class charT, class traits>
void
basic_netbuf<charT,traits>::close() {
	//cout << "close called" << endl;
	if(!is_open())
		return;
	//cout << "closing" << endl;
	::close(sock);
	//cout << "closed" << endl;
	sock = INVALID_SOCKET;
}

template <class charT, class traits>
void
basic_netbuf<charT,traits>::reconnect() {
	if(sockFromServer) {
		listen(tgtAddress,isDatagram);
	} else {
		open(tgtAddress,isDatagram);
	}
}

template <class charT, class traits>
void
basic_netbuf<charT,traits>::printBuffer(const char *buf, int buflen, const char *header, const char *prefix) {
	fputs(header, stderr);
	int line_begin = 0;
	for(int i = 0; i < buflen; i++) {
		if(buf[i] == '\n') {
			line_begin = 1;
			fputc('\n', stderr);
		} else {
			if(line_begin) fputs(prefix, stderr);
			line_begin = 0;
			fputc(buf[i], stderr);
		}
	}
}

template <class charT, class traits>
void
basic_netbuf<charT,traits>::in_sync() {
	if(!is_open()) {
		//cout << "netstream error: must open connection before reading from it" << endl;
		return;
	}
	update_status();
	if(!is_open())
		return; // just discovered close, don't complain
	if(gptr()>egptr())
		gbump( egptr()-gptr() );
	if(gptr()==eback()) {
		std::cerr << "netstream error: in_sync without room in input buffer" << std::endl;
		return;
	}
	unsigned long n = gptr()-eback()-1;
	charT temp_buf[n];
	ssize_t used;
	if(isDatagram) {
		sockaddr_in peer = IPaddr::ANY.get_addr();
		socklen_t peer_size=sizeof(peer);
		used = recvfrom(sock, temp_buf, n*sizeof(charT), 0, reinterpret_cast<sockaddr *>(&peer), &peer_size);
		peerAddress.set_addr(ntohl(peer.sin_addr.s_addr), ntohs(peer.sin_port));
	} else {
		used = read(sock, temp_buf, n*sizeof(charT));
	}
	/*if(used==(ssize_t)-1)
		perror("netstream Read error");
	else
		std::cout << "Read '" << std::string(temp_buf,used) << "'" << std::endl;*/
	while(used==0 || used==(ssize_t)-1) {
		if(errno==EINTR) {
			return;
		}
		//std::cout << "netstream error: connection dropped" << std::endl;
		close();
		if(autoReconnect)
			reconnect();
		if(!is_open()) {
			return;
		}
		if(isDatagram) {
			sockaddr_in peer = IPaddr::ANY.get_addr();
			socklen_t peer_size=sizeof(peer);
			used = recvfrom(sock, temp_buf, n*sizeof(charT), 0, reinterpret_cast<sockaddr *>(&peer), &peer_size);
			peerAddress.set_addr(ntohl(peer.sin_addr.s_addr), ntohs(peer.sin_port));
			std::cout << "setting " << peerAddress.get_display_num() << ":" << peerAddress.get_port() << std::endl;
		} else {
			used = read(sock, temp_buf, n*sizeof(charT));
		}
	}
	if(is_echoing)
		printBuffer(temp_buf, used, "netstream receiving: ", "» ");
	//TODO - what if sizeof(charT)>1?  We might need to worry about getting/storing partial char
	OTAssert("Partial char was dropped!",((ssize_t)((used/sizeof(charT))*sizeof(charT))==used));
	used/=sizeof(charT);
	size_t remain = egptr()-eback()-used;
	traits::move(eback(),egptr()-remain,remain);
	traits::copy(egptr()-used,temp_buf,used);
	gbump( -used );
}

template <class charT, class traits>
void
basic_netbuf<charT,traits>::out_flush() {
	if(!is_open()) {
		//std::cout << "netstream error: must open connection before writing to it" << std::endl;
		return;
	}
	update_status();
	if(!is_open())
		return; // just discovered close, don't complain
	size_t n = (pptr()-pbase())*sizeof(charT);
	//std::cout << "Writing '" << std::string(pbase(),n) << "'" << std::endl;
	if(n==0)
		return;
	while(n>0) {
		ssize_t sent;
		if(isDatagram && peerAddress.get_num()==IPaddr::BROADCAST.get_num()) {
			sockaddr_in peer = peerAddress.get_addr();
			sent = sendto(sock, pbase(), n, 0, reinterpret_cast<sockaddr *>(&peer), sizeof(peer));
		} else {
			sent = write(sock, pbase(), n);
		}
		if(sent < 0) {
			perror("netstream flush");
			if(errno==EINTR)
				return;
			//cout << "netstream error: write error" << endl;
			close();
			if(autoReconnect)
				reconnect();
			if(!is_open())
				return;
			continue;
		}
		if(is_echoing)
			printBuffer(pbase(), sent, "netstream sending: ", "« ");
		n -= sent;
		// could be trouble if partial character sent:
		if(n!=0)
			traits::move(pbase(),pbase()+sent/sizeof(charT),n/sizeof(charT));
		pbump( -sent/sizeof(charT) );
	}
}


template <class charT, class traits>
inline std::streamsize
basic_netbuf<charT, traits>::showmanyc() {
	//caller is supposed to check this before calling...
	//if(gptr()<egptr())
	//	return egptr()-gptr();
	update_status();
	return (is_open() && canRead) ? 1 : 0;
}

template <class charT, class traits>
inline typename basic_netbuf<charT, traits>::int_type
basic_netbuf<charT, traits>::underflow() {
	in_sync();
	if(gptr()<egptr())
		return traits::to_int_type(*gptr());
//	cout << "UNDERFLOW" << endl;
	return traits::eof();
}

template <class charT, class traits>
inline typename basic_netbuf<charT, traits>::int_type
basic_netbuf<charT, traits>::uflow() {
	in_sync();
	if(gptr()<egptr()) {
		int_type ans = traits::to_int_type(*gptr());
		gbump(1);
		return ans;
	}
//	cout << "UNDERFLOW" << endl;
	return traits::eof();
}

template <class charT, class traits>
inline typename basic_netbuf<charT, traits>::int_type
basic_netbuf<charT, traits>::overflow(int_type c) { 
	out_flush();
	if(!is_open())
		return traits::eof();
	if(!traits::eq_int_type(c, traits::eof())) {
		*pptr() = c;
		pbump(1);
	}
	return traits::not_eof(c);
}

//template <class charT, class traits>
//inline basic_netbuf<charT, traits>::_Myt //not supported - don't know details of expected implementation
//basic_netbuf<charT, traits>::setbuf(char_type* /*s*/, streamsize /*n*/) {
//	return this;
//}

template <class charT, class traits>
typename basic_netbuf<charT, traits>::pos_type
basic_netbuf<charT, traits>::seekoff(off_type off, std::ios_base::seekdir way, std::ios_base::openmode which /*= ios_base::in | ios_base::out*/) {
	bool dog = (which & std::ios_base::in);
	bool dop = (which & std::ios_base::out);
	charT* newg, *newp;
	int gb,pb;
	switch(way) {
		case std::ios_base::beg: {
			newg = eback()+off;
			gb = newg-gptr();
			newp = pbase()+off;
			pb = newp-pptr();
		} break;
		case std::ios_base::cur: {
			newg = gptr()+off;
			newp = pptr()+off;
			gb=pb=static_cast<int>(off);
		} break;
		case std::ios_base::end: {
			newg = egptr()+off;
			gb = newg-gptr();
			newp = epptr()+off;
			pb = newp-pptr();
		} break;
		default:
			return pos_type(off_type(-1));
	}
	if((dog && (newg<eback() || egptr()<newg)) || (dop && (newp<pbase() || epptr()<newp)))
		return pos_type(off_type(-1));
	if(dog)
		gbump(gb);
	if(dop) {
		pbump(pb);
		return pptr()-pbase();
	} else {
		return gptr()-eback();
	}
}

template <class charT, class traits>
inline int
basic_netbuf<charT, traits>::sync() {
	out_flush();
	//in_sync();
	return is_open()?0:-1;
}

#undef OTAssert

#endif
