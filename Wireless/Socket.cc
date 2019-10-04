#include "Socket.h"
#include <stdio.h>
#include "Wireless.h"
#ifndef PLATFORM_APERIOS
#  include "IPC/Thread.h"
#  include <sys/types.h>
#  include <sys/socket.h>
#  include <netinet/in.h>
#  include <netdb.h>
#  include <arpa/inet.h>
#  include <fcntl.h>
#endif
#include "Shared/Config.h"
#include "Shared/MarkScope.h"
#include <unistd.h>

Socket* sout=NULL;
Socket* serr=NULL;

int Socket::setTransport(TransportType_t tr) {
#ifdef PLATFORM_APERIOS
	trType=tr;
#else
	if(trType==tr && endpoint!=-1)
		return 0;
	trType=tr;
	init();
#endif
	return 0;
}


Socket::~Socket() {
#ifndef PLATFORM_APERIOS
	if(endpoint!=-1)
		::close ( endpoint );
	if(recvBuffer!=NULL)
		delete [] recvBuffer;
	if(sendBuffer!=NULL)
		delete [] sendBuffer;
#endif
}

byte*
Socket::getWriteBuffer(int bytesreq)
{
#ifndef PLATFORM_APERIOS
  MarkScope l(wireless->getLock());
#endif
	byte* buf=NULL;
  if (sendBufSize-writeSize>=bytesreq
      && state==CONNECTION_CONNECTED)
    buf=writeData+writeSize;
  else if (state!=CONNECTION_CONNECTED) {
		if(forwardSock!=NULL)
			buf=forwardSock->getWriteBuffer(bytesreq);
		else if(textForward)
			buf=(byte*)(textForwardBuf=new char[bytesreq]);
  }
#ifndef PLATFORM_APERIOS
	// the lock is about to expire as we leave, but if buf is
	// valid, we want to retain the lock until write() is called
	if(buf!=NULL)
		wireless->getLock().useResource(Resource::emptyData);
#endif
	return buf;
}

void
Socket::write(int size)
{
#ifndef PLATFORM_APERIOS
#  ifdef DEBUG
	//we should be coming in with a lock from getWriteBuffer...
	if(Thread::Lock * lock=dynamic_cast<Thread::Lock*>(&wireless->getLock())) {
		if(lock->getLockLevel()==0) {
			serr->printf("Socket: Attempted write() without previously successful getWriteBuffer()\n");
			return;
		}
	}
#  endif
#endif
  writeSize+=size;
	if(textForwardBuf) {
		::write(STDOUT_FILENO,textForwardBuf,size);
		delete textForwardBuf;
		textForwardBuf=NULL;
	} else
		flush();
#ifndef PLATFORM_APERIOS
  wireless->getLock().releaseResource(Resource::emptyData);
#endif
}

int
Socket::read()
{
  return -1;
}

byte*
Socket::getReadBuffer()
{
	return NULL;
//  return readData+readSize;
}

void
Socket::init()
{
#ifndef PLATFORM_APERIOS
  MarkScope l(wireless->getLock());
#endif
  sendSize=0;
  writeSize=0;
  peer_addr=peer_port=-1;
#ifndef PLATFORM_APERIOS
	if(endpoint!=-1)
		if(::close(endpoint)==-1)
			perror("Wireless::close(): close");
	state = CONNECTION_CLOSED;
	endpoint = ::socket ( AF_INET,trType,0 );
	if(endpoint==-1) {
		perror("Socket::init(): socket()");
		return;
	}
	// TIME_WAIT - argh
	int on = 1;
	if ( ::setsockopt ( endpoint, SOL_SOCKET, SO_REUSEADDR, ( const char* ) &on, sizeof ( on ) ) == -1 ) {
		perror("Socket::init(): SO_REUSEADDR setsockopt");
	}
	if(trType==Socket::SOCK_DGRAM) {
		if ( ::setsockopt ( endpoint, SOL_SOCKET, SO_BROADCAST, ( const char* ) &on, sizeof ( on ) ) == -1 ) {
			perror("Socket::init(): SO_BROADCAST setsockopt");
		}
		const int sndbuf=(1<<16)-28;
		if ( ::setsockopt ( endpoint, SOL_SOCKET, SO_SNDBUF, ( const char* ) &sndbuf, sizeof ( sndbuf ) ) == -1 ) {
			perror("Socket::init(): SO_SNDBUF setsockopt");
		}
	}
	if( ::fcntl(endpoint,F_SETFL,O_NONBLOCK) ==-1 ) {
		perror("Socket::init(): fcntl");
	}
#endif
}

int
Socket::setFlushType(FlushType_t fType)
{
  if (state != CONNECTION_CLOSED) return -1;
  flType=fType;
  return 0;
}

void
Socket::flush()
{
#ifndef PLATFORM_APERIOS
  MarkScope l(wireless->getLock());
#endif
  if (state!=CONNECTION_CONNECTED) {
		if(forwardSock!=NULL)
			return forwardSock->flush();
	} else {
		if (flType==FLUSH_NONBLOCKING) {
			if (!tx) {
				byte *tempData=sendData;
				int tempSize=sendSize;
				sendData=writeData;
				writeData=tempData;
				sendSize=writeSize;
				writeSize=tempSize;
				wireless->send(sock);
			}
		} else {
			sendData=writeData;
			sendSize=writeSize;
			wireless->blockingSend(sock);
			writeSize=0;
		}
	}
}

int
Socket::pprintf(int vlevel, const char *fmt, ...)
{
  if (vlevel>verbosity) return 0;

  int ret;
  va_list al;
  va_start(al,fmt);
  ret=this->printf(fmt, al); 
  va_end(al);

  return ret;
}

int
Socket::printf(const char *fmt, ...)
{
  va_list al;
  va_start(al,fmt);
	int ret=vprintf(fmt,al);
  va_end(al);
  return ret;
}

int
Socket::vprintf(const char *fmt, va_list al)
{
	/*
  if (state==CONNECTION_CONNECTED && (sendBufSize-writeSize<256)) {
    flush();
    if (sendBufSize-writeSize<256)
      return -1;
  }*/

  if (state!=CONNECTION_CONNECTED) {
		if(forwardSock!=NULL)
			return forwardSock->vprintf(fmt,al);
		if(textForward)
			return vfprintf(stdout, fmt, al);
  } else {
#ifndef PLATFORM_APERIOS
		MarkScope l(wireless->getLock());
#endif
		int ret=vsnprintf((char *)(writeData+writeSize), sendBufSize-writeSize, fmt, al);
		writeSize+=ret;
		flush();
		return ret;
  }
	return -1;
}

int
Socket::write(const byte *buf, int size)
{
  if (state!=CONNECTION_CONNECTED) {
		if(forwardSock!=NULL)
			return forwardSock->write(buf,size);
		if(textForward)
			return ::write(STDOUT_FILENO,buf,size);
	} else {
#ifndef PLATFORM_APERIOS
		MarkScope l(wireless->getLock());
#endif
		byte *destbuf=getWriteBuffer(size);
		if (destbuf==NULL) return -1;
		memcpy(destbuf, buf, size);
		write(size);
		return size;
	}
	return -1;
}

int
Socket::read(byte * /*buf*/, int /*size*/)
{  
/*  if (size>=recvBufSize-recvPos) return -1;
  memcpy(buf,recvData+recvPos,size);*/
  return -1;
}

std::string Socket::getPeerAddressAsString() const {
	char buf[20];
	snprintf(buf,20,"%hhu.%hhu.%hhu.%hhu",(unsigned char)(peer_addr>>24),(unsigned char)(peer_addr>>16),(unsigned char)(peer_addr>>8),(unsigned char)peer_addr);
	//const unsigned char *b=(const unsigned char*)&peer_addr;
	//snprintf(buf,20,"%hhu.%hhu.%hhu.%hhu",b[0],b[1],b[2],b[3]);
	//snprintf(buf,20,"%hhu.%hhu.%hhu.%hhu",peer_addr&0xFF,(peer_addr>>8)&0xFF,(peer_addr>>16)&0xFF,(peer_addr>>24)&0xFF);
	return buf;
}

/*! @file
 * @brief Implements Tekkotsu wireless Socket class, also sout and serr
 * @author alokl (Creator)
 */

