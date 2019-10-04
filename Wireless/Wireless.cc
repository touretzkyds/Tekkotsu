#include "Wireless.h"
#include "Socket.h"
#include <cstring>
#include <cstdio>
#include "Shared/ProjectInterface.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>

#include "SocketListener.h"

Wireless *wireless=NULL;

#ifdef PLATFORM_APERIOS
#  include <OPENR/OSyslog.h>
#  include <OPENR/OPENRAPI.h>
#  include <ant.h>
#  include <EndpointTypes.h>
#  include <TCPEndpointMsg.h>
#  include <UDPEndpointMsg.h>
#  include "aperios/MMCombo/entry.h"

using namespace std;

Wireless::Wireless ()
	: ipstackRef(), myOID(), freeSockets(), usedSockets(), usedSocketsInvalidated(false)
{
	ipstackRef = antStackRef("IPStack");
	WhoAmI(&myOID);

	sockets[0]=new DummySocket(0);
	for (int sock = 1; sock < WIRELESS_MAX_SOCKETS; sock++) {
		sockets[sock]=NULL;
		freeSockets.push_back(sock);
	}
}

Wireless::~Wireless ()
{
	if(usedSockets.size()>0) {
		cerr << "WARNING: Wireless deleted with open Sockets" << endl;
		for(list<int>::const_iterator it=usedSockets.begin(); it!=usedSockets.end(); ++it) {
			delete sockets[*it];
			sockets[*it]=NULL;
		}
		freeSockets.insert(freeSockets.end(),usedSockets.begin(),usedSockets.end());
		usedSockets.clear();
		usedSocketsInvalidated=true;
	}
}

Socket* Wireless::socket(Socket::TransportType_t ttype)
{
	return socket(ttype, WIRELESS_DEF_RECV_SIZE, WIRELESS_DEF_SEND_SIZE);
}

Socket* Wireless::socket(Socket::TransportType_t ttype, int recvsize, int sendsize)
{
	if (freeSockets.empty()
			|| (recvsize + sendsize) <= 256) return sockets[0];
	int sock_num=freeSockets.front();
	freeSockets.pop_front();
	usedSockets.push_back(sock_num);
	usedSocketsInvalidated=true;

	sockets[sock_num]=new Socket(sock_num);
	sockets[sock_num]->sendBufSize=sendsize;
	sockets[sock_num]->recvBufSize=recvsize;
	sockets[sock_num]->setTransport(ttype);

	// setup send buffer
	antEnvCreateSharedBufferMsg sendBufferMsg(sendsize*2);
	sendBufferMsg.Call(ipstackRef, sizeof(sendBufferMsg));
	if (sendBufferMsg.error != ANT_SUCCESS) return sockets[0];

	sockets[sock_num]->sendBuffer = sendBufferMsg.buffer;
	sockets[sock_num]->sendBuffer.Map();
	sockets[sock_num]->sendData = ( byte * ) ( sockets[sock_num]->sendBuffer.GetAddress() );

	// setup receive buffer
	antEnvCreateSharedBufferMsg recvBufferMsg(recvsize*2);
	recvBufferMsg.Call(ipstackRef, sizeof(recvBufferMsg));
	if (recvBufferMsg.error != ANT_SUCCESS) return sockets[0];

	sockets[sock_num]->recvBuffer = recvBufferMsg.buffer;
	sockets[sock_num]->recvBuffer.Map();
	sockets[sock_num]->recvData = ( byte * ) ( sockets[sock_num]->recvBuffer.GetAddress() );

	sockets[sock_num]->readData = sockets[sock_num]->recvData + recvsize;
	sockets[sock_num]->writeData = sockets[sock_num]->sendData + sendsize;

	return sockets[sock_num];
}

int Wireless::listen(int sock, int port)
{
	if ( port <= 0 || port >= 65535 || sock <= 0 || sock >= WIRELESS_MAX_SOCKETS || sockets[sock] == NULL
			 || sockets[sock]->state != Socket::CONNECTION_CLOSED )return -1;

	sockets[sock]->server_port = port;
	sockets[sock]->init();

	if (sockets[sock]->trType==Socket::SOCK_STREAM) {
		// create endpoint
		antEnvCreateEndpointMsg tcpCreateMsg( EndpointType_TCP, ( sockets[sock]->recvBufSize + sockets[sock]->sendBufSize ) * 3 );
		tcpCreateMsg.Call( ipstackRef, sizeof( tcpCreateMsg ) );
		if ( tcpCreateMsg.error != ANT_SUCCESS ) return -1;
		sockets[sock]->endpoint = tcpCreateMsg.moduleRef;

		// listen
		TCPEndpointListenMsg listenMsg( sockets[sock]->endpoint, IP_ADDR_ANY, port );
		listenMsg.continuation = ( void * ) sock;

		listenMsg.Send( ipstackRef, myOID, Extra_Entry[entryListenCont], sizeof( listenMsg ) );

		sockets[sock]->state = Socket::CONNECTION_LISTENING;
		return 0;
	} else if (sockets[sock]->trType==Socket::SOCK_DGRAM) {
		// create endpoint
		antEnvCreateEndpointMsg udpCreateMsg( EndpointType_UDP, ( sockets[sock]->recvBufSize + sockets[sock]->sendBufSize ) * 3 );
		udpCreateMsg.Call( ipstackRef, sizeof( udpCreateMsg ) );
		if ( udpCreateMsg.error != ANT_SUCCESS ) return -1;

		// bind socket
		sockets[sock]->endpoint = udpCreateMsg.moduleRef;
		UDPEndpointBindMsg bindMsg( sockets[sock]->endpoint, IP_ADDR_ANY, port );
		bindMsg.Call( ipstackRef, sizeof( bindMsg ) );
		bindMsg.continuation = ( void * ) sock;

		sockets[sock]->state = Socket::CONNECTION_CONNECTING;

		receive( sock );

		return 0;

	}

	else
		return -1;
}

/** Tell the ipstack we want to recieve messages with this function. */

int Wireless::connect( int sock, const char * ipaddr, int port )
{
	if ( port <= 0 || port >= 65535 || sock <= 0 || sock >= WIRELESS_MAX_SOCKETS || sockets[sock] == NULL
			 || ( sockets[sock]->trType == Socket::SOCK_STREAM && sockets[sock]->state != Socket::CONNECTION_CLOSED ) ) return -1;

	sockets[sock]->init();
	if (sockets[sock]->trType==Socket::SOCK_STREAM) {
		// create endpoint
		antEnvCreateEndpointMsg tcpCreateMsg( EndpointType_TCP, ( sockets[sock]->recvBufSize + sockets[sock]->sendBufSize ) * 3 );
		tcpCreateMsg.Call( ipstackRef, sizeof( tcpCreateMsg ) );
		if ( tcpCreateMsg.error != ANT_SUCCESS ) return -1;
		sockets[sock]->endpoint = tcpCreateMsg.moduleRef;

		// connect
		TCPEndpointConnectMsg connectMsg( sockets[sock]->endpoint, IP_ADDR_ANY, IP_PORT_ANY, ipaddr, port );
		connectMsg.continuation = ( void * ) sock;

		connectMsg.Send( ipstackRef, myOID, Extra_Entry[entryConnectCont], sizeof( connectMsg ) );
		sockets[sock]->peer_addr=connectMsg.fAddress.Address();
		sockets[sock]->peer_port=connectMsg.fPort;

		sockets[sock]->state = Socket::CONNECTION_CONNECTING;
		return 0;
	}

	else if ( sockets[sock]->trType == Socket::SOCK_DGRAM )
		{
			// connect
			UDPEndpointConnectMsg connectMsg( sockets[sock]->endpoint, ipaddr, port );

			connectMsg.continuation = ( void * ) sock;

			connectMsg.Send( ipstackRef, myOID, Extra_Entry[entryConnectCont], sizeof( connectMsg ) );
			sockets[sock]->peer_addr=connectMsg.address.Address();
			sockets[sock]->peer_port=connectMsg.port;

			sockets[sock]->state = Socket::CONNECTION_CONNECTED;
			//std::cout << "Sock " << sock << " connected via UDP to IP " << ipaddr << " port " << port << std::flush << std::endl;

			return 0;
		}

	else
		{
			return -1;
		}
}

void
Wireless::ListenCont(void* msg)
{
try {
	antEnvMsg * Msg = ( antEnvMsg * ) msg;
	int sock = ( int )( Msg->continuation );

	if ( sockets[sock]->trType == Socket::SOCK_STREAM )
		{
			TCPEndpointListenMsg * listenMsg = ( TCPEndpointListenMsg * ) antEnvMsg::Receive( msg );

			if ( listenMsg->error != TCP_SUCCESS )
				{
					sockets[sock]->state = Socket::CONNECTION_ERROR;

					// no use recycling since its a resource issue
					return;
				}
			sockets[sock]->peer_addr=listenMsg->fAddress.Address();
			sockets[sock]->peer_port=listenMsg->fPort;

			sockets[sock]->state = Socket::CONNECTION_CONNECTED;
			//sockets[sock]->local_ipaddr = listenMsg->lAddress.Address();
			//cout << "Listen set lip: " << local_ipaddr << endl;
			receive( sock );
		}

} catch(const std::exception& ex) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during Wireless Listen callback",&ex))
		throw;
} catch(...) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during Wireless Listen callback",NULL))
		throw;
}
}

void
Wireless::ConnectCont(void *msg)
{
try {
	antEnvMsg * Msg = ( antEnvMsg * ) msg;
	int sock = ( int )( Msg->continuation );

	if ( sockets[sock]->trType == Socket::SOCK_STREAM )
		{
			TCPEndpointConnectMsg * connectMsg = ( TCPEndpointConnectMsg * ) antEnvMsg::Receive( msg );
			if ( connectMsg->error != TCP_SUCCESS )
				{
					sockets[sock]->state = Socket::CONNECTION_ERROR;
					return;
				}

			sockets[sock]->state = Socket::CONNECTION_CONNECTED;
			//sockets[sock]->local_ipaddr = connectMsg->lAddress.Address();
			//cout << "Connect set lip: " << local_ipaddr << endl;
			receive( sock );
		}

} catch(const std::exception& ex) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during Wireless Connect callback",&ex))
		throw;
} catch(...) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during Wireless Connect callback",NULL))
		throw;
}
}

void
Wireless::BindCont(void *msg)
{
try {
	UDPEndpointBindMsg* bindMsg = (UDPEndpointBindMsg*) antEnvMsg::Receive( msg );
	int sock = (int)bindMsg->continuation;

	if (bindMsg->error != UDP_SUCCESS) {
		sockets[sock]->state = Socket::CONNECTION_ERROR;
		return;
	}

	sockets[sock]->state = Socket::CONNECTION_CONNECTED;
	/*	if(bindMsg->address.Address()!=0) {
		//sockets[sock]->local_ipaddr = bindMsg->address.Address();
		//cout << "Bind set lip: " << local_ipaddr << endl;
	} else {
		//cout << "Bind got 0" << endl;
		}*/

} catch(const std::exception& ex) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during Wireless Bind callback",&ex))
		throw;
} catch(...) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during Wireless Bind callback",NULL))
		throw;
}
}

void
Wireless::send(int sock)
{
	if ( sock <= 0 || sock >= WIRELESS_MAX_SOCKETS || sockets[sock] == NULL || sockets[sock]->state != Socket::CONNECTION_CONNECTED
			 || sockets[sock]->sendSize <= 0 ) return;

	if ( sockets[sock]->trType == Socket::SOCK_STREAM )
		{
			TCPEndpointSendMsg sendMsg( sockets[sock]->endpoint, sockets[sock]->sendData, sockets[sock]->sendSize );
			sendMsg.continuation = ( void * ) sock;

			sockets[sock]->tx = true;
			sendMsg.Send( ipstackRef, myOID, Extra_Entry[entrySendCont], sizeof( TCPEndpointSendMsg ) );
			sockets[sock]->sendSize = 0;
		}

	else if ( sockets[sock]->trType == Socket::SOCK_DGRAM )
		{
			UDPEndpointSendMsg sendMsg( sockets[sock]->endpoint, sockets[sock]->sendData, sockets[sock]->sendSize );

			// this field is just hijacked to id the socket # this message is being sent across
			sendMsg.continuation = ( void * ) sock;

			sockets[sock]->tx = true;
			sendMsg.Send( ipstackRef, myOID, Extra_Entry[entrySendCont], sizeof( UDPEndpointSendMsg ) );
			sockets[sock]->sendSize = 0;
		}
}

void
Wireless::SendCont(void* msg)
{
try {
	antEnvMsg * Msg = ( antEnvMsg * ) msg;
	int sock = ( int )( Msg->continuation );

	if ( sockets[sock]->trType == Socket::SOCK_STREAM )
		{
			TCPEndpointSendMsg * sendMsg = ( TCPEndpointSendMsg * ) antEnvMsg::Receive( msg );
			sockets[sock]->tx = false;
			if ( sendMsg->error != TCP_SUCCESS )
				{
					sockets[sock]->state = Socket::CONNECTION_ERROR;
					close( sock );
					return;
				}
		}

	else if ( sockets[sock]->trType == Socket::SOCK_DGRAM )
		{
			UDPEndpointSendMsg * sendMsg = ( UDPEndpointSendMsg * ) antEnvMsg::Receive( msg );
			sockets[sock]->tx = false;
			if ( sendMsg->error != UDP_SUCCESS )
				{
					sockets[sock]->state = Socket::CONNECTION_ERROR;
					close( sock );
					return;
				}
		}

	sockets[sock]->flush();

} catch(const std::exception& ex) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during Wireless Send callback",&ex))
		throw;
} catch(...) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during Wireless Send callback",NULL))
		throw;
}
}

/*! @bug This doesn't actually seem to block until the message is
*	fully sent... a crash immediately after this will still cause a
*	line or two to be dropped.  This is still less dropped than
*	regular send, but doesn't do much good for debugging until we fix
*	this. (if we can...) */
void
Wireless::blockingSend(int sock)
{
	if ( sock <= 0 || sock >= WIRELESS_MAX_SOCKETS || sockets[sock] == NULL || sockets[sock]->state != Socket::CONNECTION_CONNECTED
			 || sockets[sock]->sendSize <= 0 ) return;

	if ( sockets[sock]->trType == Socket::SOCK_STREAM )
		{
			TCPEndpointSendMsg sendMsg( sockets[sock]->endpoint, sockets[sock]->sendData, sockets[sock]->sendSize );
			sendMsg.continuation = ( void * ) sock;

			sockets[sock]->tx=true;
			sockets[sock]->sendSize = 0;
			sendMsg.Call( ipstackRef, sizeof( TCPEndpointSendMsg ) );
			sockets[sock]->tx = false;
		}

	// no double buffering
}

void
Wireless::setReceiver(int sock, int (*rcvcbckfn) (char*, int) )
{
	if (sock<=0 || sock>=WIRELESS_MAX_SOCKETS || sockets[sock]==NULL) return;

	sockets[sock]->rcvcbckfn=rcvcbckfn;
}

void Wireless::setReceiver(int sock, SocketListener *listener) {
	if (sock <= 0 || sock >= WIRELESS_MAX_SOCKETS || sockets[sock] == NULL)
		return;

	sockets[sock]->sckListener = listener;
}

void
Wireless::receive(int sock)
{
	if ( sock <= 0 || sock >= WIRELESS_MAX_SOCKETS || sockets[sock] == NULL
			 || ( sockets[sock]->trType == Socket::SOCK_STREAM && sockets[sock]->state != Socket::CONNECTION_CONNECTED ) )
		return;

	if ( sockets[sock]->trType == Socket::SOCK_STREAM )
		{
			TCPEndpointReceiveMsg receiveMsg( sockets[sock]->endpoint, sockets[sock]->recvData, 1, sockets[sock]->recvBufSize );
			receiveMsg.continuation = ( void * ) sock;
			receiveMsg.Send( ipstackRef, myOID, Extra_Entry[entryReceiveCont], sizeof( receiveMsg ) );
		}

	else if ( sockets[sock]->trType == Socket::SOCK_DGRAM )
		{
			UDPEndpointReceiveMsg receiveMsg( sockets[sock]->endpoint, sockets[sock]->recvData, sockets[sock]->recvBufSize );
			receiveMsg.continuation = ( void * ) sock;
			receiveMsg.Send( ipstackRef, myOID, Extra_Entry[entryReceiveCont], sizeof( receiveMsg ) );
		}

	sockets[sock]->rx = true;
}

void
Wireless::receive(int sock, int (*rcvcbckfn) (char*, int) )
{
	if (sock<=0 || sock>=WIRELESS_MAX_SOCKETS || sockets[sock]==NULL
			|| sockets[sock]->state != Socket::CONNECTION_CONNECTED) return;

	sockets[sock]->rcvcbckfn = rcvcbckfn;

	if ( sockets[sock]->trType == Socket::SOCK_STREAM )
		{
			TCPEndpointReceiveMsg receiveMsg( sockets[sock]->endpoint, sockets[sock]->recvData, 1, sockets[sock]->recvBufSize );
			receiveMsg.continuation = ( void * ) sock;
			receiveMsg.Send( ipstackRef, myOID, Extra_Entry[entryReceiveCont], sizeof( receiveMsg ) );
		}

	else if ( sockets[sock]->trType == Socket::SOCK_DGRAM )
		{
			UDPEndpointReceiveMsg receiveMsg( sockets[sock]->endpoint, sockets[sock]->recvData, sockets[sock]->recvBufSize );
			receiveMsg.continuation = ( void * ) sock;
			receiveMsg.Send( ipstackRef, myOID, Extra_Entry[entryReceiveCont], sizeof( receiveMsg ) );
		}

	sockets[sock]->rx = true;
}

void
Wireless::ReceiveCont(void* msg)
{
try {
	// get the socket index before casting the message into UDP or TCP form
	antEnvMsg * Msg = ( antEnvMsg * ) msg;
	int sock = ( int )( Msg->continuation );

	if ( sock <= 0 || sock >= WIRELESS_MAX_SOCKETS || sockets[sock] == NULL
			 || ( sockets[sock]->state != Socket::CONNECTION_CONNECTED && sockets[sock]->state != Socket::CONNECTION_CONNECTING ) )
		return;

	if ( sockets[sock]->trType == Socket::SOCK_STREAM )
		{
			TCPEndpointReceiveMsg * receiveMsg = ( TCPEndpointReceiveMsg * ) antEnvMsg::Receive( msg );
			if ( receiveMsg->error != TCP_SUCCESS )
				{
					sockets[sock]->state = Socket::CONNECTION_ERROR;
					close( sock );
					return;
				}

			sockets[sock]->recvSize = receiveMsg->sizeMin;

			if (sockets[sock]->sckListener != NULL) {
				sockets[sock]->sckListener->processData((char *)sockets[sock]->recvData,
														sockets[sock]->recvSize);

			} else if (sockets[sock]->rcvcbckfn != NULL) {
				sockets[sock]->rcvcbckfn((char *)sockets[sock]->recvData,
										 sockets[sock]->recvSize);
			}
			sockets[sock]->recvSize = 0;

		}

	else if ( sockets[sock]->trType == Socket::SOCK_DGRAM )
		{
			UDPEndpointReceiveMsg * receiveMsg;
			receiveMsg = ( UDPEndpointReceiveMsg * ) antEnvMsg::Receive( msg );
			sockets[sock]->recvSize = receiveMsg->size;

			if ( receiveMsg->error == UDP_SUCCESS )
				{
					// if this UDP connection is not connected yet, connect it
					// to the address & port of the computer that sent this message.
					// This allows us to send UDP messages to any address instead of
					// hard-coding a specific address beforehand

					sockets[sock]->peer_addr=receiveMsg->address.Address();
					sockets[sock]->peer_port=receiveMsg->port;
					if ( !strncmp( "connection request", ( char * ) sockets[sock]->recvData, 18 ) ) {
						// clear this message from the receiving buffer
						sockets[sock]->recvData += sockets[sock]->recvSize;

						if ( sockets[sock]->state != Socket::CONNECTION_CONNECTED ) {
							char caller[14];
							receiveMsg->address.GetAsString( caller );
							connect( sock, caller, receiveMsg->port );
						}

					} else if (sockets[sock]->sckListener != NULL) {
						sockets[sock]->sckListener->processData((char *)sockets[sock]->recvData, sockets[sock]->recvSize);

					} else if ( sockets[sock]->rcvcbckfn != NULL )
						sockets[sock]->rcvcbckfn( ( char * ) sockets[sock]->recvData, sockets[sock]->recvSize );

				}

			sockets[sock]->recvSize = 0;

		}

	receive( sock );

} catch(const std::exception& ex) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during Wireless Receive callback",&ex))
		throw;
} catch(...) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during Wireless Receive callback",NULL))
		throw;
}
}

void
Wireless::close(int sock)
{
	if (sockets[sock]->state == Socket::CONNECTION_CLOSED ||
			sockets[sock]->state == Socket::CONNECTION_CLOSING) return;

	if (!(sockets[sock]->server_port>0 && sockets[sock]->daemon)) {
		sockets[sock]->recvBuffer.UnMap();
		antEnvDestroySharedBufferMsg receiveBufferMsg(sockets[sock]->recvBuffer);
		receiveBufferMsg.Call(ipstackRef, sizeof(antEnvDestroySharedBufferMsg));
		sockets[sock]->sendBuffer.UnMap();
		antEnvDestroySharedBufferMsg sendBufferMsg(sockets[sock]->sendBuffer);
		sendBufferMsg.Call(ipstackRef, sizeof(antEnvDestroySharedBufferMsg));
	}

	if ( sockets[sock]->trType == Socket::SOCK_STREAM )
		{
			TCPEndpointCloseMsg closeMsg( sockets[sock]->endpoint );
			closeMsg.continuation = ( void * ) sock;
			closeMsg.Send( ipstackRef, myOID, Extra_Entry[entryCloseCont], sizeof( closeMsg ) );
		}

	else if ( sockets[sock]->trType == Socket::SOCK_DGRAM )
		{
			UDPEndpointCloseMsg closeMsg( sockets[sock]->endpoint );
			closeMsg.continuation = ( void * ) sock;
			closeMsg.Send( ipstackRef, myOID, Extra_Entry[entryCloseCont], sizeof( closeMsg ) );
		}

	sockets[sock]->peer_addr=sockets[sock]->peer_port=-1;

	sockets[sock]->state = Socket::CONNECTION_CLOSING;
}

uint32 Wireless::getIPAddress(unsigned int /*idx=0*/) {
	uint32 local_ipaddr = 0;
	//from OPEN-R sample ERA201D1Info:
	antEnvInitGetParamMsg getParamMsg("ETHER_IP");
	getParamMsg.Call(ipstackRef, sizeof(getParamMsg));
	if (getParamMsg.error == ANT_SUCCESS && getParamMsg.paramType == antEnv_InitParam_String) {
		//cout << "******** RECEIVED " << getParamMsg.value.str << endl;
		unsigned int i=0;
		for(int j=3; j>=0; j--) {
			unsigned int b=0;
			while(i<ANTENV_VALUE_LENGTH_MAX && getParamMsg.value.str[i]!='.' && getParamMsg.value.str[i]!='\0')
				b=b*10+(getParamMsg.value.str[i++]-'0');
			i++; //skip over '.'
			local_ipaddr+=b<<(j*8);
			//cout << j << ": " << b << ' ' << local_ipaddr << endl;
		}
	} else {
		OSYSLOG1((osyslogERROR,"getParamMsg.Call() FAILED %d", getParamMsg.error));
	}
	return local_ipaddr;
}

void
Wireless::CloseCont(void* msg)
{
try {
	antEnvMsg * closeMsg = ( antEnvMsg * ) antEnvMsg::Receive( msg );
	int sock = ( int )( closeMsg->continuation );
	if ( sockets[sock] == NULL )
		return;

	sockets[sock]->state = Socket::CONNECTION_CLOSED;
	sockets[sock]->peer_addr=sockets[sock]->peer_port=-1;
	if ( sockets[sock]->server_port > 0 && sockets[sock]->daemon )
		{
			// recycle if server
			listen( sock, sockets[sock]->server_port );
		}

	else
		{
			delete( sockets[sock] );
			sockets[sock] = NULL;
			freeSockets.push_back( sock );
			for(list<int>::iterator it=usedSockets.begin(); it!=usedSockets.end(); ++it)
				if(*it==sock) {
					usedSockets.erase(it);
					usedSocketsInvalidated=true;
					break;
				}
		}

} catch(const std::exception& ex) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during Wireless Close callback",&ex))
		throw;
} catch(...) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during Wireless Close callback",NULL))
		throw;
}
}

#else // PLATFORM_LOCAL
#  include "IPC/Thread.h"
#  include <sys/types.h>
#  include <sys/socket.h>
#  include <netinet/in.h>
#  include <netdb.h>
#  include <arpa/inet.h>
#  include <fcntl.h>
#  include <unistd.h>
#  include <iostream>
#  include <errno.h>
#  include "Shared/MarkScope.h"

using namespace std;


Wireless::Wireless ()
	: callbackLock(NULL), interruptChk(-1), interruptCtl(-1), rfds(), wfds(), efds(), fdsMax(0), freeSockets(), usedSockets(), usedSocketsInvalidated(false)
{
	sockets[0]=new DummySocket(0);
	for (int sock = 1; sock < WIRELESS_MAX_SOCKETS; sock++) {
		sockets[sock]=NULL;
		freeSockets.push_back(sock);
	}
	int p[2];
	pipe(p);
	interruptChk=p[0];
	interruptCtl=p[1];
	fdsMax=interruptChk;
	if( ::fcntl(interruptChk,F_SETFL,O_NONBLOCK) ==-1 ) {
		perror("Wireless::Wireless(): fcntl");
	}
	FD_ZERO(&rfds);
	FD_SET(interruptChk,&rfds);
	FD_ZERO(&wfds);
	FD_ZERO(&efds);
}

Wireless::~Wireless ()
{
	MarkScope l(getLock());
	::close(interruptChk);
	::close(interruptCtl);
	interruptChk=interruptCtl=-1;
	if(usedSockets.size()>0) {
		cerr << "WARNING: Wireless deleted with open Sockets" << endl;
		for(list<int>::const_iterator it=usedSockets.begin(); it!=usedSockets.end(); ++it) {
			delete sockets[*it];
			sockets[*it]=NULL;
		}
		freeSockets.insert(freeSockets.end(),usedSockets.begin(),usedSockets.end());
		usedSockets.clear();
		usedSocketsInvalidated=true;
	}
	delete sockets[0]; // DummySocket
}

void Wireless::setReceiver(int sock, int (*rcvcbckfn) (char*, int) ) {
	sockets[sock]->rcvcbckfn=rcvcbckfn;
}

void Wireless::setReceiver(int sock, SocketListener *listener) {
	sockets[sock]->sckListener = listener;
}

void Wireless::close(int sock) {
	MarkScope l(getLock());
	if ( sock <= 0 || sock >= WIRELESS_MAX_SOCKETS || sockets[sock] == NULL)
		return;
	sockets[sock]->flush();
	sockets[sock]->peer_port = sockets[sock]->peer_addr = -1;
	if(sockets[sock]->daemon) {
		sockets[sock]->init();
		listen(sock,sockets[sock]->server_port);
	} else {
		bool found=false;
		for(list<int>::iterator it=usedSockets.begin(); it!=usedSockets.end(); ++it) {
			if(*it==sock) {
				usedSockets.erase(it);
				found = usedSocketsInvalidated = true;
				break;
			}
		}
		if(!found) {
			cerr << "WARNING: Could not find socket " << sock << " in usedSockets list of size " << usedSockets.size() << endl;
			return;
		}
		Socket * s=sockets[sock];
		sockets[sock] = NULL; //we don't delete the socket here -- wakeup() will cause pollProcess to do that
		wakeup(s); //avoid select giving error about bad FD
		freeSockets.push_back( sock );
	}
}

int Wireless::connect(int sock, const char* ipaddr, int port) {
	MarkScope l(getLock());
	if ( port <= 0 || port >= 65535 || sock <= 0 || sock >= WIRELESS_MAX_SOCKETS
			 || sockets[sock] == NULL || (sockets[sock]->state != Socket::CONNECTION_CLOSED && sockets[sock]->trType!=Socket::SOCK_DGRAM) )
		return -1;

	if(sockets[sock]->endpoint==-1)
		sockets[sock]->init();
	int endpoint=sockets[sock]->endpoint;

	sockaddr_in m_addr;
	m_addr.sin_family = AF_INET;
	m_addr.sin_port = htons ( port );
	struct hostent* hostips = gethostbyname(ipaddr);
	if ( hostips==NULL) {
		cerr << "Wireless::connect(): gethostbyname returned NULL, h_errno==" << h_errno;
		switch(h_errno) {
		case HOST_NOT_FOUND: cerr << " (HOST_NOT_FOUND)" << endl; break;
		case NO_ADDRESS: cerr << " (NO_ADDRESS)" << endl; break;
			//case NO_DATA: cerr << " (NO_DATA)" << endl; break; //NO_DATA==NO_ADDRESS
		case NO_RECOVERY: cerr << " (NO_RECOVERY)" << endl; break;
		case TRY_AGAIN: cerr << " (TRY_AGAIN)" << endl; break;
		default: cerr << " (unknown error code!)" << endl; break;
		}
		return -1;
	}
	memcpy(&m_addr.sin_addr,hostips->h_addr_list[0],sizeof(m_addr.sin_addr));

	int status = ::connect ( endpoint, ( sockaddr * ) &m_addr, sizeof ( m_addr ) );
	if ( status == 0 )
		sockets[sock]->state=Socket::CONNECTION_CONNECTED;
	else if(errno==EINPROGRESS)
		sockets[sock]->state=Socket::CONNECTION_CONNECTING;
	else {
		perror("Wireless::connect(): connect");
		return -1;
	}
	sockets[sock]->peer_port = port;
	sockets[sock]->peer_addr = ntohl(m_addr.sin_addr.s_addr);
	//cout << "connecting " << sockets[sock]->getPeerAddressAsString() << ' ' << sockets[sock]->getPeerPort() << "... " <<status << endl;
	//this will allow sock to be added to wfds so we can tell when the connection goes through
	wakeup();
	return 0;
}

int Wireless::listen(int sock, int port) {
  MarkScope l(getLock());
  if ( port <= 0 || port >= 65535 || sock <= 0 || sock >= WIRELESS_MAX_SOCKETS
       || sockets[sock] == NULL || sockets[sock]->state != Socket::CONNECTION_CLOSED )
		return -1;
  sockets[sock]->server_port = port;
  sockets[sock]->init();
  int endpoint=sockets[sock]->endpoint;
  if ( endpoint<0 )
	return -1;
  sockaddr_in m_addr;
  m_addr.sin_family = AF_INET;
  m_addr.sin_addr.s_addr = INADDR_ANY;
  m_addr.sin_port = htons ( port );

	int bind_return = ::bind ( endpoint,( struct sockaddr * ) &m_addr,sizeof ( m_addr ) );
	if ( bind_return == -1 ) {
		perror("Wireless::listen: bind");
		return -1;
	}
	if(sockets[sock]->trType==Socket::SOCK_STREAM) {
		int listen_return = ::listen ( endpoint, MAXCONNECTIONS );
		if ( listen_return == -1 ) {
			perror("Wireless::listen: listen");
			return -1;
		}
	}
	sockets[sock]->state = Socket::CONNECTION_LISTENING;
	//this will allow sock to be added to rfds so we can tell when a connection is available
	wakeup();
	return 0;
}

Socket* Wireless::socket(Socket::TransportType_t ttype) {
	return socket(ttype, WIRELESS_DEF_RECV_SIZE, WIRELESS_DEF_SEND_SIZE);
}
Socket* Wireless::socket(Socket::TransportType_t ttype, int recvsize, int sendsize) {
	MarkScope l(getLock());
	if (freeSockets.empty()
			|| (recvsize + sendsize) <= 256) return sockets[0];
	int sock_num=freeSockets.front();
	freeSockets.pop_front();
	usedSockets.push_back(sock_num);
	usedSocketsInvalidated=true;

	sockets[sock_num]=new Socket(sock_num);

	sockets[sock_num]->sendBufSize=sendsize;
	sockets[sock_num]->sendBuffer=new char[sockets[sock_num]->sendBufSize*2];
	//double buffered sending
	sockets[sock_num]->sendData=(byte*)sockets[sock_num]->sendBuffer;
	sockets[sock_num]->writeData=(byte*)sockets[sock_num]->sendBuffer+sockets[sock_num]->sendBufSize;

	sockets[sock_num]->recvBufSize=recvsize;
	sockets[sock_num]->recvBuffer = new char[sockets[sock_num]->recvBufSize];
	sockets[sock_num]->recvData=(byte*)sockets[sock_num]->recvBuffer; //reading is single buffered

	sockets[sock_num]->setTransport(ttype);

	return sockets[sock_num];
}

/*! There's probably better ways to implement this...
 *  (run through the interface list?  How does ifconfig do it?) */
uint32 Wireless::getIPAddress(unsigned int idx/*=0*/) {
	char buf[ 255 ];
	if(gethostname( buf, 255)!=0) {
		perror("Wireless::getIPAddress(): gethostname");
		return 0;
	}
	struct hostent * h = gethostbyname( buf );
	if(h==NULL) {
		herror("Wireless::getIPAddress(): gethostbyname");
		return 0;
	}
	//check to make sure 'idx' is valid
	for(unsigned int x=0; x<=idx; x++)
		if(h->h_addr_list[x]==NULL)
			return 0;
	//if we got here, it's valid
	return *(uint32*)h->h_addr_list[idx];
}

void
Wireless::send(int sock)
{
	MarkScope l(getLock());
	if ( sock <= 0 || sock >= WIRELESS_MAX_SOCKETS || sockets[sock] == NULL
	    || sockets[sock]->state != Socket::CONNECTION_CONNECTED || sockets[sock]->sendSize <= 0 )
		return;

	//we could defer all sending to the poll, but let's give a shot at sending it out right away to reduce latency
	int s=sockets[sock]->endpoint;
	int sent=::send(s,sockets[sock]->sendData+sockets[sock]->sentSize,sockets[sock]->sendSize-sockets[sock]->sentSize,0);
	if(sent==-1) {
		if(errno==ECONNREFUSED) {
			close(sock);
		} else {
			perror("Wireless::send(): send");
			cerr << "Wireless::send() data size was " << sockets[sock]->sendSize-sockets[sock]->sentSize << endl;
			sockets[sock]->tx = false;
			sockets[sock]->sendSize = sockets[sock]->sentSize = 0;
		}
	} else {
		sockets[sock]->sentSize+=sent;
		if(sockets[sock]->sentSize==sockets[sock]->sendSize) {
			sockets[sock]->tx = false;
			sockets[sock]->sendSize = sockets[sock]->sentSize = 0;
			sockets[sock]->flush();
		} else {
			sockets[sock]->tx = true;
			//more work will be done in poll()
			//this will wake up the poll thread to make it check for pending writes (signaled by tx flag)
			wakeup();
		}
	}
}

void
Wireless::blockingSend(int sock)
{
	MarkScope l(getLock());
	if ( sock <= 0 || sock >= WIRELESS_MAX_SOCKETS || sockets[sock] == NULL
	    || sockets[sock]->state != Socket::CONNECTION_CONNECTED || sockets[sock]->sendSize <= 0 )
		return;

	while(sockets[sock]->sentSize<sockets[sock]->sendSize) {
		fd_set bs_wfds;
		FD_ZERO(&bs_wfds);
		FD_SET(sockets[sock]->endpoint, &bs_wfds);
		int retval = select(sockets[sock]->endpoint+1, NULL, &bs_wfds, NULL, NULL);
		if(retval==0)
			continue;
		if(retval==-1) {
			perror("Wireless::poll(): select");
			return;
		}
		if(sockets[sock]->tx) //block on leftover non-blocking send
			continue;
		int sent=::send(sockets[sock]->endpoint,sockets[sock]->sendData+sockets[sock]->sentSize,sockets[sock]->sendSize-sockets[sock]->sentSize,0);
		if(sent==-1) {
			if(errno==EAGAIN) {
				cerr << "Wireless::blockingSend(): send() was not ready, even though select() said it was" << endl;
				continue;
			}
			perror("Wireless::blockingSend(): send");
			sockets[sock]->tx = false;
			sockets[sock]->sendSize = sockets[sock]->sentSize = 0;
			return;
		}
		sockets[sock]->sentSize+=sent;
	}
	sockets[sock]->sendSize = sockets[sock]->sentSize = 0;
}

void Wireless::pollSetup() {
	FD_ZERO(&rfds);
	FD_ZERO(&wfds);
	FD_ZERO(&efds);
	FD_SET(interruptChk, &rfds);

	fdsMax=interruptChk;
	MarkScope l(getLock());
	//cout << "pollSetup " << usedSockets.size() << endl;
	for(list<int>::const_iterator it=usedSockets.begin(); it!=usedSockets.end(); ++it) {
		if(sockets[*it]==NULL) {
			cerr << "ERROR: Wireless::pollSetup() encountered NULL socket " << *it << endl;
			continue;
		}
		if(sockets[*it]->endpoint==-1) {
			cerr << "ERROR Wireless::pollSetup() encountered bad endpoint " << *it << endl;
			continue;
		}
		if(sockets[*it]->state!=Socket::CONNECTION_CLOSED && sockets[*it]->state!=Socket::CONNECTION_ERROR)
			FD_SET(sockets[*it]->endpoint, &rfds);
		if(sockets[*it]->state==Socket::CONNECTION_CONNECTING || sockets[*it]->tx)
			FD_SET(sockets[*it]->endpoint, &wfds);
		FD_SET(sockets[*it]->endpoint, &efds);
		if(sockets[*it]->endpoint>fdsMax)
			fdsMax=sockets[*it]->endpoint;
	}
}

/*! @param tv  how long to wait -- NULL will wait indefinitely until a socket event occurs
 *  @return true if there was a socket event to process, false if timed out */
bool Wireless::pollTest(struct timeval* tv) {
	int retval = select(fdsMax+1, &rfds, &wfds, &efds, tv);
	if(retval==-1 && errno!=EINTR)
		perror("Wireless::pollTest(): select");
	return (retval!=0);
}

void Wireless::pollProcess() {
	MarkScope cl(getCallbackLock()); //note how this will go out of scope and release the lock if an exception occurs... sexy!
	MarkScope l(getLock());
	if(FD_ISSET(interruptChk,&rfds)) {
		//wakeup sent to handle non-blocking write
		int res=1;
		//cout << "Clearing interrupts..." << flush;
		while(res>0) {
			Socket * del=NULL;
			res=::read(interruptChk,&del,sizeof(del));
			//cout << ' ' << del << flush;
			if(del!=NULL)
				delete del;
		}
		//cout << " done" << endl;
	}
	usedSocketsInvalidated=false;
	for(list<int>::const_iterator it=usedSockets.begin(); it!=usedSockets.end(); ++it) {
		if(sockets[*it]==NULL) {
			cerr << "NULL socket " << *it << endl;
			continue;
		}
		if(sockets[*it]->endpoint==-1) {
			cerr << "bad endpoint " << *it << endl;
			continue;
		}
		int s=sockets[*it]->endpoint;
		if(FD_ISSET(s,&rfds)) {
			FD_CLR(s,&rfds); // in case we loop do to invalidated iterator during user callback
			//cout << *it << " set in read" << endl;
			if(sockets[*it]->state==Socket::CONNECTION_CONNECTING) {
				//cout << "Wireless::pollProcess(): read set on connecting" << endl;
				sockets[*it]->state=Socket::CONNECTION_CONNECTED;
			}
			if(sockets[*it]->state==Socket::CONNECTION_LISTENING) {
				if(sockets[*it]->trType==Socket::SOCK_STREAM) {
					sockaddr_in m_addr;
					socklen_t addrlen=sizeof(m_addr);
					int n=accept(s,(sockaddr*)&m_addr,&addrlen);
					if(n==-1) {
						if(errno!=EAGAIN) //EAGAIN indicates we were woken due to some other issue, like a previous close completing
							perror("Wireless::pollProcess(): accept");
						continue;
					}
					sockets[*it]->peer_addr=ntohl(m_addr.sin_addr.s_addr);
					sockets[*it]->peer_port=ntohs(m_addr.sin_port);
					//this closes the server socket -- we'll want to address this
					if(::close(s)==-1)
						perror("Wireless::pollProcess(): close");
					s=sockets[*it]->endpoint=n;
					sockets[*it]->state=Socket::CONNECTION_CONNECTED;
					//cout << "Accepted connection" << endl;
				} else {
					//cout << "UDP accept" << endl;
					sockaddr_in m_addr;
					socklen_t addrlen=sizeof(m_addr);
					sockets[*it]->recvSize = recvfrom(s,sockets[*it]->recvData,sockets[*it]->recvBufSize,0,(sockaddr*)&m_addr,&addrlen);
					if(sockets[*it]->recvSize==-1) {
						perror("Wireless::pollProcess(): acception recvfrom");
						continue;
					}
					/* //this can cause trouble for broadcasts... we'll make the user call 'connect' themselves if they want to send back
					 //cout << "connecting..." << endl;
					int ret = ::connect ( s, (sockaddr*) &m_addr, sizeof ( m_addr ) );
					if ( ret==-1 && errno!=EINPROGRESS ) {
						perror("Wireless::pollProcess(): connect");
						continue;
					}
					//cout << "UDP accepted!" << endl;
					sockets[*it]->state=CONNECTION_CONNECTING;
					 */
					sockets[*it]->peer_addr=ntohl(m_addr.sin_addr.s_addr);
					sockets[*it]->peer_port=ntohs(m_addr.sin_port);
					if(sockets[*it]->recvSize!=0) {
						//cout << "Read " << sockets[*it]->recvSize << " bytes " << sockets[*it]->rcvcbckfn << endl;
						if ( !strncmp( "connection request", ( char * ) sockets[*it]->recvData, 18 ) ) {
							// clear this message from the receiving buffer
							if ( sockets[*it]->state != Socket::CONNECTION_CONNECTED )
								connect( *it, sockets[*it]->getPeerAddressAsString().c_str(), sockets[*it]->getPeerPort() );
						} else if (sockets[*it]->sckListener != NULL) {
							try {
								sockets[*it]->sckListener->processData((char *)sockets[*it]->recvData,
																	   sockets[*it]->recvSize);
							} catch(const std::exception& ex) {
								if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during networking received data callback",&ex))
									throw;
							} catch(...) {
								if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during networking received data callback",NULL))
									throw;
							}
						} else if ( sockets[*it]->rcvcbckfn != NULL ) {
							try {
								sockets[*it]->rcvcbckfn( ( char * ) sockets[*it]->recvData, sockets[*it]->recvSize );
							} catch(const std::exception& ex) {
								if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during networking received data callback",&ex))
									throw;
							} catch(...) {
								if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during networking received data callback",NULL))
									throw;
							}
						}
						if(usedSocketsInvalidated) {
							pollProcess(); // iterator lost, restart processing (tail recursive loop)
							return;
						}
						sockets[*it]->recvSize = 0;
					}
				}
			} else if(sockets[*it]->state==Socket::CONNECTION_CONNECTED || sockets[*it]->state==Socket::CONNECTION_CLOSING) {
				sockets[*it]->recvSize = recvfrom(s,sockets[*it]->recvData,sockets[*it]->recvBufSize,0,NULL,NULL);
				if(sockets[*it]->recvSize==-1) {
					if(errno!=EAGAIN) { //may have just completed connection, not a problem
						if(errno==ECONNREFUSED || errno==ECONNRESET) {
							//cerr << "connection refused: endpoint=" << s << " sock=" << *it << " Socket=" << sockets[*it] << endl;
							list<int>::const_iterator tmp=it;
							//a UDP server could come in here if the client closes down (i.e. packet is refused)
							if(!sockets[*it]->daemon) //don't decrement if the socket is going to stay open
								--it;
							close(*tmp);
							continue;
						}
						perror("Wireless::pollProcess(): recvfrom");
					}
				} else if(sockets[*it]->recvSize==0) {
					list<int>::const_iterator tmp=it--;
					close(*tmp);
					//cout << "closed connection" << endl;
					continue;
				} else {
					// cout << "Read " << sockets[*it]->recvSize << " bytes " << sockets[*it]->rcvcbckfn << endl;
					if (sockets[*it]->sckListener != NULL) {
						try {
							sockets[*it]->sckListener->processData((char *)sockets[*it]->recvData,
																	   sockets[*it]->recvSize);
						} catch(const std::exception& ex) {
							if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during networking received data callback",&ex))
								throw;
						} catch(...) {
							if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during networking received data callback",NULL))
								throw;
						}
					} else if ( sockets[*it]->rcvcbckfn != NULL ) {
						try {
							sockets[*it]->rcvcbckfn( ( char * ) sockets[*it]->recvData, sockets[*it]->recvSize );
						} catch(const std::exception& ex) {
							if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during networking received data callback",&ex))
								throw;
						} catch(...) {
							if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during networking received data callback",NULL))
								throw;
						}
					}
					if(usedSocketsInvalidated) {
						pollProcess(); // iterator lost, restart processing (tail recursive loop)
						return;
					}
					sockets[*it]->recvSize = 0;
				}
			} else {
				int dropped=0,n=0;
				char buf[100];
				while((n=recvfrom(s,buf,100,0,NULL,NULL))>0)
					dropped+=n;
				cerr << "Wireless::pollProcess(): socket (sys=" << s << ", tk="<<*it<<") read flag in bad state (" << sockets[*it]->state << "), " << dropped << " bytes were dropped, ending with:" << endl;
				for(int i=0; i<n; i++)
					if(isprint(buf[i]))
						cerr << buf[i];
					else
						cerr << '.';
				cerr << endl;
			}
		}
		if(FD_ISSET(s,&wfds)) {
			FD_CLR(s,&wfds); // in case we loop do to invalidated iterator during user callback
			//cout << *it << " set in write" << endl;
			if(sockets[*it]->state==Socket::CONNECTION_CONNECTING) {
				sockets[*it]->state=Socket::CONNECTION_CONNECTED;
			} else if(sockets[*it]->state==Socket::CONNECTION_CONNECTED) {
				if(!sockets[*it]->tx) {
					//cerr << "Wireless::pollProcess(): write available on non-tx socket??" << endl;
					//can happen on a refused connection
				} else {
					int sent=::send(s,sockets[*it]->sendData+sockets[*it]->sentSize,sockets[*it]->sendSize-sockets[*it]->sentSize,0);
					if(sent==-1) {
						perror("Wireless::pollProcess(): send");
						sockets[*it]->tx = false;
						sockets[*it]->sendSize = sockets[*it]->sentSize = 0;
					} else {
						sockets[*it]->sentSize+=sent;
						if(sockets[*it]->sentSize==sockets[*it]->sendSize) {
							sockets[*it]->tx = false;
							sockets[*it]->sendSize = sockets[*it]->sentSize = 0;
							sockets[*it]->flush();
						}
					}
				}
			} else {
				cerr << "Wireless::pollProcess(): socket write flag in bad state" << endl;
			}
		}
		if(FD_ISSET(s,&efds)) {
			FD_CLR(s,&efds); // in case we loop do to invalidated iterator during user callback
			cerr << "Socket exception: " << flush;
			int err=0;
			socklen_t errlen=sizeof(err);
			if ( ::getsockopt ( s, SOL_SOCKET, SO_ERROR, &err, &errlen ) == -1 ) {
				perror("Wireless::processPoll(): getsockopt");
			}
			cerr << err << " endpoint=" << s << " sock=" << *it << " Socket=" << sockets[*it] << endl;
		}
	}
}

/*! @param del if non-NULL, will cause the socket to be closed and deleted */
void Wireless::wakeup(Socket * del/*=NULL*/) {
	::write(interruptCtl,&del,sizeof(del));
}

Resource& Wireless::getLock() {
	static Thread::Lock lock;
	return lock;
}

void Wireless::setCallbackLock(Resource& l) {
	MarkScope pcl(getCallbackLock()); //put a lock on previous callback until we've switched
	callbackLock=&l;
}

void Wireless::clearCallbackLock() {
	callbackLock=NULL;
}

uint32 Wireless::getIFAddress(const char *interface) {
	int fd;
	struct ifreq ifr;

	fd = ::socket(AF_INET, SOCK_DGRAM, 0);

	// Want an IPv4 IP address
	ifr.ifr_addr.sa_family = AF_INET;

	// Want IP address attached to "eth0"
	strncpy(ifr.ifr_name, interface, IFNAMSIZ-1);

	ioctl(fd, SIOCGIFADDR, &ifr);

	close(fd);

	return ntohl(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr.s_addr);
}

#endif

/*! @file
 * @brief Interacts with the system to provide networking services
 * @author alokl (Creator)
 * @author Erik Berglund and Bryan Johnson (UDP support)
 * @author ejt simulator support
 *
 * @verbinclude CMPack_license.txt
 */
