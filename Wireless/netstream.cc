#ifndef PLATFORM_APERIOS
#include "netstream.h"
#include <iostream>
#include <sstream>

using namespace std;

const IPaddr IPaddr::ANY(INADDR_ANY);
const IPaddr IPaddr::BROADCAST(INADDR_BROADCAST);

const unsigned int IPaddr::maxHostNameLen = 256;

IPaddr::IPaddr() : server(), ipname(), ipport(0) { Init(); }

IPaddr::IPaddr(const ipnum_t& num) : server(), ipname(), ipport(0) { Init(); set_num(num); }

IPaddr::IPaddr(const ipname_t& name) : server(), ipname(), ipport(0) { Init(); set_name(name); }

IPaddr::IPaddr(const ipnum_t& num, const ipport_t& port) : server(), ipname(), ipport(0) { Init(); set_addr(num,port); }

IPaddr::IPaddr(const ipname_t& name, const ipport_t& port) : server(), ipname(), ipport(0) { Init(); set_addr(name,port); }

bool IPaddr::set_num(const ipnum_t& num) {
	if(get_num()!=num) {
		Init();
		server.sin_family      = AF_INET;
		server.sin_addr.s_addr = htonl(num);
		server.sin_port        = htons(ipport);
		struct in_addr a;
		a.s_addr = server.sin_addr.s_addr;
		char addrname[maxHostNameLen];
		inet_ntop(AF_INET,&a,addrname,maxHostNameLen);
		ipname=addrname;
	}
	return true;
}

bool IPaddr::set_name(const ipname_t& name) {
	Init();
	if(name.find(":")!=ipname_t::npos) {
		size_t pos = name.find(":");
		int p;
		if(!(stringstream(name.substr(pos+1)) >> p) || p<0 || p>=(1<<16)) {
			cerr << "Bad port number " << name.substr(pos+1) << endl;
			return false;
		}
		set_port(p);
		return set_name(name.substr(0,pos));
	}
	if(!isalpha(name[0])) {
		// in case the string holds a dotted decimal we can convert directly
		struct in_addr a;
		if(inet_pton(AF_INET,name.c_str(),&a)<=0) {
			cerr << "IPaddr error: inet_pton failed: " << name << endl;
			server.sin_addr.s_addr = INADDR_NONE;
			return false;
		}
		return set_num(ntohl(a.s_addr));
	} else {
		Init();
		struct hostent * data = gethostbyname(name.c_str());
		if(data == NULL) {
			cerr << "IPaddr error: gethostbyname failed: " << name << endl;
			server.sin_addr.s_addr = INADDR_NONE;
			return false;
		}
		ipname=name;

		memcpy((char *) &server.sin_addr, data->h_addr_list[0], data->h_length);
		server.sin_family = data->h_addrtype;
		server.sin_port = htons(ipport);
	}
	return true;
}

IPaddr::ipname_t IPaddr::get_display_num() const {
	struct in_addr a;
	a.s_addr = server.sin_addr.s_addr;
	char addrname[maxHostNameLen];
	inet_ntop(AF_INET,&a,addrname,maxHostNameLen);
	return addrname;
}

IPaddr::ipname_t IPaddr::get_rname() const {
	struct in_addr a;
	a.s_addr = server.sin_addr.s_addr;
	struct hostent * data = gethostbyaddr((char*)&a,sizeof(a),AF_INET);
	if(data==NULL) {
		cerr << "IPaddr error: gethostbyaddr failed: " << ipname << endl;
		return "";
	}
	return data->h_name;
}

void IPaddr::Init() {
	ipname="";
	memset((char *) &server, 0, sizeof(server));
	server.sin_addr.s_addr = INADDR_NONE;
}

bool netstream_server::serve(const IPaddr& addr, bool useDatagram/*=false*/) {
	if(opsock>=0)
		close();
	tgtAddress=addr;
	datagram=useDatagram;
	
	// create socket
	opsock = ::socket(AF_INET, datagram ? (int)SOCK_DGRAM : (int)SOCK_STREAM, 0);
	if(opsock < 0) {
		perror("netstream socket");
		//cout << "netstream error: socket failed to create stream socket" << endl;
		opsock=-1;
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
	}
	
	sockaddr_in server = addr.get_addr();
	
	// bind socket to specified address
	if(::bind(opsock, (const sockaddr *) &server, sizeof(server)) != 0) {
		//perror("netstream bind");
		close();
		return false;
	}
	
	// tell OS to start listening
	if(::listen(opsock, 0) != 0) {
		perror("netstream listen");
		close();
		return false;
	}
	
	return true;
}


#endif
