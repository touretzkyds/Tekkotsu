#include "EchoBehavior.h"
#include "Wireless/Wireless.h"
#include "Shared/string_util.h"
#include "Events/TextMsgEvent.h"
#include <vector>
#include <string>

REGISTER_BEHAVIOR_MENU_OPT(EchoBehavior,"TekkotsuMon",BEH_NONEXCLUSIVE);

using namespace std;

EchoBehavior* EchoBehavior::theOne=NULL;
unsigned short EchoBehavior::port=11011;
const char * const EchoBehavior::routeNames[EchoBehavior::NUM_ROUTE] = 
{
	"TCP Server", "UDP Server","TCP Client","UDP Client"
};

void EchoBehavior::doStart() {
	BehaviorBase::doStart(); // do this first (required)
	setupNetwork();
	erouter->addListener(this,EventBase::textmsgEGID);
}

void EchoBehavior::doStop() {
	erouter->removeListener(this);
	teardownNetwork();
	BehaviorBase::doStop(); // do this last (required)
}

void EchoBehavior::setupNetwork() {
	sockets[STCP]=wireless->socket(Socket::SOCK_STREAM);
	sockets[SUDP]=wireless->socket(Socket::SOCK_DGRAM);
	socks[STCP]=sockets[STCP]->sock;
	socks[SUDP]=sockets[SUDP]->sock;
	wireless->setDaemon(sockets[STCP],true);
	wireless->setDaemon(sockets[SUDP],true);
	wireless->setReceiver(sockets[STCP]->sock, server_callbackT);
	wireless->setReceiver(sockets[SUDP]->sock, server_callbackU);
	wireless->listen(sockets[STCP]->sock, port);
	wireless->listen(sockets[SUDP]->sock, port);
}

void EchoBehavior::teardownNetwork() {
	wireless->setDaemon(sockets[STCP],false);
	wireless->setDaemon(sockets[SUDP],false);
	wireless->close(sockets[STCP]);
	wireless->close(sockets[SUDP]);
	if(sockets[CTCP]!=NULL && wireless->isConnected(socks[CTCP]))
		wireless->close(sockets[CTCP]);
	if(sockets[CUDP]!=NULL && wireless->isConnected(socks[CUDP]))
		wireless->close(sockets[CUDP]);
	for(unsigned int i=0; i<NUM_ROUTE; i++)
		sockets[i]=NULL;
}

void EchoBehavior::doEvent() {
	//Check for connections which have just been closed
	for(unsigned int i=0; i<NUM_ROUTE; i++)
		if(sockets[i]!=NULL && !sockets[i]->getDaemon() && !wireless->isConnected(socks[i]))
			sockets[i]=NULL;

	//handle the event
	if(const TextMsgEvent * msg = dynamic_cast<const TextMsgEvent*>(event)) {
		vector<string> args;
		vector<unsigned int> offs;
		string_util::parseArgs(msg->getText(),args,offs);
		if(args.size()==0)
			return;
		if(args[0]=="open") {
			if(args.size()<4) {
				serr->printf("syntax: %s (tcp|udp) host port\n",args[0].c_str());
				return;
			}
			if(string_util::makeLower(args[1])=="tcp") {
				if(sockets[CTCP]==NULL) {
					sockets[CTCP]=wireless->socket(Socket::SOCK_STREAM);
					socks[CTCP]=sockets[CTCP]->sock;
					wireless->setReceiver(sockets[CTCP], client_callbackT);
				}
				wireless->connect(sockets[CTCP],args[2].c_str(),atoi(args[3].c_str()));
			} else if(string_util::makeLower(args[1])=="udp") {
				if(sockets[CUDP]==NULL) {
					sockets[CUDP]=wireless->socket(Socket::SOCK_DGRAM);
					socks[CUDP]=sockets[CUDP]->sock;
					wireless->setReceiver(sockets[CUDP], client_callbackU);
				}
				wireless->connect(sockets[CUDP],args[2].c_str(),atoi(args[3].c_str()));
			} else {
				serr->printf("syntax: %s (tcp|udp) host port\n",args[0].c_str());
				serr->printf("  first argument '%s' must be either 'tcp' or 'udp'\n",args[1].c_str());
				return;
			}
		} else if(args[0]=="status") {
			cout << "Listening on port " << port << endl;
			for(unsigned int i=0; i<NUM_ROUTE; i++)
				cout << routeNames[i] << ": " << (sockets[i]!=NULL && wireless->isConnected(socks[i]) ? "Connected" : "Not Connected") << endl;
			for(unsigned int i=0; i<NUM_ROUTE; i++) {
				cout << "Route from " << routeNames[i] << ": ";
				for(unsigned int j=0; j<NUM_ROUTE; j++)
					cout << route[i][j] << ' ';
				cout << endl;
			}
		} else if(args[0]=="relay" || args[0]=="unlink") {
			unsigned char from=(unsigned char)-1U;
			unsigned char to=(unsigned char)-1U;
			bool val = (args[0]=="relay");
			unsigned int i=1;
			for(; i<args.size(); i++) {
				if(string_util::makeLower(args[i])=="to")
					break;
				intersect(from,args[i]);
			}
			if(i==args.size()) {
				serr->printf("syntax: %s [udp|tcp|client|server]* to [udp|tcp|client|server]*\n",args[0].c_str());
				return;
			}
			i++;
			for(; i<args.size(); i++)
				intersect(to,args[i]);
			for(unsigned int r=0; r<NUM_ROUTE; r++) {
				if(from & (1<<r))
					for(unsigned int j=0; j<NUM_ROUTE; j++)
						if(to & (1<<j))
							route[r][j]=val;
			}
		} else if(args[0]=="port") {
			int p=atoi(args[1].c_str());
			if(p==0) {
				serr->printf("invalid port value\n");
				return;
			}
			port=p;
			teardownNetwork();
			setupNetwork();
		} else {
			for(unsigned int i=0; i<NUM_ROUTE; i++)
				if(sockets[i]!=NULL)
					sockets[i]->printf("%s\n",msg->getText().c_str());
		}
	} else {
		serr->printf("Bad event received: %s\n",event->getName().c_str());
	}
}

void EchoBehavior::intersect(unsigned char& bits, std::string arg) {
	arg=string_util::makeLower(arg);
	unsigned char mask=0;
	if(arg=="server")
		mask = (1<<SUDP) | (1<<STCP) ;
	else if(arg=="client")
		mask = (1<<CUDP) | (1<<CTCP) ;
	else if(arg=="tcp")
		mask = (1<<STCP) | (1<<CTCP) ;
	else if(arg=="udp")
		mask = (1<<SUDP) | (1<<CUDP) ;
	else
		cerr << "Unknown argument '" << arg << "'" << endl;
	bits&=mask;
}

void EchoBehavior::processCallback(EchoBehavior::routeIndex_t src, char *buf, int bytes) {
	buf[bytes]='\0'; //hack - better hope it's not a full buffer
	sout->printf("From %s:%d\n",sockets[src]->getPeerAddressAsString().c_str(),sockets[src]->getPeerPort());
	sout->printf("Message is: '%s'\n",buf);
	
	//udp server doesn't automatically connect
	if(src==SUDP && sockets[SUDP]!=NULL && !wireless->isConnected(socks[SUDP])) {
		for(unsigned int i=0; i<NUM_ROUTE; i++)
			if(route[i][SUDP]) {
				//we found someone who wants to send output to the udp server, so we'd better connect it
				wireless->connect(sockets[SUDP],sockets[SUDP]->getPeerAddressAsString().c_str(),sockets[SUDP]->getPeerPort());
				break;
			}
	} 
	
	for(unsigned int i=0; i<NUM_ROUTE; i++) {
		if(route[src][i] && sockets[i]!=NULL) {
			if(wireless->isConnected(socks[i]))
				sockets[i]->write((byte*)buf,bytes);
			else if(!sockets[i]->getDaemon())
				sockets[i]=NULL;
		}
	}
}
int EchoBehavior::server_callbackT(char *buf, int bytes) {
	cout << "TCP Server received " << bytes << " bytes" << endl;
	theOne->processCallback(STCP,buf,bytes);
	return 0;
}
int EchoBehavior::client_callbackT(char *buf, int bytes) {
	cout << "TCP Client received " << bytes << " bytes" << endl;
	theOne->processCallback(CTCP,buf,bytes);
	return 0;
}
int EchoBehavior::server_callbackU(char *buf, int bytes) {
	cout << "UDP Server received " << bytes << " bytes" << endl;
	theOne->processCallback(SUDP,buf,bytes);
	return 0;
}
int EchoBehavior::client_callbackU(char *buf, int bytes) {
	cout << "UDP Client received " << bytes << " bytes" << endl;
	theOne->processCallback(CUDP,buf,bytes);
	return 0;
}


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
