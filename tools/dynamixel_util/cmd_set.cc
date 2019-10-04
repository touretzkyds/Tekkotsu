#include "dynamixel_util.h"

using namespace std;

int cmd_set(std::istream& is, std::ostream& os, servoid setSrc, servoid setDst);

int cmd_scan(std::istream& is, std::ostream& os, const std::vector<std::pair<servoid,servoid> >& scanRanges);

//! this version does the parsing, then calls the other version for execution
int cmd_set(std::istream& is, std::ostream& os, int argc, const char* argv[]) {
	servoid setSrc=DynamixelProtocol::INVALID_ID;
	servoid setDst=DynamixelProtocol::INVALID_ID;

	int i=0;
	if(i>=argc-2) {
		cerr << "ERROR: '" << argv[0] << "' requires two arguments: (ALL|<src_id>) <dest_id>" << endl;
		return 2;
	}
	if(i<argc-3) {
		cerr << "ERROR: Extra arguments to '" << argv[0] << "': " << argv[i+3] << endl;
		return 2;
	}
	if(strcasecmp(argv[i+1],"all")==0) {
		setSrc=DynamixelProtocol::BROADCAST_ID;
	} else {
		unsigned int src = atoi(argv[i+1]);
		if(src>DynamixelProtocol::MAX_ID) {
			if(src==DynamixelProtocol::BROADCAST_ID)
				cerr << "ERROR: '" << argv[0] << "' source id is the broadcast address (" << DynamixelProtocol::BROADCAST_ID << ").\nPlease use 'ALL' or an id in the range (0-"<< DynamixelProtocol::MAX_ID << ")" << endl;
			else
				cerr << "ERROR: '" << argv[0] << "' source id is out of range (0-" << DynamixelProtocol::MAX_ID << ")" << endl;
			return 2;
		}
		setSrc=src;
	}
	unsigned int dst = atoi(argv[i+2]);
	if(dst>DynamixelProtocol::MAX_ID) {
		cerr << "ERROR: '" << argv[0] << "' destination id is out of range (0-" << DynamixelProtocol::MAX_ID << ")" << endl;
		return 2;
	}
	setDst=dst;
	
	return cmd_set(is,os,setSrc,setDst);
}


//! this version does the actual execution of the command
int cmd_set(std::istream& is, std::ostream& os, servoid setSrc, servoid setDst) {
	void * res;
	if(setSrc!=DynamixelProtocol::BROADCAST_ID) {
		res = DynamixelProtocol::PingThread(is,os,setSrc, -1U).join();
		if(res==Thread::CANCELLED || res==NULL) {
			cerr << "ERROR: source servo not responding." << endl;
			return 1;
		}
	}
	res = DynamixelProtocol::PingThread(is,os,setDst, -1U).join();
	if(res!=Thread::CANCELLED && res!=NULL) {
		cerr << "ERROR: a servo already exists at the target address, would collide." << endl;
		return 1;
	}
	if(setSrc!=DynamixelProtocol::BROADCAST_ID) {
		DynamixelProtocol::write(os,DynamixelProtocol::SetStatusResponseLevelCmd(setSrc,DynamixelProtocol::RESPOND_ALL)).flush();
		res = ReadResponseThread(is,timeout,setSrc,"set status response level").join();
		if(res==Thread::CANCELLED || res==NULL) {
			cerr << "ERROR: lost servo comm during setup" << endl;
			return 1;
		}
	}

	cout << "Setting ";
	if(setSrc==DynamixelProtocol::BROADCAST_ID) cout << "ALL servos"; else cout << "servo " << (int)setSrc;
	cout << " to " << (int)setDst << "... " << flush;
	DynamixelProtocol::write(os,DynamixelProtocol::SetServoIDCmd(setSrc,setDst)).flush();
	cout << "done!" << endl;
	
	if(setSrc!=DynamixelProtocol::BROADCAST_ID) {
		cout << "Checking response... " << flush;
		res = ReadResponseThread(is,timeout,setSrc,"set").join();
		if(res==NULL || res==Thread::CANCELLED) {
			cerr << "ERROR: No response from servo, 'set' command may not have been received" << endl;
			return 1;
		} else {
			cout << "'set' command verified!" << endl;
		}
		
	} else { // was a broadcast, no response packet will be returned by servo...
		cout << "Checking ";
		if(int err=cmd_scan(is, os, vector<pair<servoid,servoid> >(1,make_pair(setDst,setDst))))
			return err;
	}
	
	return EXIT_SUCCESS;
}

static bool registered = registerCommand("set",cmd_set);
