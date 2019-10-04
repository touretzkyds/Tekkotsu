#include "dynamixel_util.h"

using namespace std;

int cmd_read(std::istream& is, std::ostream& os, servoid readID, unsigned char readStartAddr, unsigned char readStopAddr);

//! this version does the parsing, then calls the other version for execution
int cmd_read(std::istream& is, std::ostream& os, int argc, const char* argv[]) {
	servoid readID=DynamixelProtocol::INVALID_ID;
	unsigned char readStartAddr=0;
	unsigned char readStopAddr=73;

	int i=0;
	if(++i>argc-1) {
		cerr << "ERROR: '" << argv[0] << "' requires servo id parameter" << endl;
		return 2;
	}
	unsigned int sid = atoi(argv[i++]);
	if(i<argc) {
		readStartAddr = atoi(argv[i++]);
		if(i>=argc) {
			cerr << "ERROR: '" << argv[0] << "' requires a start and an end address" << endl;
			return 2;
		}
		readStopAddr = atoi(argv[i++]);
	}
	if(i<argc) {
		cerr << "Extra arguments to '" << argv[0] << "': " << argv[i] << endl;
		return 2;
	}
	if(sid>DynamixelProtocol::MAX_ID) {
		if(sid==DynamixelProtocol::BROADCAST_ID)
			cerr << "ERROR: '" << argv[0] << "' source id is the broadcast address (" << DynamixelProtocol::BROADCAST_ID << ").\nPlease use 'ALL' or an id in the range (0-"<< DynamixelProtocol::MAX_ID << ")" << endl;
		else
			cerr << "ERROR: '" << argv[0] << "' source id is out of range (0-" << DynamixelProtocol::MAX_ID << ")" << endl;
		return 2;
	}
	readID=sid;
	
	return cmd_read(is, os, readID, readStartAddr, readStopAddr);
}

//! this version does the actual execution of the command
int cmd_read(std::istream& is, std::ostream& os, servoid readID, 
						 unsigned char readStartAddr, unsigned char readStopAddr) {
	size_t len=readStopAddr-readStartAddr+1;
	if(!os || !is) {
		cerr << "No connection!  Check path setting." << endl;
		return 2;
	}
	cout << "Sending query..." << endl;
	DynamixelProtocol::write(os,DynamixelProtocol::ReadCmd(readID,readStartAddr,len)).flush();
	if(!os) {
		cerr << "Lost communication or disconnected" << endl;
		return 2;
	}
	DynamixelProtocol::GenericResponseHeader respHeader;
	cout << "Reading response..." << endl;
	is.read(respHeader,sizeof(respHeader));
	if(!is) {
		cerr << "Lost communication or disconnected" << endl;
		return 2;
	}
	if(respHeader.resplen!=len+2) {
		cerr << "WARNING: Unexpected result length " << (int)(respHeader.resplen) << " received, " << len+2 << " expected." << endl;
		len=respHeader.resplen-2;
	}
	if(respHeader.resplen<=2) {
		// this shouldn't happen...
		if(respHeader.servoid != readID) {
			cerr << "WARNING: Received response from servo " << (int)(respHeader.servoid) << ", expected " << (int)(readID) << "" << endl;
		}
		if(respHeader.error!=0) {
			DynamixelProtocol::reportErrors(respHeader.servoid,-1U,respHeader.error);
		}
	} else {
		std::vector<unsigned char> respBody(len);
		is.read((char*)&respBody[0],len);
		unsigned char checksum;
		is.read((char*)&checksum,1);
		unsigned char calcChecksum = ~( DynamixelProtocol::nchecksum(respHeader,sizeof(respHeader)) + DynamixelProtocol::nchecksum(&respBody[0],len) );
		if(calcChecksum != checksum) {
			cerr << "WARNING: Bad checksum " << (int)(checksum) << ", expected " << (int)(calcChecksum) << endl;
		}
		if(respHeader.servoid != readID) {
			cerr << "WARNING: Received response from servo " << (int)(respHeader.servoid) << ", expected " << (int)(readID) << "" << endl;
		}
		if(respHeader.error!=0) {
			DynamixelProtocol::reportErrors(respHeader.servoid,-1U,respHeader.error);
		}
		cout << setw(10) << left << "ADDRESS" << "\tVALUE\n" << right;
		for(unsigned int i=0; i<len; ++i) {
			unsigned int addr = i+readStartAddr;
			cout << setw(3) << setfill(' ') << addr << " (0x" << hex << setw(2) << setfill('0') << addr << ')' << dec << '\t'
				<< setw(3) << setfill(' ') << (int)respBody[i] << " (0x" << hex << setw(2) << setfill('0') << (int)respBody[i] << ")\n" << dec;
		}
	}
	
	return EXIT_SUCCESS;
}

static bool registered = registerCommand("read",cmd_read);
