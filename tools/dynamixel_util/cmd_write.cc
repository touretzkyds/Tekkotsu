#include "dynamixel_util.h"
#include <numeric>

using namespace std;

int cmd_write(std::istream& is, std::ostream& os, std::vector<std::pair<servoid,servoid> > writeRanges, unsigned char writeAddr, std::vector<unsigned char> writeValues);

//! this version does the parsing, then calls the other version for execution
int cmd_write(std::istream& is, std::ostream& os, int argc, const char* argv[]) {
	vector<pair<servoid,servoid> > writeRanges;
	unsigned char writeAddr=(unsigned char)-1U;
	vector<unsigned char> writeValues;

	int i=0;
	if(++i>argc-2) {
		cerr << "ERROR: '" << argv[0] << "' usage: <servo_id> <address>=<value> [value2 ...]" << endl;
		return 2;
	}
	if(strcmp(argv[i],"all")==0) {
		writeRanges.push_back(make_pair(DynamixelProtocol::BROADCAST_ID,DynamixelProtocol::BROADCAST_ID));
		++i;
	} else try {
		i += readRanges(argc-i-1,&argv[i],writeRanges,argv[0],filterEquals);
	} catch(...) {
		return 2;
	}
	
	if(const char* eq = strchr(argv[i],'=')) {
		// '=' given without whitespace: addr=value
		string addr(argv[i],0,eq-argv[i]);
		string value(&eq[1]);
		int x=-1;
		if(!(std::stringstream(addr) >> x) || static_cast<unsigned int>(x)>255) {
			cerr << "Bad address: " << addr << endl;
			return 2;
		}
		writeAddr = x;
		x=-1;
		if(!(std::stringstream(value) >> x) || static_cast<unsigned int>(x)>255) {
			cerr << "Bad value: " << value << endl;
			return 2;
		}
		writeValues.push_back(x);
		++i; // done with addr=value token
		while(i<argc) {
			x=-1;
			if(!(std::stringstream(argv[i++]) >> x) || static_cast<unsigned int>(x)>255) {
				cerr << "Bad value: " << argv[i-1] << endl;
				return 2;
			}
			writeValues.push_back(x);
		}
		
	} else {
		
		// '=' given as separate token: addr = value
		int x=-1;
		if(!(std::stringstream(argv[i++]) >> x) || static_cast<unsigned int>(x)>255) {
			cerr << "Bad address: " << argv[i-1] << endl;
			return 2;
		}
		writeAddr = x;
		if(i>=argc || string(argv[i])!=string("=")) {
			cerr << "ERROR: '" << argv[0] << "' should end with <addr>=<value> (did not find '=')" << endl;
			return 2;
		}
		++i; // skip "=", here apparently as a separate argument
		while(i<argc) {
			x=-1;
			if(!(std::stringstream(argv[i++]) >> x) || static_cast<unsigned int>(x)>255) {
				cerr << "Bad value: " << argv[i-1] << endl;
				return 2;
			}
			writeValues.push_back(x);
		}
	}
	
	return cmd_write(is, os, writeRanges, writeAddr, writeValues);
}

//! this version does the actual execution of the command
int cmd_write(std::istream& is, std::ostream& os, std::vector<std::pair<servoid,servoid> > writeRanges, unsigned char writeAddr, std::vector<unsigned char> writeValues) {
	if(writeRanges.size()>1)
		cout << "Scanning..." << endl;
	DynamixelProtocol::write(os,DynamixelProtocol::SetStatusResponseLevelCmd(DynamixelProtocol::RESPOND_ALL)).flush();
	DynamixelProtocol::WriteHeader header(writeAddr,writeValues.size());
	for(vector<pair<servoid,servoid> >::const_iterator it=writeRanges.begin(); it!=writeRanges.end(); ++it) {
		for(unsigned int sid=it->first; sid<=it->second; ++sid) {
			if(sid==DynamixelProtocol::BROADCAST_ID) {
				cout << "Broadcasting write command..." << flush;
			} else {
				cout << "Servo " << sid << "..." << flush;
			}
			
			header.servoid = sid;
			os.write(header,sizeof(header));
			os.write((char*)&writeValues[0],writeValues.size());
			unsigned char checksum = std::accumulate(writeValues.begin(),writeValues.end(),0);
			checksum = ~( nchecksum(header,sizeof(header)) + checksum );
			os.write((char*)&checksum,1);
			os.flush();
			if(!os) {
				cerr << "Lost communications." << endl;
				return 1;
			}
			
			if(sid==DynamixelProtocol::BROADCAST_ID) {
				cout << "complete" << endl;
			} else {
				void * res = ReadResponseThread(is,timeout,sid,"write").join();
				if(res==Thread::CANCELLED) {
					cerr << "ERROR: no response, servo may be disconnected" << endl;
					is.clear(); // reset input stream so we can try the next one
				} else if(res!=NULL) {
					cout << "verified" << endl;
				}
			}
		}
	}
	return EXIT_SUCCESS;
}

static bool registered = registerCommand("write",cmd_write);
