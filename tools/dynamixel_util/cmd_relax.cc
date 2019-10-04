#include "dynamixel_util.h"

using namespace std;

int cmd_relax(std::istream& is, std::ostream& os, const std::vector<std::pair<servoid,servoid> >& relaxRanges);

//! this version does the parsing, then calls the other version for execution
int cmd_relax(std::istream& is, std::ostream& os, int argc, const char* argv[]) {
	vector<pair<servoid,servoid> > relaxRanges;
	
	if(argc<2 || strcmp(argv[1],"all")==0) {
		relaxRanges.push_back(make_pair(DynamixelProtocol::BROADCAST_ID,DynamixelProtocol::BROADCAST_ID));
	} else try {
		readRanges(argc-1,&argv[1],relaxRanges,argv[0]);
	} catch(...) {
		return 2;
	}
	
	return cmd_relax(is, os, relaxRanges);
}

int cmd_relax(std::istream& is, std::ostream& os, const std::vector<std::pair<servoid,servoid> >& relaxRanges) {
	for(vector<pair<servoid,servoid> >::const_iterator it=relaxRanges.begin(); it!=relaxRanges.end(); ++it) {
		for(unsigned int sid=it->first; sid<=it->second; ++sid) {
			if(sid==DynamixelProtocol::BROADCAST_ID) {
				DynamixelProtocol::write(os,DynamixelProtocol::BroadcastTorqueCmd(false));
				DynamixelProtocol::write(os,DynamixelProtocol::BroadcastTorqueEntry());
				os.flush(); 
				// no response from a broadcast
			} else {
				std::cout << "Relaxing servo " << int(sid) << "... " << flush;
				// Set torque-enable to false to relax the joint.
				// Torque will be automatically re-enabled by the next move command.
				DynamixelProtocol::BroadcastTorqueCmd tcmd(false);
				tcmd.servoid=sid;
				updateChecksum(tcmd);
				DynamixelProtocol::write(os,tcmd).flush();
				os.flush();
				void * res = ReadResponseThread(is,timeout,sid,"relax").join();
				if (res==Thread::CANCELLED) {
					cerr << "ERROR: no response, servo may be disconnected" << endl;
					is.clear(); // reset input stream so we can try the next one
				} else if (res!=NULL) {
					cout << "verified" << endl;
				}
				// Set max torque to clear the effect of error conditions such as LOAD_ERROR.
				DynamixelProtocol::BroadcastTorqueEntry tentry(0xff, 0x03);
				tentry.servoid=sid;
				updateChecksum(tentry);
				DynamixelProtocol::write(os,tentry).flush();
				ReadResponseThread(is,timeout,sid,"relax").join(); 
			}
		}
	}
	return EXIT_SUCCESS;
}

static bool registered = registerCommand("relax",cmd_relax);
