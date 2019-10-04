#include "dynamixel_util.h"

using namespace std;

int cmd_freeze(std::istream& is, std::ostream& os, const std::vector<std::pair<servoid,servoid> >& freezeRanges);

int cmd_freeze(std::istream& is, std::ostream &os, int argc, const char* argv[]) {
	vector<pair<servoid,servoid> > freezeRanges;

	try {
		readRanges(argc-1,&argv[1],freezeRanges,argv[0]);
	} catch(...) {
		return 2;
	}

	return cmd_freeze(is, os, freezeRanges);
}

//! this version does the actual execution of the command
int cmd_freeze(std::istream& is, std::ostream& os, const std::vector<std::pair<servoid,servoid> >& inRanges) {
	std::vector<std::pair<servoid,servoid> > freezeRanges=inRanges;
    
	for (vector<pair<servoid,servoid> >::const_iterator it=freezeRanges.begin(); it!=freezeRanges.end(); ++it) {
		for (unsigned int sid=it->first; sid<=it->second; ++sid) {
			int rlen = 70;
			DynamixelProtocol::write(os,DynamixelProtocol::ReadCmd(sid,0x24,rlen)).flush();
			if (!os) {
				cerr << "Lost communication or disconnected" << endl;
				return 2;
			}
			DynamixelProtocol:: GenericResponseHeader respHeader;
			if (!is) {
				cerr << "Lost communication or disconnected" << endl;
				return 2;
			}

			is.read(respHeader, sizeof(respHeader));

			if (respHeader.resplen != rlen + 2) {
				cerr << "WARNING: Unexpected result length: " << ((int)respHeader.resplen) << " vs expected " << rlen << endl;
				continue;
			}
			std::vector<unsigned char> respBody(rlen);
			is.read((char*) &respBody[0], rlen);
			unsigned char checksum;
			is.read((char*)&checksum,1);
			unsigned char calcChecksum = ~( DynamixelProtocol::nchecksum(respHeader, sizeof(respHeader)) +
																			DynamixelProtocol::nchecksum(&respBody[0], rlen) );
			if (calcChecksum != checksum) {
				cerr << "WARNING: Bad checksum " << (int)(checksum) 
						 << ", expected " << (int)(calcChecksum) << endl;
				continue;
			}
			if (respHeader.servoid != sid) {
				cerr << "WARNING: Received response from servo " << (int)(respHeader.servoid)
						 << ", expected " << (int)(sid) << "" << endl;
				continue;
			}
			if (respHeader.error!=0)
				DynamixelProtocol::reportErrors(respHeader.servoid,-1U,respHeader.error);

			short position = respBody[0] | ((short) (respBody[1])) << 8;
			const unsigned short moveSpeed=128; //! default speed 
			DynamixelProtocol::write(os,DynamixelProtocol::SetPosSpeedCmd(sid,position,moveSpeed)).flush();
			void * res = ReadResponseThread(is,timeout,sid,"freeze").join();
            
			cout << "Servo: " << ((int) sid)
					 << "  Pos: " << setw(4) << setfill('0') << position
					 << "   ";
            
			if (res==Thread::CANCELLED) {
				cerr << "ERROR: no response, servo may be disconnected" << endl;
				is.clear(); // reset input stream so we can try the next one
			} else if (res!=NULL) {
				cout << "\nverified" << endl;
			}

			dispErrors(cout,respHeader.error);
			//cout << endl;
		}
	}
	return 0;
}

static bool registered = registerCommand("freeze",cmd_freeze);
