#include "dynamixel_util.h"
#include <iomanip>
#include <unistd.h>

using namespace std;

int cmd_scan(std::istream& is, std::ostream& os, const std::vector<std::pair<servoid,servoid> >& scanRanges);

//! this version does the parsing, then calls the other version for execution
int cmd_scan(std::istream& is, std::ostream& os, int argc, const char* argv[]) {
	vector<pair<servoid,servoid> > scanRanges;
	
	try {
		readRanges(argc-1,&argv[1],scanRanges,argv[0]);
	} catch(...) {
		return 2;
	}
	
	return cmd_scan(is, os, scanRanges);
}

//! this version does the actual execution of the command
int cmd_scan(std::istream& is, std::ostream& os, const std::vector<std::pair<servoid,servoid> >& inRanges) {
	std::vector<std::pair<servoid,servoid> > scanRanges=inRanges;
	if(scanRanges.size()==0)
		scanRanges.push_back(make_pair(0,DynamixelProtocol::MAX_ID));
	if(scanRanges.size()>1 || scanRanges.front().first!=scanRanges.front().second)
		cout << "Scanning..." << endl;
	for(vector<pair<servoid,servoid> >::const_iterator it=scanRanges.begin(); it!=scanRanges.end(); ++it) {
		for(unsigned int sid=it->first; sid<=it->second; ++sid) {
			stringstream ss;
			ss << sid << ":";
			if(inRanges.size()>0)
				cout << "Servo " << left << setw(5) << ss.str() << flush;
			is.clear(); // doesn't appear to be necessary, but just in case...
			char * model = (char*)DynamixelProtocol::PingThread(is,os,sid, -1U).join();
			if(model==Thread::CANCELLED || model==NULL) {
				if(inRanges.size()>0)
					cout << "not found" << endl;
			} else {
				if(inRanges.size()>0)
					cout << model << " detected, ";
				else
					cout << "Servo " << left << setw(5) << ss.str() << model << " detected, ";
				/******* if detected, show monitor information *****/
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

				is.clear();
				is.read(respHeader, sizeof(respHeader));

				if (respHeader.resplen != rlen + 2) {
					cerr << "WARNING: Unexpected result length: " << ((int)respHeader.resplen) << " vs expected " << rlen << endl;
					continue;
				}
				std::vector<unsigned char> respBody(rlen);
				is.read((char*) &respBody[0], rlen);
				unsigned char checksum;
				is.read((char*)&checksum,1);
				unsigned char calcChecksum = ~( DynamixelProtocol::nchecksum(respHeader, sizeof(respHeader))
																				+ 					DynamixelProtocol::nchecksum(&respBody[0], rlen));
				if (calcChecksum != checksum) {
					cerr << "WARNING: Bad checksum " << (int)(checksum) << ", expected " << (int)(calcChecksum) << endl;
					continue;
				}
				if (respHeader.servoid != sid) {
					cerr << "WARNING: Received response from servo " << (int)(respHeader.servoid) << ", expected " << (int)(sid) << "" << endl;
					continue;
				}
				if (respHeader.error!=0)
					DynamixelProtocol::reportErrors(respHeader.servoid,-1U,respHeader.error);

				short position = respBody[0] | ((short) (respBody[1])) << 8;
				short load = respBody[4] | ((short) (respBody[5]) & 0x3) << 8;
				if (respBody[5] & 0x4)
					load *= 1;
				else
					load *= -1;

				float voltage = respBody[6] / 10.f;
				unsigned int tempC = respBody[7];
				float tempF = 9.f/5.f * tempC + 32;

				int current = respBody[68] | ((int)(respBody[69])<< 8);

				cout << "  Pos: " << setw(4) << right << setfill(' ') << position
						 << "  Load: " << setw(5) << setfill(' ') << load
						 << "  Volts: " << setw(4) << std::fixed << std::setprecision(1) << voltage
						 << "  Temp: " << tempC << " (" << int(tempF) << " F)"
						 << "  Current: " << setw(4) << current
						 << "   ";
				dispErrors(cout,respHeader.error);
				cout << endl;
			}
			unsigned int usecs = 100000; // 100 milliseconds
			usleep(usecs);
		}
	}
	return EXIT_SUCCESS;
}

static bool registered = registerCommand("scan",cmd_scan);
