#include <unistd.h>   // for usleep
#include "dynamixel_util.h"

using namespace std;

int cmd_mon(std::istream& is, std::ostream& os, const std::vector<std::pair<servoid,servoid> >& monRanges);

int cmd_mon(std::istream& is, std::ostream &os, int argc, const char* argv[]) {
	vector<pair<servoid,servoid> > monRanges;

	try {
		readRanges(argc-1,&argv[1],monRanges,argv[0]);
	} catch(...) {
		return 2;
	}

	return cmd_mon(is, os, monRanges);
}

//! this version does the actual execution of the command
int cmd_mon(std::istream& is, std::ostream& os, const std::vector<std::pair<servoid,servoid> >& inRanges) {
	std::vector<std::pair<servoid,servoid> > monRanges=inRanges;
    
	while (1) {
		bool firstServo = true;
		for(vector<pair<servoid,servoid> >::const_iterator it=monRanges.begin(); it!=monRanges.end(); ++it) {
			for(unsigned int sid=it->first; sid<=it->second; ++sid) {
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

				cout << (firstServo ? "-> " : "   ");
				cout << "Servo: " << setw(3) << ((int) sid) 
					<< "  Pos: " << setw(4) << setfill('0') << position
					<< "  Load:" << setw(5) << setfill(' ') << load
					<< "  Volts: " << setw(4) << std::fixed << std::setprecision(1) << voltage
					<< "  Temp: " << tempC << " (" << int(tempF) << " F)"
					<< "  Current: " << setw(4) << current
					<< "   ";
				dispErrors(cout,respHeader.error);
				cout << endl;
				firstServo = false;
			}
		}
		usleep(1000000);
	}
	return 0;
}

static bool registered = registerCommand("mon",cmd_mon);
