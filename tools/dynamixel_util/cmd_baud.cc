#include "dynamixel_util.h"
#include <cmath>

using namespace std;

int cmd_baud(std::istream& is, std::ostream& os, unsigned char baudDivisor);

//! this version does the parsing, then calls the other version for execution
int cmd_baud(std::istream& is, std::ostream& os, int argc, const char* argv[]) {
	unsigned char baudDivisor=(unsigned char)-1U;

	try {
		int i=0;
		if(++i>argc-1) {
			cerr << "ERROR: '" << argv[0] << "' requires 'rate' or 'divisor', followed by baud value" << endl;
			return 2;
		}
		if(strcasecmp("rate",argv[i])==0) {
			if(++i>argc-1) {
				cerr << "ERROR: '" << argv[0] << " rate' requires value" << endl;
				return 2;
			}
			int rate = atoi(argv[i]);
			int divisor = static_cast<int>(rintf( 2000000.f / rate ) - 1);
			if(divisor<1 || divisor>253) {
				cerr << "ERROR: baud divisor value of " << divisor << " is out of range." << endl;
				return 2;
			}
			float resRate = 2000000.f / (divisor+1);
			if(fabsf(resRate-rate)/rate>.03) {
				cerr << "ERROR: specified baud rate of " << rate << " is more than 3% from closest achievable rate of " << resRate << "." << endl;
				return 2;
			}
			baudDivisor=divisor;
			cout << "Setting divisor " << (int)baudDivisor << ", servo baud rate is " << resRate << endl;
		} else if(strcasecmp("div",argv[i])==0 || strcasecmp("divisor",argv[i])==0) {
			if(++i>argc-1) {
				cerr << "ERROR: '" << argv[0] << " divisor' requires value" << endl;
				return 2;
			}
			int divisor = atoi(argv[i]);
			if(divisor<1 || divisor>253) {
				cerr << "ERROR: baud divisor value of " << divisor << " is out of range." << endl;
				return 2;
			}
			float resRate = 2000000.f / (divisor+1);
			baudDivisor=divisor;
			cout << "Setting divisor " << (int)baudDivisor << ", servo baud rate is " << resRate << endl;
		} else {
			cerr << "ERROR: must specify 'rate' or 'divisor' for baud command." << endl;
			return 2;
		}
	} catch(...) {
		return 2;
	}
	
	return cmd_baud(is, os, baudDivisor);
}

//! this version does the actual execution of the command
int cmd_baud(std::istream& is, std::ostream& os, unsigned char baudDivisor) {
	if(baudDivisor==(unsigned char)-1U) {
		cerr << "Uninitialized baud divisor?!?!" << endl;
		return 2;
	}
	DynamixelProtocol::write(os,DynamixelProtocol::BroadcastBaudCmd(baudDivisor)).flush();
	return EXIT_SUCCESS;
}

static bool registered = registerCommand("baud",cmd_baud);
