#include "local/DeviceDrivers/DynamixelProtocol.h"
#include "Shared/plistPrimitives.h"
#include "Shared/TimeET.h"
#include "IPC/Thread.h"

#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>

typedef unsigned char servoid;

////////////////////////////////////
//          Commands              //
////////////////////////////////////

//! type of function callback used with registerCommand()
typedef int (*cmd_t)(std::istream& is, std::ostream& os, int argc, const char* argv[]);

//! allows commands to indirectly declare themselves at static initialization
bool registerCommand(const std::string& name, cmd_t cmd);


////////////////////////////////////
//        Utility Stuff           //
////////////////////////////////////

//! For readRanges(), this filter accepts everything
inline bool filterNone(int, const char*[]) { return true; }
//! For readRanges(), this filter accepts everything that isn't an assignment (also checks the following argument doesn't start with '=')
inline bool filterEquals(int argc, const char* argv[]) {
	return strchr(argv[0],'=')==NULL && (argc==0 || argv[1][0]!='=');
}

//! parses command line arguments indicating a series of servoid values, either individual values (e.g. 2) or ranges (e.g. 4-6)
/*! template argument allows command to specify whether to continue reading the next argument as part of the range */
template<typename T>
int readRanges(int argc, const char* argv[], std::vector<std::pair<servoid,servoid> >& ranges, std::string cmd, const T& filter);

//! Parses command line arguments indicating a series of servoid values, either individual values (e.g. 2) or ranges (e.g. 4-6)
/*! Will read all arguments up to the end of the array */
inline int readRanges(int argc, const char* argv[], std::vector<std::pair<servoid,servoid> >& ranges, std::string cmd) {
	return readRanges(argc,argv,ranges,cmd,filterNone);
}

//! Indicates how long to wait for a servo response (when passed to ReadResponseThread constructor)
extern plist::Primitive<unsigned int> timeout;

//! Read servo response to a packet to verify errors (and clear it out of the buffer for data packets!)
class ReadResponseThread : public Thread {
public:
	ReadResponseThread(std::istream& in, long timeout, servoid expectedID, const std::string& errorOrigin)
		: Thread(), is(in), failsafeTime(timeout), resp(), srcID(expectedID), errstr(errorOrigin)
	{
		start();
	}

	virtual void* run();
	std::istream& is;
	TimeET failsafeTime;
	DynamixelProtocol::WriteResponse resp;
	servoid srcID;
	std::string errstr;
};


template<typename T>
int readRanges(int argc, const char* argv[], std::vector<std::pair<servoid,servoid> >& ranges, std::string cmd, const T& filter) {
	using namespace std;
	for(int i=0; i<argc; ++i) {
		if(!filter(argc-i,&argv[i]))
			return i;
		char * dash = const_cast<char*>(strchr(argv[i],'-'));
		if(dash==NULL) {
			unsigned int x;
			if(!(stringstream(argv[i]) >> x)) {
				cerr << "ERROR: non-numeric servo id: " << argv[i] << endl;
				throw invalid_argument("source id");
			}
			if(x>DynamixelProtocol::MAX_ID) {
				cerr << "ERROR: '" << cmd << "' servo id is out of range (0-" << DynamixelProtocol::MAX_ID << ")" << endl;
				throw range_error("source id");
			}
			ranges.push_back(make_pair(x,x));
		} else {
			*dash++='\0';
			unsigned int x1=0;
			if(!(stringstream(argv[i]) >> x1)) {
				cerr << "ERROR: non-numeric servo id: " << argv[i] << endl;
				throw invalid_argument("source id");
			}
			if(x1>DynamixelProtocol::MAX_ID) {
				cerr << "ERROR: '" << cmd << "' range-min servo id is out of range (0-" << DynamixelProtocol::MAX_ID << ")" << endl;
				throw range_error("source id");
			}
			unsigned int x2;
			if(!(stringstream(dash) >> x2)) {
				cerr << "ERROR: non-numeric servo id: " << dash << endl;
				throw invalid_argument("source id");
			}
			if(x2>DynamixelProtocol::MAX_ID) {
				cerr << "ERROR: '" << cmd << "' range-max servo id is out of range (0-" << DynamixelProtocol::MAX_ID << ")" << endl;
				throw range_error("source id");
			}
			if(x2<x1) {
				cerr << "ERROR: '" << cmd << "' range-min (" << x1 << ") is greater than range-max (" << x2 << ")" << endl;
				throw range_error("source id");
			}
			ranges.push_back(make_pair(x1,x2));
		}
	}
	return argc;
}

void dispErrors(std::ostream& os, unsigned int error);
