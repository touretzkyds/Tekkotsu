#include "dynamixel_util.h"

#include "local/CommPort.h"
#include "Events/EventBase.h"
#include "Shared/plist.h"
#include "Shared/MarkScope.h"
#include "Shared/string_util.h"
#include "IPC/FailsafeThread.h"
#include <sys/stat.h>

#define TK_ENABLE_THREADING
#include "local/minisim.h"

using namespace std;
using namespace plist;

extern void* erouter;  //!< Dummy version of erouter so we can link against DynamixelProtocol.cc.
void* erouter = NULL;

#ifdef __APPLE__
const char DEFAULT_CONF[] = "~/Library/Preferences/org.tekkotsu.dynamixel_util.plist";
#else
const char DEFAULT_CONF[] = "~/.dynamixel_util.plist";
#endif

//================================================================

Primitive<unsigned int> timeout(DynamixelProtocol::PingThread::getTimeout());

std::map<std::string, cmd_t>& commands() {
	static std::map<std::string, cmd_t> cmds;
	return cmds;
}

void dispCommPorts(ostream& os) {
	set<string> commNames;
	CommPort::getRegistry().getTypeNames(commNames);
	for(set<string>::iterator it=commNames.begin(); it!=commNames.end(); ++it)
		os << ' ' << *it;
	os << endl;
}

void usage(ostream& os, string name) {
	os <<
	"Usage: " << name << " [options] scan [<id>|<range> ...]\n"
	"       " << name << " [options] set (ALL|<src_id>) <dest_id>\n"
	"       " << name << " [options] baud (rate | divisor) <x>\n"
	"       " << name << " [options] mon [<id>|<range> ...] \n"
	"       " << name << " [options] move (ALL|<id>|<range>) <pos> [<speed>]\n"
	"       " << name << " [options] move12 (ALL|<id>|<range>) <pos> [<speed>]\n"
	"       " << name << " [options] freeze [<id>|<range> ...]\n"
	"       " << name << " [options] relax (ALL|<id>|<range> ...)\n"
	"       " << name << " [options] read <id> [ <start> <end> ]\n"
	"       " << name << " [options] write (ALL|<id>|<range> ...) <addr>=<value>[ <value> ...]\n"
	"       " << name << " [options] rot <id> <speed> <duration>\n"
	"       " << name << " [options] perf [sync] [save <file>] (<id>|range ...)\n"
	"       " << name << " [options] conf\n"
	"\n"
	"scan: the program will attempt to ping each of the specified servos, \n"
	"      where each argument is an ID or an inclusive range 'm-n'.  If no\n"
	"      arguments are specified, it will scan the entire valid range (0-253).\n"
	"\n"
	" set: change the ID value of specified servo(s) to <dest_id>.  If ALL is\n"
	"      specified as the source, the command will be broadcast, and affect\n"
	"      all connected servos.  Otherwise a single servo can be specified to\n"
	"      reassign its ID without affecting any others which may be connected.\n"
	"\n"
	"baud: Sets the baud rate or divisor to the specified value for all\n"
	"      connected servos.  Baud rates must be within 3% of a divisor,\n"
	"      as defined by: RATE = 2000000 / (DIVISOR + 1)\n"
	"\n"
	" mon: Print the position and load of specified servo in a loop\n"
  "\n"
	"move: Send a position command to specified servo(s).  If the position\n"
	"      argument is negative or has a leading '+', it will be interpreted\n"
	"      as relative to the center of motion; otherwise absolute CCW position.\n"
	"\n"
	"move12: Send a position command to MX Series servo(s).  If the position\n"
	"      argument is negative or has a leading '+', it will be interpreted\n"
	"      as relative to the center of motion; otherwise absolute CCW position.\n"
	"\n"
	"freeze: Tells the specified servos to hold their current positions.\n"
	"\n"	
	"relax:Sends a torque disable command to specified servos.\n"
	"\n"
	"read: Dump all registers for the specified servo, optionally specifying\n"
	"      beginning and ending register addresses (inclusive).\n"
	"\n"
	"write:Stores a value at the specified address for each of specified servos.\n"
	"      If multiple values are provided, they will be written to subsequent\n"
	"      registers.\n"
	"\n"
	" rot: rotate the specified servo continuously for a specified period\n"
	"      of time at a set speed. A negative speed indicates CCW direction;\n"
	"      a positive indicates CW. Duration is measured in milliseconds.\n"
	"\n"
	"perf: Benchmark performance testing, attempts to read and write from\n"
	"      specified servo(s) as quickly as possible until enter is pressed\n"
	"      or stdin is closed, then reports statistics.\n"
	"      If the 'sync' option is specified, a sync_write command will be used\n"
	"      instead of writing to each servo individually.\n"
	"      The 'save' option will cause raw sample data to be saved to the\n"
	"      specified file. The first line will be cycle end times, second line\n"
	"      will be read latencies, and third line will be write latencies.\n"
	"\n"
	"conf: displays the current configuration settings and then exits\n"
	"\n"
	"OPTIONS\n"
	"      -r,--read <file>        Reads configuration settings from <file>\n"
	"      -w,--write <file>       Writes final configuration settings to <file>\n"
	"                              (If <file> is 'default', saves at path below)\n"
	"      -c,--comm <commport>    Choose a comm port (defaults to SerialCommPort)\n"
	"      <key>=<value>           Assigns a new value to a configuration setting\n"
	"                              (overrides any settings read from file)\n"
	"\n"
	"      The setting file will store the comm port type.  If you specify a new\n"
	"      type with --comm, it will overwrite the previously read settings with\n"
	"      the defaults for the new comm type.\n"
	"      \n"
	"      Default configuration settings are read from:\n"
	"          " << DEFAULT_CONF << "\n"
	"      \n"
	"COMMPORTS\n"
	"      Available comm ports are:\n"
	"     ";
	dispCommPorts(os);
}

int main(int argc, const char* argv[]) {
	minisim::AutoScopeInit initTekkotsu;
	
	// usually we'll use a SerialCommPort if nothing else is specified...
	const char * const DEFAULT_COMMPORT="SerialCommPort";
	
	// parameters from command line
	string saveFile;
	string commport;
	bool needComm=false;
	enum command_t {
		GENERIC_COMMAND, CONF_COMMAND, NO_COMMAND
	} command=NO_COMMAND;
	map<string,string> assignments;
	
	// during processing, we'll load configuration files here
	// and then pass the final settings on to the comm port
	// (once we know what kind to use!)
	Dictionary conf;
	
	////////////////////////////////////
	// Process command line arguments //
	////////////////////////////////////
	if(argc<=1) {
		usage(cout,argv[0]);
		return 2;
	}
	
	int commandArg=-1;
	for(int i=1; i<argc; ++i) {
		string a = argv[i];
		
		if(a=="-h" || a=="--help") {
			usage(cout,argv[0]);
			return 0;
		
		} else if(a=="-r" || a=="--read") {
			if(i>=argc-1) {
				cerr << "ERROR: '" << a << "' requires an argument" << endl;
				return 2;
			}
			conf.loadFile(argv[++i]);
		
		} else if(a=="-w" || a=="--write") {
			if(i>=argc-1) {
				cerr << "ERROR: '" << a << "' requires an argument" << endl;
				return 2;
			}
			++i;
			saveFile = (string(argv[i])=="default") ? string_util::tildeExpansion(DEFAULT_CONF) : argv[i];
		
		} else if(a=="-c" || a=="--comm") {
			if(i>=argc-1) {
				cerr << "ERROR: '" << a << "' requires an argument" << endl;
				return 2;
			}
			commport = argv[++i];
			
		} else if(a=="conf" || a=="config") {
			if(i<argc-1) {
				cerr << "Extra arguments to 'conf': " << argv[i+1] << endl;
				return 2;
			}
			command=CONF_COMMAND;
		
		} else if(commands().find(a)!=commands().end()) {
			commandArg=i;
			needComm=true;
			command=GENERIC_COMMAND;
			break;
			
		} else {
			char * eq = const_cast<char*>(strchr(argv[i],'='));
			if(eq==NULL) {
				cerr << "Invalid argument '" << argv[i] << "', pass -h for help." << endl;
				return 2;
			} else {
				*eq++='\0';
				assignments[argv[i]]=eq;
			}
		}
	}
	
	if(conf.size()==0) {
		std::string defConf = string_util::tildeExpansion(DEFAULT_CONF);
		struct stat sb;
		if(stat(defConf.c_str(),&sb)==0)
			conf.loadFile(defConf.c_str());
	}
	
	
	////////////////////////////////////
	//      Construct CommPort        //
	////////////////////////////////////
	Dictionary::const_iterator dit = conf.findEntry(".type");
	if(dit!=conf.end()) {
		if(commport.size()==0)
			commport = dit->second->toString();
		else if(commport!=dit->second->toString())
			conf.clear();
	}
	if(commport.size()==0) {
		commport=DEFAULT_COMMPORT;
		conf.addValue("Baud",1000000);
		conf.addValue("Path","/dev/ttyUSB0");
	}
	
	CommPort * comm = CommPort::getRegistry().create(commport,commport);
	if(comm==NULL) {
		cerr << "Error creating comm port of type '" << commport << "'.  Known types are:\n";
		dispCommPorts(cerr);
		return 2;
	}
	comm->addEntry("Timeout",timeout,"Time to wait for servo response before giving up (milliseconds)");
	comm->set(conf); // copy over values we read from disk
	
	
	////////////////////////////////////
	//     Variable Assignments       //
	////////////////////////////////////
	for(map<string,string>::const_iterator it=assignments.begin(); it!=assignments.end(); ++it) {
		ObjectBase * ob = comm->resolveEntry(it->first);
		if(ob==NULL) {
			cerr << "Invalid configuration key '" << it->first << "', settings are:\n";
			plist::filteredDisplay(cerr,*comm,"^[^.].*",REG_EXTENDED,-1U);
			return 2;
		} else {
			if(PrimitiveBase * pb = dynamic_cast<PrimitiveBase*>(ob)) {
				pb->set(it->second);
			} else {
				cerr << "Cannot assign value to dictionary key '" << it->first << "', settings are:\n";
				plist::filteredDisplay(cerr,*comm,"^[^.].*",REG_EXTENDED,-1U);
				return 2;
			}
		}
	}
	DynamixelProtocol::PingThread::setTimeout(timeout);
	
	
	////////////////////////////////////
	//    Write Settings to Disk      //
	////////////////////////////////////
	if(saveFile.size()>0)
		comm->saveFile(saveFile.c_str());
	
	
	////////////////////////////////////
	//        Open CommPort           //
	////////////////////////////////////
	istream is(NULL);
	ostream os(NULL);
	
	struct CommAutoClose {
		CommAutoClose() : comm(NULL) {}
		~CommAutoClose() { if(comm!=NULL) comm->close(); }
		CommPort * comm;
	} autoCloseComm;
	
	if(needComm) {
		cout << "Opening comm port..." << flush;
		if(!comm->open() || !comm->isWriteable() || !comm->isReadable()) {
			cerr << "Could not open comm port" << endl;
			comm->close();
			return 1;
		}
		autoCloseComm.comm=comm;
		
		is.rdbuf(&comm->getReadStreambuf());
		if(!comm->isReadable()) {
			cerr << "Could not open comm port for reading" << endl;
			return 1;
		}
		is.clear();
		is.exceptions(ios_base::badbit);
		
		os.rdbuf(&comm->getWriteStreambuf());
		if(!comm->isWriteable()) {
			cerr << "Could not open comm port for writing" << endl;
			return 1;
		}
		cout << "done!" << endl;
	}
	
	
	////////////////////////////////////
	//          Run Command           //
	////////////////////////////////////
	if(command!=NO_COMMAND && command!=CONF_COMMAND && commandArg<0) {
		cerr << "Internal error, commandArg was not set" << endl;
		return EXIT_FAILURE;
	}
	
	switch(command) {
		
	case GENERIC_COMMAND:
		return (commands()[argv[commandArg]])(is, os, argc-commandArg, &argv[commandArg]);
	
	case CONF_COMMAND:
		plist::filteredDisplay(cerr,*comm,"^[^.].*",REG_EXTENDED,-1U);
		break;
	
	case NO_COMMAND:
		usage(cout,argv[0]);
		break;
	}
	
	
	////////////////////////////////////
	//           Cleanup              //
	////////////////////////////////////
	
	// rely on destructors for cleanup :)
	return 0;
}


////////////////////////////////////
//        Helper Functions        //
////////////////////////////////////

bool registerCommand(const std::string& name, cmd_t cmd) {
	commands()[name]=cmd;
	return true;
}

void dispErrors(ostream& os, unsigned int error) {
	for(size_t i=0; i<8*sizeof(error); ++i) {
		if(error & (1<<i)) {
			os << ' ' << DynamixelProtocol::ResponseErrorNames[i];
		}
	}
}

void* ReadResponseThread::run() {
    FailsafeThread failsafe(*this,failsafeTime,true);
	is.read(resp,sizeof(resp));
	testCancel();
	if(!failsafe.isEngaged()) 
		failsafe.stop();
	if(!is || is.gcount()!=sizeof(resp)) {
		cerr << "ERROR: bad read from comm port (read " << is.gcount() << " of " << sizeof(resp) << " bytes)" << endl;
		return NULL;
	} else if(!DynamixelProtocol::validate(resp)) {
		cerr << "ERROR: Received response with bad checksum, cannot verify successful '" << errstr << "'" << endl;
		return NULL;
	} else if(resp.servoid!=srcID) {
		cerr << "ERROR: Got a '" << errstr <<
			"' response from wrong servo (got "<<(int)resp.servoid
				 <<", wanted original id " <<(int)srcID<<")" << endl;
		return NULL;
	} else if(resp.error & (DynamixelProtocol::CHECKSUM_ERROR | DynamixelProtocol::INSTRUCTION_ERROR)) {
		cerr << "ERROR: Received response from '" << errstr << "' on " << (int)resp.servoid
				 << ", but with error(s) " << (int)resp.error << ":";
		dispErrors(cerr,resp.error);
		cerr << endl;
		return NULL;
	} else if(resp.error!=0) {
		cerr << "WARNING: Received response from '" << errstr << "' on " << (int)resp.servoid << ", but with alarm condition(s) " << (int)resp.error << ":";
		dispErrors(cerr,resp.error);
		cerr << endl;
	}
	return &resp;
}
