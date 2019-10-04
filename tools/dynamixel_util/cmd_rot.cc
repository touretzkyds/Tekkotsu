#include "dynamixel_util.h"
#include <numeric>
#include <unistd.h>  // for usleep
using namespace std;

int cmd_rot(std::istream& is, std::ostream &os, int argc, const char* argv[]) {
    if (argc < 2) {
        cerr << "ERROR: '" << argv[0] << "' requires servo id parameter" << endl;
        return 2;
    }

    unsigned int sid = atoi(argv[1]);
    if (sid > DynamixelProtocol::MAX_ID || sid < 0) {
        cerr << "ERROR: '" << argv[0] << "' source id is out of range (0-" << DynamixelProtocol::MAX_ID << ")" << endl;
	return 2;
    }
    if (!os || !is) {
        cerr << "ERROR: No connection! Check path setting." << endl;
        return 2;
    }

    int speed = 0;
    int duration = 0;

    if (argc > 2) {
        speed = atoi(argv[2]);
	duration = atoi(argv[3]);
    }

    // set servo to wheel mode
    int zerodata = 0; // 4 0 bytes
    DynamixelProtocol::WriteHeader header(0x6, 4);
    header.servoid = sid;
    os.write(header,sizeof(header));
    os.write((char*)(&zerodata),4);
    unsigned char checksum = std::accumulate((char*)(&zerodata),(char*)(&zerodata)+3,0);
    checksum = ~( nchecksum(header,sizeof(header)) + checksum );
    os.write((char*)&checksum,1);
    os.flush();
    if (!os) {
        cerr << "ERROR: lost communications." << endl;
	return 2;
    }
    void *res = ReadResponseThread(is, timeout, sid, "write").join();
    if (res == Thread::CANCELLED) {
        cerr << "ERROR: no response, servo may be disconnected" << endl;
	is.clear();
	return 2;
    }

    // format speed/direction info and start spinning
    short dir = 0;
    if (speed < 0) {
        speed *= -1;
    } else {
        dir = 0b10000000000;

    }
    if (speed > 1023) {
        speed = 1023;
    }
    short data = ((short)speed) | dir;
    DynamixelProtocol::write(os, DynamixelProtocol::SetPosSpeedCmd(sid,(short)0,data)).flush();
    res = ReadResponseThread(is,timeout,sid,"rot").join();
    if (res == Thread::CANCELLED) {
        cerr << "ERROR: no response, servo may be disconnected" << endl;
	is.clear();
	return 2;
    }

    if (duration != 0) {
        // sleep for however long they asked us to spin for, or as close as is possible
        usleep(duration * 1000);

	// now we are awake, we should stop the servo
	data = 0;
	DynamixelProtocol::write(os, DynamixelProtocol::SetPosSpeedCmd(sid,(short)0,data)).flush();
	res = ReadResponseThread(is,timeout,sid,"rot").join();
	if (res == Thread::CANCELLED) {
            cerr << "ERROR: no response, servo may be disconnected" << endl;
	    is.clear();
	    return 2;
	}
    }

    return EXIT_SUCCESS;
}

static bool registered = registerCommand("rot", cmd_rot);
