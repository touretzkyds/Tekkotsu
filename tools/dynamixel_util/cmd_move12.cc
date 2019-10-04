#include "dynamixel_util.h"

using namespace std;

int cmd_move12(std::istream& is, std::ostream& os, const std::vector<std::pair<servoid,servoid> >& moveRanges, unsigned short movePos, unsigned short moveSpeed);

//! this version does the parsing, then calls the other version for execution
int cmd_move12(std::istream& is, std::ostream& os, int argc, const char* argv[]) {
	vector<pair<servoid,servoid> > moveRanges;
	unsigned short movePos=0;
	unsigned short moveSpeed=0;
	
	int i=1;
	if(argc-i<2) {
		cerr << "ERROR: '" << argv[0] << "' requires servoid id parameter and target position, and optional speed" << endl;
		return 2;
	}
	
	if(strcmp(argv[i],"all")==0) {
		moveRanges.push_back(make_pair(DynamixelProtocol::BROADCAST_ID,DynamixelProtocol::BROADCAST_ID));
		++i;
	} else try {
		i += readRanges(1,&argv[i],moveRanges,argv[0]);
	} catch(...) {
		return 2;
	}
	
	const char* moveAmountStr = argv[i++];
	bool centered = ( *moveAmountStr=='+' || *moveAmountStr=='-');
	int moveAmount=0;
	if(!(stringstream(moveAmountStr)>>moveAmount)) {
		cerr << "ERROR: bad move target value " << moveAmountStr << endl;
		return 2;
	}
	if ( !centered ) {
		if ( std::abs(moveAmount) < 4096 ) {
			movePos = moveAmount;
		} else {
			cerr << "ERROR: move value must be between 0 and 4095" << endl;
			return 2;
		}
	} else if ( moveAmount > 2047 ) {
		cerr << "ERROR: signed moves must be between 0 and 2048" << endl;
		return 2;
	} else {
		movePos = moveAmount + 2048;
	}
	moveSpeed = ( i >= argc ) ? 128 : atoi(argv[i++]);
	
	return cmd_move12(is, os, moveRanges, movePos, moveSpeed);
}

int cmd_move12(std::istream& is, std::ostream& os, const std::vector<std::pair<servoid,servoid> >& moveRanges, unsigned short movePos, unsigned short moveSpeed) {
	for(vector<pair<servoid,servoid> >::const_iterator it=moveRanges.begin(); it!=moveRanges.end(); ++it) {
		for(unsigned int sid=it->first; sid<=it->second; ++sid) {
			if(sid==DynamixelProtocol::BROADCAST_ID) {
				std::cout << "Moving all servo to " << movePos << " with speed " << moveSpeed << endl;
				DynamixelProtocol::write(os,DynamixelProtocol::SetPosSpeedCmd(sid,movePos,moveSpeed)).flush(); 
				// no response from a broadcast
			} else {
				std::cout << "Moving servo " << int(sid) << " to " << movePos << " with speed " << moveSpeed << endl;
				DynamixelProtocol::write(os,DynamixelProtocol::SetPosSpeedCmd(sid,movePos,moveSpeed)).flush(); 
				void * res = ReadResponseThread(is,timeout,sid,"move12").join();
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

static bool registered = registerCommand("move12",cmd_move12);
