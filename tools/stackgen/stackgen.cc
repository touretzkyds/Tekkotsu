#include <iostream>
#include <sstream>
#include <stdexcept>
#include <cassert>
#include <ctime>
#include <cmath>
#include <vector>

#include "Shared/fmat.h"
#include "Wireless/netstream.h"
#include "Motion/KinematicJoint.h"

using namespace std;

static const float DEFAULT_DIM=75;
static const float DEFAULT_GAP=25;
static const float DEFAULT_HEIGHT=75;
static const float DEFAULT_MASS=0.3;
static const size_t DEFAULT_LEVELS=5;

float blockD=DEFAULT_DIM;
float blockH=DEFAULT_HEIGHT;
float blockG=DEFAULT_GAP;
float blockMass=DEFAULT_MASS;
fmat::Column<3> pos;
float ori=0;
size_t levels = DEFAULT_LEVELS;
std::string mirageHost = "localhost:19785";
bool clearBlocks=false;
std::vector<std::string> materials;

void usageAndExit(const std::string& name, const std::string& error="") {
	if(error.size()>0)
		std::cerr << "ERROR: " << error << std::endl;
	std::cerr << "Usage: " << name << " <OPTIONS>\n"
	"\n"
	"    -d, --dimension <n>          Dimension of the blocks (mm) [" << DEFAULT_DIM << "]\n"
	"    -g, --gap <n>                Gap between blocks (mm) [" << DEFAULT_GAP << "]\n"
	"    -h, --height <n>             Height of each block (mm) [" << DEFAULT_HEIGHT << "]\n"
	"    -l, --levels <n>             Number of levels to create [" << DEFAULT_LEVELS << "]\n"
	"    -p, --position <n> <n> [<n>] Position of stack (mm) [0 0 0]\n"
	"    -o, --orientation <n>        Orientation of stack about z (degrees) [0]\n"
	"    -w, --mass                   Mass of each block [" << DEFAULT_MASS << "]\n"
	"    -m, --mirage [host[:port]]   Display maze in Mirage [localhost:19785]\n"
	"    -c, --clear                  Clears the blocks from Mirage\n"
	"        --materials <name> [...] List of material names to be used in sequence [Wood]\n"
	<< std::flush;
	exit(error.size()>0 ? 2 : 0);
}

class argument_error : public std::exception {
public:
	argument_error(const std::string& _msg, const std::string& _arg) : std::exception(), msg(_msg), arg(_arg) {}
	virtual ~argument_error() throw() {}
	virtual const char* what() const throw() {
		return ("Bad "+msg+" argument: '" + arg + "'").c_str();
	}
	const std::string msg;
	const std::string arg;
};

int main(int argc, char* argv[]) {
	// ********************************* //
	//        Process command line       //
	// ********************************* //
	try {
		for(int x=1; x<argc; x++) {
			if(strcmp(argv[x],"-d")==0 || strcmp(argv[x],"--dimension")==0) {
				if(x==argc-1)
					throw runtime_error("dimension must take an argument");
				if(!(stringstream(argv[++x])>>blockD))
					throw argument_error("dimension",argv[x]);
			} else if(strcmp(argv[x],"-g")==0 || strcmp(argv[x],"--gap")==0) {
				if(x==argc-1)
					throw runtime_error("gap must take an argument");
				if(!(stringstream(argv[++x])>>blockG))
					throw argument_error("gap",argv[x]);
			} else if(strcmp(argv[x],"-h")==0 || strcmp(argv[x],"--height")==0) {
				if(x==argc-1)
					throw runtime_error("height must take an argument");
				if(!(stringstream(argv[++x])>>blockH))
					throw argument_error("height",argv[x]);
			} else if(strcmp(argv[x],"-w")==0 || strcmp(argv[x],"--mass")==0) {
				if(x==argc-1)
					throw runtime_error("mass must take an argument");
				if(!(stringstream(argv[++x])>>blockMass))
					throw argument_error("mass",argv[x]);
			} else if(strcmp(argv[x],"-l")==0 || strcmp(argv[x],"--levels")==0) {
				if(x==argc-1)
					throw runtime_error("levels must take an argument");
				if(!(stringstream(argv[++x])>>levels))
					throw argument_error("levels",argv[x]);
			} else if(strcmp(argv[x],"-m")==0 || strcmp(argv[x],"--mirage")==0) {
				if(x==argc-1 || argv[x+1][0]=='-') {
					mirageHost = "localhost:19785";
				} else if(strchr(argv[x+1],':')==NULL) {
					mirageHost = argv[++x];
					mirageHost += ":19785";
				} else {
					mirageHost = argv[++x];
				}
			} else if(strcmp(argv[x],"-p")==0 || strcmp(argv[x],"--position")==0) {
				if(x>=argc-2)
					throw runtime_error("position must specify at least two arguments");
				if(!(stringstream(argv[++x])>>pos[0]))
					throw argument_error("position x",argv[x]);
				if(!(stringstream(argv[++x])>>pos[1]))
					throw argument_error("position y",argv[x]);
				if(x<argc-1 && argv[x+1][0]!='-')
					if(!(stringstream(argv[++x])>>pos[2]))
						throw argument_error("position z",argv[x]);
			} else if(strcmp(argv[x],"-o")==0 || strcmp(argv[x],"--orientation")==0) {
				if(x==argc-1)
					throw runtime_error("orientation must take an argument");
				if(!(stringstream(argv[++x])>>ori))
					throw argument_error("orientation",argv[x]);
				ori*=(float)M_PI/180;
			} else if(strcmp(argv[x],"--materials")==0) {
				if(x==argc-1 || argv[x+1][0]=='-')
					throw runtime_error("materials must take at least one argument");
				while(x<argc-1 && argv[x+1][0]!='-')
					materials.push_back(std::string(argv[++x]));
			} else if(strcmp(argv[x],"-c")==0 || strcmp(argv[x],"--clear")==0) {
				clearBlocks=true;
			} else if(strcmp(argv[x],"--help")==0) {
				usageAndExit(argv[0]);
			} else {
				throw argument_error("unknown",argv[x]);
			}
		}
	} catch(const std::exception& ex) {
		usageAndExit(argv[0],ex.what());
	}
	
	if(materials.size()==0)
		materials.push_back("Wood");
	
	std::cout << "Stack " << levels << " levels, total " << levels*blockD + (levels-1)*blockG << " wide by " << levels*blockH << " high." << std::endl;
	std::cout << "Placed at (" << pos[0] << ',' << pos[1] << ',' << pos[2] << ") rotated " << ori << " radians" << std::endl;
	
	ionetstream mirage;
	
	// open connection
	std::cout << "Connecting..." << std::flush;
	if(!mirage.open(mirageHost)) {
		std::cout << "refused." << std::endl;
		return EXIT_FAILURE;
	}
	std::cout << "connected " << mirage.getPeerAddress().get_rname() << ":" << mirage.getPeerAddress().get_port() << std::endl;
	mirage << "<messages>\n";
	
	for(size_t y=0; y<levels; ++y) {
		const size_t n = levels-y;
		size_t materialCnt=materials.size()+y;
		for(size_t x=0; x<n; ++x) {
			// this will hold the keys and values we want to send to mirage
			plist::Dictionary msg;
	
			// name for the maze within mirage
			std::stringstream idss;
			idss << "stack-" << x << ',' << y;
			msg.addEntry("ID",new plist::Primitive<std::string>(idss.str()));
	
			// position of the block
			const float w = (blockG + blockD) * (n-1);
			fmat::Column<3> blockPos;
			blockPos[0] = x*(blockG + blockD) - w/2;
			blockPos[1] = 0;
			blockPos[2] = y*blockH + blockH/2;
			blockPos = fmat::Quaternion::aboutZ(ori)*blockPos + pos;
			plist::ArrayOf<plist::Primitive<float> > location(3,0);
			blockPos.exportTo(location);
			msg.addEntry("Location",location);
			std::cout << "block " << x << ' ' << y << " @ " << blockPos << std::endl;
	
			// this keeps the maze instantiated in Mirage even when we close our connection
			msg.addEntry("Persist",new plist::Primitive<bool>(!clearBlocks));
	
			// create the maze walls as components of a immobile kinematic joint
			KinematicJoint block;
			block.mass = blockMass;
			block.model = "CollisionModel";
			block.material = materials[materialCnt=(materialCnt+1)%materials.size()];
			block.collisionModel = "Cube";
			block.collisionModelScale[0]=block.collisionModelScale[1]=blockD;
			block.collisionModelScale[2]=blockH;
			msg.addEntry("Model",new KinematicJointSaver(block));
			
			msg.saveStream(mirage,true);
		}
	}
	
	// write the message and exit
	mirage << "</messages>";
	mirage.flush();
	mirage.close();
}

