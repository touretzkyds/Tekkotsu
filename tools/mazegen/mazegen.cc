#include <iostream>
#include <sstream>
#include <stdexcept>
#include <cassert>
#include <ctime>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "mazegen.h"

using namespace std;

static const size_t DEFAULT_WIDTH=10;
static const size_t DEFAULT_HEIGHT=10;
static const float DEFAULT_CONNECT=1;

float cellDim=1000;
float wallHeight=650;
float wallDepth=50;
std::string mirageHost;
size_t startX=-1U,startY=-1U;

void usageAndExit(const std::string& name, const std::string& error="") {
	if(error.size()>0)
		std::cerr << "ERROR: " << error << std::endl;
	std::cerr << "Usage: " << name << " <OPTIONS>\n"
	"\n"
	"    -w, --width <n>            Width of the maze [" << DEFAULT_WIDTH << "]\n"
	"    -h, --height <n>           Height of the maze [" << DEFAULT_HEIGHT << "]\n"
	"    -c, --connect <x>          Floating point value:\n"
	"                                 0 - empty space\n"
	"                                 1 - fully connected hallways [default]\n"
	"                                 >1 - 'n' separate regions\n"
	"    -u, --human                Display human readable format\n"
	"    -m, --mirage [host[:port]] Display maze in Mirage (with optional port)\n"
	"    -s, --seed <n>             Specify random number seed [clock]\n"
	"    -i, --start <n:0-width>\\   Specify initial starting position [random]\n"
	"                <n:0-height>\n"
	<< std::flush;
	exit(error.size()>0 ? 2 : 0);
}

class argument_error : public std::exception {
public:
	argument_error(const std::string& _msg, const std::string& _arg) : msg(_msg), arg(_arg) {}
	virtual ~argument_error() throw() {}
	virtual const char* what() const throw() {
		return ("Bad "+msg+" argument: '" + arg + "'").c_str();
	}
	const std::string msg;
	const std::string arg;
};

template<typename T>
inline T pickRandom(T range) {
	double rnd = random()/double(1U<<31);
	return static_cast<T>(rnd*range);
}

void printMap(std::ostream& os, map_t m);
void sendMazeMirage(map_t map);

int main(int argc, char* argv[]) {
	size_t width=DEFAULT_WIDTH;
	size_t height=DEFAULT_HEIGHT;
	float connect=DEFAULT_CONNECT;
	bool humanReadable=false;
	bool setSeed=false;
	unsigned int seed=time(NULL);
	bool rndStart=true;
	
	// ********************************* //
	//        Process command line       //
	// ********************************* //
	try {
		for(int x=1; x<argc; x++) {
			if(strcmp(argv[x],"-w")==0 || strcmp(argv[x],"--width")==0) {
				if(x==argc-1)
					throw runtime_error("width must take an argument");
				if(!(stringstream(argv[++x])>>width))
					throw argument_error("width",argv[x]);
			} else if(strcmp(argv[x],"-h")==0 || strcmp(argv[x],"--height")==0) {
				if(x==argc-1)
					throw runtime_error("height must take an argument");
				if(!(stringstream(argv[++x])>>height))
					throw argument_error("height",argv[x]);
			} else if(strcmp(argv[x],"-c")==0 || strcmp(argv[x],"--connect")==0) {
				if(x==argc-1)
					throw runtime_error("connect must take an argument");
				if(!(stringstream(argv[++x])>>connect))
					throw argument_error("connect",argv[x]);
			} else if(strcmp(argv[x],"-u")==0 || strcmp(argv[x],"--human")==0) {
				humanReadable=true;
			} else if(strcmp(argv[x],"-m")==0 || strcmp(argv[x],"--mirage")==0) {
				if(x==argc-1 || argv[x+1][0]=='-') {
					mirageHost = "localhost:19785";
				} else if(strchr(argv[x+1],':')==NULL) {
					mirageHost = argv[++x];
					mirageHost += ":19785";
				} else {
					mirageHost = argv[++x];
				}
			} else if(strcmp(argv[x],"-s")==0 || strcmp(argv[x],"--seed")==0) {
				if(x==argc-1)
					throw runtime_error("seed must specify an argument");
				if(!(stringstream(argv[++x])>>seed))
					throw argument_error("seed",argv[x]);
				setSeed=true;
			} else if(strcmp(argv[x],"-i")==0 || strcmp(argv[x],"--start")==0) {
				if(x>=argc-2)
					throw runtime_error("start must specify two arguments");
				rndStart=false;
				if(!(stringstream(argv[++x])>>startX))
					throw argument_error("x start",argv[x]);
				if(!(stringstream(argv[++x])>>startY))
					throw argument_error("y start",argv[x]);
			} else if(strcmp(argv[x],"--help")==0) {
				usageAndExit(argv[0]);
			} else {
				throw argument_error("unknown",argv[x]);
			}
		}
	} catch(const std::exception& ex) {
		usageAndExit(argv[0],ex.what());
	}
	
	srandom(seed);
	if(!setSeed)
		std::cout << "Seed " << seed << std::endl;
	if(startX>=width || startY>=height) {
		startX = pickRandom(width);
		startY = pickRandom(height);
		std::cout << "Start position is " << startX << ',' << startY << std::endl;
	}
	
	
	// ********************************* //
	//         Generate the maze         //
	// ********************************* //
	map_t map(height);
	for(map_t::iterator it=map.begin(); it!=map.end(); ++it)
		it->resize(width);
	
	const size_t INTERIOR_WALLS=width*height*2-width-height; //number of interior walls
	const size_t MAZE_WALLS=INTERIOR_WALLS-width*height+1; //number of interior walls in a ideal maze
	size_t walls=MAZE_WALLS;
	if(connect<1)
		walls=static_cast<size_t>(connect*walls+.5f);
	if(connect>1)
		walls+=static_cast<size_t>(connect-.5f);
	
	size_t wallsRemain=INTERIOR_WALLS;
	//first remove only 'safe' walls to generate ideal maze
	while(wallsRemain>MAZE_WALLS && wallsRemain>walls) {
		size_t x = pickRandom(width*height);
		size_t r = x/width;
		size_t c = x-r*width;
		if(pickRandom(2)) {
			// remove bottom?
			if(r==height-1)
				continue; // bottom row can't remove bottom
			if(&map[r][c].findRoot()!=&map[r+1][c].findRoot()) {
				map[r][c].unionBottom(map[r+1][c]);
				--wallsRemain;
			}
		} else {
			// remove right?
			if(c==width-1)
				continue; // right column can't remove right
			if(&map[r][c].findRoot()!=&map[r][c+1].findRoot()) {
				map[r][c].unionRight(map[r][c+1]);
				--wallsRemain;
			}
		}
	}
	//now remove additional walls from the ideal maze, if any
	for(;wallsRemain>walls;wallsRemain--) {
		while(true) {
			size_t x = pickRandom(width*height);
			size_t r = x/width;
			size_t c = x-r*width;
			if(pickRandom(2)) {
				// remove bottom?
				if(r==height-1)
					continue; // bottom right can't remove anything
				if(map[r][c].bottomWall()) {
					map[r][c].unionBottom(map[r+1][c]);
					break;
				}
			} else {
				// remove right
				if(c==width-1)
					continue; // right column can't remove right
				if(map[r][c].rightWall()) {
					map[r][c].unionRight(map[r][c+1]);
					break;
				}
			}
		}
	}
	
	if(humanReadable) {
		printMap(cout,map);
	}
	if(mirageHost.size()>0) {
		sendMazeMirage(map);
	}
}

void printMap(std::ostream& os, map_t m) {
	const size_t height = m.size();
	const size_t width = (height==0) ? 0 : m[0].size();
	
	const std::string VERTWALL="|";
	const std::string HORZWALL="--";
	const std::string CORNERINT="+";
	const std::string CORNERHORZ="-";
	const std::string CORNERVERT="|";
	const std::string EMPTYVERT=" ";
	const std::string EMPTYHORZ="  ";
	const std::string EMPTYCORNER=".";
	const std::string EMPTYCELL="  ";
	
	os << (CORNERINT);
	for(size_t c=0; c<width; c++) {
		os << HORZWALL << (m[0][c].rightWall()?CORNERINT:CORNERHORZ);
	}
	os << '\n';
	for(size_t r=0; r<height; r++) {
		os << VERTWALL;
		for(size_t c=0; c<width; c++)
			os << EMPTYCELL << (m[r][c].rightWall()?VERTWALL:EMPTYVERT);
		os << '\n';
		if(r<height-1 && !m[r][0].bottomWall())
			os << CORNERVERT;
		else
			os << CORNERINT;
		for(size_t c=0; c<width; c++) {
			if(m[r][c].bottomWall()) {
				os << HORZWALL;
				if(!m[r][c].rightWall() && (r==height-1 || !m[r+1][c].rightWall()))
					os << CORNERHORZ;
				else 
					os << CORNERINT;
			} else {
				os << EMPTYHORZ;
				if(c<width-1 && m[r][c+1].bottomWall()) {
					if(!m[r][c].rightWall() && (r==height-1 || !m[r+1][c].rightWall()))
						os << CORNERHORZ;
					else
						os << CORNERINT;
				} else {
					if(m[r][c].rightWall() || (r==height-1 || m[r+1][c].rightWall()))
						os << CORNERVERT;
					else
						os << EMPTYCORNER;
				}
			}
		}
		os << '\n';
	}
}

