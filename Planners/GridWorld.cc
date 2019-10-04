#include "Planners/GridWorld.h"
#include <cstdlib>

const float GridWorld::HCOST=1;
const float GridWorld::VCOST=2;
//const float GridWorld::DCOST=0; // no diagonals
//const float GridWorld::DCOST=HCOST+VCOST; // equal cost
//const float GridWorld::DCOST=std::min(HCOST,VCOST)/2; // low cost
//const float GridWorld::DCOST=(HCOST+VCOST)/2; // avg cost
const float GridWorld::DCOST=hypotf(HCOST,VCOST); // cartesian

float GridWorld::heuristic(const State& st, const State& goal) const {
	float dx=std::abs((ssize_t)(goal.c-st.c));
	float dy=std::abs((ssize_t)(goal.r-st.r));
	
	if(DCOST<=0 || DCOST>=HCOST+VCOST)
		return dx*HCOST + dy*VCOST; // manhattan distance (no diagonals)
	
	if(DCOST<(HCOST+VCOST)/2)
		return std::max(dx,dy)*DCOST;
	
	if(DCOST<hypotf(HCOST,VCOST))
		return (dx>dy) ? dy*DCOST+(dx-dy)*HCOST : dx*DCOST+(dy-dx)*VCOST; // low cost diagonals
	
	return hypotf(dx*HCOST, dy*VCOST); // cartesian space (assume sqrt(2) diagonals)
}

const std::vector<std::pair<float,GridWorld::State> >& GridWorld::expand(const State* /*parent*/, const State& st, const State& /*goal*/) const {
	using std::make_pair;
	static std::vector<std::pair<float,GridWorld::State> > neighbors;
	neighbors.resize(0);
	if(st.r>0 && isspace(world[st.r-1][st.c])) {
		neighbors.push_back(make_pair(VCOST,State(st.r-1,st.c)));
		if(DCOST>0) {
			// diagonals:
			if(st.c>0 && isspace(world[st.r-1][st.c-1]))
				neighbors.push_back(make_pair(DCOST,State(st.r-1,st.c-1)));
			if(st.c<world[st.r-1].size()-1 && isspace(world[st.r-1][st.c+1]))
				neighbors.push_back(make_pair(DCOST,State(st.r-1,st.c+1)));
		}
	}
	if(st.r<world.size()-1 && isspace(world[st.r+1][st.c])) {
		neighbors.push_back(make_pair(VCOST,State(st.r+1,st.c)));
		if(DCOST>0) {
			// diagonals:
			if(st.c>0 && isspace(world[st.r+1][st.c-1]))
				neighbors.push_back(make_pair(DCOST,State(st.r+1,st.c-1)));
			if(st.c<world[st.r+1].size()-1 && isspace(world[st.r+1][st.c+1]))
				neighbors.push_back(make_pair(DCOST,State(st.r+1,st.c+1)));
		}
	}
	if(st.c>0 && isspace(world[st.r][st.c-1]))
		neighbors.push_back(make_pair(HCOST,State(st.r,st.c-1)));
	if(st.c<world[st.r].size()-1 && isspace(world[st.r][st.c+1]))
		neighbors.push_back(make_pair(HCOST,State(st.r,st.c+1)));
	
	return neighbors;
}

std::istream& operator>>(std::istream& is, GridWorld& gw) {
	gw.world.clear();
	std::string str;
	while(std::getline(is,str))
		gw.world.push_back(std::vector<char>(str.begin(),str.end()));
	return is;
}

std::ostream& operator<<(std::ostream& os, const GridWorld& gw) {
	using std::setw;
	const size_t WIDTH=4;
	os << setw(WIDTH) << ' ' << "  ";
	for(size_t c=0; c<gw.world[0].size(); ++c)
		os << (c%10);
	os << '\n' << setw(WIDTH) << ' ' << ' ';
	for(size_t c=0; c<gw.world[0].size()+2; ++c)
		os << "X";
	os << '\n';
	for(size_t r=0; r<gw.world.size(); ++r) {
		os << setw(WIDTH) << r << ' ';
		os << "X";
		for(size_t c=0; c<gw.world[r].size(); ++c) {
			os << gw.world[r][c];
		}
		os << "X\n";
	}
	os << setw(WIDTH) << ' ' << ' ';
	for(size_t c=0; c<gw.world[0].size()+2; ++c)
		os << "X";
	os << '\n';
	return os;
}

