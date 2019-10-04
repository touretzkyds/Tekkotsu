#include <iostream>
#include "Planners/AStar.h"
#include "Planners/GridWorld.h"

using namespace std;
using namespace AStar;

int main(int argc, char* argv[]) {
	if(argc!=5) {
		cerr << "Usage: " << argv[0] << " start_r start_c goal_r goal_c" << endl;
		return EXIT_FAILURE;
	}
	GridWorld::State initial, goal;
	initial.r = atoi(argv[1]);
	initial.c = atoi(argv[2]);
	goal.r = atoi(argv[3]);
	goal.c = atoi(argv[4]);
	
	cout << "Reading map from stdin..." << endl;
	GridWorld gw;
	cin >> gw;
	cout << "Map:" << endl;
	gw[initial]='S';
	gw[goal]='G';
	cout << gw;
	gw[initial]=gw[goal]=' ';
	
	Results<GridWorld::State> res = astar(gw,initial,goal);
	
	for(Results<GridWorld::State>::set_iterator it=res.open.begin(); it!=res.open.end(); ++it) {
		gw[(*it)->state] = isspace(gw[(*it)->state]) ? '?' : '0';
	}
	for(Results<GridWorld::State>::set_iterator it=res.closed.begin(); it!=res.closed.end(); ++it) {
		gw[(*it)->state] = isspace(gw[(*it)->state]) ? '.' : '^';
	}
	for(Results<GridWorld::State>::path_iterator it=res.path.begin(); it!=res.path.end(); ++it) {
		gw[*it] = gw[*it]=='.' ? '*' : 'q';
	}
	gw[initial]='S';
	gw[goal]='G';
	cout << gw;
	cout << "Nodes expanded: " << res.closed.size()+res.open.size() << " closed " << res.closed.size() << " open " << res.open.size() << endl;
	cout << "Path cost: " << res.priorities.front()->cost << endl;
	
	return EXIT_SUCCESS;
}