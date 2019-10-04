#include "keygroup.h"
#include "keypoint.h"
#include "model.h"
#include "object.h"
#include <limits>

int keygroup::keygroupID = 0;

keygroup::keygroup(int ID) : id(ID), M(NULL), neighbors(), keypts() {}

bool keygroup::isNeighbor(keygroup *g) {
	for (int i = 0; i < (int)neighbors.size(); i++)
		if (g == neighbors[i]) return true;
	
	return false;
}
int keygroup::compareTo(keygroup *g) {
	double sum = 0;
	for(size_t i = 0; i < keypts.size(); i++)
		sum = sum + (*keypts[i]).sqDist(*g->keypts[i]);
	return (int)sum;
}
int keygroup::compareTo(std::vector<keypoint*>* keypoints) {
	double sum = 0;
	for(size_t i = 0; i < keypts.size(); i++)
		sum = sum + (*keypts[i]).sqDist(*(*keypoints)[i]);
	return (int)sum;
}
keypoint* keygroup::bestMatchInGroup(keypoint* key, double* sqDist){
	keypoint *bestKey = NULL;
	double bestSqDist = std::numeric_limits<double>::infinity();
	for (int i = 0; i < (int)keypts.size(); i++){
		double d = key->sqDist(*(keypts[i]));
		if (d < bestSqDist){
			bestSqDist = d;
			bestKey = keypts[i];
		}
	}
	*sqDist = bestSqDist;
	return bestKey;
}

void keygroup::writeToFile(std::ofstream& outfile){
	outfile << id << std::endl;
	outfile << M->O->getID() << "\t" << M->getID() << std::endl;
	outfile << neighbors.size() << std::endl;
	for (unsigned int i = 0; i < neighbors.size(); i++){
		outfile << neighbors[i]->M->O->getID() << "\t" << neighbors[i]->M->getID() << "\t" << neighbors[i]->getID() << "\t";
	}
	outfile << std::endl;
	outfile << keypts.size() << std::endl;
	for (unsigned int i = 0; i < keypts.size(); i++){
		outfile << keypts[i]->getID() << "\t";
	}
	outfile << std::endl;
}
