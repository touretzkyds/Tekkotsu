#ifndef __KEYGROUP_H
#define __KEYGROUP_H

#include <vector>
#include <fstream>

class keypoint;
class model;

class keygroup{
private:
	int id;
public:
	keygroup(int ID=getNewKeygroupID());
  	
	model *M;
	std::vector<keygroup*> neighbors;
	std::vector<keypoint*> keypts;
  	
	bool isNeighbor(keygroup *g);
	int compareTo(std::vector<keypoint*>* keypoints);
	int compareTo(keygroup *g);		
	keypoint* bestMatchInGroup(keypoint* key, double* sqDist);
  	
	int getID() const { return id; }
	
	static int getNewKeygroupID() { return keygroupID++; }
	
	void writeToFile(std::ofstream& outfile);
	
private:
	static int keygroupID;
	
	keygroup(const keygroup&); // Do not use
	keygroup& operator=(const keygroup&); // Do not use
};

#endif
