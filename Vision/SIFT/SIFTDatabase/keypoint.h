#ifndef __KEYPOINT_H
#define __KEYPOINT_H

#include <vector>
#include <fstream>

#define KEYPOINTDIM 128
#define MAXIMAGEDIM 320

class keygroup;

class keypoint {
private:
	int id;
	
public:
	//		bool valid;
	keygroup* G;
	double modelX, modelY, modelScale, modelOrientation;
	double imageX, imageY, imageScale, imageOrientation;
	int imageIntScale, imageIntOctave;
	std::vector<double> desc;
	
	int generation;
	
	std::vector<double>    bestDist;
	std::vector<keypoint*> bestMatch;
	
	keypoint* finalMatch;
	std::vector<keypoint*> matches;
	
	std::vector<keypoint*> modelMatches;
	std::vector<double>    modelMatchErrors;
	
	bool isInlier;
	
	keypoint(int ID=getNewKeypointID());
	
	static int getNewKeypointID() { return keypointID++; }
	
	double sqDist(keypoint& key);
	
	void print();
	
	void writeToFile(std::ofstream& outfile, bool indicateDim);
	void writeToFile(std::ofstream& outfile);
	
	int getID() const { return id; }
	
	static int getKeypointID() { return keypointID; }
	static void setKeypointID(int kid) { keypointID = kid; }
	
private:
	static int keypointID;
	keypoint(const keypoint&); // Do not use
	keypoint& operator=(const keypoint&); // Do not use
};

#endif
