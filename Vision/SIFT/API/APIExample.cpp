#include <iostream>
#include <string>
#include <vector>

#include "SiftTekkotsu.h"

using namespace std;

void convertPGMImgToImageBuffer(PGMImg& pgmImg, ImageBuffer& buffer);

int main(int argc, char** argv){
	
	SiftTekkotsu mySiftTekkotsu;
	
	/// Set parameters
	mySiftTekkotsu.setParameter("probOfMatch", 0.9);
	
	
	///Training phase
	
	// Creating an ImageBuffer from an actual PGM image for use
	ImageBuffer buffer;
	PGMImg pgmImg;
	pgmImg.fromFile("../images/duck/duck01.pgm");
	convertPGMImgToImageBuffer(pgmImg, buffer);
	
	// Create database of duck images
	int duckID = mySiftTekkotsu.train_addNewObject(buffer);
	mySiftTekkotsu.setObjectName(duckID, "Duckie-duck");
	cout << "Duck21: " << mySiftTekkotsu.train_addToObject(duckID, "../images/duck/duck21.pgm") << endl << endl << endl << endl;
	cout << "Duck08: " << mySiftTekkotsu.train_addNewModel(duckID, "../images/duck/duck08.pgm") << endl << endl << endl << endl;
	
	// Remember to free ImageBuffer memory after use
	free(buffer.byteArray);
	
	// Create database of doubleshot can images
	int doubleshotID = mySiftTekkotsu.train_addNewObject("../images/doubleshot/doubleshot01.pgm");
	mySiftTekkotsu.setObjectName(doubleshotID, "Coffee can");
	cout << "Doubleshot21: " << mySiftTekkotsu.train_addToObject(doubleshotID, "../images/doubleshot/doubleshot21.pgm") << endl << endl << endl << endl;
	cout << "Doubleshot05: " << mySiftTekkotsu.train_addToObject(doubleshotID, "../images/doubleshot/doubleshot05.pgm") << endl << endl << endl << endl;
	
	mySiftTekkotsu.saveToFile("duckAndDoubleshot.kb", false);
	mySiftTekkotsu.loadFile("duckAndDoubleshot.kb");
	
	/// Matching tests
	vector<SiftMatch*> matchesFound;
	
	// Look for duck objects only
	cout << "Matching ../images/duckanddoubleshot.pgm against duck:\n";
	mySiftTekkotsu.findObjectInImage(duckID, "../images/duckanddoubleshot.pgm", matchesFound);
	for (unsigned int i = 0; i < matchesFound.size(); i++){
		cout << "Match # " << i << ": " << mySiftTekkotsu.getObjectName(matchesFound[i]->objectID) << endl;
		matchesFound[i]->print("\t");
		delete matchesFound[i];
	}
	matchesFound.clear();
	cout << endl << endl << endl << endl;
	
	// Look for doubleshot can objects only
	cout << "Matching ../images/duckanddoubleshot.pgm against doubleshot:\n";
	mySiftTekkotsu.findObjectInImage(doubleshotID, "../images/duckanddoubleshot.pgm", matchesFound);
	for (unsigned int i = 0; i < matchesFound.size(); i++){
		cout << "Match # " << i << ": " << mySiftTekkotsu.getObjectName(matchesFound[i]->objectID) << endl;
		matchesFound[i]->print("\t");
		delete matchesFound[i];
	}
	matchesFound.clear();
	cout << endl << endl << endl << endl;
	
	// Look for all known objects of any kind
	cout << "Matching ../images/duckanddoubleshot.pgm against all objects:\n";
	mySiftTekkotsu.findAllObjectsInImage("../images/duckanddoubleshot.pgm", matchesFound);
	for (unsigned int i = 0; i < matchesFound.size(); i++){
		cout << "Match # " << i << ": " << mySiftTekkotsu.getObjectName(matchesFound[i]->objectID) << endl;
		matchesFound[i]->print("\t");
		delete matchesFound[i];
	}
	matchesFound.clear();
	cout << endl << endl << endl << endl;
	
}

