#include "keypoint.h"
#include "keygroup.h"
#include "model.h"
#include "object.h"
#include <iostream>

using namespace std;

int keypoint::keypointID = 0;

keypoint::keypoint(int ID)
: id(ID), G(NULL), modelX(-1), modelY(-1), modelScale(-1), modelOrientation(-1),
imageX(-1), imageY(-1), imageScale(-1), imageOrientation(-1),
imageIntScale(1), imageIntOctave(1), desc(), generation(-1),
bestDist(), bestMatch(), finalMatch(NULL), matches(), modelMatches(), modelMatchErrors(),
isInlier(false) {}

double keypoint::sqDist(keypoint& key){
	double dist = 0.0;
	for (int i = 0; i < KEYPOINTDIM; i++){
		double diff = this->desc[i] - key.desc[i];
		dist += (diff * diff);
	}
	
	return dist;
}

void keypoint::print(){
	cout << "modelX:           " << modelX           << endl;
	cout << "modelY:           " << modelY           << endl;
	cout << "modelScale:       " << modelScale       << endl;
	cout << "modelOrientation: " << modelOrientation << endl;
	cout << "imageX:           " << imageX           << endl;
	cout << "imageY:           " << imageY           << endl;
	cout << "imageScale:       " << imageScale       << endl;
	cout << "imageOrientation: " << imageOrientation << endl;
	cout << "desc: (";
	int i;
	for (i = 0; i < KEYPOINTDIM - 1; i++){
		cout << desc[i] << ", ";
	}
	cout << desc[i] << ")\n";
}


void keypoint::writeToFile(std::ofstream& outfile, bool indicateDim){
	if (indicateDim) outfile << desc.size();
}

void keypoint::writeToFile(std::ofstream& outfile){
	outfile << id << endl;
	outfile << G->M->O->getID() << "\t" << G->M->getID() << "\t" << G->getID() << endl;
	// 	outfile << valid << endl;
	outfile << imageX << "\t" << imageY << "\t" << imageScale << "\t" << imageOrientation << "\t" << imageIntScale << "\t" << imageIntOctave << endl;
	outfile << modelX << "\t" << modelY << "\t" << modelScale << "\t" << modelOrientation << endl;
	outfile << generation << endl;
	if (finalMatch == NULL){
		outfile << 0 << endl;
	}else{
		outfile << 1 << endl;
		outfile << finalMatch->id << endl;
	}
	outfile << matches.size() << endl;
	for (unsigned int i = 0; i < matches.size(); i++){
		outfile << matches[i]->id << "\t";
	}
	outfile << endl;
	outfile << desc.size() << endl;
	for (unsigned int i = 0; i < desc.size(); i++){
		outfile << desc[i] << "\t";
	}
	outfile << endl;
	// 	outfile << bestDist.size() << endl;
	// 	for (unsigned int i = 0; i < bestDist.size(); i++){
	// 		outfile << bestDist[i] << "\t";
	// 	}
	// 	outfile << endl;
	// 	outfile << bestMatch.size() << endl;
	// 	for (unsigned int i = 0; i < bestMatch.size(); i++){
	// 		outfile << "\t" << bestMatch[i]->G->M->O->getID() << "\t" << bestMatch[i]->G->M->getID() << "\t" << bestMatch[i]->G->getID() << "\t" << bestMatch[i]->getID();
	// 	}
	// 	outfile << endl;
	// 	if (finalMatch == NULL){
	// 		outfile << 0 << endl;
	// 	}else{
	// 		outfile << 1 << endl;
	// 		outfile << finalMatch->G->M->O->getID() << "\t" << finalMatch->G->M->getID() << "\t" << finalMatch->G->getID() << "\t" << finalMatch->getID() << endl;
	// 	}
	// 	outfile << modelMatches.size() << endl;
	// 	for (unsigned int i = 0; i < modelMatches.size(); i++){
	// 		outfile << "\t" << modelMatches[i]->G->M->O->getID() << "\t" << modelMatches[i]->G->M->getID() << "\t" << modelMatches[i]->G->getID() << "\t" << modelMatches[i]->getID();
	// 	}
	// 	outfile << endl;
	// 	outfile << modelMatchErrors.size() << endl;
	// 	for (unsigned int i = 0; i < modelMatchErrors.size(); i++){
	// 		outfile << modelMatchErrors[i] << "\t";
	// 	}
	// 	outfile << endl;
	// 	outfile << isInlier << endl;
	
}
