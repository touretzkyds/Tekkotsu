#ifndef _CAMERADATA_H_
#define _CAMERADATA_H_

#include "Shared/RobotInfo.h"
#include "Shared/plist.h"
#include "Shared/Config.h"

#include <iostream>
using namespace std;

struct CameraData : public plist::Dictionary {
	//! The camera alignment homography matrix
	plist::ArrayOf<plist::ArrayOf<plist::Primitive<float> > > homography;
	
	CameraData() : plist::Dictionary(false), homography(3, plist::ArrayOf<plist::Primitive<float> >(3, 0)) {
		addEntry("homography", homography, "Fill this with the approopriate homography matrix for camera alignment. You should probably\n"
				 " use the calibrator unless you really know what you're doing");
		
		for(unsigned i = 0; i < 3; i++)
			for(unsigned j = 0; j < 3; j++)
				homography[i][j] = 0;
		homography [0][0] = homography[1][1] = homography[2][2] = 1;
	}
	
	void loadCameraData() {
		std::string path = config->getFileSystemRoot() + "/config/"+CameraName+".xml";
		
		int ret = plist::Dictionary::loadFile(path.c_str());
		if(ret == 0) {
			cout<<"--- Unable to load homography for \""<<path<<"\""<<endl;
			return;
		}
		for(unsigned i = 0; i < 3; i++)
			for(unsigned j = 0; j < 3; j++)
				CameraHomography(i,j) = homography[i][j];
	}
	
	void saveCameraData() {
		for(unsigned i = 0; i < 3; i++)
			for(unsigned j = 0; j < 3; j++)
				homography[i][j] = CameraHomography(i,j);
		
		std::string path = config->getFileSystemRoot() + "/config/"+CameraName+".xml";
		int ret = plist::Dictionary::saveFile(path.c_str());
		if(ret == 0)
			cout<<"--- Unable to save homography in \""<<path<<"\""<<endl;
	}
};

extern CameraData* cameraData;

#endif
