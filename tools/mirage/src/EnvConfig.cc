#include "EnvConfig.h"

using namespace std; 

INSTANTIATE_NAMEDENUMERATION_STATICS(EnvConfig::Background::model_t);
INSTANTIATE_NAMEDENUMERATION_STATICS(EnvConfig::Environment::MoCapLevel);

namespace EnvConfig {
	const char* const Background::modelNames[5] = { "", "Plane", "Box", "Dome", NULL };
	const char* const Environment::mocapLevelNames[5] = { "None", "Root", "Leaves", "All", NULL };
	
	const std::string& getCurrentLoadingFile() {
		return singleton().getCurrentLoadingFile();
	}
	
	Environment& singleton() {
		static Environment env;
		return env;
	}
	
	void Environment::resetDefaults() {
		Light * l = new Light;
		l->location[0] = 3000;
		l->location[1] = -5000;
		l->location[2] = 8000;
		lights.addEntry("light1",l);
		
		camera.location[0] = -600;
		camera.location[1] = -800;
		camera.location[2] = 1200;
		// camera is initially pointing straight down, this makes it look a little above (0,0,0)
		// could use "point at" settings, but don't want to require users to override that
		float zrot = std::atan2(camera.location[0],-camera.location[1]);
		float xrot = std::atan2(-camera.location[1],camera.location[2]) + (float)M_PI/12;
		fmat::Quaternion qc = fmat::Quaternion::aboutZ(zrot) * fmat::Quaternion::aboutX(xrot);
		qc.exportTo(camera.orientation[0],camera.orientation[1],camera.orientation[2]);
		
		resetScenery();
	}
		
	void Environment::resetScenery() {
		// ground
		PhysicalObject * g = new PhysicalObject;
		g->model = "Disc";
		g->modelScale[0]=g->modelScale[1]=40000;
		g->material = "Grid";
		g->collisionModelScale[0]=g->collisionModelScale[1]=40000;
		g->collisionModel="Plane";
		/*
		// Sometimes using plane causes trouble, can use box for ground instead:
		g->collisionModelOffset[2]=-0.5;
		g->collisionModel="Cube";
		 */
		objects.addEntry("ground",g);
		
		// put a reference frame at the world origin
		// (not using component of ground because we want this marker constant even if someone does a ModelOffset on ground)
		PhysicalObject * rf = new PhysicalObject;
		rf->visible=false;
		rf->model = "ReferenceFrame";
		rf->modelScale[0]=rf->modelScale[1]=rf->modelScale[2]=3;
		objects.addEntry("WorldOrigin",rf);
		
	}
	
	unsigned int Environment::loadFile(const char* filename) {
		curLoadingFile=filename;
		try {
			unsigned int ans = plist::Dictionary::loadFile(filename);
			curLoadingFile.clear();
			return ans;
		} catch(...) {
			curLoadingFile.clear();
			throw;
		}
	}

}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
