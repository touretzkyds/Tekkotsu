//-*-c++-*-
#ifndef INCLUDED_Physics_h_
#define INCLUDED_Physics_h_

#include "Planners/Dynamics/PhysicsWorld.h"
#include "Shared/plist.h"

//! description of Physics
class Physics : virtual public plist::Dictionary, public PhysicsWorld {
public:
	static Physics& singleton() { return (inst!=NULL) ? *inst : *(inst = new Physics); }
	static void close() { delete inst; inst = NULL; }
	
	static size_t update(); //!< triggers motion controls, physics simulation, and graphics sync, returns number of phyics sub-steps taken
	
	static float getSpaceScale() { return singleton().spaceScale; }
	static float getMassScale() { return singleton().massScale; }
	
	plist::Primitive<unsigned int> stepsPerFrame;
	plist::Primitive<float> physicsFPS;
protected:
	Physics() : plist::Dictionary(), stepsPerFrame(10), physicsFPS(100) {
		addEntry("StepsPerFrame",stepsPerFrame,"Maximum physics updates per graphics frame. (default 10)");
		addEntry("PhysicsFPS",physicsFPS,"Target physics update rate, recommended to keep above 60. (default 100)");
		setLoadSavePolicy(FIXED,SYNC);
	}
	
	static Physics * inst;
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
