//-*-c++-*-
#ifndef INCLUDED_WaypointWalk_h_
#define INCLUDED_WaypointWalk_h_

#include "IPC/ListMemBuf.h"
#include "WaypointEngine.h"
#include "WalkMC.h"

//! Combines a WaypointEngine with a WalkMC so you can walk between a set of waypoints
/*! Note the use of a template so we can have dedicate more or less
 *  space without modifying the class.
 *  
 *  But for everyday use, you can just use the ::WaypointWalkMC typedef
 *  which will default to a maximum of 100 waypoints */
class WaypointWalkMC : public WalkMC, public WaypointEngine {
public:
	
	//!constructor
	WaypointWalkMC()
		: WalkMC(), WaypointEngine()
	{}

	//!constructor
	WaypointWalkMC(char * f)
		: WalkMC(), WaypointEngine(f)
	{}
	
	void setDirty() { dirty = true; }

	//! so we can get our hooks in to modify the target velocity
	virtual int updateOutputs() {
		//std::cout << "WaypointWalkMC::updateOutputs()  " << WaypointEngine::curVel[0] << " " << WaypointEngine::curVel[1] << " " << WaypointEngine::curVel[2] << " isRunning=" << isRunning << " dirty=" << dirty << std::endl;
#ifdef TGT_HAS_WHEELEDWALK
		dirty = true;  // necessary so the null case with no waypoints still generates a LocomotionEvent
#endif
		if ( WaypointEngine::cycle() )
			WalkMC::setTargetVelocity(WaypointEngine::curVel[0],WaypointEngine::curVel[1],WaypointEngine::curVel[2]);
		//cout << get_time()-waypointTime << " Cur: ("<<curPos[0]<<','<<curPos[1]<<','<<curPos[2]<<")  Ideal: ("<<idealPos[0]<<','<<idealPos[1]<<','<<idealPos[2]<<','<<idealPos[3]<<")  Vel: ("<<curVel[0]<<','<<curVel[1]<<','<<curVel[2]<<")" << endl;
		return WalkMC::updateOutputs();
	}

	virtual int LoadWaypointFile(const char * f) { return WaypointEngine::loadFile(f); } //!< allows loading a waypoint file
	virtual int SaveWaypointFile(const char * f) const { return WaypointEngine::saveFile(f); } //!< allows saving a waypoint file
	virtual int LoadWalkMCFile(const char * f) { return WalkMC::loadFile(f); } //!< allows loading a WalkMC parameter file
	virtual int SaveWalkMCFile(const char * f) const { return WalkMC::saveFile(f); } //!< allows saving a WalkMC parameter file

};

/*! @file
 * @brief Defines WaypointWalk, which combines a WaypointEngine with a WalkMC so you can walk between a set of waypoints
 * @author ejt (Creator)
 */

#endif
