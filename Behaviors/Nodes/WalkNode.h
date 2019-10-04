//-*-c++-*-
#ifndef INCLUDED_WalkNode_h_
#define INCLUDED_WalkNode_h_

#include <iostream>

#include "Events/LocomotionEvent.h"
#include "Motion/WalkMC.h"
#include "Behaviors/Nodes/MCNode.h"
#include "Shared/attributes.h"

extern const char defWalkNodeName[];
extern const char defWalkNodeDesc[];

//! A request to walk at a specified velocity or for a specific distance, to be used in a SignalTrans
/*! Recommended usage is via the velocity() and displacement() generators */
class WalkRequest {
public:
	//! Constructor
	WalkRequest() : x(0), y(0), a(0), time(-1), velocityMode(true), storedVelocities(false), filename() {}
	
	//! Constructor
	WalkRequest(const std::string& paramFile) : x(0), y(0), a(0), time(-1), velocityMode(true), storedVelocities(false), filename(paramFile) {}
	
	//! Constructor: velocities in mm/sec and radians/sec
	WalkRequest(float xvel, float  yvel, float avel, const std::string& paramFile="") :
	x(xvel), y(yvel), a(avel), time(-1), velocityMode(true), storedVelocities(true), filename(paramFile) {}
	
	//! Constructor
	/*! @param x_ if @a isVelocity, the speed (mm/s) to move forward, else the distance (mm)
	 *  @param y_ if @a isVelocity, the speed (mm/s) to move to the left, else the distance (mm)
	 *  @param a_ if @a isVelocity, the speed (radians/s) to rotate to the left, else the distance (radians)
	 *  @param time_ the duration of the walk (seconds), if isVelocity is false the velocity will still be subject to the walk's calculated limits, so just pass 0 for max speed
	 *  @param isVelocity controls interpretation of @a x, @a y, and @a a
	 *  @param paramFile loads a parameter file to control walk gait */
	WalkRequest(float x_, float y_, float a_, float time_, bool isVelocity, const std::string& paramFile="") :
	x(x_), y(y_), a(a_), time(time_), velocityMode(isVelocity), storedVelocities(true), filename(paramFile) {}
	
	//! Generator to call the constructor with appropriate arguments, a little more concise/readable than using the constructor directly
	/*! @param x the speed (mm/s) to move forward
	 *  @param y the speed (mm/s) to move to the left
	 *  @param a the speed (radians/s) to rotate to the left
	 *  @param time the duration of the walk (seconds)
	 *  @param paramFile loads a parameter file to control walk gait */
	static WalkRequest velocity(float x, float y, float a, float time=-1, const std::string& paramFile="") { return WalkRequest(x, y, a, time, true, paramFile); }
	
	//! Generator to call the constructor with appropriate arguments, a little more concise/readable than using the constructor directly
	/*! @param x the distance (mm) to move forward
	 *  @param y the distance (mm) to move to the left
	 *  @param a the distance (radians) to rotate to the left
	 *  @param time the duration of the walk (seconds), the velocity will still be subject to the walk's calculated limits, so just pass 0 for max speed
	 *  @param paramFile loads a parameter file to control walk gait */
	static WalkRequest displacement(float x, float y, float a, float time=0, const std::string& paramFile="") { return WalkRequest(x, y, a, time, false, paramFile); }
	
	float x, y, a, time;
	bool velocityMode,  storedVelocities;
	std::string filename;
	
public:
	bool operator==(const WalkRequest& other) const;
};

std::ostream& operator<<(std::ostream &os, const WalkRequest &req);

//! A StateNode for walking using WalkMC
/*! The WalkNode can handle either velocities or displacements.
 *
 *  To specify a velocity with no end, use the 3-tuple constructor:
 *  @code
 *  WalkNode(vx,vy,va)
 *  @endcode
 *
 *  To specify a velocity for a specific amount of time, which will then trigger a completion event:
 *  @code
 *  WalkNode(vx,vy,va, time, WalkNode::VEL) 
 *  @endcode
 *  (You could also use the "endless" velocity constructor with a TimeOutTrans)
 *
 *  To have WalkNode interpret the (x,y,a) as a distance to travel instead of velocity, pass false at the end instead:
 *  @code
 *  WalkNode(dx,dy,da, time, WalkNode::DISP)
 *  @endcode
 *  The time will be subject to speed limits of the underlying WalkMC, so it may take more time to complete.
 *  In particular, this means you can simply pass time=0 for a displacement and this will be extended to the maximum speed:
 *  @code
 *  WalkNode(dx,dy,da, 0, WalkNode::DISP); // zero time implies max speed for displacements
 *  @endcode
 */
class WalkNode : public MCNode<WalkMC,defWalkNodeName,defWalkNodeDesc> {
public:
	enum Mode_t {
		DISP=0, //!< use with constructors to indicate displacement mode
		VEL=1 //!< use with constructors to indicate velocity mode
	};
	
	//! constructor
	WalkNode() : MCNode<WalkMC,defWalkNodeName,defWalkNodeDesc>(), req() {}
	
	//! constructor
	WalkNode(const std::string& name, const std::string filename="") :
	MCNode<WalkMC,defWalkNodeName,defWalkNodeDesc>(name), req(filename) {}
	
	//!constructor: velocities in mm/sec and radians/sec
	WalkNode(float xvel, float yvel, float avel, const std::string filename="") : 
	MCNode<WalkMC,defWalkNodeName,defWalkNodeDesc>(), req(xvel,yvel,avel,filename) {}
	
	//!constructor: velocities in mm/sec and radians/sec
	WalkNode(const std::string& name, float xvel, float yvel, float avel, const std::string filename="") :
	MCNode<WalkMC,defWalkNodeName,defWalkNodeDesc>(name), req(xvel,yvel,avel,filename) {}
	
	//!constructor: displacements in mm and radians, time parameter in seconds but will be extended to fit within max speed
	/*! @param x if @a velocityMode is VEL, the speed (mm/s) to move forward, else the distance (mm)
	 *  @param y if @a velocityMode is VEL, the speed (mm/s) to move to the left, else the distance (mm)
	 *  @param a if @a velocityMode is VEL, the speed (radians/s) to rotate to the left, else the distance (radians)
	 *  @param time the duration of the walk (seconds), if isVelocity is false the velocity will still be subject to the walk's calculated limits, so just pass 0 for max speed
	 *  @param velocityMode controls interpretation of @a x, @a y, and @a a
	 *  @param filename loads a parameter file to control walk gait */
	WalkNode(float x, float y, float a, float time, Mode_t velocityMode, const std::string filename="") : 
	MCNode<WalkMC,defWalkNodeName,defWalkNodeDesc>(), req(x,y,a,time,velocityMode,filename) {}
	
	//!constructor: pass displacement as velocity or distance and time, isVelocity controls interpretation of x, y, and a
	/*! @param name an instance name for the node (mostly useful for debugging)
	 *  @param x if @a velocityMode is VEL, the speed (mm/s) to move forward, else the distance (mm)
	 *  @param y if @a velocityMode is VEL, the speed (mm/s) to move to the left, else the distance (mm)
	 *  @param a if @a velocityMode is VEL, the speed (radians/s) to rotate to the left, else the distance (radians)
	 *  @param time the duration of the walk (seconds), if velocityMode is DISP the velocity will still be subject to the walk's calculated limits, so just pass 0 for max speed
	 *  @param velocityMode controls interpretation of @a x, @a y, and @a a
	 *  @param filename loads a parameter file to control walk gait */
	WalkNode(const std::string& name, float x, float y, float a, float time, Mode_t velocityMode, const std::string filename="") :
	MCNode<WalkMC,defWalkNodeName,defWalkNodeDesc>(name), req(x,y,a,time,velocityMode,filename) {}
	
	//! sets the velocity of the walk
	/*! @param xdist x displacement (mm, positive is forward)
	 *  @param ydist y displacement (mm, positive is left)
	 *  @param adist angular displacement (rad, positive is counter-clockwise)
	 *  @param time how many seconds to take to achieve displacement (limited to walk's max speed, so time=0 implies max speed) */
	void setDisplacement(float xdist, float ydist, float adist, float time=0);
	
	//! sets the velocity of the walk
	/*! @param xvel x velocity (mm/s, positive is forward)
	 *  @param yvel y velocity (mm/s, positive is left)
	 *  @param avel angular velocity (rad/s, positive is counter-clockwise) */
	void setVelocity(float xvel, float yvel, float avel);
	
	//! sets the velocity of the walk
	/*! @param xvel x velocity (mm/s, positive is forward)
	 *  @param yvel y velocity (mm/s, positive is left)
	 *  @param avel angular velocity (rad/s, positive is counter-clockwise)
	 *  @param time is seconds until stopping and posting completion */
	void setVelocity(float xvel, float yvel, float avel, float time);
	
	virtual void stop();
	
protected:
	virtual void preStart();
	virtual void postStart();
	
	WalkRequest req; //!< stores settings from constructor or a signal transition
	
};

#endif
