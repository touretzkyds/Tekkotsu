//-*-c++-*-
#ifndef INCLUDED_WheeledWalkMC_h_
#define INCLUDED_WheeledWalkMC_h_

#include "Motion/MotionCommand.h"
#include "Motion/MotionManager.h"
#include "Shared/get_time.h"
#include "Shared/Measures.h"
#include "Shared/RobotInfo.h"
#include "Shared/plist.h"

//! Provides a 'WalkMC' implementation for wheeled robots (diff-drive or theoretically holonomic configurations as well)
/*! Uses kinematic description to determine location and orientation of wheels
 *  and computes appropriate wheel velocities to produce a target motion.
 *  Can handle skid-steer (diff drive) type motion, including holonomic wheels,
 *  but does not handle ackerman style (steered) configurations.
 *  
 *  Also does not handle 'slanted' wheels optimally: Assumes wheels are perpendicular
 *  to the ground, that the base frame Z axis is perpendicular to the ground frame.
 *  This could be added with a bit more math when computing wheel positions.
 *
 *  This motion command also assumes that wheels are commanded by mm/sec ground
 *  speed, not rad/sec rotational speed.
 *
 *  For simulation in Mirage, make sure all wheels are marked by adding a Velocity=true
 *  entry in the ControllerInfo of the corresponding kinematics configuration, and that
 *  the x and y dimensions of the collision model are set for the wheel radius.
 *
 *  TODO: Test on a holonomic platform, should handle it, but originally only
 *  tested on diff-drive like Create.  Also, perhaps add config settings to override #rotationCenter and
 *  a flag to have updateWheelConfig called before each updateOutputs. */
class WheeledWalkMC : public MotionCommand, public virtual plist::Dictionary {
public:
	//! constructor
	WheeledWalkMC() :
		plist::Dictionary(), MotionCommand(),
#if defined(TGT_IS_KOBUKI) || defined(TGT_IS_CREATE2)
		lastTickLeft(state->sensors[LeftEncoderOffset]),
		lastTickRight(state->sensors[RightEncoderOffset]),
		lastDistTraveled(0), lastAngTraveled(0),
#endif
		preferredXVel(), preferredYVel(), preferredAngVel(), 
    targetVel(), targetAngVel(0), targetDur(0), targetDist(0), targetAngDist(0),
		maxVel(), maxAngVel(0), rotationCenter(), displacementMode(false),
		travelStartTime(0), travelStartDist(0), travelStartAngle(0),
		dirty(false)
	{
		addEntry("PreferredXVel",preferredXVel,"optimal X velocity for unspecified displacements (mm/s)");
		addEntry("PreferredYVel",preferredYVel,"optimal Y velocity for unspecified displacements (mm/s)");
		addEntry("PreferredAngVel",preferredAngVel,"optimal angular velocity for unspecified displacements (rad/s)");
		setLoadSavePolicy(FIXED,SYNC);
		resetConfig();
	}
	
	void resetConfig(); //!< reset and reload configuration settings, implies call to updateWheelConfig()

	void setDirty() { dirty = true; }

	virtual int updateOutputs();
	virtual int isDirty() { return dirty; }
	virtual int isAlive();
	virtual void start();
	virtual void stop();
	
	//!  Posts a LocomotionEvent and sets velocities to zero. Also forces an output frame setting wheel velocities to zero; needed because if we remove a motion command there may be nothing left to zero the velocities.
	virtual void zeroVelocities();
	
	float getMaxXVel() const { return maxVel[0]; }
	float getMaxYVel() const { return maxVel[1]; }
	float getMaxAVel() const { return maxAngVel; }
	
	unsigned int getTravelTime() { return get_time()-travelStartTime; } //!< the amount of time (ms) we have been travelling the current vector
	
	//! Returns the current x and y velocities in mm/sec
	const fmat::Column<2>& getTargetVelocity() const { return targetVel; };
	
	//! Returns the current angular velocity in radians/sec
	float getTargetAngVelocity() const { return targetAngVel; }
	
	//! Specify the desired body velocity in x and y (millimeters per second) and angular velocity (radians per second)
	virtual void getTargetVelocity(float &xvel, float &yvel, float &avel) {
		xvel = targetVel[0];
		yvel = targetVel[1];
		avel = targetAngVel;
	}
	
	//! Specify the desired body velocity in x and y (millimeters per second) and angular velocity (radians per second); does not stop automatically
	virtual void setTargetVelocity(float xvel, float yvel, float avel);
	
	//! Specify the desired body velocity in x and y (millimeters per second) and angular velocity (radians per second), and amount of time before stopping
	virtual void setTargetVelocity(float xvel, float yvel, float avel, float time);
	
	//! Specify the desired body displacement in x and y (millimeters) and a (radians)
	/*! Corresponding velocity will be limited to max velocity, so setting time=0 implies max speed */
	virtual void setTargetDisplacement(float xdisp, float ydisp, float adisp, float time=0);
	
	//! Specify body displacement and speed
	void setTargetDisplacement(float xdisp, float ydisp, float adisp, float xvel, float yvel, float avel);

	//! Recomputes wheel positions and orientations.  Automatically called by constructor, but may need to recall if wheel positions are actuated.
	/*! Includes a call to updateWheelVels() in case wheel positions change. */
	virtual void updateWheelConfig();
#if defined(TGT_IS_KOBUKI) || defined(TGT_IS_CREATE2)
	unsigned short lastTickLeft;
	unsigned short lastTickRight;
	float lastDistTraveled;
	float lastAngTraveled;
#endif
	plist::Primitive<float> preferredXVel; //!< optimal X velocity for unspecified displacements
	plist::Primitive<float> preferredYVel; //!< optimal Y velocity for unspecified displacements
	plist::Primitive<float> preferredAngVel; //!< optimal angular velocity for unspecified displacements
	
protected:
	void updateWheelVels(); //!< updates WheelInfo::targetVel values based on #targetVel and #targetAngVel
	
	fmat::Column<2> targetVel; //!< the requested xy velocity of the body (ignoring parameterized body motion, like sway or surge), millimeters per second
	float targetAngVel; //!< the requested angular velocity of the body, radians per second
	unsigned int targetDur; //!< duration in msecs for the current displacement
	float targetDist; //!< forward distance we want to travel
	float targetAngDist; //!< angular distance we want to travel
	fmat::Column<2> maxVel; //!< maximum velocity in x,y
	float maxAngVel; //!< maximum angular velocity
	
	fmat::Column<2> rotationCenter; //!< point to use as center of rotations, defaults to center of wheels
	bool displacementMode; //!< If true, velocity will be set to 0 when desired displacement is achieved
	unsigned int travelStartTime; //!< the time of the last call to setTargetVelocity - handy to check the time we've been traveling current vector
	float travelStartDist; //!< The forward odometry reading in millimeters at the time of the last call to setTargetVelocity
	float travelStartAngle; //!< The angular odometry reading in radians at the time of the last call to setTargetVelocity
	bool dirty; //!< set to true by updateWheelVels(), cleared up updateOutputs()
	
	static constexpr float EPSILON = 1e-4f;

	//! stores information about the configuration and current target state of each wheel
	struct WheelInfo {
		WheelInfo() : valid(false), position(), direction(), targetVel(0) {}
		bool valid; //!< set to false if the wheel axle is vertical
		fmat::Column<2> position; //!< the center of the wheel's contact
		fmat::Column<2> direction; //!< the direction of forward motion produced by this wheel
		float targetVel; //!< the current wheel velocity to supply to motman via updateOutputs, is calculated by setTargetVelocity()
	};
	WheelInfo wheels[NumWheels];
};

/*! @file
 * @brief Defines WheeledWalkMC, which provides a 'WalkMC' implementation for wheeled robots (diff-drive or theoretically holonomic configurations as well)
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
