//-*-c++-*-
#ifndef INCLUDED_XWalkMC_h_
#define INCLUDED_XWalkMC_h_

#include "Motion/XWalkParameters.h"
#include "Motion/MotionCommand.h"
#include "Motion/MotionManager.h"
#include "Motion/KinematicJoint.h"
#include "Motion/IKSolver.h"
#include "Shared/fmatSpatial.h"
#include "Shared/Config.h"
#include "Shared/get_time.h"
#include "IPC/DriverMessaging.h"
#include <set>

//! Extreme walk engine handles legged locomotion on hexapods and beyond
class XWalkMC : public MotionCommand, public XWalkParameters {
public:
	//! constructor
  XWalkMC();
	
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

	//! Specify the desired body velocity in x and y (millimeters per second) and angular velocity (radians per second), and amount of time (seconds) before stopping
	virtual void setTargetVelocity(float xvel, float yvel, float avel, float time);
	
	//! Specify the desired body displacement in x and y (millimeters) and angle (radians), optionally specify time (seconds)
	/*! Corresponding velocity will be limited to max velocity, so setting time=0 implies max speed */
	virtual void setTargetDisplacement(float xdisp, float ydisp, float adisp, float time=0);

	virtual void setTargetDisplacement(float xdisp, float ydisp, float adisp, float xvel, float yvel, float avel) {}

	virtual int updateOutputs();

  void setDirty() { dirty = true; }

	virtual int isDirty() {
		for(unsigned int i=0; i<NumLegs; ++i)
			if(legStates[i].support<1)
				return dirty=true;
		return dirty = ( (targetVel.norm()>=EPSILON || std::abs(targetAngVel)>=EPSILON || !active.empty()) );
	}

	virtual int isAlive() { return true; }

	virtual void start();
	
	virtual void stop();

	//! Posts a LocomotionEvent and sets velocities to zero. Also forces an output frame setting wheel velocities to zero; needed because if we remove a motion command there may be nothing left to zero the velocities.
	virtual void zeroVelocities();

	
#if 0
	//! Adjusts body height to lock knees and minimize joint torque to hold position, doesn't move feet
	/*! Implies a call to setTargetVelocity(0,0,0); later calls to set a non-zero velocity will cancel the leg lock. */
	void lockLegsInPlace();
#endif
	
protected:
	//! recomputes each leg's LegState::neutralPos based on current values in #p
	void updateNeutralPos(unsigned int curTime);
	// updates period based on stride length parameter, returns true if no significant motion is currently requested
	bool resetPeriod(float time, float speed);
	//! recomputes LegState::downPos and LegState::liftPos based on specified state variables, such that the direction of stride rotates about the current position
	void resetLegState(unsigned int leg, float phase, const fmat::Column<3>& curPos, bool inAir, float speed);
	
	void updateOutputsInitial(unsigned int curTime, const fmat::Column<3>& ground, const fmat::Column<3>& gravity, IKSolver::Point tgts[]);
	void updateOutputsWalking(float dt, const fmat::Column<3>& ground, const fmat::Column<3>& gravity, float speed, IKSolver::Point tgts[]);
	void sendLoadPredictions(IKSolver::Point tgts[]);
	
	//! updates #globPhase based on specified stride time (relative to #startTime)
	void computePhase(float time);
	//! computes current leg state based on #globPhase
	void computeLegPhase(unsigned int leg, bool& inAir, float& phase);
	//! computes the leg position (in xy plane only) based on leg state
	void computeCurrentPosition(unsigned int leg, bool inAir, float speed, float phase, fmat::Column<3>& tgt);
	//! computes body position based on settings in #p, which should then be added to leg positions before projecting onto ground plane
	void computeCurrentBodyOffset(float* legPhases, float speed, float& offx, float& offy, float& offa);
	//! solves inverse kinematics and send affected output values to motion manager
	void solveIK(unsigned int leg, const IKSolver::Point& tgt);
	
	// Given the mass, its position x and y, and a list of contact points, stores results into @a pressures
	/*! @param mass being supported in kilograms
	 *  @param massx center of mass along x axis, in millimeters
	 *  @param massy center of mass along y axis, in millimeters
	 *  @param contacts position of each support contact, in millimeters
	 *  @param[out] pressures will be resized and overwritten with pressure at each corresponding point in @a contacts, in newtons
	 *  
	 *  Note the z position is irrelevant, all points should be projected to the same plane, normal to gravity vector.<br>
	 *  Thanks Jacqueline K. Libby for helping figure out the free-body diagram/computation upon which this function is based. */
	void computePressure(float mass, float massx, float massy, const std::vector<fmat::Column<2> >& contacts, std::vector<float>& pressures);
	
	//! recursively processes contents of @a src, creating a ParameterTransition for each entry to smoothly update corresponding entries in @a dst
	void spiderSettings(plist::DictionaryBase& src, plist::DictionaryBase& dst);
	//! recursively processes contents of @a src, creating a ParameterTransition for each entry to smoothly update corresponding entries in @a dst
	void spiderSettings(plist::ArrayBase& src, plist::ArrayBase& dst);
	
	//! 
	class ParameterTransition : plist::PrimitiveListener {
	public:
		//! constructor, specify source to be monitored, destination to be kept in sync, a collection to be updated when transitions are in progress, and desired duration of transitions
		ParameterTransition(const plist::PrimitiveBase& srcVal, plist::PrimitiveBase& dstVal, std::set<ParameterTransition*>& activeTrans, unsigned int dur) : src(srcVal), dst(dstVal), active(activeTrans), curd(0), cura(0),
		startTime(), duration(dur), lastUpdate(), decelerate(false)
		{
			dst=src;
			src.addPrimitiveListener(this);
		}
		//! destructor
		~ParameterTransition() { src.removePrimitiveListener(this); }
		virtual void plistValueChanged(const plist::PrimitiveBase& /*pl*/);
		bool update(unsigned int curTime);
		
		const plist::PrimitiveBase& src; //!< the parameter which is being monitored for changes (a member of the superclass of XWalkMC)
		plist::PrimitiveBase& dst; //!< the parameter which is updated with smoothly transitioning values based on #src
		std::set<ParameterTransition*>& active; //!< collection which ParameterTransition will insert itself when src has been changed and transition is in progress
		float curd; //!< current velocity of parameter (d for derivative)
		float cura; //!< current acceleration of parameter
		unsigned int startTime; //!< time at which src was changed
		unsigned int duration; //!< how much time to use in transitioning (#cura is then based on difference between #src and #dst, and this)
		unsigned int lastUpdate; //!< time of last call to update() or reset()
		bool decelerate; //!< set to true if in the deceleration phase of transitioning
	};
	
	
	bool dirty; //!< true if update required
	fmat::Column<2> targetVel; //!< the requested xy velocity of the body (ignoring parameterized body motion, like sway or surge), millimeters per second
	float targetAngVel; //!< the requested angular velocity of the body, radians per second
	fmat::Column<2> targetDisp; //!< the requesed xy displacement of the body, in millimeters
	float targetAngDisp; //!< the requested angular displacement of the body in radians
	bool velocityMode; //!< True if we just want to maintain a velocity; false if we're trying to achieve a displacement
	unsigned int displacementTime; //!< time in msecs since setTargetDisplacement called
	bool plantingLegs; //!< True if we've finished a displacement and are waiting for all legs to come to ground before posting a status event
	bool initialPlacement; //!< set to true when legs are in an unknown configuration (e.g. following start())
	XWalkParameters p; //!< current parameter values, subject to smoothed transition from those as members of the superclass (see ParameterTransition)
	std::set<ParameterTransition*> transitions; //!< full collection of parameter listeners
	std::set<ParameterTransition*> active; //!< collection of those listeners which are actively transitioning their corresponding parameters in #p
	float startTime; //!< must be float (not unsigned int) because value can go negative: phase offset of "walk time" from system get_time() (milliseconds)
	float period; //!< the time between leg lifts (milliseconds), calculated value to yield target speed (#targetVel)
	float globPhase; //!< the current phase within the walk, in turn controls phase of individual legs via updateLegPhase()
	fmat::Column<2> rotationCenter; //!< if the target angular velocity (#targetAngVel) is producing a reasonable curvature vs. targetVel, this is the point about which the body is arcing
	DriverMessaging::FixedPoints contactMsg; //!< list of current contact points, for better Mirage simulation
	
	//! contains cached information about each leg's stride
	struct LegState {
		//! constructor
		LegState() : inAir(false), onGround(false), initialHeight(), support(1), reorder(-1U), neutralPos(), liftPos(), downPos() {}
		
		//! True if the leg is currently in the return flight phase, false if its xy motion is linked to the ground.
		/*! Note that if this is false, the leg may be in the process of raising or lowering to the ground, but not actually providing support */
		bool inAir;
		//! True if the leg is currently providing support on the ground, false if in the air (including raise or lower)
		bool onGround;
		float initialHeight; //!< during initial placement, stores the height of the leg when start() was called
		float support; //!< at the beginning and end of support phase, leg is lifted... this indicates how much lift has occurred/remains (range 0: in air, to 1: on ground)
		unsigned int reorder; //!< if a valid leg index (i.e. not -1U), indicates the index of the leg from which the phase offset parameter (XWalkParameters::LegParameters::flightPhase) should be used
		fmat::Column<2> neutralPos; //!< indicates the mid-stride foot position
		fmat::Column<2> liftPos; //!< indicates the position the leg was lifted from during flight, or the last target point during support
		fmat::Column<2> downPos; //!< indicates the position the leg will be placed following flight, or would have been placed to maintain continuity if target velocities change (see resetLegState())
	};
	LegState legStates[NumLegs]; //!< storage of cached stride information for each leg
	
	static KinematicJoint* kine;
	static KinematicJoint* childMap[NumReferenceFrames];
};

/*! @file
 * @brief Defines XWalkMC, which DESCRIPTION
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
