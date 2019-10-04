//-*-c++-*-
#ifndef INCLUDED_IKCalliope_h_
#define INCLUDED_IKCalliope_h_

#include "Motion/IKSolver.h"

#if defined(TGT_IS_CALLIOPE3) && defined(TGT_HAS_ARMS)

class IKCalliope : public IKSolver {
public:
	static const float EPSILON;
	//! constructor
	IKCalliope();

	using IKSolver::solve;
	using IKSolver::step;

	virtual bool solve(const Point& pEff, const Rotation& oriEff, KinematicJoint& j,
										 const Position& pTgt, float posPri, 
										 const Orientation& oriTgt, float oriPri) const;

	virtual StepResult_t step(const Point& pEff, const Rotation& oriEff, KinematicJoint& j,
														const Position& pTgt, float pDist, float posPri, 
														const Orientation& oriTgt, float oriDist, float oriPri) const;
	
	//! holds the class name, set via registration with the DeviceDriver registry
	static const std::string autoRegisterIKCalliope;

};

#elif defined(TGT_IS_CALLIOPE5) && defined(TGT_HAS_ARMS)

//! Kinematics solver for the 5-DOF Calliope Arm
class IKCalliope : public IKSolver {
public:
	static const float EPSILON;
	//! constructor
	IKCalliope();
	
	using IKSolver::solve;
	using IKSolver::step;
	
	//! Perform IK
	virtual bool solve(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, const Position& pTgt, float posPri, const Orientation& oriTgt, float oriPri) const;
	
	//! Carried over from other implementations
	virtual StepResult_t step(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, const Position& pTgt, float pDist, float posPri, const Orientation& oriTgt, float oriDist, float oriPri) const;
	
protected:
	//! Test if a side grasp is being requested
	static bool isSideGrasp(const Rotation &oriEff, const Orientation &oriTgt);

	//! Test if an overhead grasp is being requested
	static bool isOverheadGrasp(const Rotation &oriEff, const Orientation &oriTgt);

  //! Test if constraints are simple enough to solve analytically
	static bool analyticallySolvable(const Rotation &oriEff,
																	 const Position &pTgt, const Orientation &oriTgt);

	//! Provides fall back to the gradient solver if we can't solve analytically
	static bool gradientSolve(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, 
														const Position& pTgt, float posPri, const Orientation& oriTgt, float oriPri);

	//! Struct to enclose joint angles for 5-dof solution
	struct Solutions {
		Solutions() : angles(),noSols(),valid() {}
		fmat::Matrix<2,3> angles;
		int noSols;
		bool valid;
	};
	
	//! Attempt to find 3-link IK solution, for target point @a t, effector offset @a pEff, and preferred tool angle @a prefPhi
	/*! If a solution cannot be found, try ±1°, ±2°, ±3°, etc, until a valid solution is found.
	 *  If no valid solution can be found for any angle, just point the effector toward the target. */
	Solutions closestThreeLinkIK(const fmat::Column<2>& t, bool posPri, float prefPhi, const fmat::Column<2>& delta) const;
	
	//! Reaches the arm straight out in the direction of the target.
	/*! For use in case we cannot reach the target point with any tool angle phi */
	Solutions pointToward(const fmat::Column<2>& target, bool posPri, float prefPhi) const;
	
	//! Determines whether the angles that we've solved for are valid
	/*! Basically, we're checking to see if any angles in either solution
	 *  are outside of Calliope's joint limits. */
	Solutions validAngles(Solutions solutions) const;
	
	//! Performs 3-link IK.
	/*! Calls twoLinkIK, then populates solution struct with tool angle @a phi. */
	Solutions threeLinkIK(fmat::Column<2> t, float phi) const;
	
	//! Performs 2-link IK
	/*! Brings the base of the wrist to the target point @a t */

	fmat::Matrix<2,2> twoLinkIK(fmat::Column<2> t) const;
	
	//! Returns the index of the closest solution (0 or 1).
	/*! If there's only one solution, returns 0. */
	int closestSolution(KinematicJoint& j, Solutions s) const;
	
	//! Assures that the arm is in the given orientation
	/*! For use when we need to constrain orientation,
	 *  and aren't concerned with position as much. */
	void assureOrientation(Solutions& s, float phi) const;
	
	//! holds the class name, set via registration with the DeviceDriver registry
	static const std::string autoRegisterIKCalliope;
	
	//! Length of backarm and forearm
	float L1, L2;
	
	//! Limits of rotation of Calliope's Arm Joints
	float jointLimits[3][2];
	
	//! Extra angle between the base and the gripper to be taken into account
	float qGripper;
	
	//! Extra angle between the base and the wrist rotate to be taken into account
	float qWristRot;
	
	//! qOffset of the ArmBaseOffset
	float qOffset;
	
	//! Transforms from the BaseFrame to the ArmBase
	fmat::Transform baseToArm;
	
	//! Rotation from BaseFrame to the ArmBase
	fmat::Quaternion baseToArmRot;
	
	//! Distance from the arm base to the shoulder position.
	float shoulderOffset;
};

#elif defined(TGT_IS_CALLIOPE2) && defined(TGT_HAS_ARMS)

//! Kinematics solver for the 2-DOF Calliope Arm
class IKCalliope : public IKSolver {
public:
	static const float EPSILON;
	//! constructor
	IKCalliope();
	
	using IKSolver::solve;
	using IKSolver::step;
	
	virtual bool solve(const Point& pEff, const Rotation& oriEff, KinematicJoint& j,
										 const Position& pTgt, float posPri, 
										 const Orientation& oriTgt, float oriPri) const;

	virtual StepResult_t step(const Point& pEff, const Rotation& oriEff, KinematicJoint& j,
														const Position& pTgt, float pDist, float posPri, 
														const Orientation& oriTgt, float oriDist, float oriPri) const;
	
private:
	//! holds the class name, set via registration with the DeviceDriver registry
	static const std::string autoRegisterIKCalliope;
	
	//! cached transformation that converts points from the robot's base to the arm's base.
	fmat::Transform baseToArm;
	
	//! cached angle offset for arm base
	plist::Angle qOffset;
	
	//! cached offset for gripper
	plist::Angle gripperOffset;
	
	//! length of the forearm
	fmat::fmatReal forearmLength;
};

#  endif

#endif
