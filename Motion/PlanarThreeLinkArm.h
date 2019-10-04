//-*-c++-*-
#ifndef INCLUDED_PlanarThreeLinkArm_h_
#define INCLUDED_PlanarThreeLinkArm_h_

#ifdef TGT_IS_HANDEYE

#include "Motion/IKSolver.h"
#include "Shared/fmatCore.h"
#include "Motion/Kinematics.h"
#include "Shared/RobotInfo.h"

//! Kinematics solver for an arm composed of three consecutive joints in the same plane
class PlanarThreeLinkArm : public IKSolver {
public:
	//! Constructor: use the last three links in the chain
	PlanarThreeLinkArm();
	
	//! Constructor: @offset specifies the first joint in the chain
	PlanarThreeLinkArm(unsigned int offset);
	
	using IKSolver::solve;
	using IKSolver::step;
	
	//! Solve to get an 'effector' (@a pEff, @a oriEff, relative to link following @a j) to a solution of @a pTgt, @a oriTgt (or at least a local minimum)
	/*! @a posPri and @a oriPri specify relative weighting of each solution in case they are mutually exclusive */
	virtual bool solve(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, const Position& pTgt, float posPri, const Orientation& oriTgt, float oriPri) const;
	virtual StepResult_t step(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, const Position& pTgt, float pDist, float posPri, const Orientation& oriTgt, float oriDist, float oriPri) const;	
	
	struct Solutions {
		Solutions() : angles(),noSols(),valid() {}
		fmat::Matrix<2,3> angles; // double check all references to Solutions.angles
		//fmat::Matrix<3,2> angles;  // This is what is should be!
		int noSols;
		bool valid;
	};
	
	void PrintVariables() const;
	PlanarThreeLinkArm::Solutions solveWithOffset(float x, float y, float pref_phi, fmat::Column<3> baseOffset = fmat::pack(0.0f, 0.0f,1.0f)) const;
	PlanarThreeLinkArm::Solutions invKin3LinkRelaxPhi(float x, float y, float pref_phi) const;
	PlanarThreeLinkArm::Solutions nearConfig(PlanarThreeLinkArm::Solutions sol, float elbow, bool valid) const;
	PlanarThreeLinkArm::Solutions punt(float x, float y) const;
	PlanarThreeLinkArm::Solutions validAngles(PlanarThreeLinkArm::Solutions solutions) const;
	PlanarThreeLinkArm::Solutions invKin3Link(float x,float y,float phi) const;
	fmat::Matrix<2,2> invKin2Link(float x,float y,float link1,float link2) const;
	PlanarThreeLinkArm::Solutions invKin2Link(float x,float y) const;
	float angDist(float a1,float a2) const;
	float angNorm(float a) const;
	fmat::Matrix<2,2> rot(float t) const;
	//get Joint lengths 
	float getL1() const;
	float getL2() const;
	float getL3() const;
	//set Joint lengths
	void setL1(float length1, float jointMax, float jointMin);
	void setL2(float length2, float jointMax, float jointMin);
	void setL3(float length3, float jointMax, float jointMin);
	
private:
	//! holds the class name, set via registration with the DeviceDriver registry
	static const std::string autoRegisterPlanarThreeLinkArm;
	float L1, L2, L3;
	float jointLimits[3][2];
};

#endif  // TGT_IS_HANDEYE

#endif
