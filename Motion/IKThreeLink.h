//-*-c++-*-
#ifndef INCLUDED_IKThreeLink_h_
#define INCLUDED_IKThreeLink_h_

#include "Shared/RobotInfo.h"
#if (defined(TGT_HAS_LEGS) && !defined(TGT_IS_AIBO)) || defined(TGT_HAS_HEAD)

#include "IKSolver.h"

//! Performs analytical solution for position control of a variety of three or fewer link configurations
/*! Link configurations must conform to one of these patterns (R=revolute joint, P=prismatic joint):
 *  - R
 *  - P
 *  - R₁R₂, where R₁ and R₂ are either parallel or orthogonal
 *  - RP, where R and P are orthogonal
 *  - R₁R₂R₃, where R₁ and R₂ are orthogonal, and R₂ and R₃ are either orthogonal (Aibo leg) or parallel (Chiara leg)
 *  - R₁R₂P, where R₁ and R₂ are orthogonal, P is orthogonal to R₂, and P intersects R₁ (Pan/Tilt camera)
 *
 *  Orientation solutions are not supported (yet?)
 *  Solutions will degrade gracefully when out of reach, either effector range or joint limit, and should return
 *  the closest possible solution.
 */
class IKThreeLink : public IKSolver {
public:
	//! constructor
	IKThreeLink() : IKSolver(), invertThird(false), hasInverseSolution(false), inverseKnee() {}
	
	virtual bool solve(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, const Position& pTgt, float posPri, const Orientation& oriTgt, float oriPri) const;
	using IKSolver::solve;
	
	virtual IKSolver::StepResult_t step(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, const Position& pTgt, float pDist, float posPri, const Orientation& oriTgt, float oriDist, float oriPri) const;
	using IKSolver::step;
	
protected:
	//! roundoff for numerical error (probably should split this for angular vs. linear values)
	static const float EPSILON;
	//! searches @a eff parents to assign @a remain mobile joints into @a links
	static unsigned int setLinks(KinematicJoint& eff, KinematicJoint* links[], unsigned int remain);
	
	//! forwards to either computeFirstLinkRevolute() or computeFirstLinkPrismatic() based on @a curlink
	void computeFirstLink(KinematicJoint& curlink, const fmat::Column<3>& Pobj, KinematicJoint& endlink, const fmat::Column<3>& Plink, bool& valid) const {
		if(curlink.jointType==KinematicJoint::PRISMATIC) {
			computeFirstLinkPrismatic(curlink,Pobj,endlink,Plink,valid);
		} else {
			computeFirstLinkRevolute(curlink,Pobj,endlink,Plink,valid);
		}
	}
	//! sets the angle of curlink based directly on the projected angle of the objective minus the projected angle of the effector point (Plink)
	void computeFirstLinkRevolute(KinematicJoint& curlink, const fmat::Column<3>& Pobj, KinematicJoint& endlink, const fmat::Column<3>& Plink, bool& valid) const;
	//! sets the angle of curlink based directly on the projected angle of the objective minus the projected angle of the effector point (Plink)
	void computeFirstLinkPrismatic(KinematicJoint& curlink, const fmat::Column<3>& Pobj, KinematicJoint& endlink, const fmat::Column<3>& Plink, bool& valid) const;
	
	//! forwards to either computeSecondLinkRevolute() or computeSecondLinkPrismatic() based on @a curlink
	void computeSecondLink(KinematicJoint& curlink, const fmat::Column<3>& Pobj, KinematicJoint& endlink, const fmat::Column<3>& Plink, bool invert, bool& valid) const {
		if(curlink.jointType==KinematicJoint::PRISMATIC) {
			computeSecondLinkPrismatic(curlink,Pobj,endlink,Plink,invert,valid);
		} else {
			computeSecondLinkRevolute(curlink,Pobj,endlink,Plink,invert,valid);
		}
	}
	//! sets the angle of curlink based on the elevation of the objective point vs effector point, projecting about parent link's z axis (which will be set from computeFirstLink())
	void computeSecondLinkRevolute(KinematicJoint& curlink, const fmat::Column<3>& Pobj, KinematicJoint& endlink, const fmat::Column<3>& Plink, bool invert, bool& valid) const;
	//! sets the length of curlink based on the distance of the objective point, projecting about parent link's z axis (which will be set from computeFirstLink())
	void computeSecondLinkPrismatic(KinematicJoint& curlink, const fmat::Column<3>& Pobj, KinematicJoint& endlink, const fmat::Column<3>& Plink, bool invert, bool& valid) const;
	
	//! forwards to either computeThirdLinkRevolute() or computeThirdLinkPrismatic() based on @a curlink
	void computeThirdLink(KinematicJoint& curlink, const fmat::Column<3>& Pobj, KinematicJoint& endlink, const fmat::Column<3>& Plink, bool invert, bool& valid) const {
		if(curlink.jointType==KinematicJoint::PRISMATIC) {
			computeThirdLinkPrismatic(curlink,Pobj,endlink,Plink,invert,valid);
		} else {
			computeThirdLinkRevolute(curlink,Pobj,endlink,Plink,invert,valid);
		}
	}
	//! sets the angle of curlink based on length of thigh (distance from curlink to parent origin), and distance from curlink to effector point, to achieve desired distance from parent to effector.
	void computeThirdLinkRevolute(KinematicJoint& curlink, const fmat::Column<3>& Pobj, KinematicJoint& endlink, const fmat::Column<3>& Plink, bool invert, bool& valid) const;
	//! sets the length of curlink based on length of thigh (distance from curlink to parent origin), the angle between the parent and effector point, and the distance from parent to objective.
	void computeThirdLinkPrismatic(KinematicJoint& curlink, const fmat::Column<3>& Pobj, KinematicJoint& endlink, const fmat::Column<3>& Plink, bool invert, bool& valid) const;
	
	//! solves for a prismatic link, pass the objective distance squared, the "neck" distance squared, and the inner product of the neck angle, i.e. inner = neck·linkZ = neckD · 1 · cos(neckAng)
	static fmat::fmatReal computePrismaticQ(fmat::fmatReal objD2, fmat::fmatReal neckD2, fmat::fmatReal inner);
	
	//! ensures that @a t is in the range ±π  (upper boundary may not be inclusive...?)
	static float normalize_angle(float x) { return x - static_cast<float>( rint(x/(2*M_PI)) * (2*M_PI) ); }
	
	bool invertThird; //!< there are two knee solutions, this will choose the non-default solution.
	
	mutable bool hasInverseSolution; //!< set to true if there is another solution within range
	mutable float inverseKnee; //!< alternative angle for knee

private:
	//! holds the class name, set via registration with the DeviceDriver registry
	static const std::string autoRegisterIKThreeLink;
};
/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif

#endif
