#if (defined(TGT_IS_CALLIOPE) || defined(TGT_IS_CALLIOPE3)) && defined(TGT_HAS_ARMS)

#include "IKCalliope.h"
#include "IKGradientSolver.h"
#include "Kinematics.h"

const float IKCalliope::EPSILON = 1e-3;
const std::string IKCalliope::autoRegisterIKCalliope = IKSolver::getRegistry().registerType<IKCalliope>("IKCalliope");

#if defined(TGT_IS_CALLIOPE3)

IKCalliope::IKCalliope() {}

bool IKCalliope::solve(const Point& pEff, const Rotation& oriEff, KinematicJoint& j,
                       const Position& pTgt, float posPri, 
                       const Orientation& oriTgt, float oriPri) const {
  if ( j.outputOffset != GripperFrameOffset ) {
		throw std::runtime_error("IKCalliope::solve only accepts arm gripper joints.");
    return false;
  }
  KinematicJoint *shoulder = j.getParent();
  KinematicJoint *base = shoulder->getParent();
  shoulder->tryQ(0.0);   // *** hack for now
  base->tryQ(0.5); // *** hack for now
  return true;
}

IKSolver::StepResult_t IKCalliope::step(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, const Position& pTgt, float pDist, float posPri, const Orientation& oriTgt, float oriDist, float oriPri) const {
	// find the closest target point
	fmat::Transform Tj = j.getFullT();
	fmat::Column<3> pEffBase(Tj*pEff);
	fmat::Quaternion qj = j.getWorldQuaternion();
	Rotation oEffBase = qj*oriEff;
	Point de;
	pTgt.computeErrorGradient(pEffBase,oEffBase,de);
	StepResult_t res=SUCCESS;
	if(de.norm()>pDist) {
		de*=pDist/de.norm();
		res = PROGRESS;
	}
	Point pEffBaseP(pEffBase[1],pEffBase[2],pEffBase[3]);
	Point pTgtP = pEffBaseP+de;
	if(solve(pEff,oriEff,j,pTgtP,posPri,oriTgt,oriPri))
		return res;
	return LIMITS;
}

#elif defined(TGT_IS_CALLIOPE5)

//================ Calliope5 version ================

IKCalliope::IKCalliope() : L1(), L2(), jointLimits(), qGripper(0), qWristRot(0), qOffset(0),
			   baseToArm(), baseToArmRot(), shoulderOffset(0) {
	const KinematicJoint* j = kine->getKinematicJoint(ArmWristOffset);
	if ( j == NULL )
		throw std::runtime_error("IKCalliope can't find the ARM:wrist joint!");
	
	L1 = j->getParent()->r;
	L2 = j->r;
	jointLimits[0][0] = j->getParent()->getParent()->qmin;
	jointLimits[0][1] = j->getParent()->getParent()->qmax;
	jointLimits[1][0] = j->getParent()->qmin;	
	jointLimits[1][1] = j->getParent()->qmax;
	jointLimits[2][0] = j->qmin;
	jointLimits[2][1] = j->qmax;
	
	fmat::SubVector<2> gripperToBase(kine->linkToLink(GripperFrameOffset, ArmBaseOffset).translation());
	fmat::SubVector<2> wristToBase(kine->linkToLink(ArmWristOffset, ArmBaseOffset).translation());
	fmat::SubVector<2> wristRotToBase(kine->linkToLink(WristRotateOffset, ArmBaseOffset).translation());
	qGripper = fmat::atan(gripperToBase) - fmat::atan(wristToBase);
	qWristRot = fmat::atan(wristRotToBase) - fmat::atan(wristToBase);
	qOffset = kine->getKinematicJoint(ArmBaseOffset)->qOffset;
	baseToArm = kine->getKinematicJoint(ArmBaseOffset)->getTo().inverse();
	baseToArmRot = fmat::Quaternion::aboutZ(-qOffset) * fmat::Quaternion::fromMatrix(baseToArm.rotation());
	shoulderOffset = kine->getKinematicJoint(ArmShoulderOffset)->getTo().translation()[0];
}

/*****************

This code doesn't set the wrist or wristrot angle properly if the
effector is GripperFrame.  Should check if oriTgt is a Parallel
constraint, and if so, if y=1 set wristrot to pi/2; if z=1 then
solve 2-link IK to put the wrist above the target and then set phi
to point the fingers down.

*/

bool IKCalliope::solve(const Point& pEff, const Rotation& oriEff, KinematicJoint& j,
		       const Position& pTgt, float posPri,
		       const Orientation& oriTgt, float oriPri) const {
	// we can only work on arm joints
	if ( j.outputOffset != GripperFrameOffset &&
	     (j.outputOffset < ArmBaseOffset || j.outputOffset > WristRotateOffset) ) {
		throw std::runtime_error("IKCalliope::solve only accepts arm joints.");
		return false;
	}
	
	if ( ! IKCalliope::analyticallySolvable(oriEff, pTgt, oriTgt) )
		return IKCalliope::gradientSolve(pEff, oriEff, j, pTgt, posPri, oriTgt, oriPri);

	// translate target point from BaseFrame to the frame of the ARM:base.
	const Point& pTgtPoint = dynamic_cast<const Point&>(pTgt);
	Point pTgtBase(baseToArm * pTgtPoint);
	
	// get a rotation value for the target
	Rotation oriTgt2;
	if ( dynamic_cast<const Rotation*>(&oriTgt) != NULL )
		oriTgt2 = dynamic_cast<const Rotation&>(oriTgt);
	else { // must be Parallel
		const Parallel &par = dynamic_cast<const Parallel&>(oriTgt);
		oriTgt2 = fmat::Quaternion::fromAxis(fmat::pack(par.x, par.y, par.z));
	}
	//const Rotation& oriTgt2 = dynamic_cast<const Rotation&>(oriTgt);
	
	// target orientation in the target link's frame
	Rotation oriTgtV;
	
	// matrix representation of oriTgtV
	fmat::Matrix<3,3> ori;
	
	// angle of the base joint, offset included
	float qBase;
	
	// phi, for 3-link IK
	float y, z;
	
	// select whether we focus on position, or not
	bool position = posPri >= oriPri ? true : false;
	
	if (j.outputOffset == ArmShoulderOffset || j.outputOffset == ArmElbowOffset || j.outputOffset == ArmWristOffset) {
		// rotate -90° about the x axis
		oriTgtV = fmat::Quaternion::aboutX(-M_PI_2) * baseToArmRot * oriTgt2;
		ori = oriTgtV.toMatrix();
		qBase = std::atan2(pTgtBase.y, pTgtBase.x) - qOffset;
		
		// qBase = std::acos(oriTgtV.toMatrix()(2,2));
	}
	else if (j.outputOffset == WristRotateOffset) {
		// rotate 90° about the y axis
		oriTgtV = fmat::Quaternion::aboutY(M_PI_2) * baseToArmRot * oriTgt2;
		ori = oriTgtV.toMatrix();
		
		// determine phi
		if (std::abs(ori(0,2)) - 1.f <= EPSILON) {
			z = (std::acos(ori(1,1)) + std::asin(ori(1,0))) / 2.f;
			y = std::asin(ori(0,2));
		}
		else {
			float cosZ = -ori(0,0) / std::cos(std::asin(ori(0,2)));
			z = std::acos(cosZ);
			y = std::atan2(ori(0,2), ori(0,0) / cosZ);
		}
		qBase = std::atan2(pTgtBase.y, pTgtBase.x) - qOffset - qWristRot;
		
		// qBase = std::asin(oriTgtV.toMatrix()(1,2) / std::cos(y)) - qWristRot;
	}
	else if (j.outputOffset == GripperFrameOffset) {
		// rotate 90° about the y axis
		oriTgtV = fmat::Quaternion::aboutY(M_PI_2) * baseToArmRot * oriTgt2;
		ori = oriTgtV.toMatrix();
		
		// determine phi
		if (std::abs(ori(0,2)) - 1.f <= EPSILON) {
			z = (std::acos(ori(1,1)) + std::asin(ori(1,0))) / 2.f;
			y = std::asin(ori(0,2));
		}
		else {
			float cosZ = -ori(0,0) / std::cos(std::asin(ori(0,2)));
			z = std::acos(cosZ);
			y = std::atan2(ori(0,2), ori(0,0) / cosZ);
		}
		qBase = std::atan2(pTgtBase.y, pTgtBase.x) - qOffset - qGripper;
		
		// qBase = std::asin(oriTgtV.toMatrix()(1,2) / std::cos(y)) - qGripper;
	}
	else if (j.outputOffset == ArmBaseOffset) {
		// no rotation other than to ArmBase
		oriTgtV = baseToArmRot * oriTgt2;
		ori = oriTgtV.toMatrix();
		
		// take yaw angle
		qBase = oriTgtV.ypr()[0];
	}
	
	// if we're just solving for the base or shoulder, we're done.
	if (j.outputOffset == ArmBaseOffset) {
		return j.tryQ(qBase);
	}
	else if (j.outputOffset == ArmShoulderOffset) {
		j.getParent()->tryQ(qBase);
		j.tryQ(std::acos(ori(1,1)));
		return (j.getWorldPosition() - pTgtPoint).sumSq() < 25.0f;
	}
	
	// If we get here, we're solving for the elbow, wrist, or gripper.
	// Convert target point to shoulder reference frame.
	fmat::Column<2> pTgtV;
	float realQ = qBase + qOffset;
	if (j.outputOffset == WristRotateOffset) realQ += qWristRot;
	if (j.outputOffset == GripperFrameOffset) realQ += qGripper;
	if (std::abs(std::cos(realQ)) < EPSILON)
		pTgtV[0] = pTgtBase.y;
	else
		pTgtV[0] = pTgtBase.x / std::cos(realQ);
	pTgtV[1] = pTgtBase.z;
	// shift from ARM:base -> ARM:shoulder for new origin for IK
	pTgtV[0] -= shoulderOffset;
	
	if (j.outputOffset == ArmElbowOffset) {
		j.getParent()->getParent()->tryQ(qBase);
		
		float qShoulder = fmat::atan(pTgtV);
		j.getParent()->tryQ(qShoulder);
		
		// if oriTgtV, solve for shoulder joint
		if (!position)
			j.tryQ(std::acos(ori(1,1)) - qShoulder);
		
		return (j.getWorldPosition() - pTgtPoint).sumSq() < 25.0f;
	}
	else if (j.outputOffset == ArmWristOffset) {
		j.getParent()->getParent()->getParent()->tryQ(qBase);
		
		// 3-link IK
		Solutions s = closestThreeLinkIK(pTgtV, position, std::acos(ori(1,1)), fmat::ZERO2);
		
		int c = closestSolution(j, s);
		j.getParent()->getParent()->tryQ(s.angles(c,0));
		j.getParent()->tryQ(s.angles(c,1));
		j.tryQ(s.angles(c,2));
		
		return s.valid;
	}
	else if (j.outputOffset == WristRotateOffset) {
		j.getParent()->getParent()->getParent()->getParent()->tryQ(qBase);
		
		// 3-link IK
		Solutions s = closestThreeLinkIK(pTgtV, position, y, fmat::pack(0, j.r));
		
		int c = closestSolution(j, s);
		j.getParent()->getParent()->getParent()->tryQ(s.angles(c,0));
		j.getParent()->getParent()->tryQ(s.angles(c,1));
		j.getParent()->tryQ(s.angles(c,2));
		j.tryQ(z + qBase);
		
		return s.valid;
	}
	else if (j.outputOffset == GripperFrameOffset) {
		j.getParent()->getParent()->getParent()->getParent()->getParent()->tryQ(qBase);
		
		// 3-link IK
		Solutions s = closestThreeLinkIK(pTgtV, position, y, fmat::pack(-j.d, j.getParent()->r));
		
		int c = closestSolution(j, s);
		j.getParent()->getParent()->getParent()->getParent()->tryQ(s.angles(c,0));
		j.getParent()->getParent()->getParent()->tryQ(s.angles(c,1));
		j.getParent()->getParent()->tryQ(s.angles(c,2));
		j.getParent()->tryQ(z + qBase);
		
		return s.valid;
	}
	else
		// should never get here
		return false;
}

IKCalliope::Solutions IKCalliope::closestThreeLinkIK(const fmat::Column<2>& t, bool posPri, float prefPhi, const fmat::Column<2>& delta) const {
	fmat::Column<2> target = t + fmat::rotation2D(prefPhi) * delta;
	IKCalliope::Solutions solutions = threeLinkIK(target, prefPhi);	
	if (solutions.valid)
		return solutions;
	
	else {
		/* If solution cannot be found with specified phi,
		 try ±{1,2,3,...,180} degrees until achievable angles are found */
		float pstep = M_PI/180;
		for (float deltaPhi = pstep; deltaPhi <= M_PI; deltaPhi += pstep) {
			AngSignPi angleUp(prefPhi + deltaPhi);
			target = t + fmat::rotation2D(angleUp) * delta;
			solutions = threeLinkIK(target, angleUp);
			if (solutions.valid) {
				if (!posPri) {
					assureOrientation(solutions, prefPhi);
					solutions.valid = false;
				}
				return solutions;
			}
			
			AngSignPi angleDown(prefPhi - deltaPhi);
			target = t + fmat::rotation2D(angleDown) * delta;
			solutions = threeLinkIK(target, angleDown);
			if (solutions.valid) {
				if (!posPri) {
					assureOrientation(solutions, prefPhi);
					solutions.valid = false;
				}
				return solutions;
			}
		}
	}
	
	return pointToward(t, posPri, prefPhi);
}

IKCalliope::Solutions IKCalliope::threeLinkIK(fmat::Column<2> t, float phi) const {
	IKCalliope::Solutions solutions;
	fmat::Matrix<2,2> sols_2link;
	fmat::Column<2> q3;
	float c2 = (t.sumSq() - L1*L1 - L2*L2) / (2*L1*L2);
	if(c2*c2 > 1) {
		solutions.noSols = 0;
		solutions.valid = false;
		return solutions;
	}
	else {
		sols_2link = twoLinkIK(t);
		q3[0] = phi - sols_2link(0,0) - sols_2link(0,1);
		q3[1] = phi - sols_2link(1,0) - sols_2link(1,1);
		solutions.angles(0,0) = sols_2link(0,0); solutions.angles(0,1) = sols_2link(0,1); solutions.angles(0,2) = q3[0];
		solutions.angles(1,0) = sols_2link(1,0); solutions.angles(1,1) = sols_2link(1,1); solutions.angles(1,2) = q3[1];
		solutions.noSols = 2;
		solutions = validAngles(solutions);
		return solutions;
	}
}

IKCalliope::Solutions IKCalliope::pointToward(const fmat::Column<2>& target, bool posPri, float prefPhi) const {
	IKCalliope::Solutions solutions;
	float q1 = fmat::atan(target), q2 = 0.0f, q3 = 0.0f;
	q1 = std::min(std::max(q1,jointLimits[0][0]),jointLimits[0][1]);
	solutions.angles(0,0) = q1; solutions.angles(0,1) = q2;
	if (posPri)
		solutions.angles(0,2) = q3;
	else
		solutions.angles(0,2) = prefPhi - q2 - q1;
	solutions.noSols = 1;
	solutions.valid = false;
	return solutions;
}

fmat::Matrix<2,2> IKCalliope::twoLinkIK(fmat::Column<2> t) const {
	fmat::Matrix<2,2> ret;
	float c2 = (t.sumSq() - L1*L1 - L2*L2) / (2*L1*L2);
	float s2 = std::sqrt(1 - c2*c2);
	float q2 = std::atan2(s2, c2);
	float kQ = std::atan2(L2*s2,L1 + L2*c2);
	float q1plus = fmat::atan(t) + kQ;
	float q1minus = fmat::atan(t) - kQ;
	ret(0,0) = q1plus; ret(0,1) = -q2;
	ret(1,0) = q1minus; ret(1,1) = +q2;
	return ret;
}

int IKCalliope::closestSolution(KinematicJoint& j, IKCalliope::Solutions s) const {
	if (!(s.valid && s.noSols == 2))
		return 0;
	
	KinematicJoint* jP = &j;
	
	while (jP->outputOffset != ArmElbowOffset)
		jP = jP->getParent();
	
	float jVal = jP->getQ();
	
	// if we're sticking straight up, we want to go for the elbow up solution.
	if (jVal < EPSILON)
		return (s.angles(1,1) > s.angles(0,1) ? 0 : 1);
	
	float a0 = jVal - s.angles(0,1);
	float a1 = jVal - s.angles(1,1);
	float d0 = a0 * a0;
	float d1 = a1 * a1;
	
	return (d0 < d1 ? 0 : 1);
}

void IKCalliope::assureOrientation(IKCalliope::Solutions &s, float phi) const {
	for (int i = 0; i < 2; i++) {
		float q2 = phi - s.angles(i,1) - s.angles(i,0);
		float q2Diff = 0;
		if (q2 > jointLimits[2][1]) {
			q2Diff = jointLimits[2][1] - q2;
			s.angles(i,2) = jointLimits[2][1];
		}
		else if (q2 < jointLimits[2][0]) {
			q2Diff = jointLimits[2][0] - q2;
			s.angles(i,2) = jointLimits[2][0];
		}
		else {
			s.angles(i,2) = q2;
			continue;
		}
		
		// if the wrist angle can't get to where we want it, modify the elbow
		float q1 = s.angles(i,1) - q2Diff;
		float q1Diff = 0;
		if (q1 > jointLimits[1][1]) {
			q1Diff = jointLimits[1][1] - q1;
			s.angles(i,1) = jointLimits[1][1];
		}
		else if (q1 < jointLimits[1][0]) {
			q1Diff = jointLimits[1][0] - q1;
			s.angles(i,1) = jointLimits[1][0];
		}
		else {
			s.angles(i,1) = q1;
			continue;
		}
		
		// if the elbow angle can't get to where we want it, modify the shoulder
		s.angles(i,0) = s.angles(i,0) - q1Diff;
		continue;
	}
	
	return;
}

IKCalliope::Solutions IKCalliope::validAngles(IKCalliope::Solutions solutions) const {
	int valid_solutions[2] = {0, 0}; // assume both solutions are invalid
	for (int sol_no = 0; sol_no < solutions.noSols; sol_no++) {
		// Check for joint limits, any solution with a joint outside the limits is an invalid solution
		int val_angles = 0;
		for (int joint_no = 0; joint_no < 3; joint_no++)
			if (solutions.angles(sol_no,joint_no) > jointLimits[joint_no][0] && solutions.angles(sol_no,joint_no) < jointLimits[joint_no][1])
				val_angles++;
			
		if (val_angles == 3)
			valid_solutions[sol_no] = 1;
	}	
	if (valid_solutions[0]+valid_solutions[1] == 2) {      // both solutions were valid
		solutions.noSols = 2;
		solutions.valid = true;
		return solutions;
	}
	else if (valid_solutions[0]+valid_solutions[1] == 0) { // both solutions were invalid
		solutions.noSols = 0;
		solutions.valid = false;
		return solutions;
	}
	else {                                                  // one solution was invalid
		if (valid_solutions[0] == 0)
			solutions.angles.row(0) = solutions.angles.row(1);
		solutions.noSols = 1;
		solutions.valid = true;
		return solutions;
	}
}

IKSolver::StepResult_t IKCalliope::step(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, const Position& pTgt, float pDist, float posPri, const Orientation& oriTgt, float oriDist, float oriPri) const {
	// find the closest target point
	fmat::Transform Tj = j.getFullT();
	fmat::Column<3> pEffBase(Tj*pEff);
	fmat::Quaternion qj = j.getWorldQuaternion();
	Rotation oEffBase = qj*oriEff;
	Point de;
	pTgt.computeErrorGradient(pEffBase,oEffBase,de);
	StepResult_t res = SUCCESS;
	if(de.norm() > pDist) {
		de *= pDist/de.norm();
		res = PROGRESS;
	}
	Point pEffBaseP(pEffBase[1],pEffBase[2],pEffBase[3]);
	Point pTgtP = pEffBaseP+de;
	if(solve(pEff,oriEff,j,pTgtP,posPri,oriTgt,oriPri))
		return res;
	return LIMITS;
}

bool IKCalliope::isSideGrasp(const Rotation &oriEff, const Orientation &oriTgt) {
	const Parallel *par = dynamic_cast<const Parallel*>(&oriTgt);
	return (par != NULL && *par == Parallel(0,0,1) &&
					oriEff == Rotation(fmat::Quaternion::aboutX(-M_PI/2)));
}

bool IKCalliope::isOverheadGrasp(const Rotation &oriEff, const Orientation &oriTgt) {
	const Parallel *par = dynamic_cast<const Parallel*>(&oriTgt);
	return (par != NULL && *par == Parallel(0,0,1) && oriEff == Rotation());
}


bool IKCalliope::analyticallySolvable(const Rotation &oriEff,
																			const Position &pTgt, const Orientation &oriTgt) {
	// Can handle position = Point but not Line or Plane
	if ( dynamic_cast<const Point*>(&pTgt) == NULL )
		return false;
	// Can always handle orientation = Rotation
	if ( dynamic_cast<const Rotation*>(&oriTgt) != NULL )
		return true;
	return false;  //**** force use of gradient solver for now
	// Can handle two special orientation = Parallel cases
	if ( isSideGrasp(oriEff,oriTgt) || isOverheadGrasp(oriEff,oriTgt) )
		return true;
	// If we reach here, we have an unusual Parallel constraint: punt
	return false;
}

bool IKCalliope::gradientSolve(const Point& pEff, const Rotation& oriEff,
															 KinematicJoint& j, 
															 const Position& pTgt, float posPri, 
															 const Orientation& oriTgt, float oriPri) {
	static IKGradientSolver solver(750, 0.5f, 0.001f, 0.2/5000);
	j.getParent()->tryQ(M_PI/2);
	j.getParent()->getParent()->tryQ(1.38);
	j.getParent()->getParent()->getParent()->tryQ(-2.28);
	j.getParent()->getParent()->getParent()->getParent()->tryQ(1.01);
	j.getParent()->getParent()->getParent()->getParent()->getParent()->tryQ(0.);
	std::cout << "Calling gradientSolve" << std::endl;
	return solver.solve(pEff, oriEff, j, pTgt, posPri, oriTgt, oriPri);
}

#  elif defined(TGT_IS_CALLIOPE2)

//================ Calliope2 Version ================

IKCalliope::IKCalliope() : baseToArm(), qOffset(), gripperOffset(0), forearmLength() {
	const KinematicJoint* j = kine->getKinematicJoint(GripperFrameOffset);
	
	forearmLength = (j->getWorldPosition() - j->getParent()->getWorldPosition()).norm();
	qOffset = kine->getKinematicJoint(ArmBaseOffset)->qOffset;
	gripperOffset = kine->getKinematicJoint(GripperFrameOffset)->qOffset;
	baseToArm = kine->getKinematicJoint(ArmBaseOffset)->getTo().inverse();
}

bool IKCalliope::solve(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, const Position& pTgt, float posPri, const Orientation& oriTgt, float oriPri) const {
	// translate target point from BaseFrame to the frame of the ARM:base.
	const Point& pTgtPoint = dynamic_cast<const Point&>(pTgt);
	Point pTgtBase(baseToArm * pTgtPoint);
	Point pTgtShoulder(kine->getKinematicJoint(ArmShoulderOffset)->getTo().inverse() * pTgtPoint);
	
	// use std::atan2 to solve for base link
	/* note that in reality, the GripperFrame is shifted 1.283cm to the left of the
	 axis pointed to by the arm, and that this solver does not take into account
	 that shift due to its insignificance. */
	float qBase = std::atan2(pTgtBase.y, pTgtBase.x);
	j.getParent()->getParent()->tryQ(qBase-qOffset);
	
	/* we want the Z component of the specified target point to line up with the Gripper
	 so that perhaps from there, the robot can move forward to touch the point. */
	float shoulderHeight = kine->getPosition(ArmShoulderOffset)[2];
	float opposite = pTgtShoulder[1] - shoulderHeight - gripperOffset;
	// std::cout << "pTgtBase=" << pTgtBase << "   pTgtShoulder=" << pTgtShoulder
	//	  << "  gripperOffset=" << gripperOffset << std::endl;
	float qShoulder;
	if (std::abs(opposite) < forearmLength)
		qShoulder = std::asin(opposite / forearmLength);
	else
		qShoulder = opposite > 0 ? M_PI_2 : -M_PI_2;
	j.getParent()->tryQ(qShoulder);
	
	return (j.getWorldPosition() - pTgtPoint).norm() < 5.0f;
}

IKSolver::StepResult_t IKCalliope::step(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, const Position& pTgt, float pDist, float posPri, const Orientation& oriTgt, float oriDist, float oriPri) const {
	// find the closest target point
	fmat::Transform Tj = j.getFullT();
	fmat::Column<3> pEffBase(Tj*pEff);
	fmat::Quaternion qj = j.getWorldQuaternion();
	Rotation oEffBase = qj*oriEff;
	Point de;
	pTgt.computeErrorGradient(pEffBase,oEffBase,de);
	StepResult_t res=SUCCESS;
	if(de.norm()>pDist) {
		de*=pDist/de.norm();
		res = PROGRESS;
	}
	Point pEffBaseP(pEffBase[1],pEffBase[2],pEffBase[3]);
	Point pTgtP = pEffBaseP+de;
	if(solve(pEff,oriEff,j,pTgtP,posPri,oriTgt,oriPri))
		return res;
	return LIMITS;
}

#  endif  // Calliope2


#endif // TGT_IS_CALLIOPE && TGT_HAS_ARMS
