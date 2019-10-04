#include "Shared/RobotInfo.h"
#ifdef TGT_IS_HANDEYE

#include "Shared/debuget.h"
#include "PlanarThreeLinkArm.h"
#include "Shared/fmatSpatial.h"
#include <math.h>

const std::string PlanarThreeLinkArm::autoRegisterPlanarThreeLinkArm = IKSolver::getRegistry().registerType<PlanarThreeLinkArm>("PlanarThreeLinkArm");

PlanarThreeLinkArm::PlanarThreeLinkArm() : L1(), L2(), L3(), jointLimits() {
	//    unsigned int lastOff = capabilities.findOutputOffset("ARM:wristYaw");
	unsigned int lastOff;
#if defined(TGT_CHIARA) || defined(TGT_HANDEYE)
	lastOff = RobotInfo::GripperFrameOffset;
#elif defined(TGT_HAS_ARMS)
	lastOff = ArmOffset+NumArmJoints-1;
#else
	return;
#endif
	
	const KinematicJoint* j = kine->getKinematicJoint(lastOff);
	if ( j == NULL )
		throw std::runtime_error("PlanarThreeLinkArm can't find joint for end of arm");
	
	
	L1 = j->getParent()->getParent()->r;
	L2 = j->getParent()->r;
	L3 = j->r;
	
#if defined(TGT_HANDEYE)
	j = j->getParent();
#endif
	
	jointLimits[0][0] = j->getParent()->getParent()->qmin;
	jointLimits[0][1] = j->getParent()->getParent()->qmax;
	jointLimits[1][0] = j->getParent()->qmin;	
	jointLimits[1][1] = j->getParent()->qmax;
	jointLimits[2][0] = j->qmin;
	jointLimits[2][1] = j->qmax;
    
	//std::cout << "L1=" << L1 << " L2=" << L2 << " L3=" << L3 << std::endl;
}

PlanarThreeLinkArm::PlanarThreeLinkArm(unsigned int offset) : L1(), L2(), L3(), jointLimits() {
	float linkLengths[3];
	for ( unsigned int j = offset; j < offset+3; j++) {
		linkLengths[j-offset] = kine->getKinematicJoint(j)->nextJoint()->r;
		jointLimits[j-offset][0] = kine->getKinematicJoint(j)->qmin; 
		jointLimits[j-offset][1] = kine->getKinematicJoint(j)->qmax;
	}
	
	L1 = linkLengths[0];
	L2 = linkLengths[1];
	L3 = linkLengths[2];
	
	//std::cout << "L1=" << L1 << " L2=" << L2 << " L3=" << L3 << std::endl;
}


void PlanarThreeLinkArm::PrintVariables() const { 
	std::cout << "L1: " << L1 << "\tL2: " << L2 << "\tL3 :" << L3 << std::endl;
	std::cout << "Shoulder joint limits:\t" << jointLimits[0][0] << "\t" << jointLimits[0][1] << std::endl;
	std::cout << "Elbow joint limits:   \t" << jointLimits[1][0] << "\t" << jointLimits[1][1] << std::endl;
	std::cout << "Wrist joint limits:   \t" << jointLimits[2][0] << "\t" << jointLimits[2][1] << std::endl;
}

bool PlanarThreeLinkArm::solve(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, const Position& pTgt, float posPri, const Orientation& oriTgt, float oriPri) const {
	
	// use const Position& pTgt as the target postion
	const Point& pTgtV = dynamic_cast<const Point&>(pTgt);
	
	// use const Orientation& oriTgt as target orientation
	const Rotation& oriTgtV = dynamic_cast<const Rotation&>(oriTgt);
	float ori = oriTgtV.axisComponent(fmat::UNIT3_Z);
	
	// Determine where the Gripper center should be
	fmat::Column<2> target = fmat::rotation2D(ori) * fmat::pack(pEff.x, pEff.y) + fmat::pack(pTgtV.x, pTgtV.y);
	
	PlanarThreeLinkArm::Solutions solutions = invKin3LinkRelaxPhi(target[0], target[1], ori);
	
	j.getParent()->setQ(solutions.angles(0,2));
	j.getParent()->getParent()->setQ(solutions.angles(0,1));
	j.getParent()->getParent()->getParent()->setQ(solutions.angles(0,0));
	return true;
}

PlanarThreeLinkArm::Solutions PlanarThreeLinkArm::invKin3LinkRelaxPhi(float x, float y, float pref_phi) const {
	PlanarThreeLinkArm::Solutions solutions;
	float delta_phi;	
	if ( sqrt(x*x + y*y) > (L1+L2+L3) ) { // punt if point is out of reach
		return punt(x, y);
	}
	
	solutions = invKin3Link(x,y,pref_phi);	
	if( solutions.valid )
		return solutions;
	// Otherwise change last_phi by 10 degrees until achievable angles are found
	else {
		for( int i = 1; i <= 18; i++) {
			delta_phi = i*10*(float)M_PI/180;
			solutions = invKin3Link(x,y,angNorm(pref_phi + delta_phi));
			if( solutions.valid ) {
				return solutions;
			}
			solutions = invKin3Link(x,y,angNorm(pref_phi - delta_phi));
			if( solutions.valid ) {
				return solutions;
			}
		}
	}	
	solutions.noSols = 0;  solutions.valid = false;
	return solutions;
}

PlanarThreeLinkArm::Solutions PlanarThreeLinkArm::solveWithOffset(float x, float y, float pref_phi, fmat::Column<3> baseOffset) const {
	fmat::Column<3> basePoint = fmat::pack(x, y, 0);
	
	fmat::Column<3> offset = fmat::rotationZ(pref_phi) * baseOffset;
	fmat::Column<3> point = basePoint + offset;
	return invKin3Link(point[0], point[1], pref_phi);
}

PlanarThreeLinkArm::Solutions PlanarThreeLinkArm::invKin3Link(float x,float y,float phi) const {
	PlanarThreeLinkArm::Solutions solutions;
	fmat::Matrix<1,2> d3, p2;
	fmat::Matrix<2,2> sols_2link;
	fmat::Column<2> q3;	
	d3(0,0) = L3 * std::cos(phi); d3(0,1) = L3 * std::sin(phi);
	p2(0,0) = x - d3(0,0); p2(0,1) = y - d3(0,1);
	float p2x = p2(0,0);    float p2y = p2(0,1);
	float c2 = (p2x*p2x + p2y*p2y - L1*L1 - L2*L2) / (2*L1*L2);
	if(c2*c2 > 1) {
		solutions.noSols = 0;
		solutions.valid = false;
		return solutions;
	}
	else {
		sols_2link = invKin2Link(p2x,p2y,L1,L2);
		q3[0] = phi - sols_2link(0,0) - sols_2link(0,1);
		q3[1] = phi - sols_2link(1,0) - sols_2link(1,1);
		solutions.angles(0,0) = sols_2link(0,0); solutions.angles(0,1) = sols_2link(0,1); solutions.angles(0,2) =  q3[0];
		solutions.angles(1,0) = sols_2link(1,0); solutions.angles(1,1) = sols_2link(1,1); solutions.angles(1,2) =  q3[1];
		solutions.noSols = 2;
		solutions = validAngles(solutions);
		return solutions;
	}
}

PlanarThreeLinkArm::Solutions PlanarThreeLinkArm::nearConfig(PlanarThreeLinkArm::Solutions sol, float elbow, bool valid) const{
	PlanarThreeLinkArm::Solutions ret_sol;	
	if ( (sol.angles(0,1)/fabs(sol.angles(0,1))) == (elbow/fabs(elbow)) ) {
		ret_sol.angles(0,0) = sol.angles(0,0); ret_sol.angles(0,1) = sol.angles(0,1); ret_sol.angles(0,2) = sol.angles(0,2);
		ret_sol.noSols = 1; ret_sol.valid = valid;
		return ret_sol;
	}
	if (sol.noSols == 2) {
		if ( (sol.angles(1,1)/fabs(sol.angles(1,1))) == (elbow/fabs(elbow)) ) {
			ret_sol.angles(0,0) = sol.angles(1,0); ret_sol.angles(0,1) = sol.angles(1,1); ret_sol.angles(0,2) = sol.angles(1,2);
			ret_sol.noSols = 1; ret_sol.valid = valid;
			return ret_sol;
		}
	}	
	ret_sol.noSols = 0;  ret_sol.valid = false;
	return ret_sol;
}

PlanarThreeLinkArm::Solutions PlanarThreeLinkArm::punt(float x, float y) const {
	PlanarThreeLinkArm::Solutions solutions;
	float q1 = std::atan2(y,x), q2 = 0.0f, q3 = 0.0f;
	q1 = std::min(std::max(q1,jointLimits[0][0]),jointLimits[0][1]);
	solutions.angles(0,0) = q1; solutions.angles(0,1) = q2; solutions.angles(0,2) = q3;
	solutions.noSols = 1;
	solutions.valid = false;
	return solutions;
}

fmat::Matrix<2,2> PlanarThreeLinkArm::invKin2Link(float x,float y,float link1,float link2) const {
	fmat::Matrix<2,2> ret;
	float c2 = (x*x + y*y - link1*link1 - link2*link2) / (2*link1*link2);
	float s2minus = std::sqrt(1-c2*c2);
	float s2plus = -std::sqrt(1-c2*c2);
	float q2minus = std::atan2(s2minus,c2);
	float q2plus = std::atan2(s2plus,c2);	
	float K1 = link1 + link2*c2;
	float K2plus = link2*s2plus;
	float K2minus = link2*s2minus;	
	float q1minus = std::atan2(y,x) - std::atan2(K2minus,K1);
	float q1plus = std::atan2(y,x) - std::atan2(K2plus,K1);	
	ret(0,0) = q1plus; ret(0,1) = q2plus;
	ret(1,0) = q1minus; ret(1,1) = q2minus;	
	return ret;
}

PlanarThreeLinkArm::Solutions PlanarThreeLinkArm::invKin2Link(float x,float y) const {
	PlanarThreeLinkArm::Solutions ret;
	float c2 = (x*x + y*y - L1*L1 - L2*L2) / (2*L1*L2);
	
	if(c2*c2 > 1){
		bool noValid = true;
		while(noValid){
			if(std::fabs(x) > std::fabs(y)){
				if(x > 0)				
					x = x - 1;
				else 
					x = x + 1;
			}
			else if (std::fabs(y) > std::fabs(x)){
				if(y > 0)				
					y = y - 1;
				else 
					y = y + 1;
			}
			else{
				if(x > 0)
					x = x - 1; 
				else
					x = x + 1;
				if(y > 0)
					y = y -	1;		
				else 
					y = y + 1;
			}
			c2 = (x*x + y*y - L1*L1 - L2*L2) / (2*L1*L2);	
			if(c2*c2 <= 1)
				noValid = false;
		}//end while
	}//end if	
	
	float s2minus = std::sqrt(1-c2*c2);
	float s2plus = -std::sqrt(1-c2*c2);
	float q2minus = std::atan2(s2minus,c2);
	float q2plus = std::atan2(s2plus,c2);	
	float K1 = L1 + L2*c2;
	float K2plus = L2*s2plus;
	float K2minus = L2*s2minus;	
	float q1minus = std::atan2(y,x) - std::atan2(K2minus,K1);
	float q1plus = std::atan2(y,x) - std::atan2(K2plus,K1);	
	ret.angles(0,0) = q1plus; ret.angles(0,1) = q2plus; ret.angles(0,2) = 0.0;
	ret.angles(1,0) = q1minus; ret.angles(1,1) = q2minus; ret.angles(1,2) = 0.0;
	return ret;
}
PlanarThreeLinkArm::Solutions PlanarThreeLinkArm::validAngles(PlanarThreeLinkArm::Solutions solutions) const {
	int valid_solutions[2] = {0, 0}; // assume both solutions are invalid
	fmat::Matrix<1,3> angles_set_1, angles_set_2;
	angles_set_1(0,0)=solutions.angles(0,0); angles_set_1(0,1)=solutions.angles(0,1); angles_set_1(0,2)=solutions.angles(0,2);
	angles_set_2(0,0)=solutions.angles(1,0); angles_set_2(0,1)=solutions.angles(1,1); angles_set_2(0,2)=solutions.angles(1,2);
	for( int sol_no = 0; sol_no < solutions.noSols; sol_no++ ) {
		// Check for joint limits, any solution with a joint outside the limits is an invalid solution
		int val_angles = 0;
		for( int joint_no = 0; joint_no < 3; joint_no++ ) {
			if( !(solutions.angles(sol_no,joint_no) < jointLimits[joint_no][0] || solutions.angles(sol_no,joint_no) > jointLimits[joint_no][1]) )
				val_angles++;
		}
		if( val_angles == 3 )
			valid_solutions[sol_no] = 1;
	}	
	if( valid_solutions[0]+valid_solutions[1] == 2 ) {      // both solutions were valid
		solutions.noSols = 2;
		solutions.valid = true;
		return solutions;
	}
	else if( valid_solutions[0]+valid_solutions[1] == 0 ) { // both solutions were invalid
		solutions.noSols = 0;
		solutions.valid = false;
		return solutions;
	}
	else {                                                  // one solution was invalid
		if( valid_solutions[0] == 0 ) {
			solutions.angles(0,0) = solutions.angles(1,0);
			solutions.angles(0,1) = solutions.angles(1,1);
			solutions.angles(0,2) = solutions.angles(1,2);
		}
		solutions.noSols = 1;
		solutions.valid = true;
		return solutions;
	}
}

float PlanarThreeLinkArm::angDist(float a1,float a2) const {
	// returns distance between two angles
	// answer is between 0 and pi
	float angle = std::fmod(std::abs(a1 - a2),float(2*M_PI));
	if( angle > (float)M_PI )
		angle = float(2*M_PI) - angle;
	return angle;
}

float PlanarThreeLinkArm::angNorm(float a) const {
	// Normalize an angle into the range -pi to pi
	float bias = (float)M_PI * (a/std::abs(a));
	float b = std::fmod(a+bias, float(2*M_PI)) - bias;
	return b;
}

fmat::Matrix<2,2> PlanarThreeLinkArm::rot(float t) const {
	// Returns a rotation matrix of t radians
	fmat::Matrix<2,2> r;
	r(0,0)=std::cos(t); r(0,1)=-std::sin(t);
	r(1,0)=std::sin(t); r(1,1)=std::sin(t);
	return r;
}

float PlanarThreeLinkArm::getL1() const{
	return (this->L1);
}

float PlanarThreeLinkArm::getL2() const{
	return (this->L2);
}

float PlanarThreeLinkArm::getL3() const{
	return (this->L3);
}

void PlanarThreeLinkArm::setL1(float length1, float jointMax, float jointMin){
	this->L1 = length1;
	this->jointLimits[0][0] = jointMin;
	this->jointLimits[0][1] = jointMax;
}

void PlanarThreeLinkArm::setL2(float length2, float jointMax, float jointMin){
	this->L2 = length2;
	this->jointLimits[1][0] = jointMin;
	this->jointLimits[1][1] = jointMax;
}

void PlanarThreeLinkArm::setL3(float length3, float jointMax, float jointMin){
	this->L3 = length3;
	this->jointLimits[2][0] = jointMin;
	this->jointLimits[2][1] = jointMax;
}

IKSolver::StepResult_t PlanarThreeLinkArm::step(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, const Position& pTgt, float pDist, float posPri, const Orientation& oriTgt, float oriDist, float oriPri) const {
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

#endif //TGT_HANDEYE
