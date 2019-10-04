#include "Shared/RobotInfo.h"
#if (defined(TGT_HAS_LEGS) && !defined(TGT_IS_AIBO)) || defined(TGT_HAS_HEAD)

#include "IKThreeLink.h"
#include "Shared/debuget.h"
#include "Shared/fmatSpatial.h"
#include <limits>

using namespace std;

const float IKThreeLink::EPSILON=1e-3f;
const std::string IKThreeLink::autoRegisterIKThreeLink = IKSolver::getRegistry().registerType<IKThreeLink>("IKThreeLink");

bool IKThreeLink::solve(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, const Position& pTgt, float posPri, const Orientation& oriTgt, float oriPri) const {
	bool converges[3] = { true, true, true };
	KinematicJoint* links[3] = { NULL, NULL, NULL };
	float origKnee;
	setLinks(j,links,3);
	
	// we're going to do an analytic solution for closest point on target surface.
	// find this target point
	fmat::Transform Tj = j.getFullT();
	fmat::Column<3> pEffBase(Tj*pEff);
	fmat::Quaternion qj = j.getWorldQuaternion(); //cout << "WorldQ: " << qj;
	Rotation oEffBase = qj*oriEff;
	fmat::Column<3> de;
	pTgt.computeErrorGradient(pEffBase,oEffBase,de);
	
	if(links[0]==NULL || posPri<=0) { // no mobile links
		// maybe we're conveniently at the solution... check error
		std::cerr << "ERROR: IKThreeLink trying to solve without any mobile joints (" << j.outputOffset.get() << ")" << std::endl;
		return de.norm()<=EPSILON;
	}
	
	origKnee = (links[2]!=NULL) ? links[2]->getQ() : 0;
	hasInverseSolution=false;
	fmat::Column<3> pTgtV = pEffBase + de;
	
	/*std::cout << "Q prior " << links[0]->getQ();
	if(links[1])
		cout << ' ' << links[1]->getQ();
	if(links[2])
		cout << ' ' << links[2]->getQ();
	cout << std::endl;*/
	if(links[1]!=NULL) {
		if(links[2]!=NULL)
			computeThirdLink(*links[2], pTgtV, j, pEff, invertThird, converges[2]);
		computeSecondLink(*links[1], pTgtV, j, pEff, false, converges[1]);
	}
	computeFirstLink(*links[0], pTgtV, j, pEff, converges[0]);
	
	if(hasInverseSolution && (!converges[0] || !converges[1])) {
		// we hit a joint limit, try inverse solution?
		//cout << "Hit limit, trying inverse " << inverseKnee << endl;
		float defaultKnee = links[2]->getQ();
		links[2]->tryQ(inverseKnee);
		if(links[1]!=NULL)
			computeSecondLink(*links[1], pTgtV, j, pEff, false, converges[1]);
		computeFirstLink(*links[0], pTgtV, j, pEff, converges[0]);

		/*std::cout << "Inverse Q post " << links[0]->getQ();
		if(links[1])
			cout << ' ' << links[1]->getQ();
		if(links[2])
			cout << ' ' << links[2]->getQ();
		cout << std::endl;*/

		if((!converges[0] || !converges[1]) && std::abs(origKnee-inverseKnee) > std::abs(origKnee-defaultKnee)) {
			// no luck, go back to default if it was closer to current position
			//cout << "Still hit limit, going back" << endl;
			links[2]->tryQ(defaultKnee);
			if(links[1]!=NULL)
				computeSecondLink(*links[1], pTgtV, j, pEff, false, converges[1]);
			computeFirstLink(*links[0], pTgtV, j, pEff, converges[0]);
		}
	}
	
	/*std::cout << "Q post " << links[0]->getQ();
	if(links[1])
		cout << ' ' << links[1]->getQ();
	if(links[2])
		cout << ' ' << links[2]->getQ();
	cout << std::endl;*/
	
	//check if root link is maxed out
	if(!converges[0]) {
		//redo child links since their parent was limited, first child is now root
		if(links[1]!=NULL) {
			hasInverseSolution=false;
			if(links[2]!=NULL) {
				computeSecondLink(*links[2], pTgtV, j, pEff, invertThird, converges[2]);
			}
			computeFirstLink(*links[1], pTgtV, j, pEff, converges[1]);
			if(hasInverseSolution && !converges[1]) {
				float defaultKnee = links[2]->getQ();
				links[2]->tryQ(inverseKnee);
				computeFirstLink(*links[1], pTgtV, j, pEff, converges[1]);
				if(!converges[1] && std::abs(origKnee-inverseKnee) > std::abs(origKnee-defaultKnee)) {
					// no luck, go back to default if it was closer to current position
					links[2]->tryQ(defaultKnee);
					computeFirstLink(*links[1], pTgtV, j, pEff, converges[1]);
				}
			}
		}
	}
	
	//check again, maybe now middle link is limited
	if(!converges[1] && links[2]!=NULL) {
		//redo last link
		computeFirstLink(*links[2], pTgtV, j, pEff, converges[2]);
	}
	
	de = j.getFullT()*pEff - pTgtV;
	return de.norm()<=EPSILON;
}

IKSolver::StepResult_t IKThreeLink::step(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, const Position& pTgt, float pDist, float posPri, const Orientation& oriTgt, float oriDist, float oriPri) const {
	// find the closest target point
	fmat::Transform Tj = j.getFullT();
	fmat::Column<3> pEffBase(Tj*pEff);
	fmat::Quaternion qj = j.getWorldQuaternion(); //cout << "WorldQ: " << qj;
	Rotation oEffBase = qj*oriEff;
	Point de;
	pTgt.computeErrorGradient(pEffBase,oEffBase,de);
	
	StepResult_t res=SUCCESS;
	if(de.norm()>pDist) {
		de*=pDist/de.norm();
		res = PROGRESS;
	}
	Point pTgtP = pEffBase+de;
	if(solve(pEff,oriEff,j,pTgtP,posPri,oriTgt,oriPri))
		return res;
	return LIMITS;
}

unsigned int IKThreeLink::setLinks(KinematicJoint& eff, KinematicJoint* links[], unsigned int remain) {
	if(remain==0)
		return 0;
	if(eff.isMobile())
		--remain;
	unsigned int unused=remain;
	if(eff.getParent()!=NULL)
		unused=setLinks(*eff.getParent(),links,remain);
	if(eff.isMobile()) {
		links[remain-unused]=&eff;
		//cout << "Link " << remain-unused << " " << eff.outputOffset << " range " << eff.qmin << " to " << eff.qmax << endl;
	}
	return unused;
}

void IKThreeLink::computeFirstLinkRevolute(KinematicJoint& curlink, const fmat::Column<3>& Pobj, KinematicJoint& endlink, const fmat::Column<3>& Plink, bool& valid) const {
	// transform from world to joint origin (not applying link tranform, so as if q=0)
	fmat::Transform tr = (curlink.getParent()==NULL) ? curlink.getTo().inverse() : (curlink.getParent()->getFullT() * curlink.getTo()).inverse();
	fmat::Column<3> cObj = tr*Pobj; // so objective in 'native' reference frame
	fmat::Column<3> cEff = endlink.getT(curlink)*Plink; // effector relative to link transform
	
	//cout << "LINK1 " << cObj << ' ' << cEff << endl;
		
	if(std::abs(cEff[0])<EPSILON && std::abs(cEff[1])<EPSILON) {
		//Plink is along axis of rotation - nothing we do is going to move it, so don't move at all
		valid=true; //debatable
		return;
	}
	if(std::abs(cObj[0])<EPSILON && std::abs(cObj[1])<EPSILON) {
		//objective point is along axis of rotation - current location is as good as any, so don't move at all
		valid=true; //debatable
		return;
	}
	float ao=atan2(cObj[1],cObj[0]); // angle of objective
	float ae=atan2(cEff[1],cEff[0]); // offset caused by effector
	valid=curlink.tryQ(normalize_angle(ao-ae-curlink.qOffset));
}

void IKThreeLink::computeFirstLinkPrismatic(KinematicJoint& curlink, const fmat::Column<3>& Pobj, KinematicJoint& endlink, const fmat::Column<3>& Plink, bool& valid) const {
	// transform from world to joint origin (not applying link tranform, so as if q=0)
	fmat::Transform tr = (curlink.getParent()==NULL) ? curlink.getTo().inverse() : (curlink.getParent()->getFullT() * curlink.getTo()).inverse();
	fmat::Column<3> cObj = tr*Pobj; // so objective in 'native' reference frame
	//cout << "LINK1 " << cObj << ' ' << endl;
	valid=curlink.tryQ(cObj[2] - curlink.qOffset);
}

/*! assumes that z axis of parent frame and current frame are either parallel or orthogonal
 *  
 *  todo: would be nice if this could handle intermediary angles... currently anything not parallel is assumed to be orthogonal */
void IKThreeLink::computeSecondLinkRevolute(KinematicJoint& curlink, const fmat::Column<3>& Pobj, KinematicJoint& endlink, const fmat::Column<3>& Plink, bool invert, bool& valid) const {
	// link point in current frame
	fmat::Column<3> cEff = endlink.getT(curlink)*Plink;
	if(std::abs(cEff[0])<EPSILON && std::abs(cEff[1])<EPSILON) {
		//Plink is along axis of rotation - nothing we do is going to move it, so don't move at all
		valid=true; //debatable
		return;
	}
	
	//cout << "LINK2 " << cEff << endl;
	
	// object point in parent frame
	fmat::Column<3> pObj = curlink.getParent()->getFullInvT()*Pobj;
	
	if(std::abs(curlink.getRotation()(2,2)) >= (1-EPSILON)) {
		// z axis parallel
		
		// law of cosines
		//fmat::Column<3> pCur = curlink.getPosition();
		float adj1 = curlink.r;
		float adj2 = hypotf(cEff[0],cEff[1]);
		float opp = hypotf(pObj[0],pObj[1]);
		float a;
		if(adj1 + opp <= adj2 || adj2 + opp <= adj1) {
			// objective too close in -- fold up
			//cout << "too close!" << endl;
			a = 0;
		} else if(adj1 + adj2 <= opp) {
			// objective too far away -- extend out
			//cout << "too far!" << endl;
			a = (float)M_PI;
		} else {
			float ca = ( adj1*adj1 + adj2*adj2 - opp*opp ) / ( 2*adj1*adj2);
			//cout << "Law of cosines: " << adj1 << ' ' << adj2 << ' ' << opp << " = " << ca << endl;
			a = std::acos(ca);
		}
		
		float offset = -curlink.qOffset - std::atan2(cEff[1],cEff[0]);
		if(curlink.r>0)
			offset+=(float)M_PI;
		
		// solutions are offset±a
		float q1,q2;
		if(invert) {
			q1 = offset-a;
			q2 = offset+a;
		} else {
			q1 = offset+a;
			q2 = offset-a;
		}
		
		//std::cout << "angle " << a << " (" << a/M_PI*180 << "°) offset " << offset << " q1 " << q1 << " (" << q1/M_PI*180 << "°)" << " q2 " << q2 << " (" << q2/M_PI*180 << "°)" << std::endl;
		valid = curlink.tryQ( normalize_angle(q1) );
		if(!valid) { // try other solution
			//cout << "trying inverse" << endl;
			valid = curlink.tryQ( normalize_angle(q2) );
			if(!valid) { // go back
				//cout << "reverting inverse" << endl;
				valid = curlink.tryQ( normalize_angle(q1) );
			}
		} else if(std::abs(q1-q2)>EPSILON && curlink.qmin<=q2 && q2<=curlink.qmax) {
			//cout << "has inverse" << endl;
			hasInverseSolution=true;
			inverseKnee=q2;
		}
		
	} else {
		// Assume orthogonal curlink z and parent z
		// Need to solve for q: (cr + coh/tan(q))² + lz² = por²
		// Simplifies to quadratic: x² + 2·cr·x + cr² + lz² - por² = 0, where x=coh/tan(q)
		
		float cr = std::abs((float)curlink.r); // radius from parent z aka hip length
		//float lr = hypotf(cEff[0],cEff[1]); // radius of end link aka effector about current z
		float lz = std::abs(cEff[2]); // effector offset along current z
		float coh = pObj[2] - curlink.d; // height of objective from plane of current link about parent z
		if(curlink.alpha<0)
			coh=-coh; // sign depends on direction of parent z vs. curlink
		float por = hypotf(pObj[0],pObj[1]); // radius of objective from parent z
		//cout << "cr " << cr << " lz " << lz << " coh " << coh << " por " << por << endl;
		
		float x;
		if(por<lz) {
			// no solution — effector is too far to the side (b*b - 4*c) < 0
			//cout << "no sol" << endl;
			// stay at current position?
			valid=false; //debatable
			return;
		} else if(false /*por < cr*/) {
			// above the hip, choose "inner" solution
			//cout << "inner" << endl;
			//float a = 1
			float b = 2*cr;
			float c = cr*cr + lz*lz - por*por;
			x = ( -b + std::sqrt(b*b - 4*c) ) / 2;
			//cout << "b " << b << " c " << c << " x " << x << endl;
		} else {
			// outside the hip, choose "outer" solution
			//cout << "outer" << endl;
			//float a = 1
			float b = 2*cr;
			float c = cr*cr + lz*lz - por*por;
			x = ( -b + std::sqrt(b*b - 4*c) ) / 2;
			//cout << "b " << b << " c " << c << " x " << x << endl;
		}
		//if(cEff[0]<0)
		//	x=-x;
		
		// x = coh / tan(q), solve for q:
		float q = std::atan2(coh, x);
		//cout << "pre offset q " << q << " offset " << -std::atan2(cEff[1],cEff[0]) << endl;
		q -= curlink.qOffset + std::atan2(cEff[1],cEff[0]); // cancel offsets from effector point and qOffset
		valid = curlink.tryQ( normalize_angle(q) );
	}
}

/*! First link will perform some rotation, this function needs to set
 *  the distance to match the projection of the objective into the
 *  plane of rotation.  Very similar to computeThirdLinkPrismatic()
 *  except how this projection is done (no ring of motion) */
void IKThreeLink::computeSecondLinkPrismatic(KinematicJoint& curlink, const fmat::Column<3>& Pobj, KinematicJoint& endlink, const fmat::Column<3>& Plink, bool invert, bool& valid) const {
	KinematicJoint* tilt = curlink.getParent();
	while(!tilt->isMobile())
		tilt=tilt->getParent();
	
	// object point in pan's frame
	fmat::Column<3> tObj = tilt->getFullInvT()*Pobj;
	//cout << "tObj " << tObj << endl;
	
	fmat::Transform curToTilt = curlink.getParent()->getT(*tilt) * curlink.getTo();
	
	const fmat::Column<3>& neck = curToTilt.translation();
	float neckD2 = neck.sumSq();
	
	const fmat::Column<3>& tcz = curToTilt.column(2);
	float inner = fmat::dotProduct(tcz,-neck); // a·b·cos(C), |tcz|=1 so actually neckD·cos(C) 
	//cout << "neck " << std::sqrt(neckD2) << " inner " << inner << endl;
	
	float tod2 = tObj[0]*tObj[0] + tObj[1]*tObj[1];
	//cout << "tod " << std::sqrt(tod2) << endl;
	
	valid = curlink.tryQ(computePrismaticQ(tod2,neckD2,inner) - curlink.qOffset);
}


/*! We'll compute the knee angle first, using:
 *  - length of the thigh ('thigh')
 *  - distance from knee (curlink) to Plink, projected to plane of curlink's rotation (cEffD)
 *  - distance from shoulder (previous link) to Pobj (ringObjD)
 *
 *  Two knee (curlink) configurations are supported: parallel to parLink (where parLink.d can be bundled as a z offset of the effector)
 *  or parallel to grandparent, with parLink.r==0.
 *  In the former case, the parent link's origin forms a ring about the grandparent's
 *  link of radius parLink.r.  It is the distance of the objective to the ring (or just the center point
 *  when parLink.r==0) that curlink must match the effector's distance.
 */
void IKThreeLink::computeThirdLinkRevolute(KinematicJoint& curlink, const fmat::Column<3>& Pobj, KinematicJoint& endlink, const fmat::Column<3>& Plink, bool invert, bool& valid) const {
	// link point in current frame
	fmat::Column<3> cEff = endlink.getT(curlink)*Plink;
	if(std::abs(cEff[0])<EPSILON && std::abs(cEff[1])<EPSILON) {
		//Plink is along axis of rotation - nothing we do is going to move it, so don't move at all
		valid=true; //debatable
		return;
	}
	//cout << curlink.outputOffset << " cEff " << cEff << " dist " << cEff.norm() << " (in plane " << fmat::SubVector<2>(cEff).norm() << ")" << endl;
	
	KinematicJoint& parLink = *curlink.getParent();
	//std::cout << "par to cur dist " << curlink.getPosition().norm() << ' ' << curlink.getPosition() << std::endl;
	KinematicJoint& gparLink = *parLink.getParent();
	//std::cout << "check ring r " << parLink.r << ' ' << (parLink.getPosition() - fmat::pack(0,0,parLink.d)).norm() << std::endl;
	
	// object point in grandparent's frame
	fmat::Column<3> gpObj = gparLink.getFullInvT()*Pobj;
	gpObj[2]-=parLink.d; // subtract parLink height so now relative to plane of ring of motion
	//cout << "gpObj " << gpObj << endl;
	
	float thigh, thigh2; // distance from parent to current, projected to curlink's plane of rotation
	float ringObjD2; // distance from parent's ring of motion to objective, projected to curlink's plane of rotation
	
	// check condition α = ±90° (e.g. Aibo legs, knee is perpendicular to elevator)
	const bool KNEE_PERP = std::abs(std::abs((float)curlink.alpha) - (float)M_PI_2) < EPSILON;
	if(KNEE_PERP) {
		// math gets wonky if parLink.r≠0, effector's z component non-planar to objective radial, don't know how to recover
		ASSERT(parLink.r==0,"IKThreeLink can't handle perpendicular knees with non-zero hip radius");
		
		thigh2 = curlink.r*curlink.r + curlink.d*curlink.d;
		thigh = std::sqrt(thigh2);
		
		float lz = cEff[2]; // distance of link point from knee rotation plane
		float rd = std::sqrt(gpObj[0]*gpObj[0] + gpObj[1]*gpObj[1]) - std::abs((float)parLink.r); // distance from ring of motion in grandparent xy
		ringObjD2 = rd*rd + gpObj[2]*gpObj[2] - lz*lz;
		if(ringObjD2<0)
			ringObjD2=0;
		
	} else { // assume Chiara legs, knee is parallel to elevator
		//cout << "PARALLEL" << endl;
		thigh = std::abs((float)curlink.r);
		thigh2=thigh*thigh;
		
		float lz = cEff[2] - curlink.d; // distance of link point from knee rotation plane
		
		// now account for curlink.d and any pl.z, need to project objective distance onto curlink plane of rotation
		// grandparent origin, curlink origin, and objective form right triangle.  curlink-objective is parallel to grandparent xy
		float d2 = gpObj[0]*gpObj[0] + gpObj[1]*gpObj[1] - lz*lz;
		if(d2<0) 
			d2=0;
		float rd = std::sqrt(d2) - std::abs((float)parLink.r); // distance from ring of motion in grandparent xy
		//cout << "d2 " << d2 << " lz " << lz << " rd " << rd << endl;
		ringObjD2 = rd*rd + gpObj[2]*gpObj[2];

		/* Singularity issue: current implementation cannot reach points "behind" (x<0) the hip because we are
		 setting oringd to be the closest point on the *ring*, but with hip rotation limits we actually have an *arc*.
		 However this limitation prevents a singularity as the target point passes behind the hip, we would have to
		 instantly snap 180° to the opposite side of the arc to continue tracking.  Instead what we do here is
		 to allow the hip to hit its joint limit and just stay there on the same side, even though technically the
		 point is within reach.  We could do something like add a flag to allow this solution in case the instantanious
		 motion is acceptable. */
	}
	float ringObjD = std::sqrt(ringObjD2);
	
	// distance from curlink to pLink projected to curlink's xy plane:
	float ln2 = cEff[0]*cEff[0] + cEff[1]*cEff[1];
	float ln = std::sqrt(ln2);
	
	//cout << "thigh " << thigh << " oringd " << ringObjD << " link " << ln << endl;
	
	// test for reachability otherwise angle will be NaN
	float a;
	if( (ln + ringObjD <= thigh) || (ringObjD + thigh <= ln)) {
		//cout << "out of reach, interior" << endl;
		a = 0;
		
	} else if(ln + thigh <= ringObjD) {
		//cout << "out of reach, exterior" << endl;
		a = (float)M_PI;
		
	} else {
		// law of cosines to find angle between parent and solution point
		a = std::acos( (ln2 + thigh2 - ringObjD2) / (2*ln*thigh) );
	}
	float offset = -curlink.qOffset - std::atan2(cEff[1],cEff[0]);
	
	if(KNEE_PERP) {
		if(curlink.alpha<0)
			offset -= std::atan2((float)curlink.d,(float)curlink.r);
		else
			offset += std::atan2((float)curlink.d,(float)curlink.r);
		a = (float)M_PI-a;
	} else if(curlink.r>0) {
		a = (float)M_PI-a;
	}

	// solutions are offset ± a
	float q1,q2;
	if(invert) {
		q1 = offset-a;
		q2 = offset+a;
	} else {
		q1 = offset+a;
		q2 = offset-a;
	}
	//std::cout << "angle " << a << " (" << a/M_PI*180 << "°) offset " << offset << " q1 " << q1 << " (" << q1/M_PI*180 << "°) q2 " << q2 << " (" << q2/M_PI*180 << ")" << std::endl;
	
	valid = curlink.tryQ( normalize_angle(q1) );
	if(!valid) { // try other solution
		//cout << "trying inverse" << endl;
		valid = curlink.tryQ( normalize_angle(q2) );
		if(!valid) { // go back
			//cout << "reverting inverse" << endl;
			valid = curlink.tryQ( normalize_angle(q1) );
		}
	} else if(std::abs(q1-q2)>EPSILON && curlink.qmin<=q2 && q2<=curlink.qmax) {
		//cout << "has inverse" << endl;
		hasInverseSolution=true;
		inverseKnee=q2;
	}
}

/*! This version is intended mainly for solving to have a pan-tilt camera look at a point in space.
 *  The camera is the third link, and should be marked as prismatic.  There can be immobile
 *  frames between this and the tilt joint, as we often want a specific configuration of x and y axes
 *  to match image coordinates.  Z must point out of the camera, toward the scene.
 *
 *  @a Plink is ignored.  This only solves for the z axis, and that must intersect the pan axis.
 *  (The math gets really hairy if the camera is not aligned to a radial from the pan joint.)
 *  
 *  The approach is similar to computeThirdLinkRevolute(), using
 *  - length of the neck, i.e. distance from tilt to camera
 *  - angle at the camera (curlink) between neck link and z axis
 *  - distance from tilt's ring of motion to Pobj (ringObjD)
 */
void IKThreeLink::computeThirdLinkPrismatic(KinematicJoint& curlink, const fmat::Column<3>& Pobj, KinematicJoint& endlink, const fmat::Column<3>&, bool invert, bool& valid) const {
	//cout << "PRISMATIC" << endl;
	
	KinematicJoint* tilt = curlink.getParent();
	while(!tilt->isMobile())
		tilt=tilt->getParent();
	KinematicJoint* pan = tilt->getParent();
	
	// object point in pan's frame
	fmat::Column<3> gpObj = pan->getFullInvT()*Pobj;
	gpObj[2]-=tilt->d; // subtract tilt height so now relative to plane of ring of motion
	//cout << "gpObj " << gpObj << endl;
	
	fmat::Transform curToTilt = curlink.getParent()->getT(*tilt) * curlink.getTo();
	
	const fmat::Column<3>& neck = curToTilt.translation();
	float neckD2 = neck.sumSq();
	
	const fmat::Column<3>& tcz = curToTilt.column(2);
	float inner = fmat::dotProduct(tcz,-neck); // a·b·cos(C), |tcz|=1 so actually neckD·cos(C) 
	//cout << "neck " << std::sqrt(neckD2) << " inner " << inner << endl;
	
	float d2 = gpObj[0]*gpObj[0] + gpObj[1]*gpObj[1];
	float rd = std::sqrt(d2) - std::abs((float)tilt->r); // distance from ring of motion in grandparent xy
	float ringObjD2 = rd*rd + gpObj[2]*gpObj[2];
	//cout << "d " << std::sqrt(d2) << " rd " << rd << " ringObjD " << std::sqrt(ringObjD2) << endl;
	
	valid = curlink.tryQ(computePrismaticQ(ringObjD2,neckD2,inner) - curlink.qOffset);
}

fmat::fmatReal IKThreeLink::computePrismaticQ(fmat::fmatReal objD2, fmat::fmatReal neckD2, fmat::fmatReal inner) {
	// law of cosines: c² = a² + b² - 2·a·b·cos(C)
	// where c = ringObjD, a = neckD, a·b·cos(C) = inner, solve for b:
	// b² - 2·a·b·cos(C) + a² - c² = 0
	// apply quadratic formula:
	// a'x² + b'x + c' = 0, x = (-b' ± √(b'² - 4·a'·c'))/(2a')
	// where a' = 1, b' = -2·neckD·cos(C), c' = neckD2 - ringObjD2, x=b
	float b = -2*inner;
	float c = neckD2 - objD2;
	float r = b*b - 4*c;
	if(r<0) {
		//cout << "NO SOLUTION" << endl;
		// closest solution is at right angle to objD extension, aka inner
		float q2 = neckD2 - inner*inner;
		if(q2<0) {
			// should only happen due to numerical roundoff, if ever
			cerr << "IKThreeLink::computePrismaticQ numeric error, q² < 0 : " << q2 << " from " << neckD2 << ' ' << inner << ' ' << objD2 << endl;
			return 0;
		} else {
			return std::sqrt(q2);
		}
	} else {
		return (-b + std::sqrt(r))/2; // only want largest/positive solution
	}
}



/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
