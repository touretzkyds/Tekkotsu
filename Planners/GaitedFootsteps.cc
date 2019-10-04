#include "Shared/RobotInfo.h"
// only makes sense for legged, but also aperios compiler lacks tr1/functional_hash.h
#if defined(TGT_HAS_LEGS) && !defined(TGT_IS_AIBO) && (!defined(__GNUC__) || __GNUC__>=4)

#include "GaitedFootsteps.h"
#include "Motion/KinematicJoint.h"
#include "Motion/IKSolver.h"
#include <algorithm>
#include <iterator>

fmat::fmatReal GaitedFootsteps::State::ROUND=1/6.0;
const fmat::fmatReal GaitedFootsteps::State::ORIROUND=30/M_PI;

GaitedFootsteps::~GaitedFootsteps() {
	delete kinematics;
	kinematics=NULL;
	for(std::vector<PlannerObstacle2D*>::const_iterator it=obstacles.begin(); it!=obstacles.end(); ++it)
		delete *it;
	obstacles.clear();
}

float GaitedFootsteps::heuristic(const State& st, const State& goal) const {
	if(speed>0)
		return (st.pos - goal.pos).norm() / speed * relaxation;
	else
		return (st.pos - goal.pos).norm() / p.strideLenX * flightDuration * relaxation;
}

bool GaitedFootsteps::validate(const State& st) const {
	return true;
	/*const std::set<size_t>& group = (st.phase==0) ? groups.back() : groups[st.phase-1];
	for(std::set<size_t>::const_iterator lit=group.begin(); lit!=group.end(); ++lit) {
		for(std::vector< PlannerObstacle* >::const_iterator oit=obstacles.begin(); oit!=obstacles.end(); ++oit) {
			if((*oit)->collides(st.footPos[*lit])) {
				return false;
			}
		}
	}
	return true;*/
}

//! Generates a vector of successor states, paired with the cost to get to that state from @a st
/*! Note that this implementation returns a reference to a static instance, which is not thread safe but slightly faster.
 *
 * for each leg which can be lifted:
 *     expand each step position for that leg
 *     body motion: stance motion before lift and during flight?
 */
const std::vector<std::pair<float,GaitedFootsteps::State> >& GaitedFootsteps::expand(const State* parent, const State& st, const State& goal) const {
	//std::cout << "Expanding " << st << std::endl;
	static std::vector<std::pair<float,GaitedFootsteps::State> > ans;
	ans.clear();
	
	// check stability of support feet...
	std::vector<fmat::Column<2> > support;
	support.reserve(NumLegs - groups[st.phase].size());
	fmat::Column<3> gravOriAxis = fmat::crossProduct(fmat::UNIT3_Z,gravity);
	fmat::Quaternion gravOri = fmat::Quaternion::fromAxisAngle(gravOriAxis,std::asin(gravOriAxis.norm()));
	for(size_t i=0; i<NumLegs; ++i) {
		if(groups[st.phase].count(i) == 0) {
			fmat::Column<3> lpos3 = fmat::pack( st.oriRot.transpose()*(st.footPos[i]-st.pos), 0);
			support.push_back(fmat::SubVector<2,const fmat::fmatReal>(gravOri * lpos3));
			// also check feet kinematics... might be unnecessary
			p.projectToGround(ground, p.groundPlane[3], gravity, lpos3);
			IKSolver& solver = childMap[FootFrameOffset + i]->getIK();
			bool success = solver.solve(fmat::Column<3>(),*childMap[FootFrameOffset + i],IKSolver::Point(lpos3));
			if(!success) // foot has stretched out of reach
				return ans;
		}
	}
	ConvexPolyObstacle supportPoly;
	supportPoly.hull(std::set<fmat::Column<2> >(support.begin(),support.end()));
	fmat::Column<3> com; float totalMass;
	kinematics->sumCenterOfMass(com,totalMass); // not accounting for motion of flight legs on center of mass (com)!
	if(!supportPoly.collides(fmat::SubVector<2,const fmat::fmatReal>(gravOri * com)))
		return ans;

	const fmat::Column<2> gd = goal.pos - st.pos;
	const float goalDir = stepReorient ? std::atan2(gd[1],gd[0])-st.oriAngle : 0;
	addCandidate(parent, st, goalDir, ans, gd.norm());
	addCandidate(parent, st, goalDir+(float)M_PI, ans);
	for(size_t i=1; i<ncand/2; ++i) {
		float per = i/float(ncand/2);
		//per*=per; // weights steps "forward" (but commented-out because too tight)
		addCandidate(parent, st, goalDir + static_cast<float>(M_PI)*per, ans);
		addCandidate(parent, st, goalDir - static_cast<float>(M_PI)*per, ans);
	}
	
	/*if(groups.size()>2) {
		// if more than two groups, consider a no-op to go out of phase
		// this relies on checking leg kinematics however, to ensure a
		// leg doesn't get ignored and dragged along
		ans.push_back(std::make_pair(0,st));
		State& nxt = ans.back().second;
		if(++nxt.phase >= groups.size())
			nxt.phase=0;
	}*/
	
	/*for(size_t i=0; i<ans.size(); ++i) {
		std::cout << ans[i].first << ' ' << ans[i].second << std::endl;
	}*/

	return ans;
}

PlannerObstacle2D* GaitedFootsteps::checkObstacles(const fmat::Column<2>& pt) const {
	for(std::vector< PlannerObstacle2D* >::const_iterator oit=obstacles.begin(); oit!=obstacles.end(); ++oit) {
		//cout << ' ' << (*oit)->collides(lpos2);
		if((*oit)->collides(pt)) {
			//cout << endl;
			return *oit;
		}
	}
	return NULL;
}

bool GaitedFootsteps::addRotation(const State& st, float strideAngle, float strideDist, const fmat::Column<2>& strideDir, const fmat::Column<2>& stride, std::vector<std::pair<float,GaitedFootsteps::State> >& candidates, float maxDist) const {
	float rotAngle = 2 * std::atan2(strideDist/2,rotDist); // heuristic to limit rotation speed
	if(strideDir[0]>0) {
		//forward motion
		if(strideAngle>0) {
			//walking in positive rotation
			rotAngle = std::min(strideAngle,rotAngle); // don't over-rotate
		} else {
			// walking in negative rotation
			rotAngle = -std::min(-strideAngle,rotAngle); // don't over-rotate
		}
	} else {
		if(strideAngle<0)
			rotAngle=-rotAngle;
	}
	fmat::Matrix<2,2> rot = fmat::rotation2D(st.oriAngle+rotAngle);
	
	std::vector< std::pair<size_t,fmat::Column<2> > > footPos;
	footPos.reserve(groups[st.phase].size());
	for(std::set<size_t>::const_iterator lit=groups[st.phase].begin(); lit!=groups[st.phase].end(); ++lit) {
		// lpos3 is egocentric 3D coordinates
		fmat::Column<2> lpos2 = st.pos + rot*(neutrals[*lit] + stride);
		//cout << "Leg " << *lit << " at " << lpos2;
		PlannerObstacle2D* obs = checkObstacles(lpos2);
		if(obs!=NULL) {
			if(!stepRehab)
				return false;
			
			fmat::Column<2> v = obs->gradient(lpos2);
			lpos2 += v + v*(0.25f/v.norm()); // add a small margin to avoid numerical instability on obstacle boundary
			// convert new lpos2 back to egocentric lpos3
			fmat::Column<3> lpos3 = fmat::pack( rot.transpose()*(lpos2-st.pos), 0);
			
			// now check IK is still valid (or update lpos2 with reached position?)
			//bool success=true;
			p.projectToGround(ground, p.groundPlane[3], gravity, lpos3);
			IKSolver& solver = childMap[FootFrameOffset + *lit]->getIK();
			bool success = solver.solve(fmat::Column<3>(),*childMap[FootFrameOffset + *lit],IKSolver::Point(lpos3));
			/*for(unsigned int j=0; j<JointsPerLeg; ++j) {
				KinematicJoint * ckj = childMap[LegOffset + JointsPerLeg*leg + j];
				if(ckj!=NULL)
					sd.legJoints[j] = ckj->getQ();
			}*/
			// retrieve achieved position in case gait sends leg out of reach
			//lpos3 = childMap[FootFrameOffset+leg]->getWorldPosition();
			
			if(!success)
				return false;
			
			// if still colliding, give up
			if(checkObstacles(lpos2)!=NULL)
				return false;
		}
		footPos.push_back(std::make_pair(*lit,lpos2));
		//cout << endl;
	}
	
	float t = strideDist / groups.size(); // distance body travels before next step
	if(maxDist>0)
		t = std::min(maxDist,t);
	
	candidates.push_back(std::make_pair(speed>0?t/speed:flightDuration,st));
	State& nxt = candidates.back().second;
	if(++nxt.phase >= groups.size())
		nxt.phase=0;
	for(size_t i=0; i<footPos.size(); ++i)
		nxt.footPos[footPos[i].first] = footPos[i].second;
	nxt.oriAngle += rotAngle / groups.size();
	nxt.oriRot = fmat::rotation2D(nxt.oriAngle);
	
	nxt.pos += st.oriRot*strideDir*t;
	return true;
}

void GaitedFootsteps::addCandidate(const State* parent, const State& st, float angle, std::vector<std::pair<float,GaitedFootsteps::State> >& candidates, float maxDist/*=0*/) const {
	const fmat::Column<2> dir = fmat::pack(std::cos(angle),std::sin(angle));
	if(parent!=NULL && fmat::dotProduct(dir,st.pos-parent->pos)<-0.5)
		return; // don't consider actions which reverse previous motion
	const fmat::Column<2> d = fmat::pack(p.strideLenX*dir[1], p.strideLenY*dir[0]);
	const float strideDist = p.strideLenX*p.strideLenY / d.norm(); // distance of this step
	
	const fmat::Column<2> stride = dir*strideDist;
	addRotation(st,angle,strideDist,dir,stride,candidates,maxDist);
	//if(addRotation(st,angle,strideDist,dir,stride,candidates,maxDist))
	//	return;
	
	std::vector< std::pair<size_t,fmat::Column<2> > > footPos;
	footPos.reserve(groups[st.phase].size());
	for(std::set<size_t>::const_iterator lit=groups[st.phase].begin(); lit!=groups[st.phase].end(); ++lit) {
		// lpos3 is egocentric 3D coordinates
		fmat::Column<2> lpos2 = st.pos + st.oriRot * (neutrals[*lit] + stride);
		//cout << "Leg " << *lit << " at " << lpos2;
		PlannerObstacle2D* obs = checkObstacles(lpos2);
		if(obs!=NULL) {
			if(!stepRehab)
				return;
			
			fmat::Column<2> v = obs->gradient(lpos2);
			lpos2 += v + v*(0.25f/v.norm()); // add a small margin to avoid numerical instability on obstacle boundary
			// convert new lpos2 back to egocentric lpos3
			fmat::Column<3> lpos3 = fmat::pack(st.oriRot.transpose()*(lpos2-st.pos),0);
			
			// now check IK is still valid (or update lpos2 with reached position?)
			//bool success=true;
			p.projectToGround(ground, p.groundPlane[3], gravity, lpos3);
			IKSolver& solver = childMap[FootFrameOffset + *lit]->getIK();
			bool success = solver.solve(fmat::Column<3>(),*childMap[FootFrameOffset + *lit],IKSolver::Point(lpos3));
			/*for(unsigned int j=0; j<JointsPerLeg; ++j) {
				KinematicJoint * ckj = childMap[LegOffset + JointsPerLeg*leg + j];
				if(ckj!=NULL)
					sd.legJoints[j] = ckj->getQ();
			}*/
			// retrieve achieved position in case gait sends leg out of reach
			//lpos3 = childMap[FootFrameOffset+leg]->getWorldPosition();
			
			if(!success)
				return;
			
			// if still colliding, give up
			if(checkObstacles(lpos2)!=NULL)
				return;
		}
		footPos.push_back(std::make_pair(*lit,lpos2));
		//cout << endl;
	}
	
	float t = strideDist / groups.size(); // distance body travels before next step
	if(maxDist>0)
		t = std::min(maxDist,t);
	
	candidates.push_back(std::make_pair(speed>0?t/speed:flightDuration,st));
	State& nxt = candidates.back().second;
	if(++nxt.phase >= groups.size())
		nxt.phase=0;
	for(size_t i=0; i<footPos.size(); ++i)
		nxt.footPos[footPos[i].first] = footPos[i].second;
	
	nxt.pos += st.oriRot*dir*t;
	
	//std::cout << "\tCandidate " << candidates.back().first << ' ' << nxt.pos << std::endl;
	
	/*for(size_t leg=0; leg<NumLegs; ++leg) {
		StepData sd;
		(fmat::SubVector<2>)sd.pos = neutralPos[leg] + stride;
		XWalkParameters::projectToGround(ground, p.groundPlane[3], gravity, sd.pos);
		IKSolver& solver = childMap[FootFrameOffset+leg]->getIK();
		bool success = solver.solve(fmat::Column<3>(),*childMap[FootFrameOffset+leg],IKSolver::Point(sd.pos));
		for(unsigned int j=0; j<JointsPerLeg; ++j) {
			KinematicJoint * ckj = childMap[LegOffset + JointsPerLeg*leg + j];
			if(ckj!=NULL)
				sd.legJoints[j] = ckj->getQ();
		}
		// retrieve achieved position in case gait sends leg out of reach
		sd.pos = childMap[FootFrameOffset+leg]->getWorldPosition();
		//std::cout << sd.pos[0] <<'\t' << sd.pos[1] << '\t' << sd.pos[2] << std::endl;
		if(!success)
			std::cerr << "WARNING: unable to reach footstep at angle " << ang/M_PI*180 << "° on leg " << leg << std::endl;
		candidates[leg].push_back(sd);
	}*/
}

void GaitedFootsteps::setGait(const KinematicJoint& kj, const XWalkParameters& xp, size_t discretization) {
	std::fill(childMap, childMap+NumReferenceFrames, static_cast<KinematicJoint*>(NULL));
	delete kinematics;
	kinematics = dynamic_cast<KinematicJoint*>(kj.clone());
	kinematics->buildChildMap(childMap,0,NumReferenceFrames);

	ncand = discretization;
	//p = xp; // buggy
	p.setParseTree(xp.stealParseTree());
	p.readParseTree();
	
	p.packGroundGravity(ground,gravity);
	
	// each mid-stride position is base primarily on the location of the first leg joint
	// the strideBias then offsets the x position, and the stance width offsets the y position
	neutrals.resize(NumLegs);
	fmat::Column<2> neutralPos[NumLegs];
	for(unsigned int leg=0; leg<NumLegs; ++leg) {
		XWalkParameters::LegParameters& legParam = p.legParams[leg];
            #if defined(TGT_IS_MANTIS)
            const unsigned int JointsPerLeg = 4; // hack! fix it later
            #endif
        KinematicJoint * legRoot = childMap[LegOffset+JointsPerLeg*leg];
		fmat::Column<3> fstPos = legRoot->getWorldPosition();
		neutralPos[leg][0] = fstPos[0] + legParam.strideBias;
		neutralPos[leg][1] = ((fstPos[1]<0) ? -*legParam.stanceWidth : *legParam.stanceWidth);
		
		neutrals[leg] = neutralPos[leg];
		//std::cout << neutrals[leg] << std::endl;
		//(fmat::SubVector<2>)neutrals[leg] = neutralPos[leg];
		//XWalkParameters::projectToGround(ground, p.groundPlane[3], gravity, neutrals[leg]);
		
		flightDuration = std::max(p.legParams[leg].flightDuration/1000.f, flightDuration);
	}

	// Rotations will be done about a point a "body width" away, yields natural
	// (and efficient) arcing motion instead of stop-and-turn jitters
	// So, have to find a general estimate of how far this is: max of most distant neutral pos
	rotDist=0;
	for(unsigned int leg=0; leg<NumLegs; ++leg) {
		rotDist = std::max(neutrals[leg].norm(), rotDist);
	}
	
	// The footstep is half the stride length in each direction
	// However, feet can have different air times, which decreases their stride
	// since they spend more of the cycle in the air
	// stride[leg] = groundTime[leg] * speed
	// groundTime[leg] = period - airTime[leg]
	// stride[leg] = (period - airTime[leg]) * speed
	
	// maxGroundTime = maxStride / speed
	// period = maxGroundTime + minAirTime
	// period = maxStride / speed + minAirTime
	
	// stride[leg] = (maxStride / speed + minAirTime - airTime[leg]) * speed
	// stride[leg] = maxStride + minAirTime*speed - airTime[leg]*speed
	
	// For now, ignoring this timing issue, assume all feet take equal-length steps
	/*std::vector< std::vector<StepData> > candidates;
	candidates.clear();
	candidates.resize(NumLegs);
	for(size_t i=0; i<discretization; ++i) {
		float ang = static_cast<float>(M_PI)*2*i/discretization;
		fmat::Column<2> dir = fmat::pack(std::cos(ang),std::sin(ang));
		fmat::Column<2> d = fmat::pack(p.strideLenX*dir[1], p.strideLenY*dir[0]);
		float t = p.strideLenX*p.strideLenY / std::sqrt(d[0]*d[0] + d[1]*d[1]); // distance along dir
		fmat::Column<2> stride = dir*t/2; // divide by 2 because stride is diameter, we want radius
		
		for(size_t leg=0; leg<NumLegs; ++leg) {
			StepData sd;
			(fmat::SubVector<2>)sd.pos = neutralPos[leg] + stride;
			XWalkParameters::projectToGround(ground, p.groundPlane[3], gravity, sd.pos);
			IKSolver& solver = childMap[FootFrameOffset+leg]->getIK();
			bool success = solver.solve(fmat::Column<3>(),*childMap[FootFrameOffset+leg],IKSolver::Point(sd.pos));
			for(unsigned int j=0; j<JointsPerLeg; ++j) {
				KinematicJoint * ckj = childMap[LegOffset + JointsPerLeg*leg + j];
				if(ckj!=NULL)
					sd.legJoints[j] = ckj->getQ();
			}
			// retrieve achieved position in case gait sends leg out of reach
			sd.pos = childMap[FootFrameOffset+leg]->getWorldPosition();
			//std::cout << sd.pos[0] <<'\t' << sd.pos[1] << '\t' << sd.pos[2] << std::endl;
			if(!success)
				std::cerr << "WARNING: unable to reach footstep at angle " << ang/M_PI*180 << "° on leg " << leg << std::endl;
			candidates[leg].push_back(sd);
		}
	}*/
	
	groups.clear();
	std::map< float, std::set<size_t> > phasedGroups;
	for(size_t leg=0; leg<NumLegs; ++leg)
		phasedGroups[p.legParams[leg].flightPhase].insert(leg);
	for(std::map< float, std::set<size_t> >::const_iterator it=phasedGroups.begin(); it!=phasedGroups.end(); ++it)
		groups.push_back(it->second);
	
	for(size_t i=0; i<groups.size(); ++i) {
		std::cout << "Group " << i << ": ";
		for(std::set<size_t>::const_iterator git=groups[i].begin(); git!=groups[i].end(); ++git)
			std::cout << "\t" << *git;
		std::cout << std::endl;
	}

	State::ROUND = 1 / (std::min(p.strideLenX,p.strideLenY)/groups.size()/2);
	std::cout << p.strideLenX << ' ' << p.strideLenY << " Rounding factor " << State::ROUND << std::endl;
}

#endif
