//-*-c++-*-
#ifndef INCLUDED_GaitedFootsteps_h_
#define INCLUDED_GaitedFootsteps_h_

/*
 analyze gait
	footstep limits
	group legs
 save q for each leg, link within group
 expand for each candidate
 	foot contact, body motion
 	body motion implied by contact chosen?
 	contacts filtered by reachability
 if a leg has obstacles:
	select from other groups (make up new ones?)
	test stability
	increment cost of motion?
*/

#include "Shared/fmat.h"
#include "Shared/RobotInfo.h"
#include "Motion/XWalkParameters.h"
#include "Planners/PlannerObstacles.h"
#include <tr1/functional>

class KinematicJoint;

class GaitedFootsteps {
public:
	GaitedFootsteps()
		: speed(0), flightDuration(0), stepDuration(), relaxation(1), rotDist(0), stepReorient(true), stepRehab(true),
		kinematics(NULL), ncand(), p(), ground(), gravity(), neutrals(), groups(), obstacles()
	{}
	
	struct State {
		State() : pos(), oriRot(fmat::Matrix<2,2>::identity()), oriAngle(0), phase(0) {}
		fmat::Column<2> pos;
		fmat::Matrix<2,2> oriRot;
		fmat::fmatReal oriAngle;
		fmat::Column<2> footPos[NumLegs];
		float legJoints[NumLegJoints];
		unsigned int phase;
		static fmat::fmatReal ROUND; //!< UGLY HACK - this is assigned in GaitedFootsteps::setGait
		static const fmat::fmatReal ORIROUND;
		size_t hash() const {
			const long x = static_cast<long>(pos[0]*ROUND);
			const long y = static_cast<long>(pos[1]*ROUND);
			const long a = static_cast<long>(oriAngle*ORIROUND);
			// with some trial and error, this distributes very nicely, no deeper understanding...
			return std::tr1::hash<fmat::fmatReal>()((x<<16) + (a<<8) + y);
		}
		/*bool operator<(const State& st) const {
			const float * a = &pos[0];
			const float * b = &st.pos[0];
			for(size_t i=0; i<sizeof(State)/sizeof(float); ++i) {
				if(a[i]<b[i])
					return true;
				if(a[i]>b[i])
					return false;
			}
			return false;
		}*/
		bool operator==(const State& st) const {
			// This causes crashes (at unordered_set destruction, but probably anytime)
			//  because unordered_set requires that equivalent values have equivalent hash:
			//return ((pos-st.pos).norm()<ROUND);
			// Instead, restrict equivalency to the same rounding method used in hash
			const long mx = static_cast<long>(pos[0]*ROUND);
			const long my = static_cast<long>(pos[1]*ROUND);
			const long ma = static_cast<long>(oriAngle*ORIROUND);
			const long stx = static_cast<long>(st.pos[0]*ROUND);
			const long sty = static_cast<long>(st.pos[1]*ROUND);
			const long sta = static_cast<long>(st.oriAngle*ORIROUND);
			return mx==stx && my==sty && ma==sta;
		}
		friend std::ostream& operator<<(std::ostream& os, const State& st) {
			return os << &st << ": " << st.pos << " @ "  << st.oriAngle << '\n'
			<< fmat::SubMatrix<2,NumLegs,const float>(&st.footPos[0][0]) << std::endl;
		}
	};
	
	~GaitedFootsteps();
	
	//! returns true if the state does not collide with any obstacles
	bool validate(const State& st) const;
	
	//! The heuristic function accepts the current state, and goal state, and should return an admissable (aka optimistic) estimate of the remaining cost
	float heuristic(const State& st, const State& goal) const;
	
	//! Generates a vector of successor states, paired with the cost to get to that state from @a st
	/*! Note that this implementation returns a reference to a static instance, which is not thread safe but slightly faster */
	const std::vector<std::pair<float,GaitedFootsteps::State> >& expand(const State* parent, const State& st, const State& goal) const;
	
	void setGait(const KinematicJoint& kj, const XWalkParameters& xp, size_t discretization);
	
	void addObstacle(PlannerObstacle2D* obs) { obstacles.push_back(obs); }
	std::vector< PlannerObstacle2D* >& getObstacles() { return obstacles; }
	const std::vector< PlannerObstacle2D* >& getObstacles() const { return obstacles; }
	
	float speed;
	float flightDuration;
	float stepDuration;
	float relaxation;
	float rotDist;
	
	bool stepReorient;
	bool stepRehab;
	
	KinematicJoint * kinematics;
	KinematicJoint* childMap[NumReferenceFrames];
	struct StepData {
		StepData() : pos() {}
		fmat::Column<3> pos;
        #if defined(TGT_IS_MANTIS)
        static const int JointsPerLeg = 4; // hack! fix it later
        #endif
		float legJoints[JointsPerLeg];
//                float legJoints[4];
	};
	size_t ncand;
	XWalkParameters p;
	fmat::Column<3> ground;
	fmat::Column<3> gravity;
	std::vector< fmat::Column<2> > neutrals;
	std::vector< std::set<size_t> > groups;
	std::vector< PlannerObstacle2D* > obstacles;
	
	PlannerObstacle2D* checkObstacles(const fmat::Column<2>& pt) const;
	bool addRotation(const State& st, float strideAngle, float strideDist, const fmat::Column<2>& strideDir, const fmat::Column<2>& stride, std::vector<std::pair<float,GaitedFootsteps::State> >& candidates, float maxDist) const;
	void addCandidate(const State* parent, const State& st, float angle, std::vector<std::pair<float,GaitedFootsteps::State> >& candidates, float maxDist=0) const;
	
private:
	GaitedFootsteps(const GaitedFootsteps&); //!< do not use
	GaitedFootsteps& operator=(const GaitedFootsteps&); //!< do not use
};

#endif
