#include "Shared/RobotInfo.h"
#if defined(TGT_HAS_LEGS) && !defined(TGT_IS_AIBO)

#include "XWalkMC.h"
#include "Events/LocomotionEvent.h"
#include "Motion/IKGradientSolver.h"
#include "Motion/IKThreeLink.h"
#include "Motion/Kinematics.h"
#include "Shared/newmat/newmatap.h"
#include <cmath>

using namespace std; 

KinematicJoint* XWalkMC::kine=NULL;
KinematicJoint* XWalkMC::childMap[NumReferenceFrames];

XWalkMC::XWalkMC() : MotionCommand(), XWalkParameters(), dirty(false),
										 targetVel(), targetAngVel(0), targetDisp(), targetAngDisp(0),
										 velocityMode(true), displacementTime(0), plantingLegs(false), initialPlacement(true), p(),
										 transitions(), active(), startTime(get_time()), period(0), 
										 globPhase(0), rotationCenter(), contactMsg() {
	if (kine==NULL) {
		kine = new KinematicJoint;
		kine->loadFile(::config->makePath(config->motion.kinematics).c_str());
		kine->buildChildMap(childMap,0,NumReferenceFrames);
	}
	loadFile(::config->motion.makePath("xwalk.plist").c_str());
	p.loadFile(::config->motion.makePath("xwalk.plist").c_str());
	spiderSettings(*this,p);
	if(kine!=NULL && kine->getBranches().size()>0)
		updateNeutralPos((unsigned int)startTime);
	setTargetVelocity(targetVel[0],targetVel[1],targetAngVel);
}

void XWalkMC::setTargetVelocity(float x, float y, float a) {
	// std::cout << (void*)(this) << ":id=" << getID() << "   setTargetVelocity(" << x << ", " << y << ", " << a << ", " << setVelocityMode << ")"
	//  << "   active.size()=" << active.size() << "   startTime=" << startTime << std::endl;
	velocityMode = true;
	if(targetVel[0]==x && targetVel[1]==y && targetAngVel==a)
		return;
	
	unsigned int sysTime = get_time();
	float time = (sysTime-startTime)/1000.f;
	computePhase(time);
	
	float speed=targetVel.norm(), origSpeed=speed;
	bool inAir[NumLegs];
	float phases[NumLegs];
	fmat::Column<3> curPos[NumLegs];
	for(unsigned int leg=0; leg<NumLegs; ++leg) {
		computeLegPhase(leg,inAir[leg],phases[leg]);
		computeCurrentPosition(leg, inAir[leg], speed, phases[leg], curPos[leg]);
	}
	fmat::Column<2> origOff;
	float origOffA=0;
	if(adaptiveLegOrder)
		computeCurrentBodyOffset(phases, speed, origOff[0], origOff[1], origOffA);
	
	/*std::cout << "Original phases:";
	 for(unsigned int leg=0; leg<NumLegs; ++leg)
	 std::cout << " " << phases[leg];
	 std::cout << std::endl;*/
	
	unsigned int origOrder[NumLegs];
	for(unsigned int leg=0; leg<NumLegs; ++leg)
		origOrder[leg]=legStates[leg].reorder;
	
	fmat::Column<3> origVel=fmat::pack(targetVel,targetAngVel);
	targetVel[0]=x;
	targetVel[1]=y;
	targetAngVel=a;
	fmat::Column<3> newVel=fmat::pack(targetVel,targetAngVel);
	LocomotionEvent e(EventBase::locomotionEGID, getID(), EventBase::statusETID);
	e.setXYA(x, y, a);
	postEvent(e);
	
	speed=targetVel.norm();
	float dirAlignment = fmat::dotProduct(origVel,newVel)/origSpeed/speed;
	
	bool standStill = resetPeriod(time,speed);
	
	if(standStill) {
		// no-op
	} else if(!adaptiveLegOrder) {
		for(unsigned int leg=0; leg<NumLegs; ++leg)
			legStates[leg].reorder = leg;
	} else if(std::abs(dirAlignment-1) > EPSILON) {
		// rotate leg ordering by direction of motion so gait is consistent relative to direction
		// e.g. ripple gait always moves from back to front, *in direction of motion*
		
		// find nominal "forward" leg ordering
		std::pair<float,unsigned int> fAngle[NumLegs];
		for(unsigned int leg=0; leg<NumLegs; ++leg) {
			// hack ... fix it later
            #ifdef TGT_IS_MANTIS
            static const int JointsPerLeg = 4;
            #endif
            fmat::Column<3> ep = childMap[LegOffset+leg*JointsPerLeg]->getT(*kine).column(3);
			fAngle[leg] = std::pair<float,unsigned int>(std::atan2(ep[1],ep[0]),leg);
		}
		std::stable_sort(fAngle,fAngle+NumLegs);
		
		// target heading
		float dir = 0;
		if (speed>EPSILON)
			dir += std::atan2(y,x);
		if (std::abs(targetAngVel) > speed*EPSILON) {
			fmat::Column<2> v=rotationCenter-targetVel;
			dir += std::atan2(v[1],v[0]) - std::atan2(rotationCenter[1],rotationCenter[0]);
		}
		
		// rotate legs by target heading
		std::pair<float,unsigned int> tAngle[NumLegs];
		for(unsigned int leg=0; leg<NumLegs; ++leg) {
			tAngle[leg] = std::pair<float,unsigned int>(fAngle[leg].first-dir,fAngle[leg].second);
			if(tAngle[leg].first<(float)M_PI)
				tAngle[leg].first+=2*(float)M_PI;
			if(tAngle[leg].first>(float)M_PI)
				tAngle[leg].first-=2*(float)M_PI;
		}
		std::stable_sort(tAngle,tAngle+NumLegs);
		
		// pull out reordered mapping
		for(unsigned int leg=0; leg<NumLegs; ++leg)
			legStates[fAngle[leg].second].reorder = tAngle[leg].second;
	}
	
	if(initialPlacement)
		return;
	
	bool legsReordered=false;
	for(unsigned int leg=0; leg<NumLegs; ++leg) {
		if(origOrder[leg]!=legStates[leg].reorder) {
			legsReordered=true;
			break;
		}
	}
	
	if(legsReordered) {
		unsigned int bestLeg=-1U;
		//float bestLegPhase=0;
		// fix phase stuff, want to align global phase so the same leg(s) are in the air
		for(unsigned int leg=0; leg<NumLegs; ++leg) {
			if(inAir[leg]) {
				// found leg in air, use it
				bestLeg=leg;
				break;
			} else {
				// also keep track of next flight (max phase)
				// if we don't find a leg in the air, we'll use this instead
				/*if(phases[leg]>bestLegPhase) {
				 bestLegPhase=phases[leg];
				 bestLeg=leg;
				 }*/
			}
		}
		if(bestLeg==-1U) {
			// keep global phase match, might not have crossed gait change threshold anyway...
		} else {
			float legphase;
			bool inAirTmp;
			computeLegPhase(bestLeg,inAirTmp,legphase);
			
			float tgtPhase = phases[bestLeg];
			//cout << origVel << " dot " << fmat::pack(targetVel,targetAngVel) << " = " << fmat::dotProduct(origVel,fmat::pack(targetVel,targetAngVel)) << endl;
			if(dirAlignment<0)
				tgtPhase = 1-tgtPhase; // reverse direction, reverse phase
			
			float shift;
			if(inAirTmp)
				shift = (legphase-tgtPhase)*p.legParams[bestLeg].flightDuration;
			else {
				float strideTime = period - p.legParams[bestLeg].flightDuration/1000.f;
				shift = (1-tgtPhase)*p.legParams[bestLeg].flightDuration - (p.legParams[bestLeg].flightDuration+legphase*strideTime*1000.f);
			}
			startTime-=shift; // convert and round to nearest millisecond
			time = (sysTime-startTime)/1000.f;
			computePhase(time);
		}
		
		bool inAirTmp; // ignored, don't want to overwrite original inAir entries
		//std::cout << "Reordered phases on "<< bestLeg << ": " << phases[bestLeg] << endl;
		for(unsigned int leg=0; leg<NumLegs; ++leg)
			computeLegPhase(leg,inAirTmp,phases[leg]);
		//std::cout << "Now: " << phases[bestLeg] << endl;
		/*for(unsigned int leg=0; leg<NumLegs; ++leg)
		 std::cout << " " << phases[leg];
		 std::cout << std::endl;*/
		
	}
	
	for(unsigned int leg=0; leg<NumLegs; ++leg)
		resetLegState(leg,phases[leg],curPos[leg],inAir[leg],speed);
	
	// if we shifted global phase to keep a leg in the air, need to fix the global offset
	if(legsReordered) {
		fmat::Column<2> off;
		float offA=0;
		computeCurrentBodyOffset(phases, speed, off[0], off[1], offA);
		off-=origOff;
		for(unsigned int leg=0; leg<NumLegs; ++leg)
			if(!inAir[leg])
				legStates[leg].downPos-=off;
		// todo: if we add a 'twist' parameter to do periodic offA adjustments, will need to something for that here
	}
}

void XWalkMC::setTargetVelocity(float xvel, float yvel, float avel, float time) {
	plantingLegs = false;
	targetDisp = fmat::pack(xvel*time,yvel*time);
	targetAngDisp = avel*time;
	displacementTime = get_time();
	if ( xvel == 0 && yvel == 0 && avel == 0 ) {
		setTargetVelocity(0, 0, 0);
	} else {
		setTargetVelocity(xvel,yvel,avel);
		velocityMode = false;
	}
}

void XWalkMC::setTargetDisplacement(float xdisp, float ydisp, float adisp, float time) {
	// std::cout << (void*)(this) << ":id=" << getID() <<"   setTargetDisplacement(" << xdisp << ", " << ydisp << ", " << adisp << ")" << std::endl;
	if ( xdisp == 0 && ydisp == 0 && adisp == 0 ) {
		velocityMode = false;
		plantingLegs = false;
		targetDisp = fmat::pack(xdisp,ydisp);
		targetAngDisp = adisp;
		displacementTime = get_time();
		setTargetVelocity(0, 0, 0);
		return;
	}
	const float tx = std::abs(xdisp/getMaxXVel());
	const float ty = std::abs(ydisp/getMaxYVel());
	const float ta = std::abs(adisp/getMaxAVel());
	const float maxTime =  std::max(time,std::max(tx,std::max(ty,ta)));
	setTargetVelocity(xdisp/maxTime, ydisp/maxTime, adisp/maxTime, maxTime);
	velocityMode = false;
}

int XWalkMC::updateOutputs() {
	unsigned int curTime=get_time();
	unsigned int dispTime = curTime-displacementTime;
	float dispSecs = dispTime / 1000.f;
	
	if(!isDirty()) {
		if ( plantingLegs ) {
			postEvent(EventBase(EventBase::motmanEGID, getID(), EventBase::statusETID, dispTime));
			plantingLegs = false;
		}
		return 0;
	}
	
	if ( ! velocityMode )  { // we're trying to travel a specific displacement
		if ( fabs(dispSecs*targetVel[0]) >= fabs(targetDisp[0]) &&
				fabs(dispSecs*targetVel[1]) >= fabs(targetDisp[1]) &&
				fabs(dispSecs*targetAngVel) >= fabs(targetAngDisp) ) {
			plantingLegs = true;
			setTargetVelocity(0, 0, 0);
			//return JointsPerLeg*NumLegs;
		}
	}
	
	// for z position, need to drop the x,y point along gravity vector to the ground plane
	fmat::Column<3> ground, gravity;
	packGroundGravity(ground, gravity);
	
	float speed = targetVel.norm();
	float dt = (curTime-startTime)/1000.f;
	
	kine->pullChildrenQFromArray(state->outputs, 0, NumOutputs);
	
	if(active.size()>0) {
		updateNeutralPos(curTime);
		// could probably be more selective about when we do the rest of this...
		/*computePhase(dt);
		 resetPeriod(dt,speed);
		 dt = (curTime-startTime)/1000.f;
		 for(unsigned int leg=0; leg<NumLegs; ++leg) {
		 float phase;
		 computeLegPhase(leg,legStates[leg].inAir,phase);
		 //fmat::Column<3> curPos = childMap[FootFrameOffset+leg]->getWorldPosition();
		 //resetLegState(leg,phase,curPos,legStates[leg].inAir,speed);
		 }*/
	}
	
	IKSolver::Point tgts[NumLegs];
	
	if(initialPlacement)
		updateOutputsInitial(curTime,ground,gravity,tgts);
	else
		updateOutputsWalking(dt,ground,gravity,speed,tgts);
	
	sendLoadPredictions(tgts);
	
	/*if(targetVelX==0 && targetVelY==0 && targetVelA==0) {
	 if(!resetOnStop)
	 return 0;
	 }*/
    #ifdef TGT_IS_MANTIS
    static const int JointsPerLeg = 4; // hack! fix it later
    #endif
	return JointsPerLeg*NumLegs;
}

void XWalkMC::start() {
	MotionCommand::start();
	kine->pullChildrenQFromArray(state->outputs, 0, NumOutputs);
	#ifdef TGT_IS_BIPED
	initialPlacement=false;
	#else
	initialPlacement=true;
	#endif
	startTime=get_time();
	if(period == 0)
		for(unsigned int leg=0; leg<NumLegs; ++leg)
			period += legParams[leg].totalDuration() * 2 / 1000.f;
	computePhase(0);
	for(unsigned int leg=0; leg<NumLegs; ++leg) {
		fmat::Column<3> curPos = childMap[FootFrameOffset+leg]->getWorldPosition();
		legStates[leg].liftPos = fmat::SubVector<2>(curPos);
		legStates[leg].inAir=true;
		legStates[leg].support=0;
		legStates[leg].initialHeight = curPos[2];
		resetLegState(leg,0,curPos,true,0);
	}
}

void XWalkMC::stop() {
	DriverMessaging::LoadPrediction loads;
	for(unsigned int i=0; i<NumLegs; ++i) {
		KinematicJoint * kj = childMap[FootFrameOffset+i];
		while(kj!=NULL) {
			if(kj->outputOffset<NumOutputs)
				loads.loads[kj->outputOffset.get()] = 0;
			kj=kj->getParent();
		}
	}
	DriverMessaging::postMessage(loads);
	
	// clear contact points too
	if(contactMsg.size()>0) {
		contactMsg.clear();
		contactMsg.flushOnMotionUpdate=false; // send immediately (if no other motion, might not send)
		DriverMessaging::postMessage(contactMsg);
	}
	
	MotionCommand::stop();
}

void XWalkMC::zeroVelocities() {
	unsigned int t=get_time();
	setTargetVelocity(0,0,0);
	LocomotionEvent e(EventBase::locomotionEGID, getID(), EventBase::deactivateETID, t-displacementTime); // velocities default to 0
	postEvent(e);
	displacementTime = t;
}


#if 0
void XWalkMC::lockLegsInPlace() {
	setTargetVelocity(0,0,0);
	
	float minHeight=std::numeric_limits<float>::max();
	for(unsigned int leg=0; leg<NumLegs; ++leg) {
		bool inAir;
		float phases;
		computeLegPhase(leg,inAir,phases);
		fmat::Column<3> curPos;
		computeCurrentPosition(leg, inAir, 0, phases, curPos);
		
		KinematicJoint * kj = childMap[FootFrameOffset+leg];
		float legr=0; // this will be the maximum distance from elevator to foot when leg is stretched out
		while(kj!=NULL && kj->outputOffset!=LegOffset+leg*JointsPerLeg) {
			legr += kj->r;
			kj = kj->getParent();
		}
		fmat::Transform baseToElv = childMap[LegOffset+leg*JointsPerLeg+1]->getFullInvT();
		fmat::Column<2> curPosElv(baseToElv*curPos);
		float curr = curPosElv.norm(); // this is the current distance from elevator to foot
		
		//float ang = baseToElv;
		
		float tgtheight = 0;
	}
}
#endif

void XWalkMC::updateNeutralPos(unsigned int curTime) {
//  return;   ***** DEBUG HACK *****
	std::set<ParameterTransition*> deactivated;
	for(std::set<ParameterTransition*>::const_iterator it=active.begin(); it!=active.end(); ++it)
		if(!(*it)->update(curTime))
			deactivated.insert(*it);
	for(std::set<ParameterTransition*>::const_iterator it=deactivated.begin(); it!=deactivated.end(); ++it)
		active.erase(*it);
	
	// each mid-stride position is base primarily on the location of the first leg joint
	// the strideBias then offsets the x position, and the stance width offsets the y position
	for(unsigned int leg=0; leg<NumLegs; ++leg) {
		LegParameters& legParam = p.legParams[leg];
        #ifdef TGT_IS_MANTIS
        static const int JointsPerLeg = 4; // hack! fix it later
        #endif
        KinematicJoint * legRoot = childMap[LegOffset+JointsPerLeg*leg];
		fmat::Column<3> fstPos = legRoot->getWorldPosition();
		legStates[leg].neutralPos[0] = fstPos[0] + legParam.strideBias;
		legStates[leg].neutralPos[1] = ((fstPos[1]<0) ? -*legParam.stanceWidth : *legParam.stanceWidth);
	}
    return;
}

bool XWalkMC::resetPeriod(float time, float speed) {
	bool standStill=false;
	
	if (std::abs(targetAngVel) <= speed*EPSILON) {
		// straight line motion
		if(speed>EPSILON) {
			// Since all legs on the ground move the same speed, spending more time on ground means taking longer stride
			// Since all legs share the same period, period - airtime = groundtime, so minimum air time is maximum ground time
			// Thus the minimum air time determines the period which produces the correct speed at the maximal stride
			float minAirTime = std::numeric_limits<float>::infinity();
			for(unsigned int leg=0; leg<NumLegs; ++leg) {
				float t = legParams[leg].totalDuration();
				//float t = legParams[leg].flightDuration; // *TRANSITION_STRIDE* //
				if(t<minAirTime)
					minAirTime=t;
			}
			minAirTime/=1000.f; // convert to seconds
			
			// Maximum stride length is determined by direction of motion (max stride x vs. y)
			// find intersection of targetVel with max-stride-elipse
			fmat::Column<2> d = fmat::pack(strideLenX*targetVel[1], strideLenY*targetVel[0]); // conveniently, don't need to normalize
			float groundTime = strideLenX*strideLenY / std::sqrt(d[0]*d[0] + d[1]*d[1]); // in seconds
			period = groundTime + minAirTime;
			//std::cout << "new period " << period << " " << groundTime << ' ' << targetVel << " " << speed << std::endl;
		} else {
			period = nominalPeriod();
			standStill=true;
		}
		
	} else { // arcing motion
		// turning, need to find center of rotation, handle legs correctly (inner legs don't move as fast...)
		rotationCenter[0] = -targetVel[1];
		rotationCenter[1] = targetVel[0];
		rotationCenter/=targetAngVel;
		
		// pick the shortest period so other legs will not exceed maximum stride length
		// This isn't quite right, is finding groundTime based on straight line motion
		// instead of arcing motion, yet arcing motion is what's used later.
		period = std::numeric_limits<float>::infinity();
		for(unsigned int leg=0; leg<NumLegs; ++leg) {
			fmat::Column<2> radial = legStates[leg].neutralPos - rotationCenter;
			if(radial.norm() < EPSILON) // neutral point is *on* rotation center, ignore it
				continue;
			// this does some fancy calculation to get ground time
			// basically an optimization of (x/a)^2 + (y/b)^2 = 1/t 
			fmat::Column<2> ivel = fmat::pack(-radial[1],radial[0])*targetAngVel; // instantaneous velocity
			fmat::Column<2> d = fmat::pack(strideLenX*ivel[1], strideLenY*ivel[0]); // conveniently, don't need to normalize
			float groundTime = strideLenX*strideLenY / d.norm(); // in seconds
			float airTime = legParams[leg].totalDuration() / 1000.f;
			//float airTime = legParams[leg].flightDuration / 1000.f; // *TRANSITION_STRIDE* //
			if(groundTime + airTime < period)
				period = groundTime + airTime;
			//std::cout << "testing leg " << leg << " " << period << " " << groundTime << ' ' << ivel << std::endl;
		}
		//std::cout << "new period " << period << std::endl;
	}
	
	if(!initialPlacement) {
		float origPhase=globPhase;
		computePhase(time);
		float shift = (origPhase-globPhase)*period;
		startTime-=shift*1000.f; // convert and round to nearest millisecond
		globPhase = origPhase;
		// to verify:
		//time = (sysTime-startTime)/1000.f;
		//computePhase(time);
		//cout << globPhase << " vs " << origPhase << endl;
	}
	
	return standStill;
}

void XWalkMC::resetLegState(unsigned int leg, float phase, const fmat::Column<3>& curPos, bool inAir, float speed) {
	fmat::Column<3> tgt;
	if(inAir) {
		legStates[leg].downPos = legStates[leg].neutralPos;
		// now take a half reverse-step to find optimal new downPos
		computeCurrentPosition(leg, false, speed, -0.5f, tgt);
		legStates[leg].downPos = fmat::SubVector<2>(tgt);
		// adjust liftPos to ensure smooth motion to new target
		/* Ideally, solve the inAir leg position equation used by computeCurrentPosition for lift, holding current position:
		 (lift-down) * phase + down = cur
		 However, as we get close to down, leg reorders can cause large, instantaneous movements (asking for numerical stability)
		 Instead, if less than half of flight remains, rotate remaining distance in direction of downPos */
		fmat::Column<2> cur(curPos); // ugly, need ConstSubVector...
		if(phase>.8) {
			// close to lift, solve in terms of down:  (cur - down) / phase + down
			legStates[leg].liftPos = (cur-legStates[leg].downPos)/phase + legStates[leg].downPos;
		} else {
			// close to down, solve in terms of lift: |cur - lift| / (1-phase) * phase * normalize(down - cur)
			fmat::Column<2> downDir = legStates[leg].downPos - cur;
			downDir/=downDir.norm();
			(cur-legStates[leg].liftPos).norm()/(1-phase)*phase * downDir;
		}
	} else {
		legStates[leg].downPos = fmat::Column<2>(curPos);
		// now take reverse-step of the current phase to find new downPos
		computeCurrentPosition(leg, false, speed, -phase, tgt);
		legStates[leg].downPos = fmat::SubVector<2>(tgt);
	}
}


void XWalkMC::updateOutputsInitial(unsigned int curTime, const fmat::Column<3>& ground, const fmat::Column<3>& gravity, IKSolver::Point tgts[]) {
	initialPlacement = false;
	bool contactMsgDirty=false; // set to true if there's actually a *new* contact in contactMsg
	
	float phases[NumLegs];
	bool inAirFixed = true;
	for(unsigned int leg=0; leg<NumLegs; ++leg) {
		computeLegPhase(leg,inAirFixed,phases[leg]);
	}
	
	fmat::Column<2> off;
	float offa=0;
	computeCurrentBodyOffset(phases, 0, off[0], off[1], offa);
	fmat::Matrix<2,2> offaM = fmat::rotation2D(offa);
	
	for(unsigned int leg=0; leg<NumLegs; ++leg) {
		legStates[leg].support=0;
		tgts[leg][0]=legStates[leg].neutralPos[0];
		tgts[leg][1]=legStates[leg].neutralPos[1];
		tgts[leg][2]=p.legParams[leg].flightHeight;
		(fmat::SubVector<2>)tgts[leg] = offaM * fmat::SubVector<2>(tgts[leg]) + off;
		
		float raiseDur = p.legParams[leg].raiseDuration*2;
		float lowerDur = p.legParams[leg].lowerDuration*2;
		float flightDur = p.legParams[leg].flightDuration*2;
		
		// move linearly to flight height above belly
		// note assumption that z=0 is belly of robot...
		fmat::Column<3> curP(legStates[leg].liftPos);
		curP[2] = legStates[leg].initialHeight;
		float phase;
		if(curP[2]>tgts[leg][2]) {
			// type A: already above ground, go straight to target
			phase = (curTime-startTime)/(raiseDur+flightDur);
			if(phase<1) {
				tgts[leg] *= phase;
				tgts[leg] += curP*(1-phase);
				initialPlacement = true;
			}
		} else {
			// type B: raise straight up first to lower onto belly, then move to target
			phase = (curTime-startTime)/raiseDur;
			if(phase<1) {
				// moving up
				tgts[leg][0] = curP[0];
				tgts[leg][1] = curP[1];
				tgts[leg][2] *= phase;
				tgts[leg][2] += curP[2]*(1-phase);
				initialPlacement = true;
			} else {
				// moving to target, assuming we're already at height
				phase = (curTime-startTime-raiseDur)/flightDur;
				if(phase<1) {
					curP[2] = tgts[leg][2];
					tgts[leg] *= phase;
					tgts[leg] += curP*(1-phase);
					initialPlacement = true;
				}
			}
		}
		if(phase>=1) {
			// above target, now lower back down to ground, then increase support
			phase = (curTime-startTime-raiseDur-flightDur)/(lowerDur);
			
			if(p.groundPlane[3] > 0) {
				phase += 1;
				//fmat::Column<3> physPos = childMap[FootFrameOffset+leg]->getWorldPosition();
				fmat::Column<3>& physPos = tgts[leg]; // use the target position instead of sensing, trade off robustness to interference vs. precision
				legStates[leg].liftPos = fmat::SubVector<2>(physPos);
				legStates[leg].initialHeight = physPos[2];
			}
			
			if(phase<1) {
				// lower to ground plane
				curP=tgts[leg];
				tgts[leg][2]=0;
				tgts[leg] *= phase;
				tgts[leg] += curP*(1-phase);
				// compute and store actual position in case we hit joint limits or obstacle
				//fmat::Column<3> physPos = childMap[FootFrameOffset+leg]->getWorldPosition();
				fmat::Column<3>& physPos = tgts[leg]; // use the target position instead of sensing, trade off robustness to interference vs. precision
				legStates[leg].liftPos = fmat::SubVector<2>(physPos);
				initialPlacement = true;
			} else {
				phase-=1;
				// lower to support height, lifting body (now support activated)
				if(phase<1) {
					if(legParams[leg].usable) {
						if(legStates[leg].inAir && !contactMsgDirty) {
							contactMsg.clear();
							contactMsgDirty=true;
						}
						contactMsg.addEntry(FootFrameOffset+leg, fmat::Column<3>()); //need to add all contacts, not just new ones
					}

					legStates[leg].inAir=false;
					legStates[leg].support=.5f; // not 1 so isDirty will still return true, not 0 so pressures/loads will be computed
					if(p.groundPlane[3] <= 0)
						curP[2]=0;
					tgts[leg]=curP;
					projectToGround(ground, p.groundPlane[3], gravity, tgts[leg]);
					tgts[leg] *= phase;
					tgts[leg] += curP*(1-phase);
					initialPlacement = true;
				}
			}
		}
	}
	if(contactMsgDirty) {
		contactMsg.flushOnMotionUpdate=true; // delay message to sync with corresponding joint angles
		DriverMessaging::postMessage(contactMsg);
	}
	if(initialPlacement) {
		for(unsigned int leg=0; leg<NumLegs; ++leg)
			solveIK(leg,tgts[leg]);
	} else {
		// sync time to end of leg 0 flight
		unsigned int timingleg = legStates[0].reorder==-1U ? 0 : legStates[0].reorder;
		float endflightphase = p.legParams[timingleg].flightPhase + p.legParams[0].flightDuration/1000.f/period;
		float shift = (endflightphase-globPhase)*period;
		startTime-=shift*1000.f; // convert and round to nearest millisecond
		computePhase((curTime-startTime)/1000.f);
		kine->pullChildrenQFromArray(state->outputs, 0, NumOutputs);
		for(unsigned int leg=0; leg<NumLegs; ++leg) {
			bool inAir; // ignored, all feet are on ground
			float phase;
			computeLegPhase(leg,inAir,phase);
			// todo: need to project back to xy plane along gravity vector
			fmat::Column<3> physPos = childMap[FootFrameOffset+leg]->getWorldPosition();
		//	offaM * fmat::SubVector<2>(tgts[leg]) + off;
			(fmat::SubVector<2>)physPos = offaM.transpose()*fmat::SubVector<2>(physPos) - offaM.transpose()*off;
			resetLegState(leg,phase,physPos,false,targetVel.norm());
			legStates[leg].support=1;
			legStates[leg].onGround=true;
		}
		LocomotionEvent e(EventBase::locomotionEGID,getID(),EventBase::activateETID,0);
		e.setXYA(targetVel[0], targetVel[1], targetAngVel);
		postEvent(e);
	}
	displacementTime=get_time();
}

void XWalkMC::updateOutputsWalking(float dt, const fmat::Column<3>& ground, const fmat::Column<3>& gravity, float speed, IKSolver::Point tgts[]) {
	/*TimeET walkTime;
	 static float walkAvg=0;
	 static int walkcnt=0;
	 static float ikAvg=0;
	 static int ikcnt=0;*/
	
	bool inAir[NumLegs];
	float phases[NumLegs];
	
	// compute xy leg positions
	
	computePhase(dt);
	
	for(unsigned int leg=0; leg<NumLegs; ++leg) {
		computeLegPhase(leg,inAir[leg],phases[leg]);
		//cout << leg << " inAir " << inAir[leg] << " phase " << phases[leg] << endl;
		if(legStates[leg].inAir!=inAir[leg]) {
			// this is a bit of a hack, walk sometimes lifts legs during "pure" parameter transition
			// so only initiate a leg lift if we are actually moving.  Always pass touch-downs through.
			if(speed>0 || std::abs(targetAngVel)>EPSILON || !inAir[leg]) {
				legStates[leg].inAir=inAir[leg];
			} else { // hold the leg on the ground
				inAir[leg] = false;
			}
			if(inAir[leg]) {
				legStates[leg].downPos = legStates[leg].neutralPos;
				computeCurrentPosition(leg, false, speed, -0.5f, tgts[leg]);
				legStates[leg].downPos = fmat::SubVector<2>(tgts[leg]);
			}
		}
	}
	
	fmat::Column<2> off;
	float offa=0;
	computeCurrentBodyOffset(phases, speed, off[0], off[1], offa);
	fmat::Matrix<2,2> offaM = fmat::rotation2D(offa);
	
	for(unsigned int leg=0; leg<NumLegs; ++leg) {
		// get some statistics on leg position
		/*if(legStates[leg].support==1) {
		 float dist = p.groundPlane[3] - fmat::dotProduct(childMap[FootFrameOffset+leg]->getWorldPosition(),ground);
		 float align = fmat::dotProduct(gravity,ground);
		 if(align<EPSILON) // we're trying to walk up a perfectly vertical cliff...
		 align=EPSILON; //pretend it's near vertical
		 cout << leg << ' ' << phases[leg] << ' ' << (dist/align) << endl;
		 }*/
		
		computeCurrentPosition(leg, inAir[leg], speed, phases[leg], tgts[leg]);
		if(!inAir[leg])
			legStates[leg].liftPos = fmat::SubVector<2>(tgts[leg]);
		(fmat::SubVector<2>)tgts[leg] = offaM * fmat::SubVector<2>(tgts[leg]) + off;
	}
	
	// do the "extra", non-leg joints before leg IK in case we're changing earlier joints (e.g. rotor joints on Chiara)
	for(plist::DictionaryOf<SinusoidalParameters>::const_iterator it=nonLegJoints.begin(); it!=nonLegJoints.end(); ++it) {
		unsigned int idx = capabilities.findOutputOffset(it->first.c_str());
		if(idx!=-1U) {
			float val = (*it->second)(globPhase,phases);
			motman->setOutput(this,idx,val);
			if(KinematicJoint * kj = childMap[idx])
				kj->freezeQ(val);
		}
	}
	
	bool contactChanged=false; // in the following loop, will set this flag if a foot has transitioned to/from full support (LegState::onGround toggled)

	// for each leg, project foot position to ground plane along gravity vector,
	// solve inverse kinematics, and send joint angles to motion manager
	for(unsigned int leg=0; leg<NumLegs; ++leg) {
		projectToGround(ground, p.groundPlane[3], gravity, tgts[leg]);
		
		if(inAir[leg]) {
			tgts[leg] += ground*(p.legParams[leg].flightHeight + bounce(globPhase,phases));
			legStates[leg].support=0;
		} else {
			float strideTime = period - p.legParams[leg].flightDuration/1000.f;
			float height = p.legParams[leg].flightHeight;
			float t = (1-phases[leg])*strideTime;
			float lowerDur = p.legParams[leg].lowerDuration/1000.f;
			float raiseDur = p.legParams[leg].raiseDuration/1000.f;
			if(t>strideTime - lowerDur && legStates[leg].support<1) { // don't jump into lowering if not previously raised
				float acc = 4 * p.legParams[leg].flightHeight / lowerDur / lowerDur;
				t = strideTime - t;
				if(t<lowerDur/2) {
					height -= acc * t * t / 2;
				} else {
					t = lowerDur - t;
					height = acc * t * t / 2;
				}
				tgts[leg] += ground*height;
				legStates[leg].support=0;//1-height/p.legParams[leg].flightHeight;
			} else if(t<raiseDur && (speed>EPSILON || std::abs(targetAngVel)>EPSILON)) { // don't initiate a raise if not walking
				float acc = 4 * p.legParams[leg].flightHeight / raiseDur / raiseDur;
				if(t<raiseDur/2) {
					height -= acc * t * t / 2;
				} else {
					t = raiseDur - t;
					height = acc * t * t / 2;
				}
				tgts[leg] += ground*height;
				legStates[leg].support=0;//1-height/p.legParams[leg].flightHeight;
			} else {
				legStates[leg].support=1;
			}
			tgts[leg] += ground*bounce(globPhase,phases);
			
			if( (legStates[leg].support>=1) != legStates[leg].onGround) {
				contactChanged=true;
				/*static unsigned int times[NumLegs];
				static fmat::Column<3> downs[NumLegs];
				if(legStates[leg].support>=1) {
					times[leg]=get_time();
					downs[leg]=tgts[leg];
					float stride = (legStates[leg].neutralPos - legStates[leg].downPos).norm();
					float airTime = (legParams[leg].flightDuration) / 1000.f;
					//float airTime = (legParams[leg].raiseDuration + legParams[leg].flightDuration + legParams[leg].lowerDuration) / 1000.f; // *TRANSITION_STRIDE* //
					std::cout << "leg " << leg << " non-air stride " << stride*2 << " speed " << stride*2/(period-airTime) << std::endl;
				} else {
					float stride = (downs[leg] - tgts[leg]).norm();
					std::cout << "leg " << leg << " ground stride " << stride << " speed " << stride/(get_time()-times[leg])*1000 << std::endl;
				}*/
			}
			legStates[leg].onGround = (legStates[leg].support>=1);
		}
		
		// inverse kinematics to get leg to target, and send values to motion manager
		//TimeET ikTime;
		solveIK(leg,tgts[leg]);
		//ikAvg+=ikTime.Age().Value();
		//++ikcnt;
	}
	
	if(contactChanged) {
		// send driver message reporting leg contacts
		if(DriverMessaging::hasListener(DriverMessaging::FixedPoints::NAME)) {
			contactMsg.clear();
			for(unsigned int leg=0; leg<NumLegs; ++leg) {
				if(legStates[leg].onGround && legParams[leg].usable)
					contactMsg.addEntry(FootFrameOffset+leg, fmat::Column<3>());
			}
			contactMsg.flushOnMotionUpdate=true; // delay message to sync with corresponding joint angles
			postMessage(contactMsg);
		}
	}
	
	/*walkAvg+=walkTime.Age().Value();
	 ++walkcnt;
	 std::cout << "Calculation time " << ikAvg/ikcnt << ' ' << walkAvg/walkcnt << std::endl;*/
}

void XWalkMC::sendLoadPredictions(IKSolver::Point tgts[]) {
	// predict servo loads by solving for pressure distribution over the supporting feet, then
	// using Jacobian of each leg to find it's servo's loads
	DriverMessaging::LoadPrediction loads;
	
	// first, find center of mass and count ground legs
	KinematicJoint * base = childMap[BaseFrameOffset];
	std::set<KinematicJoint*> nonsupports(base->getBranches().begin(),base->getBranches().end());
	std::map<unsigned int, KinematicJoint*> supports;
	for(unsigned int leg=0; leg<NumLegs; ++leg) {
		if(legStates[leg].inAir || legStates[leg].support<EPSILON || !legParams[leg].usable) {
			// todo: these servos are marked unloaded, but do carry some load from the leg itself... could process that
            #ifdef TGT_IS_MANTIS
            static const int JointsPerLeg = 4; // hack! fix it later
            #endif
            KinematicJoint * j = childMap[LegOffset+JointsPerLeg*leg];
			while(j->getParent()!=base) {
				if(j->outputOffset<NumOutputs)
					loads.loads[j->outputOffset.get()]=0;
				j = j->getParent();
			}
		} else {
            #ifdef TGT_IS_MANTIS
            static const int JointsPerLeg = 4; // hack! fix it later
            #endif
            KinematicJoint * j = childMap[LegOffset+JointsPerLeg*leg];
			while(j->getParent()!=base)
				j = j->getParent();
			supports[leg]=j;
			nonsupports.erase(j);
		}
	}
	if(supports.size()>0) {
		fmat::Column<4> com = base->getMassVector();
		for(std::set<KinematicJoint*>::const_iterator it=nonsupports.begin(); it!=nonsupports.end(); ++it)
			com += (*it)->getTq() * (*it)->sumCenterOfMass();
		//cout << "Center of Mass " << com[3] << ": " << com/com[3] << endl;
		
		std::vector<fmat::Column<2> > contacts;
		// todo: should probably use forward kinematics of effector instead of target point in case of IK failure (e.g. joint limit)
		for(std::map<unsigned int, KinematicJoint*>::const_iterator it=supports.begin(); it!=supports.end(); ++it)
			contacts.push_back(fmat::SubVector<2>(tgts[it->first]));
		// todo: project contacts into plane normal to gravity vector
		
		std::vector<float> pressures;
		std::vector<unsigned int> negatives;
		do {
			computePressure(com[3],com[0]/com[3],com[1]/com[3],contacts,pressures);
			
			// drop negative pressures and recompute
			// this drops all negatives all at once... might be better to drop only the most negative and then recompute?
			negatives.clear();
			std::map<unsigned int, KinematicJoint*>::const_iterator it=supports.end();
			for(unsigned int i=pressures.size(); i!=0;) {
				--it;
				if(pressures[--i]<0) {
					negatives.push_back(it->first);
					contacts.erase(contacts.begin()+i);
				}
			}
			for(unsigned int i=0; i<negatives.size(); ++i)
				supports.erase(negatives[i]);
		} while(!negatives.empty());
		
		// now for each support, find servo torques in that chain
		std::vector<float>::const_iterator pressureIt=pressures.begin();
		for(std::map<unsigned int, KinematicJoint*>::const_iterator it=supports.begin(); it!=supports.end(); ++it,++pressureIt) {
			std::vector<fmat::Column<6> > j;
			KinematicJoint * kj = childMap[FootFrameOffset+it->first];
			// todo: should probably use forward kinematics of effector instead of target point in case of IK failure (e.g. joint limit)
			kj->getFullJacobian(tgts[it->first],j);
			while(kj!=NULL) {
				if(kj->outputOffset<NumOutputs) {
					// todo: pressure should be along gravity, not z axis
					loads.loads[kj->outputOffset.get()] = (*pressureIt)*j[kj->getDepth()][2]/1000;
				}
				kj=kj->getParent();
			}
		}
		DriverMessaging::postMessage(loads);
	}
}


void XWalkMC::computePhase(float time) {
	if(!isDirty())
		return;
	globPhase = time/period;
	globPhase -= std::floor(globPhase);
}

void XWalkMC::computeLegPhase(unsigned int leg, bool& inAir, float& phase) {
	unsigned int timingLeg = leg;
	if(adaptiveLegOrder && legStates[leg].reorder!=-1U)
		timingLeg = legStates[leg].reorder;
	
	float legphase = globPhase - p.legParams[timingLeg].flightPhase;
	if(legphase<0)
		legphase+=1;
	float flightphase = p.legParams[leg].flightDuration/1000.f/period;
	inAir=(legphase<flightphase);
	phase = (inAir) ? 1-legphase/flightphase : (legphase-flightphase)/(1-flightphase);
	if(phase<0)
		phase+=1;
}

void XWalkMC::computeCurrentPosition(unsigned int leg, bool inAir, float speed, float phase, fmat::Column<3>& tgt) {
	const fmat::Column<2>& sp = legStates[leg].downPos;
	
	if(inAir) {
		(fmat::SubVector<2>)(tgt) = sp + phase*(legStates[leg].liftPos - sp);
		//cout << leg << " down " << sp <<  " lift " << legStates[leg].liftPos << " target " << tgt << endl;
		
	} else { 
		float nonFlightTime = period - legParams[leg].flightDuration/1000.f;
		// the raise and lower are considered part of the "ground" phase
		//nonFlightTime -= (legParams[leg].raiseDuration + legParams[leg].lowerDuration) / 1000.f; // *TRANSITION_STRIDE* //
		
		if (std::abs(targetAngVel) <= speed*EPSILON) { // straight line motion
			(fmat::SubVector<2>)(tgt) = sp + -phase*nonFlightTime*targetVel;
		} else { // arcing motion
			// rotate target point about the previously determined center of rotation
			(fmat::SubVector<2>)(tgt) = fmat::rotation2D(-phase*nonFlightTime*targetAngVel) * (sp-rotationCenter) + rotationCenter;
		}
	}
}

void XWalkMC::computeCurrentBodyOffset(float* legPhases, float speed, float& offx, float& offy, float& offa) {
	offx = surge(globPhase,legPhases);
	offy = sway(globPhase,legPhases);
	offa = 0;
	if(rotateBodyMotion && (offx!=0 || offy!=0) && std::abs(speed)>EPSILON) {
		float s = targetVel[1]/speed, c = targetVel[0]/speed;
		float tmpx = c*offx - s*offy;
		float tmpy = s*offx + c*offy;
		offx=tmpx;
		offy=tmpy;
	}
	offx-=p.offsetX;
	offy-=p.offsetY;
	offa-=p.offsetA;
}

void XWalkMC::solveIK(unsigned int leg, const IKSolver::Point& tgt) {
	if(!legParams[leg].usable)
		return;

	KinematicJoint * eff = childMap[FootFrameOffset+leg];

#ifndef TGT_IS_BIPED
	eff = childMap[FootFrameOffset+leg];
	eff->getIK().solve(IKSolver::Point(), *eff, tgt);
#else
	/*float theta = 3.14159/2.;
	if(abs(tgt[1])!=23)
	    theta = atan(tgt[2]/(abs(tgt[1]-23));
    if(leg)
    {
        if(tgt[1] != -23.)
            theta = atan(groundPlane[3]/(tgt[1]+23.));
        else
            theta = -3.14159/2.;
    }
    else
    {
        if(tgt[1] != 23.)
            theta = atan(groundPlane[3]/(tgt[1]-23.));
        else
            theta = 3.14159/2.;
    }*/

	IKSolver::Point new_tgt;
	new_tgt[0] = tgt[0];
	new_tgt[1] = tgt[1];
	new_tgt[2] = tgt[2]+55;
    /*if(theta < 0)
	    new_tgt[1] = tgt[1]+44.0*cos(theta);
    else
        new_tgt[1] = tgt[1]-44.0*cos(theta);
	new_tgt[2] = tgt[2]+11.0+44.0*sin(theta+3.14159/2);*/
	
	fmat::Column<3> anklePos = childMap[LegOffset+leg*JointsPerLeg + LegAnkleOffset]->getPosition();

	eff = childMap[LegOffset+LegKneeOffset+leg*JointsPerLeg];
	eff->getIK().solve(anklePos, *eff, new_tgt);
	float temp = eff->getQ() + eff->getParent()->getQ();     //instead of finding the orientation of the knee I mirrored the hip joints to force the legs to be parellel to the ground.  Doesn't quite work with sidestepping motion, might also solve turning issues?
	float other_tmp = eff->getParent()->getParent()->getQ();
	eff = childMap[FootFrameOffset+leg];
	eff->getParent()->tryQ(other_tmp*-1);
	eff->getParent()->getParent()->tryQ(temp);
	/*if(leg)
	{
	    float ankleAng = eff->getParent()
	    eff->getParent()->tryQ(theta-1.57);
	    eff->getParent()->getParent()->tryQ(0);
	}
	else
	{
	    eff->getParent()->tryQ((theta-1.57)*-1);
	    eff->getParent()->getParent()->tryQ(0);
	}*/
#endif

	/*if(leg==1)
	 std::cout << "Leg " << leg << " end pos: " << eff->getWorldPosition() << std::endl;*/
	for(; eff!=NULL; eff=eff->getParent()) {
		if(eff->outputOffset!=plist::OutputSelector::UNUSED && eff->outputOffset<NumOutputs) {
			/*if(leg==1)
			 std::cout << eff->outputOffset.get() << " set to " << eff->getQ() << std::endl;*/
#ifndef PLATFORM_APERIOS
			if(!std::isnan(eff->getQ()))
#endif
			{
				motman->setOutput(this, eff->outputOffset, eff->getQ());
			}
		}
	}
}

void XWalkMC::computePressure(float mass, float massx, float massy, const std::vector<fmat::Column<2> >& contacts, std::vector<float>& pressures) {
	const float gAcc = 9.80665f;
	NEWMAT::ColumnVector weight(3);
	const float force = mass*gAcc;
	weight(1) = force;
	weight(2) = massx/1000.f*force;
	weight(3) = massy/1000.f*force;
	
	//cout << mass << " @ " << massx <<',' << massy << endl;
	
	// now build matrix of statics constraints:
	NEWMAT::Matrix statics(3,contacts.size());
	for(unsigned int i=0; i<contacts.size(); ++i) {
		statics(1,i+1) = 1;
		statics(2,i+1) = contacts[i][0]/1000.f;
		statics(3,i+1) = contacts[i][1]/1000.f;
	}
	
	NEWMAT::ColumnVector f;
	if(statics.ncols() == 0) {
		// no legs on the ground?  wtf.
	} else if(statics.ncols() < 3) {
		// over constrained, linear least squares
		NEWMAT::Matrix U, V;
		NEWMAT::DiagonalMatrix Q;
		NEWMAT::SVD(statics,Q,U,V,true,true);
		for(int i=1; i<=Q.ncols(); ++i) {
			if(Q(i)<EPSILON)
				Q(i)=0;
			else
				Q(i)=1/Q(i);
		}
		f = V*Q*U.t()*weight;
	} else if(statics.ncols() == 3) {
		// square matrix, easy invert
		try {
			f = statics.i()*weight;
		} catch(...) {
			// hmm, if that failed probably singular, fall back on SVD
			NEWMAT::Matrix U, V;
			NEWMAT::DiagonalMatrix Q;
			NEWMAT::SVD(statics,Q,U,V,true,true);
			for(int i=1; i<=Q.ncols(); ++i) {
				if(Q(i)<EPSILON)
					Q(i)=0;
				else
					Q(i)=1/Q(i);
			}
			f = V*Q*U.t()*weight;
		}
	} else {
		// under constrained, Moore-Penrose psuedo inverse
		NEWMAT::Matrix U, V;
		NEWMAT::DiagonalMatrix Q;
		NEWMAT::SVD(statics.t(),Q,U,V,true,true);
		for(int i=1; i<=Q.ncols(); ++i) {
			if(Q(i)<EPSILON)
				Q(i)=0;
			else
				Q(i)=1/Q(i);
		}
		f = U*Q*V.t()*weight;
	}
	
	pressures.resize(contacts.size());
	for(unsigned int i=0; i<contacts.size(); ++i)
		pressures[i]=f(i+1);
}


void XWalkMC::spiderSettings(plist::DictionaryBase& src, plist::DictionaryBase& dst) {
	for(plist::DictionaryBase::const_iterator itS=src.begin(), itD=dst.begin(); itS!=src.end(); ++itS, ++itD) {
		if(plist::DictionaryBase* dcol=dynamic_cast<plist::DictionaryBase*>(itS->second)) {
			spiderSettings(*dcol,dynamic_cast<plist::DictionaryBase&>(*itD->second));
		} else if(plist::ArrayBase* acol=dynamic_cast<plist::ArrayBase*>(itS->second)) {
			spiderSettings(*acol,dynamic_cast<plist::ArrayBase&>(*itD->second));
		} else if(plist::PrimitiveBase* prim=dynamic_cast<plist::PrimitiveBase*>(itS->second)) {
			plist::PrimitiveBase& primDst = dynamic_cast<plist::PrimitiveBase&>(*itD->second);
			ParameterTransition * trans = new ParameterTransition(*prim,primDst,active,transitionDuration);
			transitions.insert(trans);
		}
	}
}

void XWalkMC::spiderSettings(plist::ArrayBase& src, plist::ArrayBase& dst) {
	for(unsigned int i=0; i<src.size(); ++i) {
		if(plist::DictionaryBase* dcol=dynamic_cast<plist::DictionaryBase*>(&src[i])) {
			spiderSettings(*dcol,dynamic_cast<plist::DictionaryBase&>(dst[i]));
		} else if(plist::ArrayBase* acol=dynamic_cast<plist::ArrayBase*>(&src[i])) {
			spiderSettings(*acol,dynamic_cast<plist::ArrayBase&>(dst[i]));
		} else if(plist::PrimitiveBase* prim=dynamic_cast<plist::PrimitiveBase*>(&src[i])) {
			plist::PrimitiveBase& primDst = dynamic_cast<plist::PrimitiveBase&>(dst[i]);
			ParameterTransition * trans = new ParameterTransition(*prim,primDst,active,transitionDuration);
			transitions.insert(trans);
		}
	}
}


void XWalkMC::ParameterTransition::plistValueChanged(const plist::PrimitiveBase& /*pl*/) {
	decelerate=false;
	lastUpdate=startTime=get_time();
	float dur = duration/2.f;
	cura = 2 * ( (src.castTo<float>()-dst.castTo<float>())/2 - curd * dur ) / (dur*dur);
	active.insert(this);
}

bool XWalkMC::ParameterTransition::update(unsigned int curTime) {
	if(curTime>startTime+duration) {
		curd=0;
		dst=src;
		return false;
	}
	float srcV = src.castTo<float>(), dstV = dst.castTo<float>();
	float dt = (startTime+duration)-lastUpdate; // time remaining
	float f = dstV + curd*dt + 0.5f * -cura*dt*dt; // predicted position if we start/continue decelerating now...
	if( (f>=srcV && dstV<=srcV) || (f<=srcV && dstV>=srcV) )
		decelerate=true;
	if(decelerate) {
		// would overshoot, decelerate
		float dist = srcV-dstV;
		// this would keep the target time constant, but discontinuity in velocity
		//curd = 2 * dist / dt;
		//float acc = ( dist - 2 * curd * dt ) / (dt*dt);
		// instead, this lets time flex, keeps other parameters continuous
		float acc =  - curd*curd / (2 * dist);
		startTime = (unsigned int)(curTime + 2 * dist / curd - duration + .5f);
		// std::cout << "  ParameterTransition::update: startTime=" << startTime << std::endl;
		dt = curTime-lastUpdate; // now compute current position given current time
		f = plist::Primitive<float>(dstV + curd*dt + acc * dt * dt / 2);
		// don't overshoot:
		if( (f>=srcV && dstV<=srcV) || (f<=srcV && dstV>=srcV) ) {
			curd=0;
			dst=src;
			return false;
		}
		dst = f;
		curd += acc*dt;
	} else {
		// accelerate
		dt = curTime-lastUpdate+.5f;
		dst = plist::Primitive<float>(dstV + curd*dt + cura*dt*dt/2);
		curd += cura*dt;
	}
	lastUpdate=curTime;
	return true;
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif // TGT_HAS_LEGS
