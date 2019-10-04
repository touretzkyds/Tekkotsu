#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_WHEELS

#include "WheeledWalkMC.h"
#include "Kinematics.h"
#include "Events/LocomotionEvent.h"
#include "Shared/Config.h"
#include <sys/stat.h>

using namespace std; 

void WheeledWalkMC::resetConfig() {
	updateWheelConfig();  // calculates maxVel values
	// default to going half maximum speed
	preferredXVel = getMaxXVel() / 2;
	preferredYVel = getMaxYVel() / 2;
	preferredAngVel = getMaxAVel() / 2;
	// check if a config file is available to alter preferred speeds
	std::string file = config->motion.makePath("wheeledwalk.plist");
	struct stat statbuf;
	if(::stat(file.c_str(),&statbuf)==0)
		loadFile(file.c_str()); // override with settings from file
}

int WheeledWalkMC::updateOutputs() {
	//std::cout << "WheeledWalkMC::updateOutputs " << (void*)this << "  displacementMode=" << displacementMode
	//					<< " dirty=" << dirty << std::endl;
	unsigned int t = get_time();
	unsigned int dur = t - travelStartTime;
	if (!dirty) { // Not dirty means current walk values previously sent to motman, and haven't changed
		if (displacementMode) {
#if defined(TGT_IS_CREATE)
			float distTraveled = state->sensors[EncoderDistanceOffset] - travelStartDist;
			float angTraveled = state->sensors[EncoderAngleOffset]*M_PI/180.0 - travelStartAngle;
			/* 
				 The Create updates its EncoderDistance value slowly enough
				 that it jumps in increments of 7 to 20 mm for typical travel
				 speeds.  To minimize the under/overshoot we should stop when we are
				 within jump/2 of the target displacement.  At default speed, jump is 14,
				 so use a fudgeFactor of 7 for now.  Should determine dynamically.
			*/
			float distFudgeFactor = 7;  // should determine empirically since this varies by speed
			float angFudgeFactor = 2.5 / 180 * M_PI;
#elif defined(TGT_IS_CREATE2)
			// *** replace this with encoder calculation
			float distTraveled = state->sensors[EncoderDistanceOffset] - travelStartDist;
			float angTraveled = state->sensors[EncoderAngleOffset]*M_PI/180.0 - travelStartAngle;
			float distFudgeFactor = 0;
			float angFudgeFactor = 0;
#elif defined(TGT_IS_KOBUKI)
			unsigned short currentTickLeft = state->sensors[LeftEncoderOffset];
			float leftDiffTicks = (float)(short)((currentTickLeft - lastTickLeft) & 0xffff);
			lastTickLeft = currentTickLeft;
			unsigned short currentTickRight = state->sensors[RightEncoderOffset];
			float rightDiffTicks = (double)(short)((currentTickRight - lastTickRight) & 0xffff);
			lastTickRight = currentTickRight;	

			float dx = RobotInfo:: wheelRadius * RobotInfo::tickToRad * (leftDiffTicks + rightDiffTicks)/2.0;
  		float dtheta = RobotInfo::wheelRadius * RobotInfo::tickToRad * (rightDiffTicks - leftDiffTicks)/RobotInfo::wheelBase;
  		float distTraveled = float(dx + lastDistTraveled);
  		float angTraveled = float(dtheta + lastAngTraveled);
			lastDistTraveled = distTraveled;
			lastAngTraveled = angTraveled;
			float distFudgeFactor = 0;
			float angFudgeFactor = 0 / 180 * M_PI;
#else
#  error "Wheeled walk can't do odometry: target is not a CREATE or KOBUKI"
#endif
			//std::cout << "wwmc:  dist = " << dist << " targetDist = " << targetDist << "     angdist = " << angdist*180/M_PI << std::endl;
			// Must use zeroVelocities() here, not
			// setTargetVelocity, because we must post a
			// locomotionEvent to let Mirage know we're stopping.
			// std::cout<< ">>> Debug: angTraveled = " << angTraveled << "   targetAngDist = " << targetAngDist << std::endl;
			if ( (targetDist != 0 && fabs(distTraveled)+distFudgeFactor >= fabs(targetDist)) ||
					 (targetDist == 0 && fabs(angTraveled)+angFudgeFactor >= fabs(targetAngDist)) ) {
		  	zeroVelocities();
				postEvent(EventBase(EventBase::motmanEGID, getID(), EventBase::statusETID, dur));
			}
		}
		if(targetVel[0]==0 && targetVel[1]==0 && targetAngVel==0)
			return 0;
	}
	if (dirty) {
		travelStartTime = t;
#if defined(TGT_IS_CREATE) || defined(TGT_IS_CREATE2)
		travelStartDist = state->sensors[EncoderDistanceOffset];
		travelStartAngle = state->sensors[EncoderAngleOffset]*M_PI/180.0;
#elif defined(TGT_IS_KOBUKI)
		travelStartDist = 0;
		travelStartAngle = 0;
#endif
		LocomotionEvent e(EventBase::locomotionEGID, getID(), EventBase::statusETID, dur);
		e.setXYA(targetVel[0], targetVel[1], targetAngVel);
		postEvent(e);
	}
	for(unsigned int i=0; i<NumWheels; ++i) {
		if(wheels[i].valid) {
			motman->setOutput(this, WheelOffset+i, wheels[i].targetVel);
		}
	}
	dirty=false;
	return NumWheels;
}

void WheeledWalkMC::start() {
	dirty = true;
	travelStartTime = get_time();
#if	defined(TGT_IS_CREATE) || defined(TGT_IS_CREATE2)
	travelStartDist = state->sensors[EncoderDistanceOffset];
	travelStartAngle = state->sensors[EncoderAngleOffset]*M_PI/180.0;
#elif defined(TGT_IS_KOBUKI)
	travelStartDist = 0;
	travelStartAngle = 0;
#endif
}

void WheeledWalkMC::stop() {
	zeroVelocities();
	MotionCommand::stop();
}

int WheeledWalkMC::isAlive() {
	return !displacementMode || getTravelTime()<=targetDur || dirty;
}

void WheeledWalkMC::zeroVelocities() {
	if(targetVel[0]==0 && targetVel[1]==0 && targetAngVel==0)
		return;
	unsigned int t = getTravelTime(); // cache because travelStartTime is about to be reset
	setTargetVelocity(0, 0, 0);
	for (unsigned int i = WheelOffset; i < WheelOffset + NumWheels; i++)
		motman->setOutput(this, i, 0.0f);
	postEvent(LocomotionEvent(EventBase::locomotionEGID, getID(), EventBase::statusETID, t));
}

void WheeledWalkMC::setTargetVelocity(float xvel, float yvel, float avel) {
	// std::cout << "setTargetVelocity xvel=" << xvel << " avel=" << avel << std::endl;
	displacementMode=false;
	if(std::abs(targetVel[0]-xvel)<0.01f && std::abs(targetVel[1]-yvel)<0.01f && std::abs(targetAngVel-avel)<0.001f)
		return; // don't bother with updates (and in particular, associated LocomotionEvents) from small rounding errors
	targetVel[0] = xvel;
	targetVel[1] = yvel;
	targetAngVel = avel;
	updateWheelVels();
}

void WheeledWalkMC::setTargetVelocity(float xvel, float yvel, float avel, float time) {
	displacementMode=true;
	if(time<=0 || (xvel==0 && yvel==0 && avel==0) ) {
		targetVel[0] = targetVel[1] = targetAngVel = 0;
		targetDur = 0;
	} else {
		targetVel[0] = xvel;
		targetVel[1] = yvel;
		targetAngVel = avel;
		targetAngDist = avel * time;  // need this for updateWheelVels
		targetDur = static_cast<unsigned int>(time*1000 + 0.5);
	}
	updateWheelVels();
}

void WheeledWalkMC::setTargetDisplacement(float xdisp, float ydisp, float adisp, float time) {
	// std::cout << (void*)(this) << ":id=" << getID() <<"   setTargetDisplacement(" << xdisp << ", " << ydisp << ", " << adisp << ")" << std::endl;
	if ( xdisp == 0 && ydisp == 0 && adisp == 0 ) {
		setTargetVelocity(0, 0, 0, 0);
		return;
	}
	
	// select preferred velocity if time==0, otherwise cap at maximum
	float xCap=getMaxXVel(), yCap=getMaxYVel(), aCap=getMaxAVel();
	if(time==0) {
		if(xCap>preferredXVel) xCap = preferredXVel;
		if(yCap>preferredYVel) yCap = preferredYVel;
		if(aCap>preferredAngVel) aCap = preferredAngVel;
	}
  xCap = 100; // ***** GREASY HACK TO MAKE CREATE SPEED WORK
	
	// test each cap, may be constrained - set to 0
	const float tx = xCap>0 ? std::abs(xdisp/xCap) : 0;
	const float ty = yCap>0 ? std::abs(ydisp/yCap) : 0;
	const float ta = aCap>0 ? std::abs(adisp/aCap) : 0;
	float maxTime =  std::max(time,std::max(tx,std::max(ty,ta)));
	
	// round up to closest FrameTime * NumFrames multiple for even better precision
	maxTime = std::ceil(maxTime * 1000 / (FrameTime*NumFrames)) * (FrameTime*NumFrames) / 1000.f;
	//std::cout << "xCap=" << xCap << " tx=" << tx << " xdisp=" << xdisp << " maxTime=" << maxTime << std::endl;
  //std::cout << "maxVel is " << maxVel[0] << " " << maxVel[1] << " " << maxVel[2] << " preferreXVel=" << preferredXVel << std::endl;
	targetDist = xdisp;
	targetAngDist = adisp;
	setTargetVelocity(xdisp/maxTime, ydisp/maxTime, adisp/maxTime, maxTime);
}

void WheeledWalkMC::setTargetDisplacement(float xdisp, float ydisp, float adisp, float xvel, float yvel, float avel) {
	//std::cout << "WheeledWalkMC::setTargetDisplacement xdisp=" << xdisp << " xvel=" << xvel
	//					<< "  adisp=" << adisp << " avel=" << avel << std::endl;
	targetVel = fmat::pack(xdisp>=0 ? xvel : -xvel, ydisp>=0 ? yvel : -yvel);
	targetAngVel = adisp >= 0 ? avel : -avel;
	targetDist = xdisp;
	targetAngDist = adisp;
	displacementMode = true;
	updateWheelVels();
	dirty = true;
}
void WheeledWalkMC::updateWheelVels() {
	if ( (displacementMode && std::abs(targetAngDist) < 0.01) || std::abs(targetAngVel) <= targetVel.norm()*EPSILON) {
		// straight line motion
#ifdef TGT_IS_KOBUKI
		//std::cout << "targetVel[0] = " << targetVel[0];
		wheels[SpeedOffset].targetVel = targetVel[0];
		wheels[RadiusOffset].targetVel = 0;
#else
		for(unsigned int i=0; i<NumWheels; ++i)
			if (wheels[i].valid)
				wheels[i].targetVel = fmat::dotProduct(wheels[i].direction,targetVel);
#endif
	} else if ( (displacementMode && std::abs(targetDist) < 1) || std::abs(targetVel[0]) <= EPSILON) {
			// pure turn	
#ifdef TGT_IS_KOBUKI
			wheels[SpeedOffset].targetVel = RobotInfo::wheelBase / 2 * targetAngVel;
			wheels[RadiusOffset].targetVel = 1;
#else
		for (unsigned int i=0; i<NumWheels; ++i)
			if (wheels[i].valid) {
				fmat::Column<2> p = wheels[i].position - rotationCenter;
				wheels[i].targetVel = fmat::dotProduct(wheels[i].direction,fmat::pack(-p[1],p[0])) * targetAngVel;
			}
#endif
		} else {
			// arc
#ifdef TGT_IS_KOBUKI
			float turnRadius = targetVel[0] / targetAngVel;
			if ( turnRadius > 0.0 )
				wheels[SpeedOffset].targetVel = (turnRadius + RobotInfo::wheelBase/2) * targetAngVel;
			else
				wheels[SpeedOffset].targetVel = (turnRadius - RobotInfo::wheelBase/2) * targetAngVel;
			wheels[RadiusOffset].targetVel = turnRadius;
#else
		// arcing motion, rotate target point about the previously determined center of rotation
		// std::cout <<"targetVel[0] = " << targetVel[0] << " targetVel[1] = " << targetVel[1]
		//					<< " targetAngVel = " << targetAngVel << std::endl;
		fmat::Column<2> arcCenter = fmat::pack(-targetVel[1],targetVel[0]) / targetAngVel + rotationCenter;
		for(unsigned int i=0; i<NumWheels; ++i) {
			if(!wheels[i].valid)
				continue;
			fmat::Column<2> p = wheels[i].position - arcCenter;
			wheels[i].targetVel = fmat::dotProduct(wheels[i].direction,fmat::pack(-p[1],p[0])) * targetAngVel;
		}
#endif
	}
	dirty = true;
}

void WheeledWalkMC::updateWheelConfig() {
#ifdef TGT_IS_KOBUKI
	// Kobuki cannot control individual wheel speeds, so the algorithm below doesn't apply
	wheels[0].valid = wheels[1].valid = true;
	maxVel = fmat::pack(700.f, 0.f);
	maxAngVel = 6.0f;
#else
	fmat::Column<2> wheelSum;
	unsigned int avail=0;
	
	// find position and orientation of each joint
	for(unsigned int i=0; i<NumWheels; ++i) {
		fmat::Transform t = kine->linkToBase(WheelOffset+i);
		wheels[i].direction = fmat::pack(t(1,2),-t(0,2));
		fmat::fmatReal dm=wheels[i].direction.norm();
		if(dm<std::numeric_limits<float>::epsilon()*10) {
			std::cerr << "WARNING: " << outputNames[WheelOffset+i] << " has a vertical axle, ignoring it" << std::endl;
			wheels[i].direction=0.f;
			continue; // wheel is on its side, does not contribute
		} else if(dm<1-std::numeric_limits<float>::epsilon()*10) {
			// this could be supported, just need to compute position at contact with ground
			std::cerr << "WARNING: " << outputNames[WheelOffset+i] << " has a non-planar axle, this may affect motion accuracy" << std::endl;
		}
		wheels[i].direction/=dm;
		wheels[i].position = fmat::SubVector<2>(t.translation());
		wheelSum+=wheels[i].position;
		++avail;
		wheels[i].valid=true;
	}
	rotationCenter = wheelSum / avail;
	
	maxVel = (NumWheels==0) ? 0 : std::numeric_limits<fmat::fmatReal>::infinity();
	maxAngVel=0.f;
	for(unsigned int i=0; i<NumWheels; ++i) {
		if(!wheels[i].valid)
			continue;
		
		// maximum velocity is the minimum full-power wheel contribution
		float v = std::min(std::abs(outputRanges[WheelOffset+i][0]),std::abs(outputRanges[WheelOffset+i][1]));
		fmat::Column<2> d = wheels[i].direction*v;
		maxVel[0] = std::min(maxVel[0],d[0]);
		maxVel[1] = std::min(maxVel[1],d[1]);
		
		// maximum rotation is sum of contributions
		fmat::Column<2> curCof = (wheelSum - wheels[i].position) / (avail-1);
		fmat::Column<2> norm = (wheels[i].position - curCof);
		norm -= fmat::dotProduct(wheels[i].direction,norm) * wheels[i].direction;
		float base = norm.norm();
		const float MAX_ROT = M_PI*3; // limit crazy rotational speeds in case of short base
		float av = (base*MAX_ROT > v ) ? v / base : 0; // send limit to zero because centrally applied instead of inf. rotation
		maxAngVel += av;
	}
#endif	
	if(targetVel[0]!=0 || targetVel[1]!=0 || targetAngVel!=0)
		updateWheelVels();
}

/*! @file
 * @brief Impliments WheeledWalkMC, which provides a 'WalkMC' implementation for wheeled robots (diff-drive or theoretically holonomic configurations as well)
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
